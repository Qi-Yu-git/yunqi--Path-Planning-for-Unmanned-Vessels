using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Runtime.InteropServices;
using System.Diagnostics;
using System.Threading;
using System.Threading.Tasks;

using OpenCvSharp;
using OpenCvSharp.Dnn;
using UnityEngine;

// 解决Debug冲突：明确使用UnityEngine.Debug
using Debug = UnityEngine.Debug;

// 新增命名空间，避免类名全局冲突
namespace YoloV8Detection
{
    /// <summary>
    /// YOLOv8目标检测引擎，封装OpenCV DNN推理功能（适配 (1,84,8400) 模型格式 + 低版本兼容）
    /// 核心优化：适配84列模型（4坐标+80类别，无单独置信度列），修复检测解析逻辑
    /// 新增功能：手动实现Mat转Texture2D（兼容低版本OpenCvSharp）、跨帧追踪、模型预热、批量检测、性能监控等
    /// </summary>
    public class YoloV8Engine : IDisposable
    {
        #region 私有字段
        private Net _net;                  // OpenCV DNN网络实例
        private string _modelPath;         // 模型路径
        private float _confidenceThreshold;// 置信度阈值（0-1）
        private float _iouThreshold;       // IOU阈值（0-1）
        private List<string> _classNames;  // 类别名称列表
        private bool _isInitialized;       // 初始化状态标记
        private readonly object _lockObj = new object(); // 线程同步锁
        private Size _inputSize = new Size(640, 640); // 模型输入尺寸
        private bool _isNoSeparateConfidence; // 是否为无单独置信度列的模型（84列格式）
        private bool _useCuda = true;      // 是否优先使用CUDA（新增：控制后端）

        // 日志相关字段（替换原有日志控制，接入统一配置）
        private readonly YoloLogSettings _logSettings;
        private readonly YoloLogSettings.LogModule _currentModule = YoloLogSettings.LogModule.YoloV8Engine;
        private float _lastAggregateLogTime;
        private Dictionary<string, int> _classCountAggregate = new Dictionary<string, int>();

        // 追踪相关字段
        private readonly Dictionary<string, int> _classTrackIdCounter = new Dictionary<string, int>();
        private readonly object _trackIdLock = new object();
        private Dictionary<int, YoloResult> _lastFrameResults = new Dictionary<int, YoloResult>();
        private readonly object _trackMatchLock = new object();

        // 性能监控字段
        private readonly Dictionary<string, List<float>> _inferenceTimeStats = new Dictionary<string, List<float>>();
        private int _detectionErrorCount = 0;
        private readonly object _statsLock = new object();

        // 启停控制 + 动态频率调节字段
        private bool _isDetectionEnabled = true ;          // 检测总开关 false
        private float _targetDetectionFrequency = 30f;    // 目标检测帧率（帧/秒）
        private float _detectionInterval => 1f / _targetDetectionFrequency; // 检测间隔（秒）
        private float _lastDetectionTime;                 // 上次检测时间戳
        private readonly object _frequencyLock = new object(); // 频率控制锁

        // 异步推理新增字段
        private readonly SemaphoreSlim _threadSemaphore; // 线程池控制
        private CancellationTokenSource _cts; // 异步任务取消令牌
        private readonly TaskScheduler _mainThreadScheduler; // 主线程调度器
        private Task _currentDetectionTask; // 当前异步任务缓存
        private readonly object _taskLock = new object(); // 任务锁
        #endregion

        #region 公共属性
        public bool IsInitialized => _isInitialized;
        public Size InputSize => _inputSize;
        public IReadOnlyList<string> ClassNames => _classNames.AsReadOnly();
        public bool UseCuda { get => _useCuda; set => _useCuda = value; } // 暴露CUDA控制

        /// <summary>
        /// 是否有异步检测任务在执行
        /// </summary>
        public bool IsDetectingAsync => _currentDetectionTask != null && !_currentDetectionTask.IsCompleted;

        /// <summary>
        /// 异步检测完成回调（主线程执行）
        /// </summary>
        public Action<List<YoloResult>> OnDetectionCompleted;

        // 保留原有日志配置兼容（内部映射到统一日志配置）
        public bool LogModelProcessing
        {
            get => _logSettings.IsModuleEnabled(_currentModule) &&
                  _logSettings.GetModuleLogLevel(_currentModule) <= YoloLogSettings.LogLevel.Debug;
            set => _logSettings.SetModuleLogLevel(_currentModule, value ? YoloLogSettings.LogLevel.Debug : YoloLogSettings.LogLevel.None);
        }
        public bool LogNmsResults
        {
            get => _logSettings.IsModuleEnabled(_currentModule) &&
                  _logSettings.GetModuleLogLevel(_currentModule) <= YoloLogSettings.LogLevel.Info;
            set => _logSettings.SetModuleLogLevel(_currentModule, value ? YoloLogSettings.LogLevel.Info : YoloLogSettings.LogLevel.None);
        }
        public float AggregateLogInterval { get; set; } = 1f; // 聚合日志输出间隔（秒）
        public List<string> LogIncludedClasses { get; set; } = new List<string>();
        public List<string> LogExcludedClasses { get; set; } = new List<string>();

        /// <summary>
        /// 检测总开关（启停控制）
        /// </summary>
        public bool IsDetectionEnabled
        {
            get => _isDetectionEnabled;
            set
            {
                _isDetectionEnabled = value;
                LogInfo(value ? "✅ 检测功能已启用" : "❌ 检测功能已禁用");
            }
        }

        /// <summary>
        /// 目标检测帧率（动态调节，范围1-60帧/秒）
        /// </summary>
        public float TargetDetectionFrequency
        {
            get => _targetDetectionFrequency;
            set
            {
                lock (_frequencyLock)
                {
                    _targetDetectionFrequency = Mathf.Clamp(value, 1f, 60f);
                    LogInfo($"🔄 检测频率已调整为：{_targetDetectionFrequency:F1} 帧/秒（间隔：{_detectionInterval:F3} 秒）");
                }
            }
        }

        /// <summary>
        /// 当前是否达到检测频率要求（用于外部判断是否执行检测）
        /// </summary>
        public bool CanDetectNow
        {
            get
            {
                lock (_frequencyLock)
                {
                    return _isDetectionEnabled && (Time.time - _lastDetectionTime >= _detectionInterval);
                }
            }
        }
        #endregion

        #region 构造函数
        /// <summary>
        /// 构造函数（新增统一日志配置参数 + 异步初始化）
        /// </summary>
        /// <param name="modelPath">模型文件路径</param>
        /// <param name="classNames">类别名称列表（默认COCO80类）</param>
        /// <param name="confidenceThreshold">置信度阈值</param>
        /// <param name="iouThreshold">IOU阈值</param>
        /// <param name="inputSize">模型输入尺寸</param>
        /// <param name="isNoSeparateConfidence">是否为84列模型（无单独置信度列）</param>
        /// <param name="logSettings">统一日志配置（不传则创建默认配置）</param>
        /// <param name="aggregateLogInterval">聚合日志输出间隔</param>
        /// <param name="autoWarmUp">是否自动预热模型</param>
        /// <param name="useCuda">是否优先使用CUDA推理</param>
        /// <param name="maxConcurrentThreads">最大并发线程数（异步新增）</param>
        public YoloV8Engine(string modelPath, List<string> classNames = null,
                           float confidenceThreshold = 0.5f, float iouThreshold = 0.4f,
                           Size? inputSize = null, bool isNoSeparateConfidence = true,
                           YoloLogSettings logSettings = null,
                           float aggregateLogInterval = 5f,
                           bool autoWarmUp = true,
                           bool useCuda = true,
                           int maxConcurrentThreads = 2) // 异步新增参数
        {
            _modelPath = modelPath;
            _confidenceThreshold = Mathf.Clamp01(confidenceThreshold);
            _iouThreshold = Mathf.Clamp01(iouThreshold);
            _classNames = classNames ?? GetDefaultCocoClassNames();
            _isNoSeparateConfidence = isNoSeparateConfidence;
            _useCuda = useCuda;
            if (inputSize.HasValue) _inputSize = inputSize.Value;

            // 初始化统一日志配置（修复CS0136：复用类级别字段，不重复声明局部变量）
            _logSettings = YoloLogSettings.Instance;
            AggregateLogInterval = aggregateLogInterval;

            // 监听日志配置变更
            _logSettings.OnSettingsChanged += RefreshLogConfig;

            // ===================== 异步初始化（新增） =====================
            _cts = new CancellationTokenSource();
            _threadSemaphore = new SemaphoreSlim(maxConcurrentThreads, maxConcurrentThreads);
            _mainThreadScheduler = TaskScheduler.FromCurrentSynchronizationContext();
            // ==============================================================

            try
            {
                _isInitialized = InitializeEngine();

                // 自动预热模型
                if (_isInitialized && autoWarmUp)
                {
                    WarmUpModel();
                }
            }
            catch (Exception ex)
            {
                LogError($"引擎初始化失败: {ex.Message}\n{ex.StackTrace}");
                _isInitialized = false;
            }
        }
        #endregion

        #region 私有日志方法（接入统一配置）
        /// <summary>
        /// 刷新日志配置（监听配置变更）
        /// </summary>
        private void RefreshLogConfig()
        {
            LogDebug("YoloV8Engine日志配置已更新");
        }

        /// <summary>
        /// 调试日志输出（统一控制）
        /// </summary>
        private void LogDebug(string message)
        {
            if (!_logSettings.IsModuleEnabled(_currentModule)) return;
            if (_logSettings.GetModuleLogLevel(_currentModule) > YoloLogSettings.LogLevel.Debug) return;

            YoloLogSettings.Log(YoloLogSettings.LogLevel.Info,$"[YoloV8Engine][Debug] {message}");
        }

        /// <summary>
        /// 信息日志输出（统一控制）
        /// </summary>
        private void LogInfo(string message)
        {
            if (!_logSettings.IsModuleEnabled(_currentModule)) return;
            if (_logSettings.GetModuleLogLevel(_currentModule) > YoloLogSettings.LogLevel.Info) return;

            YoloLogSettings.Log(YoloLogSettings.LogLevel.Info,$"[YoloV8Engine][Info] {message}");
        }

        /// <summary>
        /// 警告日志输出（统一控制）
        /// </summary>
        private void LogWarn(string message)
        {
            if (!_logSettings.IsModuleEnabled(_currentModule)) return;
            if (_logSettings.GetModuleLogLevel(_currentModule) > YoloLogSettings.LogLevel.Warn) return;

            YoloLogSettings.Log(YoloLogSettings.LogLevel.Warn,$"[YoloV8Engine][Warn] {message}");
        }

        /// <summary>
        /// 错误日志输出（统一控制）
        /// </summary>
        private void LogError(string message)
        {
            if (!_logSettings.IsModuleEnabled(_currentModule)) return;
            if (_logSettings.GetModuleLogLevel(_currentModule) > YoloLogSettings.LogLevel.Error) return;

            YoloLogSettings.Log(YoloLogSettings.LogLevel.Error,$"[YoloV8Engine][Error] {message}");
        }

        /// <summary>
        /// 致命错误日志输出（统一控制）
        /// </summary>
        private void LogFatal(string message)
        {
            if (!_logSettings.IsModuleEnabled(_currentModule)) return;
            if (_logSettings.GetModuleLogLevel(_currentModule) > YoloLogSettings.LogLevel.Fatal) return;

            YoloLogSettings.Log(YoloLogSettings.LogLevel.Error,$"[YoloV8Engine][Fatal] {message}");
        }
        #endregion

        #region 公共核心方法
        /// <summary>
        /// 单帧检测（整合版：保留健壮性+修复输出层+移除不存在的方法）
        /// </summary>
        /// <param name="frame">输入图像Mat</param>
        /// <returns>检测结果列表</returns>
        public List<YoloResult> Detect(Mat frame)
        {
            // 1. 启停控制 + 频率控制校验
            if (!_isDetectionEnabled)
            {
                LogDebug("❌ 检测已禁用，跳过本次推理");
                return new List<YoloResult>();
            }

            lock (_frequencyLock)
            {
                if (Time.time - _lastDetectionTime < _detectionInterval)
                {
                    LogDebug($"⏱️ 未达到检测频率要求（间隔：{_detectionInterval:F3}s，上次检测：{Time.time - _lastDetectionTime:F3}s前），跳过本次推理");
                    return new List<YoloResult>();
                }
                _lastDetectionTime = Time.time; // 更新上次检测时间
            }

            // 2. 原有空值校验逻辑（保留）
            if (_net == null || (_net != null && _net.Empty()))
            {
                LogError($"❌ 检测前校验失败：YOLO模型未初始化！_net状态：{(_net == null ? "null" : "Empty")}");
                return new List<YoloResult>();
            }
            if (!_isInitialized)
            {
                LogError("❌ 检测前校验失败：引擎未初始化完成");
                return new List<YoloResult>();
            }
            if (frame == null || frame.Empty())
            {
                LogError("❌ 检测前校验失败：输入帧为空或无效");
                return new List<YoloResult>();
            }


            // 增强空值校验
            if (_net == null || (_net != null && _net.Empty()))
            {
                LogError($"❌ 检测前校验失败：YOLO模型未初始化！_net状态：{(_net == null ? "null" : "Empty")}");
                return new List<YoloResult>();
            }
            if (!_isInitialized)
            {
                LogError("❌ 检测前校验失败：引擎未初始化完成");
                return new List<YoloResult>();
            }
            if (frame == null || frame.Empty())
            {
                LogError("❌ 检测前校验失败：输入帧为空或无效");
                return new List<YoloResult>();
            }

            lock (_lockObj)
            {
                float preprocessTime = 0, inferenceTime = 0, parseTime = 0;
                try
                {
                    int frameWidth = frame.Cols;
                    int frameHeight = frame.Rows;

                    // 1. 预处理 + 耗时统计
                    var watch = Stopwatch.StartNew();
                    using (var blob = PreprocessImage(frame))
                    {
                        watch.Stop();
                        preprocessTime = (float)watch.Elapsed.TotalMilliseconds;

                        // 2. 推理 + 耗时统计（核心修复：强制指定输出层名称）
                        watch.Restart();
                        _net.SetInput(blob);
                        string[] outputLayerNames = _net.GetUnconnectedOutLayersNames();
                        using (var originalOutput = _net.Forward(outputLayerNames.Length > 0 ? outputLayerNames[0] : ""))
                        {
                            watch.Stop();
                            inferenceTime = (float)watch.Elapsed.TotalMilliseconds;

                            // 3. 维度转换 + 解析 + 耗时统计
                            watch.Restart();
                            Mat output = originalOutput.Clone();

                            LogDebug($"原始输出形状: ({output.Size(0)}, {output.Size(1)}, {output.Size(2)})");

                            // 维度转换适配
                            if (output.Dims == 3 && output.Size(0) == 1)
                            {
                                int channel = output.Size(1);
                                int boxCount = output.Size(2);

                                if (boxCount == 8400 && (channel == 84 || channel == 85))
                                {
                                    output = output.Reshape(1, channel);
                                    output = output.T();
                                }
                                else if (channel == 8400 && (output.Size(2) == 84 || output.Size(2) == 85))
                                {
                                    output = output.Reshape(1, 8400);
                                }
                            }
                            else if (output.Dims == 2 && output.Rows != 8400 && output.Cols == 8400)
                            {
                                output = output.T();
                            }

                            LogDebug($"调整后形状: {output.Rows}行 x {output.Cols}列");

                            var results = ParseDetectionOutput(output, frameWidth, frameHeight);
                            watch.Stop();
                            parseTime = (float)watch.Elapsed.TotalMilliseconds;

                            // 释放临时Mat
                            output.Release();

                            // 记录性能统计
                            lock (_statsLock)
                            {
                                AddStat("Preprocess", preprocessTime);
                                AddStat("Inference", inferenceTime);
                                AddStat("Parse", parseTime);
                                AddStat("Total", preprocessTime + inferenceTime + parseTime);
                            }

                            // 处理检测日志
                            ProcessDetectionLogs(results);

                            return results;
                        }
                    }
                }
                catch (Exception ex)
                {
                    // 异常次数统计
                    lock (_statsLock)
                    {
                        _detectionErrorCount++;
                    }

                    // 过滤已知的Backend/Target警告，避免误报
                    if (ex.Message.Contains("preferableBackend") || ex.Message.Contains("preferableTarget"))
                    {
                        LogWarn($"⚠️ 推理后端配置警告：{ex.Message}（自动适配CPU模式）");
                    }
                    else
                    {
                        LogError($"🚫 检测出错：{ex.Message}\n堆栈信息：{ex.StackTrace}");
                        LogError($"🚫 报错时状态：_net是否为空={(_net == null ? "是" : "否")}, " +
                                      $"frame是否为空={(frame == null ? "是" : "否")}, " +
                                      $"frame是否有效={(frame?.Empty() ?? true ? "否" : "是")}");
                    }

                    // 自动降级到CPU并重试（增加_net判空）
                    try
                    {
                        if (_net == null || _net.Empty())
                        {
                            LogError("❌ _net为空，无法重试");
                            return new List<YoloResult>();
                        }

                        LogWarn("🔄 尝试降级到CPU后端重试检测...");
                        _net.SetPreferableBackend((Backend)0);
                        _net.SetPreferableTarget((Target)0);
                        return RetryDetectWithCpu(frame);
                    }
                    catch (Exception retryEx)
                    {
                        LogError($"❌ CPU重试也失败：{retryEx.Message}");
                        return new List<YoloResult>();
                    }
                }
            }
        }

        /// <summary>
        /// CPU降级重试检测（确保单定义，解决CS0111）
        /// </summary>
        /// <param name="frame">输入图像Mat</param>
        /// <returns>检测结果</returns>
        private List<YoloResult> RetryDetectWithCpu(Mat frame)
        {
            // 增加_net判空，避免空引用
            if (_net == null || _net.Empty())
            {
                LogError("❌ CPU重试失败：_net未初始化");
                return new List<YoloResult>();
            }

            if (frame == null || frame.Empty())
            {
                LogError("❌ CPU重试失败：输入帧无效");
                return new List<YoloResult>();
            }

            try
            {
                using (var blob = PreprocessImage(frame))
                {
                    _net.SetInput(blob);
                    string[] outputLayerNames = _net.GetUnconnectedOutLayersNames();
                    using (var originalOutput = _net.Forward(outputLayerNames.Length > 0 ? outputLayerNames[0] : ""))
                    {
                        Mat output = originalOutput.Clone();
                        int frameWidth = frame.Cols;
                        int frameHeight = frame.Rows;

                        // 复用维度转换逻辑
                        if (output.Dims == 3 && output.Size(0) == 1)
                        {
                            int channel = output.Size(1);
                            int boxCount = output.Size(2);
                            if (boxCount == 8400 && (channel == 84 || channel == 85))
                            {
                                output = output.Reshape(1, channel);
                                output = output.T();
                            }
                            else if (channel == 8400 && (output.Size(2) == 84 || output.Size(2) == 85))
                            {
                                output = output.Reshape(1, 8400);
                            }
                        }
                        else if (output.Dims == 2 && output.Rows != 8400 && output.Cols == 8400)
                        {
                            output = output.T();
                        }

                        var results = ParseDetectionOutput(output, frameWidth, frameHeight);
                        output.Release();
                        return results;
                    }
                }
            }
            catch (Exception ex)
            {
                LogError($"❌ RetryDetectWithCpu 执行失败：{ex.Message}");
                return new List<YoloResult>();
            }
        }
        /// <summary>
        /// 强制触发一次检测（忽略频率限制）
        /// </summary>
        /// <param name="frame">输入图像Mat</param>
        /// <returns>检测结果列表</returns>
        public List<YoloResult> ForceDetect(Mat frame)
        {
            if (!_isDetectionEnabled)
            {
                LogWarn("⚠️ 检测已禁用，强制检测仍执行");
            }

            lock (_frequencyLock)
            {
                _lastDetectionTime = 0; // 重置时间戳，强制触发
            }
            return Detect(frame);
        }

        /// <summary>
        /// 重置检测频率计时（用于场景切换等特殊场景）
        /// </summary>
        public void ResetDetectionTimer()
        {
            lock (_frequencyLock)
            {
                _lastDetectionTime = 0;
                LogInfo("⏱️ 检测频率计时已重置");
            }
        }

        /// <summary>
        /// 异步检测（非阻塞主线程）
        /// </summary>
        /// <param name="frame">输入图像Mat（会自动拷贝，避免主线程释放）</param>
        /// <returns>异步任务</returns>
        public async Task DetectAsync(Mat frame)
        {
            // 1. 前置校验
            if (!_isDetectionEnabled)
            {
                LogDebug("❌ 检测已禁用，跳过异步推理");
                OnDetectionCompleted?.Invoke(new List<YoloResult>());
                return;
            }

            lock (_frequencyLock)
            {
                if (Time.time - _lastDetectionTime < _detectionInterval)
                {
                    LogDebug($"⏱️ 未达到检测频率要求，跳过异步推理");
                    OnDetectionCompleted?.Invoke(new List<YoloResult>());
                    return;
                }
                _lastDetectionTime = Time.time;
            }

            if (_net == null || _net.Empty() || !_isInitialized || frame == null || frame.Empty())
            {
                LogError("❌ 异步检测前置校验失败");
                OnDetectionCompleted?.Invoke(new List<YoloResult>());
                return;
            }

            // 2. 避免重复提交任务
            lock (_taskLock)
            {
                if (IsDetectingAsync)
                {
                    LogDebug("⚠️ 已有异步检测任务在执行，跳过本次提交");
                    return;
                }

                // 3. 拷贝Mat到内存（避免主线程释放导致异步线程访问异常）
                Mat frameCopy = frame.Clone();
                _currentDetectionTask = ExecuteDetectionAsync(frameCopy, _cts.Token);
            }

            // 4. 等待任务完成并清理
            try
            {
                await _currentDetectionTask;
            }
            catch (OperationCanceledException)
            {
                LogDebug("🔴 异步检测任务被取消");
            }
            catch (Exception ex)
            {
                LogError($"🚫 异步检测任务异常：{ex.Message}");
            }
            finally
            {
                lock (_taskLock)
                {
                    _currentDetectionTask = null;
                }
            }
        }

        /// <summary>
        /// 异步批量检测
        /// </summary>
        /// <param name="frames">图像列表（需提前拷贝）</param>
        /// <param name="onBatchCompleted">批量完成回调（主线程）</param>
        public async Task BatchDetectAsync(List<Mat> frames, Action<List<List<YoloResult>>> onBatchCompleted)
        {
            if (frames == null || frames.Count == 0 || !_isInitialized || _net == null)
            {
                onBatchCompleted?.Invoke(new List<List<YoloResult>>());
                return;
            }

            await _threadSemaphore.WaitAsync(_cts.Token);
            try
            {
                // 后台线程执行批量检测
                var batchResults = await Task.Run(() =>
                {
                    var results = new List<List<YoloResult>>();
                    foreach (var frame in frames)
                    {
                        if (_cts.Token.IsCancellationRequested) break;
                        var frameCopy = frame.Clone();
                        results.Add(DetectInternal(frameCopy));
                        frameCopy.Release();
                    }
                    return results;
                }, _cts.Token);

                // 主线程回调
                await Task.Factory.StartNew(() =>
                {
                    onBatchCompleted?.Invoke(batchResults);
                }, _cts.Token, TaskCreationOptions.None, _mainThreadScheduler);
            }
            catch (Exception ex)
            {
                LogError($"🚫 异步批量检测失败：{ex.Message}");
                await Task.Factory.StartNew(() =>
                {
                    onBatchCompleted?.Invoke(new List<List<YoloResult>>());
                }, CancellationToken.None, TaskCreationOptions.None, _mainThreadScheduler);
            }
            finally
            {
                _threadSemaphore.Release();
            }
        }

        /// <summary>
        /// 内部同步检测逻辑（抽离原有Detect方法的核心，去掉频率控制）
        /// </summary>
        private List<YoloResult> DetectInternal(Mat frame)
        {
            if (_net == null || _net.Empty() || frame == null || frame.Empty())
                return new List<YoloResult>();

            float preprocessTime = 0, inferenceTime = 0, parseTime = 0;
            try
            {
                int frameWidth = frame.Cols;
                int frameHeight = frame.Rows;

                // 1. 预处理
                var watch = Stopwatch.StartNew();
                using (var blob = PreprocessImage(frame))
                {
                    watch.Stop();
                    preprocessTime = (float)watch.Elapsed.TotalMilliseconds;

                    // 2. 推理
                    watch.Restart();
                    _net.SetInput(blob);
                    string[] outputLayerNames = _net.GetUnconnectedOutLayersNames();
                    using (var originalOutput = _net.Forward(outputLayerNames.Length > 0 ? outputLayerNames[0] : ""))
                    {
                        watch.Stop();
                        inferenceTime = (float)watch.Elapsed.TotalMilliseconds;

                        // 3. 解析
                        watch.Restart();
                        Mat output = originalOutput.Clone();

                        // 维度转换适配
                        if (output.Dims == 3 && output.Size(0) == 1)
                        {
                            int channel = output.Size(1);
                            int boxCount = output.Size(2);
                            if (boxCount == 8400 && (channel == 84 || channel == 85))
                            {
                                output = output.Reshape(1, channel);
                                output = output.T();
                            }
                            else if (channel == 8400 && (output.Size(2) == 84 || output.Size(2) == 85))
                            {
                                output = output.Reshape(1, 8400);
                            }
                        }
                        else if (output.Dims == 2 && output.Rows != 8400 && output.Cols == 8400)
                        {
                            output = output.T();
                        }

                        var results = ParseDetectionOutput(output, frameWidth, frameHeight);
                        watch.Stop();
                        parseTime = (float)watch.Elapsed.TotalMilliseconds;

                        output.Release();

                        // 记录性能统计
                        lock (_statsLock)
                        {
                            AddStat("Preprocess", preprocessTime);
                            AddStat("Inference", inferenceTime);
                            AddStat("Parse", parseTime);
                            AddStat("Total", preprocessTime + inferenceTime + parseTime);
                        }

                        return results;
                    }
                }
            }
            catch (Exception ex)
            {
                lock (_statsLock)
                {
                    _detectionErrorCount++;
                }

                LogError($"🚫 内部检测逻辑出错：{ex.Message}");
                return RetryDetectWithCpu(frame);
            }
        }

        /// <summary>
        /// 执行异步检测（核心逻辑，运行在后台线程）
        /// </summary>
        private async Task ExecuteDetectionAsync(Mat frame, CancellationToken token)
        {
            List<YoloResult> results = new List<YoloResult>();
            await _threadSemaphore.WaitAsync(token); // 限制并发数

            try
            {
                // 切换到后台线程执行推理
                await Task.Run(() =>
                {
                    token.ThrowIfCancellationRequested();
                    lock (_lockObj)
                    {
                        // 复用原有同步检测逻辑
                        results = DetectInternal(frame);
                    }
                }, token);

                // 切换回主线程执行回调（关键：Unity的API必须在主线程调用）
                await Task.Factory.StartNew(() =>
                {
                    if (!token.IsCancellationRequested)
                    {
                        OnDetectionCompleted?.Invoke(results);
                        ProcessDetectionLogs(results); // 日志也在主线程输出
                    }
                }, token, TaskCreationOptions.None, _mainThreadScheduler);
            }
            catch (Exception ex)
            {
                LogError($"🚫 异步检测执行失败：{ex.Message}\n{ex.StackTrace}");
                // 主线程回调空结果
                await Task.Factory.StartNew(() =>
                {
                    OnDetectionCompleted?.Invoke(new List<YoloResult>());
                }, CancellationToken.None, TaskCreationOptions.None, _mainThreadScheduler);
            }
            finally
            {
                frame.Release(); // 释放拷贝的Mat
                _threadSemaphore.Release(); // 释放信号量
            }
        }
        #endregion

        /// <summary>
        /// 批量检测多帧图像
        /// </summary>
        /// <param name="frames">多帧Mat图像列表</param>
        /// <returns>每帧对应的检测结果</returns>
        public List<List<YoloResult>> BatchDetect(List<Mat> frames)
        {
            var batchResults = new List<List<YoloResult>>();
            if (frames == null || frames.Count == 0 || !_isInitialized || _net == null)
            {
                LogError("❌ 批量检测失败：输入无效或引擎未初始化");
                return batchResults;
            }

            lock (_lockObj)
            {
                try
                {
                    // 初始化结果列表
                    for (int i = 0; i < frames.Count; i++)
                    {
                        batchResults.Add(new List<YoloResult>());
                    }

                    // 批量预处理和推理
                    for (int i = 0; i < frames.Count; i++)
                    {
                        var frame = frames[i];
                        if (frame == null || frame.Empty())
                        {
                            continue;
                        }

                        using (var blob = PreprocessImage(frame))
                        {
                            _net.SetInput(blob);
                            string[] outputLayerNames = _net.GetUnconnectedOutLayersNames();
                            using (var originalOutput = _net.Forward(outputLayerNames.Length > 0 ? outputLayerNames[0] : ""))
                            {
                                Mat output = originalOutput.Clone();

                                // 维度转换
                                if (output.Dims == 3 && output.Size(0) == 1)
                                {
                                    int channel = output.Size(1);
                                    int boxCount = output.Size(2);

                                    if (boxCount == 8400 && (channel == 84 || channel == 85))
                                    {
                                        output = output.Reshape(1, channel);
                                        output = output.T();
                                    }
                                    else if (channel == 8400 && (output.Size(2) == 84 || output.Size(2) == 85))
                                    {
                                        output = output.Reshape(1, 8400);
                                    }
                                }
                                else if (output.Dims == 2 && output.Rows != 8400 && output.Cols == 8400)
                                {
                                    output = output.T();
                                }

                                // 解析结果
                                var frameResult = ParseDetectionOutput(output, frame.Cols, frame.Rows);
                                batchResults[i] = frameResult;

                                // 处理检测日志
                                ProcessDetectionLogs(frameResult);

                                output.Release();
                            }
                        }
                    }

                    return batchResults;
                }
                catch (Exception ex)
                {
                    LogError($"❌ 批量检测失败：{ex.Message}\n{ex.StackTrace}");
                    lock (_statsLock)
                    {
                        _detectionErrorCount++;
                    }
                    return batchResults;
                }
            }
        }

        /// <summary>
        /// Mat转Texture2D（兼容低版本OpenCvSharp）
        /// </summary>
        /// <param name="frame">输入Mat</param>
        /// <returns>Unity Texture2D</returns>
        public Texture2D ConvertMatToTexture(Mat frame)
        {
            if (frame == null || frame.Empty())
            {
                LogError("❌ 无法转换空的Mat对象");
                return null;
            }

            try
            {
                using (Mat rgbMat = new Mat())
                {
                    Cv2.CvtColor(frame, rgbMat, ColorConversionCodes.BGR2RGB);

                    int width = rgbMat.Cols;
                    int height = rgbMat.Rows;
                    int channels = rgbMat.Channels();

                    Texture2D texture = new Texture2D(width, height, TextureFormat.RGB24, false);
                    byte[] data = new byte[width * height * channels];
                    Marshal.Copy(rgbMat.Data, data, 0, data.Length);

                    texture.LoadRawTextureData(data);
                    texture.Apply();

                    return texture;
                }
            }
            catch (Exception ex)
            {
                LogError($"🚫 Mat转Texture2D失败: {ex.Message}\n堆栈信息：{ex.StackTrace}");
                lock (_statsLock)
                {
                    _detectionErrorCount++;
                }
                return null;
            }
        }

        /// <summary>
        /// 检测+绘制+转纹理一站式处理
        /// </summary>
        /// <param name="frame">输入图像</param>
        /// <param name="results">输出检测结果</param>
        /// <returns>带检测框的Texture2D</returns>
        public Texture2D DetectAndConvertToTexture(Mat frame, out List<YoloResult> results)
        {
            results = Detect(frame);
            DrawDetectionResults(frame, results);
            return ConvertMatToTexture(frame);
        }

        /// <summary>
        /// 更新检测阈值
        /// </summary>
        /// <param name="confidenceThreshold">置信度阈值</param>
        /// <param name="iouThreshold">IOU阈值</param>
        public void UpdateThresholds(float confidenceThreshold, float iouThreshold)
        {
            _confidenceThreshold = Mathf.Clamp01(confidenceThreshold);
            _iouThreshold = Mathf.Clamp01(iouThreshold);
        }

        /// <summary>
        /// 设置模型格式（84列/85列）
        /// </summary>
        /// <param name="isNoSeparateConfidence">是否为84列模型</param>
        public void SetModelFormat(bool isNoSeparateConfidence)
        {
            _isNoSeparateConfidence = isNoSeparateConfidence;
            LogDebug($"🔄 模型格式已切换：{(isNoSeparateConfidence ? "84列（4坐标+80类别）" : "85列（4坐标+1置信度+80类别）")}");
        }

        /// <summary>
        /// 处理检测日志（聚合+过滤，接入统一日志控制）
        /// </summary>
        /// <param name="results">检测结果</param>
        public void ProcessDetectionLogs(List<YoloResult> results)
        {
            if (!_logSettings.IsModuleEnabled(_currentModule)) return;
            if (results == null) return;

            // 聚合统计
            foreach (var result in results)
            {
                if (_classCountAggregate.ContainsKey(result.ClassName))
                    _classCountAggregate[result.ClassName]++;
                else
                    _classCountAggregate[result.ClassName] = 1;
            }

            // 定时输出聚合日志
            if (Time.time - _lastAggregateLogTime > AggregateLogInterval)
            {
                if (_classCountAggregate.Count == 0)
                {
                    if (Time.frameCount % 30 == 0)
                        LogInfo("📌 聚合统计：未检测到任何目标");
                }
                else
                {
                    string aggregateLog = "📊 聚合统计：";
                    foreach (var kvp in _classCountAggregate)
                    {
                        aggregateLog += $"{kvp.Key}({kvp.Value}) ";
                    }
                    LogInfo(aggregateLog);

                    int total = _classCountAggregate.Values.Sum();
                    if (total > 50)
                        LogWarn($"⚠️ 检测到大量目标（{total}个），可能影响性能");
                }
                _classCountAggregate.Clear();
                _lastAggregateLogTime = Time.time;
            }

            // 输出详细日志（按级别控制）
            if (results.Count == 0) return;

            LogInfo($"📌 检测到 {results.Count} 个目标");
            foreach (var result in results)
            {
                bool shouldLog = true;
                if (LogIncludedClasses.Count > 0 && !LogIncludedClasses.Contains(result.ClassName))
                    shouldLog = false;
                if (LogExcludedClasses.Count > 0 && LogExcludedClasses.Contains(result.ClassName))
                    shouldLog = false;

                if (shouldLog && result.Confidence > 0.8f)
                {
                    LogDebug($"  - 类别：{result.ClassName} | 置信度：{result.Confidence:F2} | 位置：({result.Rect.X:F1}, {result.Rect.Y:F1}, {result.Rect.Width:F1}, {result.Rect.Height:F1}) | TrackId：{result.TrackId}");
                }
            }
        }

        /// <summary>
        /// 模型预热（解决首次检测卡顿，修复Backend/Target枚举异常）
        /// </summary>
        /// <returns>预热是否成功</returns>
        public bool WarmUpModel()
        {
            // 保留原有的初始化校验逻辑，避免空指针
            if (!_isInitialized || _net == null || _net.Empty())
            {
                LogError("❌ 模型未初始化，无法预热");
                return false;
            }

            lock (_lockObj) // 保留线程安全锁，避免多线程调用冲突
            {
                try
                {
                    // 步骤1：强制指定CPU后端（使用数值枚举，兼容Unity下的OpenCV版本）
                    _net.SetPreferableBackend((Backend)0); // 0 = DNN_BACKEND_OPENCV
                    _net.SetPreferableTarget((Target)0);   // 0 = DNN_TARGET_CPU

                    // 步骤2：创建虚拟输入矩阵（匹配模型输入尺寸，float类型）
                    using (Mat dummyInput = Mat.Zeros(_inputSize.Height, _inputSize.Width, MatType.CV_32FC3))
                    {
                        // 步骤3：复用预处理逻辑（保持和实际推理一致的输入格式）
                        using (var blob = PreprocessImage(dummyInput))
                        {
                            _net.SetInput(blob);
                            string[] outputLayerNames = _net.GetUnconnectedOutLayersNames();
                            string targetLayer = outputLayerNames.Length > 0 ? outputLayerNames[0] : "";

                            // 步骤4：执行预热推理
                            _net.Forward(targetLayer);
                            LogInfo("✅ 预热推理成功（CPU模式）");
                        }
                    }

                    return true;
                }
                catch (Exception e)
                {
                    LogWarn($"⚠️ 预热推理失败，自动降级CPU: {e.Message}");
                    return false;
                }
            }
        }

        /// <summary>
        /// 动态切换模型
        /// </summary>
        /// <param name="newModelPath">新模型路径</param>
        /// <param name="newInputSize">新输入尺寸</param>
        /// <param name="isNoSeparateConfidence">是否为84列模型</param>
        /// <param name="useCuda">是否优先使用CUDA</param>
        /// <returns>切换是否成功</returns>
        public bool SwitchModel(string newModelPath, Size? newInputSize = null, bool isNoSeparateConfidence = true, bool useCuda = true)
        {
            if (string.IsNullOrEmpty(newModelPath) || !File.Exists(newModelPath))
            {
                LogError($"❌ 新模型文件不存在：{newModelPath}");
                return false;
            }

            lock (_lockObj)
            {
                try
                {
                    // 释放旧模型（修复Net无Release方法的问题）
                    if (_net != null)
                    {
                        if (!_net.Empty())
                        {
                            _net.Dispose(); // 替换Release为Dispose
                        }
                        _net = null;
                    }

                    // 更新参数
                    _modelPath = newModelPath;
                    _isNoSeparateConfidence = isNoSeparateConfidence;
                    _useCuda = useCuda;
                    if (newInputSize.HasValue)
                        _inputSize = newInputSize.Value;

                    // 加载新模型
                    _net = CvDnn.ReadNetFromOnnx(_modelPath);
                    if (_net == null || _net.Empty())
                    {
                        LogError("❌ 新模型加载失败");
                        _isInitialized = false;
                        return false;
                    }

                    // 配置后端
                    ConfigureNetBackend();
                    _isInitialized = true;

                    // 预热新模型
                    WarmUpModel();

                    LogDebug($"✅ 模型切换成功：{newModelPath}");
                    return true;
                }
                catch (Exception ex)
                {
                    LogError($"❌ 模型切换失败：{ex.Message}\n{ex.StackTrace}");
                    _isInitialized = false;
                    lock (_statsLock)
                    {
                        _detectionErrorCount++;
                    }
                    return false;
                }
            }
        }

        /// <summary>
        /// 获取推理性能统计
        /// </summary>
        /// <param name="stage">统计阶段（Preprocess/Inference/Parse/Total）</param>
        /// <returns>平均/最大/最小耗时（毫秒）</returns>
        public (float avg, float max, float min) GetInferenceStats(string stage)
        {
            lock (_statsLock)
            {
                if (!_inferenceTimeStats.ContainsKey(stage) || _inferenceTimeStats[stage].Count == 0)
                    return (0, 0, 0);

                var times = _inferenceTimeStats[stage];
                return ((float)times.Average(), (float)times.Max(), (float)times.Min());
            }
        }

        /// <summary>
        /// 获取检测异常次数
        /// </summary>
        /// <returns>异常次数</returns>
        public int GetDetectionErrorCount() => _detectionErrorCount;

        /// <summary>
        /// 重置性能统计数据
        /// </summary>
        public void ResetStats()
        {
            lock (_statsLock)
            {
                _inferenceTimeStats.Clear();
                _detectionErrorCount = 0;
            }
        }

        /// <summary>
        /// 释放资源
        /// </summary>
        public void Dispose()
        {
            // ===================== 异步资源释放（新增） =====================
            // 取消所有异步任务
            if (_cts != null)
            {
                _cts.Cancel();
                _cts.Dispose();
            }

            // 释放信号量
            _threadSemaphore?.Dispose();
            // ==============================================================

            Dispose(true);
            GC.SuppressFinalize(this);
        }

        /// <summary>
        /// 释放资源
        /// </summary>
        protected virtual void Dispose(bool disposing)
        {
            if (disposing)
            {
                // 释放托管资源
                _classNames?.Clear();
                _classCountAggregate?.Clear();
                _classTrackIdCounter?.Clear();
                _lastFrameResults?.Clear();
                _inferenceTimeStats?.Clear();
                LogIncludedClasses?.Clear();
                LogExcludedClasses?.Clear();

                // 取消日志配置监听
                _logSettings.OnSettingsChanged -= RefreshLogConfig;

                // ===================== 异步资源释放（新增） =====================
                // 清空回调
                OnDetectionCompleted = null;
                // ==============================================================
            }

            // 释放非托管资源（修复Net无Release方法）
            if (_net != null)
            {
                try
                {
                    if (!_net.Empty())
                    {
                        _net.Dispose(); // 替换Release为Dispose
                    }
                    _net = null;
                }
                catch
                {
                    // 忽略释放时的异常
                }
            }

            // 重置状态
            _isInitialized = false;
            _modelPath = null;
            _lastAggregateLogTime = 0;
            _detectionErrorCount = 0;
        }

        #region 私有核心方法
        /// <summary>
        /// 图像预处理（适配OpenCVSharp 4.7严格维度校验）
        /// </summary>
        /// <param name="frame">输入图像</param>
        /// <returns>预处理后的Blob</returns>
        private Mat PreprocessImage(Mat frame)
        {
            // ========== 4.7版本适配：移除DepthType，改用数值指定浮点类型 ==========
            // OpenCVSharp 4.7 中 BlobFromImage 不支持DepthType参数，改用CV_32F数值(5)
            Mat blob = CvDnn.BlobFromImage(
                frame,
                1.0 / 255.0,          // 归一化系数
                _inputSize,           // 输入尺寸（强制640x640）
                new Scalar(0, 0, 0),  // 均值
                swapRB: true,         // BGR转RGB（YOLO要求）
                crop: false           // 禁用裁剪（4.7裁剪会导致维度偏移）
                                      // 4.7版本移除DepthType参数，改用后续强制转换
            );
            // ==================================================

            // 4.7额外处理：强制转换为CV_32F浮点类型（替代DepthType参数）
            blob.ConvertTo(blob, MatType.CV_32F);

            // 4.7额外校验：确保Blob维度严格为(1,3,640,640)
            if (blob.Size(0) != 1 || blob.Size(1) != 3 || blob.Size(2) != _inputSize.Height || blob.Size(3) != _inputSize.Width)
            {
                LogWarn($"⚠️ Blob维度异常：({blob.Size(0)},{blob.Size(1)},{blob.Size(2)},{blob.Size(3)})，强制重置为(1,3,{_inputSize.Height},{_inputSize.Width})");
                blob = blob.Reshape(3, new[] { 1, 3, _inputSize.Height, _inputSize.Width });
            }

            return blob;
        }

        /// <summary>
        /// 解析检测输出
        /// </summary>
        /// <param name="output">模型输出Mat</param>
        /// <param name="frameWidth">原图宽度</param>
        /// <param name="frameHeight">原图高度</param>
        /// <returns>检测结果列表</returns>
        private List<YoloResult> ParseDetectionOutput(Mat output, int frameWidth, int frameHeight)
        {
            var results = new List<YoloResult>();

            LogDebug($"输出矩阵信息: 维度={output.Dims}, 形状=({output.Size(0)},{output.Size(1)})");

            if (output == null || output.Empty())
            {
                LogError("❌ 解析失败：输入输出矩阵为空");
                return results;
            }

            int rows = output.Rows;
            int cols = output.Cols;

            // 修复列数异常
            if (cols <= 0)
            {
                LogError($"❌ 解析失败：无效列数={cols}，尝试从矩阵形状重新获取");
                if (output.Dims >= 2)
                {
                    rows = output.Size(0);
                    cols = output.Size(1);
                    LogWarn($"🔄 重新获取维度：rows={rows}, cols={cols}");
                }
                if (cols <= 0)
                {
                    LogError("❌ 无法获取有效维度，解析终止");
                    return results;
                }
            }

            // 自动适配模型列数
            int expectedCols = 0;
            if (cols == 84)
            {
                expectedCols = 84;
                _isNoSeparateConfidence = true;
                LogDebug($"📌 检测到模型输出84列（4坐标+80类别），自动适配COCO80类模式");
            }
            else if (cols == 85)
            {
                expectedCols = 85;
                _isNoSeparateConfidence = false;
                LogDebug($"📌 检测到模型输出85列（4坐标+1置信度+80类别），自动适配");
            }
            else if (cols == 4)
            {
                expectedCols = 4;
                LogDebug($"📌 检测到模型仅输出4列（纯坐标），自动适配无类别模式");
            }
            else
            {
                expectedCols = _isNoSeparateConfidence
                    ? 4 + _classNames.Count
                    : 5 + _classNames.Count;
            }

            LogDebug($"📌 解析输出：行数={rows}, 列数={cols}, 预期列数={expectedCols}（模型格式：{(_isNoSeparateConfidence ? "84列" : "85列")}）");

            // 列数校验
            if (cols != expectedCols)
            {
                LogError($"❌ 输出维度不匹配：实际{cols}列，预期{expectedCols}列");
                LogError($"💡 可能原因：1.模型类别数与配置不匹配 2.模型格式设置错误（当前设置：{(_isNoSeparateConfidence ? "84列" : "85列")}）");
                return results;
            }

            // 提取输出数据
            float[] outputData;
            try
            {
                if (output.IsContinuous())
                {
                    outputData = new float[rows * cols];
                    Marshal.Copy(output.Data, outputData, 0, rows * cols);
                }
                else
                {
                    outputData = new float[rows * cols];
                    for (int i = 0; i < rows; i++)
                    {
                        for (int j = 0; j < cols; j++)
                        {
                            outputData[i * cols + j] = output.At<float>(i, j);
                        }
                    }
                }
            }
            catch (Exception ex)
            {
                LogError($"❌ 提取输出数据失败：{ex.Message}\n{ex.StackTrace}");
                lock (_statsLock)
                {
                    _detectionErrorCount++;
                }
                return results;
            }

            // 解析每个检测框
            for (int i = 0; i < rows; i++)
            {
                int baseIndex = i * cols;

                // 读取坐标
                float x = outputData[baseIndex + 0];
                float y = outputData[baseIndex + 1];
                float w = outputData[baseIndex + 2];
                float h = outputData[baseIndex + 3];

                // 计算置信度和类别
                float finalConfidence = 0;
                int maxClassId = -1;
                if (_isNoSeparateConfidence)
                {
                    float maxClassScore = 0;
                    int classStartIndex = baseIndex + 4;
                    int classEndIndex = baseIndex + 4 + _classNames.Count;

                    if (classEndIndex > outputData.Length)
                        classEndIndex = outputData.Length;

                    for (int j = classStartIndex; j < classEndIndex; j++)
                    {
                        float classScore = outputData[j];
                        if (classScore > maxClassScore)
                        {
                            maxClassScore = classScore;
                            maxClassId = j - classStartIndex;
                        }
                    }

                    finalConfidence = maxClassScore;
                }
                else
                {
                    float boxConfidence = outputData[baseIndex + 4];
                    if (boxConfidence < _confidenceThreshold)
                        continue;

                    float maxClassScore = 0;
                    int classStartIndex = baseIndex + 5;
                    int classEndIndex = baseIndex + 5 + _classNames.Count;

                    if (classEndIndex > outputData.Length)
                        classEndIndex = outputData.Length;

                    for (int j = classStartIndex; j < classEndIndex; j++)
                    {
                        float classScore = outputData[j];
                        if (classScore > maxClassScore)
                        {
                            maxClassScore = classScore;
                            maxClassId = j - classStartIndex;
                        }
                    }

                    finalConfidence = boxConfidence * maxClassScore;
                }

                // 过滤低置信度
                if (finalConfidence < _confidenceThreshold || maxClassId < 0)
                    continue;

                // 转换为原图坐标
                float left = (x - w / 2) * frameWidth;
                float top = (y - h / 2) * frameHeight;
                float width = w * frameWidth;
                float height = h * frameHeight;

                // 边界限制
                left = Mathf.Max(0, left);
                top = Mathf.Max(0, top);
                width = Mathf.Max(1, Mathf.Min(frameWidth - left, width));
                height = Mathf.Max(1, Mathf.Min(frameHeight - top, height));

                // 构造结果
                string className = maxClassId < _classNames.Count
                    ? _classNames[maxClassId]
                    : $"unknown_{maxClassId}";

                // 修复 Rect 冲突：明确使用 OpenCvSharp.Rect
                OpenCvSharp.Rect rect = new OpenCvSharp.Rect(
                    (int)left,
                    (int)top,
                    (int)width,
                    (int)height);

                results.Add(new YoloResult
                {
                    ClassId = maxClassId,
                    ClassName = className,
                    Confidence = finalConfidence,
                    Rect = rect,
                    TrackId = GenerateUniqueTrackId(className)
                });
            }

            // NMS去重 + 跨帧追踪
            var nmsResults = ApplyNonMaxSuppression(results);
            var trackedResults = MatchCrossFrameTrackIds(nmsResults);
            return trackedResults;
        }

        /// <summary>
        /// 非极大值抑制（按类别分组）
        /// </summary>
        /// <param name="results">原始检测结果</param>
        /// <returns>NMS后的结果</returns>
        private List<YoloResult> ApplyNonMaxSuppression(List<YoloResult> results)
        {
            if (results.Count == 0)
            {
                LogInfo("📌 NMS：无有效检测框");
                return results;
            }

            var nmsResults = new List<YoloResult>();
            var classGroups = results.GroupBy(r => r.ClassId).ToList();

            foreach (var group in classGroups)
            {
                var groupResults = group.ToList();
                int groupCount = groupResults.Count;

                // 修复 Rect 冲突：明确使用 OpenCvSharp.Rect
                OpenCvSharp.Rect[] boxesArray = new OpenCvSharp.Rect[groupCount];
                float[] confidencesArray = new float[groupCount];

                for (int i = 0; i < groupCount; i++)
                {
                    var result = groupResults[i];
                    // 修复 Rect 冲突：明确使用 OpenCvSharp.Rect
                    boxesArray[i] = new OpenCvSharp.Rect(
                        (int)result.Rect.X,
                        (int)result.Rect.Y,
                        (int)result.Rect.Width,
                        (int)result.Rect.Height);
                    confidencesArray[i] = result.Confidence;
                }

                // ✅ 核心修正：移除命名参数，仅按位置传参，适配所有OpenCVSharp版本
                int[] indices;
                // 标准位置参数顺序：boxesArray → confidencesArray → scoreThreshold → nmsThreshold → out indices
                CvDnn.NMSBoxes(
                    boxesArray,                // 参数1：检测框数组
                    confidencesArray,          // 参数2：置信度数组
                    _confidenceThreshold,      // 参数3：置信度阈值（float）
                    _iouThreshold,             // 参数4：IOU阈值（float）
                    out indices                // 参数5：out接收NMS后的索引数组（解决CS1503）
                );

                // 遍历NMS筛选后的索引，添加结果
                foreach (int idx in indices)
                {
                    if (idx >= 0 && idx < groupResults.Count)
                        nmsResults.Add(groupResults[idx]);
                }
            }

            if (results.Count != nmsResults.Count)
            {
                LogInfo($"📌 NMS前：{results.Count}个框，NMS后：{nmsResults.Count}个框（按类别分组去重）");
            }

            return nmsResults;
        }

        /// <summary>
        /// 配置网络后端（CUDA/CPU）- 兼容所有OpenCvSharp版本
        /// </summary>
        private void ConfigureNetBackend()
        {
            if (_net == null)
            {
                LogWarn("⚠️ _net 为空，跳过推理后端配置");
                return;
            }

            try
            {
                // 兼容所有版本：使用数值枚举而非命名枚举
                if (_useCuda)
                {
                    try
                    {
                        // DNN_BACKEND_CUDA = 3, DNN_TARGET_CUDA = 6
                        _net.SetPreferableBackend((Backend)3);
                        _net.SetPreferableTarget((Target)6);
                        LogInfo("✅ 启用CUDA后端推理");
                    }
                    catch
                    {
                        // 降级到CPU：DNN_BACKEND_OPENCV = 0, DNN_TARGET_CPU = 0
                        _net.SetPreferableBackend((Backend)0);
                        _net.SetPreferableTarget((Target)0);
                        _useCuda = false; // 标记为禁用CUDA，避免重复尝试
                        LogWarn("⚠️ CUDA配置失败，降级到CPU后端");
                    }
                }
                else
                {
                    // 强制CPU后端
                    _net.SetPreferableBackend((Backend)0);
                    _net.SetPreferableTarget((Target)0);
                    LogInfo("✅ 已配置CPU推理后端");
                }
            }
            catch (Exception ex)
            {
                LogWarn($"⚠️ 后端配置失败：{ex.Message}，使用默认CPU后端");
                // 终极兜底：强制使用数值枚举
                try
                {
                    _net.SetPreferableBackend((Backend)0);
                    _net.SetPreferableTarget((Target)0);
                }
                catch
                {
                    // 忽略最终兜底的异常
                }
            }
        }

        /// <summary>
        /// 初始化引擎
        /// </summary>
        /// <returns>是否初始化成功</returns>
        private bool InitializeEngine()
        {
            if (string.IsNullOrEmpty(_modelPath))
            {
                LogWarn("⚠️ 模型路径为空，引擎未初始化");
                return false;
            }
            if (!File.Exists(_modelPath))
            {
                LogError($"❌ 模型文件不存在: {_modelPath}");
                return false;
            }

            // ========== 新增：4.7路径兼容处理 ==========
            // 4.7对相对路径解析有问题，强制转为绝对路径
            _modelPath = Path.GetFullPath(_modelPath);
            // 替换路径分隔符（4.7不识别/）
            _modelPath = _modelPath.Replace('/', '\\');
            LogInfo($"📌 4.7兼容路径：{_modelPath}");
            // ==================================================

            try
            {
                // ========== 新增：OpenCVSharp 4.7 兼容配置 ==========
                // 放宽4.7版本对ONNX维度/算子的严格校验
                Environment.SetEnvironmentVariable("OPENCV_DNN_ONNX_ALLOW_LEGACY_MODE", "1");
                Environment.SetEnvironmentVariable("OPENCV_DNN_DISABLE_OPTIMIZATION", "1");
                // 强制4.7使用旧版ONNX解析逻辑
                Environment.SetEnvironmentVariable("OPENCV_DNN_ONNX_USE_OPSET11", "1");
                // ==================================================

                // 配置OpenCV库路径（兼容不同部署路径）
                string[] libPaths = new[]
                {
                    Path.Combine(Application.dataPath, "Packages/OpenCvSharp4.runtime.win.4.8.0.20230708/runtimes/win-x64/native/"),
                    Path.Combine(Application.dataPath, "Plugins/OpenCvSharp/"),
                    Path.Combine(Application.streamingAssetsPath, "OpenCvSharp/"),
                    Path.Combine(Application.persistentDataPath, "OpenCvSharp/")
                };

                foreach (var libPath in libPaths)
                {
                    if (Directory.Exists(libPath))
                    {
                        Environment.SetEnvironmentVariable("PATH", $"{Environment.GetEnvironmentVariable("PATH")};{libPath}");
                        LogInfo($"✅ 已添加OpenCvSharp库路径：{libPath}");
                        // 移除break，保证所有有效路径都被添加（避免漏加依赖）
                    }
                }

                // 加载模型（增强判空逻辑）
                _net = CvDnn.ReadNetFromOnnx(_modelPath);
                if (_net == null || _net.Empty())
                {
                    LogError("❌ 模型加载失败，返回的网络为空或无效");
                    return false;
                }

                // 配置推理后端（核心：修复后的后端配置逻辑）
                ConfigureNetBackend();

                // 初始化日志间隔
                if (AggregateLogInterval <= 0)
                {
                    AggregateLogInterval = 5f;
                }

                LogInfo($"✅ YOLOv8引擎初始化成功（模型格式：{(_isNoSeparateConfidence ? "84列" : "85列")}，类别数：{_classNames.Count}）");

                return true;
            }
            catch (DllNotFoundException ex)
            {
                LogError($"找不到OpenCvSharp原生库: {ex.Message}");
                LogError("💡 解决方案：1.检查OpenCvSharpExtern.dll是否存在 2.确认库版本与Unity架构匹配（x64） 3.将库文件放到Plugins/x86_64目录");
                return false;
            }
            catch (Exception ex)
            {
                // 增强错误日志：补充4.7版本相关排查点
                LogError($"加载模型失败: {ex.Message}\n{ex.StackTrace}");
                LogError($"💡 4.7版本排查：1.模型OPSET是否≤11 2.环境变量是否生效 3.模型路径是否为绝对路径（当前路径：{_modelPath}）");
                return false;
            }
        }

        /// <summary>
        /// 生成唯一TrackId
        /// </summary>
        /// <param name="className">类别名称</param>
        /// <returns>TrackId</returns>
        private int GenerateUniqueTrackId(string className)
        {
            lock (_trackIdLock)
            {
                if (!_classTrackIdCounter.ContainsKey(className))
                {
                    _classTrackIdCounter[className] = 1;
                }
                return _classTrackIdCounter[className]++;
            }
        }

        /// <summary>
        /// 跨帧匹配TrackId（IOU匹配）
        /// </summary>
        /// <param name="currentResults">当前帧结果</param>
        /// <returns>带连续TrackId的结果</returns>
        private List<YoloResult> MatchCrossFrameTrackIds(List<YoloResult> currentResults)
        {
            lock (_trackMatchLock)
            {
                if (currentResults == null || currentResults.Count == 0)
                {
                    _lastFrameResults.Clear();
                    return currentResults;
                }

                var matchedResults = new List<YoloResult>();
                var usedLastFrameIds = new HashSet<int>();

                foreach (var current in currentResults)
                {
                    float maxIou = 0;
                    YoloResult matchedLastResult = null;

                    foreach (var last in _lastFrameResults.Values)
                    {
                        if (usedLastFrameIds.Contains(last.TrackId) || current.ClassId != last.ClassId)
                            continue;

                        float iou = CalculateIOU(current.Rect, last.Rect);
                        if (iou > 0.5 && iou > maxIou)
                        {
                            maxIou = iou;
                            matchedLastResult = last;
                        }
                    }

                    // 匹配成功则复用TrackId
                    if (matchedLastResult != null)
                    {
                        current.TrackId = matchedLastResult.TrackId;
                        usedLastFrameIds.Add(matchedLastResult.TrackId);
                    }

                    matchedResults.Add(current);
                }

                // 更新上一帧缓存
                _lastFrameResults.Clear();
                foreach (var result in matchedResults)
                {
                    _lastFrameResults[result.TrackId] = result;
                }

                return matchedResults;
            }
        }

        /// <summary>
        /// 计算IOU（交并比）
        /// </summary>
        /// <param name="rectA">矩形A</param>
        /// <param name="rectB">矩形B</param>
        /// <returns>IOU值</returns>
        private float CalculateIOU(OpenCvSharp.Rect rectA, OpenCvSharp.Rect rectB)
        {
            float x1 = Mathf.Max(rectA.X, rectB.X);
            float y1 = Mathf.Max(rectA.Y, rectB.Y);
            float x2 = Mathf.Min(rectA.X + rectA.Width, rectB.X + rectB.Width);
            float y2 = Mathf.Min(rectA.Y + rectA.Height, rectB.Y + rectB.Height);

            if (x2 < x1 || y2 < y1)
                return 0;

            float intersectionArea = (x2 - x1) * (y2 - y1);
            float areaA = rectA.Width * rectA.Height;
            float areaB = rectB.Width * rectB.Height;

            // 避免除零错误
            if (areaA + areaB - intersectionArea <= 0)
                return 0;

            return intersectionArea / (areaA + areaB - intersectionArea);
        }

        /// <summary>
        /// 绘制检测结果
        /// </summary>
        /// <param name="frame">输入图像</param>
        /// <param name="results">检测结果</param>
        private void DrawDetectionResults(Mat frame, List<YoloResult> results)
        {
            if (frame == null || frame.Empty() || results == null || results.Count == 0)
                return;

            foreach (var result in results)
            {
                // 修复 Rect 冲突：明确使用 OpenCvSharp.Rect
                OpenCvSharp.Rect rect = new OpenCvSharp.Rect(
                    (int)result.Rect.X,
                    (int)result.Rect.Y,
                    (int)result.Rect.Width,
                    (int)result.Rect.Height);

                // 绘制边框（不同类别不同颜色）
                Scalar color = GetClassColor(result.ClassId);
                Cv2.Rectangle(frame, rect, color, 2);

                // 绘制标签背景
                string label = $"{result.ClassName} {result.Confidence:F2} (ID:{result.TrackId})";
                int baseLine; // 修复CS1620：out参数显式声明
                Size labelSize = Cv2.GetTextSize(label, HersheyFonts.HersheySimplex, 0.5, 1, out baseLine);

                // 修复 Rect 冲突：明确使用 OpenCvSharp.Rect
                OpenCvSharp.Rect labelRect = new OpenCvSharp.Rect(
                    (int)result.Rect.X,
                    (int)result.Rect.Y - labelSize.Height - 2,
                    labelSize.Width,
                    labelSize.Height + baseLine + 2);

                Cv2.Rectangle(frame, labelRect, color, -1);
                Cv2.PutText(frame, label, new Point((int)result.Rect.X + 1, (int)result.Rect.Y - 2),
                    HersheyFonts.HersheySimplex, 0.5, Scalar.White, 1);
            }
        }

        /// <summary>
        /// 获取类别对应的颜色（修复Unity Color枚举缺失的颜色值）
        /// </summary>
        /// <param name="classId">类别ID</param>
        /// <returns>颜色值</returns>
        private Scalar GetClassColor(int classId)
        {
            // 替换Unity Color枚举中不存在的颜色为RGB值定义
            Color[] colors = new[]
            {
                Color.red, Color.green, Color.blue, Color.yellow, Color.cyan, Color.magenta,
                Color.gray, Color.white, Color.black, new Color(1f, 0.5f, 0f), // orange
                new Color(0.5f, 0f, 0.5f), // purple
                new Color(0.6f, 0.4f, 0.2f) // brown
            };

            int colorIndex = classId % colors.Length;
            return new Scalar(
                (byte)(colors[colorIndex].b * 255),
                (byte)(colors[colorIndex].g * 255),
                (byte)(colors[colorIndex].r * 255)
            );
        }

        /// <summary>
        /// 获取默认COCO80类别名称
        /// </summary>
        /// <returns>类别列表</returns>
        private List<string> GetDefaultCocoClassNames()
        {
            return new List<string>
            {
                "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck",
                "boat", "traffic light", "fire hydrant", "stop sign", "parking meter", "bench",
                "bird", "cat", "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe",
                "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard",
                "sports ball", "kite", "baseball bat", "baseball glove", "skateboard", "surfboard",
                "tennis racket", "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl",
                "banana", "apple", "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza",
                "donut", "cake", "chair", "couch", "potted plant", "bed", "dining table", "toilet",
                "tv", "laptop", "mouse", "remote", "keyboard", "cell phone", "microwave", "oven",
                "toaster", "sink", "refrigerator", "book", "clock", "vase", "scissors", "teddy bear",
                "hair drier", "toothbrush"
            };
        }

        /// <summary>
        /// 添加性能统计
        /// </summary>
        private void AddStat(string stage, float time)
        {
            lock (_statsLock)
            {
                if (!_inferenceTimeStats.ContainsKey(stage))
                {
                    _inferenceTimeStats[stage] = new List<float>();
                }
                _inferenceTimeStats[stage].Add(time);

                // 限制统计数据量，避免内存溢出
                if (_inferenceTimeStats[stage].Count > 1000)
                {
                    _inferenceTimeStats[stage].RemoveRange(0, _inferenceTimeStats[stage].Count - 1000);
                }
            }
        }


        /// <summary>
        /// 析构函数
        /// </summary>
        ~YoloV8Engine()
        {
            Dispose(false);
        }
        #endregion
    }

    /// <summary>
    /// YOLO检测结果类
    /// </summary>
    [Serializable]
    public class YoloResult
    {
        public int ClassId { get; set; }          // 类别ID
        public string ClassName { get; set; }     // 类别名称
        public float Confidence { get; set; }     // 置信度
        // 修复 Rect 冲突：明确使用 OpenCvSharp.Rect
        public OpenCvSharp.Rect Rect { get; set; }          // 检测框
        public int TrackId { get; set; }          // 追踪ID
    }
}