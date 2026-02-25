using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Runtime.InteropServices;
using System.Diagnostics;
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

        // 日志控制字段
        private bool _logModelProcessing = true;
        private bool _logNmsResults = true;
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
        #endregion

        #region 公共属性
        public bool IsInitialized => _isInitialized;
        public Size InputSize => _inputSize;
        public IReadOnlyList<string> ClassNames => _classNames.AsReadOnly();
        public bool UseCuda { get => _useCuda; set => _useCuda = value; } // 暴露CUDA控制

        // 日志控制配置
        public bool LogModelProcessing
        {
            get => _logModelProcessing;
            set => _logModelProcessing = value;
        }
        public bool LogNmsResults
        {
            get => _logNmsResults;
            set => _logNmsResults = value;
        }
        public float AggregateLogInterval { get; set; } = 1f; // 聚合日志输出间隔（秒）
        public List<string> LogIncludedClasses { get; set; } = new List<string>();
        public List<string> LogExcludedClasses { get; set; } = new List<string>();
        #endregion

        #region 构造函数
        /// <summary>
        /// 构造函数
        /// </summary>
        /// <param name="modelPath">模型文件路径</param>
        /// <param name="classNames">类别名称列表（默认COCO80类）</param>
        /// <param name="confidenceThreshold">置信度阈值</param>
        /// <param name="iouThreshold">IOU阈值</param>
        /// <param name="inputSize">模型输入尺寸</param>
        /// <param name="isNoSeparateConfidence">是否为84列模型（无单独置信度列）</param>
        /// <param name="logModelProcessing">是否输出模型处理日志</param>
        /// <param name="logNmsResults">是否输出NMS日志</param>
        /// <param name="aggregateLogInterval">聚合日志输出间隔</param>
        /// <param name="autoWarmUp">是否自动预热模型</param>
        /// <param name="useCuda">是否优先使用CUDA推理</param>
        public YoloV8Engine(string modelPath, List<string> classNames = null,
                           float confidenceThreshold = 0.5f, float iouThreshold = 0.4f,
                           Size? inputSize = null, bool isNoSeparateConfidence = true,
                           bool logModelProcessing = false,
                           bool logNmsResults = false,
                           float aggregateLogInterval = 5f,
                           bool autoWarmUp = true,
                           bool useCuda = true)
        {
            _modelPath = modelPath;
            _confidenceThreshold = Mathf.Clamp01(confidenceThreshold);
            _iouThreshold = Mathf.Clamp01(iouThreshold);
            _classNames = classNames ?? GetDefaultCocoClassNames();
            _isNoSeparateConfidence = isNoSeparateConfidence;
            _useCuda = useCuda;
            if (inputSize.HasValue) _inputSize = inputSize.Value;

            // 接收外部日志配置
            _logModelProcessing = logModelProcessing;
            _logNmsResults = logNmsResults;
            AggregateLogInterval = aggregateLogInterval;

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
                Debug.LogError($"引擎初始化失败: {ex.Message}\n{ex.StackTrace}");
                _isInitialized = false;
            }
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
            // 增强空值校验
            if (_net == null || (_net != null && _net.Empty()))
            {
                Debug.LogError($"❌ 检测前校验失败：YOLO模型未初始化！_net状态：{(_net == null ? "null" : "Empty")}");
                return new List<YoloResult>();
            }
            if (!_isInitialized)
            {
                Debug.LogError("❌ 检测前校验失败：引擎未初始化完成");
                return new List<YoloResult>();
            }
            if (frame == null || frame.Empty())
            {
                Debug.LogError("❌ 检测前校验失败：输入帧为空或无效");
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

                            if (_logModelProcessing)
                                Debug.Log($"原始输出形状: ({output.Size(0)}, {output.Size(1)}, {output.Size(2)})");

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

                            if (_logModelProcessing)
                                Debug.Log($"调整后形状: {output.Rows}行 x {output.Cols}列");

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
                        Debug.LogWarning($"⚠️ 推理后端配置警告：{ex.Message}（自动适配CPU模式）");
                    }
                    else
                    {
                        Debug.LogError($"🚫 检测出错：{ex.Message}\n堆栈信息：{ex.StackTrace}");
                        Debug.LogError($"🚫 报错时状态：_net是否为空={(_net == null ? "是" : "否")}, " +
                                      $"frame是否为空={(frame == null ? "是" : "否")}, " +
                                      $"frame是否有效={(frame?.Empty() ?? true ? "否" : "是")}");
                    }

                    // 自动降级到CPU并重试（增加_net判空）
                    try
                    {
                        if (_net == null || _net.Empty())
                        {
                            Debug.LogError("❌ _net为空，无法重试");
                            return new List<YoloResult>();
                        }

                        Debug.LogWarning("🔄 尝试降级到CPU后端重试检测...");
                        _net.SetPreferableBackend((Backend)0);
                        _net.SetPreferableTarget((Target)0);
                        return RetryDetectWithCpu(frame);
                    }
                    catch (Exception retryEx)
                    {
                        Debug.LogError($"❌ CPU重试也失败：{retryEx.Message}");
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
                Debug.LogError("❌ CPU重试失败：_net未初始化");
                return new List<YoloResult>();
            }

            if (frame == null || frame.Empty())
            {
                Debug.LogError("❌ CPU重试失败：输入帧无效");
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
                Debug.LogError($"❌ RetryDetectWithCpu 执行失败：{ex.Message}");
                return new List<YoloResult>();
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
                Debug.LogError("❌ 批量检测失败：输入无效或引擎未初始化");
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
                    Debug.LogError($"❌ 批量检测失败：{ex.Message}\n{ex.StackTrace}");
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
                Debug.LogError("❌ 无法转换空的Mat对象");
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
                Debug.LogError($"🚫 Mat转Texture2D失败: {ex.Message}\n堆栈信息：{ex.StackTrace}");
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
            if (_logModelProcessing)
                Debug.Log($"🔄 模型格式已切换：{(isNoSeparateConfidence ? "84列（4坐标+80类别）" : "85列（4坐标+1置信度+80类别）")}");
        }

        /// <summary>
        /// 处理检测日志（聚合+过滤）
        /// </summary>
        /// <param name="results">检测结果</param>
        public void ProcessDetectionLogs(List<YoloResult> results)
        {
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
                        Debug.Log("📌 聚合统计：未检测到任何目标");
                }
                else
                {
                    string aggregateLog = "📊 聚合统计：";
                    foreach (var kvp in _classCountAggregate)
                    {
                        aggregateLog += $"{kvp.Key}({kvp.Value}) ";
                    }
                    Debug.Log(aggregateLog);

                    int total = _classCountAggregate.Values.Sum();
                    if (total > 50)
                        Debug.LogWarning($"⚠️ 检测到大量目标（{total}个），可能影响性能");
                }
                _classCountAggregate.Clear();
                _lastAggregateLogTime = Time.time;
            }

            // 输出详细日志
            if (results.Count == 0 || !_logNmsResults) return;

            Debug.Log($"📌 检测到 {results.Count} 个目标");
            foreach (var result in results)
            {
                bool shouldLog = true;
                if (LogIncludedClasses.Count > 0 && !LogIncludedClasses.Contains(result.ClassName))
                    shouldLog = false;
                if (LogExcludedClasses.Count > 0 && LogExcludedClasses.Contains(result.ClassName))
                    shouldLog = false;

                if (shouldLog && result.Confidence > 0.8f)
                {
                    Debug.Log($"  - 类别：{result.ClassName} | 置信度：{result.Confidence:F2} | 位置：({result.Rect.X:F1}, {result.Rect.Y:F1}, {result.Rect.Width:F1}, {result.Rect.Height:F1}) | TrackId：{result.TrackId}");
                }
            }
        }

        /// <summary>
        /// 模型预热（解决首次检测卡顿，修复Backend/Target枚举异常）
        /// </summary>
        /// <returns>预热是否成功</returns>
        public bool WarmUpModel()
        {
            if (!_isInitialized || _net == null || _net.Empty())
            {
                Debug.LogError("❌ 模型未初始化，无法预热");
                return false;
            }

            lock (_lockObj)
            {
                try
                {
                    using (var dummyMat = new Mat(_inputSize.Height, _inputSize.Width, MatType.CV_8UC3, Scalar.All(0)))
                    {
                        // 复用预处理逻辑，避免重复代码
                        using (var blob = PreprocessImage(dummyMat))
                        {
                            _net.SetInput(blob);
                            string[] outputLayerNames = _net.GetUnconnectedOutLayersNames();
                            string targetLayer = outputLayerNames.Length > 0 ? outputLayerNames[0] : "";

                            // 提前捕获后端不兼容问题，避免抛出异常
                            try
                            {
                                _net.Forward(targetLayer);
                            }
                            catch (Exception forwardEx)
                            {
                                Debug.LogWarning($"⚠️ 预热推理失败，自动降级CPU: {forwardEx.Message}");
                                // 强制CPU后端（使用数值枚举，避免版本兼容问题）
                                _net.SetPreferableBackend((Backend)0); // DNN_BACKEND_OPENCV
                                _net.SetPreferableTarget((Target)0);   // DNN_TARGET_CPU
                                _net.Forward(targetLayer);
                            }
                        }
                    }

                    if (_logModelProcessing)
                        Debug.Log("✅ 模型预热完成，首次检测无卡顿");
                    return true;
                }
                catch (Exception ex)
                {
                    // 仅记录警告，不统计为错误（预热失败不影响核心功能）
                    Debug.LogWarning($"⚠️ 模型预热警告：{ex.Message}（不影响检测功能）");
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
                Debug.LogError($"❌ 新模型文件不存在：{newModelPath}");
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
                        Debug.LogError("❌ 新模型加载失败");
                        _isInitialized = false;
                        return false;
                    }

                    // 配置后端
                    ConfigureNetBackend();
                    _isInitialized = true;

                    // 预热新模型
                    WarmUpModel();

                    if (_logModelProcessing)
                        Debug.Log($"✅ 模型切换成功：{newModelPath}");
                    return true;
                }
                catch (Exception ex)
                {
                    Debug.LogError($"❌ 模型切换失败：{ex.Message}\n{ex.StackTrace}");
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
            Dispose(true);
            GC.SuppressFinalize(this);
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
                Debug.LogWarning($"⚠️ Blob维度异常：({blob.Size(0)},{blob.Size(1)},{blob.Size(2)},{blob.Size(3)})，强制重置为(1,3,{_inputSize.Height},{_inputSize.Width})");
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

            if (_logModelProcessing)
                Debug.Log($"输出矩阵信息: 维度={output.Dims}, 形状=({output.Size(0)},{output.Size(1)})");

            if (output == null || output.Empty())
            {
                Debug.LogError("❌ 解析失败：输入输出矩阵为空");
                return results;
            }

            int rows = output.Rows;
            int cols = output.Cols;

            // 修复列数异常
            if (cols <= 0)
            {
                Debug.LogError($"❌ 解析失败：无效列数={cols}，尝试从矩阵形状重新获取");
                if (output.Dims >= 2)
                {
                    rows = output.Size(0);
                    cols = output.Size(1);
                    Debug.LogWarning($"🔄 重新获取维度：rows={rows}, cols={cols}");
                }
                if (cols <= 0)
                {
                    Debug.LogError("❌ 无法获取有效维度，解析终止");
                    return results;
                }
            }

            // 自动适配模型列数
            int expectedCols = 0;
            if (cols == 84)
            {
                expectedCols = 84;
                _isNoSeparateConfidence = true;
                if (_logModelProcessing)
                    Debug.Log($"📌 检测到模型输出84列（4坐标+80类别），自动适配COCO80类模式");
            }
            else if (cols == 85)
            {
                expectedCols = 85;
                _isNoSeparateConfidence = false;
                if (_logModelProcessing)
                    Debug.Log($"📌 检测到模型输出85列（4坐标+1置信度+80类别），自动适配");
            }
            else if (cols == 4)
            {
                expectedCols = 4;
                if (_logModelProcessing)
                    Debug.Log($"📌 检测到模型仅输出4列（纯坐标），自动适配无类别模式");
            }
            else
            {
                expectedCols = _isNoSeparateConfidence
                    ? 4 + _classNames.Count
                    : 5 + _classNames.Count;
            }

            if (_logModelProcessing)
                Debug.Log($"📌 解析输出：行数={rows}, 列数={cols}, 预期列数={expectedCols}（模型格式：{(_isNoSeparateConfidence ? "84列" : "85列")}）");

            // 列数校验
            if (cols != expectedCols)
            {
                Debug.LogError($"❌ 输出维度不匹配：实际{cols}列，预期{expectedCols}列");
                Debug.LogError($"💡 可能原因：1.模型类别数与配置不匹配 2.模型格式设置错误（当前设置：{(_isNoSeparateConfidence ? "84列" : "85列")}）");
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
                Debug.LogError($"❌ 提取输出数据失败：{ex.Message}\n{ex.StackTrace}");
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

                results.Add(new YoloResult
                {
                    ClassId = maxClassId,
                    ClassName = className,
                    Confidence = finalConfidence,
                    Rect = new Rect2d(left, top, width, height),
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
                if (_logNmsResults)
                    Debug.Log("📌 NMS：无有效检测框");
                return results;
            }

            var nmsResults = new List<YoloResult>();
            var classGroups = results.GroupBy(r => r.ClassId).ToList();

            foreach (var group in classGroups)
            {
                var groupResults = group.ToList();
                int groupCount = groupResults.Count;

                OpenCvSharp.Rect[] boxesArray = new OpenCvSharp.Rect[groupCount];
                float[] confidencesArray = new float[groupCount];

                for (int i = 0; i < groupCount; i++)
                {
                    var result = groupResults[i];
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

            if (_logNmsResults && results.Count != nmsResults.Count)
            {
                Debug.Log($"📌 NMS前：{results.Count}个框，NMS后：{nmsResults.Count}个框（按类别分组去重）");
            }

            return nmsResults;
        }

        private bool InitializeEngine()
        {
            if (string.IsNullOrEmpty(_modelPath))
            {
                Debug.LogWarning("⚠️ 模型路径为空，引擎未初始化");
                return false;
            }
            if (!File.Exists(_modelPath))
            {
                Debug.LogError($"❌ 模型文件不存在: {_modelPath}");
                return false;
            }

            // ========== 新增：4.7路径兼容处理 ==========
            // 4.7对相对路径解析有问题，强制转为绝对路径
            _modelPath = Path.GetFullPath(_modelPath);
            // 替换路径分隔符（4.7不识别/）
            _modelPath = _modelPath.Replace('/', '\\');
            Debug.Log($"📌 4.7兼容路径：{_modelPath}");
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
                        if (_logModelProcessing)
                            Debug.Log($"✅ 已添加OpenCvSharp库路径：{libPath}");
                        // 移除break，保证所有有效路径都被添加（避免漏加依赖）
                    }
                }

                // 加载模型（增强判空逻辑）
                _net = CvDnn.ReadNetFromOnnx(_modelPath);
                if (_net == null || _net.Empty())
                {
                    Debug.LogError("❌ 模型加载失败，返回的网络为空或无效");
                    return false;
                }

                // 适配OpenCVSharp 4.7.0：移除DnnInvoke依赖，直接验证CUDA可用性
                try
                {
                    // 第一步：尝试配置CUDA后端（用数值枚举避免枚举名不兼容）
                    _net.SetPreferableBackend((Backend)3); // DNN_BACKEND_CUDA
                    _net.SetPreferableTarget((Target)6);   // DNN_TARGET_CUDA

                    // 第二步：用空推理验证CUDA是否真的可用（避免配置成功但运行失败）
                    using (var dummyBlob = CvDnn.BlobFromImage(
                        new Mat(_inputSize.Height, _inputSize.Width, MatType.CV_8UC3, Scalar.All(0)),
                        1.0 / 255.0,
                        _inputSize,
                        new Scalar(0, 0, 0),
                        swapRB: true,
                        crop: false))
                    {
                        _net.SetInput(dummyBlob);
                        string[] outputLayers = _net.GetUnconnectedOutLayersNames();
                        _net.Forward(outputLayers.Length > 0 ? outputLayers[0] : "");

                        // 能走到这里说明CUDA完全可用
                        Debug.Log("✅ CUDA配置成功！YOLO将使用GPU推理");
                        _useCuda = true;
                    }
                }
                catch (Exception ex)
                {
                    // CUDA配置/推理失败，自动降级到CPU
                    Debug.LogWarning($"⚠️ CUDA不可用（原因：{ex.Message}），自动切换到CPU推理");
                    _net.SetPreferableBackend((Backend)0); // DNN_BACKEND_OPENCV
                    _net.SetPreferableTarget((Target)0);   // DNN_TARGET_CPU
                    _useCuda = false;
                }


                // 配置推理后端（核心：显式指定4.7兼容的后端/目标）
                ConfigureNetBackend();

                // 初始化日志间隔
                if (AggregateLogInterval <= 0)
                {
                    AggregateLogInterval = 5f;
                }

                if (_logModelProcessing)
                    Debug.Log($"✅ YOLOv8引擎初始化成功（模型格式：{(_isNoSeparateConfidence ? "84列" : "85列")}，类别数：{_classNames.Count}）");

                return true;
            }
            catch (DllNotFoundException ex)
            {
                Debug.LogError($"找不到OpenCvSharp原生库: {ex.Message}");
                Debug.LogError("💡 解决方案：1.检查OpenCvSharpExtern.dll是否存在 2.确认库版本与Unity架构匹配（x64） 3.将库文件放到Plugins/x86_64目录");
                return false;
            }
            catch (Exception ex)
            {
                // 增强错误日志：补充4.7版本相关排查点
                Debug.LogError($"加载模型失败: {ex.Message}\n{ex.StackTrace}");
                Debug.LogError($"💡 4.7版本排查：1.模型OPSET是否≤11 2.环境变量是否生效 3.模型路径是否为绝对路径（当前路径：{_modelPath}）");
                return false;
            }
        }
        /// <summary>
        /// 配置推理后端（兼容OpenCVSharp 4.7，优先CUDA降级CPU）
        /// </summary>
        private void ConfigureNetBackend()
        {
            if (_net == null)
            {
                if (_logModelProcessing)
                    Debug.LogWarning("⚠️ _net 为空，跳过推理后端配置");
                return;
            }

            try
            {
                if (_useCuda)
                {
                    // 尝试CUDA配置，失败自动降级
                    try
                    {
                        _net.SetPreferableBackend((Backend)3); // DNN_BACKEND_CUDA
                        _net.SetPreferableTarget((Target)6);   // DNN_TARGET_CUDA
                        if (_logModelProcessing)
                            Debug.Log("✅ 已配置CUDA GPU推理后端");
                    }
                    catch
                    {
                        // CUDA配置失败，自动降级CPU
                        _net.SetPreferableBackend((Backend)0);
                        _net.SetPreferableTarget((Target)0);
                        _useCuda = false; // 标记为禁用CUDA，避免重复尝试
                        if (_logModelProcessing)
                            Debug.LogWarning("⚠️ CUDA配置失败，降级到CPU后端");
                    }
                }
                else
                {
                    // 强制CPU后端
                    _net.SetPreferableBackend((Backend)0);
                    _net.SetPreferableTarget((Target)0);
                    if (_logModelProcessing)
                        Debug.Log("✅ 已配置CPU推理后端");
                }
            }
            catch (Exception ex)
            {
                Debug.LogWarning($"⚠️ 配置推理后端失败，使用默认CPU: {ex.Message}");
                // 终极兜底
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
        private float CalculateIOU(Rect2d rectA, Rect2d rectB)
        {
            float x1 = Mathf.Max((float)rectA.X, (float)rectB.X);
            float y1 = Mathf.Max((float)rectA.Y, (float)rectB.Y);
            float x2 = Mathf.Min((float)rectA.X + (float)rectA.Width, (float)rectB.X + (float)rectB.Width);
            float y2 = Mathf.Min((float)rectA.Y + (float)rectA.Height, (float)rectB.Y + (float)rectB.Height);

            if (x2 < x1 || y2 < y1)
                return 0;

            float intersectionArea = (x2 - x1) * (y2 - y1);
            float areaA = (float)(rectA.Width * rectA.Height);
            float areaB = (float)(rectB.Width * rectB.Height);

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
        /// 获取默认COCO80类名称
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
        /// 添加性能统计数据
        /// </summary>
        /// <param name="stage">统计阶段</param>
        /// <param name="time">耗时（毫秒）</param>
        private void AddStat(string stage, float time)
        {
            if (!_inferenceTimeStats.ContainsKey(stage))
                _inferenceTimeStats[stage] = new List<float>();

            // 仅保留最近100条数据，防止内存溢出
            if (_inferenceTimeStats[stage].Count >= 100)
                _inferenceTimeStats[stage].RemoveAt(0);

            _inferenceTimeStats[stage].Add(time);
        }

        /// <summary>
        /// 析构函数（兜底释放）
        /// </summary>
        ~YoloV8Engine()
        {
            Dispose(false);
        }

        /// <summary>
        /// 释放资源（完善版）
        /// </summary>
        /// <param name="disposing">是否释放托管资源</param>
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
        #endregion
    }

    /// <summary>
    /// YOLO检测结果类（确保全局唯一定义，解决二义性）
    /// </summary>
    [Serializable]
    public class YoloResult
    {
        public int ClassId { get; set; }          // 类别ID
        public string ClassName { get; set; }     // 类别名称
        public float Confidence { get; set; }     // 置信度
        public Rect2d Rect { get; set; }          // 检测框
        public int TrackId { get; set; }          // 追踪ID
    }
}