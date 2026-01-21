// 顶部命名空间补充
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Runtime.InteropServices;
using OpenCvSharp;
using OpenCvSharp.Dnn;
using UnityEngine;

/// <summary>
/// YOLOv8目标检测引擎，封装OpenCV DNN推理功能（适配 (1,84,8400) 模型格式 + 低版本兼容）
/// 核心优化：适配84列模型（4坐标+80类别，无单独置信度列），修复检测解析逻辑
/// 新增功能：手动实现Mat转Texture2D（兼容低版本OpenCvSharp）
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

    // 日志控制字段
    private bool _logModelProcessing = true;
    private bool _logNmsResults = true;
    private float _lastAggregateLogTime;
    private Dictionary<string, int> _classCountAggregate = new Dictionary<string, int>();
    #endregion

    #region 公共属性
    public bool IsInitialized => _isInitialized;
    public Size InputSize => _inputSize;
    public IReadOnlyList<string> ClassNames => _classNames.AsReadOnly();

    // 日志控制配置
    [Header("日志控制")]
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
    // 在YoloV8Engine的构造函数中添加日志参数
    public YoloV8Engine(string modelPath, List<string> classNames = null,
                       float confidenceThreshold = 0.5f, float iouThreshold = 0.4f,
                       Size? inputSize = null, bool isNoSeparateConfidence = true,
                       bool logModelProcessing = false,  // 新增：控制模型处理日志
                       bool logNmsResults = false,       // 新增：控制NMS日志
                       float aggregateLogInterval = 5f)  // 新增：控制聚合日志间隔
    {
        _modelPath = modelPath;
        _confidenceThreshold = Mathf.Clamp01(confidenceThreshold);
        _iouThreshold = Mathf.Clamp01(iouThreshold);
        _classNames = classNames ?? GetDefaultCocoClassNames();
        _isNoSeparateConfidence = isNoSeparateConfidence;
        if (inputSize.HasValue) _inputSize = inputSize.Value;

        // ========== 接收外部日志配置 ==========
        _logModelProcessing = logModelProcessing;
        _logNmsResults = logNmsResults;
        AggregateLogInterval = aggregateLogInterval;

        try
        {
            _isInitialized = InitializeEngine();
        }
        catch (Exception ex)
        {
            Debug.LogError($"引擎初始化失败: {ex.Message}\n{ex.StackTrace}");
            _isInitialized = false;
        }
    }
    #endregion

    #region 公共方法
    public List<YoloResult> Detect(Mat frame)
    {
        // 增强空值校验，提前拦截可能的空引用
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
            try
            {
                int frameWidth = frame.Cols;
                int frameHeight = frame.Rows;
                using (var blob = PreprocessImage(frame))
                {
                    _net.SetInput(blob);
                    string[] outputLayerNames = _net.GetUnconnectedOutLayersNames();
                    Mat output = _net.Forward(outputLayerNames[0]);

                    // 核心修复：低版本兼容的维度转换（适配 (1,84,8400)）
                    if (_logModelProcessing)
                        Debug.Log($"原始输出形状: ({output.Size(0)}, {output.Size(1)}, {output.Size(2)})");

                    if (output.Dims == 3 && output.Size(0) == 1)
                    {
                        int channel = output.Size(1);
                        int boxCount = output.Size(2);

                        // 处理 (1,84,8400) 或 (1,85,8400) 格式
                        if (boxCount == 8400 && (channel == 84 || channel == 85))
                        {
                            // 低版本兼容方案：先压缩为2维 (channel, 8400) → 转置为 (8400, channel)
                            output = output.Reshape(1, channel); // (1,84,8400) → (84, 8400)
                            output = output.T();                // (84,8400) → (8400,84)
                        }
                        // 处理 (1,8400,84) 或 (1,8400,85) 格式
                        else if (channel == 8400 && (output.Size(2) == 84 || output.Size(2) == 85))
                        {
                            output = output.Reshape(1, 8400); // 直接压缩为 (8400,84)
                        }
                    }
                    // 处理 (84,8400) 或 (85,8400) 2维格式
                    else if (output.Dims == 2 && output.Rows != 8400 && output.Cols == 8400)
                    {
                        output = output.T(); // 转置为 (8400,84)
                    }

                    if (_logModelProcessing)
                        Debug.Log($"调整后形状: {output.Rows}行 x {output.Cols}列");

                    var results = ParseDetectionOutput(output, frameWidth, frameHeight);
                    output.Release();
                    return results;
                }
            }
            catch (Exception ex)
            {
                // 增强异常日志信息
                Debug.LogError($"🚫 检测出错：{ex.Message}\n堆栈信息：{ex.StackTrace}");
                Debug.LogError($"🚫 报错时状态：_net是否为空={(_net == null ? "是" : "否")}, " +
                              $"frame是否为空={(frame == null ? "是" : "否")}, " +
                              $"frame是否有效={(frame?.Empty() ?? true ? "否" : "是")}");
                return new List<YoloResult>();
            }
        }
    }

    // ===================== 修复核心：手动实现Mat转Texture2D（兼容低版本OpenCvSharp） =====================
    /// <summary>
    /// 将OpenCV的Mat图像转换为Unity的Texture2D（手动实现，兼容所有OpenCvSharp版本）
    /// </summary>
    /// <param name="frame">OpenCV图像矩阵</param>
    /// <returns>Unity纹理对象（null表示转换失败）</returns>
    public Texture2D ConvertMatToTexture(Mat frame)
    {
        // 空值/无效帧校验（与原代码校验风格一致）
        if (frame == null || frame.Empty())
        {
            Debug.LogError("❌ 无法转换空的Mat对象");
            return null;
        }

        try
        {
            // 1. 转换颜色空间：BGR → RGB（解决颜色颠倒）
            Mat rgbMat = new Mat();
            Cv2.CvtColor(frame, rgbMat, ColorConversionCodes.BGR2RGB);

            // 2. 获取图像数据
            int width = rgbMat.Cols;
            int height = rgbMat.Rows;
            int channels = rgbMat.Channels();

            // 3. 创建Texture2D（确保格式匹配）
            Texture2D texture = new Texture2D(width, height, TextureFormat.RGB24, false);

            // 4. 读取Mat数据到字节数组
            byte[] data = new byte[width * height * channels];
            Marshal.Copy(rgbMat.Data, data, 0, data.Length);

            // 5. 加载数据到Texture2D
            texture.LoadRawTextureData(data);
            texture.Apply();

            // 6. 释放临时Mat
            rgbMat.Release();

            return texture;
        }
        catch (Exception ex)
        {
            // 异常日志风格与原代码保持一致
            Debug.LogError($"🚫 Mat转Texture2D失败: {ex.Message}\n堆栈信息：{ex.StackTrace}");
            return null;
        }
    }

    // ===================== 新增扩展功能：检测+绘制+转纹理 =====================
    /// <summary>
    /// 一站式完成：检测目标 → 绘制检测框 → 转换为Unity纹理
    /// 直接返回可显示的检测结果纹理
    /// </summary>
    /// <param name="frame">输入图像</param>
    /// <param name="results">输出检测结果列表</param>
    /// <returns>带检测框的Unity纹理</returns>
    public Texture2D DetectAndConvertToTexture(Mat frame, out List<YoloResult> results)
    {
        results = Detect(frame); // 调用原有检测逻辑
        DrawDetectionResults(frame, results); // 绘制检测框
        return ConvertMatToTexture(frame); // 转换为纹理
    }

    public void UpdateThresholds(float confidenceThreshold, float iouThreshold)
    {
        _confidenceThreshold = Mathf.Clamp01(confidenceThreshold);
        _iouThreshold = Mathf.Clamp01(iouThreshold);
    }

    /// <summary>
    /// 切换模型格式（84列/85列）
    /// </summary>
    public void SetModelFormat(bool isNoSeparateConfidence)
    {
        _isNoSeparateConfidence = isNoSeparateConfidence;
        if (_logModelProcessing)
            Debug.Log($"🔄 模型格式已切换：{(isNoSeparateConfidence ? "84列（4坐标+80类别）" : "85列（4坐标+1置信度+80类别）")}");
    }

    /// <summary>
    /// 处理检测日志（包含聚合和过滤功能）
    /// </summary>
    public void ProcessDetectionLogs(List<YoloResult> results)
    {
        // 聚合统计
        foreach (var result in results)
        {
            if (_classCountAggregate.ContainsKey(result.ClassName))
                _classCountAggregate[result.ClassName]++;
            else
                _classCountAggregate[result.ClassName] = 1;
        }

        // 定时输出聚合结果
        if (Time.time - _lastAggregateLogTime > AggregateLogInterval)
        {
            if (_classCountAggregate.Count == 0) // 修复：Count() → Count（字典的Count是属性）
            {
                // 降低空结果日志频率
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

                // 检测到过多目标时警告
                int total = _classCountAggregate.Values.Sum();
                if (total > 50)
                    Debug.LogWarning($"⚠️ 检测到大量目标（{total}个），可能影响性能");
            }
            _classCountAggregate.Clear();
            _lastAggregateLogTime = Time.time;
        }

        // 输出详细日志（带过滤）
        if (results == null || results.Count == 0) return; // 修复：Count() → Count

        Debug.Log($"📌 检测到 {results.Count} 个目标"); // 修复：Count() → Count
        foreach (var result in results)
        {
            // 检查是否需要输出该类别的详细日志
            bool shouldLog = true;
            if (LogIncludedClasses.Count > 0 && !LogIncludedClasses.Contains(result.ClassName)) // 修复：Count() → Count
                shouldLog = false;
            if (LogExcludedClasses.Count > 0 && LogExcludedClasses.Contains(result.ClassName)) // 修复：Count() → Count
                shouldLog = false;

            if (shouldLog && result.Confidence > 0.8f)
            {
                Debug.Log($"  - 类别：{result.ClassName} | 置信度：{result.Confidence:F2} | 位置：({result.Rect.X:F1}, {result.Rect.Y:F1}, {result.Rect.Width:F1}, {result.Rect.Height:F1})");
            }
        }
    }

    public void Dispose()
    {
        Dispose(true);
        GC.SuppressFinalize(this);
    }
    #endregion

    #region 私有方法
    /// <summary>
    /// 图像预处理（适配低版本OpenCvSharp）
    /// </summary>
    private Mat PreprocessImage(Mat frame)
    {
        // 移除MatType参数，避免低版本不兼容
        return CvDnn.BlobFromImage(
            frame,
            1 / 255.0,
            _inputSize,
            new Scalar(0, 0, 0),
            true,  // 交换RB通道（OpenCV默认BGR，模型输入RGB）
            false);
    }

    /// <summary>
    /// 解析检测输出（核心修复：适配84列模型，修正置信度计算逻辑）
    /// </summary>
    private List<YoloResult> ParseDetectionOutput(Mat output, int frameWidth, int frameHeight)
    {
        var results = new List<YoloResult>();

        // 增加详细维度日志（带开关控制）
        if (_logModelProcessing)
            Debug.Log($"输出矩阵信息: 维度={output.Dims}, 形状=({output.Size(0)},{output.Size(1)})");

        // 校验输出矩阵有效性
        if (output == null || output.Empty())
        {
            Debug.LogError("❌ 解析失败：输入输出矩阵为空");
            return results;
        }

        int rows = output.Rows;
        int cols = output.Cols;

        // 修复核心：处理列数为-1的异常情况
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

        // 核心修复：自动适配84/85/4列模型
        int expectedCols = 0;
        if (cols == 84)
        {
            expectedCols = 84;
            _isNoSeparateConfidence = true; // 强制切换为84列模式
            Debug.LogWarning($"📌 检测到模型输出84列（4坐标+80类别），自动适配COCO80类模式");
        }
        else if (cols == 85)
        {
            expectedCols = 85;
            _isNoSeparateConfidence = false; // 强制切换为85列模式
            Debug.LogWarning($"📌 检测到模型输出85列（4坐标+1置信度+80类别），自动适配");
        }
        else if (cols == 4)
        {
            expectedCols = 4;
            Debug.LogWarning($"📌 检测到模型仅输出4列（纯坐标），自动适配无类别模式");
        }
        else
        {
            // 兼容原有逻辑
            expectedCols = _isNoSeparateConfidence
                ? 4 + _classNames.Count
                : 5 + _classNames.Count;
        }
        if (_logModelProcessing)
            Debug.Log($"📌 解析输出：行数={rows}, 列数={cols}, 预期列数={expectedCols}（模型格式：{(_isNoSeparateConfidence ? "84列" : "85列")}）");

        // 严格匹配列数（避免解析错误）
        if (cols != expectedCols)
        {
            Debug.LogError($"❌ 输出维度不匹配：实际{cols}列，预期{expectedCols}列");
            Debug.LogError($"💡 可能原因：1.模型类别数与配置不匹配 2.模型格式设置错误（当前设置：{(_isNoSeparateConfidence ? "84列" : "85列")}）");
            return results;
        }

        // 提取输出数据（兼容连续/非连续矩阵，修复低版本数据提取问题）
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
                // 低版本兼容：逐元素读取，避免GetArray方法异常
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
            return results;
        }

        // 遍历所有候选框（8400个）
        for (int i = 0; i < rows; i++)
        {
            int baseIndex = i * cols;

            // 1. 读取边界框参数（模型输出已归一化到0~1）
            float x = outputData[baseIndex + 0];
            float y = outputData[baseIndex + 1];
            float w = outputData[baseIndex + 2];
            float h = outputData[baseIndex + 3];

            // 2. 计算置信度（根据模型格式切换逻辑）
            float finalConfidence = 0;
            int maxClassId = -1;
            if (_isNoSeparateConfidence)
            {
                // 84列模型：无单独置信度列，取最大类别概率作为置信度
                float maxClassScore = 0;
                int classStartIndex = baseIndex + 4; // 类别从第4列开始
                int classEndIndex = baseIndex + 4 + _classNames.Count; // 修复：Count() → Count

                // 确保不超出数组范围
                if (classEndIndex > outputData.Length)
                    classEndIndex = outputData.Length;

                for (int j = classStartIndex; j < classEndIndex; j++)
                {
                    float classScore = outputData[j]; // 模型已通过Sigmoid激活（0~1）
                    if (classScore > maxClassScore)
                    {
                        maxClassScore = classScore;
                        maxClassId = j - classStartIndex; // 类别ID从0开始
                    }
                }

                finalConfidence = maxClassScore;
            }
            else
            {
                // 85列模型：单独置信度列 + 类别分数
                float boxConfidence = outputData[baseIndex + 4];
                if (boxConfidence < _confidenceThreshold)
                    continue; // 过滤低置信度框

                // 查找最高分数的类别
                float maxClassScore = 0;
                int classStartIndex = baseIndex + 5;
                int classEndIndex = baseIndex + 5 + _classNames.Count; // 修复：Count() → Count

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

                // 最终置信度 = 框置信度 × 类别分数
                finalConfidence = boxConfidence * maxClassScore;
            }

            // 过滤低置信度目标
            if (finalConfidence < _confidenceThreshold || maxClassId < 0)
                continue;

            // 3. 转换为图像实际坐标（反归一化 + 边界限制，修复负数坐标问题）
            float left = (x - w / 2) * frameWidth;  // 左上角x
            float top = (y - h / 2) * frameHeight;  // 左上角y
            float width = w * frameWidth;           // 宽度
            float height = h * frameHeight;         // 高度

            // 确保坐标在图像范围内（避免负数或超出图像大小）
            left = Mathf.Max(0, left);
            top = Mathf.Max(0, top);
            width = Mathf.Max(1, Mathf.Min(frameWidth - left, width));
            height = Mathf.Max(1, Mathf.Min(frameHeight - top, height));

            // 4. 构造检测结果（确保类别名称有效）
            string className = maxClassId < _classNames.Count // 修复：Count() → Count
                ? _classNames[maxClassId]
                : $"unknown_{maxClassId}";

            results.Add(new YoloResult
            {
                ClassId = maxClassId,
                ClassName = className,
                Confidence = finalConfidence,
                Rect = new Rect2d(left, top, width, height)
            });
        }

        // 5. 执行NMS去重（优化：按类别分组，避免不同类别互相抑制）
        return ApplyNonMaxSuppression(results);
    }

    /// <summary>
    /// 非极大值抑制（NMS）- 按类别分组，修复重复框问题
    /// 终极修复：完全适配最低版本OpenCvSharp的NMSBoxes调用方式
    /// </summary>
    private List<YoloResult> ApplyNonMaxSuppression(List<YoloResult> results)
    {
        if (results.Count == 0) // 修复：Count() → Count
        {
            if (_logNmsResults)
                Debug.Log("📌 NMS：无有效检测框");
            return results;
        }

        var nmsResults = new List<YoloResult>();

        // 按类别分组执行NMS，避免不同类别框互相抑制（如"人"和"车"）
        var classGroups = results.GroupBy(r => r.ClassId).ToList();
        foreach (var group in classGroups)
        {
            var groupResults = group.ToList();
            int groupCount = groupResults.Count; // 修复：Count() → Count

            // 准备NMS所需参数（转换为数组，适配最低版本API）
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

            // ========== 终极修复：最低版本OpenCvSharp的NMSBoxes调用方式 ==========
            // 兼容最老版本的API签名（所有参数都显式传递，且带out）
            int[] indices = new int[0];
            CvDnn.NMSBoxes(
                boxesArray,        // 参数1：边界框数组（必须是数组）
                confidencesArray,  // 参数2：置信度数组（必须是数组）
                _confidenceThreshold, // 参数3：置信度阈值
                _iouThreshold,     // 参数4：IOU阈值
                out indices);      // 参数5：输出索引（必须带out，核心修复CS1620）

            // 添加NMS后的结果
            foreach (int idx in indices)
            {
                if (idx >= 0 && idx < groupResults.Count) // 修复：Count() → Count
                    nmsResults.Add(groupResults[idx]);
            }
        }

        // 只在数量变化时输出NMS日志
        if (_logNmsResults && results.Count != nmsResults.Count) // 修复：Count() → Count
        {
            Debug.Log($"📌 NMS前：{results.Count}个框，NMS后：{nmsResults.Count}个框（按类别分组去重）"); // 修复：Count() → Count
        }
        return nmsResults;
    }

    /// <summary>
    /// 初始化引擎（修复库路径加载、模型验证问题）
    /// </summary>
    private bool InitializeEngine()
    {
        // ========== 1. 先检查模型文件（核心逻辑不能被跳过） ==========
        if (string.IsNullOrEmpty(_modelPath) || !File.Exists(_modelPath))
        {
            Debug.LogError($"模型文件不存在: {_modelPath}");
            return false;
        }

        try
        {
            // ========== 2. 配置库路径（核心逻辑） ==========
            var libPath = Path.Combine(Application.dataPath, "Packages/OpenCvSharp4.runtime.win.4.8.0.20230708/runtimes/win-x64/native/");
            if (!Directory.Exists(libPath))
            {
                libPath = Path.Combine(Application.dataPath, "Plugins/OpenCvSharp/");
            }
            Environment.SetEnvironmentVariable("PATH", $"{Environment.GetEnvironmentVariable("PATH")};{libPath}");

            // 仅在日志开启时输出库路径
            if (_logModelProcessing)
                Debug.Log($"✅ 已添加OpenCvSharp库路径：{libPath}");

            // ========== 3. 加载模型（核心逻辑） ==========
            _net = CvDnn.ReadNetFromOnnx(_modelPath);
            if (_net == null || _net.Empty())
            {
                Debug.LogError("❌ 模型加载失败，返回的网络为空或无效");
                return false;
            }

            // ========== 4. 配置后端 + 日志控制（可外部配置） ==========
            ConfigureNetBackend();

            // ========== 5. 日志控制（可通过外部参数调整） ==========
            _logModelProcessing = false;   // 控制“模型处理/加载”相关日志
            _logNmsResults = false;        // 控制“NMS去重”相关日志
            AggregateLogInterval = 5f;     // 控制“聚合统计”日志的输出间隔

            // ========== 6. 初始化成功日志（仅在日志开启时输出） ==========
            if (_logModelProcessing)
                Debug.Log($"✅ YOLOv8引擎初始化成功（模型格式：{(_isNoSeparateConfidence ? "84列" : "85列")}，类别数：{_classNames.Count}）"); // 修复：Count() → Count

            return true;
        }
        catch (DllNotFoundException ex)
        {
            Debug.LogError($"找不到OpenCvSharp原生库: {ex.Message}");
            Debug.LogError("💡 解决方案：1.检查OpenCvSharpExtern.dll是否存在 2.确认库版本与Unity架构匹配（x64）");
            return false;
        }
        catch (Exception ex)
        {
            Debug.LogError($"加载模型失败: {ex.Message}\n{ex.StackTrace}");
            return false;
        }
    }

    /// <summary>
    /// 配置网络后端（CPU兼容模式，避免CUDA依赖）
    /// </summary>
    private void ConfigureNetBackend()
    {
        if (_net == null) return;

        try
        {
            // 使用数值常量代替枚举（避免低版本OpenCvSharp枚举不存在）
            _net.SetPreferableBackend(0); // 0 = Backend.OPENCV
            _net.SetPreferableTarget(0);  // 0 = Target.CPU
            if (_logModelProcessing)
                Debug.Log("✅ 已配置CPU推理后端（兼容模式，无需CUDA）");
        }
        catch (Exception ex)
        {
            Debug.LogWarning($"⚠️ 配置后端时警告：{ex.Message}，使用默认配置");
        }
    }

    /// <summary>
    /// 默认COCO 80类名称列表（与官方一致，避免类别数量错误）
    /// </summary>
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

    // ===================== 新增私有方法：绘制检测结果 =====================
    /// <summary>
    /// 在Mat图像上绘制检测框和类别信息（与原代码风格一致）
    /// </summary>
    /// <param name="frame">待绘制的图像</param>
    /// <param name="results">检测结果列表</param>
    private void DrawDetectionResults(Mat frame, List<YoloResult> results)
    {
        if (frame == null || frame.Empty() || results == null || results.Count == 0) // 修复：Count() → Count
            return;

        foreach (var result in results)
        {
            // 绘制边界框（红色，线宽2）- 显式指定OpenCvSharp.Rect避免命名冲突
            OpenCvSharp.Rect rect = new OpenCvSharp.Rect(
                (int)result.Rect.X,
                (int)result.Rect.Y,
                (int)result.Rect.Width,
                (int)result.Rect.Height);

            Cv2.Rectangle(
                frame,
                rect,
                Scalar.Red,
                2);

            // 绘制类别+置信度标签背景
            string label = $"{result.ClassName} {result.Confidence:F2}";
            int baseLine; // 修复：移除数组，直接用int变量
            // ========== 核心修复：GetTextSize的baseLine参数加out ==========
            Size labelSize = Cv2.GetTextSize(label, HersheyFonts.HersheySimplex, 0.5, 1, out baseLine);

            // 显式指定OpenCvSharp.Rect
            OpenCvSharp.Rect labelRect = new OpenCvSharp.Rect(
                (int)result.Rect.X,
                (int)result.Rect.Y - labelSize.Height - 2,
                labelSize.Width,
                labelSize.Height + baseLine + 2);

            Cv2.Rectangle(
                frame,
                labelRect,
                Scalar.Red,
                -1); // 填充背景

            // 绘制标签文字（白色）
            Cv2.PutText(
                frame,
                label,
                new Point((int)result.Rect.X + 1, (int)result.Rect.Y - 2),
                HersheyFonts.HersheySimplex,
                0.5,
                Scalar.White,
                1);
        }
    }


    /// <summary>
    /// 释放资源（修复资源泄漏问题）
    /// </summary>
    protected virtual void Dispose(bool disposing)
    {
        if (disposing)
        {
            // 释放托管资源
            _classNames?.Clear();
            _classCountAggregate?.Clear();
        }
        // 核心修复：释放OpenCV网络资源（避免内存泄漏导致的维度解析异常）
        if (_net != null)
        {
            _net.Dispose(); // 正确的释放方法

            _net = null;
        }
    }
    #endregion // 补充缺失的#endregion，修复CS1038
}