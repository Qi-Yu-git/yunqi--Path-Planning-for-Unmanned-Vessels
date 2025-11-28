using System;
using System.Collections.Generic;
using System.IO;
using OpenCvSharp;
using OpenCvSharp.Dnn;
using UnityEngine;

/// <summary>
/// YOLOv8目标检测引擎，封装OpenCV DNN推理功能
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
    #endregion

    #region 公共属性
    public bool IsInitialized => _isInitialized;
    public Size InputSize => _inputSize;
    public IReadOnlyList<string> ClassNames => _classNames.AsReadOnly();
    #endregion

    #region 构造函数
    public YoloV8Engine(string modelPath, List<string> classNames = null,
                       float confidenceThreshold = 0.5f, float iouThreshold = 0.4f,
                       Size? inputSize = null)
    {
        _modelPath = modelPath;
        _confidenceThreshold = Mathf.Clamp01(confidenceThreshold);
        _iouThreshold = Mathf.Clamp01(iouThreshold);
        _classNames = classNames ?? GetDefaultCocoClassNames();
        if (inputSize.HasValue) _inputSize = inputSize.Value;

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
        if (!_isInitialized || frame == null || frame.Empty())
        {
            Debug.LogWarning("检测条件不满足");
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
                    Mat[] outputs = new Mat[1];
                    string[] outputLayerNames = _net.GetUnconnectedOutLayersNames();
                    _net.Forward(outputs, outputLayerNames);

                    var results = ParseDetectionOutput(outputs[0], frameWidth, frameHeight);
                    foreach (var output in outputs)
                        output.Release();

                    return results;
                }
            }
            catch (Exception ex)
            {
                Debug.LogError($"检测出错: {ex.Message}");
                return new List<YoloResult>();
            }
        }
    }

    public void UpdateThresholds(float confidenceThreshold, float iouThreshold)
    {
        _confidenceThreshold = Mathf.Clamp01(confidenceThreshold);
        _iouThreshold = Mathf.Clamp01(iouThreshold);
    }

    public void Dispose()
    {
        Dispose(true);
        GC.SuppressFinalize(this);
    }
    #endregion

    #region 私有方法
    private Mat PreprocessImage(Mat frame)
    {
        // 修正BlobFromImage参数（移除MatType参数，适配低版本OpenCvSharp）
        return CvDnn.BlobFromImage(
            frame,
            1 / 255.0,
            _inputSize,
            new Scalar(0, 0, 0),
            true,
            false);
    }

    private List<YoloResult> ParseDetectionOutput(Mat output, int frameWidth, int frameHeight)
    {
        var results = new List<YoloResult>();
        int rows = output.Rows;
        int cols = output.Cols;

        if (cols != 5 + _classNames.Count)
        {
            Debug.LogError("输出维度不匹配");
            return results;
        }

        // 修正GetArray方法调用（使用低版本兼容写法）
        float[] outputData = new float[rows * cols];
        output.GetArray(out outputData);

        for (int i = 0; i < rows; i++)
        {
            int baseIndex = i * cols;
            float boxConfidence = outputData[baseIndex + 4];
            if (boxConfidence < _confidenceThreshold)
                continue;

            int classId = -1;
            float maxClassScore = 0;
            for (int j = 5; j < cols; j++)
            {
                float classScore = outputData[baseIndex + j];
                if (classScore > maxClassScore)
                {
                    maxClassScore = classScore;
                    classId = j - 5;
                }
            }

            if (maxClassScore < _confidenceThreshold || classId < 0 || classId >= _classNames.Count)
                continue;

            float cx = outputData[baseIndex + 0] * frameWidth;
            float cy = outputData[baseIndex + 1] * frameHeight;
            float w = outputData[baseIndex + 2] * frameWidth;
            float h = outputData[baseIndex + 3] * frameHeight;

            float x1 = Mathf.Max(0, cx - w / 2);
            float y1 = Mathf.Max(0, cy - h / 2);
            float x2 = Mathf.Min(frameWidth, cx + w / 2);
            float y2 = Mathf.Min(frameHeight, cy + h / 2);

            results.Add(new YoloResult
            {
                ClassId = classId,
                ClassName = _classNames[classId],
                Confidence = boxConfidence * maxClassScore,
                Rect = new Rect2d(x1, y1, x2 - x1, y2 - y1)
            });
        }

        return ApplyNonMaxSuppression(results);
    }

    private List<YoloResult> ApplyNonMaxSuppression(List<YoloResult> results)
    {
        if (results.Count == 0)
            return results;

        float[] confidences = new float[results.Count];
        Rect2d[] boxes = new Rect2d[results.Count];

        for (int i = 0; i < results.Count; i++)
        {
            confidences[i] = results[i].Confidence;
            boxes[i] = results[i].Rect;
        }

        int[] indices;
        CvDnn.NMSBoxes(boxes, confidences, _confidenceThreshold, _iouThreshold, out indices);

        var nmsResults = new List<YoloResult>();
        foreach (int idx in indices)
        {
            if (idx >= 0 && idx < results.Count)
                nmsResults.Add(results[idx]);
        }

        return nmsResults;
    }

    private bool InitializeEngine()
    {
        if (string.IsNullOrEmpty(_modelPath) || !File.Exists(_modelPath))
        {
            Debug.LogError($"模型文件不存在: {_modelPath}");
            return false;
        }

        try
        {
            Debug.Log($"开始加载模型: {_modelPath}");
            _net = CvDnn.ReadNetFromOnnx(_modelPath);

            if (_net == null || _net.Empty())
            {
                Debug.LogError("模型加载失败，网络为空");
                return false;
            }

            ConfigureNetBackend();
            Debug.Log("模型加载成功");
            return true;
        }
        catch (Exception ex)
        {
            Debug.LogError($"加载模型时发生异常: {ex.Message}\n{ex.StackTrace}");
            return false;
        }
    }

    // 修正版：配置网络后端（移除Cuda和GraphicsDeviceType相关检测）
    private void ConfigureNetBackend()
    {
        if (_net == null) return;

        // 直接使用CPU后端配置，避免CUDA相关API调用
        ConfigureCpuBackend();
    }

    // CPU后端配置
    private void ConfigureCpuBackend()
    {
        _net.SetPreferableBackend(Backend.OPENCV);
        _net.SetPreferableTarget(Target.CPU);
        Debug.Log("使用CPU推理");
    }

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

    protected virtual void Dispose(bool disposing)
    {
        if (disposing)
        {
            // 释放托管资源
        }
        // 释放非托管资源
        if (_net != null)
        {
            _net.Dispose();
            _net = null;
        }
    }

    ~YoloV8Engine()
    {
        Dispose(false);
    }
    #endregion
}