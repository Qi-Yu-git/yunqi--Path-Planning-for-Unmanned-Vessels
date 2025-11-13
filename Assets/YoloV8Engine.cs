using System;
using System.Collections.Generic;
using OpenCvSharp;
using OpenCvSharp.Dnn;
using UnityEngine; // 添加Unity命名空间（解决Debug未定义）

public class YoloV8Engine : IDisposable
{
    private Net net; // OpenCV DNN网络实例
    private float confidenceThreshold; // 置信度阈值
    private float iouThreshold; // IOU阈值
    // COCO数据集类别名称（YOLOv8预训练模型默认类别，顺序不可修改）
    private readonly List<string> classNames = new List<string>
    {
        "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck",
        "boat", "traffic light", "fire hydrant", "stop sign", "parking meter", "bench",
        "bird", "cat", "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra",
        "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee",
        "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove",
        "skateboard", "surfboard", "tennis racket", "bottle", "wine glass", "cup",
        "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange",
        "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch",
        "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse",
        "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink",
        "refrigerator", "book", "clock", "vase", "scissors", "teddy bear", "hair drier",
        "toothbrush"
    };

    /// <summary>
    /// 初始化YOLOv8引擎
    /// </summary>
    /// <param name="modelPath">ONNX模型路径</param>
    /// <param name="confidenceThreshold">置信度阈值</param>
    /// <param name="iouThreshold">IOU阈值</param>
    public YoloV8Engine(string modelPath, float confidenceThreshold = 0.5f, float iouThreshold = 0.4f)
    {
        this.confidenceThreshold = confidenceThreshold;
        this.iouThreshold = iouThreshold;

        // 加载ONNX模型（OpenCV DNN自动解析）
        net = CvDnn.ReadNetFromOnnx(modelPath);
        if (net.Empty())
            throw new Exception($"无法加载YOLOv8模型，请检查路径：{modelPath}");

        // 修复：移除CUDA相关代码（OpenCvSharp4.runtime.windows默认不包含CUDA模块，避免编译错误）
        // 若需GPU加速，需手动安装带CUDA的OpenCV，并替换OpenCvSharp4的DLL
        net.SetPreferableBackend(Backend.OPENCV);
        net.SetPreferableTarget(Target.CPU);
        Debug.Log("YOLOv8使用CPU推理（如需GPU加速，需配置带CUDA的OpenCV环境）");
    }

    /// <summary>
    /// 执行目标检测
    /// </summary>
    /// <param name="frame">输入图像（OpenCV Mat格式）</param>
    /// <returns>检测结果列表</returns>
    public List<YoloResult> Detect(Mat frame)
    {
        int frameWidth = frame.Cols;
        int frameHeight = frame.Rows;

        // 图像预处理：缩放至YOLOv8输入尺寸（640x640）、归一化
        Mat blob = CvDnn.BlobFromImage(
            frame, 1 / 255.0, new Size(640, 640),
            new Scalar(0, 0, 0), true, false
        );
        net.SetInput(blob);

        // 模型推理（前向传播）
        // 修复：用Mat[]替代MatVector（OpenCvSharp4部分版本不兼容MatVector）
        Mat[] outputs = new Mat[1];
        string[] outputLayerNames = net.GetUnconnectedOutLayersNames();
        net.Forward(outputs, outputLayerNames); // 显式指定输出层名称

        // 解析推理结果
        List<YoloResult> results = ParseDetectionOutput(outputs[0], frameWidth, frameHeight);

        // 释放临时资源
        blob.Release();
        foreach (var output in outputs) output.Release();

        return results;
    }

    /// <summary>
    /// 解析YOLOv8输出格式（[1, 84, 8400]）
    /// </summary>
    private List<YoloResult> ParseDetectionOutput(Mat output, int frameWidth, int frameHeight)
    {
        List<YoloResult> results = new List<YoloResult>();
        int rows = output.Rows; // 8400个候选框
        int cols = output.Cols; // 84 = 4（坐标）+ 1（置信度）+ 80（类别分数）

        for (int i = 0; i < rows; i++)
        {
            // 获取候选框置信度（是否包含目标）
            float boxConfidence = output.At<float>(i, 4);
            if (boxConfidence < confidenceThreshold)
                continue;

            // 计算最高置信度的类别
            int classId = -1;
            float maxClassScore = 0;
            for (int j = 5; j < cols; j++)
            {
                float classScore = output.At<float>(i, j);
                if (classScore > maxClassScore)
                {
                    maxClassScore = classScore;
                    classId = j - 5; // 类别ID（0-79）
                }
            }

            // 过滤低置信度类别
            if (maxClassScore < confidenceThreshold || classId < 0)
                continue;

            // 计算真实图像坐标（反归一化）
            float cx = output.At<float>(i, 0) * frameWidth; // 中心点X
            float cy = output.At<float>(i, 1) * frameHeight; // 中心点Y
            float w = output.At<float>(i, 2) * frameWidth; // 宽度
            float h = output.At<float>(i, 3) * frameHeight; // 高度
            float x1 = cx - w / 2; // 左上角X
            float y1 = cy - h / 2; // 左上角Y

            // 添加到结果列表
            results.Add(new YoloResult
            {
                ClassId = classId,
                ClassName = classNames[classId],
                Confidence = boxConfidence * maxClassScore, // 综合置信度
                Rect = new Rect2d(x1, y1, w, h)
            });
        }

        // 非极大值抑制（NMS）：去除重复检测框
        results = ApplyNonMaxSuppression(results);

        return results;
    }

    /// <summary>
    /// 非极大值抑制（NMS）：过滤重叠检测框
    /// </summary>
    private List<YoloResult> ApplyNonMaxSuppression(List<YoloResult> results)
    {
        if (results.Count == 0)
            return results;

        // 提取置信度和边界框
        float[] confidences = new float[results.Count];
        Rect2d[] boxes = new Rect2d[results.Count];
        for (int i = 0; i < results.Count; i++)
        {
            confidences[i] = results[i].Confidence;
            boxes[i] = results[i].Rect;
        }

        // 执行NMS
        int[] indices;
        CvDnn.NMSBoxes(boxes, confidences, confidenceThreshold, iouThreshold, out indices);

        // 筛选NMS后的结果
        List<YoloResult> nmsResults = new List<YoloResult>();
        foreach (int idx in indices)
        {
            nmsResults.Add(results[idx]);
        }

        return nmsResults;
    }

    /// <summary>
    /// 释放资源（实现IDisposable接口）
    /// </summary>
    public void Dispose()
    {
        net?.Dispose();
    }
}

/// <summary>
/// YOLO检测结果数据结构
/// </summary>
public class YoloResult
{
    public int ClassId { get; set; } // 类别ID
    public string ClassName { get; set; } // 类别名称
    public float Confidence { get; set; } // 综合置信度
    public Rect2d Rect { get; set; } // 边界框（屏幕坐标）
}