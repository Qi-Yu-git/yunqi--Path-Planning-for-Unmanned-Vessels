using UnityEngine;
using System.Collections.Generic;
using OpenCvSharp;
using System;
using System.Threading;
using System.Threading.Tasks;

public class YOLOv8Detector : MonoBehaviour
{
    [Header("YOLO配置")]
    public string modelPath = "Assets/Models/yolov8n.onnx"; // ONNX模型路径（需自行放置）
    public float confidenceThreshold = 0.5f; // 置信度阈值（可调整）
    public float iouThreshold = 0.4f; // IOU阈值（非极大值抑制）

    [Header("相机配置")]
    public Camera detectionCamera; // 检测用相机（需在Inspector赋值）
    public float maxDetectionDistance = 20f; // 最大检测距离（与局部安全距离匹配）
    public LayerMask obstacleLayers; // 障碍物图层（需在Inspector勾选障碍物层）

    [Header("性能配置")]
    public int detectionIntervalMs = 100; // 检测间隔(毫秒)，100=10FPS（平衡性能）
    public bool useAsyncDetection = true; // 异步检测（避免卡顿）

    private YoloV8Engine yoloEngine; // YOLO引擎实例
    private List<DetectedObject> detectedObjects = new List<DetectedObject>();
    private float lastDetectionTime;
    private CancellationTokenSource cts;

    // 检测到的物体数据结构
    public struct DetectedObject
    {
        public string className; // 类别名称（如boat、ship）
        public float confidence; // 置信度
        public OpenCvSharp.Rect2d screenRect; // 明确指定OpenCvSharp的Rect2d（解决命名冲突）
        public Vector3 worldPosition; // 世界坐标（用于避障）
        public Vector3 velocity; // 速度向量（暂未实现）
    }

    void Start()
    {
        // 自动赋值相机（若未手动指定）
        if (detectionCamera == null)
            detectionCamera = GetComponent<Camera>();

        // 初始化YOLO引擎
        try
        {
            yoloEngine = new YoloV8Engine(modelPath, confidenceThreshold, iouThreshold);
            Debug.Log("YOLOv8引擎初始化成功！模型路径：" + modelPath);
        }
        catch (Exception e)
        {
            Debug.LogError($"YOLOv8初始化失败: {e.Message}，请检查模型路径和OpenCV库是否导入");
        }

        cts = new CancellationTokenSource();
    }

    void Update()
    {
        if (yoloEngine == null) return;

        // 定时执行检测（避免每帧检测卡顿）
        if (Time.time - lastDetectionTime > detectionIntervalMs / 1000f)
        {
            lastDetectionTime = Time.time;
            if (useAsyncDetection)
                DetectAsync(); // 异步检测（推荐）
            else
                Detect(); // 同步检测（仅用于调试）
        }
    }

    // 同步检测（主线程执行，可能卡顿）
    private void Detect()
    {
        Mat frame = CaptureCameraFrame();
        if (frame.Empty()) return;

        var results = yoloEngine.Detect(frame);
        ProcessDetectionResults(results);
        frame.Release(); // 释放OpenCV资源
    }

    // 异步检测（子线程执行，不卡主线程）
    private async void DetectAsync()
    {
        try
        {
            Mat frame = CaptureCameraFrame();
            if (frame.Empty()) return;

            // 子线程执行推理
            var results = await Task.Run(() => yoloEngine.Detect(frame), cts.Token);
            ProcessDetectionResults(results);
            frame.Release();
        }
        catch (OperationCanceledException)
        {
            Debug.Log("YOLO检测任务已取消");
        }
    }
    // 捕获相机画面并转换为OpenCV格式（Mat）
    private Mat CaptureCameraFrame()
    {
        // 创建临时RenderTexture存储相机画面
        RenderTexture rt = new RenderTexture(Screen.width, Screen.height, 24);
        detectionCamera.targetTexture = rt;
        Texture2D frameTexture = new Texture2D(Screen.width, Screen.height, TextureFormat.RGB24, false);

        // 渲染相机画面到RenderTexture
        detectionCamera.Render();
        RenderTexture.active = rt;
        frameTexture.ReadPixels(new UnityEngine.Rect(0, 0, Screen.width, Screen.height), 0, 0);
        frameTexture.Apply();

        // 转换Texture2D到OpenCV的Mat（RGB格式）
        Color32[] pixels = frameTexture.GetPixels32();
        Mat frame = new Mat(Screen.height, Screen.width, MatType.CV_8UC3);

        // 转换像素数据为byte数组（RGB格式）
        byte[] pixelBytes = new byte[pixels.Length * 3];
        int index = 0;
        foreach (Color32 pixel in pixels)
        {
            pixelBytes[index++] = pixel.r;
            pixelBytes[index++] = pixel.g;
            pixelBytes[index++] = pixel.b;
        }

        // 修复：使用Marshal.Copy将byte[]复制到Mat的内存缓冲区
        System.Runtime.InteropServices.Marshal.Copy(
            pixelBytes,       // 源数组（byte[]）
            0,                // 源数组起始索引
            frame.Data,       // 目标内存指针（Mat的数据地址）
            pixelBytes.Length // 复制的字节数
        );

        // 颜色通道转换（Unity是RGB，OpenCV默认是BGR）
        Cv2.CvtColor(frame, frame, ColorConversionCodes.RGB2BGR);

        // 清理临时资源
        detectionCamera.targetTexture = null;
        RenderTexture.active = null;
        Destroy(rt);
        Destroy(frameTexture);

        return frame;
    }

    // 处理YOLO检测结果，转换为世界坐标
    private void ProcessDetectionResults(List<YoloResult> results)
    {
        detectedObjects.Clear();
        foreach (var result in results)
        {
            // 只保留障碍物类别（根据YOLO模型类别调整，此处适配COCO数据集）
            if (IsObstacleClass(result.ClassName))
            {
                DetectedObject obj = new DetectedObject();
                obj.className = result.ClassName;
                obj.confidence = result.Confidence;
                obj.screenRect = result.Rect;

                // 计算障碍物世界坐标（通过射线检测，确保精准定位）
                // 修复double转float错误：显式转换为float
                Vector2 screenCenter = new Vector2(
                    (float)(result.Rect.X + result.Rect.Width / 2),
                    (float)(result.Rect.Y + result.Rect.Height / 2)
                );
                Ray ray = detectionCamera.ScreenPointToRay(screenCenter);
                if (Physics.Raycast(ray, out RaycastHit hit, maxDetectionDistance, obstacleLayers))
                {
                    obj.worldPosition = hit.point;
                    detectedObjects.Add(obj);
                }
            }
        }
    }

    // 判定是否为需要避障的类别（可根据实际场景扩展）
    private bool IsObstacleClass(string className)
    {
        // COCO数据集类别：boat（船）、ship（轮船）、truck（卡车）等可视为水上障碍物
        return className == "boat" || className == "ship" || className == "truck" ||
               className == "car" || className == "bicycle" || className == "motorcycle";
    }

    // 供LocalPlanner调用：获取检测到的障碍物世界坐标
    public List<Vector3> GetDetectedObstaclePositions()
    {
        List<Vector3> positions = new List<Vector3>();
        foreach (var obj in detectedObjects)
        {
            positions.Add(obj.worldPosition);
        }
        return positions;
    }

    // 销毁时释放资源
    void OnDestroy()
    {
        cts?.Cancel();
        yoloEngine?.Dispose();
    }
}