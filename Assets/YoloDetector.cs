using UnityEngine;
using OpenCvSharp;
using System;
using System.Collections.Generic;
using System.IO;

// 避免Rect类型冲突（明确指定Unity的Rect）
using UnityRect = UnityEngine.Rect;

/// <summary>
/// YOLOv8目标检测Unity组件
/// 封装图像采集、格式转换和检测结果可视化
/// </summary>
public class YoloDetector : MonoBehaviour
{
    [Header("模型配置")]
    public string modelPath = "yolov8n.onnx"; // 模型文件名（默认放在StreamingAssets目录）
    public float confidenceThreshold = 0.5f;  // 置信度阈值（0-1）
    public float iouThreshold = 0.4f;         // IOU阈值（0-1）

    [Header("数据源选择")]
    [Tooltip("切换使用场景相机/USB摄像头")]
    public bool useSceneCamera = false;
    public Camera sceneCamera;                // 场景中的检测相机
    public WebCamTexture webCamTexture;       // USB摄像头纹理

    [Header("显示配置")]
    public bool logDetectionResults = true;   // 是否输出检测日志
    public bool drawBoundingBoxes = true;     // 是否在Game视图绘制边界框
    public Color boxColor = Color.red;        // 边界框颜色
    public Color labelColor = Color.green;    // 标签背景色

    private YoloV8Engine _yoloEngine;         // YOLO引擎实例
    private Mat _frameMat;                    // OpenCV图像缓存
    private Texture2D _sceneCamTexture;       // 场景相机纹理
    private RenderTexture _tempRenderTexture; // 临时渲染纹理（场景相机用）
    private GUIStyle _boxStyle;               // 边界框GUI样式
    private GUIStyle _labelStyle;             // 标签GUI样式

    private YoloV8Engine yoloEngine;
    void Start()
    {
        // 初始化YOLO引擎
        InitYoloEngine();

        // 初始化GUI样式
        InitGUIStyles();

        // 初始化数据源
        if (useSceneCamera)
        {
            InitSceneCamera();
        }
        else
        {
            InitWebCamera();
        }
    }

    void Update()
    {
        // 增加更严格的空值检查
        if (_yoloEngine == null || !_yoloEngine.IsInitialized)
        {
            Debug.LogWarning("YOLO引擎未初始化，跳过检测");
            return;
        }

        // 处理不同的数据源帧
        if (useSceneCamera && sceneCamera != null)
        {
            ProcessSceneCameraFrame();
        }
        else if (!useSceneCamera && webCamTexture != null && webCamTexture.isPlaying)
        {
            ProcessWebCameraFrame();
        }
    }

    /// <summary>
    /// 初始化YOLO引擎（含路径验证）
    /// </summary>
    private void InitYoloEngine()
    {
        try
        {
            // 确保StreamingAssets目录存在
            if (!Directory.Exists(Application.streamingAssetsPath))
            {
                Directory.CreateDirectory(Application.streamingAssetsPath);
                Debug.LogWarning("创建了缺失的StreamingAssets目录，请将模型文件放入该目录");
            }

            // 拼接完整模型路径
            string fullModelPath = Path.Combine(Application.streamingAssetsPath, modelPath);

            // 验证模型文件存在性
            if (!File.Exists(fullModelPath))
            {
                Debug.LogError($"模型文件不存在：{fullModelPath}\n请检查路径是否正确");
                return;
            }

            // 初始化引擎（使用默认COCO类别）
            _yoloEngine = new YoloV8Engine(
                fullModelPath,
                confidenceThreshold: confidenceThreshold,
                iouThreshold: iouThreshold
            );

            if (_yoloEngine.IsInitialized)
            {
                Debug.Log("YOLO引擎初始化成功");
            }
            else
            {
                Debug.LogError("YOLO引擎初始化失败");
            }
        }
        catch (Exception e)
        {
            Debug.LogError($"初始化YOLO引擎出错：{e.Message}\n{e.StackTrace}");
        }
    }

    /// <summary>
    /// 初始化场景相机
    /// </summary>
    private void InitSceneCamera()
    {
        if (sceneCamera == null)
        {
            // 自动查找名为"DetectionCamera"的相机
            sceneCamera = GameObject.Find("DetectionCamera")?.GetComponent<Camera>();
            if (sceneCamera == null)
            {
                Debug.LogError("未找到场景相机，请在Inspector中指定");
                return;
            }
        }

        // 创建临时渲染纹理
        _tempRenderTexture = new RenderTexture(
            sceneCamera.pixelWidth,
            sceneCamera.pixelHeight,
            24
        );
        sceneCamera.targetTexture = _tempRenderTexture;

        // 创建场景相机纹理
        _sceneCamTexture = new Texture2D(
            _tempRenderTexture.width,
            _tempRenderTexture.height,
            TextureFormat.RGBA32,
            false
        );

        Debug.Log("场景相机初始化完成");
    }

    /// <summary>
    /// 初始化USB摄像头
    /// </summary>
    private void InitWebCamera()
    {
        if (webCamTexture == null)
        {
            WebCamDevice[] devices = WebCamTexture.devices;
            if (devices.Length == 0)
            {
                Debug.LogError("未检测到可用的USB摄像头");
                return;
            }

            // 使用第一个可用摄像头，设置分辨率为1280x720
            webCamTexture = new WebCamTexture(devices[0].name, 1280, 720);
        }

        if (!webCamTexture.isPlaying)
        {
            webCamTexture.Play();
            Debug.Log("USB摄像头启动成功");
        }
    }

    /// <summary>
    /// 处理场景相机帧
    /// </summary>
    private void ProcessSceneCameraFrame()
    {
        // 读取渲染纹理到Texture2D
        RenderTexture.active = _tempRenderTexture;
        _sceneCamTexture.ReadPixels(
            new UnityRect(0, 0, _tempRenderTexture.width, _tempRenderTexture.height),
            0, 0
        );
        _sceneCamTexture.Apply();
        RenderTexture.active = null;

        // 转换为OpenCV Mat并检测
        _frameMat = Texture2DToMat(_sceneCamTexture);
        List<YoloResult> results = _yoloEngine.Detect(_frameMat);

        // 处理检测结果
        ProcessDetectionResults(results, _tempRenderTexture.width, _tempRenderTexture.height);

        // 释放资源
        _frameMat.Release();
    }

    /// <summary>
    /// 处理USB摄像头帧
    /// </summary>
    private void ProcessWebCameraFrame()
    {
        // 转换为OpenCV Mat并检测
        _frameMat = WebCamTextureToMat(webCamTexture);
        List<YoloResult> results = _yoloEngine.Detect(_frameMat);

        // 处理检测结果
        ProcessDetectionResults(results, webCamTexture.width, webCamTexture.height);

        // 释放资源
        _frameMat.Release();
    }

    /// <summary>
    /// WebCamTexture转换为OpenCV Mat
    /// </summary>
    private Mat WebCamTextureToMat(WebCamTexture texture)
    {
        // 临时Texture2D存储摄像头数据
        Texture2D tempTex = new Texture2D(texture.width, texture.height, TextureFormat.RGBA32, false);
        tempTex.SetPixels(texture.GetPixels());
        tempTex.Apply();

        // 转换为Mat并翻转（解决摄像头镜像问题）
        Mat mat = Texture2DToMat(tempTex);
        Cv2.Flip(mat, mat, FlipMode.Y); // Y轴翻转

        // 释放临时纹理
        Destroy(tempTex);
        return mat;
    }

    /// <summary>
    /// Texture2D转换为OpenCV Mat（通用方法）
    /// </summary>
    private Mat Texture2DToMat(Texture2D texture)
    {
        byte[] bytes = texture.EncodeToPNG();
        Mat mat = Cv2.ImDecode(bytes, ImreadModes.Color); // 解码为BGR格式
        return mat;
    }

    /// <summary>
    /// 处理检测结果（日志+绘制）
    /// </summary>
    private void ProcessDetectionResults(List<YoloResult> results, int frameWidth, int frameHeight)
    {
        if (results == null || results.Count == 0)
        {
            if (logDetectionResults)
                Debug.Log("未检测到任何目标");
            return;
        }

        // 输出检测日志
        if (logDetectionResults)
        {
            Debug.Log($"检测到{results.Count}个目标：");
            foreach (var result in results)
            {
                Debug.Log(result.ToString());
            }
        }

        // 绘制边界框（延迟到OnGUI执行）
        if (drawBoundingBoxes)
        {
            foreach (var result in results)
            {
                DrawBoundingBox(result, frameWidth, frameHeight);
            }
        }
    }

    /// <summary>
    /// 初始化GUI样式（边界框和标签）
    /// </summary>
    private void InitGUIStyles()
    {
        // 边界框样式
        _boxStyle = new GUIStyle
        {
            normal = { background = MakeTex(1, 1, new Color(0, 0, 0, 0)) }, // 透明背景
            border = new RectOffset(2, 2, 2, 2)
        };

        // 标签样式
        _labelStyle = new GUIStyle
        {
            normal = { background = MakeTex(1, 1, labelColor), textColor = Color.white },
            padding = new RectOffset(5, 5, 2, 2),
            fontSize = 12
        };
    }

    /// <summary>
    /// 在Game视图绘制边界框（通过OnGUI）
    /// </summary>
    private void OnGUI()
    {
        if (!drawBoundingBoxes || _boxStyle == null || _labelStyle == null)
            return;

        // 确保在Repaint事件时绘制（避免重复绘制）
        if (Event.current.type != EventType.Repaint)
            return;
    }

    /// <summary>
    /// 绘制单个目标的边界框和标签
    /// </summary>
    private void DrawBoundingBox(YoloResult result, int frameWidth, int frameHeight)
    {
        float screenScaleX = (float)Screen.width / frameWidth;
        float screenScaleY = (float)Screen.height / frameHeight;

        // 明确访问正确的Rect成员
        float x = (float)result.Rect.X * screenScaleX;
        float y = (float)(frameHeight - result.Rect.Y - result.Rect.Height) * screenScaleY;
        float width = (float)result.Rect.Width * screenScaleX;
        float height = (float)result.Rect.Height * screenScaleY;

        _boxStyle.normal.textColor = boxColor;
        GUI.Box(new UnityRect(x, y, width, height), "", _boxStyle);

        // 明确访问ClassName和Confidence成员
        string label = $"{result.ClassName} {result.Confidence:F2}";
        GUI.Label(new UnityRect(x, y - 20, width, 20), label, _labelStyle);
    }

    /// <summary>
    /// 创建纯色纹理（用于GUI样式）
    /// </summary>
    private Texture2D MakeTex(int width, int height, Color color)
    {
        Color[] pixels = new Color[width * height];
        for (int i = 0; i < pixels.Length; i++)
        {
            pixels[i] = color;
        }
        Texture2D tex = new Texture2D(width, height);
        tex.SetPixels(pixels);
        tex.Apply();
        return tex;
    }

    /// <summary>
    /// 释放资源（避免内存泄漏）
    /// </summary>
    void OnDestroy()
    {
        // 释放YOLO引擎
        _yoloEngine?.Dispose();

        // 停止摄像头
        if (webCamTexture != null && webCamTexture.isPlaying)
        {
            webCamTexture.Stop();
        }

        // 释放场景相机资源
        if (sceneCamera != null)
        {
            sceneCamera.targetTexture = null;
        }
        Destroy(_tempRenderTexture);
        Destroy(_sceneCamTexture);

        Debug.Log("检测资源已释放");
    }
}