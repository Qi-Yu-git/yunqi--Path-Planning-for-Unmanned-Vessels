using UnityEngine;
using OpenCvSharp;
using System;
using System.Collections.Generic;
using System.IO;
using System.Threading.Tasks;

// 避免Rect类型冲突（明确指定Unity的Rect）
using UnityRect = UnityEngine.Rect;

/// <summary>
/// YOLOv8目标检测Unity组件（优化版）
/// 特性：线程安全、资源自动释放、双数据源支持、检测框正确绘制
/// </summary>
public class YoloDetector : MonoBehaviour
{
    [Header("模型核心配置")]
    [Tooltip("模型文件名（需放在StreamingAssets目录）")]
    public string modelPath = "yolov8n.onnx";
    [Tooltip("置信度阈值（0-1，值越高检测越严格）")]
    public float confidenceThreshold = 0.5f;
    [Tooltip("IOU阈值（0-1，用于过滤重复检测框）")]
    public float iouThreshold = 0.4f;

    [Header("数据源配置")]
    [Tooltip("true=使用场景相机，false=使用USB摄像头")]
    public bool useSceneCamera = true;
    [Tooltip("场景检测相机（需提前在场景中创建）")]
    public Camera sceneCamera;
    [Tooltip("USB摄像头分辨率（默认1280x720）")]
    public Vector2 webCamResolution = new(1280, 720);

    [Header("显示与性能配置")]
    [Tooltip("是否在控制台输出检测日志")]
    public bool logDetectionResults = true;
    [Tooltip("是否在Game视图绘制检测框")]
    public bool drawBoundingBoxes = true;
    [Tooltip("检测框颜色")]
    public Color boxColor = Color.red;
    [Tooltip("标签背景色")]
    public Color labelColor = Color.green;
    [Tooltip("检测间隔（秒），值越小检测越频繁（建议≥0.05）")]
    public float detectInterval = 0.1f;
    [Tooltip("检测框线宽（像素）")]
    public int boxLineWidth = 2;
    [Tooltip("标签字体大小")]
    public int labelFontSize = 12;

    // 私有成员
    private YoloV8Engine _yoloEngine;
    private Mat _frameMat;
    private Texture2D _sceneCamTexture;
    private RenderTexture _tempRenderTexture;
    private WebCamTexture _webCamTexture;
    private GUIStyle _boxStyle;
    private GUIStyle _labelStyle;
    private float _lastDetectTime;
    private readonly object _resultLock = new();
    private List<YoloResult> _detectionResults = new();
    private int _lastFrameWidth;
    private int _lastFrameHeight;

    void Start()
    {
        try
        {
            // 初始化顺序：样式 → 数据源 → 引擎
            InitGUIStyles();
            InitDataSource();
            InitYoloEngine();

            // 初始化主线程调度器
            UnityMainThreadDispatcher.Init();
        }
        catch (Exception e)
        {
            Debug.LogError($"初始化失败：{e.Message}\n{e.StackTrace}");
        }
    }

    void Update()
    {
        // 检测频率控制
        if (Time.time - _lastDetectTime < detectInterval) return;
        _lastDetectTime = Time.time;

        // 引擎未就绪则跳过
        if (_yoloEngine == null || !_yoloEngine.IsInitialized)
        {
            Debug.LogWarning("YOLO引擎未初始化，跳过检测");
            return;
        }

        // 异步处理检测（避免阻塞主线程）
        _ = ProcessDetectionAsync();
    }

    /// <summary>
    /// 异步处理检测流程（优化性能）
    /// </summary>
    private async Task ProcessDetectionAsync()
    {
        try
        {
            List<YoloResult> results = new();
            int frameWidth = 0;
            int frameHeight = 0;

            // 根据数据源获取帧并检测
            if (useSceneCamera && sceneCamera != null)
            {
                (Mat sceneMat, int w, int h) = await CaptureSceneCameraFrameAsync();
                if (sceneMat != null && !sceneMat.Empty())
                {
                    results = _yoloEngine.Detect(sceneMat);
                    frameWidth = w;
                    frameHeight = h;
                    sceneMat.Release();
                }
            }
            else if (!useSceneCamera && _webCamTexture != null && _webCamTexture.isPlaying)
            {
                (Mat webMat, int w, int h) = CaptureWebCameraFrame();
                if (webMat != null && !webMat.Empty())
                {
                    results = _yoloEngine.Detect(webMat);
                    frameWidth = w;
                    frameHeight = h;
                    webMat.Release();
                }
            }

            // 线程安全更新检测结果
            lock (_resultLock)
            {
                _detectionResults = results;
                _lastFrameWidth = frameWidth;
                _lastFrameHeight = frameHeight;
            }

            // 主线程输出日志（使用静态类调用方式）
            if (logDetectionResults)
            {
                UnityMainThreadDispatcher.Enqueue(() =>
                {
                    ProcessDetectionLogs(results);
                });
            }
        }
        catch (Exception e)
        {
            Debug.LogError($"检测异常：{e.Message}\n{e.StackTrace}");
        }
    }

    /// <summary>
    /// 初始化YOLO引擎（含路径验证和异常处理）
    /// </summary>
    private void InitYoloEngine()
    {
        try
        {
            // 验证StreamingAssets目录
            if (!Directory.Exists(Application.streamingAssetsPath))
            {
                Directory.CreateDirectory(Application.streamingAssetsPath);
                Debug.LogWarning("已自动创建StreamingAssets目录，请将模型文件放入该目录");
            }

            // 拼接完整路径
            string fullModelPath = Path.Combine(Application.streamingAssetsPath, modelPath);

            // 验证模型文件
            if (!File.Exists(fullModelPath))
            {
                Debug.LogError($"模型文件不存在：{fullModelPath}\n请检查文件路径和名称是否正确");
                return;
            }

            // 初始化引擎
            _yoloEngine = new YoloV8Engine(
                fullModelPath,
                confidenceThreshold: confidenceThreshold,
                iouThreshold: iouThreshold,
                inputSize: new Size(640, 640)
            );

            if (_yoloEngine.IsInitialized)
            {
                Debug.Log($"✅ YOLO引擎初始化成功！类别数：{_yoloEngine.ClassNames.Count}");
            }
            else
            {
                Debug.LogError("❌ YOLO引擎初始化失败");
            }
        }
        catch (DllNotFoundException e)
        {
            Debug.LogError($"❌ 缺少OpenCvSharp依赖库：{e.Message}\n请确保已导入OpenCvSharp相关包");
        }
        catch (Exception e)
        {
            Debug.LogError($"❌ 引擎初始化异常：{e.Message}\n{e.StackTrace}");
        }
    }

    /// <summary>
    /// 初始化数据源（场景相机/USB摄像头）
    /// </summary>
    private void InitDataSource()
    {
        if (useSceneCamera)
        {
            InitSceneCamera();
        }
        else
        {
            InitWebCamera();
        }
    }

    /// <summary>
    /// 初始化场景相机
    /// </summary>
    private void InitSceneCamera()
    {
        // 自动查找相机
        if (sceneCamera == null)
        {
            sceneCamera = GameObject.Find("DetectionCamera")?.GetComponent<Camera>();
            if (sceneCamera == null)
            {
                sceneCamera = Camera.main;
                Debug.LogWarning("未指定场景相机，自动使用主相机");
            }
        }

        // 创建渲染纹理
        _tempRenderTexture = new RenderTexture(
            sceneCamera.pixelWidth,
            sceneCamera.pixelHeight,
            24,
            RenderTextureFormat.ARGB32
        );
        sceneCamera.targetTexture = _tempRenderTexture;

        // 创建纹理缓存
        _sceneCamTexture = new Texture2D(
            _tempRenderTexture.width,
            _tempRenderTexture.height,
            TextureFormat.RGBA32,
            false
        );

        Debug.Log($"✅ 场景相机初始化完成：分辨率({_tempRenderTexture.width}x{_tempRenderTexture.height})");
    }

    /// <summary>
    /// 初始化USB摄像头
    /// </summary>
    private void InitWebCamera()
    {
        // 检查摄像头设备
        WebCamDevice[] devices = WebCamTexture.devices;
        if (devices.Length == 0)
        {
            Debug.LogError("❌ 未检测到可用的USB摄像头");
            return;
        }

        // 初始化摄像头纹理
        _webCamTexture = new WebCamTexture(
            devices[0].name,
            (int)webCamResolution.x,
            (int)webCamResolution.y,
            30
        );

        // 启动摄像头
        _webCamTexture.Play();
        if (_webCamTexture.isPlaying)
        {
            Debug.Log($"✅ USB摄像头启动成功：{devices[0].name}，分辨率({_webCamTexture.width}x{_webCamTexture.height})");
        }
        else
        {
            Debug.LogError("❌ USB摄像头启动失败");
        }
    }

    /// <summary>
    /// 异步捕获场景相机帧（避免阻塞主线程）
    /// </summary>
    /// <summary>
    /// 异步捕获场景相机帧（修复主线程调用问题）
    /// </summary>
    // 修改CaptureSceneCameraFrameAsync方法中的主线程调度部分
    private async Task<(Mat, int, int)> CaptureSceneCameraFrameAsync()
    {
        int frameWidth = 0;
        int frameHeight = 0;
        byte[] imageBytes = null;

        var tcs = new TaskCompletionSource<bool>();
        UnityMainThreadDispatcher.Enqueue(() =>
        {
            try
            {
                lock (sceneCamera)
                {
                    if (_tempRenderTexture == null || _sceneCamTexture == null)
                    {
                        tcs.SetResult(false);
                        return;
                    }

                    // 读取渲染纹理（必须在主线程执行）
                    RenderTexture.active = _tempRenderTexture;
                    _sceneCamTexture.ReadPixels(
                        new UnityRect(0, 0, _tempRenderTexture.width, _tempRenderTexture.height),
                        0, 0
                    );
                    _sceneCamTexture.Apply();
                    RenderTexture.active = null;

                    frameWidth = _tempRenderTexture.width;
                    frameHeight = _tempRenderTexture.height;

                    // 在主线程中执行EncodeToPNG
                    imageBytes = _sceneCamTexture.EncodeToPNG();

                    tcs.SetResult(true);
                }
            }
            catch (Exception ex)
            {
                tcs.SetException(ex);
            }
        });

        // 等待主线程操作完成
        await tcs.Task;

        // 在子线程中进行纹理转换（不涉及Unity API）
        return await Task.Run(() =>
        {
            if (imageBytes == null) return (null, 0, 0);

            // 转换为Mat
            Mat mat = Texture2DToMat(imageBytes);
            return (mat, frameWidth, frameHeight);
        });
    }
    /// <summary>
    /// 捕获USB摄像头帧
    private (Mat, int, int) CaptureWebCameraFrame()
    {
        if (_webCamTexture == null || !_webCamTexture.isPlaying) return (null, 0, 0);

        // 转换为Texture2D（在主线程执行）
        Texture2D tempTex = new Texture2D(
            _webCamTexture.width,
            _webCamTexture.height,
            TextureFormat.RGBA32,
            false
        );
        tempTex.SetPixels(_webCamTexture.GetPixels());
        tempTex.Apply();

        // 在主线程中执行EncodeToPNG
        byte[] bytes = tempTex.EncodeToPNG();

        // 释放临时纹理
        Destroy(tempTex);

        // 转换为Mat并翻转（解决镜像问题）
        Mat mat = Texture2DToMat(bytes);
        Cv2.Flip(mat, mat, FlipMode.Y);

        return (mat, _webCamTexture.width, _webCamTexture.height);
    }
    /// <summary>
    /// Texture2D转OpenCV Mat（优化颜色空间转换）
    /// </summary>
    /// <summary>
    /// Texture2D转OpenCV Mat（优化颜色空间转换）
    /// </summary>
    private Mat Texture2DToMat(byte[] imageBytes)
    {
        Mat mat = Cv2.ImDecode(imageBytes, ImreadModes.Color); // BGR格式
        Cv2.CvtColor(mat, mat, ColorConversionCodes.BGR2RGB); // 转为RGB格式（匹配YOLO输入）
        return mat;
    }

    /// <summary>
    /// 处理检测日志输出
    /// </summary>
    private void ProcessDetectionLogs(List<YoloResult> results)
    {
        if (results == null || results.Count == 0)
        {
            Debug.Log("📌 未检测到任何目标");
            return;
        }

        Debug.Log($"📌 检测到 {results.Count} 个目标：");
        foreach (var result in results)
        {
            Debug.Log($"  - 类别：{result.ClassName} | 置信度：{result.Confidence:F2} | 位置：({result.Rect.X:F1}, {result.Rect.Y:F1}, {result.Rect.Width:F1}, {result.Rect.Height:F1})");
        }
    }

    /// <summary>
    /// 初始化GUI样式（优化绘制效果）
    /// </summary>
    private void InitGUIStyles()
    {
        // 检测框样式（仅绘制边框，无背景）
        _boxStyle = new GUIStyle
        {
            normal = { background = MakeTex(1, 1, new Color(0, 0, 0, 0)) },
            border = new RectOffset(boxLineWidth, boxLineWidth, boxLineWidth, boxLineWidth)
        };

        // 标签样式（半透明背景，居中文字）
        _labelStyle = new GUIStyle
        {
            normal = { background = MakeTex(1, 1, labelColor), textColor = Color.white },
            padding = new RectOffset(8, 8, 2, 2),
            fontSize = labelFontSize,
            alignment = TextAnchor.MiddleCenter,
            wordWrap = false
        };
    }

    /// <summary>
    /// OnGUI绘制检测框（正确的绘制时机）
    /// </summary>
    private void OnGUI()
    {
        if (!drawBoundingBoxes || _boxStyle == null || _labelStyle == null) return;
        if (_detectionResults == null || _detectionResults.Count == 0) return;

        // 线程安全访问检测结果
        lock (_resultLock)
        {
            foreach (var result in _detectionResults)
            {
                DrawSingleBoundingBox(result);
            }
        }
    }

    /// <summary>
    /// 绘制单个目标的检测框和标签（适配屏幕分辨率）
    /// </summary>
    private void DrawSingleBoundingBox(YoloResult result)
    {
        if (_lastFrameWidth == 0 || _lastFrameHeight == 0) return;

        // 计算屏幕缩放比例
        float scaleX = (float)Screen.width / _lastFrameWidth;
        float scaleY = (float)Screen.height / _lastFrameHeight;

        // 转换为屏幕坐标（适配Unity Y轴方向）
        float x = (float)result.Rect.X * scaleX;
        float y = Screen.height - (float)(result.Rect.Y + result.Rect.Height) * scaleY;
        float width = (float)result.Rect.Width * scaleX;
        float height = (float)result.Rect.Height * scaleY;

        // 限制坐标在屏幕内
        x = Mathf.Clamp(x, 0, Screen.width - width);
        y = Mathf.Clamp(y, 0, Screen.height - height);

        // 绘制检测框
        _boxStyle.normal.textColor = boxColor;
        GUI.Box(new UnityRect(x, y, width, height), "", _boxStyle);

        // 绘制标签（避免超出屏幕顶部）
        float labelY = Mathf.Max(y - 25, 0);
        string labelText = $"{result.ClassName} {result.Confidence:F2}";
        GUI.Label(new UnityRect(x, labelY, width, 25), labelText, _labelStyle);
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
        Texture2D tex = new Texture2D(width, height, TextureFormat.ARGB32, false);
        tex.SetPixels(pixels);
        tex.Apply();
        return tex;
    }

    /// <summary>
    /// 释放资源（避免内存泄漏）
    /// </summary>
    private void OnDestroy()
    {
        // 释放YOLO引擎
        _yoloEngine?.Dispose();

        // 停止USB摄像头
        if (_webCamTexture != null && _webCamTexture.isPlaying)
        {
            _webCamTexture.Stop();
            Destroy(_webCamTexture);
        }

        // 释放场景相机资源
        if (sceneCamera != null)
        {
            sceneCamera.targetTexture = null;
        }
        Destroy(_tempRenderTexture);
        Destroy(_sceneCamTexture);

        // 释放OpenCV资源
        _frameMat?.Release();

        // 清理主线程调度器
        UnityMainThreadDispatcher.Cleanup();

        Debug.Log("🔌 检测资源已成功释放");
    }

    /// <summary>
    /// 编辑器模式下更新GUI样式（实时预览配置变化）
    /// </summary>
    private void OnValidate()
    {
        if (Application.isPlaying && _boxStyle != null && _labelStyle != null)
        {
            _boxStyle.border = new RectOffset(boxLineWidth, boxLineWidth, boxLineWidth, boxLineWidth);
            _labelStyle.normal.background = MakeTex(1, 1, labelColor);
            _labelStyle.fontSize = labelFontSize;
        }
    }
}

// 辅助类：主线程调度器（修复静态类相关错误）
public static class UnityMainThreadDispatcher
{
    // 静态队列存储需要在主线程执行的操作
    private static readonly Queue<Action> _actions = new Queue<Action>();
    private static GameObject _dispatcherObj;
    private static DispatcherBehaviour _dispatcher;
    private static readonly object _lock = new object();

    /// <summary>
    /// 初始化调度器
    /// </summary>
    public static void Init()
    {
        if (_dispatcherObj == null)
        {
            lock (_lock)
            {
                if (_dispatcherObj == null)
                {
                    _dispatcherObj = new GameObject("UnityMainThreadDispatcher");
                    _dispatcher = _dispatcherObj.AddComponent<DispatcherBehaviour>();
                    UnityEngine.Object.DontDestroyOnLoad(_dispatcherObj);
                }
            }
        }
    }

    /// 异步入队主线程执行的操作
    /// </summary>
    public static Task EnqueueAsync(Action action)
    {
        var tcs = new TaskCompletionSource<bool>();

        Enqueue(() =>
        {
            try
            {
                action();
                tcs.SetResult(true);
            }
            catch (Exception ex)
            {
                tcs.SetException(ex);
            }
        });

        return tcs.Task;
    }

    /// <summary>
    /// 入队主线程执行的操作
    /// </summary>
    public static void Enqueue(Action action)
    {
        if (action == null) return;

        lock (_lock)
        {
            _actions.Enqueue(action);
        }
    }

    /// <summary>
    /// 清理调度器资源
    /// </summary>
    public static void Cleanup()
    {
        lock (_lock)
        {
            if (_dispatcherObj != null)
            {
                UnityEngine.Object.Destroy(_dispatcherObj);
                _dispatcherObj = null;
                _dispatcher = null;
            }
            _actions.Clear();
        }
    }

    /// <summary>
    /// 调度器行为类，负责在主线程执行队列中的操作
    /// </summary>
    private class DispatcherBehaviour : MonoBehaviour
    {
        private void Update()
        {
            lock (_lock)
            {
                while (_actions.Count > 0)
                {
                    try
                    {
                        _actions.Dequeue().Invoke();
                    }
                    catch (Exception e)
                    {
                        Debug.LogError($"主线程调度器执行失败：{e.Message}");
                    }
                }
            }
        }
    }
}