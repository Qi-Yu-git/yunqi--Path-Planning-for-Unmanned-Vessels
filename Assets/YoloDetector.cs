using UnityEngine;
using OpenCvSharp;
using System;
using System.Collections.Generic;
using System.IO;
using System.Threading.Tasks;
using System.Threading;
using UnityRect = UnityEngine.Rect;

public class YoloDetector : MonoBehaviour
{
    [Header("模型核心配置")]
    public string modelPath = "yolov8n.onnx";
    public float confidenceThreshold = 0.5f;
    public float iouThreshold = 0.4f;

    [Header("数据源配置")]
    public bool useSceneCamera = true;
    public Camera sceneCamera;
    public Vector2 webCamResolution = new(1280, 720);

    [Header("显示与性能配置")]
    public bool logDetectionResults = true;
    public bool drawBoundingBoxes = true;
    public Color boxColor = Color.red;
    public Color labelColor = Color.green;
    public float detectInterval = 0.1f;
    public int boxLineWidth = 2;
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
    private readonly object _resultLock = new object();
    private List<YoloResult> _detectionResults = new List<YoloResult>();
    private int _lastFrameWidth;
    private int _lastFrameHeight;

    // 公开线程安全的检测结果
    public List<YoloResult> DetectedResults
    {
        get
        {
            lock (_resultLock)
            {
                return new List<YoloResult>(_detectionResults);
            }
        }
        private set
        {
            lock (_resultLock)
            {
                _detectionResults = value ?? new List<YoloResult>();
            }
        }
    }

    void Start()
    {
        try
        {
            InitGUIStyles();
            InitDataSource();
            InitYoloEngine();
            UnityMainThreadDispatcher.Init();
        }
        catch (Exception e)
        {
            Debug.LogError($"初始化失败：{e.Message}\n{e.StackTrace}");
        }
    }

    void Update()
    {
        if (Time.time - _lastDetectTime < detectInterval) return;
        _lastDetectTime = Time.time;

        if (_yoloEngine == null || !_yoloEngine.IsInitialized)
        {
            Debug.LogWarning("YOLO引擎未初始化，跳过检测");
            return;
        }

        _ = ProcessDetectionAsync();
    }

    private async Task ProcessDetectionAsync()
    {
        try
        {
            List<YoloResult> results = new();
            int frameWidth = 0;
            int frameHeight = 0;

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

            // 核心修复：同步帧尺寸+检测结果
            lock (_resultLock)
            {
                _detectionResults = new List<YoloResult>(results);
                _lastFrameWidth = frameWidth;
                _lastFrameHeight = frameHeight;
                DetectedResults = new List<YoloResult>(results);
                Debug.Log($"[Yolo] 检测结果更新：{results.Count}个目标，绘制开关：{drawBoundingBoxes}，帧尺寸：{frameWidth}x{frameHeight}");
            }

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

    private void InitYoloEngine()
    {
        try
        {
            if (!Directory.Exists(Application.streamingAssetsPath))
            {
                Directory.CreateDirectory(Application.streamingAssetsPath);
                Debug.LogWarning("已自动创建StreamingAssets目录，请将模型文件放入该目录");
            }

            string fullModelPath = Path.Combine(Application.streamingAssetsPath, modelPath);
            if (!File.Exists(fullModelPath))
            {
                Debug.LogError($"模型文件不存在：{fullModelPath}");
                return;
            }

            _yoloEngine = new YoloV8Engine(
                fullModelPath,
                confidenceThreshold: confidenceThreshold,
                iouThreshold: iouThreshold,
                logModelProcessing: true,
                logNmsResults: false,
                aggregateLogInterval: 10f
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
            Debug.LogError($"❌ 缺少OpenCvSharp依赖库：{e.Message}");
        }
        catch (Exception e)
        {
            Debug.LogError($"❌ 引擎初始化异常：{e.Message}\n{e.StackTrace}");
        }
    }

    // 添加在YoloDetector.cs的类中（建议放在私有方法区域，如InitDataSource之后）
    /// <summary>
    /// 将YOLO检测的图像坐标转换为场景世界坐标（适配X-Y平面）
    /// </summary>
    public Vector3 ConvertYoloToWorldPosition(Rect2d rect)
    {
        if (sceneCamera == null)
        {
            Debug.LogError("[YoloDetector] 场景相机未初始化，无法转换坐标");
            return Vector3.zero;
        }

        // 计算图像中心点（YOLO的Rect坐标）
        float imgCenterX = (float)(rect.X + rect.Width / 2);
        float imgCenterY = (float)(rect.Y + rect.Height / 2);

        // 图像坐标转屏幕坐标（适配相机分辨率）
        float screenX = Mathf.Clamp(imgCenterX, 0, sceneCamera.pixelWidth);
        float screenY = Mathf.Clamp(imgCenterY, 0, sceneCamera.pixelHeight);

        // 屏幕坐标转射线（Z轴在你的场景中是高度，这里固定高度为0）
        Ray ray = sceneCamera.ScreenPointToRay(new Vector3(screenX, screenY, 0));

        // 假设地面在Z=0平面（根据你的场景调整）
        Plane groundPlane = new Plane(Vector3.forward, 0); // Z轴朝前作为高度轴
        if (groundPlane.Raycast(ray, out float distance))
        {
            Vector3 worldPos = ray.GetPoint(distance);
            // 修正：在你的坐标系中，Y轴是前后方向，Z轴固定为高度
            return new Vector3(worldPos.x, worldPos.y, 0); // 忽略Z轴（高度）
        }

        Debug.LogWarning("[YoloDetector] 坐标转换失败");
        return Vector3.zero;
    }

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

    private void InitSceneCamera()
    {
        if (sceneCamera == null)
        {
            sceneCamera = GameObject.Find("DetectionCamera")?.GetComponent<Camera>();
            if (sceneCamera == null)
            {
                sceneCamera = Camera.main;
                Debug.LogWarning("未指定场景相机，自动使用主相机");
            }
        }

        _tempRenderTexture = new RenderTexture(
            sceneCamera.pixelWidth,
            sceneCamera.pixelHeight,
            24,
            RenderTextureFormat.ARGB32
        );
        sceneCamera.targetTexture = _tempRenderTexture;

        _sceneCamTexture = new Texture2D(
            _tempRenderTexture.width,
            _tempRenderTexture.height,
            TextureFormat.RGBA32,
            false
        );

        Debug.Log($"✅ 场景相机初始化完成：分辨率({_tempRenderTexture.width}x{_tempRenderTexture.height})");
    }

    private void InitWebCamera()
    {
        WebCamDevice[] devices = WebCamTexture.devices;
        if (devices.Length == 0)
        {
            Debug.LogError("❌ 未检测到可用的USB摄像头");
            return;
        }

        _webCamTexture = new WebCamTexture(
            devices[0].name,
            (int)webCamResolution.x,
            (int)webCamResolution.y,
            30
        );

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

                    RenderTexture.active = _tempRenderTexture;
                    _sceneCamTexture.ReadPixels(
                        new UnityRect(0, 0, _tempRenderTexture.width, _tempRenderTexture.height),
                        0, 0
                    );
                    _sceneCamTexture.Apply();
                    RenderTexture.active = null;

                    frameWidth = _tempRenderTexture.width;
                    frameHeight = _tempRenderTexture.height;
                    imageBytes = _sceneCamTexture.EncodeToPNG();
                    tcs.SetResult(true);
                }
            }
            catch (Exception ex)
            {
                tcs.SetException(ex);
            }
        });

        await tcs.Task;

        return await Task.Run(() =>
        {
            if (imageBytes == null) return (null, 0, 0);
            Mat mat = Texture2DToMat(imageBytes);
            return (mat, frameWidth, frameHeight);
        });
    }

    private (Mat, int, int) CaptureWebCameraFrame()
    {
        if (_webCamTexture == null || !_webCamTexture.isPlaying) return (null, 0, 0);

        Texture2D tempTex = new Texture2D(
            _webCamTexture.width,
            _webCamTexture.height,
            TextureFormat.RGBA32,
            false
        );
        tempTex.SetPixels(_webCamTexture.GetPixels());
        tempTex.Apply();

        byte[] bytes = tempTex.EncodeToPNG();
        Destroy(tempTex);

        Mat mat = Texture2DToMat(bytes);
        Cv2.Flip(mat, mat, FlipMode.Y);
        return (mat, _webCamTexture.width, _webCamTexture.height);
    }

    private Mat Texture2DToMat(byte[] imageBytes)
    {
        Mat mat = Cv2.ImDecode(imageBytes, ImreadModes.Color);
        Cv2.CvtColor(mat, mat, ColorConversionCodes.BGR2RGB);
        return mat;
    }

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
            Debug.Log($"  - 类别：{result.ClassName} | 置信度：{result.Confidence:F2} | 边界框：({result.Rect.X:F1}, {result.Rect.Y:F1}, {result.Rect.Width:F1}, {result.Rect.Height:F1})");
        }
    }

    private void InitGUIStyles()
    {
        // 检测框样式（修复线宽不生效问题）
        _boxStyle = new GUIStyle
        {
            normal = { background = MakeTex(1, 1, new Color(0, 0, 0, 0)) },
            border = new RectOffset(boxLineWidth, boxLineWidth, boxLineWidth, boxLineWidth),
            stretchWidth = true,
            stretchHeight = true
        };

        // 标签样式
        _labelStyle = new GUIStyle
        {
            normal = { background = MakeTex(1, 1, labelColor), textColor = Color.white },
            padding = new RectOffset(8, 8, 2, 2),
            fontSize = labelFontSize,
            alignment = TextAnchor.MiddleCenter,
            wordWrap = false
        };
    }

    // 核心修复：OnGUI绘制逻辑（修正缩放+边界）
    private void OnGUI()
    {
        if (!drawBoundingBoxes || _boxStyle == null || _labelStyle == null)
        {
            Debug.LogWarning($"[Yolo] 绘制跳过：drawBoundingBoxes={drawBoundingBoxes}，样式={(_boxStyle == null ? "空" : "正常")}");
            return;
        }

        var results = DetectedResults;
        if (results == null || results.Count == 0)
        {
            Debug.Log("[Yolo] 无检测结果可绘制");
            return;
        }

        // 修复：强制同步帧尺寸（避免0值）
        if (_lastFrameWidth == 0 || _lastFrameHeight == 0)
        {
            _lastFrameWidth = sceneCamera != null ? sceneCamera.pixelWidth : Screen.width;
            _lastFrameHeight = sceneCamera != null ? sceneCamera.pixelHeight : Screen.height;
        }

        Debug.Log($"[Yolo] 开始绘制{results.Count}个检测框，帧尺寸：{_lastFrameWidth}x{_lastFrameHeight}，屏幕尺寸：{Screen.width}x{Screen.height}");

        foreach (var result in results)
        {
            DrawSingleBoundingBox(result);
        }
    }

    // 修复：红框绘制（适配任意分辨率）
    private void DrawSingleBoundingBox(YoloResult result)
    {
        if (sceneCamera == null) return;

        // 1. YOLO归一化坐标（0-1范围）转视口坐标
        float viewportX = (float)(result.Rect.X + result.Rect.Width / 2);
        float viewportY = 1 - (float)(result.Rect.Y + result.Rect.Height / 2); // 翻转Y轴（YOLO原点在左上角，Unity在左下角）

        // 2. 视口坐标转屏幕坐标（适配检测相机）
        Vector3 screenPos = sceneCamera.ViewportToScreenPoint(new Vector3(viewportX, viewportY, 0));

        // 3. 计算屏幕空间的宽高
        float screenWidth = sceneCamera.pixelWidth;
        float screenHeight = sceneCamera.pixelHeight;
        float boxWidth = (float)result.Rect.Width * screenWidth;
        float boxHeight = (float)result.Rect.Height * screenHeight;

        // 4. 边界修正（防止超出屏幕）
        float x = Mathf.Clamp(screenPos.x - boxWidth / 2, 0, screenWidth - boxWidth);
        float y = Mathf.Clamp(screenPos.y - boxHeight / 2, 0, screenHeight - boxHeight);

        Debug.Log($"[Yolo] 绘制目标：{result.ClassName}，屏幕坐标：({x:F1},{y:F1}) 尺寸：{boxWidth:F1}x{boxHeight:F1}");

        // 5. 绘制红框和标签
        _boxStyle.normal.textColor = boxColor;
        GUI.Box(new UnityRect(x, y, boxWidth, boxHeight), "", _boxStyle);

        float labelY = Mathf.Max(y - 25, 0);
        string labelText = $"{result.ClassName} {result.Confidence:F2}";
        GUI.Label(new UnityRect(x, labelY, boxWidth, 25), labelText, _labelStyle);
    }

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

    private void OnDestroy()
    {
        _yoloEngine?.Dispose();

        if (_webCamTexture != null && _webCamTexture.isPlaying)
        {
            _webCamTexture.Stop();
            Destroy(_webCamTexture);
        }

        if (sceneCamera != null)
        {
            sceneCamera.targetTexture = null;
        }

        Destroy(_tempRenderTexture);
        Destroy(_sceneCamTexture);
        _frameMat?.Release();
        UnityMainThreadDispatcher.Cleanup();

        Debug.Log("🔌 检测资源已成功释放");
    }

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

// 主线程调度器（完整实现）
public static class UnityMainThreadDispatcher
{
    private static readonly Queue<Action> _actions = new Queue<Action>();
    private static GameObject _dispatcherObj;
    private static DispatcherBehaviour _dispatcher;
    private static readonly object _lock = new object();

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

    public static void Enqueue(Action action)
    {
        if (action == null) return;
        lock (_lock)
        {
            _actions.Enqueue(action);
        }
    }

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