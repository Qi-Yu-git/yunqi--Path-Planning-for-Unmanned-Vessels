using UnityEngine;
using OpenCvSharp;
using System;
using System.Collections.Generic;
using System.IO;
using System.Threading.Tasks;
using System.Threading;
using UnityRect = UnityEngine.Rect;
using YoloV8Detection;

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
    public bool drawBoundingBoxes = true;
    public Color boxColor = Color.red;
    public Color labelColor = Color.green;
    public float detectInterval = 0.2f;
    public int boxLineWidth = 2;
    public int labelFontSize = 12;

    [Header("CLAHE 增强配置")]
    [Tooltip("是否启用 CLAHE 图像增强")]
    public bool enableCLAHE = true;

    private CLAHEPreprocessor claheProcessor;

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
    private List<YoloV8Detection.YoloResult> _detectionResults = new List<YoloV8Detection.YoloResult>();
    private int _lastFrameWidth;
    private int _lastFrameHeight;

    public List<YoloV8Detection.YoloResult> DetectedResults
    {
        get
        {
            lock (_resultLock)
            {
                return new List<YoloV8Detection.YoloResult>(_detectionResults);
            }
        }
        private set
        {
            lock (_resultLock)
            {
                _detectionResults = value == null ? new List<YoloV8Detection.YoloResult>() : value;
            }
        }
    }

    void Start()
    {
        try
        {
            InitCLAHE();
            InitGUIStyles();
            InitDataSource();
            InitYoloEngine();
            UnityMainThreadDispatcher.Init();
        }
        catch (Exception e)
        {
            YoloV8Detection.YoloLogSettings.Log(
                YoloV8Detection.YoloLogSettings.LogModule.YoloDetector,
                YoloV8Detection.YoloLogSettings.LogLevel.Error,
                $"初始化失败：{e.Message}\n{e.StackTrace}"
            );
        }
    }

    private void InitCLAHE()
    {
        claheProcessor = GetComponent<CLAHEPreprocessor>();
        if (claheProcessor == null)
        {
            claheProcessor = gameObject.AddComponent<CLAHEPreprocessor>();
        }
        Debug.Log("[YoloDetector] CLAHE 预处理器已初始化");
    }

    void Update()
    {
        if (Time.time - _lastDetectTime < detectInterval) return;
        _lastDetectTime = Time.time;

        if (_yoloEngine == null || !_yoloEngine.IsInitialized)
        {
            YoloV8Detection.YoloLogSettings.Log(
                YoloV8Detection.YoloLogSettings.LogModule.YoloDetector,
                YoloV8Detection.YoloLogSettings.LogLevel.Warn,
                "YOLO引擎未初始化，跳过检测"
            );
            return;
        }

        _ = ProcessDetectionAsync();
    }

    private async Task ProcessDetectionAsync()
    {
        try
        {
            List<YoloV8Detection.YoloResult> results = new List<YoloV8Detection.YoloResult>();
            int frameWidth = 0;
            int frameHeight = 0;

            if (useSceneCamera && sceneCamera != null)
            {
                (Mat sceneMat, int w, int h) = await CaptureSceneCameraFrameAsync();
                if (sceneMat != null && !sceneMat.Empty())
                {
                    // ====== 1. 先应用 CLAHE 增强 ======
                    if (enableCLAHE && claheProcessor != null)
                    {
                        sceneMat = claheProcessor.ApplyCLAHE(sceneMat);
                    }

                    // ====== 2. 再执行检测 ======
                    results = new List<YoloV8Detection.YoloResult>(_yoloEngine.Detect(sceneMat));
                    frameWidth = w;
                    frameHeight = h;

                    // ====== 3. 最后释放 ======
                    sceneMat.Release();
                }
            }
            else if (!useSceneCamera && _webCamTexture != null && _webCamTexture.isPlaying)
            {
                (Mat webMat, int w, int h) = CaptureWebCameraFrame();
                if (webMat != null && !webMat.Empty())
                {
                    // ====== 1. 先应用 CLAHE 增强 ======
                    if (enableCLAHE && claheProcessor != null)
                    {
                        webMat = claheProcessor.ApplyCLAHE(webMat);
                    }

                    // ====== 2. 再执行检测 ======
                    results = new List<YoloV8Detection.YoloResult>(_yoloEngine.Detect(webMat));
                    frameWidth = w;
                    frameHeight = h;

                    // ====== 3. 最后释放 ======
                    webMat.Release();
                }
            }

            lock (_resultLock)
            {
                _detectionResults = new List<YoloV8Detection.YoloResult>(results);
                _lastFrameWidth = frameWidth;
                _lastFrameHeight = frameHeight;
                DetectedResults = new List<YoloV8Detection.YoloResult>(results);

                YoloV8Detection.YoloLogSettings.Log(
                    YoloV8Detection.YoloLogSettings.LogModule.YoloDetector,
                    YoloV8Detection.YoloLogSettings.LogLevel.Info,
                    $"[Yolo] 检测结果更新：{results.Count}个目标，帧尺寸：{frameWidth}x{frameHeight}"
                );
            }

            UnityMainThreadDispatcher.Enqueue(() =>
            {
                ProcessDetailedDetectionLogs(results);
            });
        }
        catch (Exception e)
        {
            YoloV8Detection.YoloLogSettings.Log(
                YoloV8Detection.YoloLogSettings.LogModule.YoloDetector,
                YoloV8Detection.YoloLogSettings.LogLevel.Error,
                $"检测异常：{e.Message}\n{e.StackTrace}"
            );
        }
    }

    private void InitYoloEngine()
    {
        try
        {
            string fullModelPath = Path.Combine(Application.dataPath, "Models/yolov8n.onnx");
            fullModelPath = fullModelPath.Replace('/', '\\');

            if (!File.Exists(fullModelPath))
            {
                YoloV8Detection.YoloLogSettings.Log(
                    YoloV8Detection.YoloLogSettings.LogModule.YoloDetector,
                    YoloV8Detection.YoloLogSettings.LogLevel.Error,
                    $"模型文件不存在：{fullModelPath}\n请确认模型已放入Assets/Models目录"
                );
                return;
            }

            YoloV8Detection.YoloLogSettings logSettings = FindAnyObjectByType<YoloV8Detection.YoloLogSettings>();
            if (logSettings == null)
            {
                GameObject logObj = new GameObject("[YoloLogSettings]");
                logSettings = logObj.AddComponent<YoloV8Detection.YoloLogSettings>();
                DontDestroyOnLoad(logObj);
            }

            _yoloEngine = new YoloV8Detection.YoloV8Engine(
                modelPath: fullModelPath,
                confidenceThreshold: confidenceThreshold,
                iouThreshold: iouThreshold,
                logSettings: logSettings,
                useCuda: true
            );
            _yoloEngine.LogModelProcessing = true;
            _yoloEngine.LogNmsResults = false;

            if (_yoloEngine.IsInitialized)
            {
                YoloV8Detection.YoloLogSettings.Log(
                    YoloV8Detection.YoloLogSettings.LogModule.YoloDetector,
                    YoloV8Detection.YoloLogSettings.LogLevel.Info,
                    $"✅ YOLO引擎初始化成功！类别数：{_yoloEngine.ClassNames.Count}"
                );
            }
            else
            {
                YoloV8Detection.YoloLogSettings.Log(
                    YoloV8Detection.YoloLogSettings.LogModule.YoloDetector,
                    YoloV8Detection.YoloLogSettings.LogLevel.Error,
                    "❌ YOLO引擎初始化失败"
                );
            }
        }
        catch (DllNotFoundException e)
        {
            YoloV8Detection.YoloLogSettings.Log(
                YoloV8Detection.YoloLogSettings.LogModule.YoloDetector,
                YoloV8Detection.YoloLogSettings.LogLevel.Error,
                $"❌ 缺少OpenCvSharp依赖库：{e.Message}"
            );
        }
        catch (Exception e)
        {
            YoloV8Detection.YoloLogSettings.Log(
                YoloV8Detection.YoloLogSettings.LogModule.YoloDetector,
                YoloV8Detection.YoloLogSettings.LogLevel.Error,
                $"❌ 引擎初始化异常：{e.Message}\n{e.StackTrace}"
            );
        }
    }

    public Vector3 ConvertYoloToWorldPosition(Rect2d rect)
    {
        if (sceneCamera == null)
        {
            YoloV8Detection.YoloLogSettings.Log(
                YoloV8Detection.YoloLogSettings.LogModule.YoloDetector,
                YoloV8Detection.YoloLogSettings.LogLevel.Error,
                "[YoloDetector] 场景相机未初始化，无法转换坐标"
            );
            return Vector3.zero;
        }

        float imgCenterX = (float)(rect.X + rect.Width / 2);
        float imgCenterY = (float)(rect.Y + rect.Height / 2);

        float screenX = Mathf.Clamp(imgCenterX, 0, sceneCamera.pixelWidth);
        float screenY = Mathf.Clamp(imgCenterY, 0, sceneCamera.pixelHeight);

        Ray ray = sceneCamera.ScreenPointToRay(new Vector3(screenX, screenY, 0));
        Plane groundPlane = new Plane(Vector3.up, 0.4f);

        if (groundPlane.Raycast(ray, out float distance))
        {
            Vector3 worldPos = ray.GetPoint(distance);
            return new Vector3(worldPos.x, worldPos.y, 0);
        }

        YoloV8Detection.YoloLogSettings.Log(
            YoloV8Detection.YoloLogSettings.LogModule.YoloDetector,
            YoloV8Detection.YoloLogSettings.LogLevel.Warn,
            "[YoloDetector] 坐标转换失败"
        );
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
                YoloV8Detection.YoloLogSettings.Log(
                    YoloV8Detection.YoloLogSettings.LogModule.YoloDetector,
                    YoloV8Detection.YoloLogSettings.LogLevel.Warn,
                    "未指定场景相机，自动使用主相机"
                );
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

        YoloV8Detection.YoloLogSettings.Log(
            YoloV8Detection.YoloLogSettings.LogModule.YoloDetector,
            YoloV8Detection.YoloLogSettings.LogLevel.Info,
            $"✅ 场景相机初始化完成：分辨率({_tempRenderTexture.width}x{_tempRenderTexture.height})"
        );
    }

    private void InitWebCamera()
    {
        WebCamDevice[] devices = WebCamTexture.devices;
        if (devices.Length == 0)
        {
            YoloV8Detection.YoloLogSettings.Log(
                YoloV8Detection.YoloLogSettings.LogModule.YoloDetector,
                YoloV8Detection.YoloLogSettings.LogLevel.Error,
                "❌ 未检测到可用的USB摄像头"
            );
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
            YoloV8Detection.YoloLogSettings.Log(
                YoloV8Detection.YoloLogSettings.LogModule.YoloDetector,
                YoloV8Detection.YoloLogSettings.LogLevel.Info,
                $"✅ USB摄像头启动成功：{devices[0].name}，分辨率({_webCamTexture.width}x{_webCamTexture.height})"
            );
        }
        else
        {
            YoloV8Detection.YoloLogSettings.Log(
                YoloV8Detection.YoloLogSettings.LogModule.YoloDetector,
                YoloV8Detection.YoloLogSettings.LogLevel.Error,
                "❌ USB摄像头启动失败"
            );
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

    private void ProcessDetailedDetectionLogs(List<YoloV8Detection.YoloResult> results)
    {
        if (results == null || results.Count == 0)
        {
            YoloV8Detection.YoloLogSettings.Log(
                YoloV8Detection.YoloLogSettings.LogModule.YoloDetector,
                YoloV8Detection.YoloLogSettings.LogLevel.Info,
                "📌 未检测到任何目标"
            );
            return;
        }

        YoloV8Detection.YoloLogSettings.Log(
            YoloV8Detection.YoloLogSettings.LogModule.YoloDetector,
            YoloV8Detection.YoloLogSettings.LogLevel.Info,
            $"📌 检测到 {results.Count} 个目标："
        );
        foreach (var result in results)
        {
            YoloV8Detection.YoloLogSettings.Log(
                YoloV8Detection.YoloLogSettings.LogModule.YoloDetector,
                YoloV8Detection.YoloLogSettings.LogLevel.Info,
                $"  - 类别：{result.ClassName} | 置信度：{result.Confidence:F2}",
                result.ClassName
            );
        }
    }

    private void InitGUIStyles()
    {
        _boxStyle = new GUIStyle
        {
            normal = { background = MakeTex(1, 1, new Color(0, 0, 0, 0)) },
            border = new RectOffset(boxLineWidth, boxLineWidth, boxLineWidth, boxLineWidth),
            stretchWidth = true,
            stretchHeight = true
        };

        _labelStyle = new GUIStyle
        {
            normal = { background = MakeTex(1, 1, labelColor), textColor = Color.white },
            padding = new RectOffset(8, 8, 2, 2),
            fontSize = labelFontSize,
            alignment = TextAnchor.MiddleCenter,
            wordWrap = false
        };
    }

    private void OnGUI()
    {
        if (!drawBoundingBoxes || _boxStyle == null || _labelStyle == null) return;

        var results = DetectedResults;
        if (results == null || results.Count == 0) return;

        if (_lastFrameWidth == 0 || _lastFrameHeight == 0)
        {
            _lastFrameWidth = sceneCamera != null ? sceneCamera.pixelWidth : Screen.width;
            _lastFrameHeight = sceneCamera != null ? sceneCamera.pixelHeight : Screen.height;
        }

        foreach (var result in results)
        {
            DrawSingleBoundingBox(result);
        }
    }

    private void DrawSingleBoundingBox(YoloV8Detection.YoloResult result)
    {
        if (sceneCamera == null) return;

        float viewportX = (float)(result.Rect.X + result.Rect.Width / 2);
        float viewportY = 1 - (float)(result.Rect.Y + result.Rect.Height / 2);

        Vector3 screenPos = sceneCamera.ViewportToScreenPoint(new Vector3(viewportX, viewportY, 0));

        float screenWidth = sceneCamera.pixelWidth;
        float screenHeight = sceneCamera.pixelHeight;
        float boxWidth = (float)result.Rect.Width * screenWidth;
        float boxHeight = (float)result.Rect.Height * screenHeight;

        float x = Mathf.Clamp(screenPos.x - boxWidth / 2, 0, screenWidth - boxWidth);
        float y = Mathf.Clamp(screenPos.y - boxHeight / 2, 0, screenHeight - boxHeight);

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

        YoloV8Detection.YoloLogSettings.Log(
            YoloV8Detection.YoloLogSettings.LogModule.YoloDetector,
            YoloV8Detection.YoloLogSettings.LogLevel.Info,
            "🔌 检测资源已成功释放"
        );
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
                YoloV8Detection.YoloLogSettings.Log(
                    YoloV8Detection.YoloLogSettings.LogModule.YoloDetector,
                    YoloV8Detection.YoloLogSettings.LogLevel.Error,
                    $"主线程调度器执行失败：{ex.Message}"
                );
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
                        YoloV8Detection.YoloLogSettings.Log(
                            YoloV8Detection.YoloLogSettings.LogModule.YoloDetector,
                            YoloV8Detection.YoloLogSettings.LogLevel.Error,
                            $"主线程调度器执行失败：{e.Message}"
                        );
                    }
                }
            }
        }
    }
}