using UnityEngine;

[RequireComponent(typeof(Camera))]
public class FixDetectionCamera : MonoBehaviour
{
    [Tooltip("检测相机输出的分辨率宽度")]
    [SerializeField] private int renderWidth = 1280;
    [Tooltip("检测相机输出的分辨率高度")]
    [SerializeField] private int renderHeight = 720;
    [Tooltip("检测相机需要渲染的图层名称（用逗号分隔，如Default,水域,陆地）")]
    [SerializeField] private string cullingLayerNames = "Default,水域,陆地"; // 改为字符串配置
    [Tooltip("检测相机的深度值（应高于主相机）")]
    [SerializeField] private int cameraDepth = 1;

    private Camera _detectCam;
    private RenderTexture _targetRenderTexture;
    private LayerMask _cullingLayers; // 延迟初始化

    void Awake()
    {
        _detectCam = GetComponent<Camera>();
        // 关键修复：在Awake中初始化图层，避免字段初始化阶段调用NameToLayer
        InitializeCullingLayers();
        InitializeDetectionCamera();
    }

    // 在 FixDetectionCamera 类中新增 OnGUI 方法，Game 视图显示采集画面
    void OnGUI()
    {
        // 仅在运行时显示
        if (!Application.isPlaying) return;

        // 检查 RenderTexture 是否有效
        if (_targetRenderTexture != null && _targetRenderTexture.IsCreated())
        {
            // 在 Game 视图左上角绘制 300x200 的小窗口，显示相机采集的画面
            GUI.DrawTexture(new Rect(10, 10, 300, 200), _targetRenderTexture);
            // 标注文字，方便识别
            GUI.Label(new Rect(10, 220, 200, 20), "DetectionCamera 采集画面");
        }
        else
        {
            GUI.Label(new Rect(10, 10, 200, 20), "❌ RenderTexture 未创建成功");
        }
    }

    // 新增：单独初始化图层
    private void InitializeCullingLayers()
    {
        _cullingLayers = LayerMask.GetMask(cullingLayerNames.Split(','));
        // 兜底：若图层配置错误，默认渲染Default层
        if (_cullingLayers == 0)
        {
            _cullingLayers = LayerMask.GetMask("Default");
            Debug.LogWarning("⚠️ 图层配置错误，默认渲染Default层");
        }
    }

    private void InitializeDetectionCamera()
    {
        if (_detectCam == null)
        {
            Debug.LogError("❌ 缺少Camera组件，无法初始化检测相机");
            return;
        }

        // 兜底方案：放弃RenderTexture，直接修复Display配置
        _detectCam.targetTexture = null;
        _detectCam.targetDisplay = 0; // 强制输出到Display 0
        _detectCam.rect = new Rect(0, 0, 1, 1); // 全屏显示

        RemoveAudioListener();
        _detectCam.enabled = true;
        _detectCam.cullingMask = _cullingLayers;
        _detectCam.clearFlags = CameraClearFlags.Skybox;
        _detectCam.depth = 1;

        Debug.Log("✅ DetectionCamera 兜底初始化完成：输出到Display 0");
    }

    // 以下方法（CreateRenderTexture/RemoveAudioListener/GetRenderTexture/OnDestroy/OnValidate）保持不变
    private void CreateRenderTexture()
    {
        if (_targetRenderTexture != null)
        {
            Destroy(_targetRenderTexture);
        }

        _targetRenderTexture = new RenderTexture(renderWidth, renderHeight, 24, RenderTextureFormat.Default);
        if (_targetRenderTexture.IsCreated())
        {
            _detectCam.targetTexture = _targetRenderTexture;
        }
        else
        {
            Debug.LogError("❌ 无法创建RenderTexture，检测相机初始化失败");
        }
    }

    private void RemoveAudioListener()
    {
        AudioListener listener = GetComponent<AudioListener>();
        if (listener != null)
        {
            Destroy(listener);
            Debug.Log("ℹ️ 已移除检测相机上的AudioListener");
        }
    }

    public RenderTexture GetRenderTexture()
    {
        return _targetRenderTexture;
    }

    void OnDestroy()
    {
        if (_targetRenderTexture != null)
        {
            Destroy(_targetRenderTexture);
            _targetRenderTexture = null;
        }
    }

    void OnValidate()
    {
        if (_detectCam == null)
            _detectCam = GetComponent<Camera>();

        if (_detectCam != null)
        {
            _detectCam.depth = cameraDepth;
        }
    }
}