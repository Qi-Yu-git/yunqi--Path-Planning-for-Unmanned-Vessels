using UnityEngine;

[RequireComponent(typeof(Camera))]
public class FixDetectionCamera : MonoBehaviour
{
    [Tooltip("检测相机需要渲染的图层名称（用逗号分隔，如Default,水域,陆地,暗礁,USV,Obstacle）")]
    [SerializeField] private string cullingLayerNames = "Default,水域,陆地,暗礁,USV,Obstacle";
    [Tooltip("检测相机的深度值（应高于主相机）")]
    [SerializeField] private int cameraDepth = 1;
    [Tooltip("检测相机视野范围（调大以覆盖更多区域）")]
    [SerializeField] private float fieldOfView = 120f;

    private Camera _detectCam;
    private LayerMask _cullingLayers;

    void Awake()
    {
        _detectCam = GetComponent<Camera>();
        // 初始化图层（修复NameToLayer调用时机问题）
        InitializeCullingLayers();
        // 初始化相机核心配置（放弃RenderTexture，直接屏幕显示）
        InitializeDetectionCamera();
    }

    /// <summary>
    /// 初始化渲染图层（关键：在Awake中执行，避免构造函数调用NameToLayer）
    /// </summary>
    private void InitializeCullingLayers()
    {
        // 拆分图层名并创建LayerMask
        string[] layerNames = cullingLayerNames.Split(',');
        _cullingLayers = 0;
        foreach (string layerName in layerNames)
        {
            string trimedName = layerName.Trim(); // 去除空格
            int layerIndex = LayerMask.NameToLayer(trimedName);
            if (layerIndex != -1)
            {
                _cullingLayers |= (1 << layerIndex);
            }
            else
            {
                Debug.LogWarning($"⚠️ 图层 {trimedName} 不存在，请检查拼写！");
            }
        }

        // 兜底：若图层配置全错，默认渲染Default层
        if (_cullingLayers == 0)
        {
            _cullingLayers = LayerMask.GetMask("Default");
            Debug.LogWarning("⚠️ 所有图层配置错误，默认渲染Default层");
        }
    }

    /// <summary>
    /// 初始化相机（放弃RenderTexture，直接屏幕显示）
    /// </summary>
    private void InitializeDetectionCamera()
    {
        if (_detectCam == null)
        {
            Debug.LogError("❌ 缺少Camera组件，无法初始化检测相机");
            return;
        }

        // 1. 移除冲突的AudioListener
        RemoveAudioListener();

        // 2. 核心配置：让相机正常显示到屏幕（Display 0）
        _detectCam.targetTexture = null; // 清空RenderTexture，改用屏幕显示
        _detectCam.targetDisplay = 1;   
        _detectCam.rect = new Rect(0, 0, 1, 1); // 全屏显示（也可设小窗口：new Rect(0,0,0.3f,0.3f)）

        // 3. 基础渲染配置
        _detectCam.enabled = true;
        _detectCam.cullingMask = _cullingLayers; // 渲染目标图层
        _detectCam.clearFlags = CameraClearFlags.Skybox;
        _detectCam.depth = cameraDepth;          // 高于主相机（主相机一般为0）
        _detectCam.fieldOfView = fieldOfView;    // 调大视野，覆盖更多场景
        _detectCam.nearClipPlane = 0.1f;         // 近裁剪面，避免近距离物体消失
        _detectCam.farClipPlane = 100f;          // 远裁剪面，覆盖场景所有物体

        Debug.Log($"✅ DetectionCamera 初始化完成：\n" +
                  $"→ 渲染图层：{cullingLayerNames}\n" +
                  $"→ 输出到 Display 0（屏幕）\n" +
                  $"→ 视野范围：{fieldOfView}°");
    }

    /// <summary>
    /// 移除多余的AudioListener，避免与主相机冲突
    /// </summary>
    private void RemoveAudioListener()
    {
        AudioListener listener = GetComponent<AudioListener>();
        if (listener != null)
        {
            Destroy(listener);
            Debug.Log("ℹ️ 已移除检测相机上的AudioListener，避免冲突");
        }
    }

    /// <summary>
    /// 供YoloDetector调用：获取当前检测相机
    /// </summary>
    /// <returns>配置好的检测相机</returns>
    public Camera GetDetectionCamera()
    {
        return _detectCam;
    }

    // 编辑器模式下实时更新配置
    void OnValidate()
    {
        if (_detectCam == null)
            _detectCam = GetComponent<Camera>();

        if (_detectCam != null)
        {
            _detectCam.depth = cameraDepth;
            _detectCam.fieldOfView = fieldOfView;
        }
    }
}