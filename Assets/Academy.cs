using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using System.Collections;

/// <summary>
/// USV强化学习环境的核心管理器
/// 负责协调环境初始化、参数管理和重置逻辑
/// </summary>
public class USV_Academy : MonoBehaviour
{
    #region 单例实例
    // 移除new关键字，解决CS0109警告
    public static USV_Academy Instance { get; private set; }
    #endregion

    #region 序列化参数
    [Header("环境配置参数")]
    [Tooltip("最小岩石数量")]
    [Min(1)] public int minRockCount = 5;

    [Tooltip("最大岩石数量")]
    [Min(1)] public int maxRockCount = 15;

    [Tooltip("USV最大速度")]
    [Range(1f, 5f)] public float maxUSVSpeed = 2.0f;

    [Tooltip("最大回合时长(秒)")]
    [Min(10f)] public float maxEpisodeTime = 60f;
    #endregion

    #region 私有变量
    // 场景核心管理器引用
    private RandomSpawnManager spawnManager;
    private USV_GlobalRLAgent usvAgent;
    private GridManager gridManager;

    // 环境参数实例
    private EnvironmentParameters envParams;

    // 依赖项加载状态
    private bool areDependenciesLoaded = false;
    #endregion

    #region 生命周期方法
    private void Awake()
    {
        // 实现单例模式
        if (Instance == null)
        {
            Instance = this;
        }
        else
        {
            Destroy(gameObject);
        }
    }

    private void Start()
    {
        // 替代原来的Initialize方法
        StartCoroutine(WaitForDependencies());
    }

    // 重命名方法，不再尝试重写不存在的方法
    public void EnvironmentReset()
    {
        if (!areDependenciesLoaded)
        {
            Debug.LogWarning("环境依赖未加载完成，无法重置环境");
            return;
        }

        ResetEnvironment();
    }
    #endregion

    #region 依赖管理
    /// <summary>
    /// 等待所有必要的依赖项加载完成
    /// </summary>
    private IEnumerator WaitForDependencies()
    {
        // 等待栅格管理器初始化 - 使用新的API替代过时方法
        while (gridManager == null)
        {
            gridManager = Object.FindFirstObjectByType<GridManager>();
            if (gridManager == null)
            {
                Debug.LogWarning("等待GridManager加载...");
                yield return new WaitForSeconds(0.1f);
            }
        }

        // 等待栅格数据准备完成
        while (!gridManager.IsGridReady())
        {
            Debug.LogWarning("等待GridManager准备栅格数据...");
            yield return new WaitForSeconds(0.1f);
        }

        // 获取其他管理器引用 - 使用新的API替代过时方法
        spawnManager = Object.FindFirstObjectByType<RandomSpawnManager>();
        usvAgent = Object.FindFirstObjectByType<USV_GlobalRLAgent>();

        // 验证所有必要的引用
        if (ValidateDependencies())
        {
            areDependenciesLoaded = true;
            envParams = Academy.Instance.EnvironmentParameters;
            RegisterEnvironmentParameters();
            Debug.Log("所有环境依赖项加载完成");
        }
        else
        {
            Debug.LogError("环境依赖项加载失败，部分组件缺失");
        }
    }

    /// <summary>
    /// 验证所有必要的依赖项是否存在
    /// </summary>
    /// <returns>如果所有依赖都存在则返回true，否则返回false</returns>
    private bool ValidateDependencies()
    {
        bool isValid = true;

        if (spawnManager == null)
        {
            Debug.LogError("USV_Academy: 未找到RandomSpawnManager");
            isValid = false;
        }

        if (usvAgent == null)
        {
            Debug.LogError("USV_Academy: 未找到USV_GlobalRLAgent");
            isValid = false;
        }

        return isValid;
    }
    #endregion

    #region 参数管理
    /// <summary>
    /// 注册环境参数回调，允许从外部(如训练配置)调整环境参数
    /// </summary>
    private void RegisterEnvironmentParameters()
    {
        if (envParams == null)
        {
            Debug.LogError("EnvironmentParameters实例为空，无法注册回调");
            return;
        }

        // 岩石数量范围参数
        envParams.RegisterCallback("rock_count_min", value =>
        {
            minRockCount = Mathf.Max(1, Mathf.RoundToInt(value));
            Debug.Log($"更新最小岩石数量: {minRockCount}");
        });

        envParams.RegisterCallback("rock_count_max", value =>
        {
            maxRockCount = Mathf.Max(minRockCount, Mathf.RoundToInt(value));
            Debug.Log($"更新最大岩石数量: {maxRockCount}");
        });

        // USV最大速度参数
        envParams.RegisterCallback("max_usv_speed", value =>
        {
            maxUSVSpeed = Mathf.Clamp(value, 1f, 5f);
            if (usvAgent != null)
            {
                usvAgent.ResetAgentState(maxUSVSpeed, maxEpisodeTime);
            }
            Debug.Log($"更新最大速度: {maxUSVSpeed}");
        });
    }
    #endregion

    #region 环境控制
    /// <summary>
    /// 重置环境到初始状态
    /// </summary>
    public void ResetEnvironment()
    {
        if (!gridManager.IsGridReady())
        {
            Debug.LogWarning("栅格未准备就绪，执行强制刷新");
            gridManager.强制刷新栅格();
            return;
        }

        if (spawnManager != null)
        {
            spawnManager.SetRockCountRange(minRockCount, maxRockCount);
            spawnManager.Regenerate();
        }

        if (usvAgent != null)
        {
            usvAgent.OnEpisodeBegin();
            usvAgent.ResetAgentState(maxUSVSpeed, maxEpisodeTime);
        }

        Debug.Log($"环境已重置 - 岩石数量范围: {minRockCount}-{maxRockCount}, 最大速度: {maxUSVSpeed}");
    }
    #endregion

    #region 公共方法
    /// <summary>
    /// 获取当前设置的最大速度
    /// </summary>
    /// <returns>最大速度值</returns>
    public float GetCurrentMaxSpeed() => maxUSVSpeed;

    /// <summary>
    /// 获取最大回合时长
    /// </summary>
    /// <returns>最大回合时长(秒)</returns>
    public float GetMaxEpisodeTime() => maxEpisodeTime;

    /// <summary>
    /// 检查环境是否已准备就绪
    /// </summary>
    /// <returns>如果环境准备就绪则返回true</returns>
    public bool IsEnvironmentReady() => areDependenciesLoaded && gridManager.IsGridReady();
    #endregion
}