using UnityEngine;
using Unity.MLAgents;
using System.Collections;

/// <summary>
/// USV强化学习环境的核心管理器
/// 负责协调环境初始化、参数管理和重置逻辑
/// </summary>
public class USV_Academy : MonoBehaviour
{
    #region 单例实例
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
    private GridManager gridManager;
    private USV_GlobalRLAgent usvAgent; // 重新持有智能体引用
    // 环境参数实例
    private EnvironmentParameters envParams;
    // 依赖项加载状态
    private bool areDependenciesLoaded = false;
    #endregion

    #region 生命周期方法
    private void Awake()
    {
        if (Instance == null)
        {
            Instance = this;
            DontDestroyOnLoad(gameObject);
        }
        else
        {
            Destroy(gameObject);
        }
    }

    private void Start()
    {
        StartCoroutine(WaitForDependencies());
    }

    /// <summary>
    /// 每帧检查回合是否结束，以便自动重置环境
    /// </summary>
    private void Update()
    {
        // 核心修改：只有任务循环开启时，才自动重置环境
        if (areDependenciesLoaded && usvAgent != null && usvAgent.IsEpisodeDone && usvAgent.enableTaskLoop)
        {
            Debug.Log("任务完成，准备重置环境...");
            ResetEnvironment();
        }
    }
    #endregion

    #region 依赖管理
    private IEnumerator WaitForDependencies()
    {
        // 修复2.0.1兼容：FindFirstObjectByType加泛型约束（旧版无，新版需要）
        while (gridManager == null)
        {
            gridManager = Object.FindFirstObjectByType<GridManager>(FindObjectsInactive.Include);
            if (gridManager == null)
            {
                Debug.LogWarning("等待GridManager加载...");
                yield return new WaitForSeconds(0.1f);
            }
        }
        Debug.Log("GridManager 已找到。");

        while (!gridManager.IsGridReady())
        {
            Debug.LogWarning("等待GridManager准备栅格数据...");
            yield return new WaitForSeconds(0.1f);
        }
        Debug.Log("GridManager 数据已就绪。");

        spawnManager = Object.FindFirstObjectByType<RandomSpawnManager>(FindObjectsInactive.Include);
        usvAgent = Object.FindFirstObjectByType<USV_GlobalRLAgent>(FindObjectsInactive.Include);

        if (ValidateDependencies())
        {
            areDependenciesLoaded = true;
            // 关键：ML-Agents 2.0.1通过Academy.Instance获取全局实例
            envParams = Academy.Instance.EnvironmentParameters;
            RegisterEnvironmentParameters();
            Debug.Log("所有环境依赖项加载完成，准备启动第一个回合...");
            ResetEnvironment();
        }
        else
        {
            Debug.LogError("环境依赖项加载失败，部分组件缺失！请检查场景设置。");
        }
    }

    private bool ValidateDependencies()
    {
        bool isValid = true;
        if (spawnManager == null) { Debug.LogError("USV_Academy: 未找到 RandomSpawnManager！"); isValid = false; }
        if (usvAgent == null) { Debug.LogError("USV_Academy: 未找到 USV_GlobalRLAgent 组件！"); isValid = false; }
        return isValid;
    }
    #endregion

    #region 参数管理
    private void RegisterEnvironmentParameters()
    {
        if (envParams == null) { Debug.LogError("EnvironmentParameters 实例为空"); return; }

        // 注册最小岩石数量回调
        envParams.RegisterCallback("rock_count_min", value =>
        {
            int newMin = Mathf.Max(1, Mathf.RoundToInt(value));
            spawnManager?.SetRockCountRange(newMin, spawnManager.currentMaxRockCount);
            Debug.Log($"[环境参数] 最小岩石数量: {newMin}");
        });

        // 注册最大岩石数量回调
        envParams.RegisterCallback("rock_count_max", value =>
        {
            int newMax = Mathf.Max(spawnManager.currentMinRockCount, Mathf.RoundToInt(value));
            spawnManager?.SetRockCountRange(spawnManager.currentMinRockCount, newMax);
            Debug.Log($"[环境参数] 最大岩石数量: {newMax}");
        });

        // 注册USV最大速度回调
        envParams.RegisterCallback("max_usv_speed", value =>
        {
            maxUSVSpeed = Mathf.Clamp(value, 1f, 5f);
            Debug.Log($"[环境参数] USV最大速度: {maxUSVSpeed}");
            usvAgent?.ResetAgentState(maxUSVSpeed, maxEpisodeTime);
        });
    }
    #endregion

    #region 环境控制
    // ===== 修改 USV_Academy.cs =====
    public void ResetEnvironment()
    {
        if (!areDependenciesLoaded) { Debug.LogWarning("环境依赖未加载完成，无法重置环境。"); return; }

        if (!gridManager.IsGridReady())
        {
            Debug.LogWarning("栅格未准备就绪，执行强制刷新...");
            gridManager.强制刷新栅格();
            StartCoroutine(WaitForGridRefreshThenReset());
            return;
        }

        // ✅【关键修改点】：即使 enableTaskLoop 没开，也必须重置智能体状态，确保参数准确。
        if (usvAgent != null)
        {
            // 强制将超时时间设定为 120 秒（大幅缩短！）
            // 120秒如果还没走完，说明避障逻辑有严重问题，直接罚死。
            usvAgent.ResetAgentState(maxUSVSpeed, 120f);
        }

        if (usvAgent != null && usvAgent.enableTaskLoop && spawnManager != null)
        {
            // 确保 spawnManager 与这里的 maxRockCount 同步（当前这里可能没做同步导致环境乱变）
            spawnManager.SetRockCountRange(minRockCount, maxRockCount);
            spawnManager.Regenerate();
        }

        Debug.Log($"环境已重置 - 岩石数量: {minRockCount}-{maxRockCount}, 最大速度: {maxUSVSpeed}, 限时: 120s");
    }

    private IEnumerator WaitForGridRefreshThenReset()
    {
        while (!gridManager.IsGridReady())
        {
            Debug.LogWarning("等待栅格强制刷新...");
            yield return new WaitForSeconds(0.1f);
        }
        Debug.Log("栅格强制刷新完成。");
        ResetEnvironment();
    }
    #endregion

    #region 公共方法
    public float GetCurrentMaxSpeed() => maxUSVSpeed;
    public float GetMaxEpisodeTime() => maxEpisodeTime;
    public bool IsEnvironmentReady() => areDependenciesLoaded && gridManager.IsGridReady();
    #endregion
}