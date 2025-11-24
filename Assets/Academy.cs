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
    // USV_Academy.cs

    // ... (其他代码) ...

    /// <summary>
    /// 每帧检查回合是否结束，以便自动重置环境
    /// </summary>
    private void Update()
    {
        // 只有当环境完全准备好后，才检查回合是否结束并自动重置
        if (areDependenciesLoaded && usvAgent != null && usvAgent.IsEpisodeDone)  // 修改 IsDone 为 IsEpisodeDone
        {
            if (usvAgent.enableTaskLoop)
            {
                Debug.Log("任务完成，准备重置环境...");
                ResetEnvironment();
            }
        }
    }

    // ... (其他代码) ...
    #endregion

    #region 依赖管理
    // Academy.cs 修改 WaitForDependencies() 方法中的智能体查找逻辑
    private IEnumerator WaitForDependencies()
    {
        while (gridManager == null)
        {
            gridManager = Object.FindFirstObjectByType<GridManager>();
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

        spawnManager = Object.FindFirstObjectByType<RandomSpawnManager>();
        // 核心修改：替换标签查找，直接查找USV_GlobalRLAgent组件（无需创建标签）
        usvAgent = Object.FindFirstObjectByType<USV_GlobalRLAgent>();

        if (ValidateDependencies())
        {
            areDependenciesLoaded = true;
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
        if (usvAgent == null) { Debug.LogError("USV_Academy: 未找到带有 'USVAgent' 标签的 USV_GlobalRLAgent！"); isValid = false; }
        return isValid;
    }
    #endregion

    #region 参数管理
    private void RegisterEnvironmentParameters()
    {
        if (envParams == null) { Debug.LogError("EnvironmentParameters 实例为空，无法注册回调。"); return; }

        envParams.RegisterCallback("rock_count_min", value => { minRockCount = Mathf.Max(1, Mathf.RoundToInt(value)); Debug.Log($"[参数更新] 最小岩石数量: {minRockCount}"); });
        envParams.RegisterCallback("rock_count_max", value => { maxRockCount = Mathf.Max(minRockCount, Mathf.RoundToInt(value)); Debug.Log($"[参数更新] 最大岩石数量: {maxRockCount}"); });
        envParams.RegisterCallback("max_usv_speed", value =>
        {
            maxUSVSpeed = Mathf.Clamp(value, 1f, 5f);
            Debug.Log($"[参数更新] USV最大速度: {maxUSVSpeed}");
            if (usvAgent != null) { usvAgent.ResetAgentState(maxUSVSpeed, maxEpisodeTime); }
        });
    }
    #endregion

    #region 环境控制
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

        if (spawnManager != null)
        {
            spawnManager.SetRockCountRange(minRockCount, maxRockCount);
            spawnManager.Regenerate();
        }

        if (usvAgent != null)
        {
            usvAgent.ResetAgentState(maxUSVSpeed, maxEpisodeTime);
            usvAgent.OnEpisodeBegin();
        }

        Debug.Log($"环境已重置 - 岩石数量范围: {minRockCount}-{maxRockCount}, 最大速度: {maxUSVSpeed}");
    }

    private IEnumerator WaitForGridRefreshThenReset()
    {
        while (!gridManager.IsGridReady()) { Debug.LogWarning("等待栅格强制刷新..."); yield return new WaitForSeconds(0.1f); }
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