using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;

/// <summary>
/// 无人船全局强化学习智能体
/// 负责全局路径规划、目标导航和奖励计算
/// 最小化配置版：简化核心逻辑，验证训练链路通畅性
/// </summary>
public class USV_GlobalRLAgent : Agent
{
    [Header("目标接近参数")]
    [Tooltip("接近目标的平滑系数，值越小减速越早")]
    public float targetProximitySmoothing = 0.3f;
    [Tooltip("最大单步奖励上限")]
    public float maxStepReward = 2f;
    [Tooltip("最小单步惩罚下限")]
    public float minStepPenalty = -1f;

    private const int TOTAL_OBSERVATIONS = 128; // 与YAML中vector_observation_size一致

    [Header("任务循环设置")]
    [Tooltip("是否启用任务自动循环")]
    public bool enableTaskLoop = true;
    [Tooltip("随机生成管理器引用")]
    public RandomSpawnManager spawnManager;

    [Tooltip("目标点Transform")]
    public Transform target;

    // 保留字段定义（保证兼容性，注释复杂依赖）
    private GridManager gridManager;
    private Rigidbody rb;
    private int gridWidth;
    private int gridHeight;
    private bool[,] passableGrid;
    private List<Vector3> safePositions;
    private float lastDistToTarget;
    private const int ViewRange = 5;
    private const float MaxSpeed = 2f;
    private const float MaxAngularSpeed = 60f;

    private Vector2Int _cachedAgentGridPos;
    private float[] _localObsBuffer;
    private bool _isObsBufferInitialized = false;
    private const int LOCAL_OBS_SIZE = (ViewRange * 2 + 1) * (ViewRange * 2 + 1);
    private float _cachedMaxGridExtent;

    private ImprovedAStar globalPathfinder;
    private int currentWaypointIndex = 0;

    private float currentMaxSpeed;
    private float currentMaxEpisodeTime;
    private float episodeStartTime;
    private BoatController boatController;

    /// <summary>
    /// 检查当前回合是否结束（自定义标记版）
    /// </summary>
    public bool IsEpisodeDone { get; private set; }

    // 新增缓存字段（保留定义，简化使用）
    private USV_LocalPlanner _localPlannerCache;
    private Vector3 _forwardDirCache;

    protected override void Awake()
    {
        base.Awake();

        // 极简初始化：仅获取刚体，移除所有外部复杂依赖
        rb = GetComponent<Rigidbody>();

        // 注释复杂组件初始化（保留代码，便于恢复）
        // boatController = GetComponent<BoatController>();
        // gridManager = UnityEngine.Object.FindFirstObjectByType<GridManager>();
        // globalPathfinder = UnityEngine.Object.FindFirstObjectByType<ImprovedAStar>();

        if (rb != null)
        {
            rb.maxAngularVelocity = 5f;
            rb.useGravity = false;
        }
        else
        {
            Debug.LogError("未找到Rigidbody组件！请给智能体添加刚体组件");
        }

        // 移除协程调用
        // if (gridManager == null)
        // {
        //     Debug.LogError("未找到GridManager组件！请确保场景中存在GridManager");
        // }
        // else
        // {
        //     StartCoroutine(WaitForGridInit());
        // }
    }

    /// <summary>
    /// 保留方法定义（空实现，保证兼容性）
    /// </summary>
    public void ResetAgentState(float maxSpeed, float maxEpisodeTime)
    {
        currentMaxSpeed = maxSpeed <= 0 ? MaxSpeed : maxSpeed;
        currentMaxEpisodeTime = maxEpisodeTime <= 0 ? 60f : maxEpisodeTime;
        episodeStartTime = Time.time;
        lastDistToTarget = target != null ? Vector3.Distance(transform.position, target.position) : 0;
    }

    /// <summary>
    /// 注释复杂协程（保留定义，便于恢复）
    /// </summary>
    private IEnumerator WaitForGridInit()
    {
        yield break;
    }

    public override void Initialize()
    {
        if (rb != null)
        {
            rb.maxAngularVelocity = 5f;
            rb.useGravity = false;
        }
        else
        {
            Debug.LogError("未找到Rigidbody组件！请给智能体添加刚体组件");
        }
        _cachedMaxGridExtent = 100f; // 固定值，简化计算
    }

    // ========== 核心修改：极简版OnEpisodeBegin ==========
    public override void OnEpisodeBegin()
    {
        IsEpisodeDone = false;
        episodeStartTime = Time.time; // 仅保留基础时间初始化

        // 极简重置：只清空刚体速度
        if (rb != null)
        {
            rb.linearVelocity = Vector3.zero;
            rb.angularVelocity = Vector3.zero;
        }

        // 禁用复杂依赖逻辑
        enableTaskLoop = false;
        currentWaypointIndex = 0;
        currentMaxSpeed = MaxSpeed;
        currentMaxEpisodeTime = 10f; // 最小化回合时间（10秒）

        Debug.Log("【极简配置】智能体基础状态重置完成");
    }

    /// <summary>
    /// 注释复杂重置协程（保留定义，便于恢复）
    /// </summary>
    private IEnumerator WaitForGridInitThenReset()
    {
        yield break;
    }

    /// <summary>
    /// 保留方法定义（空实现）
    /// </summary>
    private void GenerateSafePositions(float minDistance)
    {
        if (safePositions == null)
        {
            safePositions = new List<Vector3>();
        }
        else
        {
            safePositions.Clear();
        }
    }

    /// <summary>
    /// 保留方法定义（空实现）
    /// </summary>
    private void GenerateSafePositions()
    {
        GenerateSafePositions(1.0f);
    }

    /// <summary>
    /// 保留方法定义（空实现）
    /// </summary>
    private bool IsPositionSafe(Vector3 position, float minDistance)
    {
        return true;
    }

    /// <summary>
    /// 自定义结束回合方法
    /// </summary>
    private void EndEpisodeCustom()
    {
        IsEpisodeDone = true;
        EndEpisode();
    }

    /// <summary>
    /// 保留方法定义（空实现）
    /// </summary>
    private void NotifyBoatLoadNewPath()
    {
        Debug.LogWarning("【极简配置】跳过BoatController路径加载");
    }

    private void CleanupTensorData()
    {
        try
        {
            GC.Collect();
            GC.WaitForPendingFinalizers();
        }
        catch (Exception e)
        {
            Debug.LogError($"清理张量数据失败: {e.Message}");
        }
    }

    private void OnDestroy()
    {
        CleanupTensorData();
    }

    // ========== 核心修改：极简版CollectObservations ==========
    public override void CollectObservations(VectorSensor sensor)
    {
        // 快速失败：基础组件缺失返回空观测
        if (rb == null || target == null)
        {
            float[] emptyObs = new float[TOTAL_OBSERVATIONS];
            sensor.AddObservation(emptyObs);
            return;
        }

        // 仅保留5维核心观测，移除所有复杂计算
        float[] baseObservations = new float[5];
        Vector3 agentPos = transform.position;
        Vector3 forwardDir = transform.forward;

        // 1. 前进速度（归一化）
        float forwardSpeed = Vector3.Dot(forwardDir, rb.linearVelocity);
        baseObservations[0] = Mathf.Clamp(forwardSpeed / MaxSpeed, -1f, 1f);

        // 2. 朝向（归一化）
        baseObservations[1] = ((transform.eulerAngles.y % 360f) / 180f) - 1f;

        // 3. 目标距离（归一化，固定最大距离100米）
        float distToTarget = Vector3.Distance(agentPos, target.position);
        baseObservations[2] = Mathf.Clamp01(distToTarget / 100f);

        // 4. 目标角度（归一化）
        float angleToTarget = Vector3.SignedAngle(forwardDir, target.position - agentPos, Vector3.up);
        baseObservations[3] = angleToTarget / 180f;

        // 5. 占位值（移除路径规划依赖）
        baseObservations[4] = 0f;

        // 补全剩余观测维度（用0填充，保证总数匹配）
        float[] fullObs = new float[TOTAL_OBSERVATIONS];
        Array.Copy(baseObservations, fullObs, baseObservations.Length);

        sensor.AddObservation(fullObs);
        Debug.Log("【极简配置】观测数据生成完成（仅5维核心+补0）");
    }

    // ========== 核心修改：强制验证版OnActionReceived ==========
    public override void OnActionReceived(ActionBuffers actions)
    {
        // 增强日志：先打印进入方法的标记，确认方法被调用
        Debug.Log("【极简配置】进入OnActionReceived方法，IsEpisodeDone=" + IsEpisodeDone + "，rb是否为空=" + (rb == null));

        // 快速失败：终止状态直接返回（保留，但加日志）
        if (IsEpisodeDone || rb == null)
        {
            Debug.LogWarning("【极简配置】快速失败返回：IsEpisodeDone=" + IsEpisodeDone + "，rb是否为空=" + (rb == null));
            return;
        }

        // 强制打印动作值，确认MLAgents是否传参
        int discreteAction = Mathf.Clamp(actions.DiscreteActions[0], 0, 3);
        Debug.Log("【极简配置】收到离散动作值：" + discreteAction);

        // 仅保留前进/停止逻辑，移除旋转/减速/复杂奖励
        Vector3 forwardDir = transform.forward;

        if (discreteAction == 0) // 仅处理前进
        {
            Vector3 forwardVel = forwardDir * MaxSpeed * 0.5f; // 固定50%速度
            rb.AddForce(forwardVel, ForceMode.VelocityChange);

            // 简单速度限制
            if (rb.linearVelocity.magnitude > MaxSpeed)
            {
                rb.linearVelocity = rb.linearVelocity.normalized * MaxSpeed;
            }
            Debug.Log("【极简配置】执行前进动作，当前速度：" + rb.linearVelocity.magnitude);
        }
        else // 所有其他动作都改为停止
        {
            rb.linearVelocity = Vector3.zero;
            Debug.Log("【极简配置】执行停止动作，速度清零");
        }

        // ========== 关键修改：强制触发终止（优先验证链路） ==========
        // 1. 先打印距离和时间，确认数值
        float distToTarget = target != null ? Vector3.Distance(transform.position, target.position) : 999f;
        float episodeElapsedTime = Time.time - episodeStartTime;
        Debug.Log("【极简配置】当前到目标距离：" + distToTarget + "米，已耗时：" + episodeElapsedTime + "秒，超时阈值：" + currentMaxEpisodeTime + "秒");

        // 2. 弱化目标距离条件（从2米→5米）+ 强制超时（从10秒→5秒）
        bool reachTarget = distToTarget < 5f;
        bool timeOut = episodeElapsedTime > 5f;

        if (reachTarget) // 距离目标<5米时结束回合
        {
            AddReward(1f); // 固定奖励
            EndEpisodeCustom();
            Debug.Log("【极简配置】到达目标（距离<5米），结束回合，奖励+1");
        }
        // 超时终止（5秒）
        else if (timeOut)
        {
            AddReward(-0.5f); // 固定惩罚
            EndEpisodeCustom();
            Debug.Log("【极简配置】回合超时（>5秒），结束回合，奖励-0.5");
        }
        // 3. 终极兜底：如果5秒还没超时，强制终止（仅用于验证）
        else if (episodeElapsedTime > 5.1f)
        {
            AddReward(-1f);
            EndEpisodeCustom();
            Debug.Log("【极简配置】兜底终止回合（>5.1秒），奖励-1");
        }
    }

    // 保留空方法定义（保证兼容性，便于恢复）
    private void CalculateReward()
    {
        Debug.LogWarning("【极简配置】跳过复杂奖励计算");
    }

    private void Update()
    {
        if (rb != null)
        {
            _forwardDirCache = transform.forward;
        }
    }

    void MoveAgent(int action)
    {
        Debug.LogWarning("【极简配置】跳过MoveAgent复杂逻辑");
    }

    private void UpdateWaypointIndex()
    {
        Debug.LogWarning("【极简配置】跳过路点更新逻辑");
    }

    private bool IsPassable(Vector2Int gridPos)
    {
        return true; // 极简配置默认全可通行
    }

    private void CachePassableGrid()
    {
        Debug.LogWarning("【极简配置】跳过栅格缓存逻辑");
    }
}