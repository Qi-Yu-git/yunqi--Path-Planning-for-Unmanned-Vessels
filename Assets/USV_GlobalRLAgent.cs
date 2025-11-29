using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;
using System.Reflection;
using System.Linq;

/// <summary>
/// 无人船全局强化学习智能体
/// 负责全局路径规划、目标导航、观测收集和奖励计算
/// 融合观测稳定性修复与原有核心功能
/// </summary>
public class USV_GlobalRLAgent : Agent
{
    [Header("目标与控制器关联")]
    [Tooltip("目标点Transform（自动关联A*目标点）")]
    public Transform target;
    [Tooltip("无人船控制器引用")]
    public BoatController boatController;
    [Tooltip("随机生成管理器引用")]
    public RandomSpawnManager spawnManager;

    [Header("奖励参数（优化后合理范围）")]
    [Tooltip("接近目标的平滑系数，值越小减速越早")]
    public float targetProximitySmoothing = 0.3f;
    [Tooltip("最大单步奖励上限（建议0.1-0.5）")]
    public float maxStepReward = 0.3f;
    [Tooltip("最小单步惩罚下限（建议-0.2--0.1）")]
    public float minStepPenalty = -0.15f;
    [Tooltip("到达目标奖励（建议10-20）")]
    public float reachTargetReward = 15f;
    [Tooltip("碰撞惩罚（建议-10--5）")]
    public float collisionPenalty = -8f;
    [Tooltip("超时惩罚（建议-5--3）")]
    public float timeoutPenalty = -4f;
    [Tooltip("远离目标额外惩罚系数")]
    public float awayFromTargetPenaltyFactor = 0.2f;

    [Header("任务与调试设置")]
    [Tooltip("是否启用任务自动循环")]
    public bool enableTaskLoop = true;
    [Tooltip("调试日志开关")]
    [SerializeField] private bool debugMode = false;
    [Tooltip("观测维度校验开关（开启后强制校验128维）")]
    public bool enforceObservationCount = true;

    [Header("栅格与运动参数（可动态初始化）")]
    [Tooltip("栅格宽度（格子数量）")]
    public float gridWidth = 50f;
    [Tooltip("栅格高度（格子数量）")]
    public float gridHeight = 50f;
    [Tooltip("局部障碍物检测范围（5x5网格）")]
    public int viewRange = 5;
    [Tooltip("基准最大速度")]
    public float maxSpeed = 2f;
    [Tooltip("最大角速度")]
    public float maxAngularSpeed = 60f;

    // 核心组件引用
    private GridManager gridManager;
    private Rigidbody rb;
    private ImprovedAStar globalPathfinder;
    private USV_LocalPlanner localPlanner;

    // 栅格与路径数据
    private int gridWidthFromManager;
    private int gridHeightFromManager;
    private bool[,] passableGrid;
    private List<Vector3> safePositions;
    private int currentWaypointIndex = 0;

    // 动态状态参数
    private float lastDistToTarget;
    private float currentMaxSpeed;
    private float currentMaxEpisodeTime;
    private float episodeStartTime;

    /// <summary>
    /// 检查当前回合是否结束（通过反射获取私有字段）
    /// </summary>
    public bool IsEpisodeDone
    {
        get
        {
            try
            {
                FieldInfo fieldInfo = typeof(Agent).GetField("m_IsDone",
                    BindingFlags.Instance | BindingFlags.NonPublic);
                return fieldInfo != null && (bool)fieldInfo.GetValue(this);
            }
            catch (Exception e)
            {
                Debug.LogError($"获取回合状态失败: {e.Message}");
                return false;
            }
        }
    }

    protected override void Awake()
    {
        base.Awake();
        // 自动获取核心组件（减少Inspector赋值依赖）
        boatController = GetComponent<BoatController>() ?? FindFirstObjectByType<BoatController>();
        gridManager = FindFirstObjectByType<GridManager>();
        globalPathfinder = FindFirstObjectByType<ImprovedAStar>();
        localPlanner = GetComponent<USV_LocalPlanner>();
        rb = GetComponent<Rigidbody>();

        // 组件缺失警告
        if (boatController == null) Debug.LogError("未找到BoatController组件！");
        if (gridManager == null) Debug.LogError("未找到GridManager组件！");
        if (globalPathfinder == null) Debug.LogError("未找到ImprovedAStar组件！");
        if (rb == null) Debug.LogError("未找到Rigidbody组件！请给智能体添加刚体组件");

        // 初始化默认参数（优先用Inspector配置值）
        InitializeParameters(gridWidth, gridHeight, viewRange, maxSpeed, maxAngularSpeed);

        // 等待栅格初始化
        if (gridManager != null)
        {
            StartCoroutine(WaitForGridInit());
        }
    }

    public override void Initialize()
    {
        base.Initialize();
        // 初始化刚体参数
        if (rb != null)
        {
            rb.maxAngularVelocity = 5f;
            rb.useGravity = false;
            rb.interpolation = RigidbodyInterpolation.None;
        }
    }

    /// <summary>
    /// 初始化栅格/运动参数（支持外部动态传入）
    /// </summary>
    public void InitializeParameters(float gridWidth, float gridHeight, int viewRange, float maxSpeed, float maxAngularSpeed)
    {
        this.gridWidth = gridWidth;
        this.gridHeight = gridHeight;
        this.viewRange = Mathf.Max(1, viewRange); // 确保检测范围至少为1
        this.maxSpeed = Mathf.Max(0.5f, maxSpeed); // 最小速度限制
        this.maxAngularSpeed = Mathf.Max(10f, maxAngularSpeed); // 最小角速度限制

        if (debugMode)
        {
            Debug.Log($"参数初始化完成：栅格{gridWidth}x{gridHeight}，检测范围{viewRange}，最大速度{maxSpeed}");
        }
    }

    /// <summary>
    /// 回合开始时初始化
    /// </summary>
    public override void OnEpisodeBegin()
    {
        // 基础校验
        if (gridManager == null || safePositions == null || safePositions.Count == 0)
        {
            Debug.LogWarning("GridManager未初始化或安全位置为空，跳过回合重置");
            return;
        }

        // 任务循环：重新生成环境（礁石、起点、目标点）
        if (enableTaskLoop)
        {
            if (spawnManager != null)
            {
                spawnManager.Regenerate();
                // 延迟关联新目标点（等待生成完成）
                Invoke(nameof(BindNewTargetFromAStar), 0.3f);
            }
            else
            {
                Debug.LogError("未找到RandomSpawnManager，无法重置环境");
                EndEpisode();
                return;
            }
        }
        else
        {
            BindNewTargetFromAStar(); // 绑定已有目标点
        }

        // 重置物理状态
        if (rb != null)
        {
            rb.linearVelocity = Vector3.zero;
            rb.angularVelocity = Vector3.zero;
        }

        // 重置路径与状态
        currentWaypointIndex = 0;
        episodeStartTime = Time.time;
        lastDistToTarget = target != null ? Vector3.Distance(transform.position, target.position) : 0;

        // 通知船控制器加载新路径
        if (globalPathfinder != null)
        {
            globalPathfinder.CalculatePathAfterDelay();
            Invoke(nameof(NotifyBoatLoadNewPath), 0.5f);
        }

        if (debugMode) Debug.Log($"Episode开始：目标点位置{target?.position}，初始距离{lastDistToTarget:F2}");
    }

    /// <summary>
    /// 绑定A*生成的新目标点（解决目标点动态生成后关联问题）
    /// </summary>
    private void BindNewTargetFromAStar()
    {
        if (globalPathfinder != null && globalPathfinder.TargetPos != null)
        {
            target = globalPathfinder.TargetPos;
            if (debugMode) Debug.Log($"已绑定新目标点：{target.position}");
        }
        else
        {
            Debug.LogWarning("A*目标点为空，无法绑定");
        }
    }

    /// <summary>
    /// 通知船控制器加载新路径
    /// </summary>
    private void NotifyBoatLoadNewPath()
    {
        if (boatController != null)
        {
            boatController.isPathLoaded = false;
            boatController.TryLoadPath();
            if (debugMode) Debug.Log("通知BoatController加载新路径");
        }
        else
        {
            Debug.LogWarning("未找到BoatController，无法通知加载新路径");
        }
    }

    /// <summary>
    /// 收集观测数据（固定128维，修复观测缺失问题）
    /// </summary>
    public override void CollectObservations(VectorSensor sensor)
    {
        // 初始化观测数据列表（确保顺序和数量固定）
        List<float> observations = new List<float>();

        // 基础组件校验
        bool hasRequiredComponents = rb != null && target != null && gridManager != null;
        if (!hasRequiredComponents)
        {
            Debug.LogWarning("缺少必要组件（RB/目标点/栅格），使用占位观测值");
            // 添加128个占位观测值，避免观测维度不匹配
            for (int i = 0; i < 128; i++)
            {
                observations.Add(0f);
            }
        }
        else
        {
            // 1. 前进速度（归一化到[-1,1]）- 1维
            float forwardSpeed = Vector3.Dot(transform.forward, rb.linearVelocity);
            observations.Add(Mathf.Clamp(forwardSpeed / maxSpeed, -1f, 1f));

            // 2. 无人船朝向（归一化到[-1,1]）- 1维
            float normalizedYaw = (transform.eulerAngles.y % 360f) / 180f - 1f;
            observations.Add(normalizedYaw);

            // 3. 目标距离（归一化到[0,1]）- 1维
            float distToTarget = Vector3.Distance(transform.position, target.position);
            float normalizedDist = Mathf.Clamp01(distToTarget / (Mathf.Max(gridWidth, gridHeight) * 1.5f));
            observations.Add(normalizedDist);

            // 4. 目标方向角（归一化到[-1,1]）- 1维
            float angleToTarget = Vector3.SignedAngle(transform.forward, target.position - transform.position, Vector3.up);
            observations.Add(angleToTarget / 180f);

            // 5. 全局路径下一个航点方向角（归一化到[-1,1]）- 1维
            if (globalPathfinder != null && globalPathfinder.path != null && globalPathfinder.path.Count > currentWaypointIndex + 1)
            {
                Vector3 nextWaypointWorld = gridManager.栅格转世界(globalPathfinder.path[currentWaypointIndex + 1]);
                float angleToWaypoint = Vector3.SignedAngle(transform.forward, nextWaypointWorld - transform.position, Vector3.up);
                observations.Add(angleToWaypoint / 180f);
            }
            else
            {
                observations.Add(0f); // 路径无效时填充0
            }

            // 6. 局部障碍物检测（(2*viewRange+1)x(2*viewRange+1)网格）- 对应维度
            Vector2Int agentGridPos = gridManager.世界转栅格(transform.position);
            // 替换为固定范围（确保总维度=121，加上其他7维正好128）
            int fixedViewRange = 5; // 强制5x5网格（2*5+1=11，11x11=121维）
            for (int x = -fixedViewRange; x <= fixedViewRange; x++)
            {
                for (int z = -fixedViewRange; z <= fixedViewRange; z++)
                {
                    Vector2Int checkGrid = new Vector2Int(agentGridPos.x + x, agentGridPos.y + z);
                    bool isObstacle = !IsPassable(checkGrid);
                    observations.Add(isObstacle ? 1f : 0f);
                }
            }

            // 7. 动态障碍物速度（归一化到[-1,1]）- 2维
            if (localPlanner != null && localPlanner.dynamicObstacleVelocities.Count > 0)
            {
                Vector3 closestObsVel = localPlanner.dynamicObstacleVelocities[0];
                observations.Add(Mathf.Clamp(closestObsVel.x / 5f, -1f, 1f));
                observations.Add(Mathf.Clamp(closestObsVel.z / 5f, -1f, 1f));
            }
            else
            {
                observations.Add(0f);
                observations.Add(0f);
            }

            // 8. 路径加载状态（0=未加载，1=已加载）- 1维（补充维度到128）
            float pathLoaded = boatController != null && boatController.IsPathLoaded ? 1f : 0f;
            observations.Add(pathLoaded);
        }

        // 强制校验观测维度（确保128维）- 核心修复
        if (enforceObservationCount && observations.Count != 128)
        {
            int diff = 128 - observations.Count;
            if (diff > 0)
            {
                // 不足时补充0
                for (int i = 0; i < diff; i++)
                {
                    observations.Add(0f);
                }
                Debug.LogWarning($"观测维度不足（实际{observations.Count - diff}维），已补充{diff}个0值");
            }
            else if (diff < 0)
            {
                // 超出时截断
                observations = observations.Take(128).ToList();
                Debug.LogWarning($"观测维度超出（实际{observations.Count - diff}维），已截断到128维");
            }
        }

        // 添加所有观测值到传感器
        foreach (float obs in observations)
        {
            sensor.AddObservation(obs);
        }

        // 调试日志（仅打印前5个关键观测值）
        if (debugMode)
        {
            string obsLog = "观测数据（前5项）：";
            for (int i = 0; i < Mathf.Min(5, observations.Count); i++)
            {
                obsLog += $"{observations[i]:F3} ";
            }
            Debug.Log(obsLog);
        }
    }

    /// <summary>
    /// 接收动作并计算奖励
    /// </summary>
    public override void OnActionReceived(ActionBuffers actions)
    {
        // 基础组件校验
        if (target == null || gridManager == null || rb == null)
        {
            AddReward(minStepPenalty);
            return;
        }

        // 执行移动动作
        MoveAgent(actions.DiscreteActions[0]);

        // 计算核心状态参数
        float distToTarget = Vector3.Distance(transform.position, target.position);
        float distanceDelta = lastDistToTarget - distToTarget; // 距离变化（正=靠近，负=远离）
        float currentSpeed = rb.linearVelocity.magnitude;


        // 1. 基于距离变化的步进奖励（靠近奖励，远离惩罚）
        float proximityFactor = Mathf.Clamp01(1 - (distToTarget / (Mathf.Max(gridWidth, gridHeight) * 1f)));
        float stepReward = distanceDelta * (1 + proximityFactor * targetProximitySmoothing) * 0.15f;
        stepReward = Mathf.Clamp(stepReward, minStepPenalty, maxStepReward);
        AddReward(stepReward);

        // 2. 平滑移动奖励（鼓励稳定速度）
        float speedStabilityReward = 0.05f * (1 - Mathf.Abs(currentSpeed - maxSpeed * 0.5f) / (maxSpeed * 0.5f));
        AddReward(speedStabilityReward);

        // 3. 远离目标额外惩罚
        if (distanceDelta < 0)
        {
            float awayPenalty = distanceDelta * awayFromTargetPenaltyFactor;
            AddReward(awayPenalty);
            if (debugMode) Debug.Log($"远离目标惩罚：{awayPenalty:F3}");
        }

        // 4. 碰撞惩罚（检测当前栅格是否为障碍物）
        Vector2Int currentGrid = gridManager.世界转栅格(transform.position);
        if (!IsPassable(currentGrid))
        {
            AddReward(collisionPenalty);
            if (debugMode) Debug.Log($"碰撞惩罚：{collisionPenalty}，Episode结束");
            EndEpisode();
            return;
        }

        // 5. 到达目标奖励（距离小于2米）
        if (distToTarget < 2f)
        {
            // 低速到达额外奖励（避免冲过目标）
            float speedBonus = currentSpeed < maxSpeed * 0.3f ? 5f : 0f;
            AddReward(reachTargetReward + speedBonus);
            if (debugMode) Debug.Log($"到达目标奖励：{reachTargetReward + speedBonus:F1}，Episode结束");
            EndEpisode();
            return;
        }

        // 6. 超时惩罚（超过最大回合时间）
        if (Time.time - episodeStartTime > currentMaxEpisodeTime)
        {
            AddReward(timeoutPenalty);
            if (debugMode) Debug.Log($"超时惩罚：{timeoutPenalty}，Episode结束");
            EndEpisode();
            return;
        }

        // 7. 远离边界奖励（避免靠近地图边缘）
        Vector3 agentPos = transform.position;
        float edgeDistance = Mathf.Min(
            agentPos.x - (gridManager.栅格原点.x - gridManager.栅格宽度 * gridManager.栅格尺寸 / 2),
            (gridManager.栅格原点.x + gridManager.栅格宽度 * gridManager.栅格尺寸 / 2) - agentPos.x,
            agentPos.z - (gridManager.栅格原点.z - gridManager.栅格高度 * gridManager.栅格尺寸 / 2),
            (gridManager.栅格原点.z + gridManager.栅格高度 * gridManager.栅格尺寸 / 2) - agentPos.z
        );
        float edgeReward = Mathf.Clamp01(edgeDistance / 10f) * 0.02f;
        AddReward(edgeReward);

        // 更新状态
        UpdateWaypointIndex();
        lastDistToTarget = distToTarget;
    }

    /// <summary>
    /// 执行移动动作（前进/左转/右转/减速）
    /// </summary>
    private void MoveAgent(int action)
    {
        if (rb == null) return;

        float currentSpeed = rb.linearVelocity.magnitude;
        // 使用当前配置的最大速度（支持动态调整）
        float speedFactor = currentMaxSpeed > 0 ? currentMaxSpeed / maxSpeed : 1f;

        switch (action)
        {
            case 0: // 前进（根据当前速度调整推力）
                float forwardForce = currentSpeed < currentMaxSpeed * 0.8f
                    ? currentMaxSpeed * 0.6f * speedFactor
                    : currentMaxSpeed * 0.2f * speedFactor;
                rb.AddForce(transform.forward * forwardForce, ForceMode.VelocityChange);
                break;
            case 1: // 左转
                transform.Rotate(Vector3.up, -15f * Time.fixedDeltaTime * speedFactor);
                break;
            case 2: // 右转
                transform.Rotate(Vector3.up, 15f * Time.fixedDeltaTime * speedFactor);
                break;
            case 3: // 减速
                rb.linearVelocity *= 0.9f;
                break;
        }

        // 限制最大速度
        if (rb.linearVelocity.magnitude > currentMaxSpeed)
        {
            rb.linearVelocity = rb.linearVelocity.normalized * currentMaxSpeed;
        }
    }

    /// <summary>
    /// 更新当前路径点索引（到达当前航点后切换到下一个）
    /// </summary>
    private void UpdateWaypointIndex()
    {
        if (globalPathfinder == null || globalPathfinder.path == null || globalPathfinder.path.Count <= currentWaypointIndex)
            return;

        Vector3 currentWaypointWorld = gridManager.栅格转世界(globalPathfinder.path[currentWaypointIndex]);
        float distToWaypoint = Vector3.Distance(transform.position, currentWaypointWorld);

        if (distToWaypoint < 1.5f)
        {
            currentWaypointIndex = Mathf.Min(currentWaypointIndex + 1, globalPathfinder.path.Count - 1);
            if (debugMode) Debug.Log($"切换到航点 {currentWaypointIndex}：{gridManager.栅格转世界(globalPathfinder.path[currentWaypointIndex])}");
        }
    }

    /// <summary>
    /// 检查栅格是否可通行
    /// </summary>
    private bool IsPassable(Vector2Int gridPos)
    {
        if (gridPos.x < 0 || gridPos.x >= gridWidthFromManager ||
            gridPos.y < 0 || gridPos.y >= gridHeightFromManager)
            return false;

        return passableGrid != null && passableGrid[gridPos.x, gridPos.y];
    }

    /// <summary>
    /// 等待栅格初始化完成
    /// </summary>
    private IEnumerator WaitForGridInit()
    {
        float timeout = 15f;
        float timer = 0f;
        while (gridManager == null || !gridManager.IsGridReady())
        {
            timer += Time.deltaTime;
            if (timer > timeout)
            {
                Debug.LogError("USV_GlobalRLAgent：等待GridManager超时，强制结束等待");
                yield break;
            }
            if (debugMode) Debug.Log("等待GridManager初始化...");
            yield return new WaitForSeconds(0.5f);
        }

        // 缓存栅格数据后，强制生成安全位置
        gridWidthFromManager = gridManager.gridWidth;
        gridHeightFromManager = gridManager.gridHeight;
        CachePassableGrid();
        GenerateSafePositions();

        // 若安全位置为空，手动添加几个默认位置
        if (safePositions == null || safePositions.Count == 0)
        {
            Debug.LogError("安全位置为空，添加默认位置");
            safePositions = new List<Vector3>
        {
            gridManager.栅格转世界(new Vector2Int(10, 10)),
            gridManager.栅格转世界(new Vector2Int(20, 20))
        };
        }
        if (debugMode) Debug.Log($"GridManager初始化完成：安全位置{safePositions.Count}个");
    }
    /// <summary>
    /// 缓存栅格通行性数据
    /// </summary>
    private void CachePassableGrid()
    {
        if (gridManager == null) return;

        passableGrid = new bool[gridWidthFromManager, gridHeightFromManager];
        for (int x = 0; x < gridWidthFromManager; x++)
        {
            for (int z = 0; z < gridHeightFromManager; z++)
            {
                passableGrid[x, z] = gridManager.栅格是否可通行(new Vector2Int(x, z));
            }
        }
    }

    /// <summary>
    /// 生成安全位置列表（用于回合重置时的随机 spawn）
    /// </summary>
    private void GenerateSafePositions()
    {
        safePositions = new List<Vector3>();
        if (gridManager == null || passableGrid == null) return;

        for (int x = 0; x < gridWidthFromManager; x++)
        {
            for (int z = 0; z < gridHeightFromManager; z++)
            {
                if (passableGrid[x, z])
                {
                    safePositions.Add(gridManager.栅格转世界(new Vector2Int(x, z)));
                }
            }
        }
    }

    /// <summary>
    /// 重置智能体状态（供外部调用，动态调整参数）
    /// </summary>
    public void ResetAgentState(float maxSpeed, float maxEpisodeTime)
    {
        currentMaxSpeed = Mathf.Max(0.5f, maxSpeed); // 最小速度限制
        currentMaxEpisodeTime = Mathf.Max(10f, maxEpisodeTime); // 最小回合时间限制
        episodeStartTime = Time.time;

        if (target != null)
        {
            lastDistToTarget = Vector3.Distance(transform.position, target.position);
        }
        else
        {
            lastDistToTarget = 0;
        }

        if (debugMode) Debug.Log($"重置智能体状态：最大速度{currentMaxSpeed}，最大回合时间{currentMaxEpisodeTime}");
    }

    /// <summary>
    /// 外部调用的碰撞惩罚（供BoatController触发）
    /// </summary>
    public void OnCollisionDetected()
    {
        AddReward(collisionPenalty);
        if (debugMode) Debug.Log($"碰撞惩罚：{collisionPenalty}，Episode结束");
        EndEpisode();
    }

    /// <summary>
    /// 清理资源（避免内存泄漏）
    /// </summary>
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
}