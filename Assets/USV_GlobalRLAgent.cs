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

    // 全局路径相关
    private ImprovedAStar globalPathfinder;
    private int currentWaypointIndex = 0;

    // 动态参数
    private float currentMaxSpeed;
    private float currentMaxEpisodeTime;
    private float episodeStartTime;
    private BoatController boatController;

    /// <summary>
    /// 检查当前回合是否结束（自定义标记版，替代原有反射逻辑）
    /// </summary>
    public bool IsEpisodeDone { get; private set; }

    protected override void Awake()
    {
        base.Awake();

        // ========== 新增：自动创建/获取核心组件 ==========
        // 1. 自动添加Rigidbody
        rb = GetComponent<Rigidbody>();
        if (rb == null)
        {
            rb = gameObject.AddComponent<Rigidbody>();
            rb.maxAngularVelocity = 5f;
            rb.useGravity = false;
            rb.isKinematic = false;
            Debug.LogWarning("自动添加Rigidbody组件");
        }

        // 2. 自动创建默认目标点（避免target为空）
        if (target == null)
        {
            GameObject targetObj = new GameObject("Default_Target");
            targetObj.transform.position = new Vector3(10f, 0f, 10f); // 默认目标位置
            target = targetObj.transform;
            Debug.LogWarning("未指定target，自动创建默认目标点");
        }

        // 3. 容错获取依赖组件
        boatController = GetComponent<BoatController>();
        gridManager = FindFirstObjectByType<GridManager>();
        globalPathfinder = FindFirstObjectByType<ImprovedAStar>();

        // 4. GridManager缺失时的兜底逻辑
        if (gridManager == null)
        {
            Debug.LogError("未找到GridManager组件！使用兜底栅格数据");
            // 初始化兜底栅格参数，避免空指针
            gridWidth = 50;
            gridHeight = 50;
            passableGrid = new bool[gridWidth, gridHeight];
            for (int x = 0; x < gridWidth; x++)
                for (int z = 0; z < gridHeight; z++)
                    passableGrid[x, z] = true; // 默认全可通行
        }
        else
        {
            StartCoroutine(WaitForGridInit());
        }
    }

    /// <summary>
    /// 重置智能体状态
    /// </summary>
    /// <param name="maxSpeed">最大速度</param>
    /// <param name="maxEpisodeTime">最大回合时间</param>
    public void ResetAgentState(float maxSpeed, float maxEpisodeTime)
    {
        currentMaxSpeed = maxSpeed <= 0 ? MaxSpeed : maxSpeed;
        currentMaxEpisodeTime = maxEpisodeTime <= 0 ? 60f : maxEpisodeTime;
        episodeStartTime = Time.time;

        lastDistToTarget = target != null ? Vector3.Distance(transform.position, target.position) : 0;
    }

    /// <summary>
    /// 等待栅格初始化完成
    /// </summary>
    private IEnumerator WaitForGridInit()
    {
        if (gridManager == null) yield break;

        while (!gridManager.IsGridReady())
        {
            Debug.Log("等待GridManager初始化...");
            yield return new WaitForSeconds(0.5f);
        }

        gridWidth = gridManager.gridWidth;
        gridHeight = gridManager.gridHeight;
        CachePassableGrid();
        GenerateSafePositions(); // 调用无参方法，无歧义
        Debug.Log("GridManager初始化完成，已缓存通行性数据");
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
    }

    // ========== 核心修复：重写OnEpisodeBegin方法 ==========
    public override void OnEpisodeBegin()
    {
        IsEpisodeDone = false;
        // 启动协程等待GridManager初始化并确保安全位置非空
        StartCoroutine(WaitForGridInitThenReset());
    }

    /// <summary>
    /// 等待GridManager初始化完成后执行重置逻辑
    /// </summary>
    private IEnumerator WaitForGridInitThenReset()
    {
        // 1. 等待GridManager就绪
        while (gridManager == null || !gridManager.IsGridReady())
        {
            Debug.LogWarning("等待 GridManager 初始化...");
            yield return new WaitForSeconds(0.1f);
        }

        // 2. 重新生成安全位置（增加重试机制 + 宽松阈值）
        Vector3 safePos = Vector3.zero;
        int retryCount = 0;
        int maxRetries = 5;
        float initialMinDistance = 1.0f;
        float minDistanceStep = 0.2f;
        float currentMinDistance = initialMinDistance;

        while (safePos == Vector3.zero && retryCount < maxRetries)
        {
            GenerateSafePositions(currentMinDistance);
            if (safePositions.Count > 0)
            {
                safePos = safePositions[UnityEngine.Random.Range(0, safePositions.Count)];
            }
            else
            {
                retryCount++;
                currentMinDistance = Mathf.Max(0.1f, initialMinDistance - (retryCount * minDistanceStep));
                Debug.LogWarning($"安全位置生成失败（第{retryCount}次重试），放宽阈值到: {currentMinDistance}");
                if (retryCount >= 2) CachePassableGrid();
            }
            if (safePos == Vector3.zero) yield return new WaitForSeconds(0.05f);
        }

        // ========== 新增：强制给Agent赋值初始位置（关键！） ==========
        if (safePos != Vector3.zero)
        {
            transform.position = safePos; // 必须设置位置，否则Agent无物理体
            transform.rotation = Quaternion.identity; // 重置朝向
        }
        else
        {
            // 终极兜底：手动指定初始位置
            transform.position = new Vector3(0, 0.4f, 0);
            Debug.LogError("安全位置全空，强制设置初始位置为(0,0.4,0)");
        }

        // 3. 执行原有重置逻辑
        if (enableTaskLoop) spawnManager?.Regenerate();
        if (rb != null)
        {
            rb.linearVelocity = Vector3.zero;
            rb.angularVelocity = Vector3.zero;
        }
        currentWaypointIndex = 0;
        globalPathfinder?.CalculatePathAfterDelay();
        Invoke(nameof(NotifyBoatLoadNewPath), 0.5f);
        ResetAgentState(MaxSpeed, 60f);

        Debug.Log($"智能体重置完成：位置={transform.position}，重试次数={retryCount}");
    }

    /// <summary>
    /// 生成安全位置列表（支持动态距离阈值）
    /// </summary>
    /// <param name="minDistance">与障碍物的最小安全距离</param>
    private void GenerateSafePositions(float minDistance)
    {
        // 清空原有安全位置列表
        safePositions = new List<Vector3>();

        if (gridManager == null || passableGrid == null) return;

        // 遍历所有栅格，筛选满足安全距离的位置
        for (int x = 0; x < gridWidth; x++)
        {
            for (int z = 0; z < gridHeight; z++)
            {
                if (passableGrid[x, z])
                {
                    Vector3 worldPos = gridManager.GridToWorld(new Vector2Int(x, z));
                    // 检查该位置与周围障碍物的距离是否满足阈值
                    if (IsPositionSafe(worldPos, minDistance))
                    {
                        safePositions.Add(worldPos);
                    }
                }
            }
        }
    }

    /// <summary>
    /// 生成安全位置列表（默认阈值，无参版本）
    /// </summary>
    private void GenerateSafePositions()
    {
        GenerateSafePositions(1.0f); // 默认使用初始阈值
    }

    /// <summary>
    /// 检查指定位置是否安全（与障碍物距离满足阈值）
    /// </summary>
    /// <param name="position">世界坐标位置</param>
    /// <param name="minDistance">最小安全距离</param>
    /// <returns>是否安全</returns>
    private bool IsPositionSafe(Vector3 position, float minDistance)
    {
        // 简单碰撞检测：检查位置周围是否有障碍物
        Collider[] colliders = Physics.OverlapSphere(position, minDistance);
        foreach (var collider in colliders)
        {
            // 排除自身碰撞体
            if (collider.gameObject != gameObject && collider.CompareTag("Obstacle"))
            {
                return false;
            }
        }
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
    /// 通知船控制器加载新路径
    /// </summary>
    private void NotifyBoatLoadNewPath()
    {
        if (boatController != null)
        {
            boatController.isPathLoaded = false;
            boatController.TryLoadPath();
            Debug.Log("通知BoatController加载新路径");
        }
        else
        {
            Debug.LogWarning("未找到BoatController，无法通知加载新路径");
        }
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

    public override void CollectObservations(VectorSensor sensor)
    {
        // 初始化128维观测数组（避免补全逻辑出错）
        float[] observations = new float[TOTAL_OBSERVATIONS];
        Array.Fill(observations, 0f);
        int obsIndex = 0;

        // ========== 核心优化：无依赖兜底逻辑，确保观测值非全0 ==========
        // 1. 前进速度（归一化）- 必选字段，兜底值0.5
        float forwardSpeed = rb != null ? Vector3.Dot(transform.forward, rb.linearVelocity) : 0f;
        float normalizedSpeed = Mathf.Clamp(forwardSpeed / MaxSpeed, -1f, 1f);
        observations[obsIndex++] = normalizedSpeed != 0 ? normalizedSpeed : 0.1f; // 兜底非0

        // 2. 朝向（归一化）- 必选字段
        float normalizedYaw = ((transform.eulerAngles.y % 360f) / 180f) - 1f;
        observations[obsIndex++] = normalizedYaw;

        // 3. 目标距离（归一化）- 必选字段，兜底值0.5
        float distToTarget = target != null ? Vector3.Distance(transform.position, target.position) : 10f;
        float normalizedDist = Mathf.Clamp01(distToTarget / (Mathf.Max(gridWidth, gridHeight) * 1f));
        observations[obsIndex++] = normalizedDist != 0 ? normalizedDist : 0.5f; // 兜底非0

        // 4. 目标角度（归一化）- 必选字段
        float angleToTarget = target != null ? Vector3.SignedAngle(transform.forward, target.position - transform.position, Vector3.up) : 0f;
        float normalizedAngle = angleToTarget / 180f;
        observations[obsIndex++] = normalizedAngle;

        // 5. 全局路径方向 - 兜底值0
        float normalizedWaypointAngle = 0f;
        if (globalPathfinder != null && globalPathfinder.path != null && globalPathfinder.path.Count > currentWaypointIndex + 1 && gridManager != null)
        {
            Vector3 nextWaypoint = gridManager.GridToWorld(globalPathfinder.path[currentWaypointIndex + 1]);
            float angleToWaypoint = Vector3.SignedAngle(transform.forward, nextWaypoint - transform.position, Vector3.up);
            normalizedWaypointAngle = angleToWaypoint / 180f;
        }
        observations[obsIndex++] = normalizedWaypointAngle;

        // 6. 局部障碍物（11x11=121个）- 兜底全可通行（0）
        Vector2Int agentGridPos = gridManager != null ? gridManager.WorldToGrid(transform.position) : new Vector2Int(gridWidth / 2, gridHeight / 2);
        for (int x = -ViewRange; x <= ViewRange; x++)
        {
            for (int z = -ViewRange; z <= ViewRange; z++)
            {
                if (obsIndex >= TOTAL_OBSERVATIONS) break; // 防止数组越界

                Vector2Int checkPos = new Vector2Int(agentGridPos.x + x, agentGridPos.y + z);
                bool isObstacle = false; // 兜底无障碍物

                // 仅当栅格数据有效时才检测障碍物
                if (gridManager != null && passableGrid != null)
                {
                    isObstacle = checkPos.x < 0 || checkPos.x >= gridWidth || checkPos.y < 0 || checkPos.y >= gridHeight
                        ? true
                        : !passableGrid[checkPos.x, checkPos.y];
                }

                observations[obsIndex++] = isObstacle ? 1f : 0f;
            }
            if (obsIndex >= TOTAL_OBSERVATIONS) break;
        }

        // 7. 动态障碍物速度 - 兜底值0
        if (obsIndex < TOTAL_OBSERVATIONS) observations[obsIndex++] = 0f;
        if (obsIndex < TOTAL_OBSERVATIONS) observations[obsIndex++] = 0f;

        // 8. 补全剩余观测值（确保总数128，且非全0）
        for (int i = obsIndex; i < TOTAL_OBSERVATIONS; i++)
        {
            observations[i] = 0.01f; // 兜底极小非0值，避免全0
        }

        // 最终添加观测值
        sensor.AddObservation(observations);

        // 调试：验证观测值数量
        if (obsIndex > TOTAL_OBSERVATIONS)
        {
            Debug.LogError($"观测值数量超标：实际{obsIndex}个，期望{TOTAL_OBSERVATIONS}个");
        }
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        // 容错：如果回合已结束/核心组件缺失，直接返回
        if (IsEpisodeDone || target == null || gridManager == null || rb == null) return;

        // 初始化默认速度（避免空值）
        if (currentMaxSpeed <= 0)
        {
            currentMaxSpeed = MaxSpeed;
            Debug.LogWarning("currentMaxSpeed未初始化，使用默认值");
        }

        // ========== 核心修改：读取2维连续动作 ==========
        // 动作0：前进/后退速度 (-1~1 归一化)
        float moveForward = Mathf.Clamp(actions.ContinuousActions[0], -1f, 1f);
        // 动作1：转向速度 (-1~1 归一化，-1左转，1右转)
        float turn = Mathf.Clamp(actions.ContinuousActions[1], -1f, 1f);

        // 执行移动逻辑（适配连续动作）
        MoveAgentContinuous(moveForward, turn);

        // 计算奖励（保留原有逻辑）
        CalculateReward();

        // 通知局部规划器（保留原有逻辑）
        GetComponent<USV_LocalPlanner>()?.OnAgentActionReceived(actions);
    }

    /// <summary>
    /// 适配连续动作的移动逻辑（新增函数）
    /// </summary>
    /// <param name="forward">前进/后退指令 (-1~1)</param>
    /// <param name="turn">转向指令 (-1~1)</param>
    void MoveAgentContinuous(float forward, float turn)
    {
        if (rb == null) return;

        // 1. 前进/后退力计算
        float forwardForce = Mathf.Abs(forward) * currentMaxSpeed * 0.8f;
        rb.AddForce(transform.forward * forward * forwardForce, ForceMode.VelocityChange);

        // 速度限制
        if (rb.linearVelocity.magnitude > currentMaxSpeed)
        {
            rb.linearVelocity = rb.linearVelocity.normalized * currentMaxSpeed;
        }

        // 2. 转向力矩计算
        float rotateTorque = turn * MaxAngularSpeed * Mathf.Deg2Rad * Time.fixedDeltaTime;
        rb.AddTorque(Vector3.up * rotateTorque, ForceMode.VelocityChange);

        // 角速度限制
        if (rb.angularVelocity.magnitude > MaxAngularSpeed * Mathf.Deg2Rad)
        {
            rb.angularVelocity = rb.angularVelocity.normalized * MaxAngularSpeed * Mathf.Deg2Rad;
        }
    }

    /// <summary>
    /// 奖励计算逻辑
    /// </summary>
    private void CalculateReward()
    {
        float distToTarget = Vector3.Distance(transform.position, target.position);
        float distanceDelta = lastDistToTarget - distToTarget;

        // 距离缩短奖励
        float proximityFactor = Mathf.Clamp01(1 - (distToTarget / (Mathf.Max(gridWidth, gridHeight) * 1f)));
        float stepReward = distanceDelta * (1 + proximityFactor * targetProximitySmoothing);
        stepReward = Mathf.Clamp(stepReward, minStepPenalty, maxStepReward);
        AddReward(stepReward);

        // 速度稳定性奖励
        float currentSpeed = rb.linearVelocity.magnitude;
        float idealSpeed = MaxSpeed * 0.5f;
        float speedStabilityReward = 0.1f * (1 - Mathf.Abs(currentSpeed - idealSpeed) / idealSpeed);
        AddReward(speedStabilityReward);

        // 碰撞惩罚
        Collider[] colliders = Physics.OverlapSphere(transform.position, 0.5f);
        bool isCollided = false;
        foreach (var collider in colliders)
        {
            if (collider.gameObject != gameObject && collider.CompareTag("Obstacle"))
            {
                isCollided = true;
                break;
            }
        }
        if (isCollided)
        {
            AddReward(-50f);
            EndEpisodeCustom(); // 碰撞后结束回合
            return;
        }

        // ========== 新增：回合终止条件（避免无限循环） ==========
        // 1. 到达目标终止
        if (distToTarget < 1f)
        {
            AddReward(100f); // 到达目标奖励
            EndEpisodeCustom();
            return;
        }

        // 2. 超时终止
        if (Time.time - episodeStartTime > currentMaxEpisodeTime)
        {
            AddReward(-10f); // 超时惩罚
            EndEpisodeCustom();
            return;
        }

        // 更新最后距离
        lastDistToTarget = distToTarget;

        // 调试：输出每步奖励（确认Step循环触发）
        Debug.Log($"Step奖励：基础={stepReward}, 速度稳定性={speedStabilityReward}, 总累计={GetCumulativeReward()}");
    }

    /// <summary>
    /// 执行移动动作
    /// </summary>
    /// <param name="action">动作索引</param>
    void MoveAgent(int action)
    {
        if (rb == null) return;

        float currentSpeed = rb.linearVelocity.magnitude;
        switch (action)
        {
            case 0: // 前进
                float forwardForce = currentSpeed < currentMaxSpeed * 0.8f ? 0.6f : 0.2f;
                rb.AddForce(transform.forward * currentMaxSpeed * forwardForce, ForceMode.VelocityChange);
                if (rb.linearVelocity.magnitude > currentMaxSpeed)
                {
                    rb.linearVelocity = rb.linearVelocity.normalized * currentMaxSpeed;
                }
                break;
            case 1: // 左转
            case 2: // 右转
                float rotateDir = action == 1 ? -1f : 1f;
                rb.AddTorque(Vector3.up * rotateDir * MaxAngularSpeed * Time.fixedDeltaTime, ForceMode.VelocityChange);
                if (rb.angularVelocity.magnitude > MaxAngularSpeed * Mathf.Deg2Rad)
                {
                    rb.angularVelocity = rb.angularVelocity.normalized * MaxAngularSpeed * Mathf.Deg2Rad;
                }
                break;
            case 3: // 减速
                rb.linearVelocity *= 0.9f;
                break;
        }
    }

    /// <summary>
    /// 更新当前路径点索引
    /// </summary>
    private void UpdateWaypointIndex()
    {
        if (globalPathfinder == null || globalPathfinder.path == null ||
            globalPathfinder.path.Count <= currentWaypointIndex) return;

        float distToWaypoint = Vector3.Distance(transform.position,
            gridManager.GridToWorld(globalPathfinder.path[currentWaypointIndex]));

        if (distToWaypoint < 1.5f)
        {
            currentWaypointIndex = Mathf.Min(currentWaypointIndex + 1, globalPathfinder.path.Count - 1);
        }
    }

    /// <summary>
    /// 检查栅格是否可通行
    /// </summary>
    /// <param name="gridPos">栅格坐标</param>
    /// <returns>是否可通行</returns>
    private bool IsPassable(Vector2Int gridPos)
    {
        if (gridPos.x < 0 || gridPos.x >= gridWidth || gridPos.y < 0 || gridPos.y >= gridHeight)
            return false;

        return passableGrid != null && passableGrid[gridPos.x, gridPos.y];
    }

    /// <summary>
    /// 缓存栅格通行性数据
    /// </summary>
    private void CachePassableGrid()
    {
        if (gridManager == null) return;

        gridWidth = gridManager.gridWidth;
        gridHeight = gridManager.gridHeight;
        passableGrid = new bool[gridWidth, gridHeight];

        for (int x = 0; x < gridWidth; x++)
        {
            for (int z = 0; z < gridHeight; z++)
            {
                passableGrid[x, z] = gridManager.IsGridPassable(new Vector2Int(x, z));
            }
        }
    }
}