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
    private string _resetReason = "";
    // 全局路径相关
    private ImprovedAStar globalPathfinder;
    private int currentWaypointIndex = 0;

    // 动态参数
    private float currentMaxSpeed;
    private float currentMaxEpisodeTime;
    private float episodeStartTime;
    private BoatController boatController;

    // 新增：路径完成相关
    private bool isPathCompleted = false;
    private int consecutiveWaypointReached = 0; // 连续到达路径点计数
                                                // 新增：环境初始化标记，防止初始化阶段触发结束逻辑
    private bool isEnvironmentInitializing = false;

    [Header("回合终止参数（关键调整）")]
    [Tooltip("碰撞检测半径（增大以减少误判）")]
    public float collisionCheckRadius = 1.5f;
    [Tooltip("触发碰撞的最小障碍物数量")]
    public int minObstacleCount = 3;
    [Tooltip("目标到达阈值（增大以避免提前终止）")]
    public float targetArriveThreshold = 2.0f;
    [Tooltip("边界检测阈值系数（减小以扩大可航行区域）")]
    public float boundaryThresholdFactor = 0.8f;
    [Tooltip("路径完成判定：连续到达多少个路径点视为路径完成")]
    public int pathCompleteWaypointCount = 3;
    // 新增：路径完成后的终止阈值（独立控制）
    public float pathCompleteTerminateThreshold = 3.0f;

    /// <summary>
    /// 检查当前回合是否结束（自定义标记版，替代原有反射逻辑）
    /// </summary>
    public bool IsEpisodeDone { get; private set; }

    // 新增：防抖+日志控制（核心解决误判和刷屏问题）
    private bool _isTerminating = false; // 终止流程中标记（防止误判）
    private Coroutine _terminateCoroutine; // 终止协程引用
    private float _actionIgnoreDelay = 1.0f; // 延迟标记结束的时间（可调整）
    private static bool _ignoreLogPrinted = false; // 日志仅打印一次标记

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
        StartCoroutine(CheckInitializationStatus());
    }

    /// <summary>
    /// 重置智能体状态
    /// </summary>
    /// <param name="maxSpeed">最大速度</param>
    /// <param name="maxEpisodeTime">最大回合时间</param>
    public void ResetAgentState(float maxSpeed, float maxEpisodeTime)
    {
        currentMaxSpeed = maxSpeed <= 0 ? MaxSpeed : maxSpeed;
        currentMaxEpisodeTime = maxEpisodeTime <= 0 ? 300f : maxEpisodeTime;
        episodeStartTime = Time.time;

        lastDistToTarget = target != null ? Vector3.Distance(transform.position, target.position) : 0;

        // 重置路径完成标记
        isPathCompleted = false;
        consecutiveWaypointReached = 0;
        currentWaypointIndex = 0;
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
        // 重置所有标记（核心：解决回合重置后仍误判的问题）
        IsEpisodeDone = false;
        _isTerminating = false;
        _ignoreLogPrinted = false; // 重置日志标记
        _resetReason = "";

        StartCoroutine(WaitForGridInitThenReset());
    }
    /// <summary>
    /// 校验环境初始化状态，防止卡死
    /// </summary>
    private IEnumerator CheckInitializationStatus()
    {
        yield return new WaitForSeconds(2.0f); // 等待2秒后校验
        if (isEnvironmentInitializing)
        {
            Debug.LogError("环境初始化超时，强制标记为完成");
            isEnvironmentInitializing = false;
            // 兜底重置智能体状态
            ResetAgentState(MaxSpeed, 90f);
        }
    }

    /// <summary>
    /// 等待GridManager初始化完成后执行重置逻辑
    /// </summary>
    private IEnumerator WaitForGridInitThenReset()
    {
        // 标记：环境初始化中，禁止智能体执行动作
        isEnvironmentInitializing = true;

        // 1. 等待GridManager就绪
        while (gridManager == null || !gridManager.IsGridReady())
        {
            Debug.LogWarning("等待 GridManager 初始化...");
            yield return new WaitForSeconds(0.1f);
        }

        // 2. 重新生成安全位置（增加重试机制 + 宽松阈值）
        Vector3 safePos = Vector3.zero;
        int retryCount = 0;
        int maxRetries = 8; // 增加重试次数
        float initialMinDistance = 1.0f;
        float minDistanceStep = 0.15f;
        float currentMinDistance = initialMinDistance;

        while (safePos == Vector3.zero && retryCount < maxRetries)
        {
            GenerateSafePositions(currentMinDistance);
            if (safePositions.Count > 0)
            {
                safePos = safePositions[UnityEngine.Random.Range(0, safePositions.Count)];
                // 确保Y轴高度正确（无人船水面高度）
                safePos.y = 0.4f;
            }
            else
            {
                retryCount++;
                currentMinDistance = Mathf.Max(0.05f, initialMinDistance - (retryCount * minDistanceStep));
                Debug.LogWarning($"安全位置生成失败（第{retryCount}次重试），放宽阈值到: {currentMinDistance}");
                if (retryCount >= 2) CachePassableGrid(); // 重试2次后重新缓存栅格
            }
            if (safePos == Vector3.zero) yield return new WaitForSeconds(0.05f);
        }

        // ========== 强制给Agent赋值初始位置（关键！） ==========
        if (safePos != Vector3.zero)
        {
            transform.position = safePos;
            transform.rotation = Quaternion.identity;
        }
        else
        {
            // 终极兜底：随机生成安全区域内的位置
            safePos = new Vector3(
                UnityEngine.Random.Range(-gridWidth / 2, gridWidth / 2),
                0.4f,
                UnityEngine.Random.Range(-gridHeight / 2, gridHeight / 2)
            );
            transform.position = safePos;
            Debug.LogError($"安全位置全空，强制随机设置初始位置: {safePos}");
        }

        // 3. 执行原有重置逻辑
        if (enableTaskLoop) spawnManager?.Regenerate();
        if (rb != null)
        {
            rb.linearVelocity = Vector3.zero;
            rb.angularVelocity = Vector3.zero;
        }
        currentWaypointIndex = 0;
        consecutiveWaypointReached = 0; // 重置连续路径点计数
        isPathCompleted = false; // 重置路径完成标记
        globalPathfinder?.CalculatePathAfterDelay();
        Invoke(nameof(NotifyBoatLoadNewPath), 0.5f);
        ResetAgentState(MaxSpeed, 90f); // 延长默认回合时间到90秒

        // 新增：初始化完成后延迟0.5秒，确保状态稳定
        yield return new WaitForSeconds(0.5f);

        // 标记：初始化完成，允许智能体执行动作
        isEnvironmentInitializing = false;

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
    /// 延迟终止回合的兜底方法（避免立即终止导致的状态异常）
    /// </summary>
    private void DelayedEndEpisode()
    {
        if (!IsEpisodeDone)
        {
            EndEpisodeCustom();
        }
    }

    /// <summary>
    /// 自定义结束回合方法（修复误判+防抖）
    /// </summary>
    private void EndEpisodeCustom()
    {
        // 双重防护：防止重复终止、防止未到终点误终止
        if (IsEpisodeDone || _isTerminating) return;
        _isTerminating = true;

        // 强制停止物理移动（视觉上立即停下）
        if (rb != null)
        {
            rb.linearVelocity = Vector3.zero;
            rb.angularVelocity = Vector3.zero;
            rb.Sleep(); // 彻底休眠刚体，停止所有物理计算
        }

        // 日志输出（保留原有逻辑）
        float currentDist = Vector3.Distance(transform.position, target.position);
        float elapsedTime = Time.time - episodeStartTime;
        string endReason = "未知";
        if (_resetReason == "collision") endReason = "碰撞障碍物";
        else if (_resetReason == "timeout") endReason = "超时";
        else if (_resetReason == "target") endReason = "到达终点";
        else if (_resetReason == "boundary") endReason = "驶出边界";
        else if (_resetReason == "path_complete") endReason = "路径完成";
        else if (_resetReason == "exception") endReason = "逻辑异常";
        else if (elapsedTime < 0.1f) endReason = "初始化阶段异常触发结束";

        Debug.LogWarning($"=== 回合结束 ===");
        Debug.LogWarning($"原因：{endReason} | 结束距离终点：{currentDist:F2}米");
        Debug.LogWarning($"耗时：{elapsedTime:F2}秒 | 累计奖励：{GetCumulativeReward():F2}");

        // 延迟标记回合结束（核心：避免MLAgents帧同步导致的误判）
        if (_terminateCoroutine != null) StopCoroutine(_terminateCoroutine);
        _terminateCoroutine = StartCoroutine(DelayedMarkEpisodeDone());
    }

    /// <summary>
    /// 延迟标记回合结束（仅在真实终止时执行）
    /// </summary>
    private IEnumerator DelayedMarkEpisodeDone()
    {
        yield return new WaitForSeconds(_actionIgnoreDelay);
        // 最终标记：只有真正满足终止条件才设为true
        IsEpisodeDone = true;
        _isTerminating = false;

        // 严格控制EndEpisode调用时机，避免MLAgents状态异常
        if (!IsEpisodeDone) // 改用自定义的回合状态标记
        {
            EndEpisode();
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
        // 新增：全局异常捕获，避免逻辑错误导致的异常终止
        try
        {

            // 核心优化：1. 防抖 2. 日志仅打印一次 3. 杜绝误判
            if (IsEpisodeDone || isEnvironmentInitializing || _isTerminating)
            {
                // 日志仅首次忽略时打印（彻底解决刷屏）
                if (!_ignoreLogPrinted)
                {
                    string reason = IsEpisodeDone ? "回合已结束" :
                                   _isTerminating ? $"回合终止中（当前距离终点：{Vector3.Distance(transform.position, target.position):F2}米）" : "环境初始化中";
                    Debug.LogWarning($"⚠️ 忽略动作：{reason}");
                    _ignoreLogPrinted = true;
                }
                return;
            }
            // 重置日志标记（回合正常执行时清空）
            _ignoreLogPrinted = false;

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

            // 更新路径点状态
            UpdateWaypointProgress();

            // 计算奖励（保留原有逻辑）
            CalculateReward();

            // 通知局部规划器（保留原有逻辑）
            GetComponent<USV_LocalPlanner>()?.OnAgentActionReceived(actions);
        }
        catch (Exception ex)
        {
            Debug.LogError($"OnActionReceived异常：{ex.Message}\n{ex.StackTrace}");
            // 异常时不直接终止，仅记录并惩罚
            AddReward(-5f);
            _resetReason = "exception";
            // 延迟终止，避免连锁错误
            Invoke(nameof(EndEpisodeCustom), 0.5f);
        }
    }

    /// <summary>
    /// 更新路径点完成进度
    /// </summary>
    private void UpdateWaypointProgress()
    {
        if (globalPathfinder == null || globalPathfinder.path == null || globalPathfinder.path.Count == 0)
            return;

        // 检查是否到达当前路径点
        if (currentWaypointIndex < globalPathfinder.path.Count)
        {
            Vector3 currentWaypointPos = gridManager.GridToWorld(globalPathfinder.path[currentWaypointIndex]);
            float distToWaypoint = Vector3.Distance(transform.position, currentWaypointPos);

            if (distToWaypoint < 3.0f) // 到达路径点的判定阈值
            {
                consecutiveWaypointReached++;
                currentWaypointIndex = Mathf.Min(currentWaypointIndex + 1, globalPathfinder.path.Count - 1);
                Debug.Log($"到达路径点 {currentWaypointIndex}，连续到达数：{consecutiveWaypointReached}");
            }
            else
            {
                // 未到达则重置连续计数
                consecutiveWaypointReached = Mathf.Max(0, consecutiveWaypointReached - 1);
            }

            // 判断路径是否完成
            if (consecutiveWaypointReached >= pathCompleteWaypointCount || currentWaypointIndex >= globalPathfinder.path.Count - 1)
            {
                isPathCompleted = true;
                Debug.Log("路径完成！等待到达最终目标");
            }
        }
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

        // 路径完成奖励
        if (isPathCompleted && !IsEpisodeDone)
        {
            AddReward(50f); // 路径完成奖励
            // 路径完成后不立即终止，仅给予奖励
        }

        // 优化：碰撞检测（大幅放宽条件 + 分层惩罚）
        Collider[] colliders = Physics.OverlapSphere(transform.position, collisionCheckRadius);
        bool isCollided = false;
        int obstacleCount = 0;
        foreach (var collider in colliders)
        {
            if (collider.gameObject != gameObject && collider.CompareTag("Obstacle"))
            {
                // 新增：检测障碍物是否真正接触（而非仅进入检测半径）
                float actualDist = Vector3.Distance(transform.position, collider.transform.position);
                if (actualDist < collisionCheckRadius * 0.7f) // 仅70%半径内算有效碰撞
                {
                    obstacleCount++;
                }
            }
        }
        // 只有接触足够多的障碍物才判定为碰撞（增加容错）
        isCollided = obstacleCount >= minObstacleCount && obstacleCount > 0;

        if (isCollided)
        {
            // 分层惩罚：根据障碍物数量调整惩罚值
            float collisionPenalty = -20f * obstacleCount;
            AddReward(Mathf.Clamp(collisionPenalty, -50f, -10f));
            _resetReason = "collision";
            Debug.Log($"碰撞障碍物（{obstacleCount}个），结束回合！当前位置：{transform.position} | 惩罚：{collisionPenalty}");
            // 延迟终止，确保日志和奖励结算完成
            EndEpisodeCustom();
            return;
        }


        // ========== 优化：回合终止条件 ==========
        // 1. 到达目标终止（增大阈值，增加缓冲）
        if (distToTarget < targetArriveThreshold)
        {
            _resetReason = "target";
            AddReward(200f);
            // 增加延迟终止，避免瞬间重置导致数据丢失
            EndEpisodeCustom();
            return;
        }
        // 接近目标奖励（梯度调整）
        else if (distToTarget < 10f)
        {
            float nearTargetReward = 5f * (1 - distToTarget / 10f);
            AddReward(nearTargetReward);
        }

        // 2. 超时终止（动态延长，避免过早超时）
        float timeoutThreshold = currentMaxEpisodeTime * 2.0f; // 延长100%超时时间
        if (Time.time - episodeStartTime > timeoutThreshold)
        {
            _resetReason = "timeout";
            AddReward(-10f);
            Debug.Log($"回合超时，结束回合！耗时：{Time.time - episodeStartTime}秒 | 阈值：{timeoutThreshold}秒");
            EndEpisodeCustom();
            return;
        }

        // 3. 边界检测（动态扩大可航行区域）
        float maxBoundary = Mathf.Max(gridWidth, gridHeight) * boundaryThresholdFactor * 1.2f; // 扩大20%边界
        if (Mathf.Abs(transform.position.x) > maxBoundary || Mathf.Abs(transform.position.z) > maxBoundary)
        {
            _resetReason = "boundary";
            AddReward(-15f);
            Debug.Log($"驶出边界，结束回合！当前位置：{transform.position}，边界阈值：{maxBoundary}");
            EndEpisodeCustom();
            return;
        }

        // 4. 路径完成后到达目标才终止（修改判定逻辑）
        if (isPathCompleted && distToTarget < pathCompleteTerminateThreshold)
        {
            _resetReason = "path_complete";
            AddReward(100f);
            Debug.Log($"路径完成并接近目标，结束回合！距离目标：{distToTarget}");
            // 延迟终止，确保奖励结算完成
            EndEpisodeCustom();
            return;
        }
        // 新增：路径完成但未到终点，给予持续奖励（鼓励继续靠近）
        else if (isPathCompleted && !IsEpisodeDone)
        {
            float nearTargetBonus = 2f * (1 - distToTarget / (pathCompleteTerminateThreshold * 2));
            AddReward(Mathf.Clamp(nearTargetBonus, 0.5f, 2f));
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