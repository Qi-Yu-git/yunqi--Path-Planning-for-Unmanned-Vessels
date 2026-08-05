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

    private const int TOTAL_OBSERVATIONS = 128;

    [Header("任务循环设置")]
    [Tooltip("是否启用任务自动循环")]
    public bool enableTaskLoop = true;
    [Tooltip("随机生成管理器引用")]
    public RandomSpawnManager spawnManager;

    [Tooltip("目标点Transform")]
    public Transform target;

    [Header("COLREGs 评估器")]
    public COLREGsEvaluator colregsEvaluator;

    // ====================== 核心组件引用 ======================
    private GridManager gridManager;
    private Rigidbody rb;
    private USV_LocalPlanner localPlanner;
    private ImprovedAStar globalPathfinder;
    private BoatController boatController;

    // ====================== 栅格数据 ======================
    private int gridWidth;
    private int gridHeight;
    private bool[,] passableGrid;
    private List<Vector3> safePositions;

    // ====================== 路径与导航 ======================
    private int currentWaypointIndex = 0;
    private bool isPathCompleted = false;
    private int consecutiveWaypointReached = 0;

    // ====================== 动态参数 ======================
    private float currentMaxSpeed;
    private float currentMaxEpisodeTime;
    private float episodeStartTime;
    private float lastDistToTarget;
    private bool isEnvironmentInitializing = false;
    private string _resetReason = "";

    // ====================== 常量和阈值 ======================
    private const int ViewRange = 5;
    private const float MaxSpeed = 2f;
    private const float MaxAngularSpeed = 60f;

    // ====================== 回合控制 ======================
    public bool IsEpisodeDone { get; private set; }
    private bool _isTerminating = false;
    private Coroutine _terminateCoroutine;
    private float _actionIgnoreDelay = 1.0f;
    private static bool _ignoreLogPrinted = false;

    // ====================== 平滑奖励缓存 ======================
    private float lastOmega = 0f;

    // ====================== 好奇心模块缓存 ======================
    private float[] lastObs = new float[128];
    private float[] currentObs = new float[128];
    private float[] curiosityPrediction = new float[128];

    // ====================== 回合终止参数 ======================
    [Header("回合终止参数")]
    [Tooltip("碰撞检测半径")]
    public float collisionCheckRadius = 1.5f;
    [Tooltip("触发碰撞的最小障碍物数量")]
    public int minObstacleCount = 3;
    [Tooltip("目标到达阈值")]
    public float targetArriveThreshold = 2.0f;
    [Tooltip("边界检测阈值系数")]
    public float boundaryThresholdFactor = 0.8f;
    [Tooltip("路径完成判定：连续到达多少个路径点视为路径完成")]
    public int pathCompleteWaypointCount = 3;
    [Tooltip("路径完成后的终止阈值")]
    public float pathCompleteTerminateThreshold = 3.0f;

    // ====================== 配置 ======================
    public float desiredSpeed = 1.2f;
    public float safeNearMissDistance = 3.0f;

    // ====================== SCI 指标 ======================
    private float totalCrossTrackError;
    private float totalHeadingError;
    private int collisionCount;
    private int nearMissCount;
    private float currentEpisodeLength;

    // ====================== 第二梯队 SCI 指标 ======================
    private float totalSpeedTrackingError;
    private float totalControlEffort;
    private float minTimeToCollision;
    private float totalDistanceToGoal;

    // ====================== 奖励分量拆解 ======================
    private float reward_dist;
    private float reward_speed;
    private float reward_heading;
    private float reward_near_target;
    private float reward_collision;
    private float reward_finish;
    private float reward_timeout;
    private float reward_boundary;
    private float reward_path_complete;
    private float reward_curiosity;
    private float reward_colregs;
    private float reward_smooth;

    // ====================== 奖励调试设置 ======================
    [Header("===== 奖励调试设置 =====")]
    [Tooltip("是否启用奖励详细调试")]
    public bool enableRewardDebug = true;
    [Tooltip("调试信息更新频率（帧数间隔）")]
    public int debugUpdateInterval = 30;
    [Tooltip("是否在控制台输出奖励详情")]
    public bool logRewardDetails = false;
    [Tooltip("是否记录奖励历史")]
    public bool recordRewardHistory = true;

    // 奖励历史记录
    private Dictionary<string, List<float>> rewardHistory = new Dictionary<string, List<float>>();
    private int debugFrameCounter = 0;
    private string lastRewardLog = "";

    // 当前帧奖励详情
    private class RewardDebugInfo
    {
        public float distance;
        public float speed;
        public float heading;
        public float nearTarget;
        public float collision;
        public float finish;
        public float timeout;
        public float boundary;
        public float pathComplete;
        public float curiosity;
        public float colregs;
        public float smooth;
        public float total;
        public float stepTime;
        public float distToTarget;
        public float currentSpeed;
        public float elapsedTime;
    }
    private RewardDebugInfo currentRewardDebug = new RewardDebugInfo();

    // ============================================================
    // Unity 生命周期
    // ============================================================

    protected override void Awake()
    {
        base.Awake();

        // 1. 自动添加 Rigidbody
        rb = GetComponent<Rigidbody>();
        if (rb == null)
        {
            rb = gameObject.AddComponent<Rigidbody>();
            rb.maxAngularVelocity = 5f;
            rb.useGravity = false;
            rb.isKinematic = false;
            Debug.LogWarning("自动添加Rigidbody组件");
        }

        // 2. 自动创建默认目标点
        if (target == null)
        {
            GameObject targetObj = new GameObject("Default_Target");
            targetObj.transform.position = new Vector3(10f, 0f, 10f);
            target = targetObj.transform;
            Debug.LogWarning("未指定target，自动创建默认目标点");
        }

        // 3. 获取依赖组件
        boatController = GetComponent<BoatController>();
        gridManager = FindFirstObjectByType<GridManager>();
        globalPathfinder = FindFirstObjectByType<ImprovedAStar>();
        localPlanner = GetComponent<USV_LocalPlanner>();

        // 4. 初始化 COLREGs 评估器
        InitCOLREGsEvaluator();

        // 5. 初始化好奇心预测缓存
        Array.Fill(curiosityPrediction, 0f);
        Array.Fill(lastObs, 0f);
        Array.Fill(currentObs, 0f);

        // 6. GridManager 兜底
        if (gridManager == null)
        {
            Debug.LogError("未找到GridManager组件！使用兜底栅格数据");
            gridWidth = 50;
            gridHeight = 50;
            passableGrid = new bool[gridWidth, gridHeight];
            for (int x = 0; x < gridWidth; x++)
                for (int z = 0; z < gridHeight; z++)
                    passableGrid[x, z] = true;
        }
        else
        {
            StartCoroutine(WaitForGridInit());
        }
        StartCoroutine(CheckInitializationStatus());

        // 初始化奖励调试
        if (enableRewardDebug)
        {
            InitRewardHistory();
        }
    }

    /// <summary>
    /// 初始化 COLREGs 评估器（与 Awake 平级，非嵌套）
    /// </summary>
    private void InitCOLREGsEvaluator()
    {
        if (colregsEvaluator == null)
        {
            colregsEvaluator = GetComponent<COLREGsEvaluator>();
            if (colregsEvaluator == null)
            {
                colregsEvaluator = gameObject.AddComponent<COLREGsEvaluator>();
            }
        }
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

    // ============================================================
    // 回合控制
    // ============================================================

    public override void OnEpisodeBegin()
    {
        IsEpisodeDone = false;
        _isTerminating = false;
        _ignoreLogPrinted = false;
        _resetReason = "";

        episodeStartTime = Time.time;
        lastDistToTarget = target != null ? Vector3.Distance(transform.position, target.position) : 0;
        lastOmega = rb != null ? rb.angularVelocity.y * Mathf.Rad2Deg : 0f;

        // 重置好奇心数组
        Array.Fill(curiosityPrediction, 0f);
        Array.Fill(lastObs, 0f);
        Array.Fill(currentObs, 0f);

        // 重置各项奖励累计值
        reward_dist = 0f;
        reward_speed = 0f;
        reward_heading = 0f;
        reward_near_target = 0f;
        reward_colregs = 0f;
        reward_smooth = 0f;
        reward_curiosity = 0f;
        reward_collision = 0f;
        reward_finish = 0f;
        reward_timeout = 0f;
        reward_boundary = 0f;
        reward_path_complete = 0f;

        // 初始化奖励调试
        if (enableRewardDebug)
        {
            debugFrameCounter = 0;
            Debug.Log("🎯 新回合开始，奖励调试已启用");
        }

        StartCoroutine(WaitForGridInitThenReset());
    }

    private IEnumerator CheckInitializationStatus()
    {
        yield return new WaitForSeconds(2.0f);
        if (isEnvironmentInitializing)
        {
            Debug.LogError("环境初始化超时，强制标记为完成");
            isEnvironmentInitializing = false;
            ResetAgentState(MaxSpeed, 90f);
        }
    }

    /// <summary>
    /// 等待GridManager初始化完成后执行重置逻辑
    /// </summary>
    private IEnumerator WaitForGridInitThenReset()
    {
        isEnvironmentInitializing = true;

        // 1. 等待GridManager就绪
        while (gridManager == null || !gridManager.IsGridReady())
        {
            Debug.LogWarning("等待 GridManager 初始化...");
            yield return new WaitForSeconds(0.1f);
        }

        // 2. 重新生成安全位置
        Vector3 safePos = Vector3.zero;
        int retryCount = 0;
        int maxRetries = 8;
        float initialMinDistance = 1.0f;
        float minDistanceStep = 0.15f;
        float currentMinDistance = initialMinDistance;

        while (safePos == Vector3.zero && retryCount < maxRetries)
        {
            GenerateSafePositions(currentMinDistance);
            if (safePositions.Count > 0)
            {
                safePos = safePositions[UnityEngine.Random.Range(0, safePositions.Count)];
                safePos.y = 0.4f;
            }
            else
            {
                retryCount++;
                currentMinDistance = Mathf.Max(0.05f, initialMinDistance - (retryCount * minDistanceStep));
                Debug.LogWarning($"安全位置生成失败（第{retryCount}次重试），放宽阈值到: {currentMinDistance}");
                if (retryCount >= 2) CachePassableGrid();
            }
            if (safePos == Vector3.zero) yield return new WaitForSeconds(0.05f);
        }

        // 强制给Agent赋值初始位置
        if (safePos != Vector3.zero)
        {
            transform.position = safePos;
            transform.rotation = Quaternion.identity;
        }
        else
        {
            safePos = new Vector3(
                UnityEngine.Random.Range(-gridWidth / 2, gridWidth / 2),
                0.4f,
                UnityEngine.Random.Range(-gridHeight / 2, gridHeight / 2)
            );
            transform.position = safePos;
            Debug.LogError($"安全位置全空，强制随机设置初始位置: {safePos}");
        }

        // 3. 执行重置逻辑
        if (enableTaskLoop) spawnManager?.Regenerate();
        if (rb != null)
        {
            rb.linearVelocity = Vector3.zero;
            rb.angularVelocity = Vector3.zero;
        }
        currentWaypointIndex = 0;
        consecutiveWaypointReached = 0;
        isPathCompleted = false;
        globalPathfinder?.CalculatePathAfterDelay();
        Invoke(nameof(NotifyBoatLoadNewPath), 0.5f);
        ResetAgentState(MaxSpeed, 90f);

        yield return new WaitForSeconds(0.5f);
        isEnvironmentInitializing = false;

        Debug.Log($"智能体重置完成：位置={transform.position}，重试次数={retryCount}");
    }

    // ============================================================
    // 状态管理
    // ============================================================

    /// <summary>
    /// 重置智能体状态
    /// </summary>
    public void ResetAgentState(float maxSpeed, float maxEpisodeTime)
    {
        currentMaxSpeed = maxSpeed <= 0 ? MaxSpeed : maxSpeed;
        currentMaxEpisodeTime = maxEpisodeTime <= 0 ? 300f : maxEpisodeTime;
        episodeStartTime = Time.time;

        lastDistToTarget = target != null ? Vector3.Distance(transform.position, target.position) : 0;
        lastOmega = 0f;
        Array.Fill(curiosityPrediction, 0f);

        isPathCompleted = false;
        consecutiveWaypointReached = 0;
        currentWaypointIndex = 0;

        // 重置全部SCI指标
        totalCrossTrackError = 0;
        totalHeadingError = 0;
        collisionCount = 0;
        nearMissCount = 0;
        currentEpisodeLength = 0;

        totalSpeedTrackingError = 0;
        totalControlEffort = 0;
        minTimeToCollision = 999f;
        totalDistanceToGoal = 0;

        reward_dist = 0;
        reward_speed = 0;
        reward_heading = 0;
        reward_near_target = 0;
        reward_collision = 0;
        reward_finish = 0;
        reward_timeout = 0;
        reward_boundary = 0;
        reward_path_complete = 0;
        reward_curiosity = 0;
        reward_colregs = 0;
        reward_smooth = 0;
    }

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
        GenerateSafePositions();
        Debug.Log("GridManager初始化完成，已缓存通行性数据");
    }

    private void GenerateSafePositions(float minDistance)
    {
        safePositions = new List<Vector3>();

        if (gridManager == null || passableGrid == null) return;

        for (int x = 0; x < gridWidth; x++)
        {
            for (int z = 0; z < gridHeight; z++)
            {
                if (passableGrid[x, z])
                {
                    Vector3 worldPos = gridManager.GridToWorld(new Vector2Int(x, z));
                    if (IsPositionSafe(worldPos, minDistance))
                    {
                        safePositions.Add(worldPos);
                    }
                }
            }
        }
    }

    private void GenerateSafePositions()
    {
        GenerateSafePositions(1.0f);
    }

    private bool IsPositionSafe(Vector3 position, float minDistance)
    {
        Collider[] colliders = Physics.OverlapSphere(position, minDistance);
        foreach (var collider in colliders)
        {
            if (collider.gameObject != gameObject && collider.CompareTag("Obstacle"))
            {
                return false;
            }
        }
        return true;
    }

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

    // ============================================================
    // 回合终止
    // ============================================================

    private void DelayedEndEpisode()
    {
        if (!IsEpisodeDone)
        {
            EndEpisodeCustom();
        }
    }

    private void EndEpisodeCustom()
    {
        if (IsEpisodeDone || _isTerminating) return;
        _isTerminating = true;

        if (rb != null)
        {
            rb.linearVelocity = Vector3.zero;
            rb.angularVelocity = Vector3.zero;
            rb.Sleep();
        }

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

        if (_terminateCoroutine != null) StopCoroutine(_terminateCoroutine);
        _terminateCoroutine = StartCoroutine(DelayedMarkEpisodeDone());

        WriteFinalEpisodeStats();
    }

    private IEnumerator DelayedMarkEpisodeDone()
    {
        yield return new WaitForSeconds(_actionIgnoreDelay);
        IsEpisodeDone = true;
        _isTerminating = false;

        if (!IsEpisodeDone)
        {
            EndEpisode();
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

    // ============================================================
    // ML-Agents 核心方法
    // ============================================================

    public override void CollectObservations(VectorSensor sensor)
    {
        float[] obs = new float[TOTAL_OBSERVATIONS];
        int idx = 0;

        // [0] 纵荡速度 (归一化)
        float surgeVel = rb != null ? Vector3.Dot(transform.forward, rb.linearVelocity) : 0f;
        obs[idx++] = Mathf.Clamp(surgeVel / currentMaxSpeed, -1f, 1f);

        // [1] 转艏角速度 (归一化)
        float omega = rb != null ? rb.angularVelocity.y * Mathf.Rad2Deg : 0f;
        obs[idx++] = Mathf.Clamp(omega / MaxAngularSpeed, -1f, 1f);

        // [2] 航向与目标方位角偏差 (归一化)
        float distToTarget = target != null ? Vector3.Distance(transform.position, target.position) : 10f;
        if (target != null)
        {
            Vector3 toTarget = target.position - transform.position;
            float angleToTarget = Vector3.SignedAngle(transform.forward, toTarget, Vector3.up);
            obs[idx++] = Mathf.Clamp(angleToTarget / 180f, -1f, 1f);
        }
        else
        {
            obs[idx++] = 0f;
        }

        // [3] 到全局路径的横向偏差 (归一化)
        float crossTrackError = 0f;
        if (globalPathfinder != null && globalPathfinder.path != null && globalPathfinder.path.Count > 1 && gridManager != null)
        {
            Vector3 closest = GetClosestPointOnGlobalPath(transform.position);
            crossTrackError = Vector3.Distance(
                new Vector3(transform.position.x, 0, transform.position.z),
                new Vector3(closest.x, 0, closest.z)
            );
            // 判断左右：用路径方向叉乘
            int closestIdx = GetClosestPathIndex(transform.position);
            if (closestIdx < globalPathfinder.path.Count - 1 && closestIdx >= 0)
            {
                Vector3 pathDir = (gridManager.GridToWorld(globalPathfinder.path[Mathf.Min(closestIdx + 1, globalPathfinder.path.Count - 1)]) -
                                  gridManager.GridToWorld(globalPathfinder.path[closestIdx])).normalized;
                Vector3 toBoat = (transform.position - closest).normalized;
                float side = Vector3.Cross(pathDir, toBoat).y;
                crossTrackError *= Mathf.Sign(side);
            }
        }
        obs[idx++] = Mathf.Clamp(crossTrackError / 20f, -1f, 1f);

        // [4-124] 11x11 局部静态栅格 (121个)
        Vector2Int agentGridPos = gridManager != null ? gridManager.WorldToGrid(transform.position) : new Vector2Int(gridWidth / 2, gridHeight / 2);
        for (int x = -ViewRange; x <= ViewRange; x++)
        {
            for (int z = -ViewRange; z <= ViewRange; z++)
            {
                Vector2Int checkPos = new Vector2Int(agentGridPos.x + x, agentGridPos.y + z);
                bool isObstacle = false;

                if (gridManager != null && passableGrid != null)
                {
                    isObstacle = checkPos.x < 0 || checkPos.x >= gridWidth || checkPos.y < 0 || checkPos.y >= gridHeight
                        ? true
                        : !passableGrid[checkPos.x, checkPos.y];
                }

                // 检查动态障碍物（仅在静态无阻挡时检查）
                if (localPlanner != null && !isObstacle)
                {
                    Vector3 worldCheck = gridManager != null ? gridManager.GridToWorld(checkPos) : Vector3.zero;
                    foreach (var obsPos in localPlanner.dynamicObstacles)
                    {
                        if (Vector3.Distance(worldCheck, obsPos) < (gridManager?.gridCellSize ?? 1f))
                        {
                            isObstacle = true;
                            break;
                        }
                    }
                }

                obs[idx++] = isObstacle ? 1f : 0f;
            }
        }

        // [125-126] 最近动态障碍物相对速度 (相对速度 = 障碍物速度 - 本船速度)
        if (localPlanner != null && localPlanner.dynamicObstacles.Count > 0)
        {
            float minDist = float.MaxValue;
            Vector3 relVel = Vector3.zero;
            for (int i = 0; i < localPlanner.dynamicObstacles.Count; i++)
            {
                float d = Vector3.Distance(transform.position, localPlanner.dynamicObstacles[i]);
                if (d < minDist)
                {
                    minDist = d;
                    if (i < localPlanner.dynamicObstacleVelocities.Count)
                    {
                        relVel = localPlanner.dynamicObstacleVelocities[i] - rb.linearVelocity;
                    }
                }
            }
            obs[idx++] = Mathf.Clamp(relVel.x / currentMaxSpeed, -1f, 1f);
            obs[idx++] = Mathf.Clamp(relVel.z / currentMaxSpeed, -1f, 1f);
        }
        else
        {
            obs[idx++] = 0f;
            obs[idx++] = 0f;
        }

        // [127] 剩余距离 (归一化)
        float maxDist = Mathf.Max(gridWidth, gridHeight) * 1f;
        obs[idx++] = Mathf.Clamp01(distToTarget / maxDist);

        // 保存当前观测用于好奇心
        Array.Copy(obs, currentObs, TOTAL_OBSERVATIONS);

        sensor.AddObservation(obs);
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        try
        {
            if (IsEpisodeDone || isEnvironmentInitializing || _isTerminating)
            {
                if (!_ignoreLogPrinted)
                {
                    string reason = IsEpisodeDone ? "回合已结束" :
                                   _isTerminating ? "回合终止中" : "环境初始化中";
                    Debug.LogWarning($"⚠️ 忽略动作：{reason}");
                    _ignoreLogPrinted = true;
                }
                return;
            }
            _ignoreLogPrinted = false;

            if (IsEpisodeDone || target == null || gridManager == null || rb == null) return;

            if (currentMaxSpeed <= 0)
            {
                currentMaxSpeed = MaxSpeed;
                Debug.LogWarning("currentMaxSpeed未初始化，使用默认值");
            }

            // 读取连续动作
            float moveForward = Mathf.Clamp(actions.ContinuousActions[0], -1f, 1f);
            float turn = Mathf.Clamp(actions.ContinuousActions[1], -1f, 1f);

            MoveAgentContinuous(moveForward, turn);
            UpdateWaypointProgress();
            CalculateReward();

            // 更新SCI指标
            UpdateTrackingStats();
            CheckNearMiss();
            currentEpisodeLength += 1f;
            UpdateSecondTierStats(actions);

            // 通知局部规划器
            if (localPlanner != null)
            {
                localPlanner.OnAgentActionReceived(actions);
            }
        }
        catch (Exception ex)
        {
            Debug.LogError($"OnActionReceived异常：{ex.Message}\n{ex.StackTrace}");
            AddReward(-5f);
            _resetReason = "exception";
            Invoke(nameof(EndEpisodeCustom), 0.5f);
        }
    }

    // ============================================================
    // 动作控制
    // ============================================================

    void MoveAgentContinuous(float forward, float turn)
    {
        if (rb == null) return;

        float forwardForce = Mathf.Abs(forward) * currentMaxSpeed * 0.8f;
        rb.AddForce(transform.forward * forward * forwardForce, ForceMode.VelocityChange);

        if (rb.linearVelocity.magnitude > currentMaxSpeed)
        {
            rb.linearVelocity = rb.linearVelocity.normalized * currentMaxSpeed;
        }

        float rotateTorque = turn * MaxAngularSpeed * Mathf.Deg2Rad * Time.fixedDeltaTime;
        rb.AddTorque(Vector3.up * rotateTorque, ForceMode.VelocityChange);

        if (rb.angularVelocity.magnitude > MaxAngularSpeed * Mathf.Deg2Rad)
        {
            rb.angularVelocity = rb.angularVelocity.normalized * MaxAngularSpeed * Mathf.Deg2Rad;
        }
    }

    // ============================================================
    // 路径管理
    // ============================================================

    private void UpdateWaypointProgress()
    {
        if (globalPathfinder == null || globalPathfinder.path == null || globalPathfinder.path.Count == 0)
            return;

        if (currentWaypointIndex < globalPathfinder.path.Count)
        {
            Vector3 currentWaypointPos = gridManager.GridToWorld(globalPathfinder.path[currentWaypointIndex]);
            float distToWaypoint = Vector3.Distance(transform.position, currentWaypointPos);

            if (distToWaypoint < 3.0f)
            {
                consecutiveWaypointReached++;
                currentWaypointIndex = Mathf.Min(currentWaypointIndex + 1, globalPathfinder.path.Count - 1);
            }
            else
            {
                consecutiveWaypointReached = Mathf.Max(0, consecutiveWaypointReached - 1);
            }

            if (consecutiveWaypointReached >= pathCompleteWaypointCount || currentWaypointIndex >= globalPathfinder.path.Count - 1)
            {
                isPathCompleted = true;
            }
        }
    }

    // ============================================================
    // 辅助方法
    // ============================================================

    private Vector3 GetClosestPointOnGlobalPath(Vector3 pos)
    {
        if (globalPathfinder == null || globalPathfinder.path == null || globalPathfinder.path.Count < 2 || gridManager == null)
            return pos;

        Vector3 closest = pos;
        float minDist = float.MaxValue;

        for (int i = 0; i < globalPathfinder.path.Count - 1; i++)
        {
            Vector3 a = gridManager.GridToWorld(globalPathfinder.path[i]);
            Vector3 b = gridManager.GridToWorld(globalPathfinder.path[i + 1]);
            Vector3 proj = ProjectPointOnSegment(pos, a, b);
            float d = Vector3.Distance(pos, proj);
            if (d < minDist)
            {
                minDist = d;
                closest = proj;
            }
        }
        return closest;
    }

    private int GetClosestPathIndex(Vector3 pos)
    {
        if (globalPathfinder == null || globalPathfinder.path == null || globalPathfinder.path.Count == 0 || gridManager == null)
            return 0;

        int idx = 0;
        float minDist = float.MaxValue;
        for (int i = 0; i < globalPathfinder.path.Count; i++)
        {
            float d = Vector3.Distance(pos, gridManager.GridToWorld(globalPathfinder.path[i]));
            if (d < minDist)
            {
                minDist = d;
                idx = i;
            }
        }
        return idx;
    }

    private Vector3 ProjectPointOnSegment(Vector3 p, Vector3 a, Vector3 b)
    {
        Vector3 ab = b - a;
        float t = Vector3.Dot(p - a, ab) / Vector3.Dot(ab, ab);
        t = Mathf.Clamp01(t);
        return Vector3.Lerp(a, b, t);
    }

    /// <summary>
    /// 获取最近动态障碍物的距离（用于动态调整理想速度）
    /// </summary>
    private float GetNearestObstacleDistance()
    {
        if (localPlanner == null || localPlanner.dynamicObstacles == null || localPlanner.dynamicObstacles.Count == 0)
            return float.MaxValue;

        float minDist = float.MaxValue;
        for (int i = 0; i < localPlanner.dynamicObstacles.Count; i++)
        {
            float d = Vector3.Distance(transform.position, localPlanner.dynamicObstacles[i]);
            if (d < minDist) minDist = d;
        }
        return minDist;
    }

    // ============================================================
    // 奖励计算（优化版）
    // ============================================================

    private void CalculateReward()
    {
        float distToTarget = Vector3.Distance(transform.position, target.position);
        float distanceDelta = lastDistToTarget - distToTarget;
        float currentSpeed = rb.linearVelocity.magnitude;
        float elapsedTime = Time.time - episodeStartTime;
        float omega = rb.angularVelocity.y * Mathf.Rad2Deg;

        // 重置当前帧奖励调试数据
        currentRewardDebug.distToTarget = distToTarget;
        currentRewardDebug.currentSpeed = currentSpeed;
        currentRewardDebug.elapsedTime = elapsedTime;
        currentRewardDebug.distance = 0;
        currentRewardDebug.speed = 0;
        currentRewardDebug.heading = 0;
        currentRewardDebug.nearTarget = 0;
        currentRewardDebug.collision = 0;
        currentRewardDebug.finish = 0;
        currentRewardDebug.timeout = 0;
        currentRewardDebug.boundary = 0;
        currentRewardDebug.pathComplete = 0;
        currentRewardDebug.curiosity = 0;
        currentRewardDebug.colregs = 0;
        currentRewardDebug.smooth = 0;

        // ========== [1] 距离缩短奖励（增强权重，让每步都有明显反馈） ==========
        float maxGridDim = Mathf.Max(gridWidth, gridHeight);
        float normalizedDist = Mathf.Clamp01(distToTarget / Mathf.Max(maxGridDim, 1f));
        // 靠近目标时 distanceDelta 的权重更高；远离时惩罚也更有力
        float distanceReward = distanceDelta * (1.5f - normalizedDist * 0.5f) * 0.82f;
        distanceReward = Mathf.Clamp(distanceReward, -0.25f, 0.25f);
        AddReward(distanceReward);
        reward_dist += distanceReward;
        currentRewardDebug.distance = distanceReward;

        if (enableRewardDebug)
        {
            RecordRewardHistory("Distance", distanceReward);
            DebugRewardComponent("距离奖励", distanceReward, -0.25f, 0.25f);
        }

        // ========== [2] 速度稳定性奖励（动态理想速度，避碰时自动降低） ==========
        float nearestObstacleDist = GetNearestObstacleDistance();
        float dynamicIdealSpeed;
        if (nearestObstacleDist < 15f && nearestObstacleDist > 0.01f)
        {
            // 靠近障碍物时鼓励低速（给 COLREGs 让路）
            dynamicIdealSpeed = Mathf.Lerp(currentMaxSpeed * 0.15f, currentMaxSpeed * 0.62f, nearestObstacleDist / 15f);
        }
        else
        {
            dynamicIdealSpeed = currentMaxSpeed * 0.6f;
        }
        float speedError = Mathf.Abs(currentSpeed - dynamicIdealSpeed) / Mathf.Max(dynamicIdealSpeed, 0.1f);
        float speedStabilityReward = 0.06f * (1f - Mathf.Clamp01(speedError));
        AddReward(speedStabilityReward);
        reward_speed += speedStabilityReward;
        currentRewardDebug.speed = speedStabilityReward;

        if (enableRewardDebug)
        {
            RecordRewardHistory("Speed", speedStabilityReward);
            DebugRewardComponent("速度奖励", speedStabilityReward, 0f, 0.03f);
        }

        // ========== [3] 航向合理性奖励 ==========
        Vector3 toTarget = target.position - transform.position;
        toTarget.y = 0f; // 忽略高度差
        float angleToTarget = Vector3.SignedAngle(transform.forward, toTarget, Vector3.up);
        float normalizedAngle = Mathf.Clamp01(Mathf.Abs(angleToTarget) / 180f);
        float headingReward = 0.042f * (1f - normalizedAngle);
        AddReward(headingReward);
        reward_heading += headingReward;
        currentRewardDebug.heading = headingReward;

        if (enableRewardDebug)
        {
            RecordRewardHistory("Heading", headingReward);
            DebugRewardComponent("航向奖励", headingReward, 0f, 0.02f);
        }

        // ========== [4] 接近目标梯度奖励 ==========
        float nearTargetBonus = 0f;
        if (distToTarget < 20f)
        {
            nearTargetBonus = 0.062f * (1f - distToTarget / 20f);
            AddReward(nearTargetBonus);
            reward_near_target += nearTargetBonus;
            currentRewardDebug.nearTarget = nearTargetBonus;

            if (enableRewardDebug)
            {
                RecordRewardHistory("NearTarget", nearTargetBonus);
                DebugRewardComponent("接近目标奖励", nearTargetBonus, 0f, 0.03f);
            }
        }

        // ========== [5] COLREGs 合规奖励（限制范围，避免单步奖励过大） ==========
        if (colregsEvaluator != null && localPlanner != null)
        {
            try
            {
                var obstacles = new List<(Vector3 pos, Vector3 vel)>();
                for (int i = 0; i < localPlanner.dynamicObstacles.Count; i++)
                {
                    Vector3 vel = i < localPlanner.dynamicObstacleVelocities.Count
                        ? localPlanner.dynamicObstacleVelocities[i]
                        : Vector3.zero;
                    obstacles.Add((localPlanner.dynamicObstacles[i], vel));
                }

                float colregsReward = colregsEvaluator.GetReward(
                    transform.position,
                    transform.eulerAngles.y,
                    rb.linearVelocity.magnitude,
                    obstacles
                );
                // 限制单步 COLREGs 奖励范围，让步进奖励之间保持量级一致
                colregsReward = Mathf.Clamp(colregsReward, -0.2f, 0.15f);
                AddReward(colregsReward);
                reward_colregs += colregsReward;
                currentRewardDebug.colregs = colregsReward;

                if (enableRewardDebug)
                {
                    RecordRewardHistory("COLREGs", colregsReward);
                    DebugRewardComponent("COLREGs奖励", colregsReward, -0.2f, 0.15f);
                }
            }
            catch (Exception e)
            {
                Debug.LogWarning($"COLREGs计算异常: {e.Message}");
            }
        }

        // ========== [6] 平滑奖励（加入死区：小幅度转向不惩罚，只惩罚剧烈抖动） ==========
        float omegaDelta = Mathf.Abs(omega - lastOmega);
        if (omegaDelta > 3.0f) // 3度/步的死区（约 90度/秒 @ 30fps）
        {
            float smoothReward = -0.02f * (omegaDelta - 3.0f);
            smoothReward = Mathf.Clamp(smoothReward, -0.15f, 0f);
            AddReward(smoothReward);
            reward_smooth += smoothReward;
            currentRewardDebug.smooth = smoothReward;

            if (enableRewardDebug)
            {
                RecordRewardHistory("Smooth", smoothReward);
                DebugRewardComponent("平滑奖励", smoothReward, -0.15f, 0f);
            }
        }
        lastOmega = omega;

        // ========== [7] 好奇心内在奖励 ==========
        try
        {
            float obsChange = 0f;
            for (int i = 0; i < TOTAL_OBSERVATIONS; i++)
            {
                obsChange += Mathf.Abs(currentObs[i] - curiosityPrediction[i]);
            }
            float curiosityReward = Mathf.Clamp(obsChange * 0.005f, 0f, 0.05f);
            AddReward(curiosityReward * 0.5f);
            reward_curiosity += curiosityReward * 0.5f;
            currentRewardDebug.curiosity = curiosityReward * 0.5f;

            if (enableRewardDebug)
            {
                RecordRewardHistory("Curiosity", curiosityReward * 0.5f);
                DebugRewardComponent("好奇心奖励", curiosityReward * 0.5f, 0f, 0.025f);
            }

            // 更新预测：指数平滑
            for (int i = 0; i < TOTAL_OBSERVATIONS; i++)
            {
                curiosityPrediction[i] = 0.9f * curiosityPrediction[i] + 0.1f * currentObs[i];
            }
            // 保存当前观测供下帧使用
            System.Array.Copy(currentObs, lastObs, TOTAL_OBSERVATIONS);
        }
        catch (Exception e)
        {
            Debug.LogWarning($"好奇心计算异常: {e.Message}");
        }

        // ========== [8] 碰撞检测（明确碰撞区与预警区，降低惩罚量级） ==========
        try
        {
            // 内圈：实际碰撞区（结束回合）；外圈：预警区（影响 COLREGs）
            float collisionInnerRadius = collisionCheckRadius * 0.6f;
            float collisionOuterRadius = collisionCheckRadius;

            Collider[] colliders = Physics.OverlapSphere(transform.position, collisionOuterRadius);
            bool isCollided = false;
            int obstacleCount = 0;

            foreach (var collider in colliders)
            {
                if (collider.gameObject != gameObject && collider.CompareTag("Obstacle"))
                {
                    float actualDist = Vector3.Distance(transform.position, collider.transform.position);
                    if (actualDist < collisionInnerRadius)
                    {
                        obstacleCount++;
                    }
                }
            }
            isCollided = obstacleCount >= minObstacleCount && obstacleCount > 0;

            if (isCollided)
            {
                // 惩罚量级与步进奖励对齐，避免"一撞毁所有"导致中间行为学不到
                float collisionPenalty = -2.0f - (obstacleCount * 0.5f);
                collisionPenalty = Mathf.Clamp(collisionPenalty, -4.0f, -1.0f);
                AddReward(collisionPenalty);
                reward_collision += collisionPenalty;
                currentRewardDebug.collision = collisionPenalty;

                if (enableRewardDebug)
                {
                    RecordRewardHistory("Collision", collisionPenalty);
                    Debug.LogWarning($"💥 碰撞检测：{obstacleCount}个障碍物，惩罚：{collisionPenalty:F2}");
                    Debug.LogWarning($"   位置：{transform.position}，碰撞半径：{collisionCheckRadius}");
                }

                collisionCount++;
                _resetReason = "collision";
                UpdateTotalRewardDebug();
                EndEpisodeCustom();
                return;
            }
        }
        catch (Exception e)
        {
            Debug.LogWarning($"碰撞检测异常: {e.Message}");
        }

        // ========== [9] 目标达成（降低终局奖励绝对值，配合 gamma 折算更合理） ==========
        if (distToTarget < targetArriveThreshold)
        {
            // 时间奖励：越快越好，但上限不超过 3.0
            float timeBonus = Mathf.Max(0f, (currentMaxEpisodeTime - elapsedTime) / Mathf.Max(currentMaxEpisodeTime, 1f));
            float finishReward = 2.0f + timeBonus; // 范围 [2.0, 3.0]
            AddReward(finishReward);
            reward_finish += finishReward;
            currentRewardDebug.finish = finishReward;

            if (enableRewardDebug)
            {
                RecordRewardHistory("Finish", finishReward);
                Debug.Log($"🏁 到达目标！奖励：{finishReward:F2}，用时：{elapsedTime:F1}s");
            }

            _resetReason = "target";
            UpdateTotalRewardDebug();
            EndEpisodeCustom();
            return;
        }

        // ========== [10] 超时（去除 *2.0f，严格按设定时间执行） ==========
        if (elapsedTime > currentMaxEpisodeTime)
        {
            float timeoutPenalty = -1.0f;
            AddReward(timeoutPenalty);
            reward_timeout += timeoutPenalty;
            currentRewardDebug.timeout = timeoutPenalty;

            if (enableRewardDebug)
            {
                RecordRewardHistory("Timeout", timeoutPenalty);
                Debug.Log($"⏰ 超时！惩罚：{timeoutPenalty}，用时：{elapsedTime:F1}s");
            }

            _resetReason = "timeout";
            UpdateTotalRewardDebug();
            EndEpisodeCustom();
            return;
        }

        // ========== [11] 驶出边界（降低惩罚，与超时同量级） ==========
        float maxBoundary = maxGridDim * boundaryThresholdFactor;
        if (Mathf.Abs(transform.position.x) > maxBoundary || Mathf.Abs(transform.position.z) > maxBoundary)
        {
            float boundaryPenalty = -1.5f;
            AddReward(boundaryPenalty);
            reward_boundary += boundaryPenalty;
            currentRewardDebug.boundary = boundaryPenalty;

            if (enableRewardDebug)
            {
                RecordRewardHistory("Boundary", boundaryPenalty);
                Debug.Log($"🚫 驶出边界！惩罚：{boundaryPenalty}，位置：({transform.position.x:F1}, {transform.position.z:F1})");
            }

            _resetReason = "boundary";
            UpdateTotalRewardDebug();
            EndEpisodeCustom();
            return;
        }

        // ========== [12] 路径完成奖励（与目标达成解耦，避免重复触发） ==========
        // 仅在未到达最终目标时生效，作为中间 waypoint 奖励
        if (isPathCompleted && !IsEpisodeDone && distToTarget >= targetArriveThreshold)
        {
            float pathProgressBonus = 0.5f * (1f - Mathf.Clamp01(distToTarget / Mathf.Max(pathCompleteTerminateThreshold * 2f, 1f)));
            pathProgressBonus = Mathf.Clamp(pathProgressBonus, 0.1f, 0.5f);
            AddReward(pathProgressBonus);
            reward_path_complete += pathProgressBonus;
            currentRewardDebug.pathComplete = pathProgressBonus;

            if (enableRewardDebug)
            {
                RecordRewardHistory("PathComplete", pathProgressBonus);
                DebugRewardComponent("路径完成奖励", pathProgressBonus, 0.1f, 0.5f);
            }

            // 如果路径完成且已非常接近目标，给予中等奖励（但不与 [9] 的到达奖励叠加）
            if (distToTarget < pathCompleteTerminateThreshold && distToTarget >= targetArriveThreshold)
            {
                float pathFinishBonus = 1.5f;
                AddReward(pathFinishBonus);
                reward_path_complete += pathFinishBonus;
                currentRewardDebug.pathComplete = pathFinishBonus;

                if (enableRewardDebug)
                {
                    RecordRewardHistory("PathComplete", pathFinishBonus);
                    Debug.Log($"🛤️  路径完成并接近目标，奖励：{pathFinishBonus} | 累计奖励：{GetCumulativeReward()}");
                }

                _resetReason = "path_complete_near";
                // 注意：这里不 EndEpisode，让智能体继续尝试到达最终目标以获得更高奖励
            }
        }

        lastDistToTarget = distToTarget;

        // 更新总奖励调试
        UpdateTotalRewardDebug();
    }

    /// <summary>
    /// 更新总奖励调试信息
    /// </summary>
    private void UpdateTotalRewardDebug()
    {
        currentRewardDebug.total = currentRewardDebug.distance +
                                  currentRewardDebug.speed +
                                  currentRewardDebug.heading +
                                  currentRewardDebug.nearTarget +
                                  currentRewardDebug.collision +
                                  currentRewardDebug.finish +
                                  currentRewardDebug.timeout +
                                  currentRewardDebug.boundary +
                                  currentRewardDebug.pathComplete +
                                  currentRewardDebug.curiosity +
                                  currentRewardDebug.colregs +
                                  currentRewardDebug.smooth;

        if (enableRewardDebug)
        {
            RecordRewardHistory("Total", currentRewardDebug.total);
            UpdateRewardDebugDisplay();
        }
    }

    // ============================================================
    // 奖励调试方法
    // ============================================================

    /// <summary>
    /// 初始化奖励历史记录
    /// </summary>
    private void InitRewardHistory()
    {
        if (!recordRewardHistory) return;

        string[] rewardNames = new string[]
        {
            "Distance", "Speed", "Heading", "NearTarget", "Collision",
            "Finish", "Timeout", "Boundary", "PathComplete",
            "Curiosity", "COLREGs", "Smooth", "Total"
        };

        foreach (string name in rewardNames)
        {
            if (!rewardHistory.ContainsKey(name))
                rewardHistory[name] = new List<float>();
        }
    }

    /// <summary>
    /// 记录奖励到历史
    /// </summary>
    private void RecordRewardHistory(string name, float value)
    {
        if (!recordRewardHistory || !rewardHistory.ContainsKey(name)) return;

        rewardHistory[name].Add(value);

        // 限制历史长度防止内存溢出
        if (rewardHistory[name].Count > 10000)
            rewardHistory[name].RemoveAt(0);
    }

    /// <summary>
    /// 获取奖励统计信息
    /// </summary>
    private string GetRewardStats(string name)
    {
        if (!rewardHistory.ContainsKey(name) || rewardHistory[name].Count == 0)
            return "No data";

        var list = rewardHistory[name];
        float avg = 0;
        float min = float.MaxValue;
        float max = float.MinValue;
        float sum = 0;

        foreach (float v in list)
        {
            sum += v;
            if (v < min) min = v;
            if (v > max) max = v;
        }
        avg = sum / list.Count;

        return $"Avg:{avg:F3} Min:{min:F3} Max:{max:F3} Count:{list.Count}";
    }

    /// <summary>
    /// 调试单个奖励分量
    /// </summary>
    private void DebugRewardComponent(string name, float value, float min, float max)
    {
        if (!enableRewardDebug || !logRewardDetails) return;

        string icon = value >= 0 ? "🟢" : "🔴";
        string range = value < min ? "⚠️低于范围" : value > max ? "⚠️超出范围" : "✅";
        Debug.Log($"{icon} {name}: {value:F4} [{min:F2}, {max:F2}] {range}");
    }

    /// <summary>
    /// 更新调试显示
    /// </summary>
    private void UpdateRewardDebugDisplay()
    {
        debugFrameCounter++;
        if (debugFrameCounter % debugUpdateInterval != 0) return;

        // 构建调试字符串
        System.Text.StringBuilder sb = new System.Text.StringBuilder();
        sb.AppendLine("╔══════════════════════════════════════════════════════════╗");
        sb.AppendLine($"║  📊 奖励调试面板 - 帧 {Time.frameCount}                    ║");
        sb.AppendLine("╠══════════════════════════════════════════════════════════╣");

        // 当前状态
        sb.AppendLine($"║  位置: ({transform.position.x:F2}, {transform.position.z:F2})         ║");
        sb.AppendLine($"║  速度: {currentRewardDebug.currentSpeed:F2} m/s                    ║");
        sb.AppendLine($"║  距目标: {currentRewardDebug.distToTarget:F2}m                    ║");
        sb.AppendLine($"║  运行时间: {currentRewardDebug.elapsedTime:F1}s                    ║");
        sb.AppendLine("╠══════════════════════════════════════════════════════════╣");

        // 奖励分量
        sb.AppendLine($"║  🎯 距离奖励:     {currentRewardDebug.distance,10:F4}   ║");
        sb.AppendLine($"║  🏃 速度奖励:     {currentRewardDebug.speed,10:F4}   ║");
        sb.AppendLine($"║  🧭 航向奖励:     {currentRewardDebug.heading,10:F4}   ║");
        sb.AppendLine($"║  📍 接近目标:     {currentRewardDebug.nearTarget,10:F4}   ║");
        sb.AppendLine($"║  💥 碰撞惩罚:     {currentRewardDebug.collision,10:F4}   ║");
        sb.AppendLine($"║  🏁 完成奖励:     {currentRewardDebug.finish,10:F4}   ║");
        sb.AppendLine($"║  ⏰ 超时惩罚:     {currentRewardDebug.timeout,10:F4}   ║");
        sb.AppendLine($"║  🚫 边界惩罚:     {currentRewardDebug.boundary,10:F4}   ║");
        sb.AppendLine($"║  🛤️  路径完成:     {currentRewardDebug.pathComplete,10:F4}   ║");
        sb.AppendLine($"║  🔮 好奇心奖励:   {currentRewardDebug.curiosity,10:F4}   ║");
        sb.AppendLine($"║  ⚓ COLREGs:       {currentRewardDebug.colregs,10:F4}   ║");
        sb.AppendLine($"║  ✨ 平滑奖励:     {currentRewardDebug.smooth,10:F4}   ║");
        sb.AppendLine("╠══════════════════════════════════════════════════════════╣");
        sb.AppendLine($"║  💰 总奖励:       {currentRewardDebug.total,10:F4}   ║");
        sb.AppendLine($"║  📈 累计奖励:     {GetCumulativeReward(),10:F2}   ║");
        sb.AppendLine("╠══════════════════════════════════════════════════════════╣");

        // 统计信息（如果有历史）
        if (recordRewardHistory && rewardHistory.Count > 0)
        {
            sb.AppendLine("║  📊 历史统计 (最近)                                  ║");
            sb.AppendLine($"║  距离: {GetRewardStats("Distance")}      ║");
            sb.AppendLine($"║  速度: {GetRewardStats("Speed")}        ║");
            sb.AppendLine($"║  航向: {GetRewardStats("Heading")}       ║");
            sb.AppendLine($"║  总奖励: {GetRewardStats("Total")}        ║");
        }

        sb.AppendLine("╚══════════════════════════════════════════════════════════╝");

        // 输出到控制台或Unity的Debug
        if (logRewardDetails)
        {
            Debug.Log(sb.ToString());
        }
        else
        {
            // 只显示简化的单行信息在Inspector中
            lastRewardLog = $"Reward: {currentRewardDebug.total:F4} | Dist: {currentRewardDebug.distToTarget:F2}m | " +
                           $"Speed: {currentRewardDebug.currentSpeed:F2}m/s | " +
                           $"Dist:{currentRewardDebug.distance:F3} Col:{currentRewardDebug.collision:F3}";
        }
    }

    // ============================================================
    // SCI 指标统计
    // ============================================================

    private void UpdateTrackingStats()
    {
        if (target == null || gridManager == null || globalPathfinder == null ||
            globalPathfinder.path == null || globalPathfinder.path.Count == 0)
            return;

        Vector3 wpPos = gridManager.GridToWorld(globalPathfinder.path[Mathf.Min(currentWaypointIndex, globalPathfinder.path.Count - 1)]);
        float cte = Vector3.Distance(transform.position, wpPos);
        totalCrossTrackError += cte;

        float desiredYaw = Vector3.SignedAngle(Vector3.forward, wpPos - transform.position, Vector3.up);
        float currYaw = transform.eulerAngles.y;
        float headingErr = Mathf.Abs(Mathf.DeltaAngle(currYaw, desiredYaw));
        totalHeadingError += headingErr;
    }

    private void CheckNearMiss()
    {
        Collider[] cols = Physics.OverlapSphere(transform.position, safeNearMissDistance);
        foreach (var c in cols)
        {
            if (c.gameObject != gameObject && c.CompareTag("Obstacle"))
            {
                nearMissCount++;
                break;
            }
        }
    }

    private void UpdateSecondTierStats(ActionBuffers actions)
    {
        float currentSpeed = rb.linearVelocity.magnitude;
        totalSpeedTrackingError += Mathf.Abs(currentSpeed - desiredSpeed);

        float effort = (Mathf.Abs(actions.ContinuousActions[0]) + Mathf.Abs(actions.ContinuousActions[1])) / 2f;
        totalControlEffort += effort;

        totalDistanceToGoal += Vector3.Distance(transform.position, target.position);

        float ttc = CalculateTTC();
        if (ttc < minTimeToCollision) minTimeToCollision = ttc;
    }

    private float CalculateTTC()
    {
        Collider[] cols = Physics.OverlapSphere(transform.position, 20f);
        float minT = 999f;

        foreach (var c in cols)
        {
            if (c.gameObject == gameObject || !c.CompareTag("Obstacle")) continue;

            Vector3 deltaPos = c.transform.position - transform.position;
            deltaPos.y = 0;
            float dist = deltaPos.magnitude;
            if (dist < 1f) continue;

            Rigidbody orb = c.GetComponent<Rigidbody>();
            float relSpeed = orb != null ? orb.linearVelocity.magnitude : 0.1f;
            relSpeed += rb.linearVelocity.magnitude;
            if (relSpeed < 0.1f) relSpeed = 0.1f;

            float t = dist / relSpeed;
            if (t < minT) minT = t;
        }
        return minT;
    }

    // ============================================================
    // 回合结束统计写入
    // ============================================================

    private void WriteFinalEpisodeStats()
    {
        if (currentEpisodeLength <= 0) return;

        float avgCTE = totalCrossTrackError / currentEpisodeLength;
        float avgHdg = totalHeadingError / currentEpisodeLength;
        float avgSpeedErr = totalSpeedTrackingError / currentEpisodeLength;
        float avgEffort = totalControlEffort / currentEpisodeLength;
        float avgGoalDist = totalDistanceToGoal / currentEpisodeLength;

        // USV 性能指标
        Academy.Instance.StatsRecorder.Add("USV/CrossTrackError", avgCTE, StatAggregationMethod.Average);
        Academy.Instance.StatsRecorder.Add("USV/HeadingError", avgHdg, StatAggregationMethod.Average);
        Academy.Instance.StatsRecorder.Add("USV/CollisionCount", collisionCount, StatAggregationMethod.Sum);
        Academy.Instance.StatsRecorder.Add("USV/NearMissCount", nearMissCount, StatAggregationMethod.Sum);
        Academy.Instance.StatsRecorder.Add("USV/EpisodeLength", currentEpisodeLength, StatAggregationMethod.Average);
        Academy.Instance.StatsRecorder.Add("USV/SpeedTrackingError", avgSpeedErr, StatAggregationMethod.Average);
        Academy.Instance.StatsRecorder.Add("USV/ControlEffort", avgEffort, StatAggregationMethod.Average);
        Academy.Instance.StatsRecorder.Add("USV/MinTTC", minTimeToCollision, StatAggregationMethod.Average);
        Academy.Instance.StatsRecorder.Add("USV/DistanceToGoal", avgGoalDist, StatAggregationMethod.Average);

        // 奖励分量
        Academy.Instance.StatsRecorder.Add("Reward/Distance", reward_dist, StatAggregationMethod.Sum);
        Academy.Instance.StatsRecorder.Add("Reward/Speed", reward_speed, StatAggregationMethod.Sum);
        Academy.Instance.StatsRecorder.Add("Reward/Heading", reward_heading, StatAggregationMethod.Sum);
        Academy.Instance.StatsRecorder.Add("Reward/NearTarget", reward_near_target, StatAggregationMethod.Sum);
        Academy.Instance.StatsRecorder.Add("Reward/Collision", reward_collision, StatAggregationMethod.Sum);
        Academy.Instance.StatsRecorder.Add("Reward/Finish", reward_finish, StatAggregationMethod.Sum);
        Academy.Instance.StatsRecorder.Add("Reward/Timeout", reward_timeout, StatAggregationMethod.Sum);
        Academy.Instance.StatsRecorder.Add("Reward/Boundary", reward_boundary, StatAggregationMethod.Sum);
        Academy.Instance.StatsRecorder.Add("Reward/PathComplete", reward_path_complete, StatAggregationMethod.Sum);
        Academy.Instance.StatsRecorder.Add("Reward/COLREGs", reward_colregs, StatAggregationMethod.Sum);
        Academy.Instance.StatsRecorder.Add("Reward/Curiosity", reward_curiosity, StatAggregationMethod.Sum);
        Academy.Instance.StatsRecorder.Add("Reward/Smooth", reward_smooth, StatAggregationMethod.Sum);

        // COLREGs 合规评分
        if (colregsEvaluator != null && target != null)
        {
            var emptyObstacles = new List<(Vector3, Vector3)>();
            var (score, state) = colregsEvaluator.Evaluate(
                transform.position,
                transform.eulerAngles.y,
                rb.linearVelocity.magnitude,
                emptyObstacles
            );
            Academy.Instance.StatsRecorder.Add("COLREGs/ComplianceScore", score, StatAggregationMethod.Average);
            Academy.Instance.StatsRecorder.Add("COLREGs/State", (float)state, StatAggregationMethod.Average);
        }
    }

#if UNITY_EDITOR
    // 在Scene视图显示调试信息
    private void OnDrawGizmosSelected()
    {
        if (!enableRewardDebug || !Application.isPlaying) return;

        // 在Scene视图显示奖励信息
        Vector3 labelPos = transform.position + Vector3.up * 4f;
        string label = $"━━━━━━━━━━━━━━━━━━━━━━━━━━\n" +
                       $"  💰 总奖励: {currentRewardDebug.total:F4}\n" +
                       $"  🎯 距离: {currentRewardDebug.distance:F4}\n" +
                       $"  🏃 速度: {currentRewardDebug.speed:F4}\n" +
                       $"  🧭 航向: {currentRewardDebug.heading:F4}\n" +
                       $"  💥 碰撞: {currentRewardDebug.collision:F4}\n" +
                       $"  🏁 完成: {currentRewardDebug.finish:F4}\n" +
                       $"  ⚓ COLREGs: {currentRewardDebug.colregs:F4}\n" +
                       $"  📍 距目标: {currentRewardDebug.distToTarget:F2}m\n" +
                       $"  🚀 速度: {currentRewardDebug.currentSpeed:F2}m/s\n" +
                       $"━━━━━━━━━━━━━━━━━━━━━━━━━━";

        UnityEditor.Handles.Label(labelPos, label);
    }
#endif
}