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
/// 修复: 控制器冲突、奖励收敛稳定性
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

    // ====================== 多目标权重配置（修正版）======================
    private const float W_DISTANCE = 0.35f;    // 降低"冲向目标"的权重
    private const float W_CROSSTRACK = 0.5f;   // 大幅提高路径跟踪
    private const float W_COLREGS = 0.6f;     // 提高避碰规则权重
    private const float W_SAFETY = 1.0f;      // 降低安全惩罚，释放奖励预算
    private const float W_HEADING = 0.4f;       // 回滚
    private const float W_SMOOTH = 0.15f;       // 不变
    private const float W_TIME = 0.2f;          // 从 0.4 降低到 0.2

    // 安全通过追踪 (已废弃)
    //  private float safePassTimer = 0f;
    private const float SAFE_PASS_THRESHOLD_TTC = 5.0f;
    private const float SAFE_PASS_THRESHOLD_DIST = 5.0f;

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
    private float _actionIgnoreDelay = 0.01f;
    private static bool _ignoreLogPrinted = false;
    private bool pathCompletionRewarded = false;

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
    public float targetArriveThreshold = 1.2f;
    [Tooltip("边界检测阈值系数")]
    public float boundaryThresholdFactor = 0.8f;
    [Tooltip("路径完成判定：连续到达多少个路径点视为路径完成")]
    public int pathCompleteWaypointCount = 3;
    [Tooltip("路径完成后的终止阈值")]
    public float pathCompleteTerminateThreshold = 3.0f;

    // ====================== 配置 ======================
    public float desiredSpeed = 1.2f;
    public float safeNearMissDistance = 4.0f;

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
    private float reward_nearMiss;
    private float reward_safe_pass;
    private float reward_time;
    private float lastCollisionTime = -999f;
    private const float COLLISION_COOLDOWN = 3.0f;

    private Vector3 _lastFramePosition;
    private Coroutine _resetCoroutine;

    // ====================== 奖励平滑与跟踪缓存 ======================
    private float smoothedDistanceDelta = 0f;
    private float currentCrossTrackError = 0f;
    private float currentAngleToTarget = 0f;

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

    private Dictionary<string, List<float>> rewardHistory = new Dictionary<string, List<float>>();
    private int debugFrameCounter = 0;
    private string lastRewardLog = "";

    private float lastActionForward = 0f;
    private float lastActionTurn = 0f;

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
        public float nearMiss;
        public float safePass;
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
        pathCompletionRewarded = false;

        episodeStartTime = Time.time;
        lastDistToTarget = target != null ? Vector3.Distance(transform.position, target.position) : 0;
        lastOmega = rb != null ? rb.angularVelocity.y * Mathf.Rad2Deg : 0f;

        Array.Fill(curiosityPrediction, 0f);
        Array.Fill(lastObs, 0f);
        Array.Fill(currentObs, 0f);

        reward_dist = 0f; reward_speed = 0f; reward_heading = 0f;
        reward_near_target = 0f; reward_colregs = 0f; reward_smooth = 0f;
        reward_curiosity = 0f; reward_collision = 0f; reward_finish = 0f;
        reward_timeout = 0f; reward_boundary = 0f; reward_path_complete = 0f;
        reward_nearMiss = 0f;
        reward_safe_pass = 0f;
        reward_time = 0f;
        lastActionForward = 0f;
        lastActionTurn = 0f;

        if (enableRewardDebug)
        {
            debugFrameCounter = 0;
            Debug.Log("🎯 新回合开始，奖励调试已启用");
        }

        if (_resetCoroutine != null) StopCoroutine(_resetCoroutine);
        _resetCoroutine = StartCoroutine(WaitForGridInitThenReset());
    }

    private IEnumerator WaitForGridInitThenReset()
    {
        isEnvironmentInitializing = true;
        Debug.Log("🔄 开始智能体重置流程...");

        float gridWaitTimeout = 5.0f;
        float gridWaitTimer = 0f;
        while ((gridManager == null || !gridManager.IsGridReady()) && gridWaitTimer < gridWaitTimeout)
        {
            Debug.LogWarning($"等待 GridManager 初始化... ({gridWaitTimer:F1}s)");
            yield return new WaitForSeconds(0.1f);
            gridWaitTimer += 0.1f;
        }

        if (gridManager == null || !gridManager.IsGridReady())
        {
            Debug.LogError("❌ GridManager 初始化超时，使用默认位置");
            transform.position = new Vector3(0, 0.4f, 0);
            transform.rotation = Quaternion.identity;
            isEnvironmentInitializing = false;
            yield break;
        }

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

        if (safePos != Vector3.zero)
        {
            transform.position = safePos;
            transform.rotation = Quaternion.identity;
            Debug.Log($"✅ 智能体移动到安全位置: {safePos}");
        }
        else
        {
            safePos = new Vector3(
                UnityEngine.Random.Range(-gridWidth / 4, gridWidth / 4),
                0.4f,
                UnityEngine.Random.Range(-gridHeight / 4, gridHeight / 4)
            );
            transform.position = safePos;
            Debug.LogError($"⚠️ 安全位置全空，强制设置初始位置: {safePos}");
        }

        if (rb != null)
        {
            rb.linearVelocity = Vector3.zero;
            rb.angularVelocity = Vector3.zero;
            rb.Sleep();
        }

        currentWaypointIndex = 0;
        consecutiveWaypointReached = 0;
        isPathCompleted = false;
        pathCompletionRewarded = false;

        if (enableTaskLoop && spawnManager != null)
        {
            spawnManager.Regenerate();
            yield return new WaitForSeconds(0.1f);
        }

        // 锁定最大时长300秒，防止早期环境时间限制过短
        ResetAgentState(MaxSpeed, 300f);

        if (globalPathfinder != null)
        {
            Debug.Log("🔄 触发路径重新计算...");
            globalPathfinder.CalculatePathAfterDelay();

            float pathWaitTimeout = 5.0f;
            float pathWaitTimer = 0f;
            bool pathReady = false;

            while (pathWaitTimer < pathWaitTimeout)
            {
                if (globalPathfinder.path != null && globalPathfinder.path.Count >= 2)
                {
                    pathReady = true;
                    Debug.Log($"✅ 路径计算完成，路径点数量: {globalPathfinder.path.Count}");
                    break;
                }
                yield return new WaitForSeconds(0.05f);
                pathWaitTimer += 0.05f;
            }

            if (!pathReady)
            {
                Debug.LogWarning("⚠️ 路径计算超时，尝试重新触发");
                globalPathfinder.CalculatePathAfterDelay();
                yield return new WaitForSeconds(0.5f);

                if (globalPathfinder.path != null && globalPathfinder.path.Count >= 2)
                {
                    pathReady = true;
                    Debug.Log($"✅ 二次路径计算完成，路径点数量: {globalPathfinder.path.Count}");
                }
            }

            if (pathReady && gridManager != null)
            {
                Vector3 pathStartWorld = gridManager.GridToWorld(globalPathfinder.path[0]);
                pathStartWorld.y = 0.4f;

                if (IsPositionSafe(pathStartWorld, 0.5f))
                {
                    transform.position = pathStartWorld;
                    Debug.Log($"🎯 智能体对齐到路径起点: {pathStartWorld}");

                    if (globalPathfinder.path.Count > 1)
                    {
                        Vector3 pathSecond = gridManager.GridToWorld(globalPathfinder.path[1]);
                        pathSecond.y = 0.4f;
                        Vector3 direction = (pathSecond - pathStartWorld).normalized;
                        if (direction.sqrMagnitude > 0.001f)
                        {
                            transform.rotation = Quaternion.LookRotation(direction);
                        }
                    }
                }
                else
                {
                    Debug.LogWarning($"⚠️ 路径起点 {pathStartWorld} 不安全，保持当前位置: {transform.position}");
                }
            }
            else
            {
                Debug.LogWarning("⚠️ 路径不可用，保持当前位置");
            }
        }

        yield return null;
        Vector3 fixedPosition = transform.position;
        transform.position = fixedPosition;
        Debug.Log($"🔒 最终锁定位置: {fixedPosition}");

        if (boatController != null)
        {
            boatController.isPathLoaded = false;
            boatController.TryLoadPath();
            Debug.Log("📢 通知 BoatController 加载新路径");
        }

        isEnvironmentInitializing = false;
        IsEpisodeDone = false;
        _isTerminating = false;
        _ignoreLogPrinted = false;

        lastDistToTarget = target != null ? Vector3.Distance(transform.position, target.position) : 0;
        lastOmega = rb != null ? rb.angularVelocity.y * Mathf.Rad2Deg : 0f;
        episodeStartTime = Time.time;

        Debug.Log($"✅ 智能体重置完成！位置={transform.position}，目标={target?.position}");
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

    // ============================================================
    // 状态管理
    // ============================================================

    public void ResetAgentState(float maxSpeed, float maxEpisodeTime)
    {
        currentMaxSpeed = maxSpeed <= 0 ? MaxSpeed : maxSpeed;
        currentMaxEpisodeTime = maxEpisodeTime <= 0 ? 150f : maxEpisodeTime;
        episodeStartTime = Time.time;

        lastDistToTarget = target != null ? Vector3.Distance(transform.position, target.position) : 0;
        lastOmega = 0f;
        Array.Fill(curiosityPrediction, 0f);

        isPathCompleted = false;
        consecutiveWaypointReached = 0;
        currentWaypointIndex = 0;

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
        reward_nearMiss = 0;
        reward_safe_pass = 0;
        reward_time = 0;
        smoothedDistanceDelta = 0f;
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

        if (IsEpisodeDone)
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

        float SafeNormalize(float value, float maxValue, float defaultValue = 0f)
        {
            float absMax = Mathf.Abs(maxValue);
            if (absMax < 0.0001f)
            {
                return defaultValue;
            }
            return Mathf.Clamp(value / absMax, -1f, 1f);
        }

        float surgeVel = rb != null ? Vector3.Dot(transform.forward, rb.linearVelocity) : 0f;
        obs[idx++] = SafeNormalize(surgeVel, currentMaxSpeed);

        float omega = rb != null ? rb.angularVelocity.y * Mathf.Rad2Deg : 0f;
        obs[idx++] = SafeNormalize(omega, MaxAngularSpeed);

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

        float crossTrackError = 0f;
        if (globalPathfinder != null && globalPathfinder.path != null && globalPathfinder.path.Count > 1 && gridManager != null)
        {
            Vector3 closest = GetClosestPointOnGlobalPath(transform.position);
            crossTrackError = Vector3.Distance(
                new Vector3(transform.position.x, 0, transform.position.z),
                new Vector3(closest.x, 0, closest.z)
            );
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
            obs[idx++] = SafeNormalize(relVel.x, currentMaxSpeed);
            obs[idx++] = SafeNormalize(relVel.z, currentMaxSpeed);
        }
        else
        {
            obs[idx++] = 0f;
            obs[idx++] = 0f;
        }

        float maxDist = Mathf.Max(gridWidth, gridHeight) * 1f;
        obs[idx++] = Mathf.Clamp01(distToTarget / maxDist);

        if (idx != TOTAL_OBSERVATIONS)
        {
            Debug.LogError($"观测数量不匹配! 期望: {TOTAL_OBSERVATIONS}, 实际: {idx}");
        }

        Array.Copy(obs, currentObs, TOTAL_OBSERVATIONS);

        for (int i = 0; i < obs.Length; i++)
        {
            if (float.IsNaN(obs[i]) || float.IsInfinity(obs[i]))
            {
                Debug.LogError($"NaN/Infinity detected in obs[{i}] = {obs[i]}");
                obs[i] = 0f;
            }
            sensor.AddObservation(obs[i]);
        }
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        try
        {
            if (isEnvironmentInitializing)
            {
                if (!_ignoreLogPrinted)
                {
                    Debug.LogWarning("⚠️ 环境初始化中，忽略动作");
                    _ignoreLogPrinted = true;
                }
                return;
            }
            _ignoreLogPrinted = false;

            if (IsEpisodeDone || _isTerminating) return;

            float maxBoundary = Mathf.Max(gridWidth, gridHeight) * 0.8f;
            if (Mathf.Abs(transform.position.x) > maxBoundary || Mathf.Abs(transform.position.z) > maxBoundary)
            {
                Debug.LogWarning($"⚠️ 智能体位置超出边界: {transform.position}，强制重置");
                _resetReason = "boundary";
                EndEpisodeCustom();
                return;
            }

            if (target == null || gridManager == null || rb == null) return;
            if (currentMaxSpeed <= 0)
            {
                currentMaxSpeed = MaxSpeed;
                Debug.LogWarning("currentMaxSpeed未初始化，使用默认值");
            }

            float moveForward = Mathf.Clamp(actions.ContinuousActions[0], -1f, 1f);
            float turn = Mathf.Clamp(actions.ContinuousActions[1], -1f, 1f);

            MoveAgentContinuous(moveForward, turn);
            UpdateWaypointProgress();

            if (CalculateReward())
                return;

            float effort = (Mathf.Abs(actions.ContinuousActions[0]) + Mathf.Abs(actions.ContinuousActions[1])) / 2f;
            float effortPenalty = -0.015f * effort;
            AddReward(effortPenalty);
            totalControlEffort += effort;

            float actionDelta = Mathf.Abs(moveForward - lastActionForward) + Mathf.Abs(turn - lastActionTurn);
            float actionSmoothPenalty = -0.015f * actionDelta;
            float actionContinuityBonus = (actionDelta < 0.05f) ? 0.015f : 0f;
            float totalActionSmooth = actionSmoothPenalty + actionContinuityBonus;
            AddReward(totalActionSmooth);
            reward_smooth += totalActionSmooth;

            lastActionForward = moveForward;
            lastActionTurn = turn;

            UpdateTrackingStats();
            CheckNearMiss();
            currentEpisodeLength += 1f;
            UpdateSecondTierStats(actions);

            if (localPlanner != null)
                localPlanner.OnAgentActionReceived(actions);
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
    // 动作控制 (改为指令传递，避免物理冲突)
    // ============================================================
    void MoveAgentContinuous(float forward, float turn)
    {
        if (rb == null) return;

        if (boatController != null)
        {
            // 修复点：让 BoatController 的物理系统接管动作
            // 直接用 transform 控制转向，刚体控制线速度，避免BoatController干预
            float targetSpeed = Mathf.Abs(forward) * currentMaxSpeed;
            float targetTurnAngle = turn * 45f; // 最大转向 45 度/秒

            rb.linearVelocity = transform.forward * targetSpeed;
            transform.Rotate(Vector3.up, targetTurnAngle * Time.fixedDeltaTime);
        }
        else
        {
            // 兜底：如果 boatController 为空，使用原始加力控制
            float forwardForce = forward * currentMaxSpeed * 0.8f;
            rb.AddForce(transform.forward * forwardForce, ForceMode.VelocityChange);

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
    // 奖励计算（锚定 365 分专版）
    // ============================================================
    private bool CalculateReward()
    {
        float distToTarget = Vector3.Distance(transform.position, target.position);
        float distanceDelta = lastDistToTarget - distToTarget;
        float currentSpeed = rb.linearVelocity.magnitude;
        float elapsedTime = Time.time - episodeStartTime;
        float omega = rb.angularVelocity.y * Mathf.Rad2Deg;

        float maxGridDim = Mathf.Max((float)gridWidth, (float)gridHeight);
        float safeMaxDist = Mathf.Max(maxGridDim, 1f);
        float normalizedDist = Mathf.Clamp01(distToTarget / safeMaxDist);
        float nearestObsDist = GetNearestObstacleDistance();

        Vector3 toTarget = target.position - transform.position;
        toTarget.y = 0f;
        float cosToTarget = toTarget.sqrMagnitude > 0.001f
            ? Vector3.Dot(transform.forward, toTarget.normalized)
            : 0f;

        float r_dist = 0f, r_heading = 0f, r_colregs = 0f;
        float r_collision = 0f, r_time = 0f;
        float r_boundarySoft = 0f, r_smooth = 0f;
        float r_crossTrack = 0f;
        float r_speed = 0f;  // ← 新增

        // ========== [1] 距离奖励 ==========
        smoothedDistanceDelta = 0.7f * smoothedDistanceDelta + 0.3f * distanceDelta;
        float distBonus = smoothedDistanceDelta * 0.6f;
        float potentialReward = 0.15f * (1f - normalizedDist);
        float directionBonus = 0.2f * Mathf.Max(0f, cosToTarget);
        float proximityMalus = (nearestObsDist < 5f) ? -0.05f * (5f - nearestObsDist) : 0f;

        r_dist = Mathf.Clamp(distBonus + potentialReward + directionBonus + proximityMalus, -0.2f, 0.15f);

        // ========== [1.5] 速度奖励（新增） ==========
        float speedErr = Mathf.Abs(currentSpeed - desiredSpeed);
        r_speed = -0.06f * speedErr;
        if (cosToTarget > 0.7f && speedErr < 0.3f)
            r_speed += 0.12f;

        // ========== [2] 航向奖励 ==========
        float angleToTarget = Vector3.SignedAngle(transform.forward, toTarget, Vector3.up);
        currentAngleToTarget = angleToTarget;
        float normalizedAngle = Mathf.Clamp01(Mathf.Abs(angleToTarget) / 180f);

        if (distToTarget > 3.0f)
            r_heading = 0.4f * (1f - normalizedAngle);
        else
            r_heading = 0.1f * (1f - normalizedAngle);

        // ========== [3] 横向偏差 ==========
        if (globalPathfinder != null && globalPathfinder.path != null
            && globalPathfinder.path.Count > 1 && gridManager != null)
        {
            Vector3 closest = GetClosestPointOnGlobalPath(transform.position);
            float cte = Vector3.Distance(
                new Vector3(transform.position.x, 0, transform.position.z),
                new Vector3(closest.x, 0, closest.z)
            );
            currentCrossTrackError = cte;
            float cteNormalized = Mathf.Clamp01(cte / 10f);
            r_crossTrack = Mathf.Clamp(-1.5f * cteNormalized - 1.5f * cteNormalized * cteNormalized, -1.5f, 0f);
        }

        // ========== [4] COLREGs ==========
        var colregsObstacles = GetColregsObstacles();
        if (colregsEvaluator != null && colregsObstacles.Count > 0)
        {
            try
            {
                float rawColregsReward = colregsEvaluator.GetReward(
                    transform.position,
                    transform.eulerAngles.y,
                    rb.linearVelocity.magnitude,
                    colregsObstacles);
                r_colregs = Mathf.Clamp(rawColregsReward * 0.6f, -0.3f, 0.1f);
            }
            catch (Exception e)
            {
                Debug.LogWarning($"COLREGs计算异常: {e.Message}");
            }
        }

        // ========== [5] 平滑奖励 ==========
        float omegaDelta = Mathf.Abs(omega - lastOmega);
        r_smooth = -0.015f * omegaDelta;
        lastOmega = omega;

        // ========== [6] 碰撞 / TTC / 安全距离 ==========
        float collisionInnerRadius = collisionCheckRadius * 0.6f;
        float nearMissRadius = collisionCheckRadius * 2.0f;
        float detectionRadius = Mathf.Max(nearMissRadius, 10.0f);
        Collider[] colliders = Physics.OverlapSphere(transform.position, detectionRadius);

        bool isCollided = false;
        foreach (var col in colliders)
        {
            if (col.gameObject == gameObject) continue;
            if (!col.CompareTag("Obstacle") && !col.CompareTag("DynamicObstacle")
                && !col.CompareTag("Ship") && !col.CompareTag("USV"))
                continue;
            float actualDist = Vector3.Distance(transform.position, col.transform.position);
            if (actualDist < collisionInnerRadius)
            {
                isCollided = true;
                break;
            }
        }

        if (isCollided && (elapsedTime - lastCollisionTime) > COLLISION_COOLDOWN && elapsedTime >= 1.0f)
        {
            r_collision = -15.0f;
            collisionCount++;
            lastCollisionTime = elapsedTime;
            AddReward(r_collision);
            reward_collision += r_collision;
            _resetReason = "collision";
            EndEpisodeCustom();
            return true;
        }

        float ttc = CalculateTTCDirectional();
        if (ttc < 5.0f && ttc > 0.01f)
        {
            float ttcPenalty;
            if (ttc < 2.0f)
                ttcPenalty = -0.2f * (3.0f - ttc);
            else if (ttc < 3.0f)
                ttcPenalty = -0.05f * (3.0f - ttc);
            else
                ttcPenalty = -0.015f * (5.0f - ttc);
            r_collision += Mathf.Clamp(ttcPenalty, -0.6f, 0f);
        }

        if (nearestObsDist < safeNearMissDistance && nearestObsDist > collisionInnerRadius && nearestObsDist > 0.01f)
        {
            float nearMissPenalty = -0.8f * (safeNearMissDistance - nearestObsDist)
                                    * (1f + currentSpeed / MaxSpeed);
            r_collision += Mathf.Clamp(nearMissPenalty, -2.0f, 0f);
            reward_nearMiss += nearMissPenalty;
        }

        // ========== [7] 时间惩罚 ==========
        r_time = -0.015f;

        // ========== [8] 边界软惩罚 ==========
        float hardBoundary = maxGridDim * boundaryThresholdFactor;
        float softBoundary = hardBoundary * 0.85f;
        float distToEdgeX = hardBoundary - Mathf.Abs(transform.position.x);
        float distToEdgeZ = hardBoundary - Mathf.Abs(transform.position.z);
        float minDistToEdge = Mathf.Min(distToEdgeX, distToEdgeZ);
        float softMargin = hardBoundary - softBoundary;
        if (minDistToEdge < softMargin && minDistToEdge > 0f)
            r_boundarySoft = -0.04f * (1f - minDistToEdge / softMargin);

        // ========== [9] 多目标加权汇总（加入速度项） ==========
        float totalReward =
            W_DISTANCE * r_dist +
            W_HEADING * r_heading +
            W_CROSSTRACK * r_crossTrack +
            W_COLREGS * r_colregs +
            W_SMOOTH * r_smooth +
            W_SAFETY * r_collision +
            W_TIME * r_time +
            0.4f * r_speed +        // ← 新增
            r_boundarySoft;

        totalReward = Mathf.Clamp(totalReward, -1.0f, 1.5f);
        AddReward(totalReward);

        // 累加所有奖励分量
        reward_dist += r_dist;
        reward_heading += r_heading;
        reward_speed += r_speed;        // ← 从 0f 改为 r_speed
        reward_near_target += 0f;
        reward_colregs += r_colregs;
        reward_collision += r_collision;
        reward_time += r_time;
        reward_boundary += r_boundarySoft;
        reward_smooth += r_smooth;

        // ========== 终止条件：完成奖励（提高基线） ==========
        if (distToTarget < targetArriveThreshold)
        {
            float precisionBonus = Mathf.Max(0f, 20f * (1f - distToTarget / targetArriveThreshold));
            float finishReward = Mathf.Clamp(
                260.0f + (currentMaxEpisodeTime - elapsedTime) * 0.15f + precisionBonus,
                240f, 320f);
            AddReward(finishReward);
            reward_finish += finishReward;
            _resetReason = "target";
            EndEpisodeCustom();
            return true;
        }

        // ========== 超时终止 ==========
        if (elapsedTime > currentMaxEpisodeTime)
        {
            float timeoutPenalty = -5.0f;
            AddReward(timeoutPenalty);
            reward_timeout += timeoutPenalty;
            _resetReason = "timeout";
            EndEpisodeCustom();
            return true;
        }

        // ========== 边界硬终止 ==========
        if (Mathf.Abs(transform.position.x) > hardBoundary || Mathf.Abs(transform.position.z) > hardBoundary)
        {
            float boundaryPenalty = -5.0f;
            AddReward(boundaryPenalty);
            reward_boundary += boundaryPenalty;
            _resetReason = "boundary";
            EndEpisodeCustom();
            return true;
        }

        // ========== 路径完成奖励 ==========
        if (isPathCompleted && !pathCompletionRewarded && distToTarget >= targetArriveThreshold)
        {
            float pathProgressBonus = 30.0f;
            AddReward(pathProgressBonus);
            reward_path_complete += pathProgressBonus;
            pathCompletionRewarded = true;
        }

        // ========== 更新状态 ==========
        lastDistToTarget = distToTarget;

        // ========== 调试信息 ==========
        if (enableRewardDebug)
        {
            currentRewardDebug.distance = r_dist;
            currentRewardDebug.speed = r_speed;  // ← 从 0f 改为 r_speed
            currentRewardDebug.heading = r_heading;
            currentRewardDebug.nearTarget = 0f;
            currentRewardDebug.colregs = r_colregs;
            currentRewardDebug.smooth = r_smooth;
            currentRewardDebug.curiosity = 0f;
            currentRewardDebug.collision = r_collision;
            currentRewardDebug.nearMiss = (nearestObsDist < safeNearMissDistance && nearestObsDist > collisionInnerRadius)
                ? -0.3f * (safeNearMissDistance - nearestObsDist) : 0f;
            currentRewardDebug.safePass = 0f;
            currentRewardDebug.total = totalReward;
            currentRewardDebug.distToTarget = distToTarget;
            currentRewardDebug.currentSpeed = currentSpeed;
            currentRewardDebug.elapsedTime = elapsedTime;
            UpdateTotalRewardDebug();
        }

        return false;
    }

    private void UpdateTotalRewardDebug()
    {
        currentRewardDebug.total = currentRewardDebug.distance +
                                  currentRewardDebug.speed +
                                  currentRewardDebug.heading +
                                  currentRewardDebug.nearTarget +
                                  currentRewardDebug.collision +
                                  currentRewardDebug.nearMiss +
                                  currentRewardDebug.safePass +
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

    private void InitRewardHistory()
    {
        if (!recordRewardHistory) return;

        string[] rewardNames = new string[]
        {
            "Distance", "Speed", "Heading", "NearTarget", "Collision",
            "Finish", "Timeout", "Boundary", "PathComplete",
            "Curiosity", "COLREGs", "Smooth", "NearMiss", "SafePass", "Time", "Total"
        };

        foreach (string name in rewardNames)
        {
            if (!rewardHistory.ContainsKey(name))
                rewardHistory[name] = new List<float>();
        }
    }

    private void RecordRewardHistory(string name, float value)
    {
        if (!recordRewardHistory || !rewardHistory.ContainsKey(name)) return;

        rewardHistory[name].Add(value);

        if (rewardHistory[name].Count > 10000)
            rewardHistory[name].RemoveAt(0);
    }

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

    private void UpdateRewardDebugDisplay()
    {
        debugFrameCounter++;
        if (debugFrameCounter % debugUpdateInterval != 0) return;

        System.Text.StringBuilder sb = new System.Text.StringBuilder();
        sb.AppendLine("╔══════════════════════════════════════════════════════════╗");
        sb.AppendLine($"║  📊 奖励调试面板 - 帧 {Time.frameCount}                    ║");
        sb.AppendLine("╠══════════════════════════════════════════════════════════╣");

        sb.AppendLine($"║  位置: ({transform.position.x:F2}, {transform.position.z:F2})         ║");
        sb.AppendLine($"║  速度: {currentRewardDebug.currentSpeed:F2} m/s                    ║");
        sb.AppendLine($"║  距目标: {currentRewardDebug.distToTarget:F2}m                    ║");
        sb.AppendLine($"║  运行时间: {currentRewardDebug.elapsedTime:F1}s                    ║");
        sb.AppendLine("╠══════════════════════════════════════════════════════════╣");

        sb.AppendLine($"║  🎯 距离奖励:     {currentRewardDebug.distance,10:F4}   ║");
        sb.AppendLine($"║  🏃 速度奖励:     {currentRewardDebug.speed,10:F4}   ║");
        sb.AppendLine($"║  🧭 航向奖励:     {currentRewardDebug.heading,10:F4}   ║");
        sb.AppendLine($"║  📍 接近目标:     {currentRewardDebug.nearTarget,10:F4}   ║");
        sb.AppendLine($"║  💥 碰撞惩罚:     {currentRewardDebug.collision,10:F4}   ║");
        sb.AppendLine($"║  ⚠️ 擦边惩罚:     {currentRewardDebug.nearMiss,10:F4}   ║");
        sb.AppendLine($"║  ✅ 安全通过:     {currentRewardDebug.safePass,10:F4}   ║");
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

        if (recordRewardHistory && rewardHistory.Count > 0)
        {
            sb.AppendLine("║  📊 历史统计 (最近)                                  ║");
            sb.AppendLine($"║  距离: {GetRewardStats("Distance")}      ║");
            sb.AppendLine($"║  速度: {GetRewardStats("Speed")}        ║");
            sb.AppendLine($"║  航向: {GetRewardStats("Heading")}       ║");
            sb.AppendLine($"║  总奖励: {GetRewardStats("Total")}        ║");
        }

        sb.AppendLine("╚══════════════════════════════════════════════════════════╝");

        if (logRewardDetails)
        {
            Debug.Log(sb.ToString());
        }
        else
        {
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

        float ttc = CalculateTTCDirectional(); 
        if (ttc < minTimeToCollision) minTimeToCollision = ttc;
    }

    private float CalculateTTCDirectional()
    {
        Collider[] cols = Physics.OverlapSphere(transform.position, 20f);
        float minT = 999f;

        foreach (var c in cols)
        {
            if (c.gameObject == gameObject) continue;
            if (!c.CompareTag("Obstacle") && !c.CompareTag("DynamicObstacle") && !c.CompareTag("Ship"))
                continue;

            Vector3 deltaPos = c.transform.position - transform.position;
            deltaPos.y = 0;
            float dist = deltaPos.magnitude;
            if (dist < 0.5f) return 0f;

            Rigidbody orb = c.GetComponent<Rigidbody>();
            Vector3 otherVel = orb != null ? orb.linearVelocity : Vector3.zero;
            Vector3 relVel = otherVel - rb.linearVelocity;

            float velDotPos = Vector3.Dot(relVel, deltaPos);
            if (velDotPos >= 0) continue;

            float relSpeed = relVel.magnitude;
            if (relSpeed < 0.1f) continue;

            float t = dist / relSpeed;
            if (t < minT) minT = t;
        }
        return minT;
    }

    // ============================================================
    // 回合结束统计写入
    // ============================================================

    // ============================================================
    // 回合结束统计写入
    // ============================================================

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

        // [USV] 指标
        Academy.Instance.StatsRecorder.Add("USV/CrossTrackError", avgCTE, StatAggregationMethod.Average);
        Academy.Instance.StatsRecorder.Add("USV/HeadingError", avgHdg, StatAggregationMethod.Average);
        Academy.Instance.StatsRecorder.Add("USV/EpisodeLength", currentEpisodeLength, StatAggregationMethod.Average);
        Academy.Instance.StatsRecorder.Add("USV/SpeedTrackingError", avgSpeedErr, StatAggregationMethod.Average);
        Academy.Instance.StatsRecorder.Add("USV/ControlEffort", avgEffort, StatAggregationMethod.Average);
        Academy.Instance.StatsRecorder.Add("USV/MinTTC", minTimeToCollision, StatAggregationMethod.Average);
        Academy.Instance.StatsRecorder.Add("USV/DistanceToGoal", avgGoalDist, StatAggregationMethod.Average);
        Academy.Instance.StatsRecorder.Add("USV/CollisionCount", collisionCount, StatAggregationMethod.Sum);
        Academy.Instance.StatsRecorder.Add("USV/NearMissCount", nearMissCount, StatAggregationMethod.Sum);

        // 奖励分量（Average）
        Academy.Instance.StatsRecorder.Add("Reward/Time", reward_time, StatAggregationMethod.Average);
        Academy.Instance.StatsRecorder.Add("Reward/Distance", reward_dist, StatAggregationMethod.Average);
        Academy.Instance.StatsRecorder.Add("Reward/Speed", reward_speed, StatAggregationMethod.Average);
        Academy.Instance.StatsRecorder.Add("Reward/Heading", reward_heading, StatAggregationMethod.Average);
        Academy.Instance.StatsRecorder.Add("Reward/NearTarget", reward_near_target, StatAggregationMethod.Average);
        Academy.Instance.StatsRecorder.Add("Reward/Collision", reward_collision, StatAggregationMethod.Average);
        Academy.Instance.StatsRecorder.Add("Reward/Finish", reward_finish, StatAggregationMethod.Average);
        Academy.Instance.StatsRecorder.Add("Reward/Timeout", reward_timeout, StatAggregationMethod.Average);
        Academy.Instance.StatsRecorder.Add("Reward/Boundary", reward_boundary, StatAggregationMethod.Average);
        Academy.Instance.StatsRecorder.Add("Reward/PathComplete", reward_path_complete, StatAggregationMethod.Average);
        Academy.Instance.StatsRecorder.Add("Reward/COLREGs", reward_colregs, StatAggregationMethod.Average);
        Academy.Instance.StatsRecorder.Add("Reward/Curiosity", reward_curiosity, StatAggregationMethod.Average);
        Academy.Instance.StatsRecorder.Add("Reward/Smooth", reward_smooth, StatAggregationMethod.Average);
        Academy.Instance.StatsRecorder.Add("Reward/NearMiss", reward_nearMiss, StatAggregationMethod.Average);
        Academy.Instance.StatsRecorder.Add("Reward/SafePass", reward_safe_pass, StatAggregationMethod.Average);

        // ✅ 核心：回合平均累计奖励
        Academy.Instance.StatsRecorder.Add("Environment/Cumulative Reward", GetCumulativeReward(), StatAggregationMethod.Average);

        // ✅ 核心：单步平均质量（Average）
        Academy.Instance.StatsRecorder.Add("Policy/PerStepReward", GetCumulativeReward() / currentEpisodeLength, StatAggregationMethod.Average);

        // COLREGs 合规统计
        if (colregsEvaluator != null && target != null)
        {
            var obstacles = GetColregsObstacles();

            if (obstacles.Count == 0 && enableRewardDebug)
                Debug.LogWarning("[EpisodeEnd] COLREGs obstacles list is EMPTY!");

            var result = colregsEvaluator.Evaluate(
                transform.position,
                transform.eulerAngles.y,
                rb.linearVelocity.magnitude,
                obstacles
            );

            Academy.Instance.StatsRecorder.Add("COLREGs/ComplianceScore", result.Item1, StatAggregationMethod.Average);
            Academy.Instance.StatsRecorder.Add("COLREGs/State", (float)(int)result.Item2, StatAggregationMethod.Average);
        }
    }

    private List<(Vector3 pos, Vector3 vel)> ScanDynamicObstacles(float radius = 25f)
    {
        var obstacles = new List<(Vector3 pos, Vector3 vel)>();
        Collider[] cols = Physics.OverlapSphere(transform.position, radius);
        foreach (var col in cols)
        {
            if (col.gameObject == gameObject) continue;
            if (!col.CompareTag("DynamicObstacle") && !col.CompareTag("Ship") && !col.CompareTag("USV"))
                continue;

            Rigidbody orb = col.GetComponent<Rigidbody>();
            Vector3 vel = orb != null ? orb.linearVelocity : Vector3.zero;
            obstacles.Add((col.transform.position, vel));
        }
        return obstacles;
    }

    private List<(Vector3 pos, Vector3 vel)> GetColregsObstacles()
    {
        if (localPlanner != null && localPlanner.dynamicObstacles != null && localPlanner.dynamicObstacles.Count > 0)
        {
            var list = new List<(Vector3 pos, Vector3 vel)>();
            for (int i = 0; i < localPlanner.dynamicObstacles.Count; i++)
            {
                Vector3 vel = i < localPlanner.dynamicObstacleVelocities.Count
                    ? localPlanner.dynamicObstacleVelocities[i] : Vector3.zero;
                list.Add((localPlanner.dynamicObstacles[i], vel));
            }
            return list;
        }
        return ScanDynamicObstacles(25f);
    }

    private void OnDrawGizmosSelected()
    {
        if (!enableRewardDebug || !Application.isPlaying) return;

        Vector3 labelPos = transform.position + Vector3.up * 4f;
        string label = $"━━━━━━━━━━━━━━━━━━━━━━━━━━\n" +
                       $"  💰 总奖励: {currentRewardDebug.total:F4}\n" +
                       $"  🎯 距离: {currentRewardDebug.distance:F4}\n" +
                       $"  🏃 速度: {currentRewardDebug.speed:F4}\n" +
                       $"  🧭 航向: {currentRewardDebug.heading:F4}\n" +
                       $"  💥 碰撞: {currentRewardDebug.collision:F4}\n" +
                       $"  ⚠️ 擦边: {currentRewardDebug.nearMiss:F4}\n" +
                       $"  🏁 完成: {currentRewardDebug.finish:F4}\n" +
                       $"  ⚓ COLREGs: {currentRewardDebug.colregs:F4}\n" +
                       $"  📍 距目标: {currentRewardDebug.distToTarget:F2}m\n" +
                       $"  🚀 速度: {currentRewardDebug.currentSpeed:F2}m/s\n" +
                       $"━━━━━━━━━━━━━━━━━━━━━━━━━━";

        UnityEditor.Handles.Label(labelPos, label);
    }
}