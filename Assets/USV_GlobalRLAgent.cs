using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;
using USVGridSystem;

/// <summary>
/// 无人船全局强化学习智能体（修复版 v3）
/// 修复：权重调优、Waypoint中程奖励、碰撞分层、统计补漏、死代码清理
/// </summary>
public class USV_GlobalRLAgent : Agent
{
    [Header("目标接近参数")]
    public float targetProximitySmoothing = 0.3f;
    public float maxStepReward = 2f;
    public float minStepPenalty = -1f;

    private const int TOTAL_OBSERVATIONS = 128;

    [Header("任务循环设置")]
    public bool enableTaskLoop = true;
    public RandomSpawnManager spawnManager;
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
    public int currentWaypointIndex = 0;
    private int consecutiveWaypointReached = 0;

    // ====================== 多目标权重配置（修复版 v5）======================
    private const float W_DISTANCE = 0.8f;
    private const float W_HEADING = 0.6f;
    private const float W_CROSSTRACK = 0.8f;
    private const float W_COLREGS = 0.2f;
    private const float W_SAFETY = 0.2f;
    private const float W_SMOOTH = 0.1f;
    private const float W_TIME = 0.05f;
    private const float W_WAYPOINT = 0.3f;

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

    // ====================== 平滑奖励缓存 ======================
    private float lastOmega = 0f;

    // ====================== 回合终止参数 ======================
    [Header("回合终止参数")]
    public float collisionCheckRadius = 1.5f;
    public float targetArriveThreshold = 1.2f;
    public float boundaryThresholdFactor = 0.8f;
    public int pathCompleteWaypointCount = 3;

    // ====================== 配置 ======================
    public float desiredSpeed = 1.2f;
    public float safeNearMissDistance = 4.0f;

    // ====================== SCI 指标 ======================
    private float totalCrossTrackError;
    private float totalHeadingError;
    private int collisionCount;
    private float currentEpisodeLength;

    // ====================== 第二梯队 SCI 指标 ======================
    private float totalSpeedTrackingError;
    private float totalControlEffort;
    private float minTimeToCollision;
    private float totalDistanceToGoal;

    // ====================== 奖励分量拆解（修复后7项+终止项）======================
    private float reward_dist;
    private float reward_heading;
    private float reward_crossTrack;
    private float reward_colregs;
    private float reward_smooth;
    private float reward_safety;
    private float reward_collision;
    private float reward_finish;
    private float reward_timeout;
    private float reward_time;
    private float reward_waypoint;      // 新增

    private const float COLLISION_COOLDOWN = 3.0f;

    private Vector3 _lastFramePosition;
    private Coroutine _resetCoroutine;

    // ====================== 平滑奖励缓存 ======================
    private float smoothedDistanceDelta = 0f;
    private float currentCrossTrackError = 0f;


    // ====================== 动作连续性缓存 ======================
    private float lastActionForward = 0f;
    private float lastActionTurn = 0f;
    private float _currentActionForward = 0f;
    private float _currentActionTurn = 0f;
    // ----- 新增：统一奖励体系字段 -----
    private float _pendingWaypointReward = 0f;   // 待汇入总奖励的 Waypoint 原始值

    // ====================== 论文动作空间定义 (2.4节) ======================
    private const int ACTION_DIM = 6;
    private readonly float[] _sigmaRange = { 0.05f, 0.6f };
    private readonly float[] _ksRange = { 0.5f, 5.0f };
    private readonly float[] _dsRange = { 1.0f, 5.0f };

    private float[] _currentDwaWeights = new float[4] { 0.3f, 0.4f, 0.2f, 0.1f };
    private float _currentKs = 2.0f;
    private float _currentDs = 2.0f;

    // ====================== 奖励调试设置 ======================
    [Header("===== 奖励调试设置 =====")]
    public bool enableRewardDebug = true;
    public int debugUpdateInterval = 30;
    public bool logRewardDetails = false;
    public bool recordRewardHistory = true;

    private Dictionary<string, List<float>> rewardHistory = new Dictionary<string, List<float>>();
    private int debugFrameCounter = 0;
    private string lastRewardLog = "";

    // ====================== 新增：奖励平滑与统计 ======================
    private float smoothedReward = 0f;           // 平滑后的奖励值
    private float reward_facing = 0f;

    // ====================== 调试信息结构 ======================
    private class RewardDebugInfo
    {
        public float distance;
        public float heading;
        public float crossTrack;
        public float colregs;
        public float safety;
        public float smooth;
        public float time;
        public float waypoint;      // 新增
        public float finish;
        public float timeout;
        public float total;
        public float distToTarget;
        public float currentSpeed;
        public float elapsedTime;
        public float ttc;
    }
    private RewardDebugInfo currentRewardDebug = new RewardDebugInfo();

    // ============================================================
    // Unity 生命周期
    // ============================================================

    protected override void Awake()
    {
        base.Awake();

        rb = GetComponent<Rigidbody>();
        if (rb == null)
        {
            rb = gameObject.AddComponent<Rigidbody>();
            rb.maxAngularVelocity = 5f;
            rb.useGravity = false;
            rb.isKinematic = false;
            Debug.LogWarning("自动添加Rigidbody组件");
        }

        if (target == null)
        {
            GameObject targetObj = new GameObject("Default_Target");
            targetObj.transform.position = new Vector3(10f, 0f, 10f);
            target = targetObj.transform;
            Debug.LogWarning("未指定target，自动创建默认目标点");
        }

        boatController = GetComponent<BoatController>();
        gridManager = FindFirstObjectByType<GridManager>();
        globalPathfinder = FindFirstObjectByType<ImprovedAStar>();
        localPlanner = GetComponent<USV_LocalPlanner>();
        if (localPlanner == null)
        {
            Debug.LogError("[USV_GlobalRLAgent] USV_LocalPlanner 组件未找到！请确保 USV 对象上挂载了该组件。");
            localPlanner = gameObject.AddComponent<USV_LocalPlanner>();
        }

        InitCOLREGsEvaluator();

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

        if (enableRewardDebug) InitRewardHistory();
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

        episodeStartTime = Time.time;
        lastDistToTarget = target != null ? Vector3.Distance(transform.position, target.position) : 0;
        lastOmega = rb != null ? rb.angularVelocity.y * Mathf.Rad2Deg : 0f;

        // 清理死代码：移除未使用的 curiosity 数组初始化
        reward_dist = 0f;
        reward_heading = 0f;
        reward_crossTrack = 0f;
        reward_colregs = 0f;
        reward_smooth = 0f;
        reward_safety = 0f;
        reward_collision = 0f;
        reward_finish = 0f;
        reward_timeout = 0f;
        reward_time = 0f;
        reward_waypoint = 0f;       // 新增
        lastActionForward = 0f;
        lastActionTurn = 0f;

        _pendingWaypointReward = 0f;

        if (enableRewardDebug)
        {
            debugFrameCounter = 0;
            Debug.Log("🎯 新回合开始（修复版 v3）");
        }

        if (_resetCoroutine != null) StopCoroutine(_resetCoroutine);
        _resetCoroutine = StartCoroutine(WaitForGridInitThenReset());
    }

    private IEnumerator WaitForGridInitThenReset()
    {
        isEnvironmentInitializing = true;
        Debug.Log("🔄 开始智能体重置流程...");

        if (localPlanner != null)
        {
            localPlanner.ResetPathIndex();
        }

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

        if (enableTaskLoop && spawnManager != null)
        {
            spawnManager.Regenerate();
            yield return new WaitForSeconds(0.1f);
        }

        ResetAgentState(MaxSpeed, 90f);

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

        currentWaypointIndex = 0;
        consecutiveWaypointReached = 0;
        _pendingWaypointReward = 0f;
        reward_facing = 0f;

        totalCrossTrackError = 0;
        totalHeadingError = 0;
        collisionCount = 0;
        currentEpisodeLength = 0;

        totalSpeedTrackingError = 0;
        totalControlEffort = 0;
        minTimeToCollision = 999f;      // 修复：重置为999
        totalDistanceToGoal = 0;

        reward_dist = 0f;
        reward_heading = 0f;
        reward_crossTrack = 0f;
        reward_colregs = 0f;
        reward_smooth = 0f;
        reward_safety = 0f;
        reward_collision = 0f;
        reward_finish = 0f;
        reward_timeout = 0f;
        reward_time = 0f;
        reward_waypoint = 0f;           // 新增
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

    private void GenerateSafePositions() => GenerateSafePositions(1.0f);

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
        if (!IsEpisodeDone) EndEpisodeCustom();
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

        if (IsEpisodeDone) EndEpisode();
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

    private void OnDestroy() => CleanupTensorData();

    // ============================================================
    // ML-Agents 核心方法
    // ============================================================

    public override void CollectObservations(VectorSensor sensor)
    {
        float SafeNormalize(float value, float maxValue, float defaultValue = 0f)
        {
            float absMax = Mathf.Abs(maxValue);
            if (absMax < 0.0001f) return defaultValue;
            return Mathf.Clamp(value / absMax, -1f, 1f);
        }

        List<float> obsList = new List<float>(TOTAL_OBSERVATIONS);

        // 1. 自身状态 (3维)
        float surgeVel = rb != null ? Vector3.Dot(transform.forward, rb.linearVelocity) : 0f;
        obsList.Add(SafeNormalize(surgeVel, currentMaxSpeed));

        float omega = rb != null ? rb.angularVelocity.y * Mathf.Rad2Deg : 0f;
        obsList.Add(SafeNormalize(omega, MaxAngularSpeed));

        float distToTarget = target != null ? Vector3.Distance(transform.position, target.position) : 10f;

        // 【v10】2. 目标相对极坐标 (4维) —— 提前到前面，确保不被稀释
        float maxDist = Mathf.Max(gridWidth, gridHeight) * 1f;
        if (target != null)
        {
            Vector3 toTarget = target.position - transform.position;
            toTarget.y = 0;
            float dist = toTarget.magnitude;
            float angle = Vector3.SignedAngle(transform.forward, toTarget, Vector3.up);

            obsList.Add(Mathf.Clamp01(dist / maxDist));                    // 归一化距离
            obsList.Add(Mathf.Sin(angle * Mathf.Deg2Rad));                  // 目标方向 sin
            obsList.Add(Mathf.Cos(angle * Mathf.Deg2Rad));                  // 目标方向 cos
            obsList.Add(Mathf.Clamp(angle / 180f, -1f, 1f));                // 原始角度
        }
        else
        {
            obsList.AddRange(new float[] { 1f, 0f, 1f, 0f });
        }

        // 3. 横向偏差 (1维)
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
        obsList.Add(Mathf.Clamp(crossTrackError / 20f, -1f, 1f));

        // 【v10】4. 障碍物栅格 —— 从 11x11=121 降为 9x9=81，减少噪声
        const int NewViewRange = 4; // 原来是 5
        Vector2Int agentGridPos = gridManager != null ? gridManager.WorldToGrid(transform.position) : new Vector2Int(gridWidth / 2, gridHeight / 2);
        for (int x = -NewViewRange; x <= NewViewRange; x++)
        {
            for (int z = -NewViewRange; z <= NewViewRange; z++)
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

                obsList.Add(isObstacle ? 1f : 0f);
            }
        }

        // 5. 动态障碍物相对速度 (2维)
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
            obsList.Add(SafeNormalize(relVel.x, currentMaxSpeed));
            obsList.Add(SafeNormalize(relVel.z, currentMaxSpeed));
        }
        else
        {
            obsList.Add(0f);
            obsList.Add(0f);
        }

        // 6. 当前规划参数 (6维)
        obsList.Add(_currentDwaWeights[0]);
        obsList.Add(_currentDwaWeights[1]);
        obsList.Add(_currentDwaWeights[2]);
        obsList.Add(_currentDwaWeights[3]);
        obsList.Add(_currentKs / 5.0f);
        obsList.Add(_currentDs / 5.0f);

        // 7. 补齐或截断到 TOTAL_OBSERVATIONS
        while (obsList.Count < TOTAL_OBSERVATIONS)
        {
            obsList.Add(0f);
        }

        if (obsList.Count > TOTAL_OBSERVATIONS)
        {
            obsList = obsList.GetRange(0, TOTAL_OBSERVATIONS);
        }

        float[] obs = obsList.ToArray();

        for (int i = 0; i < obs.Length; i++)
        {
            if (float.IsNaN(obs[i]) || float.IsInfinity(obs[i]))
            {
                Debug.LogError($"NaN/Infinity detected in obs[{i}] = {obs[i]}");
                obs[i] = 0f;
            }
            sensor.AddObservation(obs[i]);
        }

        if (obs.Length != TOTAL_OBSERVATIONS)
        {
            Debug.LogWarning($"观测维度不匹配: 预期={TOTAL_OBSERVATIONS}, 实际={obs.Length}");
        }
    }

    // ============================================================
    // 修改 USV_GlobalRLAgent.OnActionReceived()
    // ============================================================
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

            var continuousActions = actions.ContinuousActions;
            if (continuousActions.Length < ACTION_DIM)
            {
                Debug.LogWarning($"[FORCE] 动作维度不足: {continuousActions.Length} < {ACTION_DIM}");
                return;
            }

            // 1. 读取6维动作
            float[] weightDeltas = new float[4];
            for (int i = 0; i < 4; i++)
            {
                weightDeltas[i] = Mathf.Clamp(continuousActions[i], -0.15f, 0.15f);
            }
            float deltaKs = Mathf.Clamp(continuousActions[4], -0.5f, 0.5f);
            float deltaDs = Mathf.Clamp(continuousActions[5], -0.5f, 0.5f);

            // 2. 更新DWA权重
            float[] newWeights = new float[4];
            for (int i = 0; i < 4; i++)
            {
                newWeights[i] = Mathf.Clamp(_currentDwaWeights[i] + weightDeltas[i], 0.05f, 0.6f);
            }
            float sum = newWeights[0] + newWeights[1] + newWeights[2] + newWeights[3];
            if (sum > 0.001f)
            {
                for (int i = 0; i < 4; i++)
                    _currentDwaWeights[i] = newWeights[i] / sum;
            }

            // 3. 更新A*参数
            _currentKs = Mathf.Clamp(_currentKs + deltaKs, 0.5f, 5.0f);
            _currentDs = Mathf.Clamp(_currentDs + deltaDs, 1.0f, 5.0f);

            // 4. 执行DWA（包含强制目标导向逻辑，确保船永远不会原地打转）
            if (localPlanner != null)
            {
                localPlanner.ExecuteDWA(_currentDwaWeights, _currentKs, _currentDs);


            }
            else
            {
                Debug.LogError("[FORCE] localPlanner 为 null! 请检查 USV 对象上是否挂载了 USV_LocalPlanner 组件");
                float forward = Mathf.Clamp(continuousActions[0], -1f, 1f);
                float turn = Mathf.Clamp(continuousActions[1], -1f, 1f);
                rb.AddForce(transform.forward * forward * 2f, ForceMode.Acceleration);
                rb.AddTorque(Vector3.up * turn * 2f, ForceMode.Acceleration);
            }

            // 缓存当前帧动作（供 CalculateReward 计算变化率）
            _currentActionForward = continuousActions[0];
            _currentActionTurn = continuousActions[1];

            // 5. 更新统计
            UpdateWaypointProgress();
            if (CalculateReward()) return;
            UpdateTrackingStats();
            currentEpisodeLength += 1f;
            lastActionForward = _currentActionForward;
            lastActionTurn = _currentActionTurn;

            // 修复：补上第二梯队统计调用
            UpdateSecondTierStats(actions);
        }
        catch (Exception ex)
        {
            Debug.LogError($"OnActionReceived 异常: {ex.Message}\n{ex.StackTrace}");
            AddReward(-5f);
            _resetReason = "exception";
            EndEpisodeCustom();
        }
    }

    private void ApplySimpleControl(float forward, float turn)
    {
        if (rb == null) return;
        float targetSpeed = forward * 1.2f;
        float targetOmega = turn * 0.26f;
        Vector3 force = (transform.forward * targetSpeed - rb.linearVelocity) * 5f;
        rb.AddForce(force, ForceMode.Acceleration);
        float torque = (targetOmega - rb.angularVelocity.y) * 2f;
        rb.AddTorque(Vector3.up * torque, ForceMode.Acceleration);
    }

    // ============================================================
    // 动作控制
    // ============================================================
    void MoveAgentContinuous(float forward, float turn)
    {
        if (rb == null) return;
        float targetSpeed = forward * currentMaxSpeed;
        float targetOmega = turn * MaxAngularSpeed * Mathf.Deg2Rad;
        Vector3 velError = transform.forward * targetSpeed - rb.linearVelocity;
        rb.AddForce(velError * 2.0f, ForceMode.Acceleration);
        float omegaError = targetOmega - rb.angularVelocity.y;
        rb.AddTorque(Vector3.up * omegaError * 1.5f, ForceMode.Acceleration);
        if (rb.linearVelocity.magnitude > currentMaxSpeed * 1.2f)
            rb.linearVelocity = rb.linearVelocity.normalized * currentMaxSpeed * 1.2f;
    }
    private void UpdateWaypointProgress()
    {
        if (globalPathfinder == null || globalPathfinder.path == null || globalPathfinder.path.Count == 0)
            return;

        if (currentWaypointIndex >= globalPathfinder.path.Count - 1)
            return;

        Vector3 currentWaypointPos = gridManager.GridToWorld(globalPathfinder.path[currentWaypointIndex]);
        float distToWaypoint = Vector3.Distance(transform.position, currentWaypointPos);

        // 【v10】阈值从 1.8f 提高到 2.5f，与 LocalPlanner 对齐
        float threshold = 2.5f;
        if (distToWaypoint < threshold)
        {
            float proximityFactor = 1f - (distToWaypoint / threshold);
            // 上限 15f，但受 approachFactor 衰减
            float rawWaypointReward = 10.0f * proximityFactor + 5.0f;

            _pendingWaypointReward += rawWaypointReward;

            currentWaypointIndex++;
            consecutiveWaypointReached++;

            if (enableRewardDebug)
                Debug.Log($"🎯 到达路径点 {currentWaypointIndex}, 原始奖励 +{rawWaypointReward:F2} (pending)");
        }
        else
        {
            consecutiveWaypointReached = Mathf.Max(0, consecutiveWaypointReached - 1);
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
    private void OnCollisionEnter(Collision collision)
    {
        if (IsEpisodeDone || _isTerminating) return;

        if (collision.gameObject.CompareTag("Obstacle") ||
            collision.gameObject.CompareTag("USV") ||
            collision.gameObject.CompareTag("DynamicObstacle"))
        {
            // 修改点：原来 -80 导致模型暴死；缩小到 -15，防止自杀
            float collisionPenalty = -15f;
            AddReward(collisionPenalty);
            reward_collision += collisionPenalty;
            collisionCount++;

            if (enableRewardDebug)
                Debug.Log($"💥 碰撞！对象={collision.gameObject.name}, 惩罚={collisionPenalty}");

            _resetReason = "collision";
            EndEpisodeCustom();
        }
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
        float minDist = float.MaxValue;

        if (localPlanner != null && localPlanner.dynamicObstacles != null
            && localPlanner.dynamicObstacles.Count > 0)
        {
            for (int i = 0; i < localPlanner.dynamicObstacles.Count; i++)
            {
                float d = Vector3.Distance(transform.position, localPlanner.dynamicObstacles[i]);
                if (d < minDist) minDist = d;
            }
        }

        float scanRadius = safeNearMissDistance * 2f;
        Collider[] cols = Physics.OverlapSphere(transform.position, scanRadius);

        foreach (var col in cols)
        {
            // 【必须添加】排除自身，防止自碰撞误判
            if (col.gameObject == gameObject) continue;

            if (!col.CompareTag("Obstacle")
                && !col.CompareTag("DynamicObstacle")
                && !col.CompareTag("Ship")
                && !col.CompareTag("USV"))
                continue;

            float d = Vector3.Distance(transform.position, col.transform.position);
            if (d < minDist) minDist = d;
        }

        return minDist;
    }
    private bool CalculateReward()
    {
        float currentForward = _currentActionForward;
        float currentTurn = _currentActionTurn;

        float distToTarget = Vector3.Distance(transform.position, target.position);
        float distanceDelta = lastDistToTarget - distToTarget;
        float currentSpeed = rb.linearVelocity.magnitude;
        float elapsedTime = Time.time - episodeStartTime;
        float omega = rb.angularVelocity.y * Mathf.Rad2Deg;

        float maxGridDim = Mathf.Max((float)gridWidth, (float)gridHeight);
        float safeMaxDist = Mathf.Max(maxGridDim, 1f);
        float distRatio = Mathf.Clamp01(distToTarget / safeMaxDist);

        // 降低平滑系数，响应更迅速
        smoothedDistanceDelta = 0.2f * smoothedDistanceDelta + 0.8f * distanceDelta;

        bool isApproaching = smoothedDistanceDelta > 0.003f && currentSpeed > 0.3f;
        float approachFactor = isApproaching ? 1.0f : 0.35f;

        // 1. 距离奖励
        float r_dist = 2.0f * smoothedDistanceDelta;
        if (distToTarget > 8.0f) r_dist -= 0.05f;
        if (distToTarget < 5.0f && smoothedDistanceDelta > 0.01f)
        {
            r_dist += 0.5f * Mathf.Exp(-distToTarget / 3f);
        }
        r_dist = Mathf.Clamp(r_dist, -2.0f, 3.0f);

        // 2. 航向奖励
        float r_heading = 0f;
        if (globalPathfinder != null && globalPathfinder.path != null
            && globalPathfinder.path.Count > 1 && gridManager != null)
        {
            int lookAheadIdx = Mathf.Min(currentWaypointIndex + 2, globalPathfinder.path.Count - 1);
            Vector3 lookAheadPos = gridManager.GridToWorld(globalPathfinder.path[lookAheadIdx]);
            lookAheadPos.y = transform.position.y;
            Vector3 toLookAhead = lookAheadPos - transform.position;
            toLookAhead.y = 0f;
            float angleToPath = Vector3.SignedAngle(transform.forward, toLookAhead, Vector3.up);
            float normalizedAngle = Mathf.Clamp01(Mathf.Abs(angleToPath) / 180f);

            if (currentSpeed > 0.3f && Mathf.Abs(omega) < 45f)
            {
                r_heading = (0.2f * (1f - normalizedAngle) - 0.02f) * approachFactor;
            }
            else if (currentSpeed > 0.1f)
            {
                r_heading = -0.02f;
            }
            else
            {
                r_heading = -0.1f;
            }
        }
        r_heading = Mathf.Clamp(r_heading, -0.2f, 0.15f);

        // 3. 目标直接朝向
        float r_facing = 0f;
        if (target != null)
        {
            Vector3 toTarget = target.position - transform.position;
            toTarget.y = 0f;
            float angleToTarget = Vector3.SignedAngle(transform.forward, toTarget, Vector3.up);
            float normTargetAngle = Mathf.Clamp01(Mathf.Abs(angleToTarget) / 180f);

            if (currentSpeed > 0.3f)
            {
                r_facing = 0.05f * (1f - normTargetAngle) - 0.01f;
            }
            else
            {
                r_facing = -0.01f;
            }
        }
        r_facing = Mathf.Clamp(r_facing, -0.1f, 0.05f);

        // 4. 横向偏差
        float r_crossTrack = 0f;
        if (globalPathfinder != null && globalPathfinder.path != null
            && globalPathfinder.path.Count > 1 && gridManager != null)
        {
            Vector3 closest = GetClosestPointOnGlobalPath(transform.position);
            float cte = Vector3.Distance(
                new Vector3(transform.position.x, 0, transform.position.z),
                new Vector3(closest.x, 0, closest.z)
            );
            currentCrossTrackError = cte;
            r_crossTrack = -0.02f * Mathf.Abs(cte);
        }

        // 5. COLREGs
        float r_colregs = 0f;
        var colregsObstacles = GetColregsObstacles();
        if (colregsEvaluator != null && colregsObstacles.Count > 0)
        {
            try
            {
                float rawColregsReward = colregsEvaluator.GetReward(
                    transform.position, transform.eulerAngles.y,
                    rb.linearVelocity.magnitude, colregsObstacles);
                r_colregs = Mathf.Clamp(rawColregsReward * 0.2f, -0.1f, 0.1f);
            }
            catch (Exception e) { Debug.LogWarning($"COLREGs计算异常: {e.Message}"); }
        }

        // 6. 平滑奖励
        float omegaDelta = Mathf.Abs(omega - lastOmega);
        float r_omegaSmooth = (omegaDelta > 30f) ? -0.002f * (omegaDelta - 30f) : 0f;
        lastOmega = omega;

        float actionDelta = Mathf.Abs(currentForward - lastActionForward) + Mathf.Abs(currentTurn - lastActionTurn);
        float r_actionSmooth = (actionDelta > 1.0f) ? -0.002f * (actionDelta - 1.0f) : 0f;
        float r_smooth = r_omegaSmooth + r_actionSmooth;

        // 7. 安全奖励：TTC
        float r_safety = 0f;
        float ttc = CalculateTTCDirectional();
        if (ttc < minTimeToCollision) minTimeToCollision = ttc;

        if (ttc > 0.01f && ttc < 10.0f)
        {
            r_safety = -0.05f * Mathf.Exp(-ttc / 3.0f);
        }

        // 【修改点】时间惩罚 —— 改为固定小惩罚，防止模型被逼疯
        // 原代码：float r_time = -0.05f * elapsedTime; （会导致随时间累加巨大惩罚）
        // 修复后：
        float r_time = -0.005f;

        // 9. Waypoint 奖励
        float r_waypoint = Mathf.Clamp(_pendingWaypointReward, 0f, 5f) * approachFactor;
        _pendingWaypointReward = 0f;

        if (r_waypoint > 0.01f)
        {
            AddReward(r_waypoint);
            reward_waypoint += r_waypoint;
        }

        // 10. 停滞惩罚
        float stagnationPenalty = 0f;
        if (currentSpeed < 0.2f && Mathf.Abs(smoothedDistanceDelta) < 0.01f && elapsedTime > 2f)
        {
            stagnationPenalty = -3.0f;
        }

        // 11. 汇总奖励
        float totalReward =
            W_DISTANCE * r_dist +
            W_HEADING * r_heading +
            W_CROSSTRACK * r_crossTrack +
            0.05f * r_facing +
            W_COLREGS * r_colregs +
            W_SAFETY * r_safety +
            W_SMOOTH * r_smooth +
            W_TIME * r_time +
            stagnationPenalty;

        // 提升奖励上下限
        totalReward = Mathf.Clamp(totalReward, -5.0f, 10.0f);

        // 12. 奖励平滑
        const float REWARD_SMOOTHING = 0.1f;
        smoothedReward = REWARD_SMOOTHING * totalReward + (1f - REWARD_SMOOTHING) * smoothedReward;
        float finalReward = smoothedReward;
        AddReward(finalReward);

        // 13. 统计记录
        reward_dist += r_dist;
        reward_heading += r_heading;
        reward_crossTrack += r_crossTrack;
        reward_colregs += r_colregs;
        reward_smooth += r_smooth;
        reward_safety += r_safety;
        reward_time += r_time;
        reward_facing += r_facing;

        lastDistToTarget = distToTarget;

        // 终止条件：到达目标
        if (distToTarget < targetArriveThreshold)
        {
            float timeBonus = Mathf.Max(0f, (currentMaxEpisodeTime - elapsedTime)) * 0.2f;
            float finishReward = 150f + timeBonus; // 【关键修改】砍半！
            finishReward = Mathf.Clamp(finishReward, 150f, 250f); // 上限也砍半

            AddReward(finishReward);
            reward_finish += finishReward;

            if (enableRewardDebug)
                Debug.Log($"🏁 到达目标！距离={distToTarget:F2}m, 耗时={elapsedTime:F1}s, 奖励={finishReward:F1}");

            _resetReason = "target";
            EndEpisodeCustom();
            return true;
        }

        // 终止条件：超时
        if (elapsedTime > currentMaxEpisodeTime)
        {
            float distFactor = Mathf.Clamp01(distToTarget / safeMaxDist);
            float timeoutPenalty = -15f - 2f * distFactor;
            AddReward(timeoutPenalty);
            reward_timeout += timeoutPenalty;

            if (enableRewardDebug)
                Debug.Log($"⏰ 超时！距离终点={distToTarget:F1}m, 惩罚={timeoutPenalty:F1}");

            _resetReason = "timeout";
            EndEpisodeCustom();
            return true;
        }

        // 终止条件：边界
        float hardBoundary = maxGridDim * boundaryThresholdFactor;
        float overBoundX = Mathf.Max(0f, Mathf.Abs(transform.position.x) - hardBoundary);
        float overBoundZ = Mathf.Max(0f, Mathf.Abs(transform.position.z) - hardBoundary);

        if (overBoundX > 0f || overBoundZ > 0f)
        {
            AddReward(-25f);
            _resetReason = "boundary";
            if (enableRewardDebug)
                Debug.Log($"🚧 越界！偏移X={overBoundX:F1} Z={overBoundZ:F1}m, 惩罚=-25");
            EndEpisodeCustom();
            return true;
        }

        // 14. 调试统计更新
        currentRewardDebug.distance = r_dist;
        currentRewardDebug.heading = r_heading;
        currentRewardDebug.crossTrack = r_crossTrack;
        currentRewardDebug.colregs = r_colregs;
        currentRewardDebug.safety = r_safety;
        currentRewardDebug.smooth = r_smooth;
        currentRewardDebug.time = r_time;
        currentRewardDebug.waypoint = r_waypoint;
        currentRewardDebug.finish = reward_finish;
        currentRewardDebug.timeout = reward_timeout;
        currentRewardDebug.total = totalReward;
        currentRewardDebug.distToTarget = distToTarget;
        currentRewardDebug.currentSpeed = currentSpeed;
        currentRewardDebug.elapsedTime = elapsedTime;
        currentRewardDebug.ttc = ttc;
        UpdateTotalRewardDebug();

        return false;
    }
    private void UpdateTotalRewardDebug()
    {
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
            "Distance", "Heading", "CrossTrack", "COLREGs", "Safety",
            "Smooth", "Time", "Waypoint", "Finish", "Timeout", "Total"  // 新增Waypoint
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
        float avg = 0, min = float.MaxValue, max = float.MinValue, sum = 0;
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
        sb.AppendLine($"║  📊 奖励调试面板 [修复版v3] - 帧 {Time.frameCount}         ║");
        sb.AppendLine("╠══════════════════════════════════════════════════════════╣");
        sb.AppendLine($"║  位置: ({transform.position.x:F2}, {transform.position.z:F2})         ║");
        sb.AppendLine($"║  速度: {currentRewardDebug.currentSpeed:F2} m/s | TTC: {currentRewardDebug.ttc:F2}s      ║");
        sb.AppendLine($"║  距目标: {currentRewardDebug.distToTarget:F2}m | 时间: {currentRewardDebug.elapsedTime:F1}s    ║");
        sb.AppendLine("╠══════════════════════════════════════════════════════════╣");
        sb.AppendLine($"║  🎯 距离奖励:     {currentRewardDebug.distance,10:F4}   ║");
        sb.AppendLine($"║  🧭 航向奖励:     {currentRewardDebug.heading,10:F4}   ║");
        sb.AppendLine($"║  🛤️  横向偏差:     {currentRewardDebug.crossTrack,10:F4}   ║");
        sb.AppendLine($"║  ⚓ COLREGs:       {currentRewardDebug.colregs,10:F4}   ║");
        sb.AppendLine($"║  🛡️  安全(TTC):    {currentRewardDebug.safety,10:F4}   ║");
        sb.AppendLine($"║  ✨ 平滑(动作):    {currentRewardDebug.smooth,10:F4}   ║");
        sb.AppendLine($"║  🎯 Waypoint:      {currentRewardDebug.waypoint,10:F4}   ║");  // 新增
        sb.AppendLine($"║  ⏰ 时间惩罚:     {currentRewardDebug.time,10:F4}   ║");
        sb.AppendLine("╠══════════════════════════════════════════════════════════╣");
        sb.AppendLine($"║  💰 总奖励:       {currentRewardDebug.total,10:F4}   ║");
        sb.AppendLine($"║  📈 累计奖励:     {GetCumulativeReward(),10:F2}   ║");
        sb.AppendLine("╠══════════════════════════════════════════════════════════╣");

        if (recordRewardHistory && rewardHistory.Count > 0)
        {
            sb.AppendLine("║  📊 历史统计 (最近)                                  ║");
            sb.AppendLine($"║  距离:   {GetRewardStats("Distance")}    ║");
            sb.AppendLine($"║  安全:   {GetRewardStats("Safety")}      ║");
            sb.AppendLine($"║  总奖励: {GetRewardStats("Total")}       ║");
        }

        sb.AppendLine("╚══════════════════════════════════════════════════════════╝");

        if (logRewardDetails) Debug.Log(sb.ToString());
        else
        {
            lastRewardLog = $"Reward: {currentRewardDebug.total:F4} | Dist: {currentRewardDebug.distToTarget:F2}m | " +
                           $"TTC: {currentRewardDebug.ttc:F2}s | Safety: {currentRewardDebug.safety:F3} | " +
                           $"Colregs: {currentRewardDebug.colregs:F3} | Waypoint: {reward_waypoint:F2}";
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
        // 修改点：合理的安全距离上限，防止无穷大值干扰统计
        float minT = 20f;

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

            float approachRate = -Vector3.Dot(relVel, deltaPos.normalized);
            if (approachRate <= 0.1f) continue;
            float t = dist / approachRate;
            if (t < minT) minT = t;
        }
        return minT;
    }

    // ============================================================
    // 回合结束统计写入（修复字段版）
    // ============================================================

    private void WriteFinalEpisodeStats()
    {
        if (currentEpisodeLength <= 0) return;

        float avgCTE = totalCrossTrackError / currentEpisodeLength;
        float avgHdg = totalHeadingError / currentEpisodeLength;
        float avgSpeedErr = totalSpeedTrackingError / currentEpisodeLength;
        float avgEffort = totalControlEffort / currentEpisodeLength;
        float avgGoalDist = totalDistanceToGoal / currentEpisodeLength;

        // USV 指标
        Academy.Instance.StatsRecorder.Add("USV/CrossTrackError", avgCTE, StatAggregationMethod.Average);
        Academy.Instance.StatsRecorder.Add("USV/HeadingError", avgHdg, StatAggregationMethod.Average);
        Academy.Instance.StatsRecorder.Add("USV/EpisodeLength", currentEpisodeLength, StatAggregationMethod.Average);
        Academy.Instance.StatsRecorder.Add("USV/SpeedTrackingError", avgSpeedErr, StatAggregationMethod.Average);
        Academy.Instance.StatsRecorder.Add("USV/ControlEffort", avgEffort, StatAggregationMethod.Average);
        Academy.Instance.StatsRecorder.Add("USV/MinTTC", minTimeToCollision, StatAggregationMethod.Average);
        Academy.Instance.StatsRecorder.Add("USV/DistanceToGoal", avgGoalDist, StatAggregationMethod.Average);
        Academy.Instance.StatsRecorder.Add("USV/CollisionCount", collisionCount, StatAggregationMethod.Sum);

        // 奖励分量（修复后7项 + 终止项）
        Academy.Instance.StatsRecorder.Add("Reward/Distance", reward_dist, StatAggregationMethod.Average);
        Academy.Instance.StatsRecorder.Add("Reward/Heading", reward_heading, StatAggregationMethod.Average);
        Academy.Instance.StatsRecorder.Add("Reward/CrossTrack", reward_crossTrack, StatAggregationMethod.Average);
        Academy.Instance.StatsRecorder.Add("Reward/COLREGs", reward_colregs, StatAggregationMethod.Average);
        Academy.Instance.StatsRecorder.Add("Reward/Safety", reward_safety, StatAggregationMethod.Average);
        Academy.Instance.StatsRecorder.Add("Reward/Smooth", reward_smooth, StatAggregationMethod.Average);
        Academy.Instance.StatsRecorder.Add("Reward/Time", reward_time, StatAggregationMethod.Average);
        Academy.Instance.StatsRecorder.Add("Reward/Waypoint", reward_waypoint, StatAggregationMethod.Average);  // 新增
        Academy.Instance.StatsRecorder.Add("Reward/Finish", reward_finish, StatAggregationMethod.Average);
        Academy.Instance.StatsRecorder.Add("Reward/Timeout", reward_timeout, StatAggregationMethod.Average);
        Academy.Instance.StatsRecorder.Add("Reward/Collision", reward_collision, StatAggregationMethod.Average);
        Academy.Instance.StatsRecorder.Add("Reward/Facing", reward_facing, StatAggregationMethod.Average);
        // 核心指标
        Academy.Instance.StatsRecorder.Add("Environment/Cumulative Reward", GetCumulativeReward(), StatAggregationMethod.Average);
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
                       $"  🧭 航向: {currentRewardDebug.heading:F4}\n" +
                       $"  🛤️  横向: {currentRewardDebug.crossTrack:F4}\n" +
                       $"  🛡️  安全: {currentRewardDebug.safety:F4}\n" +
                       $"  ⚓ COLREGs: {currentRewardDebug.colregs:F4}\n" +
                       $"  🎯 Waypoint: {reward_waypoint:F2}\n" +
                       $"  📍 距目标: {currentRewardDebug.distToTarget:F2}m\n" +
                       $"  🚀 速度: {currentRewardDebug.currentSpeed:F2}m/s\n" +
                       $"  ⏱️  TTC: {currentRewardDebug.ttc:F2}s\n" +
                       $"━━━━━━━━━━━━━━━━━━━━━━━━━━";

        UnityEditor.Handles.Label(labelPos, label);
    }
}