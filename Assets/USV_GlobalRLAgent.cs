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

        boatController = GetComponent<BoatController>();
        gridManager = UnityEngine.Object.FindFirstObjectByType<GridManager>();
        globalPathfinder = UnityEngine.Object.FindFirstObjectByType<ImprovedAStar>();
        rb = GetComponent<Rigidbody>();

        if (gridManager == null)
        {
            Debug.LogError("未找到GridManager组件！请确保场景中存在GridManager");
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
        GenerateSafePositions();
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

    public override void OnEpisodeBegin()
    {
        IsEpisodeDone = false;

        if (gridManager == null || safePositions == null || safePositions.Count == 0)
        {
            Debug.LogWarning("回合重置失败：GridManager未初始化或安全位置为空");
            return;
        }

        if (enableTaskLoop)
        {
            spawnManager?.Regenerate();
        }

        if (rb != null)
        {
            rb.linearVelocity = Vector3.zero;
            rb.angularVelocity = Vector3.zero;
        }

        currentWaypointIndex = 0;
        globalPathfinder?.CalculatePathAfterDelay();
        Invoke(nameof(NotifyBoatLoadNewPath), 0.5f);

        ResetAgentState(MaxSpeed, 60f);
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
        float[] emptyObs = new float[TOTAL_OBSERVATIONS];
        Array.Fill(emptyObs, 0f);

        if (rb == null || target == null || gridManager == null)
        {
            Debug.LogWarning("核心组件缺失，返回空观测");
            sensor.AddObservation(emptyObs);
            return;
        }

        int obsCount = 0;

        // 1. 前进速度（归一化）
        float forwardSpeed = Vector3.Dot(transform.forward, rb.linearVelocity);
        float normalizedSpeed = Mathf.Clamp(forwardSpeed / MaxSpeed, -1f, 1f);
        sensor.AddObservation(normalizedSpeed);
        obsCount++;

        // 2. 朝向（归一化）
        float normalizedYaw = ((transform.eulerAngles.y % 360f) / 180f) - 1f;
        sensor.AddObservation(normalizedYaw);
        obsCount++;

        // 3. 目标距离（归一化）
        float distToTarget = Vector3.Distance(transform.position, target.position);
        float normalizedDist = Mathf.Clamp01(distToTarget / (Mathf.Max(gridWidth, gridHeight) * 1f));
        sensor.AddObservation(normalizedDist);
        obsCount++;

        // 4. 目标角度（归一化）
        float angleToTarget = Vector3.SignedAngle(transform.forward, target.position - transform.position, Vector3.up);
        float normalizedAngle = angleToTarget / 180f;
        sensor.AddObservation(normalizedAngle);
        obsCount++;

        // 5. 全局路径方向
        float normalizedWaypointAngle = 0f;
        if (globalPathfinder != null && globalPathfinder.path != null && globalPathfinder.path.Count > currentWaypointIndex + 1)
        {
            Vector3 nextWaypoint = gridManager.GridToWorld(globalPathfinder.path[currentWaypointIndex + 1]);
            float angleToWaypoint = Vector3.SignedAngle(transform.forward, nextWaypoint - transform.position, Vector3.up);
            normalizedWaypointAngle = angleToWaypoint / 180f;
        }
        sensor.AddObservation(normalizedWaypointAngle);
        obsCount++;

        // 6. 局部障碍物（11x11）
        Vector2Int agentGridPos = gridManager.WorldToGrid(transform.position);
        for (int x = -ViewRange; x <= ViewRange; x++)
        {
            for (int z = -ViewRange; z <= ViewRange; z++)
            {
                Vector2Int checkPos = new Vector2Int(agentGridPos.x + x, agentGridPos.y + z);
                bool isObstacle = checkPos.x < 0 || checkPos.x >= gridWidth || checkPos.y < 0 || checkPos.y >= gridHeight
                    ? true
                    : !IsPassable(checkPos);

                sensor.AddObservation(isObstacle ? 1f : 0f);
                obsCount++;
            }
        }

        // 7. 动态障碍物速度
        float obsVelX = 0f;
        float obsVelZ = 0f;
        USV_LocalPlanner localPlanner = GetComponent<USV_LocalPlanner>();
        if (localPlanner != null && localPlanner.dynamicObstacleVelocities != null && localPlanner.dynamicObstacleVelocities.Count > 0)
        {
            Vector3 closestObsVel = localPlanner.dynamicObstacleVelocities[0];
            obsVelX = Mathf.Clamp(closestObsVel.x / 5f, -1f, 1f);
            obsVelZ = Mathf.Clamp(closestObsVel.z / 5f, -1f, 1f);
        }
        sensor.AddObservation(obsVelX);
        obsCount++;
        sensor.AddObservation(obsVelZ);
        obsCount++;

        // 补全缺失的观测值
        int missingObs = TOTAL_OBSERVATIONS - obsCount;
        if (missingObs > 0)
        {
            for (int i = 0; i < missingObs; i++)
            {
                sensor.AddObservation(0f);
            }
        }
        else if (missingObs < 0)
        {
            Debug.LogError($"观测值数量超标：实际{obsCount}个，期望{TOTAL_OBSERVATIONS}个");
        }
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        if (IsEpisodeDone || target == null || gridManager == null || rb == null) return;

        if (currentMaxSpeed <= 0)
        {
            currentMaxSpeed = MaxSpeed;
            Debug.LogWarning("currentMaxSpeed未初始化，使用默认值");
        }

        int discreteAction = Mathf.Clamp(actions.DiscreteActions[0], 0, 3);
        MoveAgent(discreteAction);

        CalculateReward();

        GetComponent<USV_LocalPlanner>()?.OnAgentActionReceived(actions);
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
        Vector2Int currentGrid = gridManager.WorldToGrid(transform.position);
        if (!IsPassable(currentGrid))
        {
            AddReward(-50f);
            EndEpisodeCustom();
            return;
        }

        // 到达目标奖励
        if (distToTarget < 2f)
        {
            float targetReward = currentSpeed < MaxSpeed * 0.3f ? 100f : 50f;
            AddReward(targetReward);
            EndEpisodeCustom();
            return;
        }

        // 超时惩罚
        if (currentMaxEpisodeTime > 0 && (Time.time - episodeStartTime) > currentMaxEpisodeTime)
        {
            AddReward(-20f);
            EndEpisodeCustom();
            return;
        }

        // 更新状态
        UpdateWaypointIndex();
        lastDistToTarget = distToTarget;
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

        passableGrid = new bool[gridWidth, gridHeight];
        for (int x = 0; x < gridWidth; x++)
        {
            for (int z = 0; z < gridHeight; z++)
            {
                passableGrid[x, z] = gridManager.IsGridPassable(new Vector2Int(x, z));
            }
        }
    }

    /// <summary>
    /// 生成安全位置列表
    /// </summary>
    private void GenerateSafePositions()
    {
        safePositions = new List<Vector3>();
        if (gridManager == null || passableGrid == null) return;

        for (int x = 0; x < gridWidth; x++)
        {
            for (int z = 0; z < gridHeight; z++)
            {
                if (passableGrid[x, z])
                {
                    safePositions.Add(gridManager.GridToWorld(new Vector2Int(x, z)));
                }
            }
        }
        Debug.Log($"生成 {safePositions.Count} 个安全位置");
    }
}