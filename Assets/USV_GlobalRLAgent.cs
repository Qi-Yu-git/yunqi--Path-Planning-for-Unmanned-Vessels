using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;
using System.Reflection;

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
    /// 检查当前回合是否结束
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
        boatController = GetComponent<BoatController>();
        gridManager = FindFirstObjectByType<GridManager>();
        globalPathfinder = FindFirstObjectByType<ImprovedAStar>();
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
        currentMaxSpeed = maxSpeed;
        currentMaxEpisodeTime = maxEpisodeTime;
        episodeStartTime = Time.time;

        if (target != null)
        {
            lastDistToTarget = Vector3.Distance(transform.position, target.position);
        }
        else
        {
            Debug.LogWarning("目标点未设置，无法计算初始距离");
            lastDistToTarget = 0;
        }
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
            rb.interpolation = RigidbodyInterpolation.None;
        }
        else
        {
            Debug.LogError("未找到Rigidbody组件！请给智能体添加刚体组件");
        }
    }

    public override void OnEpisodeBegin()
    {
        if (gridManager == null)
        {
            Debug.LogWarning("无法重置回合：GridManager未初始化");
            return;
        }

        if (safePositions == null || safePositions.Count == 0)
        {
            Debug.LogWarning("无法重置回合：安全位置列表为空");
            return;
        }

        // 任务循环逻辑
        if (enableTaskLoop)
        {
            if (spawnManager != null)
            {
                spawnManager.Regenerate();
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
            Debug.Log("任务循环已禁用，跳过随机生成和环境重置");
            return;
        }

        // 重置物理状态
        if (rb != null)
        {
            rb.linearVelocity = Vector3.zero;
            rb.angularVelocity = Vector3.zero;
        }

        // 重置路径相关
        currentWaypointIndex = 0;
        if (globalPathfinder != null)
        {
            globalPathfinder.CalculatePathAfterDelay();
            Invoke(nameof(NotifyBoatLoadNewPath), 0.5f);
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
        // 保底观测值，确保观测向量不为空
        sensor.AddObservation(0f);

        if (rb == null || target == null || gridManager == null)
        {
            Debug.LogWarning("缺少必要组件，无法收集完整观测值");
            return;
        }

        // 1. 前进速度（归一化）
        float forwardSpeed = Vector3.Dot(transform.forward, rb.linearVelocity);
        sensor.AddObservation(Mathf.Clamp(forwardSpeed / MaxSpeed, -1f, 1f));

        // 2. 朝向（归一化到 [-1, 1]）
        sensor.AddObservation((transform.eulerAngles.y % 360f) / 180f - 1f);

        // 3. 目标距离和角度
        float distToTarget = Vector3.Distance(transform.position, target.position);
        sensor.AddObservation(Mathf.Clamp01(distToTarget / (Mathf.Max(gridWidth, gridHeight) * 1f)));

        float angleToTarget = Vector3.SignedAngle(transform.forward,
            target.position - transform.position, Vector3.up);
        sensor.AddObservation(angleToTarget / 180f);

        // 4. 全局路径方向
        if (globalPathfinder != null && globalPathfinder.path != null &&
            globalPathfinder.path.Count > currentWaypointIndex + 1)
        {
            Vector3 nextWaypoint = gridManager.栅格转世界(globalPathfinder.path[currentWaypointIndex + 1]);
            float angleToWaypoint = Vector3.SignedAngle(transform.forward,
                nextWaypoint - transform.position, Vector3.up);
            sensor.AddObservation(angleToWaypoint / 180f);
        }
        else
        {
            sensor.AddObservation(0f);
        }

        // 5. 局部障碍物（5x5网格）
        Vector2Int agentGridPos = gridManager.世界转栅格(transform.position);
        Vector2Int checkPos = new Vector2Int();
        for (int x = -ViewRange; x <= ViewRange; x++)
        {
            for (int z = -ViewRange; z <= ViewRange; z++)
            {
                checkPos.x = agentGridPos.x + x;
                checkPos.y = agentGridPos.y + z;
                bool isObstacle = !IsPassable(checkPos);
                sensor.AddObservation(isObstacle ? 1f : 0f);
            }
        }

        // 6. 动态障碍物速度
        USV_LocalPlanner localPlanner = GetComponent<USV_LocalPlanner>();
        if (localPlanner != null && localPlanner.dynamicObstacleVelocities.Count > 0)
        {
            Vector3 closestObsVel = localPlanner.dynamicObstacleVelocities[0];
            sensor.AddObservation(closestObsVel.x / 5f);
            sensor.AddObservation(closestObsVel.z / 5f);
        }
        else
        {
            sensor.AddObservation(0f);
            sensor.AddObservation(0f);
        }
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        if (target == null || gridManager == null) return;

        // 执行移动动作
        MoveAgent(actions.DiscreteActions[0]);

        // 计算奖励
        float distToTarget = Vector3.Distance(transform.position, target.position);
        float distanceDelta = lastDistToTarget - distToTarget;

        // 1. 基于距离变化的奖励
        float proximityFactor = Mathf.Clamp01(1 - (distToTarget / (Mathf.Max(gridWidth, gridHeight) * 1f)));
        float stepReward = distanceDelta * (1 + proximityFactor * targetProximitySmoothing);
        stepReward = Mathf.Clamp(stepReward, minStepPenalty, maxStepReward);
        AddReward(stepReward);

        // 2. 平滑移动奖励
        float currentSpeed = rb.linearVelocity.magnitude;
        float speedStabilityReward = 0.1f * (1 - Mathf.Abs(currentSpeed - MaxSpeed * 0.5f) / (MaxSpeed * 0.5f));
        AddReward(speedStabilityReward);

        // 3. 碰撞惩罚
        Vector2Int currentGrid = gridManager.世界转栅格(transform.position);
        if (!IsPassable(currentGrid))
        {
            AddReward(-50f);
            EndEpisode();
            return;
        }

        // 4. 到达目标奖励
        if (distToTarget < 2f)
        {
            AddReward(currentSpeed < MaxSpeed * 0.3f ? 100f : 50f);
            EndEpisode();
            return;
        }

        // 5. 超时惩罚
        if (Time.time - episodeStartTime > currentMaxEpisodeTime)
        {
            AddReward(-20f);
            EndEpisode();
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
                if (currentSpeed < currentMaxSpeed * 0.8f)
                {
                    rb.AddForce(transform.forward * currentMaxSpeed * 0.6f, ForceMode.VelocityChange);
                }
                else
                {
                    rb.AddForce(transform.forward * currentMaxSpeed * 0.2f, ForceMode.VelocityChange);
                }
                break;
            case 1: // 左转
                transform.Rotate(Vector3.up, -15f * Time.fixedDeltaTime);
                break;
            case 2: // 右转
                transform.Rotate(Vector3.up, 15f * Time.fixedDeltaTime);
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
            gridManager.栅格转世界(globalPathfinder.path[currentWaypointIndex]));

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
        if (gridPos.x < 0 || gridPos.x >= gridWidth ||
            gridPos.y < 0 || gridPos.y >= gridHeight)
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
                passableGrid[x, z] = gridManager.栅格是否可通行(new Vector2Int(x, z));
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
                    safePositions.Add(gridManager.栅格转世界(new Vector2Int(x, z)));
                }
            }
        }

        Debug.Log($"生成 {safePositions.Count} 个安全位置");
    }
}