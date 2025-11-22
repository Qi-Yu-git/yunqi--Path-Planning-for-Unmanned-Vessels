using System.Collections;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;
using System.Collections.Generic;

public class USV_GlobalRLAgent : Agent
{
    [Tooltip("接近目标的平滑系数，值越小减速越早")]
    public float targetProximitySmoothing = 0.3f;
    [Tooltip("最大单步奖励上限")]
    public float maxStepReward = 2f;
    [Tooltip("最小单步惩罚下限")]
    public float minStepPenalty = -1f;
    [Tooltip("训练模式：true=循环训练，false=单次任务")]
    public bool isTrainingMode = true;// 训练时设为true，部署时设为false

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
    private ImprovedAStar globalPathfinder;
    private int currentWaypointIndex = 0;
    private float episodeStartTime;
    private bool isTaskCompleted = false;

    // 在USV_GlobalRLAgent.cs中修改Awake()方法
    protected override void Awake()
    {
        base.Awake();  // 调用父类的Awake方法

        // 获取Rigidbody组件
        rb = GetComponent<Rigidbody>();
        if (rb == null)
        {
            Debug.LogError("USV_GlobalRLAgent缺少Rigidbody组件！");
            return;
        }

        // 修复GridManager获取逻辑（包含API更新）
        if (GridManager.Instance != null)
        {
            gridManager = GridManager.Instance;
            Debug.Log("通过单例成功获取GridManager");
        }
        else
        {
            // 核心修改：使用新API FindFirstObjectByType替代旧API
            gridManager = FindFirstObjectByType<GridManager>();
            if (gridManager != null)
            {
                Debug.LogWarning("通过FindFirstObjectByType获取GridManager，建议检查单例是否生效");
            }
            else
            {
                Debug.LogError("未找到GridManager组件！请确认场景中存在GridManager对象且单例初始化正常");
                return;
            }
        }

        StartCoroutine(WaitForGridInit());
    }
    private IEnumerator WaitForGridInit()
    {
        while (!gridManager.IsGridReady())
        {
            Debug.Log("等待GridManager初始化...");
            yield return new WaitForSeconds(0.5f);
        }

        gridWidth = gridManager.gridWidth;
        gridHeight = gridManager.gridHeight;
        CachePassableGrid();
        GenerateSafePositions();
    }

    public override void Initialize()
    {
        if (rb != null)
        {
            rb.maxAngularVelocity = 5f;
            rb.useGravity = false;
            rb.interpolation = RigidbodyInterpolation.None;
        }
    }

    public override void OnEpisodeBegin()
    {
        if (!isTrainingMode && isTaskCompleted) return;

        StartCoroutine(WaitForGridInitThenReset());
    }

    private IEnumerator WaitForGridInitThenReset()
    {
        while (gridManager == null || !gridManager.IsGridReady())
        {
            Debug.Log("等待GridManager初始化...");
            yield return new WaitForSeconds(0.1f);
        }

        while (safePositions == null || safePositions.Count == 0)
        {
            Debug.Log("等待安全位置生成...");
            GenerateSafePositions();
            yield return new WaitForSeconds(0.1f);
        }

        // 重置无人船状态
        transform.position = GetRandomSafePosition();
        transform.rotation = Quaternion.Euler(0, Random.Range(0, 360), 0);
        if (rb != null)
        {
            rb.linearVelocity = Vector3.zero;
            rb.angularVelocity = Vector3.zero;
        }

        // 生成新场景元素
        var spawnManager = Object.FindFirstObjectByType<RandomSpawnManager>();
        if (spawnManager != null)
        {
            spawnManager.GenerateRandomRocks();
            spawnManager.GenerateRandomStartAndTarget();
            if (target != null)
            {
                target.position = spawnManager.TargetPos;
            }
        }

        // 刷新栅格和路径
        gridManager.强制刷新栅格();
        var boatController = GetComponent<BoatController>();
        if (boatController != null)
        {
            boatController.TryLoadPath();
        }

        // 初始化episode参数
        lastDistToTarget = Vector3.Distance(transform.position, target.position);
        currentWaypointIndex = 0;
        episodeStartTime = Time.time;
        isTaskCompleted = false;

        Debug.Log($"Episode重置完成：新起点={transform.position}，新终点={target.position}");
    }

    private Vector3 GetRandomSafePosition()
    {
        if (safePositions == null || safePositions.Count == 0)
        {
            GenerateSafePositions();
        }
        return safePositions[Random.Range(0, safePositions.Count)] + new Vector3(0, 0.4f, 0);
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        if (rb == null || target == null || gridManager == null) return;

        // 前进速度（归一化）
        float forwardSpeed = Vector3.Dot(transform.forward, rb.linearVelocity);
        sensor.AddObservation(Mathf.Clamp(forwardSpeed / MaxSpeed, -1f, 1f));

        // 朝向（归一化到 [-1, 1]）
        sensor.AddObservation((transform.eulerAngles.y % 360f) / 180f - 1f);

        // 目标距离和角度
        float distToTarget = Vector3.Distance(transform.position, target.position);
        sensor.AddObservation(Mathf.Clamp01(distToTarget / (Mathf.Max(gridWidth, gridHeight) * 1f)));

        float angleToTarget = Vector3.SignedAngle(transform.forward, target.position - transform.position, Vector3.up);
        sensor.AddObservation(angleToTarget / 180f);

        // 全局路径方向
        if (globalPathfinder != null && globalPathfinder.path != null && globalPathfinder.path.Count > currentWaypointIndex + 1)
        {
            Vector3 nextWaypoint = gridManager.栅格转世界(globalPathfinder.path[currentWaypointIndex + 1]);
            float angleToWaypoint = Vector3.SignedAngle(transform.forward, nextWaypoint - transform.position, Vector3.up);
            sensor.AddObservation(angleToWaypoint / 180f);
        }
        else
        {
            sensor.AddObservation(0f);
        }

        // 局部障碍物（5x5网格）
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

        // 动态障碍物速度
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
        if (target == null || gridManager == null || isTaskCompleted) return;

        MoveAgent(actions.DiscreteActions[0]);

        // 奖励计算
        float distToTarget = Vector3.Distance(transform.position, target.position);
        float distanceDelta = lastDistToTarget - distToTarget;

        // 距离变化奖励
        float proximityFactor = Mathf.Clamp01(1 - (distToTarget / (Mathf.Max(gridWidth, gridHeight) * 1f)));
        float stepReward = distanceDelta * (1 + proximityFactor * targetProximitySmoothing);
        stepReward = Mathf.Clamp(stepReward, minStepPenalty, maxStepReward);
        AddReward(stepReward);

        // 平滑移动奖励
        float currentSpeed = rb.linearVelocity.magnitude;
        float speedStabilityReward = 0.1f * (1 - Mathf.Abs(currentSpeed - MaxSpeed * 0.5f) / (MaxSpeed * 0.5f));
        AddReward(speedStabilityReward);

        // 碰撞惩罚
        Vector2Int currentGrid = gridManager.世界转栅格(transform.position);
        if (!IsPassable(currentGrid))
        {
            AddReward(-50f);
            EndCurrentEpisode();
            return;
        }

        // 到达目标奖励
        if (distToTarget < 1f)
        {
            float currentSpeedFinal = rb.linearVelocity.magnitude;
            AddReward(currentSpeedFinal < MaxSpeed * 0.3f ? 100f : 50f);
            EndCurrentEpisode();
            return;
        }

        // 超时惩罚
        if (Time.time - episodeStartTime > 30f)
        {
            AddReward(-20f);
            EndCurrentEpisode();
            return;
        }

        UpdateWaypointIndex();
        lastDistToTarget = distToTarget;
    }

    private void EndCurrentEpisode()
    {
        isTaskCompleted = true;
        EndEpisode();

        // 非训练模式下完成任务后不再自动重置
        if (!isTrainingMode)
        {
            Debug.Log("单次任务完成，停止自动重置");
        }
    }

    void MoveAgent(int action)
    {
        float currentSpeed = rb.linearVelocity.magnitude;

        switch (action)
        {
            case 0: // 前进
                if (currentSpeed < MaxSpeed * 0.8f)
                {
                    rb.AddForce(transform.forward * MaxSpeed * 0.6f, ForceMode.VelocityChange);
                }
                else
                {
                    rb.AddForce(transform.forward * MaxSpeed * 0.2f, ForceMode.VelocityChange);
                }
                break;
            case 1: // 左转
                transform.Rotate(0, -MaxAngularSpeed * Time.fixedDeltaTime, 0);
                break;
            case 2: // 右转
                transform.Rotate(0, MaxAngularSpeed * Time.fixedDeltaTime, 0);
                break;
            case 3: // 减速
                rb.linearVelocity = Vector3.Lerp(rb.linearVelocity, Vector3.zero, 0.2f);
                break;
        }
    }

    private void UpdateWaypointIndex()
    {
        if (globalPathfinder == null || globalPathfinder.path == null || globalPathfinder.path.Count == 0)
            return;

        float distanceToWaypoint = Vector3.Distance(transform.position,
            gridManager.栅格转世界(globalPathfinder.path[currentWaypointIndex]));

        if (distanceToWaypoint < 1.5f)
        {
            currentWaypointIndex = Mathf.Min(currentWaypointIndex + 1, globalPathfinder.path.Count - 1);
        }
    }

    private bool IsPassable(Vector2Int gridPos)
    {
        if (gridPos.x < 0 || gridPos.x >= gridWidth || gridPos.y < 0 || gridPos.y >= gridHeight)
            return false;

        return passableGrid != null && passableGrid[gridPos.x, gridPos.y];
    }

    private void CachePassableGrid()
    {
        passableGrid = new bool[gridWidth, gridHeight];
        for (int x = 0; x < gridWidth; x++)
        {
            for (int y = 0; y < gridHeight; y++)
            {
                passableGrid[x, y] = gridManager.栅格是否可通行(new Vector2Int(x, y));
            }
        }
    }

    private void GenerateSafePositions()
    {
        safePositions = new List<Vector3>();
        for (int x = 0; x < gridWidth; x++)
        {
            for (int y = 0; y < gridHeight; y++)
            {
                if (gridManager.栅格是否可通行(new Vector2Int(x, y)))
                {
                    safePositions.Add(gridManager.栅格转世界(new Vector2Int(x, y)));
                }
            }
        }
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var discreteActions = actionsOut.DiscreteActions;
        discreteActions[0] = 0;

        if (Input.GetKey(KeyCode.W)) discreteActions[0] = 0;
        if (Input.GetKey(KeyCode.A)) discreteActions[0] = 1;
        if (Input.GetKey(KeyCode.D)) discreteActions[0] = 2;
        if (Input.GetKey(KeyCode.S)) discreteActions[0] = 3;
    }
}