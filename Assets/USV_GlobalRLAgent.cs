using System.Collections;  // 关键：导入IEnumerator所在的命名空间
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;
using System.Collections.Generic;


public class USV_GlobalRLAgent : Agent
{
    public Transform target; // 目标点（公开，供LocalPlanner访问）
    private GridManager gridManager; // 栅格管理器
    private Rigidbody rb; // 刚体组件
    private int gridWidth; // 缓存栅格宽度（仅定义一次）
    private int gridHeight; // 缓存栅格高度（仅定义一次）
    private bool[,] passableGrid; // 缓存栅格通行性数据
    private List<Vector3> safePositions; // 预生成安全位置列表
    private float lastDistToTarget; // 记录上一帧到目标的距离
    private const int ViewRange = 5; // 局部视野范围
    private const float MaxSpeed = 2f; // 最大速度
    private const float MaxAngularSpeed = 60f; // 限制最大角速度

    // 新增：全局路径方向观测
    private ImprovedAStar globalPathfinder;
    private int currentWaypointIndex = 0;

    // USV_GlobalRLAgent.cs - Awake() 方法修改
    void Awake()
    {
        gridManager = Object.FindFirstObjectByType<GridManager>();
        globalPathfinder = Object.FindFirstObjectByType<ImprovedAStar>();
        rb = GetComponent<Rigidbody>();

        if (gridManager != null)
        {
            // 等待GridManager初始化完成后再缓存数据（关键修复）
            StartCoroutine(WaitForGridInit());
        }
        else
        {
            Debug.LogError("未找到GridManager组件！");
        }
    }

    // 新增协程：等待GridManager初始化
    private IEnumerator WaitForGridInit()  // 协程应返回非泛型IEnumerator
    {
        // 循环等待直到栅格就绪
        while (!gridManager.IsGridReady())
        {
            Debug.Log("等待GridManager初始化...");
            yield return new WaitForSeconds(0.5f);
        }

        // 栅格就绪后再缓存数据
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
        if (gridManager == null || safePositions == null || safePositions.Count == 0)
        {
            Debug.LogWarning("无法重置 episode：GridManager 未初始化或安全位置列表为空");
            return;
        }

        // 随机设置智能体和目标位置
        transform.position = safePositions[Random.Range(0, safePositions.Count)];
        target.position = safePositions[Random.Range(0, safePositions.Count)];

        // 确保起点和目标距离足够（至少5米）
        while (Vector3.Distance(transform.position, target.position) < 5f && safePositions.Count > 1)
        {
            target.position = safePositions[Random.Range(0, safePositions.Count)];
        }

        // 重置速度和路径点索引
        rb.linearVelocity = Vector3.zero;
        rb.angularVelocity = Vector3.zero;
        lastDistToTarget = Vector3.Distance(transform.position, target.position);
        currentWaypointIndex = 0;

        // 重新计算全局路径
        if (globalPathfinder != null)
        {
            globalPathfinder.CalculatePathAfterDelay();
        }
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        if (rb == null || target == null || gridManager == null) return;

        // 1. 前进速度（归一化）
        float forwardSpeed = Vector3.Dot(transform.forward, rb.linearVelocity);
        sensor.AddObservation(Mathf.Clamp(forwardSpeed / MaxSpeed, -1f, 1f));

        // 2. 朝向（归一化到 [-1, 1]）
        sensor.AddObservation((transform.eulerAngles.y % 360f) / 180f - 1f);

        // 3. 目标距离和角度
        float distToTarget = Vector3.Distance(transform.position, target.position);
        sensor.AddObservation(Mathf.Clamp01(distToTarget / (Mathf.Max(gridWidth, gridHeight) * 1f)));

        float angleToTarget = Vector3.SignedAngle(transform.forward, target.position - transform.position, Vector3.up);
        sensor.AddObservation(angleToTarget / 180f);

        // 4. 全局路径方向（新增，修复Path访问）
        if (globalPathfinder != null && globalPathfinder.path != null && globalPathfinder.path.Count > currentWaypointIndex + 1)
        {
            Vector3 nextWaypoint = gridManager.栅格转世界(globalPathfinder.path[currentWaypointIndex + 1]);
            float angleToWaypoint = Vector3.SignedAngle(transform.forward, nextWaypoint - transform.position, Vector3.up);
            sensor.AddObservation(angleToWaypoint / 180f);
        }
        else
        {
            sensor.AddObservation(0f); // 无路径时补0
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

        // 6. 动态障碍物速度（新增，从LocalPlanner获取）
        USV_LocalPlanner localPlanner = GetComponent<USV_LocalPlanner>();
        if (localPlanner != null && localPlanner.dynamicObstacleVelocities.Count > 0)
        {
            Vector3 closestObsVel = localPlanner.dynamicObstacleVelocities[0];
            sensor.AddObservation(closestObsVel.x / 5f); // 归一化到[-1,1]
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

        // 奖励函数优化
        float distToTarget = Vector3.Distance(transform.position, target.position);
        float distanceDelta = lastDistToTarget - distToTarget;

        // 增强靠近目标的奖励
        AddReward(distanceDelta * 0.5f);

        // 碰撞惩罚增强
        Vector2Int currentGrid = gridManager.世界转栅格(transform.position);
        if (!IsPassable(currentGrid))
        {
            AddReward(-50f); // 提高惩罚
            EndEpisode();
            return;
        }

        // 到达目标奖励
        if (distToTarget < 1f)
        {
            AddReward(100f); // 提高奖励
            EndEpisode();
        }

        // 超时惩罚（防止无限循环）
        if (StepCount > 5000)
        {
            AddReward(-20f);
            EndEpisode();
        }

        // 更新路径点索引
        if (globalPathfinder != null && globalPathfinder.path != null && globalPathfinder.path.Count > currentWaypointIndex)
        {
            float distToWaypoint = Vector3.Distance(transform.position, gridManager.栅格转世界(globalPathfinder.path[currentWaypointIndex]));
            if (distToWaypoint < 1.5f)
            {
                currentWaypointIndex = Mathf.Min(currentWaypointIndex + 1, globalPathfinder.path.Count - 1);
            }
        }

        lastDistToTarget = distToTarget;
    }

    // USV_GlobalRLAgent.cs - MoveAgent() 方法修改
    void MoveAgent(int action)
    {
        switch (action)
        {
            case 0: // 前进
                rb.AddForce(transform.forward * MaxSpeed * 0.8f, ForceMode.VelocityChange);
                break;
            case 1: // 左转（进一步降低角速度，从20°→15°）
                transform.Rotate(Vector3.up, -15f * Time.fixedDeltaTime);
                break;
            case 2: // 右转（同上）
                transform.Rotate(Vector3.up, 15f * Time.fixedDeltaTime);
                break;
            case 3: // 减速
                rb.linearVelocity *= 0.9f;
                break;
        }
    }
    // 检查栅格是否可通行
    private bool IsPassable(Vector2Int gridPos)
    {
        if (gridPos.x < 0 || gridPos.x >= gridWidth || gridPos.y < 0 || gridPos.y >= gridHeight)
            return false;
        return passableGrid[gridPos.x, gridPos.y];
    }

    // 缓存栅格通行性数据
    private void CachePassableGrid()
    {
        passableGrid = new bool[gridWidth, gridHeight];
        for (int x = 0; x < gridWidth; x++)
        {
            for (int z = 0; z < gridHeight; z++)
            {
                passableGrid[x, z] = gridManager.栅格是否可通行(new Vector2Int(x, z));
            }
        }
    }

    // 生成安全位置列表
    private void GenerateSafePositions()
    {
        safePositions = new List<Vector3>();
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