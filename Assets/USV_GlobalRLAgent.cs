using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;
using System.Collections.Generic; // 用于缓存安全位置列表

public class USV_GlobalRLAgent : Agent
{
    public Transform target; // 目标点
    private GridManager gridManager; // 栅格管理器
    private Rigidbody rb; // 刚体组件
    private int gridWidth; // 缓存栅格宽度（替代动态获取）
    private int gridHeight; // 缓存栅格高度（替代动态获取）
    private bool[,] passableGrid; // 缓存栅格通行性数据（减少重复查询）
    private List<Vector3> safePositions; // 预生成安全位置列表（加速重置）
    private float lastDistToTarget; // 记录上一帧到目标的距离（优化奖励计算）
    private const int ViewRange = 5; // 局部视野范围（5x5=25格，替代全局栅格）
    private const float MaxSpeed = 2f; // 最大速度（用于归一化）

    void Awake()
    {
        // 获取组件并初始化缓存
        gridManager = FindObjectOfType<GridManager>();
        rb = GetComponent<Rigidbody>();

        if (gridManager != null)
        {
            // 缓存栅格尺寸（避免每次访问GridManager）
            gridWidth = gridManager.栅格宽度;
            gridHeight = gridManager.栅格高度;
            // 预缓存所有栅格的通行性（替代每次查询）
            CachePassableGrid();
            // 预生成所有安全位置（加速OnEpisodeBegin）
            GenerateSafePositions();
        }
    }

    // 初始化代理
    public override void Initialize()
    {
        if (rb != null)
        {
            rb.maxAngularVelocity = 5f;
            // 禁用不必要的物理计算（优化性能）
            rb.useGravity = false;
            rb.interpolation = RigidbodyInterpolation.None;
        }
    }

    // 每轮开始时调用（优化重置效率）
    public override void OnEpisodeBegin()
    {
        if (gridManager == null || safePositions == null || safePositions.Count == 0) return;

        // 从预生成的安全位置中随机选择（替代重复计算）
        transform.position = safePositions[Random.Range(0, safePositions.Count)];
        target.position = safePositions[Random.Range(0, safePositions.Count)];

        // 重置物理状态
        rb.velocity = Vector3.zero;
        rb.angularVelocity = Vector3.zero;

        // 初始化距离记录（用于增量奖励）
        lastDistToTarget = Vector3.Distance(transform.position, target.position);
    }

    // 收集环境观测数据（核心优化：缩减观测维度）
    public override void CollectObservations(VectorSensor sensor)
    {
        if (rb == null || target == null || gridManager == null) return;

        // 1. 自身速度（优化：只关注前进方向速度，减少维度）
        float forwardSpeed = Vector3.Dot(transform.forward, rb.velocity);
        sensor.AddObservation(Mathf.Clamp(forwardSpeed / MaxSpeed, -1f, 1f));

        // 2. 自身朝向角（优化：归一化到-1~1范围）
        sensor.AddObservation((transform.eulerAngles.y % 360f) / 180f - 1f);

        // 3. 到目标的距离（优化：使用缓存的栅格尺寸归一化）
        float distToTarget = Vector3.Distance(transform.position, target.position);
        sensor.AddObservation(Mathf.Clamp01(distToTarget / (Mathf.Max(gridWidth, gridHeight) * 1f)));

        // 4. 到目标的角度（保持原有逻辑）
        float angleToTarget = Vector3.SignedAngle(transform.forward, target.position - transform.position, Vector3.up);
        sensor.AddObservation(angleToTarget / 180f);

        // 5. 局部视野障碍物（核心优化：替代全局栅格，维度从W*H降至25）
        Vector2Int agentGridPos = gridManager.世界转栅格(transform.position);
        Vector2Int checkPos = new Vector2Int(); // 复用变量减少GC

        for (int x = -ViewRange; x <= ViewRange; x++)
        {
            for (int z = -ViewRange; z <= ViewRange; z++)
            {
                checkPos.x = agentGridPos.x + x;
                checkPos.y = agentGridPos.y + z;
                // 使用缓存的通行性数据（避免重复调用GridManager）
                bool isObstacle = !IsPassable(checkPos);
                sensor.AddObservation(isObstacle ? 1f : 0f);
            }
        }
    }

    // 接收动作并执行（优化奖励函数和计算效率）
    public override void OnActionReceived(ActionBuffers actions)
    {
        if (target == null || gridManager == null) return;

        // 执行移动动作（优化：使用刚体物理驱动，更稳定）
        MoveAgent(actions.DiscreteActions[0]);

        // 计算当前距离并优化奖励函数
        float distToTarget = Vector3.Distance(transform.position, target.position);

        // 1. 增量奖励：向目标移动时额外奖励（引导智能体高效探索）
        float distanceDelta = lastDistToTarget - distToTarget;
        AddReward(distanceDelta * 0.1f); // 系数可调整
        lastDistToTarget = distToTarget;

        // 2. 距离奖励：保留原有逻辑但降低权重
        AddReward((1f - Mathf.Clamp01(distToTarget / (Mathf.Max(gridWidth, gridHeight) * 1f))) * 0.05f);

        // 3. 碰撞障碍物惩罚（使用缓存数据判断）
        Vector2Int currentGrid = gridManager.世界转栅格(transform.position);
        if (!IsPassable(currentGrid))
        {
            AddReward(-10f);
            EndEpisode();
            return;
        }

        // 4. 到达目标奖励
        if (distToTarget < 1f)
        {
            AddReward(20f);
            EndEpisode();
        }
    }

    // 移动方法（优化：使用刚体力驱动，更符合物理规律）
    void MoveAgent(int action)
    {
        switch (action)
        {
            case 0: // 前进（使用刚体力，避免直接修改Transform）
                rb.AddForce(transform.forward * MaxSpeed, ForceMode.VelocityChange);
                break;
            case 1: // 左转
                transform.Rotate(Vector3.up, -90f * Time.fixedDeltaTime);
                break;
            case 2: // 右转
                transform.Rotate(Vector3.up, 90f * Time.fixedDeltaTime);
                break;
        }
    }

    // 预缓存栅格通行性数据（减少重复查询GridManager）
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

    // 预生成所有安全位置（加速OnEpisodeBegin的重置过程）
    private void GenerateSafePositions()
    {
        safePositions = new List<Vector3>();
        for (float x = -14.5f; x <= 14.5f; x += 0.5f)
        {
            for (float z = -9.5f; z <= 9.5f; z += 0.5f)
            {
                Vector3 pos = new Vector3(x, 0.4f, z);
                Vector2Int gridPos = gridManager.世界转栅格(pos);
                if (IsPassable(gridPos))
                {
                    safePositions.Add(pos);
                }
            }
        }
    }

    // 快速判断栅格是否可通行（使用缓存数据）
    private bool IsPassable(Vector2Int gridPos)
    {
        // 边界检查（超出范围视为障碍物）
        if (gridPos.x < 0 || gridPos.x >= gridWidth || gridPos.y < 0 || gridPos.y >= gridHeight)
            return false;
        return passableGrid[gridPos.x, gridPos.y];
    }
}