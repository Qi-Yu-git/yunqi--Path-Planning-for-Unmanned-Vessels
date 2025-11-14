using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;
using System.Collections.Generic;

public class USV_GlobalRLAgent : Agent
{
    public Transform target; // 目标点
    private GridManager gridManager; // 栅格管理器
    private Rigidbody rb; // 刚体组件
    private int gridWidth; // 缓存栅格宽度（仅定义一次）
    private int gridHeight; // 缓存栅格高度（仅定义一次）
    private bool[,] passableGrid; // 缓存栅格通行性数据
    private List<Vector3> safePositions; // 预生成安全位置列表
    private float lastDistToTarget; // 记录上一帧到目标的距离
    private const int ViewRange = 5; // 局部视野范围
    private const float MaxSpeed = 2f; // 最大速度

    void Awake()
    {
        // 修复过时API：替换FindObjectOfType
        gridManager = Object.FindFirstObjectByType<GridManager>();
        rb = GetComponent<Rigidbody>();

        if (gridManager != null)
        {
            // 读取GridManager的公开变量（需在GridManager中定义）
            gridWidth = gridManager.gridWidth;
            gridHeight = gridManager.gridHeight;
            CachePassableGrid();
            GenerateSafePositions();
        }
        else
        {
            Debug.LogError("未找到GridManager组件！");
        }
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

        // 重置速度
        rb.linearVelocity = Vector3.zero;
        rb.angularVelocity = Vector3.zero;

        // 初始化距离
        lastDistToTarget = Vector3.Distance(transform.position, target.position);
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        if (rb == null || target == null || gridManager == null) return;

        // 观测前进速度（归一化）
        float forwardSpeed = Vector3.Dot(transform.forward, rb.linearVelocity);
        sensor.AddObservation(Mathf.Clamp(forwardSpeed / MaxSpeed, -1f, 1f));

        // 观测朝向（归一化到 [-1, 1]）
        sensor.AddObservation((transform.eulerAngles.y % 360f) / 180f - 1f);

        // 观测到目标的距离（归一化）
        float distToTarget = Vector3.Distance(transform.position, target.position);
        sensor.AddObservation(Mathf.Clamp01(distToTarget / (Mathf.Max(gridWidth, gridHeight) * 1f)));

        // 观测到目标的角度（归一化到 [-1, 1]）
        float angleToTarget = Vector3.SignedAngle(transform.forward, target.position - transform.position, Vector3.up);
        sensor.AddObservation(angleToTarget / 180f);

        // 观测局部视野内的障碍物（5x5网格）
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
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        if (target == null || gridManager == null) return;

        // 执行移动动作
        MoveAgent(actions.DiscreteActions[0]);

        // 计算奖励
        float distToTarget = Vector3.Distance(transform.position, target.position);
        float distanceDelta = lastDistToTarget - distToTarget;
        AddReward(distanceDelta * 0.1f); // 靠近目标奖励
        lastDistToTarget = distToTarget;

        // 基于距离的持续奖励
        AddReward((1f - Mathf.Clamp01(distToTarget / (Mathf.Max(gridWidth, gridHeight) * 1f))) * 0.05f);

        // 碰撞障碍物惩罚
        Vector2Int currentGrid = gridManager.世界转栅格(transform.position);
        if (!IsPassable(currentGrid))
        {
            AddReward(-10f);
            EndEpisode();
            return;
        }

        // 到达目标奖励
        if (distToTarget < 1f)
        {
            AddReward(20f);
            EndEpisode();
        }
    }

    void MoveAgent(int action)
    {
        switch (action)
        {
            case 0: // 前进
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
        Debug.Log($"生成 {safePositions.Count} 个安全位置");
    }

    // 检查栅格是否可通行
    private bool IsPassable(Vector2Int gridPos)
    {
        if (gridPos.x < 0 || gridPos.x >= gridWidth || gridPos.y < 0 || gridPos.y >= gridHeight)
            return false;
        return passableGrid[gridPos.x, gridPos.y];
    }
}