using UnityEngine;
using System.Collections.Generic;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using PPUV.YOLOv8;

[RequireComponent(typeof(USV_GlobalRLAgent), typeof(Rigidbody))]
public partial class USV_LocalPlanner : MonoBehaviour
{
    // 核心依赖组件
    private USV_GlobalRLAgent globalAgent;
    private GridManager gridManager;
    private Rigidbody rb;
    private ImprovedAStar globalPathfinder;
    public ImageInputSelector imageInputSelector;

    // 局部避障参数（优化后）
    public float localSafeDistance = 4f;         // 扩大安全距离
    public float colregsSafeDistance = 5f;       // 扩大COLREGs安全距离
    public float dwaPredictTime = 1.2f;          // 延长DWA预测时间
    public float returnToPathThreshold = 2f;

    // 动态障碍存储
    private List<Vector3> dynamicObstacles = new List<Vector3>();
    private List<Vector3> dynamicObstacleVelocities = new List<Vector3>();
    private bool isAvoidingDynamicObstacle = false;
    private int currentGlobalWaypointIndex = 0;

    // DWA速度窗口配置（增加低速选项）
    private readonly float[] linearVelOptions = { 0f, 0.2f, 0.5f, 0.8f, 1.2f, 1.5f }; // 新增更低速度选项
    private readonly float[] angularVelOptions = { -60f, -45f, -30f, -15f, 0f, 15f, 30f, 45f, 60f }; // 增加转向幅度
    private const float MaxLinearVel = 1.5f;

    // COLREGs规则参数
    private const float StarboardAvoidAngle = 45f;  // 增大转向角度
    private const float PortAvoidAngle = -45f;
    private const float HeadOnAvoidAngle = -60f;

    void Awake()
    {
        globalAgent = GetComponent<USV_GlobalRLAgent>();
        rb = GetComponent<Rigidbody>();
        gridManager = FindFirstObjectByType<GridManager>();
        globalPathfinder = FindFirstObjectByType<ImprovedAStar>();

        if (imageInputSelector == null) Debug.LogError("局部规划脚本：请赋值ImageInputSelector组件！");
        if (gridManager == null) Debug.LogError("局部规划脚本：未找到GridManager！");
        if (globalPathfinder == null) Debug.LogError("局部规划脚本：未找到全局路径规划器！");

        // 物理参数优化
        rb.linearDamping = 0.8f;          // 增加阻力，便于急停
        rb.angularDamping = 1.5f;   // 增加旋转阻力，避免过度转向
    }

    void Start()
    {
        if (globalPathfinder.path != null && globalPathfinder.path.Count > 0)
        {
            currentGlobalWaypointIndex = 0;
        }
    }

    public void OnAgentActionReceived(ActionBuffers actions)
    {
        // 1. 检测并预测动态障碍（使用增强版检测）
        DetectAndPredictDynamicObstacles();

        // 2. 确定运动目标
        Vector3 targetVelocity;
        float targetRotation;

        if (dynamicObstacles.Count > 0)
        {
            isAvoidingDynamicObstacle = true;
            (targetVelocity, targetRotation) = LocalPlannerWithCOLREGs();
        }
        else
        {
            if (isAvoidingDynamicObstacle && IsCloseToGlobalPath())
            {
                isAvoidingDynamicObstacle = false;
                Debug.Log("局部避障完成，回归全局路径");
            }
            (targetVelocity, targetRotation) = GetGlobalActionVelocity(actions.DiscreteActions[0]);
        }

        // 3. 应用运动状态（增加紧急减速逻辑）
        if (IsExtremeDanger())
        {
            targetVelocity *= 0.3f; // 极度危险时大幅减速
            Debug.LogWarning("触发紧急减速");
        }
        targetVelocity = Vector3.ClampMagnitude(targetVelocity, MaxLinearVel);
        rb.linearVelocity = new Vector3(targetVelocity.x, rb.linearVelocity.y, targetVelocity.z);
        transform.Rotate(0, targetRotation * Time.deltaTime, 0);

        // 4. 局部避障奖励
        AddLocalPlanningRewards();
    }

    /// <summary>
    /// 增强版动态障碍检测（优化预测精度）
    /// </summary>
    private void DetectAndPredictDynamicObstacles()
    {
        dynamicObstacles.Clear();
        dynamicObstacleVelocities.Clear();

        if (imageInputSelector == null) return;

        List<Vector3> currentObstacles = imageInputSelector.GetDetectedObstaclePositions();
        List<int> currentIds = imageInputSelector.GetDetectedObstacleIds();

        for (int i = 0; i < currentObstacles.Count; i++)
        {
            int obsId = currentIds[i];
            Vector3 currentPos = currentObstacles[i];
            Vector2Int gridPos = gridManager.世界转栅格(currentPos);

            if (gridManager.栅格是否可通行(gridPos))
            {
                // 优化版速度估算（线性回归）
                Vector3 predictedVel = EstimateObstacleVelocity(obsId, currentPos);
                dynamicObstacleVelocities.Add(predictedVel);

                // 预测多个时间点的位置，取最远危险点
                Vector3 predictedPos = PredictObstaclePosition(currentPos, predictedVel, predictionTime);
                dynamicObstacles.Add(predictedPos);

                // 提前检测碰撞风险
                if (IsCollisionImminent(transform.position, rb.linearVelocity,
                    currentPos, predictedVel, predictionTime))
                {
                    Debug.Log($"检测到 {predictionTime} 秒内可能发生碰撞，准备避障");
                    isAvoidingDynamicObstacle = true;
                }
            }
        }

        CleanupObstacleHistory();
    }

    /// <summary>
    /// 优化版障碍速度估算（线性回归方法）
    /// </summary>
    private Vector3 EstimateObstacleVelocity(int obsId, Vector3 currentPos)
    {
        if (!obstacleHistory.ContainsKey(obsId))
        {
            obstacleHistory[obsId] = new List<(Vector3 pos, float time)>();
        }

        // 记录带时间戳的位置
        obstacleHistory[obsId].Add((currentPos, Time.time));
        obstacleLastSeen[obsId] = Time.time;

        // 保留最近10帧数据提高精度
        while (obstacleHistory[obsId].Count > 10)
        {
            obstacleHistory[obsId].RemoveAt(0);
        }

        // 至少3个点才能做线性回归
        if (obstacleHistory[obsId].Count < 3)
        {
            return Vector3.zero;
        }

        // 线性回归计算速度
        int n = obstacleHistory[obsId].Count;
        float sumT = 0, sumX = 0, sumZ = 0;
        float sumT2 = 0, sumTX = 0, sumTZ = 0;

        for (int i = 0; i < n; i++)
        {
            float t = obstacleHistory[obsId][i].time;
            Vector3 pos = obstacleHistory[obsId][i].pos;
            sumT += t;
            sumX += pos.x;
            sumZ += pos.z;
            sumT2 += t * t;
            sumTX += t * pos.x;
            sumTZ += t * pos.z;
        }

        float denominator = n * sumT2 - sumT * sumT;
        if (denominator < 0.001f) return Vector3.zero;

        float aX = (n * sumTX - sumT * sumX) / denominator;
        float aZ = (n * sumTZ - sumT * sumZ) / denominator;

        return new Vector3(aX, 0, aZ) * obstacleSpeedFactor;
    }

    /// <summary>
    /// 预测障碍未来位置
    /// </summary>
    private Vector3 PredictObstaclePosition(Vector3 currentPos, Vector3 velocity, float time)
    {
        return currentPos + velocity * time;
    }

    /// <summary>
    /// 优化版碰撞风险检测（提前预警）
    /// </summary>
    private bool IsCollisionImminent(Vector3 usvPos, Vector3 usvVel,
                                    Vector3 obsPos, Vector3 obsVel, float time)
    {
        Vector3 relativeVel = obsVel - usvVel;
        Vector3 relativePos = obsPos - usvPos;

        // 计算碰撞时间TTC
        float dotProduct = Vector3.Dot(relativePos, relativeVel);
        if (dotProduct >= 0) return false; // 目标远离

        float ttc = -Vector3.Dot(relativePos, relativePos) / dotProduct;
        if (ttc < 0 || ttc > time) return false;

        // 预测未来位置
        Vector3 usvFuture = usvPos + usvVel * ttc;
        Vector3 obsFuture = obsPos + obsVel * ttc;

        // 扩大预警范围（1.5倍阈值）
        return Vector3.Distance(usvFuture, obsFuture) < collisionThreshold * 1.5f;
    }

    /// <summary>
    /// 检查是否处于极度危险状态
    /// </summary>
    private bool IsExtremeDanger()
    {
        if (dynamicObstacles.Count == 0) return false;

        float minDist = float.MaxValue;
        foreach (var obs in dynamicObstacles)
        {
            minDist = Mathf.Min(minDist, Vector3.Distance(transform.position, obs));
        }
        return minDist < collisionThreshold * 0.6f; // 距离小于阈值60%时触发
    }

    /// <summary>
    /// 局部规划核心（优化权重）
    /// </summary>
    private (Vector3 velocity, float rotation) LocalPlannerWithCOLREGs()
    {
        float bestScore = -Mathf.Infinity;
        Vector3 bestVelocity = Vector3.zero;
        float bestRotation = 0f;

        Vector3 nextGlobalWaypoint = GetCurrentGlobalWaypoint();

        foreach (float linearVel in linearVelOptions)
        {
            foreach (float angularVel in angularVelOptions)
            {
                (Vector3 predictedPos, Quaternion predictedRot) = PredictMotion(linearVel, angularVel, dwaPredictTime);

                // 提高避障权重至0.7
                float obstacleScore = CalculateObstacleScore(predictedPos);
                float pathTrackScore = CalculatePathTrackScore(predictedPos, nextGlobalWaypoint);
                float colregsScore = CalculateCOLREGsScore(predictedPos, predictedRot, angularVel);
                float smoothScore = CalculateSmoothScore(linearVel, angularVel);

                float totalScore = obstacleScore * 0.7f
                                 + pathTrackScore * 0.15f
                                 + colregsScore * 0.1f
                                 + smoothScore * 0.05f;

                if (totalScore > bestScore)
                {
                    bestScore = totalScore;
                    bestVelocity = predictedRot * Vector3.forward * linearVel;
                    bestRotation = angularVel;
                }
            }
        }

        return (bestVelocity, bestRotation);
    }

    /// <summary>
    /// 优化版避障评分（更严格的安全距离要求）
    /// </summary>
    private float CalculateObstacleScore(Vector3 predictedPos)
    {
        float totalScore = 0f;
        foreach (Vector3 obsPos in dynamicObstacles)
        {
            float dist = Vector3.Distance(predictedPos, obsPos);
            // 距离低于安全阈值时得分急剧下降
            if (dist < avoidDistance * 0.5f)
                totalScore += 0f; // 极近距离得0分
            else if (dist < avoidDistance)
                totalScore += (dist / avoidDistance) * 0.5f; // 中等距离得低分
            else
                totalScore += 1f; // 安全距离外得满分
        }
        return dynamicObstacles.Count > 0 ? totalScore / dynamicObstacles.Count : 1f;
    }

    /// <summary>
    /// COLREGs规则评分（增强转向惩罚）
    /// </summary>
    private float CalculateCOLREGsScore(Vector3 predictedPos, Quaternion predictedRot, float currentAngularVel)
    {
        float colregsScore = 1f;

        for (int i = 0; i < dynamicObstacles.Count; i++)
        {
            Vector3 obsPos = dynamicObstacles[i];
            Vector3 obsVel = dynamicObstacleVelocities[i];

            Vector3 relativePos = obsPos - predictedPos;
            float relativeAngle = Vector3.SignedAngle(predictedRot * Vector3.forward, relativePos, Vector3.up);
            float obsSpeed = obsVel.magnitude;

            // 对遇局面惩罚更严格
            if (Mathf.Abs(relativeAngle) < 20f && obsSpeed > 0.5f)
            {
                float rotationDiff = Mathf.Abs(currentAngularVel - HeadOnAvoidAngle);
                colregsScore *= Mathf.Clamp(1 - (rotationDiff / 60f), 0.2f, 1f);
            }
            // 右舷来船保持航向
            else if (relativeAngle > 0f && relativeAngle < 120f)
            {
                colregsScore *= Mathf.Clamp(1 - (Mathf.Abs(currentAngularVel) / 20f), 0.3f, 1f);
            }
            // 左舷来船主动避让
            else if (relativeAngle < 0f && relativeAngle > -120f)
            {
                float rotationDiff = Mathf.Abs(currentAngularVel - StarboardAvoidAngle);
                colregsScore *= Mathf.Clamp(1 - (rotationDiff / 50f), 0.2f, 1f);
            }

            // 安全距离惩罚
            float distToObs = Vector3.Distance(predictedPos, obsPos);
            if (distToObs < colregsSafeDistance)
            {
                colregsScore *= Mathf.Pow(distToObs / colregsSafeDistance, 2); // 平方关系增强惩罚
            }
        }

        return colregsScore;
    }

    private (Vector3 pos, Quaternion rot) PredictMotion(float linearVel, float angularVel, float time)
    {
        Vector3 predictedPos = transform.position;
        Quaternion predictedRot = transform.rotation;

        int steps = 8; // 增加预测步数提高精度
        float stepTime = time / steps;

        for (int i = 0; i < steps; i++)
        {
            predictedRot *= Quaternion.Euler(0, angularVel * stepTime, 0);
            predictedPos += predictedRot * Vector3.forward * linearVel * stepTime;
        }

        return (predictedPos, predictedRot);
    }

    private float CalculatePathTrackScore(Vector3 predictedPos, Vector3 nextWaypoint)
    {
        float distToWaypoint = Vector3.Distance(predictedPos, nextWaypoint);
        return Mathf.Clamp(1 - (distToWaypoint / (localSafeDistance * 2)), 0f, 1f);
    }

    private float CalculateSmoothScore(float linearVel, float angularVel)
    {
        float currentLinearVel = Vector3.Dot(transform.forward, rb.linearVelocity);
        float linearSmooth = 1 - Mathf.Abs(linearVel - currentLinearVel) / MaxLinearVel;
        float angularSmooth = 1 - Mathf.Abs(angularVel) / 60f; // 适配更大转向范围
        return (linearSmooth + angularSmooth) / 2f;
    }

    private Vector3 GetCurrentGlobalWaypoint()
    {
        if (globalPathfinder.path == null || globalPathfinder.path.Count == 0)
        {
            return globalAgent.target.position;
        }

        Vector3 currentWaypoint = gridManager.栅格转世界(globalPathfinder.path[currentGlobalWaypointIndex]);
        float distToWaypoint = Vector3.Distance(transform.position, currentWaypoint);

        if (distToWaypoint < 1f && currentGlobalWaypointIndex < globalPathfinder.path.Count - 1)
        {
            currentGlobalWaypointIndex++;
        }

        return currentWaypoint;
    }

    private bool IsCloseToGlobalPath()
    {
        if (globalPathfinder.path == null || globalPathfinder.path.Count < 2)
        {
            return true;
        }

        float minDistToPath = float.MaxValue;
        for (int i = 0; i < globalPathfinder.path.Count - 1; i++)
        {
            Vector3 waypointA = gridManager.栅格转世界(globalPathfinder.path[i]);
            Vector3 waypointB = gridManager.栅格转世界(globalPathfinder.path[i + 1]);
            float distToSegment = DistanceToLineSegment(transform.position, waypointA, waypointB);
            minDistToPath = Mathf.Min(minDistToPath, distToSegment);
        }

        return minDistToPath < returnToPathThreshold;
    }

    private float DistanceToLineSegment(Vector3 point, Vector3 lineStart, Vector3 lineEnd)
    {
        Vector3 lineDir = lineEnd - lineStart;
        float lineLength = lineDir.magnitude;
        if (lineLength < 0.01f)
        {
            return Vector3.Distance(point, lineStart);
        }

        float t = Mathf.Clamp01(Vector3.Dot(point - lineStart, lineDir) / (lineLength * lineLength));
        Vector3 closestPoint = lineStart + t * lineDir;
        return Vector3.Distance(point, closestPoint);
    }

    private void AddLocalPlanningRewards()
    {
        if (dynamicObstacles.Count == 0) return;

        // 增强避障奖励
        float minDistToObs = float.MaxValue;
        foreach (Vector3 obsPos in dynamicObstacles)
        {
            minDistToObs = Mathf.Min(minDistToObs, Vector3.Distance(transform.position, obsPos));
        }
        globalAgent.AddReward(minDistToObs / localSafeDistance * 2f); // 提高奖励系数

        // COLREGs合规奖励
        float currentAngularVel = transform.eulerAngles.y * Time.deltaTime;
        float colregsReward = CalculateCOLREGsScore(transform.position, transform.rotation, currentAngularVel);
        globalAgent.AddReward(colregsReward * 0.8f); // 提高合规奖励

        // 路径保持奖励
        if (IsCloseToGlobalPath())
        {
            globalAgent.AddReward(0.5f);
        }

        // 碰撞惩罚（更严厉）
        if (minDistToObs < collisionThreshold * 0.5f)
        {
            globalAgent.AddReward(-20f);
            EndEpisode();
        }
    }

    private (Vector3 velocity, float rotation) GetGlobalActionVelocity(int action)
    {
        Vector3 velocity = Vector3.zero;
        float rotation = 0f;

        switch (action)
        {
            case 0: // 前进
                velocity = transform.forward * MaxLinearVel;
                break;
            case 1: // 左转
                rotation = -30f;
                velocity = transform.forward * MaxLinearVel * 0.7f;
                break;
            case 2: // 右转
                rotation = 30f;
                velocity = transform.forward * MaxLinearVel * 0.7f;
                break;
        }

        return (velocity, rotation);
    }

    public void ForwardActionToLocalPlanner(ActionBuffers actions)
    {
        OnAgentActionReceived(actions);
    }

    private void EndEpisode()
    {
        globalAgent.EndEpisode();
    }
}

public partial class USV_LocalPlanner : MonoBehaviour
{
    [Header("提前避障参数")]
    public float predictionTime = 3.0f; // 延长预测时间至3秒
    public float collisionThreshold = 3.0f; // 扩大碰撞阈值
    public float avoidDistance = 4.0f; // 增大安全避让距离
    public float obstacleSpeedFactor = 0.9f; // 优化速度估算系数

    // 带时间戳的障碍历史记录（用于更精确的速度估算）
    private Dictionary<int, List<(Vector3 pos, float time)>> obstacleHistory = new Dictionary<int, List<(Vector3 pos, float time)>>();
    private Dictionary<int, float> obstacleLastSeen = new Dictionary<int, float>();
    private float obstacleHistoryTimeout = 2.0f; // 障碍超时时间

    /// <summary>
    /// 清理超时障碍历史
    /// </summary>
    private void CleanupObstacleHistory()
    {
        List<int> toRemove = new List<int>();
        foreach (var kvp in obstacleLastSeen)
        {
            if (Time.time - kvp.Value > obstacleHistoryTimeout)
            {
                toRemove.Add(kvp.Key);
            }
        }
        foreach (int id in toRemove)
        {
            obstacleHistory.Remove(id);
            obstacleLastSeen.Remove(id);
        }
    }
}