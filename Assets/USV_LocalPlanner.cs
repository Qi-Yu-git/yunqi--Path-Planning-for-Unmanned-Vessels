using UnityEngine;
using System.Collections.Generic;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;

[RequireComponent(typeof(USV_GlobalRLAgent), typeof(Rigidbody))]
public partial class USV_LocalPlanner : MonoBehaviour
{
    // 移除未定义的ImageInputSelector引用（若需动态障碍物检测，需自行实现该类）
    private USV_GlobalRLAgent globalAgent;
    private GridManager gridManager;
    private Rigidbody rb;
    private ImprovedAStar globalPathfinder;

    // 调整物理参数
    public float localSafeDistance = 5f;
    public float colregsSafeDistance = 6f;
    public float dwaPredictTime = 1.5f;
    public float returnToPathThreshold = 2f;

    // 动态障碍物信息（公开以便RL观测）
    public List<Vector3> dynamicObstacles = new List<Vector3>();
    public List<Vector3> dynamicObstacleVelocities = new List<Vector3>();
    private bool isAvoidingDynamicObstacle = false;
    private int currentGlobalWaypointIndex = 0;

    // 优化动作空间
    private readonly float[] linearVelOptions = { 0f, 0.3f, 0.6f, 0.9f, 1.2f };
    private readonly float[] angularVelOptions = { -45f, -30f, -15f, 0f, 15f, 30f, 45f };
    private const float MaxLinearVel = 1.2f;

    // COLREGs参数
    private const float StarboardAvoidAngle = 30f;
    private const float PortAvoidAngle = -30f;
    private const float HeadOnAvoidAngle = -45f;

    void Awake()
    {
        globalAgent = GetComponent<USV_GlobalRLAgent>();
        rb = GetComponent<Rigidbody>();
        gridManager = FindFirstObjectByType<GridManager>();
        globalPathfinder = FindFirstObjectByType<ImprovedAStar>();

        // 优化物理参数
        rb.linearDamping = 0.6f; // 降低阻力，增强动作反馈
        rb.angularDamping = 1.2f;

        // 初始化动态障碍物列表
        dynamicObstacles = new List<Vector3>();
        dynamicObstacleVelocities = new List<Vector3>();
    }

    void Start()
    {
        if (globalPathfinder != null && globalPathfinder.path != null && globalPathfinder.path.Count > 0)
        {
            currentGlobalWaypointIndex = 0;
        }
    }

    public void OnAgentActionReceived(ActionBuffers actions)
    {
        // 注释动态障碍物检测（若需启用，需先实现ImageInputSelector类）
        // DetectAndPredictDynamicObstacles();

        Vector3 targetVelocity;
        float targetRotation;

        // 降低局部规划优先级，减少对RL的干扰（当前默认关闭动态障碍物检测）
        if (dynamicObstacles.Count > 0 && IsCollisionImminent(transform.position, rb.linearVelocity, dynamicObstacles[0], dynamicObstacleVelocities[0], 2f))
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

        // 应用运动
        if (IsExtremeDanger())
        {
            targetVelocity *= 0.2f;
            Debug.LogWarning("触发紧急减速");
        }
        targetVelocity = Vector3.ClampMagnitude(targetVelocity, MaxLinearVel);
        rb.linearVelocity = new Vector3(targetVelocity.x, rb.linearVelocity.y, targetVelocity.z);
        transform.Rotate(0, targetRotation * Time.deltaTime, 0);

        AddLocalPlanningRewards();
    }

    // 注释动态障碍物检测方法（需ImageInputSelector类支持）
    private void DetectAndPredictDynamicObstacles()
    {
        dynamicObstacles.Clear();
        dynamicObstacleVelocities.Clear();

        // 若需启用动态障碍物检测，需实现ImageInputSelector类并解除以下注释
        /*
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
                Vector3 predictedVel = EstimateObstacleVelocity(obsId, currentPos);
                dynamicObstacleVelocities.Add(predictedVel);

                Vector3 predictedPos = PredictObstaclePosition(currentPos, predictedVel, 2f);
                dynamicObstacles.Add(predictedPos);

                if (IsCollisionImminent(transform.position, rb.linearVelocity, currentPos, predictedVel, 2f))
                {
                    isAvoidingDynamicObstacle = true;
                }
            }
        }
        */

        CleanupObstacleHistory();
    }

    // 障碍物速度估算（线性回归）
    private Vector3 EstimateObstacleVelocity(int obsId, Vector3 currentPos)
    {
        if (!obstacleHistory.ContainsKey(obsId))
        {
            obstacleHistory[obsId] = new List<(Vector3 pos, float time)>();
        }

        obstacleHistory[obsId].Add((currentPos, Time.time));
        obstacleLastSeen[obsId] = Time.time;

        while (obstacleHistory[obsId].Count > 15) // 增加历史数据量
        {
            obstacleHistory[obsId].RemoveAt(0);
        }

        if (obstacleHistory[obsId].Count < 5) // 提高线性回归精度要求
        {
            return Vector3.zero;
        }

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

        return new Vector3(aX, 0, aZ) * 0.8f; // 降低速度估算系数，避免过度避障
    }

    // 预测障碍物未来位置
    private Vector3 PredictObstaclePosition(Vector3 currentPos, Vector3 velocity, float time)
    {
        return currentPos + velocity * time;
    }

    // 碰撞风险检测
    private bool IsCollisionImminent(Vector3 usvPos, Vector3 usvVel, Vector3 obsPos, Vector3 obsVel, float time)
    {
        Vector3 relativeVel = obsVel - usvVel;
        Vector3 relativePos = obsPos - usvPos;

        float dotProduct = Vector3.Dot(relativePos, relativeVel);
        if (dotProduct >= 0) return false; // 目标远离，无碰撞风险

        float ttc = -Vector3.Dot(relativePos, relativePos) / dotProduct;
        if (ttc < 0 || ttc > time) return false; // 碰撞时间超出预测范围

        Vector3 usvFuture = usvPos + usvVel * ttc;
        Vector3 obsFuture = obsPos + obsVel * ttc;

        return Vector3.Distance(usvFuture, obsFuture) < 2.5f; // 调整碰撞阈值
    }

    // 局部规划核心（结合COLREGs规则）
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

                // 调整权重：降低避障权重，提高路径跟踪权重
                float obstacleScore = CalculateObstacleScore(predictedPos);
                float pathTrackScore = CalculatePathTrackScore(predictedPos, nextGlobalWaypoint);
                float colregsScore = CalculateCOLREGsScore(predictedPos, predictedRot, angularVel);
                float smoothScore = CalculateSmoothScore(linearVel, angularVel);

                float totalScore = obstacleScore * 0.5f
                                 + pathTrackScore * 0.3f
                                 + colregsScore * 0.15f
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

    // 障碍物规避评分（距离越远分数越高）
    private float CalculateObstacleScore(Vector3 predictedPos)
    {
        if (dynamicObstacles.Count == 0) return 1f;

        float totalScore = 0f;
        foreach (var obsPos in dynamicObstacles)
        {
            float dist = Vector3.Distance(predictedPos, obsPos);
            if (dist < avoidDistance * 0.5f)
                totalScore += 0f; // 极近距离得0分
            else if (dist < avoidDistance)
                totalScore += (dist / avoidDistance) * 0.5f; // 中等距离得低分
            else
                totalScore += 1f; // 安全距离外得满分
        }
        return totalScore / dynamicObstacles.Count;
    }

    // 路径跟踪评分（距离路径越近分数越高）
    private float CalculatePathTrackScore(Vector3 predictedPos, Vector3 nextWaypoint)
    {
        float distToWaypoint = Vector3.Distance(predictedPos, nextWaypoint);
        return Mathf.Clamp(1 - (distToWaypoint / (localSafeDistance * 2)), 0f, 1f);
    }

    // COLREGs规则评分（合规行为加分）
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

            // 对遇局面：左转避让
            if (Mathf.Abs(relativeAngle) < 20f && obsSpeed > 0.5f)
            {
                float rotationDiff = Mathf.Abs(currentAngularVel - HeadOnAvoidAngle);
                colregsScore *= Mathf.Clamp(1 - (rotationDiff / 60f), 0.2f, 1f);
            }
            // 右舷来船：保持航向
            else if (relativeAngle > 0f && relativeAngle < 120f)
            {
                colregsScore *= Mathf.Clamp(1 - (Mathf.Abs(currentAngularVel) / 20f), 0.3f, 1f);
            }
            // 左舷来船：右转避让
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

    // 动作平滑度评分（动作变化越小分数越高）
    private float CalculateSmoothScore(float linearVel, float angularVel)
    {
        float currentLinearVel = Vector3.Dot(transform.forward, rb.linearVelocity);
        float linearSmooth = 1 - Mathf.Abs(linearVel - currentLinearVel) / MaxLinearVel;
        float angularSmooth = 1 - Mathf.Abs(angularVel) / 60f; // 适配最大转向角度
        return (linearSmooth + angularSmooth) / 2f;
    }

    // 清理超时障碍物历史
    private void CleanupObstacleHistory()
    {
        List<int> toRemove = new List<int>();
        foreach (var kvp in obstacleLastSeen)
        {
            if (Time.time - kvp.Value > 3f)
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

    // 获取当前全局路径点
    private Vector3 GetCurrentGlobalWaypoint()
    {
        if (globalPathfinder == null || globalPathfinder.path == null || globalPathfinder.path.Count == 0)
        {
            return target.position;
        }

        currentGlobalWaypointIndex = Mathf.Clamp(currentGlobalWaypointIndex, 0, globalPathfinder.path.Count - 1);
        if (Vector3.Distance(transform.position, gridManager.栅格转世界(globalPathfinder.path[currentGlobalWaypointIndex])) < 1.5f)
        {
            currentGlobalWaypointIndex++;
        }

        return gridManager.栅格转世界(globalPathfinder.path[Mathf.Min(currentGlobalWaypointIndex, globalPathfinder.path.Count - 1)]);
    }

    // 检查是否靠近全局路径
    private bool IsCloseToGlobalPath()
    {
        if (globalPathfinder == null || globalPathfinder.path == null || globalPathfinder.path.Count < 2)
        {
            return true;
        }

        Vector3 closestPoint = GetClosestPointOnPath(transform.position);
        return Vector3.Distance(transform.position, closestPoint) < returnToPathThreshold;
    }

    // 获取路径上最近的点
    private Vector3 GetClosestPointOnPath(Vector3 point)
    {
        Vector3 closest = Vector3.zero;
        float minDist = float.MaxValue;

        for (int i = 0; i < globalPathfinder.path.Count - 1; i++)
        {
            Vector3 start = gridManager.栅格转世界(globalPathfinder.path[i]);
            Vector3 end = gridManager.栅格转世界(globalPathfinder.path[i + 1]);
            float dist = DistancePointToLine(point, start, end);
            if (dist < minDist)
            {
                minDist = dist;
                closest = GetPointOnLine(point, start, end);
            }
        }

        return closest;
    }

    // 计算点到线段的距离
    private float DistancePointToLine(Vector3 point, Vector3 lineStart, Vector3 lineEnd)
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

    // 获取点在线段上的投影点
    private Vector3 GetPointOnLine(Vector3 point, Vector3 lineStart, Vector3 lineEnd)
    {
        Vector3 lineDir = lineEnd - lineStart;
        float lineLength = lineDir.magnitude;
        if (lineLength < 0.01f)
        {
            return lineStart;
        }

        float t = Mathf.Clamp01(Vector3.Dot(point - lineStart, lineDir) / (lineLength * lineLength));
        return lineStart + t * lineDir;
    }

    // 预测运动状态（分步计算提高精度）
    private (Vector3 pos, Quaternion rot) PredictMotion(float linearVel, float angularVel, float time)
    {
        Vector3 predictedPos = transform.position;
        Quaternion predictedRot = transform.rotation;

        int steps = 10;
        float stepTime = time / steps;

        for (int i = 0; i < steps; i++)
        {
            predictedRot *= Quaternion.Euler(0, angularVel * stepTime, 0);
            predictedPos += predictedRot * Vector3.forward * linearVel * stepTime;
        }

        return (predictedPos, predictedRot);
    }

    // 映射RL动作到速度和旋转
    private (Vector3 velocity, float rotation) GetGlobalActionVelocity(int action)
    {
        Vector3 velocity = Vector3.zero;
        float rotation = 0f;

        switch (action)
        {
            case 0: // 前进（中等速度）
                velocity = transform.forward * linearVelOptions[3];
                break;
            case 1: // 左转（中等角度）
                rotation = angularVelOptions[1];
                velocity = transform.forward * linearVelOptions[2]; // 左转时减速
                break;
            case 2: // 右转（中等角度）
                rotation = angularVelOptions[5];
                velocity = transform.forward * linearVelOptions[2]; // 右转时减速
                break;
            case 3: // 减速
                velocity = transform.forward * linearVelOptions[1];
                break;
        }

        return (velocity, rotation);
    }

    // 检测是否处于极度危险状态（距离障碍物过近）
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

    // 局部规划奖励（增强避障和合规奖励）
    private void AddLocalPlanningRewards()
    {
        if (dynamicObstacles.Count == 0) return;

        // 避障奖励（距离越远奖励越高）
        float minDistToObs = float.MaxValue;
        foreach (Vector3 obsPos in dynamicObstacles)
        {
            minDistToObs = Mathf.Min(minDistToObs, Vector3.Distance(transform.position, obsPos));
        }
        globalAgent.AddReward(minDistToObs / localSafeDistance * 2f);

        // COLREGs合规奖励
        float currentAngularVel = transform.eulerAngles.y * Time.deltaTime;
        float colregsReward = CalculateCOLREGsScore(transform.position, transform.rotation, currentAngularVel);
        globalAgent.AddReward(colregsReward * 0.8f);

        // 路径保持奖励
        if (IsCloseToGlobalPath())
        {
            globalAgent.AddReward(0.5f);
        }

        // 近距离碰撞惩罚（严厉）
        if (minDistToObs < collisionThreshold * 0.5f)
        {
            globalAgent.AddReward(-20f);
            EndEpisode();
        }
    }

    // 结束当前episode
    private void EndEpisode()
    {
        globalAgent.EndEpisode();
    }
}

// 部分类：存储障碍物历史和配置参数
public partial class USV_LocalPlanner : MonoBehaviour
{
    public float predictionTime = 2.5f;
    public float collisionThreshold = 2.5f;
    public float avoidDistance = 3.5f;
    public float obstacleSpeedFactor = 0.8f;

    // 带时间戳的障碍历史记录（用于速度估算）
    private Dictionary<int, List<(Vector3 pos, float time)>> obstacleHistory = new Dictionary<int, List<(Vector3 pos, float time)>>();
    private Dictionary<int, float> obstacleLastSeen = new Dictionary<int, float>();

    // 目标点（关联RL Agent的target）
    private Transform target => globalAgent != null ? globalAgent.target : null;
}