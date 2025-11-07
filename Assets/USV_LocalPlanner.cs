using UnityEngine;
using System.Collections.Generic;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;

[RequireComponent(typeof(USV_GlobalRLAgent), typeof(Rigidbody))]
public class USV_LocalPlanner : MonoBehaviour
{
    // 核心依赖组件
    private USV_GlobalRLAgent globalAgent;
    private GridManager gridManager;
    private Rigidbody rb;
    private ImprovedAStar globalPathfinder;

    // 激光雷达配置（需在Inspector赋值）
    public LidarSensor lidar; // 激光雷达组件（自定义或第三方）
    public int lidarSampleCount = 360; // 激光雷达采样点数（全向检测）

    // 局部避障参数
    public float localSafeDistance = 3f; // 动态障碍安全距离
    public float colregsSafeDistance = 4f; // COLREGs规则安全距离（会船时）
    public float dwaPredictTime = 0.8f; // DWA预测时间窗口
    public float returnToPathThreshold = 2f; // 回归全局路径的距离阈值

    // 动态障碍存储
    private List<Vector3> dynamicObstacles = new List<Vector3>();
    private List<Vector3> dynamicObstacleVelocities = new List<Vector3>(); // 动态障碍速度（用于COLREGs判断）
    private bool isAvoidingDynamicObstacle = false; // 局部避障激活状态
    private int currentGlobalWaypointIndex = 0; // 跟踪当前全局航点

    // DWA速度窗口配置
    private readonly float[] linearVelOptions = { 0f, 0.3f, 0.8f, 1.2f, 1.5f }; // 可行线速度（m/s）
    private readonly float[] angularVelOptions = { -45f, -30f, -15f, 0f, 15f, 30f, 45f }; // 可行角速度（°/s）
    private const float MaxLinearVel = 1.5f; // 最大线速度（与全局配置一致）

    // COLREGs规则参数
    private const float StarboardAvoidAngle = 30f; // 右舷会船避障角度
    private const float PortAvoidAngle = -30f; // 左舷会船避障角度
    private const float HeadOnAvoidAngle = -45f; // 对遇局面避障角度

    void Awake()
    {
        // 初始化依赖组件
        globalAgent = GetComponent<USV_GlobalRLAgent>();
        rb = GetComponent<Rigidbody>();
        gridManager = FindObjectOfType<GridManager>();
        globalPathfinder = FindObjectOfType<ImprovedAStar>();

        // 参数校验
        if (lidar == null) Debug.LogError("局部规划脚本：请赋值激光雷达组件！");
        if (gridManager == null) Debug.LogError("局部规划脚本：未找到GridManager！");
        if (globalPathfinder == null) Debug.LogError("局部规划脚本：未找到全局路径规划器！");
    }

    void Start()
    {
        // 同步全局航点索引
        if (globalPathfinder.path != null && globalPathfinder.path.Count > 0)
        {
            currentGlobalWaypointIndex = 0;
        }

        // 初始化激光雷达
        lidar.Initialize(lidarSampleCount);
    }

    /// <summary>
    /// 融合局部避障的动作执行（替代原全局Agent的动作逻辑）
    /// </summary>
    public void OnAgentActionReceived(ActionBuffers actions)
    {
        // 1. 检测动态障碍（全局栅格未标注的突发障碍）
        DetectDynamicObstacles();

        // 2. 确定运动目标（局部避障/全局跟踪）
        Vector3 targetVelocity;
        float targetRotation;

        if (dynamicObstacles.Count > 0)
        {
            isAvoidingDynamicObstacle = true;
            // 动态障碍存在，执行局部避障（DWA+COLREGs）
            (targetVelocity, targetRotation) = LocalPlannerWithCOLREGs();
        }
        else
        {
            // 无动态障碍，回归全局路径跟踪
            if (isAvoidingDynamicObstacle && IsCloseToGlobalPath())
            {
                isAvoidingDynamicObstacle = false;
                Debug.Log("局部避障完成，回归全局路径");
            }

            // 执行全局强化学习决策
            (targetVelocity, targetRotation) = GetGlobalActionVelocity(actions.DiscreteActions[0]);
        }

        // 3. 应用运动状态（限制最大速度）
        targetVelocity = Vector3.ClampMagnitude(targetVelocity, MaxLinearVel);
        rb.velocity = new Vector3(targetVelocity.x, rb.velocity.y, targetVelocity.z);
        transform.Rotate(0, targetRotation * Time.deltaTime, 0);

        // 4. 局部避障奖励（融合到全局强化学习）
        AddLocalPlanningRewards();
    }

    /// <summary>
    /// 动态障碍检测（激光雷达+全局栅格对比）
    /// </summary>
    private void DetectDynamicObstacles()
    {
        dynamicObstacles.Clear();
        dynamicObstacleVelocities.Clear();

        if (lidar == null) return;

        // 更新激光雷达数据
        lidar.CompleteScan();
        float[] distances = lidar.GetDistances();
        Vector3[] obstacleWorldPositions = lidar.GetWorldPositions();
        Vector3[] obstacleVelocities = lidar.GetObstacleVelocities(); // 激光雷达获取障碍速度

        // 遍历检测结果，筛选动态障碍
        for (int i = 0; i < distances.Length; i++)
        {
            if (distances[i] < localSafeDistance)
            {
                Vector3 obstaclePos = obstacleWorldPositions[i];
                Vector2Int obstacleGridPos = gridManager.世界转栅格(obstaclePos);

                // 判定条件：全局栅格标记为可通行，但激光雷达检测到近距离障碍
                if (gridManager.栅格是否可通行(obstacleGridPos))
                {
                    dynamicObstacles.Add(obstaclePos);
                    dynamicObstacleVelocities.Add(obstacleVelocities[i]);
                }
            }
        }
    }

    /// <summary>
    /// 局部规划核心（DWA+COLREGs规则）
    /// </summary>
    private (Vector3 velocity, float rotation) LocalPlannerWithCOLREGs()
    {
        float bestScore = -Mathf.Infinity;
        Vector3 bestVelocity = Vector3.zero;
        float bestRotation = 0f;

        // 获取当前全局航点
        Vector3 nextGlobalWaypoint = GetCurrentGlobalWaypoint();

        // 遍历所有速度组合，计算最优解
        foreach (float linearVel in linearVelOptions)
        {
            foreach (float angularVel in angularVelOptions) // 当前候选角速度：angularVel
            {
                // 预测未来位置和朝向
                (Vector3 predictedPos, Quaternion predictedRot) = PredictMotion(linearVel, angularVel, dwaPredictTime);

                // 计算各项评价得分：传入当前候选角速度angularVel到COLREGs评分
                float obstacleScore = CalculateObstacleScore(predictedPos);
                float pathTrackScore = CalculatePathTrackScore(predictedPos, nextGlobalWaypoint);
                float colregsScore = CalculateCOLREGsScore(predictedPos, predictedRot, angularVel); // 新增参数：当前候选角速度
                float smoothScore = CalculateSmoothScore(linearVel, angularVel);

                // 综合得分（避障权重最高，COLREGs次之）
                float totalScore = obstacleScore * 0.5f + pathTrackScore * 0.3f + colregsScore * 0.15f + smoothScore * 0.05f;

                // 更新最优解
                if (totalScore > bestScore)
                {
                    bestScore = totalScore;
                    bestVelocity = predictedRot * Vector3.forward * linearVel;
                    bestRotation = angularVel; // 仅在此处记录最优角速度
                }
            }
        }

        return (bestVelocity, bestRotation);
    }

    /// <summary>
    /// 评价指标3：COLREGs规则合规性（修复：接收当前候选角速度）
    /// </summary>
    private float CalculateCOLREGsScore(Vector3 predictedPos, Quaternion predictedRot, float currentAngularVel) // 新增参数：currentAngularVel
    {
        float colregsScore = 1f; // 初始得分为1（无冲突时满分）

        for (int i = 0; i < dynamicObstacles.Count; i++)
        {
            Vector3 obsPos = dynamicObstacles[i];
            Vector3 obsVel = dynamicObstacleVelocities[i];

            // 计算相对位置和速度
            Vector3 relativePos = obsPos - predictedPos;
            float relativeAngle = Vector3.SignedAngle(predictedRot * Vector3.forward, relativePos, Vector3.up);
            float obsSpeed = obsVel.magnitude;

            // 1. 对遇局面（船头相对，速度相近）
            if (Mathf.Abs(relativeAngle) < 20f && obsSpeed > 0.5f)
            {
                // 需向左转向避障：使用当前候选角速度currentAngularVel（原错误：bestRotation）
                float rotationDiff = Mathf.Abs(currentAngularVel - HeadOnAvoidAngle);
                colregsScore *= Mathf.Clamp(1 - (rotationDiff / 90f), 0.3f, 1f);
            }
            // 2. 右舷来船（COLREGs规则：保持航向，让对方避让）
            else if (relativeAngle > 0f && relativeAngle < 120f)
            {
                // 尽量保持原航向：使用当前候选角速度currentAngularVel
                colregsScore *= Mathf.Clamp(1 - (Mathf.Abs(currentAngularVel) / 30f), 0.4f, 1f);
            }
            // 3. 左舷来船（COLREGs规则：主动避让）
            else if (relativeAngle < 0f && relativeAngle > -120f)
            {
                // 需向右转向避障：使用当前候选角速度currentAngularVel
                float rotationDiff = Mathf.Abs(currentAngularVel - StarboardAvoidAngle);
                colregsScore *= Mathf.Clamp(1 - (rotationDiff / 60f), 0.3f, 1f);
            }

            // 会船时需保持更远安全距离
            float distToObs = Vector3.Distance(predictedPos, obsPos);
            if (distToObs < colregsSafeDistance)
            {
                colregsScore *= distToObs / colregsSafeDistance;
            }
        }

        return colregsScore;
    }

    /// <summary>
    /// 预测运动状态（基于当前速度和角速度）
    /// </summary>
    private (Vector3 pos, Quaternion rot) PredictMotion(float linearVel, float angularVel, float time)
    {
        Vector3 predictedPos = transform.position;
        Quaternion predictedRot = transform.rotation;

        // 分段预测（提高精度）
        int steps = 5;
        float stepTime = time / steps;

        for (int i = 0; i < steps; i++)
        {
            // 旋转更新
            predictedRot *= Quaternion.Euler(0, angularVel * stepTime, 0);
            // 位置更新
            predictedPos += predictedRot * Vector3.forward * linearVel * stepTime;
        }

        return (predictedPos, predictedRot);
    }

    /// <summary>
    /// 评价指标1：避障安全性（距离障碍越远得分越高）
    /// </summary>
    private float CalculateObstacleScore(Vector3 predictedPos)
    {
        float totalScore = 0f;
        foreach (Vector3 obsPos in dynamicObstacles)
        {
            float dist = Vector3.Distance(predictedPos, obsPos);
            // 距离越近得分越低，低于安全阈值则得0分
            totalScore += Mathf.Clamp(dist / localSafeDistance, 0f, 1f);
        }
        // 平均得分（适配多个障碍场景）
        return dynamicObstacles.Count > 0 ? totalScore / dynamicObstacles.Count : 1f;
    }

    /// <summary>
    /// 评价指标2：全局路径跟踪（靠近航点得分越高）
    /// </summary>
    private float CalculatePathTrackScore(Vector3 predictedPos, Vector3 nextWaypoint)
    {
        float distToWaypoint = Vector3.Distance(predictedPos, nextWaypoint);
        // 归一化得分（最大距离为安全距离的2倍）
        return Mathf.Clamp(1 - (distToWaypoint / (localSafeDistance * 2)), 0f, 1f);
    }

   

    /// <summary>
    /// 评价指标4：运动平滑性（避免急加减速和急转弯）
    /// </summary>
    private float CalculateSmoothScore(float linearVel, float angularVel)
    {
        // 线速度平滑：接近当前速度得分高
        float currentLinearVel = Vector3.Dot(transform.forward, rb.velocity);
        float linearSmooth = 1 - Mathf.Abs(linearVel - currentLinearVel) / MaxLinearVel;

        // 角速度平滑：转角越小得分高
        float angularSmooth = 1 - Mathf.Abs(angularVel) / 45f;

        return (linearSmooth + angularSmooth) / 2f;
    }

    /// <summary>
    /// 获取当前跟踪的全局航点
    /// </summary>
    private Vector3 GetCurrentGlobalWaypoint()
    {
        if (globalPathfinder.path == null || globalPathfinder.path.Count == 0)
        {
            return globalAgent.target.position; // 无全局路径时，直接指向目标点
        }

        // 更新当前航点（到达后切换下一个）
        Vector3 currentWaypoint = gridManager.栅格转世界(globalPathfinder.path[currentGlobalWaypointIndex]);
        float distToWaypoint = Vector3.Distance(transform.position, currentWaypoint);

        if (distToWaypoint < 1f && currentGlobalWaypointIndex < globalPathfinder.path.Count - 1)
        {
            currentGlobalWaypointIndex++;
        }

        return currentWaypoint;
    }

    /// <summary>
    /// 检查是否接近全局路径（用于回归判断）
    /// </summary>
    private bool IsCloseToGlobalPath()
    {
        if (globalPathfinder.path == null || globalPathfinder.path.Count < 2)
        {
            return true;
        }

        // 计算当前位置到全局路径的最短距离
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

    /// <summary>
    /// 计算点到线段的最短距离
    /// </summary>
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

    /// <summary>
    /// 局部规划奖励函数（融合到全局强化学习）
    /// </summary>
    private void AddLocalPlanningRewards()
    {
        if (dynamicObstacles.Count == 0) return;

        // 1. 避障安全奖励（距离最近障碍越远，奖励越高）
        float minDistToObs = float.MaxValue;
        foreach (Vector3 obsPos in dynamicObstacles)
        {
            minDistToObs = Mathf.Min(minDistToObs, Vector3.Distance(transform.position, obsPos));
        }
        globalAgent.AddReward(minDistToObs / localSafeDistance * 1.5f);

        // 2. COLREGs合规奖励
        float currentAngularVel = transform.eulerAngles.y * Time.deltaTime; // 获取当前无人船实际角速度
        float colregsReward = CalculateCOLREGsScore(transform.position, transform.rotation, currentAngularVel);
        globalAgent.AddReward(colregsReward * 0.5f);

        // 3. 避障时不偏离全局路径过远奖励
        if (IsCloseToGlobalPath())
        {
            globalAgent.AddReward(0.3f);
        }
    }

    /// <summary>
    /// 转换全局Agent动作到速度和转向（适配原全局逻辑）
    /// </summary>
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

    // 供全局Agent调用的动作入口
    public void ForwardActionToLocalPlanner(ActionBuffers actions)
    {
        OnAgentActionReceived(actions);
    }
}

// 激光雷达传感器接口（需与实际激光雷达组件适配）
public abstract class LidarSensor : MonoBehaviour
{
    public abstract void Initialize(int sampleCount);
    public abstract void CompleteScan();
    public abstract float[] GetDistances();
    public abstract Vector3[] GetWorldPositions();
    public abstract Vector3[] GetObstacleVelocities();
}