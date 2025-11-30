using UnityEngine;
using System.Collections.Generic;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using OpenCvSharp;
using System.Linq;

[RequireComponent(typeof(USV_GlobalRLAgent), typeof(Rigidbody))]
public partial class USV_LocalPlanner : MonoBehaviour
{
    // 关联核心模块引用
    private YoloDetector yoloDetector;
    private USV_GlobalRLAgent globalAgent;
    private GridManager gridManager;
    private Rigidbody rb;
    private ImprovedAStar globalPathfinder;

    // 物理与避障参数
    public float localSafeDistance = 5f;
    public float colregsSafeDistance = 4f;
    public float dwaPredictTime = 1.5f;
    public float returnToPathThreshold = 1.0f;

    // 动态障碍物信息
    public List<Vector3> dynamicObstacles = new List<Vector3>();
    public List<Vector3> dynamicObstacleVelocities = new List<Vector3>();
    private bool isAvoidingDynamicObstacle = false;
    private int currentGlobalWaypointIndex = 0;

    // 动作空间配置
    private readonly float[] linearVelOptions = { 0f, 0.3f, 0.6f, 0.9f, 1.2f };
    private readonly float[] angularVelOptions = { -15f, -5f, 0f, 5f, 15f };
    private const float MaxLinearVel = 1.2f;

    // COLREGs规则参数
    private const float StarboardAvoidAngle = 30f;
    private const float PortAvoidAngle = -30f;
    private const float HeadOnAvoidAngle = -45f;


    void Awake()
    {
        globalAgent = GetComponent<USV_GlobalRLAgent>();
        rb = GetComponent<Rigidbody>();
        gridManager = FindFirstObjectByType<GridManager>();
        globalPathfinder = FindFirstObjectByType<ImprovedAStar>();
        yoloDetector = FindFirstObjectByType<YoloDetector>();

        // 优化物理参数
        rb.linearDamping = 0.6f;
        rb.angularDamping = 1.2f;
        rb.useGravity = false; // 修复：水上运动禁用重力
        rb.collisionDetectionMode = CollisionDetectionMode.ContinuousDynamic; // 连续碰撞检测

        // 初始化动态障碍物列表
        dynamicObstacles = new List<Vector3>();
        dynamicObstacleVelocities = new List<Vector3>();

        if (yoloDetector == null)
        {
            // 第一次查找：优先找活跃的YoloDetector（推荐方式）
            yoloDetector = FindFirstObjectByType<YoloDetector>();

            if (yoloDetector == null)
            {
                // 第二次查找：允许找非活跃的实例（兼容场景隐藏的情况）
                yoloDetector = FindAnyObjectByType<YoloDetector>(FindObjectsInactive.Include);

                if (yoloDetector == null)
                {
                    Debug.LogError("[USV_LocalPlanner] YoloDetector未找到！动态避障功能无法启用，请检查场景中是否存在YoloDetector组件");
                }
                else
                {
                    Debug.LogWarning("[USV_LocalPlanner] 使用FindAnyObjectByType找到YoloDetector（可能包含非活跃实例），建议手动关联以提高性能和稳定性");
                }
            }
            else
            {
                Debug.Log("[USV_LocalPlanner] 通过FindFirstObjectByType自动找到YoloDetector，避障功能正常启用");
            }
        }
        else
        {
            Debug.Log("[USV_LocalPlanner] 手动关联YoloDetector成功，避障功能正常启用");
        }
    }

    void Start()
    {
        if (globalPathfinder != null && globalPathfinder.path != null && globalPathfinder.path.Count > 0)
        {
            currentGlobalWaypointIndex = 0;
        }

        if (yoloDetector == null)
        {
            Debug.LogError("YoloDetector未找到！动态避障功能无法启用");
        }
    }

    public void OnAgentActionReceived(ActionBuffers actions)
    {
        // 动态障碍物检测与预测
        DetectAndPredictDynamicObstacles();

        Vector3 targetVelocity = Vector3.zero;
        float targetRotation = 0f;

        // 局部规划优先级判定
        bool needAvoid = false;
        if (dynamicObstacles.Count > 0)
        {
            bool isInDangerZone = dynamicObstacles.Exists(obs =>
                Vector3.Distance(transform.position, obs) < localSafeDistance);

            bool hasImminentCollision = false;
            for (int i = 0; i < dynamicObstacles.Count; i++)
            {
                if (IsCollisionImminent(transform.position, rb.linearVelocity,
                    dynamicObstacles[i], dynamicObstacleVelocities[i], dwaPredictTime))
                {
                    hasImminentCollision = true;
                    break;
                }
            }

            // 检测到障碍物时强制避障（新增sports ball）
            bool hasObstacle = dynamicObstacles.Any(obs =>
                yoloDetector.DetectedResults.Any(r =>
                    (r.ClassName.ToLower() == "unmanned boat" || r.ClassName.ToLower() == "sports ball") &&
                    Vector3.Distance(ConvertYoloToWorldPosition(r.Rect), obs) < 0.5f));

            needAvoid = isInDangerZone || hasImminentCollision || hasObstacle;
            Debug.Log($"[LocalPlanner] 避障判定：危险区域={isInDangerZone} 碰撞风险={hasImminentCollision} 检测到障碍物={hasObstacle} → 需避障={needAvoid}");
        }

        // 核心修复：强制使用避障动作覆盖全局路径
        if (needAvoid)
        {
            isAvoidingDynamicObstacle = true;
            (targetVelocity, targetRotation) = LocalPlannerWithCOLREGs();
            Debug.Log($"[LocalPlanner] 进入避障模式：速度={targetVelocity.magnitude:F2}，转向={targetRotation:F1}");
        }
        else
        {
            if (isAvoidingDynamicObstacle && IsCloseToGlobalPath())
            {
                isAvoidingDynamicObstacle = false;
                Debug.Log("LocalPlanner：局部避障完成，回归全局路径");
            }
            (targetVelocity, targetRotation) = GetGlobalActionVelocity(actions.DiscreteActions[0]);
            Debug.Log($"[LocalPlanner] 全局路径模式：动作{actions.DiscreteActions[0]} → 速度={targetVelocity.magnitude:F2}，转向={targetRotation:F1}");
        }

        // 紧急情况处理
        if (IsExtremeDanger())
        {
            targetVelocity *= 0.2f;
            Debug.LogWarning("[LocalPlanner] 触发紧急减速");
        }

        // 修复：强制应用避障动作（关键！）
        targetVelocity = Vector3.ClampMagnitude(targetVelocity, MaxLinearVel);
        rb.linearVelocity = new Vector3(targetVelocity.x, rb.linearVelocity.y, targetVelocity.z);
        transform.Rotate(0, targetRotation * Time.deltaTime, 0);

        // 局部规划奖励系统
        AddLocalPlanningRewards();
    }

    // 动态障碍物检测与轨迹预测
    private void DetectAndPredictDynamicObstacles()
    {

        dynamicObstacles.Clear();
        dynamicObstacleVelocities.Clear();

        if (yoloDetector == null || yoloDetector.DetectedResults == null)
            return;

        List<YoloResult> obstacleResults = yoloDetector.DetectedResults.FindAll(result =>
            result.ClassName.ToLower() == "unmanned boat" ||
            result.ClassName == "sports ball" ||
            result.ClassName == "mouse"
        );

        Debug.Log($"[LocalPlanner] 筛选出障碍物：{obstacleResults.Count}个（无人船：{obstacleResults.Count(r => r.ClassName.ToLower() == "unmanned boat")}，运动球：{obstacleResults.Count(r => r.ClassName.ToLower() == "sports ball")}）");
        if (obstacleResults.Count == 0) return;

        // 处理每个障碍物
        for (int i = 0; i < obstacleResults.Count; i++)
        {
            YoloResult result = obstacleResults[i];
            Vector3 worldPos = ConvertYoloToWorldPosition(result.Rect);

            // 修复：水域内的障碍物才参与避障
            if (gridManager != null)
            {
                if (worldPos.x < gridManager.WaterMinX || worldPos.x > gridManager.WaterMaxX ||
                    worldPos.z < gridManager.WaterMinZ || worldPos.z > gridManager.WaterMaxZ)
                {
                    Debug.Log($"[LocalPlanner] 障碍物{i}超出水域，跳过");
                    continue;
                }
            }

            int obsId = result.TrackId >= 0 ? result.TrackId : i;
            Vector3 predictedVel = EstimateObstacleVelocity(obsId, worldPos, result.ClassName);
            dynamicObstacleVelocities.Add(predictedVel);

            Vector3 predictedPos = PredictObstaclePosition(worldPos, predictedVel, dwaPredictTime);
            dynamicObstacles.Add(predictedPos);


            // 碰撞风险检测
            bool collisionRisk = IsCollisionImminent(transform.position, rb.linearVelocity, worldPos, predictedVel, dwaPredictTime);
            if (collisionRisk)
            {
                isAvoidingDynamicObstacle = true;
                Debug.Log($"[LocalPlanner] 障碍物{i}触发碰撞风险，强制进入避障模式");
            }

            // 检测到sports ball或无人船时强制避障
            if (result.ClassName.ToLower() == "unmanned boat" || result.ClassName.ToLower() == "sports ball")
            {
                isAvoidingDynamicObstacle = true;
                Debug.Log($"[LocalPlanner] 检测到{result.ClassName}，强制避障");
            }
        }

        CleanupObstacleHistory();
    }

    // YOLO坐标转世界坐标（修复：适配检测相机）
    // YOLO坐标转世界坐标（适配X-Y平面坐标系）
    private Vector3 ConvertYoloToWorldPosition(Rect2d rect)
    {
        // 优先使用YoloDetector的DetectionCamera
        Camera detectCamera = yoloDetector != null ? yoloDetector.sceneCamera : Camera.main;
        if (detectCamera == null)
        {
            Debug.LogError("[LocalPlanner] DetectionCamera未找到，使用主相机");
            detectCamera = Camera.main;
            if (detectCamera == null)
            {
                Debug.LogError("[LocalPlanner] 主相机也未找到，返回默认位置");
                return transform.position + transform.forward * 5f;
            }
        }

        // 1. 图像坐标转DetectionCamera视口坐标
        float screenX = Mathf.Clamp((float)rect.X + (float)rect.Width / 2, 0, detectCamera.pixelWidth);
        float screenY = Mathf.Clamp((float)rect.Y + (float)rect.Height / 2, 0, detectCamera.pixelHeight);
        Vector3 viewportPos = new Vector3(screenX / detectCamera.pixelWidth, 1 - screenY / detectCamera.pixelHeight, 1f);

        // 2. 视口转射线（适配DetectionCamera的位置X=11.623,Y=-0.036,Z=0）
        Ray ray = detectCamera.ViewportPointToRay(viewportPos);

        // 3. 射线与水域平面（Y=0.4f）相交
        float groundY = 0.4f; // USV的固定高度
        Plane groundPlane = new Plane(Vector3.up, groundY); // 法向量向上，高度Y=0.4f
        if (groundPlane.Raycast(ray, out float distance))
        {
            Vector3 worldPos = ray.GetPoint(distance);

            // 4. 水域边界校验（适配X/Y轴，你的场景中X=左右，Z=前后）
            bool inWater = true;
            if (gridManager != null)
            {
                inWater = worldPos.x >= gridManager.WaterMinX && worldPos.x <= gridManager.WaterMaxX &&
                          worldPos.z >= gridManager.WaterMinZ && worldPos.z <= gridManager.WaterMaxZ;
                Debug.Log($"[LocalPlanner] 水域边界校验：X[{gridManager.WaterMinX:F1},{gridManager.WaterMaxX:F1}] Z[{gridManager.WaterMinZ:F1},{gridManager.WaterMaxZ:F1}] → 位置({worldPos.x:F1},{worldPos.z:F1}) → {inWater}");
            }

            if (inWater)
            {
                return new Vector3(worldPos.x, groundY, worldPos.z); // 固定Y=0.4f
            }
            else
            {
                Debug.LogWarning($"[LocalPlanner] 障碍物位置超出水域，已修正");
                worldPos.x = Mathf.Clamp(worldPos.x, gridManager.WaterMinX, gridManager.WaterMaxX);
                worldPos.z = Mathf.Clamp(worldPos.z, gridManager.WaterMinZ, gridManager.WaterMaxZ);
                return new Vector3(worldPos.x, groundY, worldPos.z);
            }
        }

        Debug.LogWarning($"[LocalPlanner] 坐标转换失败，使用备选方案：{viewportPos}");
        return transform.position + transform.forward * 5f;
    }

    // 障碍物速度估算
    private Vector3 EstimateObstacleVelocity(int obsId, Vector3 currentPos, string className)
    {
        if (!obstacleHistory.ContainsKey(obsId))
        {
            obstacleHistory[obsId] = new List<(Vector3 pos, float time)>();
        }

        obstacleHistory[obsId].Add((currentPos, Time.time));
        obstacleLastSeen[obsId] = Time.time;

        int historyCount = className.ToLower() == "unmanned boat" ? 100 : 30;
        while (obstacleHistory[obsId].Count > historyCount)
        {
            obstacleHistory[obsId].RemoveAt(0);
        }

        if (obstacleHistory[obsId].Count < 8)
        {
            Debug.LogWarning($"障碍物{obsId}历史数据不足（{obstacleHistory[obsId].Count}帧），速度估算为0");
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
        if (denominator < 0.001f)
        {
            Debug.LogWarning($"障碍物{obsId}速度计算分母过小，返回0");
            return Vector3.zero;
        }

        float aX = (n * sumTX - sumT * sumX) / denominator;
        float aZ = (n * sumTZ - sumT * sumZ) / denominator;

        float speedFactor = className.ToLower() == "unmanned boat" ? 1.2f : obstacleSpeedFactor;
        Vector3 predictedVel = new Vector3(aX, 0, aZ) * speedFactor;
        Debug.Log($"障碍物{obsId}（{className}）估算速度：{predictedVel}");

        return predictedVel;
    }

    // 预测障碍物未来位置
    private Vector3 PredictObstaclePosition(Vector3 currentPos, Vector3 velocity, float time)
    {
        return currentPos + velocity * time;
    }

    // 碰撞风险检测（修复：TTC计算逻辑）
    // 碰撞风险检测（适配X-Y平面坐标系）
    private bool IsCollisionImminent(Vector3 usvPos, Vector3 usvVel, Vector3 obsPos, Vector3 obsVel, float time)
    {
        Vector3 relativeVel = obsVel - usvVel;
        Vector3 relativePos = obsPos - usvPos;

        // 1. 超出预警距离（5f）
        float warningDistance = localSafeDistance;
        if (relativePos.magnitude > warningDistance)
        {
            Debug.Log($"距离障碍物{relativePos.magnitude:F1} > {warningDistance}，无碰撞风险");
            return false;
        }

        // 2. 目标远离
        float dotProduct = Vector3.Dot(relativePos.normalized, relativeVel.normalized);
        if (dotProduct >= 0.1f)
        {
            Debug.Log($"障碍物远离（点积{dotProduct:F2}≥0.1），无碰撞风险");
            return false;
        }

        // 3. 计算碰撞时间(TTC)
        float relativeSpeed = relativeVel.magnitude;
        if (relativeSpeed < 0.01f)
        {
            Debug.LogWarning("相对速度接近0，TTC计算无效，判定为近距离风险");
            return relativePos.magnitude < collisionThreshold;
        }

        float ttc = relativePos.magnitude / relativeSpeed;
        if (ttc < 0 || ttc > time)
        {
            Debug.Log($"TTC={ttc:F1}超出有效范围（0~{time}），无碰撞风险");
            return false;
        }

        // 4. 未来位置碰撞判断（适配你的坐标：X/Z为平面，Y固定）
        Vector3 usvFuture = new Vector3(
            usvPos.x + usvVel.x * ttc,
            usvPos.y,
            usvPos.z + usvVel.z * ttc
        );
        Vector3 obsFuture = new Vector3(
            obsPos.x + obsVel.x * ttc,
            obsPos.y,
            obsPos.z + obsVel.z * ttc
        );

        float distance = Vector3.Distance(usvFuture, obsFuture);
        float collisionCheckThreshold = relativePos.magnitude < warningDistance * 0.5f ? 1.5f : 2f;
        bool isImminent = distance < collisionCheckThreshold;
        Debug.Log($"未来{ttc:F1}秒距离：{distance:F1}，阈值：{collisionCheckThreshold}，碰撞风险：{isImminent}");

        return isImminent;
    }

    // 带COLREGs的局部规划器（修复：避障动作优先级）
    private (Vector3 velocity, float rotation) LocalPlannerWithCOLREGs()
    {
        float bestScore = -Mathf.Infinity;
        Vector3 bestVelocity = Vector3.zero;
        float bestRotation = 0f;
        Vector3 nextGlobalWaypoint = GetCurrentGlobalWaypoint();

        Debug.Log($"[LocalPlanner] 开始避障规划：动作空间{linearVelOptions.Length}x{angularVelOptions.Length}，目标路径点({nextGlobalWaypoint.x:F1},{nextGlobalWaypoint.z:F1})");

        // 遍历动作空间寻找最优解
        foreach (float linearVel in linearVelOptions)
        {
            if (linearVel > MaxLinearVel * 0.8f) continue; // 避障时限制速度

            foreach (float angularVel in angularVelOptions)
            {
                (Vector3 predictedPos, Quaternion predictedRot) = PredictMotion(linearVel, angularVel, dwaPredictTime);

                // 排除碰撞动作
                bool willCollide = dynamicObstacles.Any(obs =>
                    Vector3.Distance(predictedPos, obs) < collisionThreshold * 0.8f);
                if (willCollide)
                {
                    Debug.Log($"[LocalPlanner] 动作(速度{linearVel}, 转向{angularVel})会碰撞，跳过");
                    continue;
                }

                // 多目标评分
                float obstacleScore = CalculateObstacleScore(predictedPos);
                float pathTrackScore = CalculatePathTrackScore(predictedPos, nextGlobalWaypoint);
                float colregsScore = CalculateCOLREGsScore(predictedPos, predictedRot, angularVel);
                float smoothScore = CalculateSmoothScore(linearVel, angularVel);

                // 修复：避障权重提升至0.9
                float totalScore = obstacleScore * 0.9f
                                 + pathTrackScore * 0.05f
                                 + colregsScore * 0.03f
                                 + smoothScore * 0.02f;

                if (totalScore > bestScore)
                {
                    bestScore = totalScore;
                    bestVelocity = predictedRot * Vector3.forward * linearVel;
                    bestRotation = angularVel;
                }
            }
        }

        // 兜底：无有效动作时强制远离最近障碍物
        if (bestScore < 0)
        {
            Debug.LogWarning("[LocalPlanner] 未找到最优避障动作，使用应急方案");
            Vector3 closestObs = dynamicObstacles.OrderBy(obs => Vector3.Distance(transform.position, obs)).First();
            float avoidDir = closestObs.x > transform.position.x ? -15f : 15f;
            bestVelocity = transform.forward * linearVelOptions[1]; // 低速
            bestRotation = avoidDir;
        }

        Debug.Log($"[LocalPlanner] 避障动作确定：速度{bestVelocity.magnitude:F2}，转向{bestRotation:F1}");
        return (bestVelocity, bestRotation);
    }

    // 障碍物规避评分
    private float CalculateObstacleScore(Vector3 predictedPos)
    {
        if (dynamicObstacles.Count == 0) return 1f;

        float totalScore = 0f;
        foreach (var obsPos in dynamicObstacles)
        {
            float dist = Vector3.Distance(predictedPos, obsPos);
            if (dist < avoidDistance * 0.5f)
                totalScore += 0f;
            else if (dist < avoidDistance)
                totalScore += (dist / avoidDistance) * 0.5f;
            else
                totalScore += 1f;
        }

        return totalScore / dynamicObstacles.Count;
    }

    // 路径跟踪评分
    private float CalculatePathTrackScore(Vector3 predictedPos, Vector3 nextWaypoint)
    {
        float distToWaypoint = Vector3.Distance(predictedPos, nextWaypoint);
        return Mathf.Clamp(1 - (distToWaypoint / (localSafeDistance * 2)), 0f, 1f);
    }

    // COLREGs合规评分
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

            // 对遇局面处理
            if (Mathf.Abs(relativeAngle) < 20f && obsSpeed > 0.5f)
            {
                float rotationDiff = Mathf.Abs(currentAngularVel - HeadOnAvoidAngle);
                colregsScore *= Mathf.Clamp(1 - (rotationDiff / 60f), 0.2f, 1f);
            }
            // 右舷来船处理
            else if (relativeAngle > 0f && relativeAngle < 120f)
            {
                colregsScore *= Mathf.Clamp(1 - (Mathf.Abs(currentAngularVel) / 20f), 0.3f, 1f);
            }
            // 左舷来船处理
            else if (relativeAngle < 0f && relativeAngle > -120f)
            {
                float rotationDiff = Mathf.Abs(currentAngularVel - StarboardAvoidAngle);
                colregsScore *= Mathf.Clamp(1 - (rotationDiff / 50f), 0.2f, 1f);
            }

            // 安全距离惩罚
            float distToObs = Vector3.Distance(predictedPos, obsPos);
            if (distToObs < colregsSafeDistance)
            {
                colregsScore *= Mathf.Pow(distToObs / colregsSafeDistance, 2);
            }
        }

        return colregsScore;
    }

    // 动作平滑度评分
    private float CalculateSmoothScore(float linearVel, float angularVel)
    {
        float currentLinearVel = Vector3.Dot(transform.forward, rb.linearVelocity);
        float linearSmooth = 1 - Mathf.Abs(linearVel - currentLinearVel) / MaxLinearVel;
        float angularSmooth = 1 - Mathf.Abs(angularVel) / 60f;

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
            return globalAgent.target != null ? globalAgent.target.position : transform.position;
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

    // 获取路径上最近点
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

    // 点到线段距离计算
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

    // 线段上投影点计算
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

    // 运动状态预测
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

    // RL动作映射
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
                velocity = transform.forward * linearVelOptions[2];
                break;
            case 2: // 右转（中等角度）
                rotation = angularVelOptions[3];
                velocity = transform.forward * linearVelOptions[2];
                break;
            case 3: // 减速
                velocity = transform.forward * linearVelOptions[1];
                break;
        }

        return (velocity, rotation);
    }

    // 极度危险状态检测
    private bool IsExtremeDanger()
    {
        if (dynamicObstacles.Count == 0) return false;

        float minDist = float.MaxValue;
        foreach (var obs in dynamicObstacles)
        {
            minDist = Mathf.Min(minDist, Vector3.Distance(transform.position, obs));
        }

        return minDist < collisionThreshold * 0.5f;
    }

    // 局部规划奖励系统
    private void AddLocalPlanningRewards()
    {
        if (dynamicObstacles.Count == 0) return;

        float minDistToObs = float.MaxValue;
        foreach (Vector3 obsPos in dynamicObstacles)
        {
            minDistToObs = Mathf.Min(minDistToObs, Vector3.Distance(transform.position, obsPos));
        }

        // 避障奖励
        globalAgent.AddReward(minDistToObs / localSafeDistance * 5f);

        // COLREGs合规奖励
        float currentAngularVel = transform.eulerAngles.y * Time.deltaTime;
        float colregsReward = CalculateCOLREGsScore(transform.position, transform.rotation, currentAngularVel);
        globalAgent.AddReward(colregsReward * 0.8f);

        // 路径保持奖励
        if (IsCloseToGlobalPath())
        {
            globalAgent.AddReward(0.5f);
        }

        // 近距离碰撞惩罚
        if (minDistToObs < collisionThreshold * 0.5f)
        {
            globalAgent.AddReward(-20f);
            EndEpisode();
        }

        // 接近障碍物梯度惩罚
        if (minDistToObs < collisionThreshold)
        {
            globalAgent.AddReward(-(collisionThreshold - minDistToObs) * 3f);
        }
    }

    // 结束当前episode
    private void EndEpisode()
    {
        globalAgent.EndEpisode();
    }

    // Gizmos可视化（调试用）
    private void OnDrawGizmos()
    {
        // 绘制动态障碍物（红色）
        Gizmos.color = Color.red;
        foreach (var obs in dynamicObstacles)
        {
            Gizmos.DrawSphere(obs, 0.5f);
        }

        // 绘制避障路径（黄色）
        if (isAvoidingDynamicObstacle && rb != null)
        {
            Gizmos.color = Color.yellow;
            var (predictedPos, _) = PredictMotion(rb.linearVelocity.magnitude, transform.rotation.eulerAngles.y, 1f);
            Gizmos.DrawLine(transform.position, predictedPos);
        }

        // 绘制水域边界（绿色半透明）
        if (gridManager != null)
        {
            Gizmos.color = new Color(0, 1, 0, 0.1f);
            Gizmos.DrawWireCube(
                new Vector3((gridManager.WaterMinX + gridManager.WaterMaxX) / 2, 0, (gridManager.WaterMinZ + gridManager.WaterMaxZ) / 2),
                new Vector3(gridManager.WaterMaxX - gridManager.WaterMinX, 0.1f, gridManager.WaterMaxZ - gridManager.WaterMinZ)
            );
        }
    }
}

// 部分类：存储障碍物历史和配置参数
public partial class USV_LocalPlanner : MonoBehaviour
{
    public float predictionTime = 3.0f;
    public float collisionThreshold = 2f;
    public float avoidDistance = 4.0f;
    public float obstacleSpeedFactor = 0.9f;

    // 障碍物历史记录
    private Dictionary<int, List<(Vector3 pos, float time)>> obstacleHistory = new Dictionary<int, List<(Vector3 pos, float time)>>();
    private Dictionary<int, float> obstacleLastSeen = new Dictionary<int, float>();

    // 目标点引用
    private Transform target => globalAgent != null ? globalAgent.target : null;
}