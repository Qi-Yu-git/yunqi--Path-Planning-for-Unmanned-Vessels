#pragma warning disable 0414
using UnityEngine;
using System.Collections.Generic;
using System.Linq;
using Unity.MLAgents;
using USVGridSystem;

[RequireComponent(typeof(USV_GlobalRLAgent), typeof(Rigidbody))]
public class USV_LocalPlanner : MonoBehaviour
{
    // ========== 引用 ==========
    private USV_GlobalRLAgent globalAgent;
    private Rigidbody rb;
    private GridManager gridManager;
    private ImprovedAStar globalPathfinder;
    private YoloDetector yoloDetector;

    // ========== DWA 参数 (论文 Section 2.3) ==========
    [Header("DWA 参数 (论文 Section 2.3)")]
    public float predictTime = 1.0f;
    public int predictSteps = 10;
    public float maxLinearSpeed = 1.2f;
    public float maxAngularSpeed = 0.26f;
    public float collisionHardBoundary = 0.2f;

    // ========== 动态安全距离 (论文公式16) ==========
    private float d0 = 1.2f;
    private float tau = 0.5f;
    private float cv = 0.3f;

    // ========== DWA 权重 (由PPO动态调整) ==========
    private float[] dwaWeights = new float[4] { 0.3f, 0.4f, 0.2f, 0.1f };
    private float currentKs = 2.0f;
    private float currentDs = 2.0f;

    // ========== 速度采样空间 (论文公式31-32) - 移除0速度 ==========
    // 原代码：
    // private float[] linearSamples = { 0.3f, 0.6f, 0.9f, 1.2f };
    // 改为：
    private float[] linearSamples = { 0.15f, 0.3f, 0.6f, 0.9f, 1.2f };
    private float[] angularSamples;

    // ========== 路径跟随 ==========
    public int currentPathIndex = 0;

    // ========== 障碍物追踪 ==========
    public List<Vector3> dynamicObstacles = new List<Vector3>();
    public List<Vector3> dynamicObstacleVelocities = new List<Vector3>();
    private Dictionary<int, List<(Vector3 pos, float time)>> obstacleHistory = new Dictionary<int, List<(Vector3 pos, float time)>>();
    private Dictionary<int, float> obstacleLastSeen = new Dictionary<int, float>();

    // ========== 当前控制输出 ==========
    private Vector3 currentTargetVelocity = Vector3.zero;
    private float currentTargetAngular = 0f;
    private bool hasValidControl = false;

    // ========== COLREGs 参考角速度 (论文公式26-28) ==========
    private float omegaRefHO = 25f * Mathf.Deg2Rad;
    private float omegaRefSC = 15f * Mathf.Deg2Rad;
    private float omegaRefPC = 0f;

    // ========== 平滑控制参数 ==========
    private float smoothOmega = 0f;
    private float smoothSpeed = 0f;
    private const float SMOOTH_TIME = 0.3f;

    // ========== 日志限频 ==========
    // ========== 日志限频 ==========
    private float lastWarningLogTime = -10f;
    // ========== 轨迹缓存 ==========
    private Vector3[] predictedTrajectory = new Vector3[30];

    void Awake()
    {
        globalAgent = GetComponent<USV_GlobalRLAgent>();
        rb = GetComponent<Rigidbody>();
        gridManager = FindFirstObjectByType<GridManager>();
        globalPathfinder = FindFirstObjectByType<ImprovedAStar>();
        yoloDetector = FindFirstObjectByType<YoloDetector>();

        rb.mass = 8.5f;
        rb.linearDamping = 0.12f;
        rb.angularDamping = 0.05f;
        rb.useGravity = false;
        rb.collisionDetectionMode = CollisionDetectionMode.ContinuousDynamic;
        rb.constraints = RigidbodyConstraints.FreezeRotationX |
                         RigidbodyConstraints.FreezeRotationZ |
                         RigidbodyConstraints.FreezePositionY;

        // ====== 防止刚体休眠 ======
        rb.sleepThreshold = 0f;

        // 角速度采样空间 (论文公式32)
        angularSamples = new float[] { -0.26f, -0.195f, -0.13f, -0.065f, -0.0325f, 0f, 0.0325f, 0.065f, 0.13f, 0.195f, 0.26f };
        Debug.Log($"[DWA] linearSamples长度: {linearSamples.Length}, angularSamples长度: {angularSamples.Length}");

        dynamicObstacles = new List<Vector3>();
        dynamicObstacleVelocities = new List<Vector3>();
        dwaWeights = new float[4] { 0.3f, 0.4f, 0.2f, 0.1f };
        currentPathIndex = 0;
    }

    void FixedUpdate()
    {
        if (hasValidControl && rb != null)
        {
            ApplyPhysicalControl(currentTargetVelocity, currentTargetAngular);
        }
    }

    // ============================================================
    // 核心方法：执行DWA (由PPO OnActionReceived调用)
    // ============================================================
    public void ExecuteDWA(float[] weights, float ks, float ds)
    {
        // ====== 入口日志 ======
        if (Time.frameCount % 30 == 0)
        {
            //   Debug.Log($"[DWA] ExecuteDWA 被调用, weights=[{string.Join(",", weights)}], ks={ks}, ds={ds}");
        }

        // 更新DWA权重 (论文公式42-45)
        UpdateDwaWeights(weights);
        currentKs = Mathf.Clamp(ks, 0.5f, 5.0f);
        currentDs = Mathf.Clamp(ds, 1.0f, 5.0f);

        if (globalPathfinder != null)
        {
            globalPathfinder.safeCostWeight = currentKs;
            globalPathfinder.safeDistanceRadius = currentDs;
        }

        // 检测动态障碍物
        DetectAndPredictDynamicObstacles();

        // 执行DWA采样与评估
        (Vector3 bestVel, float bestAngular) = RunDWA();

        // ====== 结果日志 ======
        if (Time.frameCount % 30 == 0)
        {
            //   Debug.Log($"[DWA] RunDWA 返回: bestVel={bestVel.magnitude:F2}, bestAngular={bestAngular:F2}");
        }

        // ====== 如果最佳速度为零===
        if (bestVel.magnitude < 0.01f && Mathf.Abs(bestAngular) < 0.001f)
        {
            Debug.LogWarning("[DWA] 最佳速度为零且无转向，强制前进");
            bestVel = transform.forward * 0.5f;
            bestAngular = 0f;
        }

        // 缓存控制指令
        currentTargetVelocity = bestVel;
        currentTargetAngular = bestAngular;
        hasValidControl = true;

        // 应用物理控制
        ApplyPhysicalControl(bestVel, bestAngular);

        if (Time.frameCount % 30 == 0)
        {
            //  Debug.Log($"[DWA] 目标: {bestVel.magnitude:F2}m/s, {bestAngular:F2}rad/s | " +
            //  $"实际: {rb.linearVelocity.magnitude:F2}m/s, {rb.angularVelocity.y:F2}rad/s");
        }
    }

    // ============================================================
    // 权重更新 (论文公式42-45)
    // ============================================================
    public void UpdateDwaWeights(float[] weights)
    {
        if (weights == null || weights.Length != 4)
        {
            Debug.LogWarning("[LocalPlanner] 传入的DWA权重数组无效，保持当前权重");
            return;
        }

        float sum = weights[0] + weights[1] + weights[2] + weights[3];
        if (sum > 0.001f)
        {
            for (int i = 0; i < 4; i++)
            {
                dwaWeights[i] = Mathf.Clamp(weights[i] / sum, 0.05f, 0.6f);
            }
        }
        else
        {
            Debug.LogWarning("[LocalPlanner] DWA权重和为0，使用默认权重");
            dwaWeights = new float[4] { 0.3f, 0.4f, 0.2f, 0.1f };
        }
    }

    public float[] GetDwaWeights() => (float[])dwaWeights.Clone();
    public float GetCurrentKs() => currentKs;
    public float GetCurrentDs() => currentDs;

    public void ResetPathIndex()
    {
        currentPathIndex = 0;
        Debug.Log("[LocalPlanner] 路径索引已重置");
    }

    // ============================================================
    // DWA主循环 - 速度空间采样 + 轨迹评分 (论文核心)
    // ============================================================
    private (Vector3, float) RunDWA()
    {
        // ========== 1. 路径检查 ==========
        if (globalPathfinder == null || globalPathfinder.path == null || globalPathfinder.path.Count == 0)
        {
            Vector3 dir = GetTargetDirection();
            if (dir.sqrMagnitude > 0.001f)
                return (dir * 0.3f, 0f);
            return (Vector3.zero, 0f);
        }

        if (gridManager == null)
            return (Vector3.zero, 0f);

        // ========== 2. 获取下一个航点 ==========
        Vector3 targetWaypoint = GetNextGlobalWaypoint();
        if (targetWaypoint == Vector3.zero)
            return (Vector3.zero, 0f);

        // ========== 3. 获取当前状态 ==========
        float currentSpeed = rb.linearVelocity.magnitude;
        float currentOmega = rb.angularVelocity.y;

        // ========== 4. 计算动态安全距离 ==========
        float safeDist = CalculateDynamicSafeDistance(currentSpeed);
        safeDist = Mathf.Min(safeDist, 2.0f);

        // ========== 5. DWA采样 ==========
        float bestScore = -999f;
        Vector3 bestVel = Vector3.zero;
        float bestOmega = 0f;
        int validSamples = 0;
        int totalSamples = linearSamples.Length * angularSamples.Length;

        // 获取目标方向
        Vector3 pathDirection = GetTargetDirection();

        foreach (float u in linearSamples)
        {
            foreach (float omega in angularSamples)
            {
                (Vector3 predPos, float predHeading) = PredictTrajectory(u, omega);

                bool isCollision = IsTrajectoryInCollision(u, omega);

                if (isCollision)
                    continue;

                validSamples++;

                float S_path = CalculatePathScore(predPos, targetWaypoint, safeDist);
                float S_obs = CalculateObstacleScore(predPos, safeDist);
                float S_smooth = CalculateSmoothScore(u, omega, currentSpeed);
                float S_colregs = CalculateCOLREGSScore(omega, predPos);

                // 【v10】新增：前进偏好（Progress Bias），防止低速犹豫
                float S_progress = u / maxLinearSpeed;

                float totalScore = dwaWeights[0] * S_path * 1.5f +
                                   dwaWeights[1] * S_obs +
                                   dwaWeights[2] * S_smooth +
                                   dwaWeights[3] * S_colregs +
                                   0.3f * S_progress; // 【v10】强制加入前进项

                if (totalScore > bestScore)
                {
                    bestScore = totalScore;
                    bestVel = pathDirection * u;
                    bestOmega = omega;
                }
            }
        }

        //Debug.Log($"[DWA] 采样: {totalSamples}组, 有效: {validSamples}组, bestScore={bestScore:F3}, bestVel={bestVel.magnitude:F2}");
        if (validSamples == 0 || bestScore < -0.1f)
        {
            if (Time.time - lastWarningLogTime > 3f)
            {
                lastWarningLogTime = Time.time;
                Debug.LogWarning($"[DWA] 无有效轨迹! 执行原地旋转扫描");
            }
            // 修改：原地旋转扫描可行方向，避免后退陷入振荡或触发新的碰撞
            float scanOmega = maxAngularSpeed * 0.5f * (Mathf.Sin(Time.time * 2f) > 0 ? 1f : -1f);
            return (transform.forward * 0.3f, scanOmega); // 保持 0.3m/s 蠕动，避免彻底卡死
        }

        if (Time.frameCount % 60 == 0)
        {
            // Debug.Log($"[DWA] 采样: {linearSamples.Length * angularSamples.Length}组, 有效: {validSamples}组 | " +
            //    $"评分: {bestScore:F3}, 速度: {bestVel.magnitude:F2}");
        }

        return (bestVel, bestOmega);
    }

    // 添加辅助方法
    private Vector3 GetTargetDirection()
    {
        Vector3 targetDir = transform.forward;

        Vector3 waypoint = GetNextGlobalWaypoint();
        if (waypoint != Vector3.zero)
        {
            Vector3 dir = (waypoint - transform.position).normalized;
            dir.y = 0;
            if (dir.sqrMagnitude > 0.001f)
            {
                targetDir = dir;
            }
        }

        if (targetDir == transform.forward && globalAgent != null && globalAgent.target != null)
        {
            Vector3 dir = (globalAgent.target.position - transform.position).normalized;
            dir.y = 0;
            if (dir.sqrMagnitude > 0.001f)
            {
                targetDir = dir;
            }
        }

        return targetDir;
    }

    // ============================================================
    // 轨迹预测 (论文公式33)
    // ============================================================
    private (Vector3, float) PredictTrajectory(float linear, float angular)
    {
        Vector3 pos = transform.position;
        float heading = transform.eulerAngles.y * Mathf.Deg2Rad;

        float dt = predictTime / predictSteps;

        for (int i = 0; i < predictSteps; i++)
        {
            heading += angular * dt;
            pos += new Vector3(Mathf.Sin(heading), 0, Mathf.Cos(heading)) * linear * dt;
            predictedTrajectory[i] = pos;
        }

        return (pos, heading);
    }

    // ============================================================
    // 四项评分函数 (论文公式34-38)
    // ============================================================

    private float CalculatePathScore(Vector3 predPos, Vector3 targetWaypoint, float safeDist)
    {
        float dPath = Vector3.Distance(predPos, targetWaypoint);
        // 使用指数衰减，让距离影响更敏感
        float normalized = Mathf.Exp(-dPath / (safeDist * 1.5f));
        return Mathf.Clamp01(normalized);
    }

    // ===== 替换 3：CalculateObstacleScore() - 只基于动态障碍物 =====
    private float CalculateObstacleScore(Vector3 predPos, float safeDist)
    {
        float minDist = float.MaxValue;

        // 只基于动态障碍物计算距离评分（固定障碍物由全局A*栅格处理）
        foreach (var obs in dynamicObstacles)
        {
            float d = Vector3.Distance(predPos, obs);
            if (d < minDist) minDist = d;
        }

        if (minDist < collisionHardBoundary)
            return -10f;
        else if (minDist < safeDist)
            return (minDist - collisionHardBoundary) / (safeDist - collisionHardBoundary);
        else
            return 1f;
    }

    private float CalculateSmoothScore(float linear, float angular, float currentSpeed)
    {
        float linearScore = 1f - Mathf.Abs(linear - currentSpeed) / maxLinearSpeed;
        float angularScore = 1f - Mathf.Abs(angular) / maxAngularSpeed;
        return (linearScore + angularScore) / 2f;
    }

    private float CalculateCOLREGSScore(float angular, Vector3 predPos)
    {
        if (dynamicObstacles.Count == 0)
            return 1f;

        float score = 1f;

        foreach (var obsPos in dynamicObstacles)
        {
            Vector3 relPos = obsPos - transform.position;
            float relAngle = Vector3.SignedAngle(transform.forward, relPos, Vector3.up);
            float absRelAngle = Mathf.Abs(relAngle);

            COLREGsState state = DetermineCOLREGsState(relAngle);

            float omegaRef = GetOmegaRef(state);
            float deviation = Mathf.Abs(angular - omegaRef);

            if (state == COLREGsState.HeadOn || state == COLREGsState.CrossingFromStarboard)
            {
                score *= Mathf.Max(0f, 1f - deviation / (maxAngularSpeed * 1.5f));
            }
            else if (state == COLREGsState.CrossingFromPort)
            {
                score *= Mathf.Max(0f, 1f - Mathf.Abs(angular) / (maxAngularSpeed * 0.5f));
            }

            float dcpa = CalculateDCPA(predPos, obsPos, angular);
            if (dcpa < 2.0f)
            {
                score *= Mathf.Max(0f, dcpa / 2.0f);
            }
        }

        return Mathf.Clamp01(score);
    }

    // ============================================================
    // COLREGs 辅助方法
    // ============================================================

    private enum COLREGsState
    {
        Normal,
        HeadOn,
        CrossingFromStarboard,
        CrossingFromPort
    }

    private COLREGsState DetermineCOLREGsState(float relAngle)
    {
        float absAngle = Mathf.Abs(relAngle);

        if (absAngle > 157.5f && absAngle < 202.5f)
            return COLREGsState.HeadOn;

        if (relAngle > 202.5f && relAngle < 337.5f)
            return COLREGsState.CrossingFromStarboard;

        if (relAngle > 22.5f && relAngle < 157.5f)
            return COLREGsState.CrossingFromPort;

        return COLREGsState.Normal;
    }

    private float GetOmegaRef(COLREGsState state)
    {
        switch (state)
        {
            case COLREGsState.HeadOn: return omegaRefHO;
            case COLREGsState.CrossingFromStarboard: return omegaRefSC;
            case COLREGsState.CrossingFromPort: return omegaRefPC;
            default: return 0f;
        }
    }

    private float CalculateDCPA(Vector3 usvPos, Vector3 obsPos, float angular)
    {
        Vector3 relPos = obsPos - usvPos;
        Vector3 usvVel = transform.forward * maxLinearSpeed;
        Vector3 obsVel = Vector3.zero;

        int idx = GetNearestObstacleIndex();
        if (idx >= 0 && idx < dynamicObstacleVelocities.Count)
            obsVel = dynamicObstacleVelocities[idx];

        Vector3 relVel = obsVel - usvVel;
        float relSpeed = relVel.magnitude;

        if (relSpeed < 0.01f)
            return relPos.magnitude;

        float t = -Vector3.Dot(relPos, relVel) / (relSpeed * relSpeed);
        if (t < 0) t = 0;

        Vector3 closestPos = relPos + relVel * t;
        return closestPos.magnitude;
    }

    // ============================================================
    // 安全校验
    // ============================================================

    private bool IsTrajectoryInCollision(float linear, float angular)
    {
        Vector3 pos = transform.position;
        float heading = transform.eulerAngles.y * Mathf.Deg2Rad;
        float dt = predictTime / predictSteps;
        Vector3 finalPos = pos;

        for (int i = 0; i < predictSteps; i++)
        {
            heading += angular * dt;
            pos += new Vector3(Mathf.Sin(heading), 0, Mathf.Cos(heading)) * linear * dt;

            // ===== 1. 动态障碍物检测（每一步都检测，因为它们会动）=====
            foreach (var obs in dynamicObstacles)
            {
                float d = Vector3.Distance(pos, obs);
                if (d < collisionHardBoundary)
                {
                    // 限频日志：每3秒最多打印一次
                    if (Time.time - lastWarningLogTime > 3f)
                    {
                        lastWarningLogTime = Time.time;
                        Debug.LogWarning($"[DWA碰撞] 动态障碍物阻挡: 距离={d:F2}, 步={i}");
                    }
                    return true;
                }
            }

            finalPos = pos;
        }

        // ===== 2. 固定障碍物栅格兜底：只检测轨迹终点，且加边界缓冲 =====
        if (IsTrajectoryInStaticGridCollision(finalPos))
        {
            // 限频日志：每3秒最多打印一次
            if (Time.time - lastWarningLogTime > 3f)
            {
                lastWarningLogTime = Time.time;
                Debug.LogWarning($"[DWA碰撞] 终点深入静态障碍物区域: {finalPos}");
            }
            return true;
        }

        return false;
    }

    /// <summary>
    /// 轻量级栅格静态碰撞检测：带边界缓冲，避免轻微擦边导致所有轨迹无效
    /// </summary>
    private bool IsTrajectoryInStaticGridCollision(Vector3 pos)
    {
        if (gridManager == null) return false;
        Vector2Int gridPos = gridManager.WorldToGrid(pos);

        // 可通行 → 安全
        if (gridManager.IsGridPassable(gridPos)) return false;

        // 不可通行 → 检查是否只是"擦边"
        Vector3 gridCenter = gridManager.GridToWorld(gridPos);
        float halfCell = gridManager.gridCellSize * 0.5f;

        // 计算轨迹点到最近栅格边界的距离
        float distToEdgeX = halfCell - Mathf.Abs(pos.x - gridCenter.x);
        float distToEdgeZ = halfCell - Mathf.Abs(pos.z - gridCenter.z);
        float minDistToEdge = Mathf.Min(distToEdgeX, distToEdgeZ);

        // 如果轨迹点只是刚越过边界（<0.25m），不算碰撞
        // 只有深入不可通行区域内部才判定
        const float EDGE_BUFFER = 0.25f;
        if (minDistToEdge < EDGE_BUFFER) return false;

        return true;
    }

    // ============================================================
    // 动态安全距离 (论文公式16)
    // ============================================================
    private float CalculateDynamicSafeDistance(float currentSpeed)
    {
        float relSpeed = 0f;
        if (dynamicObstacles.Count > 0)
        {
            int nearestIdx = GetNearestObstacleIndex();
            if (nearestIdx >= 0 && nearestIdx < dynamicObstacleVelocities.Count)
            {
                Vector3 relVel = dynamicObstacleVelocities[nearestIdx] - (rb != null ? rb.linearVelocity : Vector3.zero);
                relSpeed = relVel.magnitude;
            }
        }
        return d0 + tau * currentSpeed + cv * Mathf.Max(relSpeed, 0f);
    }

    private Vector3 GetNextGlobalWaypoint()
    {
        if (globalPathfinder == null || globalPathfinder.path == null || globalPathfinder.path.Count == 0)
            return globalAgent?.target?.position ?? transform.position + transform.forward * 10f;

        if (gridManager == null)
            return globalAgent?.target?.position ?? transform.position + transform.forward * 10f;

        int idx = globalAgent != null
            ? Mathf.Min(globalAgent.currentWaypointIndex, globalPathfinder.path.Count - 1)
            : Mathf.Min(currentPathIndex, globalPathfinder.path.Count - 1);

        Vector3 waypoint = gridManager.GridToWorld(globalPathfinder.path[idx]);
        waypoint.y = 0.4f;
        return waypoint;
    }

    // ============================================================
    // 动态障碍物检测与追踪
    // ============================================================

    // ===== 替换 1：IsDynamicObstacleClass() =====
    /// <summary>
    /// 判断是否为真正的动态障碍物（船只类）
    /// 固定障碍物（rock, buoy, obstacle）由全局A*栅格处理，不进入DWA动态列表
    /// </summary>
    private bool IsDynamicObstacleClass(string className)
    {
        string lower = className.ToLower();
        return lower.Contains("boat") || lower.Contains("ship") ||
               lower.Contains("unmanned") || lower.Contains("vessel");
    }

    // ===== 替换 2：DetectAndPredictDynamicObstacles() =====
    private void DetectAndPredictDynamicObstacles()
    {
        dynamicObstacles.Clear();
        dynamicObstacleVelocities.Clear();

        // ===== 1. YOLO视觉检测：只保留真正的动态障碍物（船只类）=====
        if (yoloDetector != null && yoloDetector.DetectedResults != null)
        {
            foreach (var result in yoloDetector.DetectedResults)
            {
                if (!IsDynamicObstacleClass(result.ClassName))
                    continue;

                Vector3 worldPos = ConvertYoloToWorldPosition(result.Rect);

                if (gridManager != null)
                {
                    if (worldPos.x < gridManager.WaterMinX || worldPos.x > gridManager.WaterMaxX ||
                        worldPos.z < gridManager.WaterMinZ || worldPos.z > gridManager.WaterMaxZ)
                        continue;
                }

                int trackId = result.TrackId >= 0 ? result.TrackId : result.GetHashCode();
                Vector3 velocity = EstimateObstacleVelocity(trackId, worldPos);
                Vector3 predPos = worldPos + velocity * predictTime;

                dynamicObstacles.Add(predPos);
                dynamicObstacleVelocities.Add(velocity);
            }
        }

        // ===== 2. 补充扫描：检测场景中的其他USV/DynamicObstacle（防止YOLO漏检或遮挡）=====
        Collider[] nearbyVessels = Physics.OverlapSphere(transform.position, 15f);
        foreach (var col in nearbyVessels)
        {
            if (col.gameObject == gameObject) continue;
            if (col.CompareTag("USV") || col.CompareTag("DynamicObstacle"))
            {
                bool alreadyTracked = false;
                for (int i = 0; i < dynamicObstacles.Count; i++)
                {
                    if (Vector3.Distance(dynamicObstacles[i], col.transform.position) < 1.0f)
                    {
                        alreadyTracked = true;
                        break;
                    }
                }
                if (!alreadyTracked)
                {
                    dynamicObstacles.Add(col.transform.position);
                    Rigidbody otherRb = col.GetComponent<Rigidbody>();
                    dynamicObstacleVelocities.Add(otherRb != null ? otherRb.linearVelocity : Vector3.zero);
                }
            }
        }

        CleanupObstacleHistory();
    }

    private Vector3 ConvertYoloToWorldPosition(OpenCvSharp.Rect rect)
    {
        Camera cam = yoloDetector?.sceneCamera ?? Camera.main;
        if (cam == null) return transform.position + transform.forward * 5f;

        float cx = (float)(rect.X + rect.Width / 2);
        float cy = (float)(rect.Y + rect.Height / 2);

        Vector3 viewportPos = new Vector3(
            cx / cam.pixelWidth,
            1 - cy / cam.pixelHeight,
            1f
        );

        Ray ray = cam.ViewportPointToRay(viewportPos);
        Plane groundPlane = new Plane(Vector3.up, 0.4f);

        if (groundPlane.Raycast(ray, out float distance))
        {
            Vector3 pos = ray.GetPoint(distance);
            pos.y = 0.4f;
            return pos;
        }

        return transform.position + transform.forward * 5f;
    }

    private Vector3 EstimateObstacleVelocity(int trackId, Vector3 currentPos)
    {
        if (!obstacleHistory.ContainsKey(trackId))
            obstacleHistory[trackId] = new List<(Vector3 pos, float time)>();

        var history = obstacleHistory[trackId];
        history.Add((currentPos, Time.time));
        obstacleLastSeen[trackId] = Time.time;

        while (history.Count > 30)
            history.RemoveAt(0);

        if (history.Count < 5)
            return Vector3.zero;

        int n = history.Count;
        float sumT = 0, sumX = 0, sumZ = 0;
        float sumT2 = 0, sumTX = 0, sumTZ = 0;

        for (int i = 0; i < n; i++)
        {
            float t = history[i].time;
            Vector3 pos = history[i].pos;
            sumT += t;
            sumX += pos.x;
            sumZ += pos.z;
            sumT2 += t * t;
            sumTX += t * pos.x;
            sumTZ += t * pos.z;
        }

        float denominator = n * sumT2 - sumT * sumT;
        if (denominator < 0.001f)
            return Vector3.zero;

        float vx = (n * sumTX - sumT * sumX) / denominator;
        float vz = (n * sumTZ - sumT * sumZ) / denominator;

        return new Vector3(vx, 0, vz);
    }

    private int GetNearestObstacleIndex()
    {
        if (dynamicObstacles.Count == 0) return -1;

        int idx = 0;
        float minDist = float.MaxValue;
        for (int i = 0; i < dynamicObstacles.Count; i++)
        {
            float d = Vector3.Distance(transform.position, dynamicObstacles[i]);
            if (d < minDist)
            {
                minDist = d;
                idx = i;
            }
        }
        return idx;
    }

    private void CleanupObstacleHistory()
    {
        List<int> toRemove = new List<int>();
        foreach (var kvp in obstacleLastSeen)
        {
            if (Time.time - kvp.Value > 3f)
                toRemove.Add(kvp.Key);
        }

        foreach (int id in toRemove)
        {
            obstacleHistory.Remove(id);
            obstacleLastSeen.Remove(id);
        }
    }

    // ============================================================
    // 物理控制（核心修改 - 使用 AddForce 方式）
    // ============================================================
    private void ApplyPhysicalControl(Vector3 targetVel, float targetAngular)
    {
        if (rb == null) return;

        // ====== 入口日志 ======
        if (Time.frameCount % 30 == 0)
        {
            //  Debug.Log($"[DWA] ApplyPhysicalControl: targetVel={targetVel.magnitude:F2}, targetAngular={targetAngular:F2}");
        }

        // 唤醒刚体
        if (rb.IsSleeping()) rb.WakeUp();

        float targetSpeed = targetVel.magnitude;

        if (targetSpeed < 0.01f)
        {
            // 停止前进
            rb.linearVelocity = Vector3.Lerp(rb.linearVelocity, Vector3.zero, Time.fixedDeltaTime * 3f);

            // 但保留转向能力（原地旋转绕过障碍物）
            float torque = Mathf.Clamp(targetAngular * 8f, -maxAngularSpeed, maxAngularSpeed);
            rb.AddTorque(Vector3.up * torque, ForceMode.Acceleration);

            if (Mathf.Abs(rb.angularVelocity.y) > maxAngularSpeed * 1.2f)
            {
                rb.angularVelocity = new Vector3(0, Mathf.Sign(rb.angularVelocity.y) * maxAngularSpeed * 1.2f, 0);
            }
            return;
        }

        // ====== 直接控制速度（使用 AddForce） ======
        Vector3 targetDir = targetVel.normalized;
        float targetSpeedClamped = Mathf.Min(targetSpeed, maxLinearSpeed);

        // 计算目标速度向量
        Vector3 targetWorldVel = targetDir * targetSpeedClamped;

        // 使用力来达到目标速度（PID风格）
        Vector3 force = (targetWorldVel - rb.linearVelocity) * 8f;  // 增加力
        rb.AddForce(force, ForceMode.Force);

        // 限制最大速度
        if (rb.linearVelocity.magnitude > maxLinearSpeed * 1.2f)
        {
            rb.linearVelocity = rb.linearVelocity.normalized * maxLinearSpeed * 1.2f;
        }

        // ====== 转向控制 ======
        // 计算目标角度和当前角度的差值
        float currentAngle = transform.eulerAngles.y * Mathf.Deg2Rad;
        float targetAngle = Mathf.Atan2(targetDir.x, targetDir.z);
        float angleDiff = Mathf.DeltaAngle(currentAngle * Mathf.Rad2Deg, targetAngle * Mathf.Rad2Deg) * Mathf.Deg2Rad;

        // 应用转向扭矩
        float steerTorque = Mathf.Clamp(angleDiff * 5f, -maxAngularSpeed, maxAngularSpeed);
        rb.AddTorque(Vector3.up * steerTorque, ForceMode.Acceleration);


        // 限制角速度
        if (Mathf.Abs(rb.angularVelocity.y) > maxAngularSpeed * 1.2f)
        {
            rb.angularVelocity = new Vector3(0, Mathf.Sign(rb.angularVelocity.y) * maxAngularSpeed * 1.2f, 0);
        }

        // 平滑角速度（可选）
        smoothOmega = Mathf.Lerp(smoothOmega, rb.angularVelocity.y, Time.fixedDeltaTime * 2f);
        smoothSpeed = Mathf.Lerp(smoothSpeed, rb.linearVelocity.magnitude, Time.fixedDeltaTime * 2f);
    }

    // ============================================================
    // 公共接口
    // ============================================================
    public Vector3 GetCurrentTargetVelocity() => currentTargetVelocity;
    public float GetCurrentTargetAngular() => currentTargetAngular;
    public bool HasValidControl() => hasValidControl;

    public void ResetControl()
    {
        hasValidControl = false;
        currentTargetVelocity = Vector3.zero;
        currentTargetAngular = 0f;
        smoothOmega = 0f;
        smoothSpeed = 0f;
    }

    void OnDestroy()
    {
        obstacleHistory.Clear();
        obstacleLastSeen.Clear();
    }

    // ============================================================
    // Gizmos调试
    // ============================================================
    void OnDrawGizmos()
    {
        if (!Application.isPlaying) return;

        Gizmos.color = Color.red;
        foreach (var obs in dynamicObstacles)
        {
            Gizmos.DrawWireSphere(obs, 0.3f);
        }

        if (hasValidControl && rb != null)
        {
            Gizmos.color = Color.cyan;
            Gizmos.DrawRay(transform.position, currentTargetVelocity.normalized * 1.5f);
            Gizmos.color = Color.yellow;
            Gizmos.DrawRay(transform.position, transform.forward * 1.5f);
        }

        if (gridManager != null)
        {
            Gizmos.color = new Color(0, 1, 0, 0.1f);
            float safeDist = CalculateDynamicSafeDistance(rb != null ? rb.linearVelocity.magnitude : 0f);
            Gizmos.DrawWireSphere(transform.position, safeDist);
            Gizmos.color = new Color(1, 0, 0, 0.1f);
            Gizmos.DrawWireSphere(transform.position, collisionHardBoundary);
        }
    }
}
#pragma warning restore 0414