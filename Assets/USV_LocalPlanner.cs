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
        rb.useGravity = false;
        rb.collisionDetectionMode = CollisionDetectionMode.ContinuousDynamic;

        dynamicObstacles = new List<Vector3>();
        dynamicObstacleVelocities = new List<Vector3>();

        if (yoloDetector == null)
        {
            yoloDetector = FindAnyObjectByType<YoloDetector>(FindObjectsInactive.Include);
            if (yoloDetector == null)
            {
                Debug.LogError("[USV_LocalPlanner] YoloDetector未找到！动态避障功能无法启用");
            }
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
        if (actions.ContinuousActions.Length >= 2)
        {
            float forward = actions.ContinuousActions[0];
            float turn = actions.ContinuousActions[1];
            rb.linearVelocity = transform.forward * forward * MaxLinearVel;
            transform.Rotate(0, turn * 60f * Time.deltaTime, 0);
        }
        else if (actions.DiscreteActions.Length >= 1)
        {
            var (vel, rot) = GetGlobalActionVelocity(actions.DiscreteActions[0]);
            rb.linearVelocity = new Vector3(vel.x, rb.linearVelocity.y, vel.z);
            transform.Rotate(0, rot * Time.deltaTime, 0);
        }
    }

    private void DetectAndPredictDynamicObstacles()
    {
        dynamicObstacles.Clear();
        dynamicObstacleVelocities.Clear();

        if (yoloDetector == null || yoloDetector.DetectedResults == null)
            return;

        List<YoloV8Detection.YoloResult> obstacleResults = yoloDetector.DetectedResults.FindAll(result =>
            result.ClassName.ToLower() == "unmanned boat" ||
            result.ClassName == "sports ball" ||
            result.ClassName == "mouse" ||
            result.ClassName == "rock" ||
            result.ClassName == "obstacle"
        );

        if (obstacleResults.Count == 0) return;

        for (int i = 0; i < obstacleResults.Count; i++)
        {
            YoloV8Detection.YoloResult result = obstacleResults[i];
            Vector3 worldPos = ConvertYoloToWorldPosition(result.Rect);

            if (gridManager != null)
            {
                if (worldPos.x < gridManager.WaterMinX || worldPos.x > gridManager.WaterMaxX ||
                    worldPos.z < gridManager.WaterMinZ || worldPos.z > gridManager.WaterMaxZ)
                {
                    continue;
                }
            }

            int obsId = result.TrackId >= 0 ? result.TrackId : i;
            Vector3 predictedVel = EstimateObstacleVelocity(obsId, worldPos, result.ClassName);
            dynamicObstacleVelocities.Add(predictedVel);

            Vector3 predictedPos = PredictObstaclePosition(worldPos, predictedVel, dwaPredictTime);
            dynamicObstacles.Add(predictedPos);

            bool collisionRisk = IsCollisionImminent(transform.position, rb.linearVelocity, worldPos, predictedVel, dwaPredictTime);
            if (collisionRisk)
            {
                isAvoidingDynamicObstacle = true;
            }

            if (result.ClassName.ToLower() == "unmanned boat" || result.ClassName.ToLower() == "sports ball")
            {
                isAvoidingDynamicObstacle = true;
            }
        }

        CleanupObstacleHistory();
    }

    private Vector3 ConvertYoloToWorldPosition(OpenCvSharp.Rect rect)
    {
        Camera detectCamera = yoloDetector != null ? yoloDetector.sceneCamera : Camera.main;
        if (detectCamera == null)
        {
            detectCamera = Camera.main;
            if (detectCamera == null)
            {
                Debug.LogError("[LocalPlanner] 未找到相机！");
                return transform.position + transform.forward * 5f;
            }
        }

        float screenX = Mathf.Clamp((float)rect.X + (float)rect.Width / 2, 0, detectCamera.pixelWidth);
        float screenY = Mathf.Clamp((float)rect.Y + (float)rect.Height / 2, 0, detectCamera.pixelHeight);
        Vector3 viewportPos = new Vector3(screenX / detectCamera.pixelWidth, 1 - screenY / detectCamera.pixelHeight, 1f);

        Ray ray = detectCamera.ViewportPointToRay(viewportPos);
        float groundY = 0.4f;
        Plane groundPlane = new Plane(Vector3.up, groundY);

        if (groundPlane.Raycast(ray, out float distance))
        {
            Vector3 worldPos = ray.GetPoint(distance);
            bool inWater = true;

            if (gridManager != null)
            {
                inWater = worldPos.x >= gridManager.WaterMinX && worldPos.x <= gridManager.WaterMaxX &&
                          worldPos.z >= gridManager.WaterMinZ && worldPos.z <= gridManager.WaterMaxZ;
            }

            if (inWater)
            {
                return new Vector3(worldPos.x, groundY, worldPos.z);
            }
            else
            {
                worldPos.x = Mathf.Clamp(worldPos.x, gridManager.WaterMinX, gridManager.WaterMaxX);
                worldPos.z = Mathf.Clamp(worldPos.z, gridManager.WaterMinZ, gridManager.WaterMaxZ);
                return new Vector3(worldPos.x, groundY, worldPos.z);
            }
        }

        return transform.position + transform.forward * 5f;
    }

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
        if (denominator < 0.001f)
        {
            return Vector3.zero;
        }

        float aX = (n * sumTX - sumT * sumX) / denominator;
        float aZ = (n * sumTZ - sumT * sumZ) / denominator;

        float speedFactor = className.ToLower() == "unmanned boat" ? 1.2f : obstacleSpeedFactor;
        Vector3 predictedVel = new Vector3(aX, 0, aZ) * speedFactor;

        return predictedVel;
    }

    private Vector3 PredictObstaclePosition(Vector3 currentPos, Vector3 velocity, float time)
    {
        return currentPos + velocity * time;
    }

    private bool IsCollisionImminent(Vector3 usvPos, Vector3 usvVel, Vector3 obsPos, Vector3 obsVel, float time)
    {
        Vector3 relativeVel = obsVel - usvVel;
        Vector3 relativePos = obsPos - usvPos;

        float warningDistance = localSafeDistance;
        if (relativePos.magnitude > warningDistance)
        {
            return false;
        }

        float dotProduct = Vector3.Dot(relativePos.normalized, relativeVel.normalized);
        if (dotProduct >= 0.1f)
        {
            return false;
        }

        float relativeSpeed = relativeVel.magnitude;
        if (relativeSpeed < 0.01f)
        {
            return relativePos.magnitude < collisionThreshold;
        }

        float ttc = relativePos.magnitude / relativeSpeed;
        if (ttc < 0 || ttc > time)
        {
            return false;
        }

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

        return isImminent;
    }

    private (Vector3 velocity, float rotation) LocalPlannerWithCOLREGs()
    {
        float bestScore = -Mathf.Infinity;
        Vector3 bestVelocity = Vector3.zero;
        float bestRotation = 0f;
        Vector3 nextGlobalWaypoint = GetCurrentGlobalWaypoint();

        foreach (float linearVel in linearVelOptions)
        {
            if (linearVel > MaxLinearVel * 0.8f) continue;

            foreach (float angularVel in angularVelOptions)
            {
                (Vector3 predictedPos, Quaternion predictedRot) = PredictMotion(linearVel, angularVel, dwaPredictTime);

                bool willCollide = dynamicObstacles.Any(obs =>
                    Vector3.Distance(predictedPos, obs) < collisionThreshold * 0.8f);
                if (willCollide)
                {
                    continue;
                }

                float obstacleScore = CalculateObstacleScore(predictedPos);
                float pathTrackScore = CalculatePathTrackScore(predictedPos, nextGlobalWaypoint);
                float colregsScore = CalculateCOLREGsScore(predictedPos, predictedRot, angularVel);
                float smoothScore = CalculateSmoothScore(linearVel, angularVel);

                float totalScore = pathTrackScore * 0.3f   // heading
                                 + obstacleScore * 0.4f    // dist
                                 + smoothScore * 0.2f      // velocity
                                 + colregsScore * 0.1f;    // COLREGs

                if (totalScore > bestScore)
                {
                    bestScore = totalScore;
                    bestVelocity = predictedRot * Vector3.forward * linearVel;
                    bestRotation = angularVel;
                }
            }
        }

        if (bestScore < 0)
        {
            Vector3 closestObs = dynamicObstacles.OrderBy(obs => Vector3.Distance(transform.position, obs)).First();
            float avoidDir = closestObs.x > transform.position.x ? -15f : 15f;
            bestVelocity = transform.forward * linearVelOptions[1];
            bestRotation = avoidDir;
        }

        return (bestVelocity, bestRotation);
    }

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

    private float CalculatePathTrackScore(Vector3 predictedPos, Vector3 nextWaypoint)
    {
        float distToWaypoint = Vector3.Distance(predictedPos, nextWaypoint);
        return Mathf.Clamp(1 - (distToWaypoint / (localSafeDistance * 2)), 0f, 1f);
    }

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

            if (Mathf.Abs(relativeAngle) < 20f && obsSpeed > 0.5f)
            {
                float rotationDiff = Mathf.Abs(currentAngularVel - HeadOnAvoidAngle);
                colregsScore *= Mathf.Clamp(1 - (rotationDiff / 60f), 0.2f, 1f);
            }
            else if (relativeAngle > 0f && relativeAngle < 120f)
            {
                colregsScore *= Mathf.Clamp(1 - (Mathf.Abs(currentAngularVel) / 20f), 0.3f, 1f);
            }
            else if (relativeAngle < 0f && relativeAngle > -120f)
            {
                float rotationDiff = Mathf.Abs(currentAngularVel - StarboardAvoidAngle);
                colregsScore *= Mathf.Clamp(1 - (rotationDiff / 50f), 0.2f, 1f);
            }

            float distToObs = Vector3.Distance(predictedPos, obsPos);
            if (distToObs < colregsSafeDistance)
            {
                colregsScore *= Mathf.Pow(distToObs / colregsSafeDistance, 2);
            }
        }

        return colregsScore;
    }

    private float CalculateSmoothScore(float linearVel, float angularVel)
    {
        float currentLinearVel = Vector3.Dot(transform.forward, rb.linearVelocity);
        float linearSmooth = 1 - Mathf.Abs(linearVel - currentLinearVel) / MaxLinearVel;
        float angularSmooth = 1 - Mathf.Abs(angularVel) / 60f;

        return (linearSmooth + angularSmooth) / 2f;
    }

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

    private bool IsCloseToGlobalPath()
    {
        if (globalPathfinder == null || globalPathfinder.path == null || globalPathfinder.path.Count < 2)
        {
            return true;
        }

        Vector3 closestPoint = GetClosestPointOnPath(transform.position);
        return Vector3.Distance(transform.position, closestPoint) < returnToPathThreshold;
    }

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

    private (Vector3 velocity, float rotation) GetGlobalActionVelocity(int action)
    {
        Vector3 velocity = Vector3.zero;
        float rotation = 0f;

        switch (action)
        {
            case 0:
                velocity = transform.forward * linearVelOptions[3];
                break;
            case 1:
                rotation = angularVelOptions[1];
                velocity = transform.forward * linearVelOptions[2];
                break;
            case 2:
                rotation = angularVelOptions[3];
                velocity = transform.forward * linearVelOptions[2];
                break;
            case 3:
                velocity = transform.forward * linearVelOptions[1];
                break;
            default:
                velocity = Vector3.zero;
                rotation = 0f;
                break;
        }

        return (velocity, rotation);
    }

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

    private void AddLocalPlanningRewards()
    {
        if (dynamicObstacles.Count == 0) return;

        float minDistToObs = float.MaxValue;
        foreach (Vector3 obsPos in dynamicObstacles)
        {
            minDistToObs = Mathf.Min(minDistToObs, Vector3.Distance(transform.position, obsPos));
        }

        globalAgent.AddReward(minDistToObs / localSafeDistance * 5f);
        float currentAngularVel = transform.eulerAngles.y * Time.deltaTime;
        float colregsReward = CalculateCOLREGsScore(transform.position, transform.rotation, currentAngularVel);
        globalAgent.AddReward(colregsReward * 0.8f);

        if (IsCloseToGlobalPath())
        {
            globalAgent.AddReward(0.5f);
        }

        if (minDistToObs < collisionThreshold * 0.5f)
        {
            globalAgent.AddReward(-20f);
            EndEpisode();
        }

        if (minDistToObs < collisionThreshold)
        {
            globalAgent.AddReward(-(collisionThreshold - minDistToObs) * 3f);
        }
    }

    private void EndEpisode()
    {
        globalAgent.EndEpisode();
    }

    private void OnDrawGizmos()
    {
        Gizmos.color = Color.red;
        foreach (var obs in dynamicObstacles)
        {
            Gizmos.DrawSphere(obs, 0.5f);
        }

        if (isAvoidingDynamicObstacle && rb != null)
        {
            Gizmos.color = Color.yellow;
            var (predictedPos, _) = PredictMotion(rb.linearVelocity.magnitude, transform.rotation.eulerAngles.y, 1f);
            Gizmos.DrawLine(transform.position, predictedPos);
        }

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

public partial class USV_LocalPlanner : MonoBehaviour
{
    public float predictionTime = 3.0f;
    public float collisionThreshold = 2f;
    public float avoidDistance = 4.0f;
    public float obstacleSpeedFactor = 0.9f;

    private Dictionary<int, List<(Vector3 pos, float time)>> obstacleHistory = new Dictionary<int, List<(Vector3 pos, float time)>>();
    private Dictionary<int, float> obstacleLastSeen = new Dictionary<int, float>();

    private Transform target => globalAgent != null ? globalAgent.target : null;
}