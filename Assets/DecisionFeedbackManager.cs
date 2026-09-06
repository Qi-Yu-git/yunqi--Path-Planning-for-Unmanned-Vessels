using UnityEngine;
using System.Collections.Generic;
using USVGridSystem;
/// <summary>
/// 决策层反馈数据结构
/// 从 PPO 决策层反馈到 A* 规划层
/// </summary>
[System.Serializable]
public struct DecisionFeedback
{
    public float colregsComplianceScore;
    public float minCPA;
    public float collisionRiskLevel;
    public bool needReplan;
    public Vector2 pathCorrectionOffset;
    public float safeDistanceMultiplier;
    public float speedMultiplier;
    public Vector3 nearestDynamicObstacle;
    public Vector3 nearestDynamicObstacleVelocity;
    public int obstacleCount;

    public override string ToString()
    {
        return $"Feedback: COLREGs={colregsComplianceScore:F2}, CPA={minCPA:F2}, Risk={collisionRiskLevel:F2}, Replan={needReplan}";
    }
}

/// <summary>
/// 决策反馈管理器 (单例)
/// 连接 USV_GlobalRLAgent 和 ImprovedAStar
/// </summary>
public class DecisionFeedbackManager : MonoBehaviour  // ← 确保继承 MonoBehaviour
{
    #region 单例
    public static DecisionFeedbackManager Instance { get; private set; }

    private void Awake()
    {
        if (Instance != null && Instance != this)
        {
            Destroy(gameObject);
            return;
        }
        Instance = this;
        //DontDestroyOnLoad(gameObject);
    }
    #endregion

    [Header("反馈配置")]
    public int feedbackUpdateInterval = 10;
    public float replanThreshold = 0.4f;
    public float riskThreshold = 0.6f;

    [Header("动态权重调整")]
    public Vector2 safeDistRange = new Vector2(0.8f, 1.5f);
    public Vector2 speedRange = new Vector2(0.5f, 1.2f);

    private DecisionFeedback currentFeedback;
    public DecisionFeedback CurrentFeedback => currentFeedback;

    private List<DecisionFeedback> feedbackHistory = new List<DecisionFeedback>();
    private const int HISTORY_LENGTH = 10;

    private USV_GlobalRLAgent globalAgent;
    private USV_LocalPlanner localPlanner;
    private ImprovedAStar pathfinder;
    private GridManager gridManager;

    private int stepCounter = 0;

    void Start()
    {
        globalAgent = FindFirstObjectByType<USV_GlobalRLAgent>();
        localPlanner = FindFirstObjectByType<USV_LocalPlanner>();
        pathfinder = FindFirstObjectByType<ImprovedAStar>();
        gridManager = FindFirstObjectByType<GridManager>();

        if (globalAgent == null)
        {
            Debug.LogWarning("[DecisionFeedback] 未找到 USV_GlobalRLAgent!");
        }
    }

    public void UpdateFeedback()
    {
        stepCounter++;

        if (stepCounter % feedbackUpdateInterval != 0)
            return;

        DecisionFeedback newFeedback = CollectFeedback();
        AddToHistory(newFeedback);
        currentFeedback = GetSmoothedFeedback();
        ApplyFeedbackToPathfinder(currentFeedback);

        if (stepCounter % 100 == 0)
        {
            Debug.Log($"[DecisionFeedback] Step {stepCounter}: {currentFeedback}");
        }
    }

    private DecisionFeedback CollectFeedback()
    {
        DecisionFeedback feedback = new DecisionFeedback();
        feedback.colregsComplianceScore = CalculateCOLREGsCompliance();
        feedback.minCPA = CalculateMinCPA();
        feedback.collisionRiskLevel = CalculateCollisionRisk();
        feedback.needReplan = feedback.colregsComplianceScore < replanThreshold ||
                             feedback.collisionRiskLevel > riskThreshold;
        feedback.pathCorrectionOffset = CalculatePathCorrectionOffset();
        feedback.safeDistanceMultiplier = Mathf.Lerp(safeDistRange.x, safeDistRange.y, 1f - feedback.colregsComplianceScore);
        feedback.speedMultiplier = Mathf.Lerp(speedRange.x, speedRange.y, 1f - feedback.collisionRiskLevel);

        if (localPlanner != null && localPlanner.dynamicObstacles.Count > 0)
        {
            int nearestIdx = GetNearestObstacleIndex();
            feedback.nearestDynamicObstacle = localPlanner.dynamicObstacles[nearestIdx];
            if (localPlanner.dynamicObstacleVelocities.Count > nearestIdx)
            {
                feedback.nearestDynamicObstacleVelocity = localPlanner.dynamicObstacleVelocities[nearestIdx];
            }
            feedback.obstacleCount = localPlanner.dynamicObstacles.Count;
        }

        return feedback;
    }

    private float CalculateCOLREGsCompliance()
    {
        if (localPlanner == null) return 0.5f;
        return 0.5f;
    }

    private float CalculateMinCPA()
    {
        if (localPlanner == null || localPlanner.dynamicObstacles.Count == 0)
            return 100f;

        float minDist = float.MaxValue;
        Vector3 usvPos = localPlanner.transform.position;

        foreach (var obs in localPlanner.dynamicObstacles)
        {
            float dist = Vector3.Distance(usvPos, obs);
            if (dist < minDist)
                minDist = dist;
        }

        return minDist;
    }

    private float CalculateCollisionRisk()
    {
        float cpa = CalculateMinCPA();
        if (cpa > 20f) return 0f;
        float risk = 1f - Mathf.Clamp01(cpa / 20f);

        if (localPlanner != null && localPlanner.dynamicObstacleVelocities.Count > 0)
        {
            float maxSpeed = 0f;
            foreach (var vel in localPlanner.dynamicObstacleVelocities)
            {
                float speed = vel.magnitude;
                if (speed > maxSpeed) maxSpeed = speed;
            }
            risk = Mathf.Clamp01(risk + maxSpeed * 0.02f);
        }

        return risk;
    }

    private Vector2 CalculatePathCorrectionOffset()
    {
        Vector2 correction = Vector2.zero;

        if (localPlanner == null || localPlanner.dynamicObstacles.Count == 0)
            return correction;

        Vector3 obsCenter = Vector3.zero;
        foreach (var obs in localPlanner.dynamicObstacles)
        {
            obsCenter += obs;
        }
        obsCenter /= localPlanner.dynamicObstacles.Count;

        Vector3 dirToCenter = (obsCenter - localPlanner.transform.position).normalized;
        Vector3 perpDir = Vector3.Cross(dirToCenter, Vector3.up).normalized;

        int leftCount = 0, rightCount = 0;
        foreach (var obs in localPlanner.dynamicObstacles)
        {
            Vector3 toObs = obs - localPlanner.transform.position;
            float side = Vector3.Dot(toObs, perpDir);
            if (side > 0) rightCount++;
            else leftCount++;
        }

        float sideFactor = rightCount > leftCount ? -1f : 1f;
        correction = new Vector2(perpDir.x * sideFactor, perpDir.z * sideFactor);

        return correction;
    }

    private int GetNearestObstacleIndex()
    {
        if (localPlanner == null || localPlanner.dynamicObstacles.Count == 0)
            return -1;

        float minDist = float.MaxValue;
        int idx = 0;
        Vector3 usvPos = localPlanner.transform.position;

        for (int i = 0; i < localPlanner.dynamicObstacles.Count; i++)
        {
            float dist = Vector3.Distance(usvPos, localPlanner.dynamicObstacles[i]);
            if (dist < minDist)
            {
                minDist = dist;
                idx = i;
            }
        }

        return idx;
    }

    private void ApplyFeedbackToPathfinder(DecisionFeedback feedback)
    {
        if (pathfinder == null) return;

        pathfinder.safeCostWeight = 3.0f * feedback.safeDistanceMultiplier;

        if (feedback.needReplan && globalAgent != null)
        {
            pathfinder.CalculatePathAfterDelay();
            Debug.Log($"[DecisionFeedback] 触发路径重规划");
        }

        if (feedback.pathCorrectionOffset.magnitude > 0.1f)
        {
            pathfinder.SetPathOffset(feedback.pathCorrectionOffset);
        }
    }

    private void AddToHistory(DecisionFeedback feedback)
    {
        feedbackHistory.Add(feedback);
        if (feedbackHistory.Count > HISTORY_LENGTH)
        {
            feedbackHistory.RemoveAt(0);
        }
    }

    private DecisionFeedback GetSmoothedFeedback()
    {
        if (feedbackHistory.Count == 0)
            return new DecisionFeedback();

        DecisionFeedback avg = new DecisionFeedback();
        int count = feedbackHistory.Count;

        foreach (var fb in feedbackHistory)
        {
            avg.colregsComplianceScore += fb.colregsComplianceScore;
            avg.minCPA += fb.minCPA;
            avg.collisionRiskLevel += fb.collisionRiskLevel;
            avg.needReplan = avg.needReplan || fb.needReplan;
        }

        avg.colregsComplianceScore /= count;
        avg.minCPA /= count;
        avg.collisionRiskLevel /= count;
        avg.pathCorrectionOffset = feedbackHistory[feedbackHistory.Count - 1].pathCorrectionOffset;
        avg.safeDistanceMultiplier = feedbackHistory[feedbackHistory.Count - 1].safeDistanceMultiplier;
        avg.speedMultiplier = feedbackHistory[feedbackHistory.Count - 1].speedMultiplier;
        avg.nearestDynamicObstacle = feedbackHistory[feedbackHistory.Count - 1].nearestDynamicObstacle;
        avg.nearestDynamicObstacleVelocity = feedbackHistory[feedbackHistory.Count - 1].nearestDynamicObstacleVelocity;
        avg.obstacleCount = feedbackHistory[feedbackHistory.Count - 1].obstacleCount;

        return avg;
    }

    public float GetCurrentCOLREGsScore()
    {
        return currentFeedback.colregsComplianceScore;
    }

    public float GetCurrentRiskLevel()
    {
        return currentFeedback.collisionRiskLevel;
    }
}