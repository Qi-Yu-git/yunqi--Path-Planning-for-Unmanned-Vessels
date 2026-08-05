// ============================================================
// 文件名: DecisionFeedbackInterface.cs
// 路径: Assets/Scripts/RL/DecisionFeedbackInterface.cs
// 描述: 决策层→规划层反馈接口 (三层闭环)
// 论文: Section 2.1 - 三层闭环反馈机制
// ============================================================

using UnityEngine;
using System.Collections.Generic;

/// <summary>
/// 决策层反馈数据结构
/// 从 PPO 决策层反馈到 A* 规划层
/// </summary>
[System.Serializable]
public struct DecisionFeedback
{
    [Header("航行合规度反馈")]
    [Tooltip("COLREGs 合规度评分 (0-1)")]
    public float colregsComplianceScore;

    [Tooltip("最小会遇距离 (CPA)")]
    public float minCPA;

    [Tooltip("碰撞风险等级 (0-1)")]
    public float collisionRiskLevel;

    [Header("路径修正建议")]
    [Tooltip("是否需要重新规划路径")]
    public bool needReplan;

    [Tooltip("路径修正方向 (偏移量)")]
    public Vector2 pathCorrectionOffset;

    [Tooltip("建议的安全距离系数")]
    public float safeDistanceMultiplier;

    [Tooltip("建议的巡航速度系数")]
    public float speedMultiplier;

    [Header("障碍物态势")]
    [Tooltip("最近动态障碍物位置")]
    public Vector3 nearestDynamicObstacle;

    [Tooltip("最近动态障碍物速度")]
    public Vector3 nearestDynamicObstacleVelocity;

    [Tooltip("障碍物数量")]
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
public class DecisionFeedbackManager : MonoBehaviour
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
        DontDestroyOnLoad(gameObject);
    }
    #endregion

    [Header("反馈配置")]
    [Tooltip("反馈更新间隔 (步数)")]
    public int feedbackUpdateInterval = 10;

    [Tooltip("路径重规划阈值 (COLREGs 评分低于此值触发)")]
    public float replanThreshold = 0.4f;

    [Tooltip("碰撞风险阈值")]
    public float riskThreshold = 0.6f;

    [Header("动态权重调整")]
    [Tooltip("安全距离系数范围")]
    public Vector2 safeDistRange = new Vector2(0.8f, 1.5f);

    [Tooltip("速度系数范围")]
    public Vector2 speedRange = new Vector2(0.5f, 1.2f);

    // 当前反馈 (最新)
    private DecisionFeedback currentFeedback;
    public DecisionFeedback CurrentFeedback => currentFeedback;

    // 反馈历史 (用于平滑)
    private List<DecisionFeedback> feedbackHistory = new List<DecisionFeedback>();
    private const int HISTORY_LENGTH = 10;

    private USV_GlobalRLAgent globalAgent;
    private USV_LocalPlanner localPlanner;
    private ImprovedAStar pathfinder;
    private GridManager gridManager;

    private int stepCounter = 0;

    void Start()
    {
        // 获取引用
        globalAgent = FindFirstObjectByType<USV_GlobalRLAgent>();
        localPlanner = FindFirstObjectByType<USV_LocalPlanner>();
        pathfinder = FindFirstObjectByType<ImprovedAStar>();
        gridManager = FindFirstObjectByType<GridManager>();

        if (globalAgent == null)
        {
            Debug.LogError("[DecisionFeedback] 未找到 USV_GlobalRLAgent!");
        }
    }

    /// <summary>
    /// 在每一步更新反馈 (由 GlobalRLAgent 调用)
    /// </summary>
    public void UpdateFeedback()
    {
        stepCounter++;

        // 间隔更新 (减少性能开销)
        if (stepCounter % feedbackUpdateInterval != 0)
            return;

        // 收集各层数据
        DecisionFeedback newFeedback = CollectFeedback();

        // 平滑处理
        AddToHistory(newFeedback);
        currentFeedback = GetSmoothedFeedback();

        // 应用反馈到规划层
        ApplyFeedbackToPathfinder(currentFeedback);

        // 日志输出 (调试)
        if (stepCounter % 100 == 0)
        {
            Debug.Log($"[DecisionFeedback] Step {stepCounter}: {currentFeedback}");
        }
    }

    /// <summary>
    /// 从各模块收集反馈数据
    /// </summary>
    private DecisionFeedback CollectFeedback()
    {
        DecisionFeedback feedback = new DecisionFeedback();

        // 1. COLREGs 合规度 (从 LocalPlanner 获取)
        feedback.colregsComplianceScore = CalculateCOLREGsCompliance();

        // 2. 最小会遇距离 CPA
        feedback.minCPA = CalculateMinCPA();

        // 3. 碰撞风险等级
        feedback.collisionRiskLevel = CalculateCollisionRisk();

        // 4. 是否需要重规划
        feedback.needReplan = feedback.colregsComplianceScore < replanThreshold ||
                             feedback.collisionRiskLevel > riskThreshold;

        // 5. 路径修正偏移 (基于障碍物态势)
        feedback.pathCorrectionOffset = CalculatePathCorrectionOffset();

        // 6. 安全距离系数 (动态调整)
        feedback.safeDistanceMultiplier = Mathf.Lerp(
            safeDistRange.x, safeDistRange.y,
            1f - feedback.colregsComplianceScore
        );

        // 7. 速度系数 (风险越高速度越慢)
        feedback.speedMultiplier = Mathf.Lerp(
            speedRange.x, speedRange.y,
            1f - feedback.collisionRiskLevel
        );

        // 8. 最近动态障碍物信息
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

    /// <summary>
    /// 计算 COLREGs 合规评分
    /// </summary>
    private float CalculateCOLREGsCompliance()
    {
        if (localPlanner == null) return 0.5f;

        // 复用 LocalPlanner 的 COLREGs 评分逻辑
        float currentAngularVel = localPlanner.transform.rotation.eulerAngles.y * Time.deltaTime;
        // 简化：从 LocalPlanner 获取当前评分
        // 实际上需要调用 LocalPlanner.CalculateCOLREGsScore
        return 0.5f; // TODO: 调用实际方法
    }

    /// <summary>
    /// 计算最小会遇距离 (CPA)
    /// </summary>
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

    /// <summary>
    /// 计算碰撞风险等级
    /// </summary>
    private float CalculateCollisionRisk()
    {
        float cpa = CalculateMinCPA();
        if (cpa > 20f) return 0f;

        // 基于 CPA 计算风险 (距离越近风险越高)
        float risk = 1f - Mathf.Clamp01(cpa / 20f);

        // 考虑相对速度 (如果有)
        if (localPlanner != null && localPlanner.dynamicObstacleVelocities.Count > 0)
        {
            float maxSpeed = 0f;
            foreach (var vel in localPlanner.dynamicObstacleVelocities)
            {
                float speed = vel.magnitude;
                if (speed > maxSpeed) maxSpeed = speed;
            }
            // 速度越快风险越高
            risk = Mathf.Clamp01(risk + maxSpeed * 0.02f);
        }

        return risk;
    }

    /// <summary>
    /// 计算路径修正偏移方向
    /// 基于障碍物分布，计算安全方向
    /// </summary>
    private Vector2 CalculatePathCorrectionOffset()
    {
        Vector2 correction = Vector2.zero;

        if (localPlanner == null || localPlanner.dynamicObstacles.Count == 0)
            return correction;

        // 计算障碍物中心方向 (应避开的方向)
        Vector3 obsCenter = Vector3.zero;
        foreach (var obs in localPlanner.dynamicObstacles)
        {
            obsCenter += obs;
        }
        obsCenter /= localPlanner.dynamicObstacles.Count;

        Vector3 dirToCenter = (obsCenter - localPlanner.transform.position).normalized;

        // 修正方向：垂直于障碍物中心方向 (向左或向右)
        Vector3 perpDir = Vector3.Cross(dirToCenter, Vector3.up).normalized;

        // 选择安全侧 (选择障碍物较少的一侧)
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

    /// <summary>
    /// 获取最近障碍物索引
    /// </summary>
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

    /// <summary>
    /// 将反馈应用到 A* 路径规划器
    /// </summary>
    private void ApplyFeedbackToPathfinder(DecisionFeedback feedback)
    {
        if (pathfinder == null) return;

        // 1. 动态调整安全距离权重
        pathfinder.safeCostWeight = 3.0f * feedback.safeDistanceMultiplier;

        // 2. 动态调整路径搜索范围 (风险高时扩大搜索)
        // 如果有搜索范围参数，在此调整

        // 3. 是否需要重规划路径
        if (feedback.needReplan && globalAgent != null)
        {
            // 触发局部路径重规划
            pathfinder.CalculatePathAfterDelay();
            Debug.Log($"[DecisionFeedback] 触发路径重规划 (COLREGs={feedback.colregsComplianceScore:F2}, Risk={feedback.collisionRiskLevel:F2})");
        }

        // 4. 路径偏移修正 (将障碍物信息传递给 A*)
        if (feedback.pathCorrectionOffset.magnitude > 0.1f)
        {
            // 可以将偏移量传递给 A* 作为路径修正
            pathfinder.SetPathOffset(feedback.pathCorrectionOffset);
        }
    }

    /// <summary>
    /// 添加反馈到历史
    /// </summary>
    private void AddToHistory(DecisionFeedback feedback)
    {
        feedbackHistory.Add(feedback);
        if (feedbackHistory.Count > HISTORY_LENGTH)
        {
            feedbackHistory.RemoveAt(0);
        }
    }

    /// <summary>
    /// 获取平滑后的反馈 (移动平均)
    /// </summary>
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

        // 取最近的路径修正偏移
        avg.pathCorrectionOffset = feedbackHistory[feedbackHistory.Count - 1].pathCorrectionOffset;
        avg.safeDistanceMultiplier = feedbackHistory[feedbackHistory.Count - 1].safeDistanceMultiplier;
        avg.speedMultiplier = feedbackHistory[feedbackHistory.Count - 1].speedMultiplier;
        avg.nearestDynamicObstacle = feedbackHistory[feedbackHistory.Count - 1].nearestDynamicObstacle;
        avg.nearestDynamicObstacleVelocity = feedbackHistory[feedbackHistory.Count - 1].nearestDynamicObstacleVelocity;
        avg.obstacleCount = feedbackHistory[feedbackHistory.Count - 1].obstacleCount;

        return avg;
    }

    /// <summary>
    /// 获取当前 COLREGs 合规评分
    /// </summary>
    public float GetCurrentCOLREGsScore()
    {
        return currentFeedback.colregsComplianceScore;
    }

    /// <summary>
    /// 获取当前碰撞风险等级
    /// </summary>
    public float GetCurrentRiskLevel()
    {
        return currentFeedback.collisionRiskLevel;
    }
}