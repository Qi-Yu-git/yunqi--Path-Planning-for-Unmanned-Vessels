// ============================================================
// 文件名: COLREGsEvaluator.cs
// 路径: Assets/Scripts/RL/COLREGsEvaluator.cs
// 描述: COLREGs 规则完整评分系统
// 论文: Section 3.3 - 船舶避碰规则合规奖励
// ============================================================

using UnityEngine;
using System.Collections.Generic;

/// <summary>
/// COLREGs 规则状态
/// </summary>
public enum COLREGsState
{
    Normal,           // 正常航行
    HeadOn,           // 对遇
    CrossingFromPort, // 左舷交叉
    CrossingFromStarboard, // 右舷交叉
    Overtaking,       // 追越
    StandOn,          // 直航船 (被让路)
    GiveWay           // 让路船
}

/// <summary>
/// COLREGs 评分器
/// 评估无人船是否遵守国际海上避碰规则
/// </summary>
public class COLREGsEvaluator : MonoBehaviour
{
    [Header("COLREGs 参数 (论文配置)")]
    [Tooltip("对遇角度阈值 (°)")]
    public float headOnAngleThreshold = 15f;

    [Tooltip("交叉角度范围 (°)")]
    public float crossingAngleMin = 15f;
    public float crossingAngleMax = 165f;

    [Tooltip("追越角度范围 (°)")]
    public float overtakingAngleMin = 165f;
    public float overtakingAngleMax = 195f;

    [Tooltip("安全距离阈值 (米)")]
    public float safeDistanceThreshold = 20f;

    [Tooltip("危险距离阈值 (米)")]
    public float dangerDistanceThreshold = 5f;

    [Tooltip("相对速度阈值")]
    public float relativeSpeedThreshold = 1f;

    [Header("奖励权重")]
    [Tooltip("规则合规奖励系数")]
    public float complianceRewardWeight = 0.8f;

    [Tooltip("违规惩罚系数")]
    public float violationPenaltyWeight = 2.0f;

    [Header("调试")]
    public bool showDebugInfo = true;

    /// <summary>
    /// 评估当前状态下的 COLREGs 评分
    /// </summary>
    /// <param name="usvPos">无人船位置</param>
    /// <param name="usvHeading">无人船航向 (度)</param>
    /// <param name="usvSpeed">无人船速度</param>
    /// <param name="obstacles">障碍物列表 (位置+速度)</param>
    /// <returns>评分 (0-1) 和 当前规则状态</returns>
    public (float score, COLREGsState state) Evaluate(
        Vector3 usvPos,
        float usvHeading,
        float usvSpeed,
        List<(Vector3 pos, Vector3 vel)> obstacles)
    {
        if (obstacles == null || obstacles.Count == 0)
            return (1f, COLREGsState.Normal);

        float totalScore = 0f;
        int validCount = 0;
        COLREGsState dominantState = COLREGsState.Normal;

        foreach (var obs in obstacles)
        {
            // 只评估有效距离内的障碍物
            float distance = Vector3.Distance(usvPos, obs.pos);
            if (distance > safeDistanceThreshold) continue;

            validCount++;

            // 计算相对参数
            Vector3 relativePos = obs.pos - usvPos;
            Vector3 relativeVel = obs.vel - new Vector3(usvSpeed * Mathf.Sin(usvHeading * Mathf.Deg2Rad), 0, usvSpeed * Mathf.Cos(usvHeading * Mathf.Deg2Rad));

            float relativeAngle = Vector3.SignedAngle(
                GetHeadingVector(usvHeading),
                relativePos.normalized,
                Vector3.up
            );

            float relativeSpeed = relativeVel.magnitude;
            float bearing = Mathf.Atan2(relativePos.x, relativePos.z) * Mathf.Rad2Deg;
            float bearingDiff = Mathf.DeltaAngle(usvHeading, bearing);

            // 确定规则状态
            COLREGsState state = DetermineCOLREGsState(bearingDiff, relativeAngle, relativeSpeed);

            // 计算该障碍物的合规评分
            float score = CalculateSingleObstacleScore(state, distance, relativeSpeed, bearingDiff);

            totalScore += score;

            // 记录主要状态 (最危险的状态)
            if (state == COLREGsState.HeadOn || state == COLREGsState.CrossingFromStarboard)
                dominantState = state;
            else if (dominantState == COLREGsState.Normal)
                dominantState = state;
        }

        // 平均分 (无障碍物时返回满分)
        float finalScore = validCount > 0 ? totalScore / validCount : 1f;

        return (Mathf.Clamp01(finalScore), dominantState);
    }

    /// <summary>
    /// 确定 COLREGs 规则状态
    /// </summary>
    private COLREGsState DetermineCOLREGsState(float bearingDiff, float relativeAngle, float relativeSpeed)
    {
        float absBearing = Mathf.Abs(bearingDiff);

        // 对遇 (0°-15° 相对航向)
        if (absBearing < headOnAngleThreshold && relativeSpeed > relativeSpeedThreshold)
            return COLREGsState.HeadOn;

        // 追越 (165°-195°)
        if (absBearing > overtakingAngleMin && absBearing < overtakingAngleMax)
            return COLREGsState.Overtaking;

        // 交叉
        if (absBearing > crossingAngleMin && absBearing < crossingAngleMax)
        {
            // 右舷交叉 (目标在右舷)
            if (bearingDiff > 0)
                return COLREGsState.CrossingFromStarboard;
            else
                return COLREGsState.CrossingFromPort;
        }

        return COLREGsState.Normal;
    }

    /// <summary>
    /// 计算单个障碍物的合规评分
    /// </summary>
    private float CalculateSingleObstacleScore(COLREGsState state, float distance, float relativeSpeed, float bearingDiff)
    {
        float score = 1f;

        // 基础安全分 (距离越近分数越低)
        float distanceFactor = Mathf.Clamp01((distance - dangerDistanceThreshold) / (safeDistanceThreshold - dangerDistanceThreshold));
        score *= distanceFactor;

        // 根据规则状态调整
        switch (state)
        {
            case COLREGsState.HeadOn:
                // 对遇应向右转向 (避让)
                // 如果正在向右转 (bearingDiff 减小) 奖励
                if (bearingDiff < 0 && Mathf.Abs(bearingDiff) > 5f)
                    score *= 1.2f; // 正确避让
                else if (Mathf.Abs(bearingDiff) < 5f)
                    score *= 0.8f; // 未及时避让
                else
                    score *= 0.5f; // 错误转向
                break;

            case COLREGsState.CrossingFromStarboard:
                // 右舷交叉：应向右转向让行
                if (bearingDiff < 0)
                    score *= 1.3f; // 正确让行
                else if (bearingDiff > 5f)
                    score *= 0.5f; // 错误
                break;

            case COLREGsState.CrossingFromPort:
                // 左舷交叉：直航船 (被让路)
                if (Mathf.Abs(bearingDiff) < 10f)
                    score *= 1.1f; // 保持航向
                else
                    score *= 0.7f; // 不应大幅转向
                break;

            case COLREGsState.Overtaking:
                // 追越：被追越船让行
                if (Mathf.Abs(bearingDiff) > 15f)
                    score *= 1.2f; // 避让
                else
                    score *= 0.6f; // 未避让
                break;

            default:
                // 正常航行
                if (distance < dangerDistanceThreshold)
                    score *= 0.3f; // 危险接近
                break;
        }

        // 考虑相对速度的影响
        if (relativeSpeed > 2f)
        {
            score *= Mathf.Clamp01(2f / relativeSpeed);
        }

        return Mathf.Clamp01(score);
    }

    /// <summary>
    /// 获取航向向量
    /// </summary>
    private Vector3 GetHeadingVector(float headingDeg)
    {
        float rad = headingDeg * Mathf.Deg2Rad;
        return new Vector3(Mathf.Sin(rad), 0, Mathf.Cos(rad));
    }

    /// <summary>
    /// 获取奖励值 (用于 PPO)
    /// </summary>
    public float GetReward(Vector3 usvPos, float usvHeading, float usvSpeed, 
                           List<(Vector3 pos, Vector3 vel)> obstacles)
    {
        var (score, state) = Evaluate(usvPos, usvHeading, usvSpeed, obstacles);
        
        // 评分映射到奖励: 高分奖励，低分惩罚
        float reward = (score - 0.5f) * 2f * complianceRewardWeight;
        
        // 危险状态额外惩罚
        if (state == COLREGsState.HeadOn && score < 0.3f)
            reward -= violationPenaltyWeight;

        return reward;
    }

    void OnDrawGizmosSelected()
    {
        if (!showDebugInfo) return;

        // 绘制安全距离和危险距离
        Gizmos.color = Color.cyan;
        Gizmos.DrawWireSphere(transform.position, safeDistanceThreshold);
        Gizmos.color = Color.red;
        Gizmos.DrawWireSphere(transform.position, dangerDistanceThreshold);
    }
}