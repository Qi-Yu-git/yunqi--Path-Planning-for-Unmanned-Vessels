using System;
using System.Collections.Generic;
using UnityEngine;

public class COLREGsEvaluator : MonoBehaviour
{
    [Header("阈值参数")]
    public float safeDistanceThreshold = 15f;
    public float dangerDistanceThreshold = 5f;
    public float collisionAvoidanceWeight = 1.0f;

    /// <summary>
    /// 评估当前会遇态势，返回合规分数（0~1）和态势类型
    /// （已修复：取所有障碍物中的最差分，避免平均分虚高）
    /// </summary>
    public (float score, COLREGsState state) Evaluate(
        Vector3 usvPos, float usvHeading, float usvSpeed,
        List<(Vector3 pos, Vector3 vel)> obstacles)
    {
        if (obstacles == null || obstacles.Count == 0)
            return (1f, COLREGsState.Normal);

        float globalMinScore = 1f;
        COLREGsState worstState = COLREGsState.Normal;
        Vector3 usvVel = GetHeadingVector(usvHeading) * Mathf.Max(usvSpeed, 0.01f);

        foreach (var obs in obstacles)
        {
            float distance = Vector3.Distance(usvPos, obs.pos);
            if (distance > safeDistanceThreshold) continue;

            bool isStatic = obs.vel.magnitude < 0.05f;

            Vector3 relPos = obs.pos - usvPos;
            Vector3 relVel = obs.vel - usvVel;

            // ---- DCPA / TCPA ----
            float relSpeedSq = relVel.sqrMagnitude;
            float tcpa = 999f;
            float dcpa = distance;

            if (relSpeedSq > 0.0001f)
            {
                float t = -Vector3.Dot(relPos, relVel) / relSpeedSq;
                if (t > 0f)
                {
                    tcpa = t;
                    Vector3 closest = relPos + relVel * t;
                    dcpa = closest.magnitude;
                }
            }

            // ---- 会遇态势 ----
            float obsHeading = obs.vel.magnitude > 0.1f
                ? Mathf.Atan2(obs.vel.x, obs.vel.z) * Mathf.Rad2Deg
                : usvHeading;

            float headingDiff = Mathf.DeltaAngle(usvHeading, obsHeading);
            float absHeadingDiff = Mathf.Abs(headingDiff);

            float bearing = Mathf.Atan2(relPos.x, relPos.z) * Mathf.Rad2Deg;
            float relBearing = Mathf.DeltaAngle(usvHeading, bearing);
            float absRelBearing = Mathf.Abs(relBearing);

            COLREGsState state = COLREGsState.Normal;

            if (absHeadingDiff > 165f && absHeadingDiff < 195f && absRelBearing < 15f)
                state = COLREGsState.HeadOn;
            else if (absHeadingDiff < 15f && absRelBearing > 165f)
                state = COLREGsState.Overtaking;
            else if (absHeadingDiff > 15f && absHeadingDiff < 165f)
            {
                if (relBearing > 0f && relBearing < 112.5f)
                    state = COLREGsState.CrossingFromStarboard;
                else if (relBearing < 0f && relBearing > -112.5f)
                    state = COLREGsState.CrossingFromPort;
            }

            // ---- 连续 DCPA 评分 ----
            float score;
            if (dcpa < dangerDistanceThreshold)
                score = 0f;
            else if (dcpa < safeDistanceThreshold)
                score = (dcpa - dangerDistanceThreshold) / (safeDistanceThreshold - dangerDistanceThreshold);
            else
                score = 1f;

            // TCPA 紧急度修正
            if (tcpa < 15f && tcpa > 0f)
            {
                float urgency = 1f - (tcpa / 15f);
                score = Mathf.Lerp(score, Mathf.Sqrt(score), urgency);
            }

            // ---- 规则修正 ----
            if (tcpa < 10f && tcpa > 0f && dcpa < safeDistanceThreshold * 0.6f)
            {
                switch (state)
                {
                    case COLREGsState.HeadOn:
                        float crossY = Vector3.Cross(usvVel.normalized, relPos.normalized).y;
                        if (crossY < -0.15f) score = Mathf.Max(score, 0.9f);
                        else if (crossY > 0.15f) score *= 0.60f;   // ← 改为 0.60
                        else score *= 0.80f;                        // ← 改为 0.80
                        break;

                    case COLREGsState.CrossingFromStarboard:
                        float approachRate = Vector3.Dot(usvVel, relPos.normalized);
                        if (usvSpeed < 0.4f || approachRate < 0f) score = Mathf.Max(score, 0.85f);
                        else score *= 0.70f;                       // ← 改为 0.70
                        break;

                    case COLREGsState.CrossingFromPort:
                        if (Mathf.Abs(Mathf.DeltaAngle(usvHeading, bearing)) < 12f) score = Mathf.Max(score, 0.9f);
                        else score *= 0.75f;                       // ← 改为 0.75
                        break;

                    case COLREGsState.Overtaking:
                        bool isOvertaking = usvSpeed > obs.vel.magnitude * 0.9f && Vector3.Dot(usvVel, relPos.normalized) > 0f;
                        if (isOvertaking) score *= 0.70f;         // ← 改为 0.70
                        else score = Mathf.Max(score, 0.8f);
                        break;
                }
            }

            // 静态障碍物权重降低
            if (isStatic) score = Mathf.Lerp(score, 1.0f, 0.3f);

            // 记录最差值
            if (score < globalMinScore)
            {
                globalMinScore = score;
                worstState = state;
            }
        }

        return (Mathf.Clamp01(globalMinScore), worstState);
    }

    private Vector3 GetHeadingVector(float headingDeg)
    {
        float rad = headingDeg * Mathf.Deg2Rad;
        return new Vector3(Mathf.Sin(rad), 0f, Mathf.Cos(rad));
    }

    /// <summary>
    /// 用于奖励计算的便捷方法（直接返回分数转换后的奖励值）
    /// </summary>
    public float GetReward(Vector3 usvPos, float usvHeading, float usvSpeed,
                           List<(Vector3 pos, Vector3 vel)> obstacles)
    {
        var (score, _) = Evaluate(usvPos, usvHeading, usvSpeed, obstacles);
        // 将分数映射到 [-0.5, 0.2] 区间，与权重配合
        return Mathf.Lerp(-0.3f, 0.3f, score);
    }
}

public enum COLREGsState
{
    Normal,
    HeadOn,
    CrossingFromStarboard,
    CrossingFromPort,
    Overtaking
}