using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators; // 这行是关键，ActionBuffers 定义在这里

public class USV_RLAgent : Agent
{
    public USV_AutoMovement manualControl; // 关联你的手动控制脚本
    public Transform targetTransform; // 目标点（场景里的“目标点”物体）

    public override void Initialize()
    {
        base.Initialize();
        Debug.Log("Agent初始化完成，等待Python连接...");
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        // 观测无人船位置（X,Z）
        sensor.AddObservation(transform.position.x);
        sensor.AddObservation(transform.position.z);
        // 观测目标点位置（X,Z）
        if (targetTransform != null)
        {
            sensor.AddObservation(targetTransform.position.x);
            sensor.AddObservation(targetTransform.position.z);
        }
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        float forward = actions.ContinuousActions[0];
        float right = actions.ContinuousActions[1];

        float moveSpeed = manualControl.usvMoveSpeed;
        Vector3 movement = new Vector3(right * moveSpeed, 0, forward * moveSpeed) * Time.deltaTime;
        transform.Translate(movement);

        // 奖励逻辑
        if (targetTransform != null)
        {
            float distance = Vector3.Distance(transform.position, targetTransform.position);
            AddReward(1f / distance);
            if (distance < 1f)
            {
                SetReward(10f);
                EndEpisode();
            }
        }
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var continuousActions = actionsOut.ContinuousActions;
        continuousActions[0] = Input.GetKey(KeyCode.W) ? 1f : Input.GetKey(KeyCode.S) ? -1f : 0f;
        continuousActions[1] = Input.GetKey(KeyCode.D) ? 1f : Input.GetKey(KeyCode.A) ? -1f : 0f;
    }

    public override void OnEpisodeBegin()
    {
        transform.position = new Vector3(Random.Range(-10, 10), manualControl.waterHeightOffset, Random.Range(-10, 10));
    }
}