using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;

// �����޸ģ�����������ΪUSV_TestAgent������Ŀǰ׺ͳһ����������������ͻ
// �ű��ļ�ͬ������Ϊ��USV_TestAgent.cs
public class USV_TestAgent : Agent
{
    // ��ѡ�������ò������ɸ�����Ҫ����
    public Rigidbody testRb;
    public Transform testTarget;

    // ML-Agents 2.0.1 ����д���������ʼ��
    public override void Initialize()
    {
        // ��ʼ���߼�����ȡ�����������������
        if (testRb == null)
        {
            testRb = GetComponent<Rigidbody>();
            if (testRb != null)
            {
                testRb.useGravity = false;
                testRb.angularDamping = 2f;
            }
        }
        Debug.Log("USV�����������ʼ�����");
    }

    // ML-Agents 2.0.1 ����д���غϿ�ʼʱ����״̬
    public override void OnEpisodeBegin()
    {
        if (testRb != null)
        {
            // ��������״̬
            testRb.linearVelocity = Vector3.zero;
            testRb.angularVelocity = Vector3.zero;
            // ����λ�ã������ã��ɸ��ݳ���������
            transform.position = new Vector3(0, 0, 0);
            transform.rotation = Quaternion.identity;
        }
    }

    // ML-Agents 2.0.1 ����д���ռ��۲�ֵ������Pythonѵ���ˣ�
    public override void CollectObservations(VectorSensor sensor)
    {
        // ʾ�����ռ���Ŀ��ľ��루��һ����0-1����1���۲�ֵ
        if (testTarget != null)
        {
            float dist = Vector3.Distance(transform.position, testTarget.position);
            sensor.AddObservation(Mathf.Clamp01(dist / 20f));
        }
        else
        {
            sensor.AddObservation(0f);
        }

        // ʾ�����ռ�ǰ���ٶȣ���һ����-1-1����1���۲�ֵ
        if (testRb != null)
        {
            float forwardSpeed = Vector3.Dot(transform.forward, testRb.linearVelocity);
            sensor.AddObservation(Mathf.Clamp(forwardSpeed / 5f, -1f, 1f));
        }
        else
        {
            sensor.AddObservation(0f);
        }
        // ע�⣺�۲�ֵ����Ҫ��YAML���õ�vector_observation_sizeһ��
    }

    // ML-Agents 2.0.1 ����д������Python�˵Ķ�����ִ��
    public override void OnActionReceived(ActionBuffers actions)
    {
        // ʾ������ɢ������3��������ǰ������ת����ת��
        int moveAction = actions.DiscreteActions[0];
        float speed = 3f;

        switch (moveAction)
        {
            case 0: // ǰ��
                testRb?.AddForce(transform.forward * speed, ForceMode.VelocityChange);
                break;
            case 1: // ��ת
                transform.Rotate(Vector3.up, -20f * Time.fixedDeltaTime);
                break;
            case 2: // ��ת
                transform.Rotate(Vector3.up, 20f * Time.fixedDeltaTime);
                break;
        }

        // ʾ�����򵥽����߼��������ã�
        if (testTarget != null && Vector3.Distance(transform.position, testTarget.position) < 2f)
        {
            AddReward(10f); // ����Ŀ��ӽ���
            EndEpisode();   // ������ǰ�غ�
        }
    }

    // ��ѡ��ML-Agents 2.0.1 ��д���ֶ����ƣ����Գ����ã�����ѵ��ʱ������
    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var discreteActions = actionsOut.DiscreteActions;
        // ����W=ǰ����A=��ת��D=��ת
        if (Input.GetKey(KeyCode.W)) discreteActions[0] = 0;
        else if (Input.GetKey(KeyCode.A)) discreteActions[0] = 1;
        else if (Input.GetKey(KeyCode.D)) discreteActions[0] = 2;
        else discreteActions[0] = -1; // �޲���
    }
}