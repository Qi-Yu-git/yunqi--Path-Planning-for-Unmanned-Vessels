using UnityEngine;

public class XZAxisAutoMove : MonoBehaviour
{
    [Header("XZ轴自动移动参数")]
    public float speed = 3f; // 移动速度
    [Tooltip("沿X轴正方向移动")]
    public bool xPositive = false;
    [Tooltip("沿X轴负方向移动")]
    public bool xNegative = false;
    [Tooltip("沿Z轴正方向移动")]
    public bool zPositive = false;
    [Tooltip("沿Z轴负方向移动")]
    public bool zNegative = false;
    public float yFixed = 0.5f; // Y轴固定高度

    private void Update()
    {
        // 计算X轴移动分量
        float x = 0f;
        if (xPositive) x += 1f;
        if (xNegative) x -= 1f;

        // 计算Z轴移动分量
        float z = 0f;
        if (zPositive) z += 1f;
        if (zNegative) z -= 1f;

        // 组合方向并归一化
        Vector3 dir = new Vector3(x, 0, z);
        if (dir.magnitude > 0)
            dir = dir.normalized;

        // 应用移动（固定Y轴）
        transform.position += dir * speed * Time.deltaTime;
        transform.position = new Vector3(
            transform.position.x,
            yFixed,
            transform.position.z
        );
    }
}