using UnityEngine;

/// <summary>
/// 无人船自主移动控制（重命名版）
/// </summary>
public class USV_AutoMovement : MonoBehaviour
{
    [Header("移动基础设置")]
    public float usvMoveSpeed = 5f; // 无人船移动速度
    public float waterHeightOffset = 0.5f; // 离水面高度偏移

    [Header("按键控制配置")]
    public KeyCode forwardControlKey = KeyCode.W; // 前进控制键
    public KeyCode backwardControlKey = KeyCode.S; // 后退控制键
    public KeyCode leftControlKey = KeyCode.A; // 左移控制键
    public KeyCode rightControlKey = KeyCode.D; // 右移控制键

    void Update()
    {
        Vector3 currentPosition = transform.position;
        float xAxisMovement = 0f;
        float zAxisMovement = 0f;

        // 检测各方向按键输入
        if (Input.GetKey(forwardControlKey))
        {
            zAxisMovement += usvMoveSpeed * Time.deltaTime;
        }
        if (Input.GetKey(backwardControlKey))
        {
            zAxisMovement -= usvMoveSpeed * Time.deltaTime;
        }
        if (Input.GetKey(leftControlKey))
        {
            xAxisMovement -= usvMoveSpeed * Time.deltaTime;
        }
        if (Input.GetKey(rightControlKey))
        {
            xAxisMovement += usvMoveSpeed * Time.deltaTime;
        }

        // 更新位置并固定水面高度
        transform.position = new Vector3(
            currentPosition.x + xAxisMovement,
            waterHeightOffset,
            currentPosition.z + zAxisMovement
        );
    }
}