using UnityEngine;

public class MoveForwardBoat : MonoBehaviour
{
    public float moveSpeed = 5f; // 前进速度
    public float heightOffset = 0.5f; // 离水面高度

    void Update()
    {
        // 沿X轴正方向（前方）移动，固定Y轴高度
        Vector3 currentPos = transform.position;
        currentPos.x += moveSpeed * Time.deltaTime;
        transform.position = new Vector3(currentPos.x, heightOffset, currentPos.z);
    }
}