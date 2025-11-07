using UnityEngine;

public class ObstacleBoatMove : MonoBehaviour
{
    public float moveSpeed = 2f; // 前进速度

    void Update()
    {
        // 沿自身Z轴（前方）持续移动
        transform.Translate(Vector3.forward * moveSpeed * Time.deltaTime);
    }
}