using UnityEngine;
public class FollowTarget : MonoBehaviour
{
    public Transform targetToFollow; // 赋值为场景中的targetPos
    [Header("三轴偏移设置")]
    public float followXOffset = 0f; // X轴偏移量
    public float followYOffset = 0.7f; // Y轴偏移量（默认沿用原配置）
    public float followZOffset = 0f; // Z轴偏移量

    void Update()
    {
        if (targetToFollow != null)
        {
            // 同步目标点位置，叠加xyz三轴独立偏移
            Vector3 targetPos = targetToFollow.position;
            transform.position = new Vector3(
                targetPos.x + followXOffset,
                targetPos.y + followYOffset,
                targetPos.z + followZOffset
            );
        }
        else
        {
            Debug.LogError("FollowTarget：请赋值需要跟随的targetPos！");
        }
    }
}