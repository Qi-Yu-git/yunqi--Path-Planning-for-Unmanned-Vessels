using UnityEngine;

public class FollowTarget : MonoBehaviour
{
    public Transform targetToFollow; // 赋值为场景中的targetPos
    public float followYOffset = 2f; // 旗帜悬浮高度（避免与水域重叠）

    void Update()
    {
        if (targetToFollow != null)
        {
            // 实时同步目标点位置，仅保留Y轴偏移
            Vector3 targetPos = targetToFollow.position;
            transform.position = new Vector3(targetPos.x, followYOffset, targetPos.z);
        }
        else
        {
            Debug.LogError("FollowTarget：请赋值需要跟随的targetPos！");
        }
    }
}