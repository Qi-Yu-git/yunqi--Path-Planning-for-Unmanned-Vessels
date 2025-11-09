using UnityEngine;
using System.Collections.Generic;

public class BoatController : MonoBehaviour
{
    // 公开参数（在Inspector赋值，已优化默认值）
    public ImprovedAStar pathfinder;
    public GridManager gridManager;
    [Tooltip("直线运动速度（建议1.5，原3）")]
    public float moveSpeed = 1.5f;
    [Tooltip("转向速度（建议1，原2）")]
    public float rotationSpeed = 1f;
    [Tooltip("路径点切换距离（建议1，原0.6）")]
    public float waypointDistance = 1f;
    public float endPointSlowRange = 2f; // 终点前减速范围
    public float minEndSpeed = 0.5f;     // 终点前最小速度

    // 私有变量
    private List<Vector2Int> gridPath;
    private List<Vector3> worldPath;
    private int currentWaypointIndex = 0;
    private Rigidbody rb;
    private bool isReachedEnd = false;
    private float currentSpeed = 0f; // 用于平滑速度过渡

    // 初始化
    void Start()
    {
        rb = GetComponent<Rigidbody>();
        if (rb == null)
        {
            Debug.LogError("无人船缺少Rigidbody组件！");
            return;
        }
        // 初始化刚体阻力（新增：增加阻尼，减少滑动）
        rb.drag = 0.5f;
        rb.angularDrag = 0.8f;

        if (pathfinder == null || gridManager == null)
        {
            Debug.LogError("请在Inspector中关联pathfinder和gridManager！");
            return;
        }
        TryLoadPath(); // 尝试加载路径
    }

    // 碰撞处理（增强版）
    private void OnCollisionEnter(Collision collision)
    {
        if (collision.collider.CompareTag("USV") || collision.collider.CompareTag("Obstacle"))
        {
            Debug.LogError("发生碰撞！暂停并重新规划路径");
            rb.velocity = Vector3.zero;
            rb.angularVelocity = Vector3.zero;
            isReachedEnd = true; // 临时停止运动
            Invoke(nameof(ResumeMovement), 1f); // 1秒后恢复
            if (pathfinder != null)
            {
                Invoke(nameof(pathfinder.CalculatePathAfterDelay), 1f); // 延迟重规划
            }
        }
    }

    // 恢复运动（退回到上一个路径点，避免持续碰撞）
    private void ResumeMovement()
    {
        currentWaypointIndex = Mathf.Max(0, currentWaypointIndex - 1);
        isReachedEnd = false;
    }

    // 尝试加载路径（失败则重试）
    private void TryLoadPath()
    {
        if (pathfinder.path != null && pathfinder.path.Count > 0)
        {
            gridPath = pathfinder.path;
            worldPath = new List<Vector3>();
            foreach (var gridPos in gridPath)
            {
                Vector3 worldPos = gridManager.栅格转世界(gridPos);
                worldPos.y = 0.05f; // 强制路径点Y轴与水域一致
                worldPath.Add(worldPos);
            }
            Debug.Log($"成功读取路径，共{worldPath.Count}个点");
            isReachedEnd = false;
            currentWaypointIndex = 0; // 重置路径点索引
        }
        else
        {
            Debug.LogWarning("路径未生成，1秒后重试...");
            Invoke(nameof(TryLoadPath), 1f);
        }
    }

    // 物理更新（优化后移动逻辑）
    void FixedUpdate()
    {
        // 固定Y轴高度，避免上下浮动
        transform.position = new Vector3(transform.position.x, 0.4f, transform.position.z);

        if (isReachedEnd || worldPath == null || worldPath.Count == 0)
            return;

        // 到达最后一个路径点
        if (currentWaypointIndex >= worldPath.Count)
        {
            rb.velocity = Vector3.zero;
            isReachedEnd = true;
            Debug.Log("已到达终点，停止移动");
            return;
        }

        // 移动到当前路径点
        Vector3 target = worldPath[currentWaypointIndex];
        Vector3 targetXZ = new Vector3(target.x, 0.4f, target.z);
        Vector3 currentXZ = new Vector3(transform.position.x, 0.4f, transform.position.z);
        float distance = Vector3.Distance(currentXZ, targetXZ);
        bool isLastWaypoint = (currentWaypointIndex == worldPath.Count - 1);
        float stopDistance = isLastWaypoint ? 0.5f : waypointDistance; // 用waypointDistance作为判断阈值

        // 到达当前路径点，切换到下一个（提前预判下一个点方向）
        if (distance <= stopDistance)
        {
            currentWaypointIndex++;
            // 提前转向下一个点，减少转向延迟
            if (currentWaypointIndex < worldPath.Count)
            {
                Vector3 nextTarget = worldPath[currentWaypointIndex];
                Vector3 nextTargetXZ = new Vector3(nextTarget.x, 0.4f, nextTarget.z);
                Quaternion nextRotation = Quaternion.LookRotation(nextTargetXZ - currentXZ);
                transform.rotation = Quaternion.Euler(0, nextRotation.eulerAngles.y, 0);
            }
            return;
        }

        // 平滑转向目标（降低旋转速度，减少抖动）
        Quaternion targetRotation = Quaternion.LookRotation(targetXZ - currentXZ);
        targetRotation = Quaternion.Euler(0, targetRotation.eulerAngles.y, 0);
        transform.rotation = Quaternion.Lerp(transform.rotation, targetRotation, rotationSpeed * Time.fixedDeltaTime);

        // 计算目标速度（终点前减速，增加平滑过渡）
        float targetSpeed = moveSpeed;
        if (isLastWaypoint)
        {
            float distanceToEnd = Vector3.Distance(currentXZ, worldPath[worldPath.Count - 1]);
            if (distanceToEnd <= endPointSlowRange)
            {
                float speedRatio = distanceToEnd / endPointSlowRange;
                targetSpeed = Mathf.Lerp(minEndSpeed, moveSpeed * 0.5f, speedRatio);
            }
            else
            {
                targetSpeed = moveSpeed * 0.5f;
            }
        }

        // 速度平滑过渡（避免突然加速/减速）
        currentSpeed = Mathf.Lerp(currentSpeed, targetSpeed, Time.fixedDeltaTime * 2f);
        Vector3 moveDir = transform.forward * currentSpeed;
        rb.velocity = new Vector3(moveDir.x, rb.velocity.y, moveDir.z);
    }
}