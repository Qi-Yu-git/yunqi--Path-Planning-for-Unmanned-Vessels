using UnityEngine;

[RequireComponent(typeof(Rigidbody))]
public class BoatController : MonoBehaviour
{
    public ImprovedAStar pathfinder;       // A*寻路组件引用（拖拽AStarManager对象）
    public GridManager gridManager;        // 栅格管理器引用（拖拽水域平面对象）
    public float moveSpeed = 5f;           // 移动速度（米/秒）
    public float rotationSpeed = 15f;      // 旋转速度（数值越大越灵敏）
    public float waypointDistance = 1f;    // 到达路径点的判定距离
    public float avoidDistance = 2f;       // 避障检测距离

    private Rigidbody rb;
    private int currentWaypointIndex = 0;
    private Vector3 targetPosition;

    // 初始化方法（正确放在类内部）
    void Start()
    {
        rb = GetComponent<Rigidbody>();
        rb.constraints = RigidbodyConstraints.FreezeRotation;
        rb.useGravity = false; // 取消重力，适配水面场景

        // 延迟0.1秒执行路径初始化（给ImprovedAStar留足路径计算时间）
        Invoke("InitTargetPosition", 0.1f);
    }

    // 单独写一个路径初始化方法
    private void InitTargetPosition()
    {
        if (pathfinder.path != null && pathfinder.path.Count > 0)
        {
            targetPosition = gridManager.栅格转世界(pathfinder.path[0]);
            Debug.Log("BoatController成功读取路径，初始目标点：" + targetPosition);

            Vector3 targetDirection = (targetPosition - transform.position).normalized;
            targetDirection.y = 0;
            if (targetDirection != Vector3.zero)
            {
                // 直接设置旋转，无任何过渡，启动时立即朝向目标
                transform.rotation = Quaternion.LookRotation(targetDirection);
                // 若使用刚体控制，同时更新刚体旋转（避免延迟）
                rb.MoveRotation(Quaternion.LookRotation(targetDirection));
            }
        }
        else
        {
            Debug.LogError("BoatController延迟读取路径仍为空，请检查ImprovedAStar的path是否赋值");
        }
    }

    void Update()
    {
        // 如果路径存在且有路径点
        if (pathfinder.path != null && pathfinder.path.Count > 0)
        {
            // 检查是否到达当前路径点
            if (Vector3.Distance(transform.position, targetPosition) < waypointDistance)
            {
                currentWaypointIndex++;
                // 检查是否到达终点
                if (currentWaypointIndex >= pathfinder.path.Count)
                {
                    StopMovement();
                    return;
                }
                // 更新目标点
                targetPosition = gridManager.栅格转世界(pathfinder.path[currentWaypointIndex]);
            }
        }
    }

    void FixedUpdate()
    {
        // 如果有目标点则移动
        if (pathfinder.path != null && pathfinder.path.Count > 0 &&
            currentWaypointIndex < pathfinder.path.Count)
        {
            // 检查前方是否有障碍物
            if (CheckForObstacles())
            {
                // 重新计算路径
                Vector2Int currentGrid = gridManager.世界转栅格(transform.position);
                Vector2Int targetGrid = gridManager.世界转栅格(pathfinder.targetPos.position);
                var newPath = pathfinder.FindPath(currentGrid, targetGrid);

                if (newPath != null)
                {
                    pathfinder.path = newPath;
                    currentWaypointIndex = 0;
                    targetPosition = gridManager.栅格转世界(pathfinder.path[0]);
                }
                else
                {
                    // 如果无法找到新路径，暂时停止
                    StopMovement();
                    return;
                }
            }

            // 移动到目标点
            MoveTowardsTarget();
        }
    }

    // 移动到目标点（平滑转向+移动）
    private void MoveTowardsTarget()
    {
        // 计算目标方向（忽略Y轴，保持水平）
        Vector3 targetDirection = (targetPosition - transform.position).normalized;
        targetDirection.y = 0;

        // 旋转朝向目标（平滑插值）
        Quaternion targetRotation = Quaternion.LookRotation(targetDirection);
        transform.rotation = Quaternion.Lerp(transform.rotation, targetRotation, rotationSpeed * Time.fixedDeltaTime);

        // 向前移动（用刚体避免穿模）
        rb.velocity = transform.forward * moveSpeed;
    }

    // 检查前方是否有障碍物（射线检测）
    private bool CheckForObstacles()
    {
        RaycastHit hit;
        // 从船的位置向前发射射线
        if (Physics.Raycast(transform.position, transform.forward, out hit, avoidDistance))
        {
            // 忽略水域平面和自身
            if (hit.collider.gameObject != gridManager.水域平面.gameObject &&
                hit.collider.gameObject != gameObject)
            {
                return true; // 检测到障碍物
            }
        }
        return false;
    }

    // 停止移动
    private void StopMovement()
    {
        rb.velocity = Vector3.zero;
    }

    // 绘制调试线（方便查看）
    void OnDrawGizmosSelected()
    {
        // 绘制到当前目标点的线
        if (pathfinder.path != null && pathfinder.path.Count > 0 &&
            currentWaypointIndex < pathfinder.path.Count)
        {
            Gizmos.color = Color.blue;
            Gizmos.DrawLine(transform.position, targetPosition);
        }

        // 绘制避障检测射线
        Gizmos.color = Color.yellow;
        Gizmos.DrawRay(transform.position, transform.forward * avoidDistance);
    }
}