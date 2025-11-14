using UnityEngine;
using System.Collections.Generic;

public class BoatController : MonoBehaviour
{
    // 公开参数（在Inspector赋值）
    public ImprovedAStar pathfinder;
    public GridManager gridManager;
    [Tooltip("直线运动速度（建议1.5）")]
    public float moveSpeed = 1.5f;
    [Tooltip("转向速度（建议1）")]
    public float rotationSpeed = 1f;
    [Tooltip("路径点切换距离（建议1）")]
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
        // 初始化刚体阻力
        rb.linearDamping = 0.5f;
        rb.angularDamping = 0.8f;

        if (pathfinder == null || gridManager == null)
        {
            Debug.LogError("请在Inspector中关联pathfinder和gridManager！");
            return;
        }
        TryLoadPath(); // 尝试加载路径
    }

    // 碰撞处理
    private void OnCollisionEnter(Collision collision)
    {
        if (collision.collider.CompareTag("USV") || collision.collider.CompareTag("Obstacle"))
        {
            Debug.LogError("发生碰撞！暂停并重新规划路径");
            rb.linearVelocity = Vector3.zero;
            rb.angularVelocity = Vector3.zero;
            isReachedEnd = true;
            Invoke(nameof(ResumeMovement), 1f);
            if (pathfinder != null)
            {
                Invoke(nameof(ReplanPath), 1f); // 延迟重规划
            }
        }
    }

    // 恢复运动
    private void ResumeMovement()
    {
        currentWaypointIndex = Mathf.Max(0, currentWaypointIndex - 1);
        isReachedEnd = false;
    }

    // 重新规划路径（适配ImprovedAStar的实际方法名）
    private void ReplanPath()
    {
        if (pathfinder != null)
        {
            // 修复：使用ImprovedAStar中实际存在的路径计算方法（根据你的代码调整）
            pathfinder.CalculatePathAfterDelay(); // 假设原方法名为CalculatePathAfterDelay
        }
    }

    // 尝试加载路径
    private void TryLoadPath()
    {
        if (pathfinder == null)
        {
            Debug.LogError("路径查找器（pathfinder）未赋值！");
            Invoke(nameof(TryLoadPath), 1f);
            return;
        }
        if (gridManager == null)
        {
            Debug.LogError("网格管理器（gridManager）未赋值！");
            Invoke(nameof(TryLoadPath), 1f);
            return;
        }

        // 校验路径数据
        if (pathfinder.path == null || pathfinder.path.Count == 0)
        {
            Debug.LogWarning("路径数据为空，1秒后重试...");
            Invoke(nameof(TryLoadPath), 1f);
            return;
        }

        // 转换路径坐标（适配GridManager的中文方法名）
        gridPath = pathfinder.path;
        worldPath = new List<Vector3>();
        foreach (var gridPos in gridPath)
        {
            if (!gridManager.IsValidGridPosition(gridPos))
            {
                Debug.LogError($"无效的栅格坐标：{gridPos}");
                worldPath = null;
                Invoke(nameof(TryLoadPath), 1f);
                return;
            }

            // 修复：使用GridManager中实际存在的方法名（中文“栅格转世界”）
            Vector3 worldPos = gridManager.栅格转世界(gridPos);
            if (float.IsNaN(worldPos.x) || float.IsNaN(worldPos.z))
            {
                Debug.LogError($"栅格转世界坐标失败：{gridPos}");
                worldPath = null;
                Invoke(nameof(TryLoadPath), 1f);
                return;
            }

            worldPos.y = 0.05f; // 强制Y轴高度
            worldPath.Add(worldPos);
        }

        if (worldPath.Count == 0)
        {
            Debug.LogError("路径转换后为空，1秒后重试...");
            Invoke(nameof(TryLoadPath), 1f);
            return;
        }

        Debug.Log($"成功读取路径，共{worldPath.Count}个点");
        isReachedEnd = false;
        currentWaypointIndex = 0;
    }

    // 物理更新
    void FixedUpdate()
    {
        // 固定Y轴高度
        transform.position = new Vector3(transform.position.x, 0.4f, transform.position.z);

        if (isReachedEnd || worldPath == null || worldPath.Count == 0)
        {
            if (worldPath == null || worldPath.Count == 0)
            {
                Debug.LogWarning("路径无效，尝试重新加载...");
                TryLoadPath();
            }
            return;
        }

        // 到达最后一个路径点
        if (currentWaypointIndex >= worldPath.Count)
        {
            rb.linearVelocity = Vector3.zero;
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
        float stopDistance = isLastWaypoint ? 0.5f : waypointDistance;

        // 到达当前路径点，切换到下一个
        if (distance <= stopDistance)
        {
            currentWaypointIndex++;
            if (currentWaypointIndex < worldPath.Count)
            {
                Vector3 nextTarget = worldPath[currentWaypointIndex];
                Vector3 nextTargetXZ = new Vector3(nextTarget.x, 0.4f, nextTarget.z);
                Quaternion nextRotation = Quaternion.LookRotation(nextTargetXZ - currentXZ);
                transform.rotation = Quaternion.Euler(0, nextRotation.eulerAngles.y, 0);
            }
            return;
        }

        // 平滑转向目标
        Quaternion targetRotation = Quaternion.LookRotation(targetXZ - currentXZ);
        targetRotation = Quaternion.Euler(0, targetRotation.eulerAngles.y, 0);
        transform.rotation = Quaternion.Lerp(transform.rotation, targetRotation, rotationSpeed * Time.fixedDeltaTime);

        // 计算目标速度（终点前减速）
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

        // 速度平滑过渡
        currentSpeed = Mathf.Lerp(currentSpeed, targetSpeed, Time.fixedDeltaTime * 2f);
        Vector3 moveDir = transform.forward * currentSpeed;
        rb.linearVelocity = new Vector3(moveDir.x, rb.linearVelocity.y, moveDir.z);
    }
}