using UnityEngine;
using System.Collections.Generic;

public class BoatController : MonoBehaviour
{
    // 公开参数（在Inspector赋值）
    public ImprovedAStar pathfinder;
    public GridManager gridManager;
    public float moveSpeed = 3f;
    public float rotationSpeed = 2f;
    public float waypointDistance = 0.6f;
    public float endPointSlowRange = 2f; // 终点前减速范围
    public float minEndSpeed = 0.5f;     // 终点前最小速度

    // 私有变量
    private List<Vector2Int> gridPath;
    private List<Vector3> worldPath;
    private int currentWaypointIndex = 0;
    private Rigidbody rb;
    private bool isReachedEnd = false;

    // 初始化
    void Start()
    {
        rb = GetComponent<Rigidbody>();
        if (rb == null)
        {
            Debug.LogError("无人船缺少Rigidbody组件！");
            return;
        }
        if (pathfinder == null || gridManager == null)
        {
            Debug.LogError("请在Inspector中关联pathfinder和gridManager！");
            return;
        }
        TryLoadPath(); // 尝试加载路径
    }

    // 在BoatController.cs中添加OnCollisionEnter方法
    private void OnCollisionEnter(Collision collision)
    {
        // 检测到其他无人船（假设标签为"USV"）
        if (collision.collider.CompareTag("USV"))
        {
            Debug.LogError("发生碰撞！");
            // 碰撞后紧急减速
            rb.velocity = Vector3.zero;
            // 尝试重新规划路径
            if (pathfinder != null)
            {
                Invoke(nameof(pathfinder.CalculatePathAfterDelay), 0.5f);
            }
        }
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
                Debug.Log($"路径点{worldPath.Count}：{worldPos}"); // 新增日志，验证Y轴
            }
            Debug.Log($"成功读取路径，共{worldPath.Count}个点");
            isReachedEnd = false;
        }
        else
        {
            Debug.LogWarning("路径未生成，1秒后重试...");
            Invoke(nameof(TryLoadPath), 1f);
        }
    }

    // 物理更新（移动逻辑）
    void FixedUpdate()
    {
        // 固定Y轴高度为0.4f
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
        Vector3 targetXZ = new Vector3(target.x, 0.4f, target.z); // 目标点也使用相同Y轴高度
        Vector3 currentXZ = new Vector3(transform.position.x, 0.4f, transform.position.z);
        float distance = Vector3.Distance(currentXZ, targetXZ);
        bool isLastWaypoint = (currentWaypointIndex == worldPath.Count - 1);
        float stopDistance = isLastWaypoint ? 0.3f : 0.6f;

        // 到达当前路径点，切换到下一个
        if (distance <= stopDistance)
        {
            currentWaypointIndex++;
            return;
        }

        // 旋转朝向目标
        Quaternion targetRotation = Quaternion.LookRotation(targetXZ - currentXZ);
        targetRotation = Quaternion.Euler(0, targetRotation.eulerAngles.y, 0);
        transform.rotation = Quaternion.Lerp(transform.rotation, targetRotation, rotationSpeed * Time.fixedDeltaTime);

        // 移动速度（终点前减速）
        float currentSpeed = moveSpeed;
        if (isLastWaypoint)
        {
            float distanceToEnd = Vector3.Distance(currentXZ, worldPath[worldPath.Count - 1]);
            if (distanceToEnd <= endPointSlowRange)
            {
                float speedRatio = distanceToEnd / endPointSlowRange;
                currentSpeed = Mathf.Lerp(minEndSpeed, moveSpeed * 0.5f, speedRatio);
            }
            else
            {
                currentSpeed = moveSpeed * 0.5f;
            }
        }

        // 应用移动
        Vector3 moveDir = transform.forward * currentSpeed;
        rb.velocity = new Vector3(moveDir.x, rb.velocity.y, moveDir.z);
    }
}

