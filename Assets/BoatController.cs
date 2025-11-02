using UnityEngine;
using System.Collections.Generic;

public class BoatController : MonoBehaviour
{
    public ImprovedAStar pathfinder; // 关联A*寻路脚本
    public GridManager gridManager; // 关联栅格管理器
    private List<Vector2Int> gridPath; // 栅格路径点列表
    private List<Vector3> worldPath; // 转换后的世界坐标路径
    private int currentWaypointIndex = 0; // 当前目标点索引

    [Header("移动参数")]
    public float moveSpeed = 3f; // 移动速度
    public float rotationSpeed = 2f; // 旋转速度（降低避免转圈）
    public float waypointDistance = 0.6f; // 到达路径点的判定距离

    private Rigidbody rb;

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
            Debug.LogError("请关联pathfinder和gridManager！");
            return;
        }

        // 延迟0.1秒读取路径（等待A*生成），失败则重试
        InvokeRepeating("TryLoadPath", 0.1f, 0.1f);
    }

    // 重试读取路径的方法
    private void TryLoadPath()
    {
        if (pathfinder.path != null && pathfinder.path.Count > 0)
        {
            gridPath = pathfinder.path;
            // 转换为世界坐标
            worldPath = new List<Vector3>();
            foreach (var gridPos in gridPath)
            {
                Vector3 worldPos = gridManager.栅格转世界(gridPos);
                worldPath.Add(worldPos);
            }
            Debug.Log($"BoatController成功读取路径，共{worldPath.Count}个世界坐标点，初始目标点：{worldPath[0]}");
            CancelInvoke("TryLoadPath"); // 成功后取消重试
        }
        else
        {
            Debug.LogWarning("路径未生成，重试中...");
        }
    }

    // 保留原FixedUpdate和MoveTowardsTarget方法不变

    void FixedUpdate()
    {
        if (worldPath == null || worldPath.Count == 0)
            return;

        // 1. 检查是否到达最后一个点，到达则停止
        if (currentWaypointIndex >= worldPath.Count)
        {
            rb.velocity = Vector3.zero; // 强制停止
            Debug.Log("已到达终点，停止移动");
            return;
        }

        // 2. 获取当前目标点（仅XZ平面）
        Vector3 target = worldPath[currentWaypointIndex];
        Vector3 targetXZ = new Vector3(target.x, transform.position.y, target.z);
        Vector3 currentXZ = new Vector3(transform.position.x, transform.position.y, transform.position.z);

        // 3. 计算距离，判断是否到达当前目标点
        float distance = Vector3.Distance(currentXZ, targetXZ);
        bool isLastWaypoint = (currentWaypointIndex == worldPath.Count - 1);
        float stopDistance = isLastWaypoint ? 0.3f : 0.6f; // 终点用更小的判定距离

        if (distance <= stopDistance)
        {
            currentWaypointIndex++;
            if (currentWaypointIndex < worldPath.Count)
            {
                Debug.Log($"更新目标点{currentWaypointIndex}：{worldPath[currentWaypointIndex]}");
            }
            return;
        }

        // 4. 旋转朝向目标（平滑旋转）
        Quaternion targetRotation = Quaternion.LookRotation(targetXZ - currentXZ);
        targetRotation = Quaternion.Euler(0, targetRotation.eulerAngles.y, 0); // 仅绕Y轴旋转
        transform.rotation = Quaternion.Lerp(transform.rotation, targetRotation, rotationSpeed * Time.fixedDeltaTime);

        // 5. 移动（到达终点前减速）
        float currentSpeed = isLastWaypoint ? moveSpeed * 0.5f : moveSpeed; // 终点前减速50%
        Vector3 moveDir = transform.forward * currentSpeed;
        rb.velocity = new Vector3(moveDir.x, rb.velocity.y, moveDir.z);
    }

    private void MoveTowardsTarget(Vector3 target)
    {
        // 忽略Y轴，仅在XZ平面移动
        Vector3 targetXZ = new Vector3(target.x, transform.position.y, target.z);
        Vector3 currentXZ = new Vector3(transform.position.x, transform.position.y, transform.position.z);

        // 计算距离，判断是否到达目标点
        float distance = Vector3.Distance(currentXZ, targetXZ);
        if (distance <= waypointDistance)
        {
            currentWaypointIndex++; // 切换到下一个点
            if (currentWaypointIndex < worldPath.Count)
            {
                Debug.Log($"更新目标点{currentWaypointIndex}：{worldPath[currentWaypointIndex]}");
            }
            return;
        }

        // 平滑旋转（仅绕Y轴）
        Quaternion targetRotation = Quaternion.LookRotation(targetXZ - currentXZ);
        targetRotation = Quaternion.Euler(0, targetRotation.eulerAngles.y, 0); // 锁定X、Z轴旋转
        transform.rotation = Quaternion.Lerp(transform.rotation, targetRotation, rotationSpeed * Time.fixedDeltaTime);

        // 移动（通过刚体控制）
        Vector3 moveDir = transform.forward * moveSpeed;
        rb.velocity = new Vector3(moveDir.x, rb.velocity.y, moveDir.z); // 保留Y轴速度（如悬浮）
    }
}
