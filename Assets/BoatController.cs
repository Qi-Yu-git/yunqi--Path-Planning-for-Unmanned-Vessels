using UnityEngine;
using System.Collections.Generic;
using System.Collections;
using System.Linq;

public class BoatController : MonoBehaviour
{
    // 公开参数（在Inspector赋值）
    public ImprovedAStar pathfinder;          // A*路径查找器引用
    public GridManager gridManager;           // 栅格管理器引用
    [Tooltip("直线运动速度（建议1.5）")]
    public float moveSpeed = 1.5f;            // 基础移动速度
    [Tooltip("转向速度（建议1）")]
    public float rotationSpeed = 1f;          // 转向平滑系数
    [Tooltip("路径点切换距离（建议1）")]
    public float waypointDistance = 1.5f;       // 路径点切换阈值
    public float endPointSlowRange = 2f;      // 终点前减速范围
    public float minEndSpeed = 0.5f;          // 终点前最小速度

    // 路径重试配置（按要求新增/调整）
    public int 最大重试次数 = 10;             // 从5调整为10，提高重试上限
    public int 路径重试次数 = 0;              // 路径加载重试计数器

    // 私有变量
    private List<Vector2Int> gridPath;        // 栅格坐标路径
    private List<Vector3> worldPath = new List<Vector3>();  // 初始化！避免null
    private int currentWaypointIndex = 0;     // 当前路径点索引
    private Rigidbody rb;                     // 刚体组件
    private bool isReachedEnd = false;        // 是否到达终点
    private bool wasPathInvalid = false;     // 跟踪路径失效状态
    public bool isPathLoaded = false;        // 标记路径是否加载完成

    // 公开属性：供外部判断路径状态（按要求保留）
    public bool IsPathLoaded => worldPath != null && worldPath.Count > 0 && isPathLoaded;

    void Start()
    {
        // 获取刚体组件
        rb = GetComponent<Rigidbody>();
        if (rb == null)
        {
            Debug.LogError("无人船缺少Rigidbody组件！请添加刚体组件。");
            return;
        }

        // 配置刚体参数
        ConfigureRigidbody();

        // 检查必要引用
        if (pathfinder == null || gridManager == null)
        {
            Debug.LogError("请在Inspector中关联pathfinder和gridManager！");
            return;
        }

        // 等待栅格初始化后加载路径
        StartCoroutine(WaitForGridInitThenLoadPath());
    }

    // 等待栅格初始化后加载路径
    private IEnumerator WaitForGridInitThenLoadPath()
    {
        while (gridManager != null && !gridManager.IsGridReady())
        {
            Debug.Log("栅格未初始化，延迟重试路径加载...");
            yield return new WaitForSeconds(0.5f);
        }
        TryLoadPath();
    }

    // 配置刚体物理参数
    private void ConfigureRigidbody()
    {
        rb.useGravity = false;                // 禁用重力（适用于水上运动）
        rb.interpolation = RigidbodyInterpolation.Interpolate; // 平滑刚体移动
        rb.collisionDetectionMode = CollisionDetectionMode.ContinuousDynamic; // 连续碰撞检测
        rb.linearDamping = 0.5f;              // 线性阻尼（阻力）
        rb.angularDamping = 2f;               // 角阻尼（旋转阻力）
    }

    // 碰撞处理（优化碰撞后路径恢复逻辑）
    private void OnCollisionEnter(Collision collision)
    {
        if (collision.collider.CompareTag("USV") || collision.collider.CompareTag("Obstacle"))
        {
            Debug.LogError($"与{collision.collider.tag}发生碰撞！重新规划路径至原目标");
            StopMovement();

            // 碰撞后位置修正（轻微后退脱离碰撞）
            if (collision.contacts.Length > 0)
            {
                Vector3 awayDir = (transform.position - collision.contacts[0].point).normalized;
                transform.position += awayDir * 0.5f;
                Debug.Log($"碰撞后位置修正：沿{awayDir}方向移动0.5米");
            }

            // 仅清空当前路径，保留目标点引用
            worldPath.Clear();
            isPathLoaded = false;
            currentWaypointIndex = 0; // 重置路径点索引

            // 通知A*重新计算路径
            if (pathfinder != null)
            {
                pathfinder.path = null; // 清除旧路径缓存
                pathfinder.CalculatePathAfterDelay(); // 立即重新计算
                Invoke(nameof(TryLoadPath), 0.5f); // 快速加载新路径
            }

            // 碰撞后立即刷新栅格障碍物状态
            if (gridManager != null)
            {
                gridManager.RefreshObstaclesOnCollision();
            }
        }
    }

    // 停止移动
    private void StopMovement()
    {
        rb.linearVelocity = Vector3.zero;
        rb.angularVelocity = Vector3.zero;
        currentSpeed = 0f;
    }

    // 恢复运动
    private void ResumeMovement()
    {
        // 回到上一个路径点重试
        currentWaypointIndex = Mathf.Max(0, currentWaypointIndex - 1);
        isReachedEnd = false;
        Debug.Log("恢复移动，继续前往路径点");
    }

    // 优化ReplanPath方法（替换不存在的RecalculatePath）
    private void ReplanPath()
    {
        if (pathfinder != null)
        {
            Debug.Log("使用原目标点重新规划路径...");
            pathfinder.path = null;  // 清除旧路径缓存
            pathfinder.CalculatePathAfterDelay();

            // 立即尝试加载路径（缩短等待时间）
            路径重试次数 = 0;
            Invoke(nameof(TryLoadPath), 0.5f);
        }
        else
        {
            Debug.LogError("路径查找器为空，尝试重新获取...");
            pathfinder = FindFirstObjectByType<ImprovedAStar>();
            Invoke(nameof(ReplanPath), 0.5f);
        }
    }

    // 公共方法：清空路径并标记路径未加载（供外部调用）
    public void ClearPathAndMarkUnloaded()
    {
        worldPath.Clear();
        isPathLoaded = false;
        currentWaypointIndex = 0;
        isReachedEnd = false;
        Debug.Log("路径已通过公共方法清空并标记为未加载");
    }

    // 优化 TryLoadPath 方法（按要求增强路径失效处理和降级逻辑）
    public void TryLoadPath()
    {
        路径重试次数++;
        if (路径重试次数 > 最大重试次数)
        {
            路径重试次数 = 0;
            Debug.LogError("路径重试次数耗尽！");
            return;
        }

        // 路径已加载且有效，无需重复加载
        if (IsPathLoaded)
        {
            Debug.Log("路径已加载，无需重复加载");
            路径重试次数 = 0;
            return;
        }

        // 栅格未初始化，延迟重试
        if (gridManager != null && !gridManager.IsGridReady())
        {
            Debug.Log($"栅格未初始化，{路径重试次数}次重试，延迟1秒...");
            Invoke(nameof(TryLoadPath), 1f);
            return;
        }

        // 检查关键引用
        if (pathfinder == null || gridManager == null)
        {
            string errorMsg = pathfinder == null ? "路径查找器未赋值！" : "网格管理器未赋值！";
            Debug.LogError(errorMsg);
            Invoke(nameof(TryLoadPath), 1f);
            return;
        }

        // 检查A*路径是否有效
        if (pathfinder.path == null || pathfinder.path.Count < 2)
        {
            Debug.LogWarning($"A* 路径无效（路径点数量：{pathfinder?.path?.Count ?? 0}），{路径重试次数}次重试...");
            pathfinder?.CalculatePathAfterDelay();
            Invoke(nameof(TryLoadPath), 1f);
            return;
        }

        // 转换路径坐标（栅格→世界）
        gridPath = pathfinder.path;
        worldPath.Clear();
        bool pathValid = true;
        foreach (var gridPos in gridPath)
        {
            if (!gridManager.IsValidGridPosition(gridPos))
            {
                Debug.LogError($"无效的栅格坐标：{gridPos}，路径转换失败");
                pathValid = false;
                break;
            }

            Vector3 worldPos = gridManager.栅格转世界(gridPos);
            if (float.IsNaN(worldPos.x) || float.IsNaN(worldPos.z) || float.IsInfinity(worldPos.x))
            {
                Debug.LogError($"栅格转世界坐标失败：{gridPos}");
                pathValid = false;
                break;
            }

            worldPos.y = 0.4f; // 固定Y轴高度
            worldPath.Add(worldPos);
        }

        // 路径转换有效，正常加载
        if (pathValid && worldPath.Count > 0)
        {
            // 路径加载成功：强制朝向最终目标点
            Vector3 finalTarget = worldPath[worldPath.Count - 1];
            Vector3 targetDir = (finalTarget - transform.position).normalized;
            transform.rotation = Quaternion.LookRotation(targetDir, Vector3.up);
            Debug.Log($"路径加载成功，已朝向最终目标点（目标位置：{finalTarget}）");

            currentWaypointIndex = 0;
            isPathLoaded = true;
            wasPathInvalid = false;
            路径重试次数 = 0;
            Debug.Log($"路径加载完成，包含{worldPath.Count}个路径点");
        }
        else
        {
            // 降级处理：直接朝向目标点移动（优化：验证目标点有效性）
            if (pathfinder.targetPos != null && !float.IsNaN(pathfinder.targetPos.position.x) && !float.IsInfinity(pathfinder.targetPos.position.x))
            {
                worldPath.Clear();
                worldPath.Add(transform.position); // 当前位置
                worldPath.Add(new Vector3(pathfinder.targetPos.position.x, 0.4f, pathfinder.targetPos.position.z)); // 目标点（固定Y轴）
                isPathLoaded = true;
                currentWaypointIndex = 0;
                Debug.Log("降级处理：直接朝向目标点移动");
                路径重试次数 = 0;
            }
            else
            {
                Debug.LogError("目标点无效，无法降级处理");
                Invoke(nameof(TryLoadPath), 1f);
            }
        }
    }

    // 朝向第一个路径点
    private void FaceFirstWaypoint()
    {
        if (worldPath.Count < 2) return;

        Vector3 targetDir = (worldPath[1] - transform.position).normalized;
        transform.rotation = Quaternion.LookRotation(targetDir, Vector3.up);
        Debug.Log($"无人船朝向已修正：{transform.forward}");
    }

    // 固定更新：路径跟随与状态处理（按要求增强路径无效检测）
    void FixedUpdate()
    {
        // 固定Y轴高度（防止上下浮动）
        transform.position = new Vector3(transform.position.x, 0.4f, transform.position.z);

        // 路径无效处理（增强：检查路径点是否含NaN/Infinity）
        bool isPathInvalid = worldPath == null || worldPath.Count == 0 ||
                          worldPath.Any(p => float.IsNaN(p.x) || float.IsNaN(p.z) || float.IsInfinity(p.x) || float.IsInfinity(p.z));

        if (isReachedEnd || isPathInvalid)
        {
            if (isPathInvalid && !wasPathInvalid)
            {
                Debug.LogWarning("路径无效，尝试重新加载...");
                CancelInvoke(nameof(TryLoadPath));
                Invoke(nameof(TryLoadPath), 1f); // 1秒重试一次
            }
            wasPathInvalid = isPathInvalid;
            return;
        }
        else if (wasPathInvalid)
        {
            wasPathInvalid = false;
            FaceFirstWaypoint();
        }

        // 到达最后一个路径点
        if (currentWaypointIndex >= worldPath.Count)
        {
            StopMovement();
            isReachedEnd = true;
            Debug.Log("已到达终点，停止移动");

            // 通知 RL Agent 结束当前回合
            USV_GlobalRLAgent rlAgent = GetComponent<USV_GlobalRLAgent>();
            if (rlAgent != null)
            {
                rlAgent.AddReward(100f);
                rlAgent.EndEpisode();
            }

            return;
        }

        // 移动到当前路径点
        Vector3 target = worldPath[currentWaypointIndex];
        Vector3 targetXZ = new Vector3(target.x, 0.4f, target.z);
        Vector3 currentXZ = new Vector3(transform.position.x, 0.4f, transform.position.z);
        float distance = Vector3.Distance(currentXZ, targetXZ);
        bool isLastWaypoint = (currentWaypointIndex == worldPath.Count - 1);
        float stopDistance = isLastWaypoint ? 1.5f : waypointDistance;

        // 到达当前路径点，切换到下一个
        if (distance <= stopDistance)
        {
            if (currentWaypointIndex + 1 < worldPath.Count)
            {
                Vector3 nextTargetXZ = new Vector3(worldPath[currentWaypointIndex + 1].x, 0.4f, worldPath[currentWaypointIndex + 1].z);
                float angleToNext = Vector3.Angle(transform.forward, nextTargetXZ - currentXZ);
                if (angleToNext < 90f)
                {
                    currentWaypointIndex++;
                }
            }
            else
            {
                currentWaypointIndex++;
            }
            return;
        }

        // 优化转向逻辑：朝向偏差大时加快转向速度
        Vector3 targetDir = (targetXZ - currentXZ).normalized;
        float angle = Vector3.Angle(transform.forward, targetDir);
        float rotationSpeedMultiplier = angle > 90f ? 2f : 1f;
        float maxRotationDelta = 30f * Time.fixedDeltaTime * rotationSpeed * rotationSpeedMultiplier;

        // 平滑转向目标方向
        Quaternion targetRotation = Quaternion.LookRotation(targetDir);
        targetRotation = Quaternion.Euler(0, targetRotation.eulerAngles.y, 0);
        transform.rotation = Quaternion.RotateTowards(transform.rotation, targetRotation, maxRotationDelta);

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

        // 速度平滑过渡（新增currentSpeed字段初始化）
        float currentSpeed = Mathf.Lerp(rb.linearVelocity.magnitude, targetSpeed, Time.fixedDeltaTime * 2f);
        Vector3 moveDir = transform.forward * currentSpeed;
        rb.linearVelocity = new Vector3(moveDir.x, rb.linearVelocity.y, moveDir.z);
    }

    // 补充缺失的currentSpeed字段初始化（修复编译错误）
    private float currentSpeed = 0f;
}