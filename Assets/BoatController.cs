using UnityEngine;
using System.Collections.Generic;
using System.Collections; // 必须导入这个命名空间！


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

    // 私有变量
    private List<Vector2Int> gridPath;        // 栅格坐标路径
    private List<Vector3> worldPath = new List<Vector3>();  // 初始化！避免null
    private int currentWaypointIndex = 0;     // 当前路径点索引
    private Rigidbody rb;                     // 刚体组件
    private bool isReachedEnd = false;        // 是否到达终点
    private float currentSpeed = 0f;          // 当前速度（用于平滑过渡）
    private int 路径重试次数 = 0;            // 路径加载重试计数器
    private const int 最大重试次数 = 5;      // 最大重试次数
    private bool wasPathInvalid = false;     // 新增字段用于跟踪路径失效状态
    public bool isPathLoaded = false;        // 新增：标记路径是否加载完成

    // 初始化
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

        // 等待栅格初始化后加载路径（新增协程等待）
        StartCoroutine(WaitForGridInitThenLoadPath());
    }

    // 新增：等待栅格初始化后加载路径
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

    // 碰撞处理
    private void OnCollisionEnter(Collision collision)
    {
        if (collision.collider.CompareTag("USV") || collision.collider.CompareTag("Obstacle"))
        {
            Debug.LogError($"与{collision.collider.tag}发生碰撞！暂停并重新规划路径");
            StopMovement();
            isReachedEnd = true;
            worldPath.Clear();
            Invoke(nameof(ResumeMovement), 1f);
            if (pathfinder != null)
            {
                // 延迟更长时间再重规划，避免碰撞瞬间连续触发
                Invoke(nameof(ReplanPath), 2f);
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

    // 重新规划路径
    private void ReplanPath()
    {
        if (pathfinder != null)
        {
            Debug.Log("触发路径重新规划...");
            pathfinder.CalculatePathAfterDelay();  // 调用A*的路径计算方法
            路径重试次数 = 0;                     // 重置重试计数器
            Invoke(nameof(TryLoadPath), 1f);       // 1秒后尝试加载新路径
        }
    }

    // 尝试加载路径
    public void TryLoadPath()
    {

        // 新增：如果路径已加载且有效，则无需重新加载
        if (isPathLoaded && worldPath != null && worldPath.Count > 0)
        {
            Debug.Log("路径已加载且有效，无需重复请求");
            return;
        }

        // 新增：如果栅格未初始化，等待后重试
        if (gridManager != null && !gridManager.IsGridReady())
        {
            Debug.Log("栅格未初始化，延迟重试路径加载...");
            Invoke(nameof(TryLoadPath), 1f);
            return;
        }

        // 原逻辑：检查路径finder是否有效
        if (pathfinder == null)
        {
            Debug.LogError("路径查找器未赋值！");
            return;
        }

        // 第一步：先检查关键引用是否存在（必做！）
        if (pathfinder == null || gridManager == null)
        {
            string errorMsg = pathfinder == null ? "路径查找器未赋值！" : "网格管理器未赋值！";
            Debug.LogError(errorMsg);
            // 重试逻辑（保持原逻辑）
            if (路径重试次数 < 最大重试次数)
            {
                路径重试次数++;
                Debug.LogWarning($"第{路径重试次数}次重试加载路径...");
                Invoke(nameof(TryLoadPath), 1f);
            }
            return;
        }

        // 第二步：检查路径是否有效（修改重试逻辑）
        if (pathfinder.path == null || pathfinder.path.Count == 0)
        {
            Debug.LogWarning($"路径数据为空，触发A*重新计算...（第{路径重试次数 + 1}次重试）");
            pathfinder.CalculatePathAfterDelay();
            路径重试次数++;
            // 延长重试间隔至1秒，确保A*有足够时间计算
            float retryDelay = 1f;
            if (路径重试次数 < 最大重试次数)
                Invoke(nameof(TryLoadPath), retryDelay);
            else
            {
                Debug.LogError("路径重试次数达到上限，强制刷新A*后重试");
                pathfinder = FindFirstObjectByType<ImprovedAStar>(); // 重新获取A*引用
                pathfinder.CalculatePathAfterDelay();
                路径重试次数 = 0;
                Invoke(nameof(TryLoadPath), 1f);
            }
            return;
        }

        // 第三步：转换路径坐标（栅格→世界）（原代码逻辑保留，无需修改）
        gridPath = pathfinder.path;
        worldPath.Clear(); // 清空旧路径，避免重复
        foreach (var gridPos in gridPath)
        {
            // 检查栅格坐标有效性
            if (!gridManager.IsValidGridPosition(gridPos))
            {
                Debug.LogError($"无效的栅格坐标：{gridPos}，路径转换失败");
                worldPath.Clear(); // 清空无效路径
                Invoke(nameof(TryLoadPath), 1f);
                return;
            }

            // 转换为世界坐标
            Vector3 worldPos = gridManager.栅格转世界(gridPos);
            if (float.IsNaN(worldPos.x) || float.IsNaN(worldPos.z))
            {
                Debug.LogError($"栅格转世界坐标失败：{gridPos}");
                worldPath.Clear();
                Invoke(nameof(TryLoadPath), 1f);
                return;
            }

            // 固定Y轴高度（适配水面）
            worldPos.y = 0.05f;
            worldPath.Add(worldPos);
        }

        // 第四步：检查转换后的路径是否为空
        if (worldPath.Count == 0)
        {
            Debug.LogError("路径转换后为空，1秒后重试...");
            Invoke(nameof(TryLoadPath), 1f);
            return;
        }

        // 路径加载成功（原逻辑保留，新增朝向修正）
        Debug.Log($"成功读取路径，共{worldPath.Count}个路径点");
        isPathLoaded = true;
        isReachedEnd = false;
        currentWaypointIndex = 0;
        路径重试次数 = 0;
        // 关键：强制朝向第一个路径点（新增）
        FaceFirstWaypoint();
    }

    // 新增：朝向第一个路径点
    private void FaceFirstWaypoint()
    {
        if (worldPath.Count < 2) return;

        // 计算朝向第一个路径点的方向
        Vector3 targetDir = (worldPath[1] - transform.position).normalized;
        // 直接设置朝向（跳过平滑转向，快速修正）
        transform.rotation = Quaternion.LookRotation(targetDir, Vector3.up);
        Debug.Log($"无人船朝向已修正：{transform.forward}");
    }

    void FixedUpdate()
    {
        // 固定Y轴高度（防止上下浮动）
        transform.position = new Vector3(transform.position.x, 0.4f, transform.position.z);

        // 路径无效时尝试重新加载（只在路径状态从有效变为无效时打印一次日志）
        if (isReachedEnd || worldPath == null || worldPath.Count == 0)
        {
            bool isPathInvalid = worldPath == null || worldPath.Count == 0;
            // 当路径从有效变为无效时，打印一次日志
            if (isPathInvalid && !wasPathInvalid)
            {
                Debug.LogWarning("路径无效，开始尝试重新加载...");
                TryLoadPath();
            }
            // 更新路径失效状态标记
            wasPathInvalid = isPathInvalid;
            return;
        }
        // 路径恢复有效时重置标记
        else if (wasPathInvalid)
        {
            wasPathInvalid = false;
        }

        // 到达最后一个路径点
        // BoatController.cs 第270行附近（终点判定处）
        if (currentWaypointIndex >= worldPath.Count)
        {
            StopMovement();
            isReachedEnd = true;
            Debug.Log("已到达终点，停止移动");

            // 新增：通知 RL Agent 结束当前回合
            USV_GlobalRLAgent rlAgent = GetComponent<USV_GlobalRLAgent>();
            if (rlAgent != null)
            {
                rlAgent.AddReward(100f); // 可选：补充终点奖励
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
        float stopDistance = isLastWaypoint ? 0.8f : waypointDistance; // 终点阈值稍大，避免过度靠近

        // 到达当前路径点，切换到下一个（优化：只有距离小于阈值且方向正确时才切换）
        if (distance <= stopDistance)
        {
            // 计算当前朝向与下一个路径点的夹角（避免反向时切换）
            if (currentWaypointIndex + 1 < worldPath.Count)
            {
                Vector3 nextTargetXZ = new Vector3(worldPath[currentWaypointIndex + 1].x, 0.4f, worldPath[currentWaypointIndex + 1].z);
                float angleToNext = Vector3.Angle(transform.forward, nextTargetXZ - currentXZ);
                if (angleToNext < 90f) // 只有朝向接近下一个路径点时才切换
                {
                    currentWaypointIndex++;
                }
            }
            else
            {
                currentWaypointIndex++; // 最后一个路径点直接切换
            }
            return;
        }

        // 优化转向逻辑：当朝向与路径方向夹角大于90度时，加快转向速度（核心修改）
        Vector3 targetDir = (targetXZ - currentXZ).normalized;
        float angle = Vector3.Angle(transform.forward, targetDir);
        // 朝向偏差大时，加快转向速度（原旋转速度 * 2）
        float rotationSpeedMultiplier = angle > 90f ? 2f : 1f;
        float maxRotationDelta = 30f * Time.fixedDeltaTime * rotationSpeed * rotationSpeedMultiplier;

        // 平滑转向目标方向（限制最大转向角度）
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
                // 距离终点越近，速度越慢
                float speedRatio = distanceToEnd / endPointSlowRange;
                targetSpeed = Mathf.Lerp(minEndSpeed, moveSpeed * 0.5f, speedRatio);
            }
            else
            {
                targetSpeed = moveSpeed * 0.5f;  // 接近终点区域时先减速到一半
            }
        }

        // 速度平滑过渡（避免突然加速/减速）
        currentSpeed = Mathf.Lerp(currentSpeed, targetSpeed, Time.fixedDeltaTime * 2f);
        Vector3 moveDir = transform.forward * currentSpeed;
        rb.linearVelocity = new Vector3(moveDir.x, rb.linearVelocity.y, moveDir.z);  // 保持Y轴速度不变
    }

    // 新增：供外部判断路径是否加载完成
    public bool IsPathLoaded => isPathLoaded;
}