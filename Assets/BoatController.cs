using UnityEngine;
using System.Collections.Generic;
using System.Collections;

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
    [SerializeField] private float waypointDistance = 1.0f;       // 匹配目标版本的1f
    public float endPointSlowRange = 2f;      // 终点前减速范围
    public float minEndSpeed = 0.5f;          // 终点前最小速度

    // 碰撞避障新增参数（可在Inspector调试）
    [Header("碰撞避障参数")]
    public float collisionAvoidanceForce = 2f; // 碰撞后避障推力
    public float collisionRotationAngle = 30f; // 碰撞后转向角度（度）
    public float collisionSpeedRecoveryTime = 0.5f; // 速度恢复时间
    public float collisionIgnoreTime = 0.3f;   // 碰撞后短时间忽略重复碰撞（防止抖动）

    // 私有变量
    private List<Vector2Int> gridPath;        // 栅格坐标路径
    private List<Vector3> worldPath = new List<Vector3>();  // 初始化！避免null
    private int currentWaypointIndex = 0;     // 当前路径点索引
    private Rigidbody rb;                     // 刚体组件
    private bool isReachedEnd = false;        // 是否到达终点
    private float currentSpeed = 0f;          // 当前速度（用于平滑过渡）
    private int 路径重试次数 = 0;            // 路径加载重试计数器
    private const int 最大重试次数 = 5;      // 最大重试次数
    public bool isPathLoaded = false;        // 新增：标记路径是否加载完成
    private Vector3 originalTargetPos;       // 新增：存储原始目标点（关键！）
    private bool isInCollisionAvoidance = false; // 是否处于避障状态
    private float lastCollisionTime = 0f;     // 最后一次碰撞时间

    // ======== 核心修复：重构Awake，优先从栅格生成有效目标点 ========
    void Awake()
    {
        // 启动协程初始化有效目标点（优先从GridManager获取通行区随机点）
        StartCoroutine(InitValidOriginalTarget());
    }

    // 新增：初始化有效原始目标点（解决全0问题）
    private IEnumerator InitValidOriginalTarget()
    {
        // 等待GridManager初始化完成
        while (gridManager == null || !gridManager.IsGridReady())
        {
            Debug.Log("等待GridManager初始化以生成有效目标点...");
            yield return new WaitForSeconds(0.1f);
        }

        // 优先级1：使用pathfinder的目标点（如果有效）
        if (pathfinder != null && pathfinder.targetWorldPos != Vector3.zero)
        {
            originalTargetPos = pathfinder.targetWorldPos;
            originalTargetPos.y = 0.05f; // 固定Y轴
            Debug.Log($"从Pathfinder初始化有效原始目标点：{originalTargetPos}");
        }
        // 优先级2：从GridManager获取通行区随机点
        else if (gridManager != null)
        {
            originalTargetPos = gridManager.GetRandomWalkablePosition();
            if (originalTargetPos == Vector3.zero)
            {
                Debug.LogError("目标点生成失败！GridManager通行区域无有效坐标");
                originalTargetPos = transform.position + new Vector3(10, 0.05f, 10); // 兜底：默认前方向10米
            }
            Debug.Log($"从GridManager通行区初始化原始目标点：{originalTargetPos}");

            // 同步更新pathfinder的目标点（保持一致性）
            if (pathfinder != null)
            {
                pathfinder.targetWorldPos = originalTargetPos;
            }
        }
        // 兜底：防止全0
        else
        {
            Debug.LogWarning("GridManager未赋值，目标点兜底初始化");
            originalTargetPos = transform.position + new Vector3(10, 0.05f, 10);
        }
    }

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

        // 配置刚体参数（保留原有配置 + 新增目标版本的阻尼参数）
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

    // 配置刚体物理参数（整合原有+目标版本的阻尼配置，移除重复赋值）
    private void ConfigureRigidbody()
    {
        rb.useGravity = false;                // 禁用重力（适用于水上运动）
        rb.interpolation = RigidbodyInterpolation.Interpolate; // 平滑刚体移动
        rb.collisionDetectionMode = CollisionDetectionMode.ContinuousDynamic; // 连续碰撞检测
        // 统一设置阻尼参数（移除重复赋值，保留目标版本命名）
        rb.linearDamping = 0.5f;              // 线性阻尼（阻力）
        rb.angularDamping = 0.8f;             // 角阻尼（旋转阻力）
    }

    // ========== 核心修改：碰撞处理逻辑（移除暂停/重规划，改为轻量避障） ==========
    private void OnCollisionEnter(Collision collision)
    {
        // 忽略重复碰撞（短时间内只处理一次）
        if (Time.time - lastCollisionTime < collisionIgnoreTime)
            return;

        if (collision.collider.CompareTag("USV") || collision.collider.CompareTag("Obstacle"))
        {
            lastCollisionTime = Time.time;
            Debug.Log($"碰撞到{collision.collider.tag}，执行轻量避障（{Time.time}）");

            // 1. 记录碰撞数据（供训练用，可根据需求扩展）
            RecordCollisionData(collision);

            // 2. 执行避障逻辑（不中断路径，仅临时调整方向/速度）
            StartCoroutine(ExecuteCollisionAvoidance(collision));
        }
    }

    // 新增：记录碰撞数据（可对接训练系统）
    private void RecordCollisionData(Collision collision)
    {
        // 示例：记录碰撞时间、位置、碰撞对象、当前速度、路径进度等
        Debug.Log($"【训练数据】碰撞时间：{Time.time}，位置：{transform.position}，碰撞对象：{collision.collider.name}，当前路径点：{currentWaypointIndex}/{worldPath.Count}");
        // 可扩展：将数据写入List/CSV/训练系统接口
    }

    // 新增：执行轻量避障（临时调整方向和速度，不重置路径）
    private IEnumerator ExecuteCollisionAvoidance(Collision collision)
    {
        isInCollisionAvoidance = true;

        // 1. 轻微减速（不停止）
        float originalSpeed = currentSpeed;
        currentSpeed = originalSpeed * 0.3f;
        rb.linearVelocity = rb.linearVelocity * 0.5f; // 降低当前速度

        // 2. 计算避障方向（远离碰撞体，或随机侧转）
        Vector3 avoidDir = GetAvoidanceDirection(collision);

        // 3. 应用避障推力和转向
        rb.AddForce(avoidDir * collisionAvoidanceForce, ForceMode.Impulse);
        Quaternion targetRot = Quaternion.LookRotation(avoidDir);
        transform.rotation = Quaternion.Lerp(transform.rotation, targetRot, 0.3f);

        // 4. 短暂保持避障状态后恢复速度
        yield return new WaitForSeconds(collisionSpeedRecoveryTime);

        // 5. 恢复速度，继续沿原路径前进
        currentSpeed = originalSpeed;
        isInCollisionAvoidance = false;
        Debug.Log("避障完成，恢复原路径运动");
    }

    // 新增：计算避障方向（远离碰撞体，优先侧方避让）
    private Vector3 GetAvoidanceDirection(Collision collision)
    {
        // 方式1：远离碰撞体中心
        Vector3 awayFromCollision = (transform.position - collision.contacts[0].point).normalized;
        awayFromCollision.y = 0; // 忽略Y轴

        // 方式2：如果远离方向无效，随机侧转（左/右）
        if (awayFromCollision.magnitude < 0.1f)
        {
            int randomSide = Random.Range(0, 2) == 0 ? -1 : 1;
            awayFromCollision = Quaternion.Euler(0, collisionRotationAngle * randomSide, 0) * transform.forward;
        }

        return awayFromCollision;
    }

    // ========== 移除：原碰撞后重规划路径的方法（不再使用） ==========
    // private void ReplanPathAfterCollision()
    // {
    //     isPathLoaded = false; // 标记路径失效
    //     TryLoadPath(); // 重新加载路径
    // }

    // ========== 移除：原恢复运动逻辑（改为避障后自动恢复） ==========
    // private void ResumeMovement()
    // {
    //     currentWaypointIndex = Mathf.Max(0, currentWaypointIndex - 1);
    //     isReachedEnd = false;
    // }

    // 停止移动（保留原有方法，兼容历史逻辑）
    private void StopMovement()
    {
        rb.linearVelocity = Vector3.zero;
        rb.angularVelocity = Vector3.zero;
        currentSpeed = 0f;
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
            // 基于原始目标点重新计算路径
            if (originalTargetPos != Vector3.zero)
            {
                Vector2Int currentGridPos = gridManager.WorldToGrid(transform.position);
                Vector2Int targetGridPos = gridManager.WorldToGrid(originalTargetPos);
                pathfinder.path = pathfinder.FindPath(currentGridPos, targetGridPos);
            }
            路径重试次数++;
            // 延长重试间隔至1秒，确保A*有足够时间计算
            float retryDelay = 1f;
            if (路径重试次数 < 最大重试次数)
                Invoke(nameof(TryLoadPath), retryDelay);
            else
            {
                Debug.LogError("路径重试次数达到上限，强制刷新A*后重试");
                pathfinder = FindFirstObjectByType<ImprovedAStar>(); // 重新获取A*引用
                // 重新赋值原始目标点
                if (pathfinder != null && pathfinder.targetWorldPos != Vector3.zero)
                {
                    originalTargetPos = pathfinder.targetWorldPos;
                }
                // 重新计算路径
                Vector2Int currentGridPos = gridManager.WorldToGrid(transform.position);
                Vector2Int targetGridPos = gridManager.WorldToGrid(originalTargetPos);
                pathfinder.path = pathfinder.FindPath(currentGridPos, targetGridPos);
                路径重试次数 = 0;
                Invoke(nameof(TryLoadPath), 1f);
            }
            return;
        }

        // 第三步：转换路径坐标（栅格→世界）（兼容目标版本的"栅格转世界"方法名）
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

            // 转换为世界坐标（兼容目标版本的方法名：优先用"栅格转世界"，兼容原有"GridToWorld"）
            Vector3 worldPos;
            try
            {
                // 尝试调用目标版本的"栅格转世界"方法
                var method = gridManager.GetType().GetMethod("栅格转世界");
                if (method != null)
                {
                    worldPos = (Vector3)method.Invoke(gridManager, new object[] { gridPos });
                }
                else
                {
                    // 兼容原有GridToWorld方法
                    worldPos = gridManager.GridToWorld(gridPos);
                }
            }
            catch
            {
                // 兜底：使用原有方法
                worldPos = gridManager.GridToWorld(gridPos);
            }

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
        // 关键：强制朝向第一个路径点（增加零向量防护）
        FaceFirstWaypoint();
    }

    // 核心修复：朝向第一个路径点（增加零向量防护，解决LookRotation报错）
    private void FaceFirstWaypoint()
    {
        // 防护1：路径点为空直接返回
        if (worldPath == null || worldPath.Count < 1)
        {
            Debug.LogError("路径点数组为空，无法朝向目标");
            return;
        }

        // 防护2：计算方向向量并校验是否为零
        Vector3 firstWaypoint = worldPath[0];
        Vector3 direction = firstWaypoint - transform.position;
        direction.y = 0; // 忽略Y轴

        // 校验方向向量是否为零（使用sqrMagnitude避免开方，性能更优）
        if (direction.sqrMagnitude < 0.001f)
        {
            Debug.LogWarning("朝向目标的方向向量为零，跳过LookRotation");
            return;
        }

        // 安全执行朝向设置
        transform.rotation = Quaternion.LookRotation(direction.normalized, Vector3.up);
        Debug.Log($"无人船朝向已修正：{transform.forward}，目标路径点：{firstWaypoint}");
    }

    // ========== 核心优化：FixedUpdate增加避障状态判断 ==========
    void FixedUpdate()
    {
        // 固定Y轴高度，避免上下浮动
        transform.position = new Vector3(transform.position.x, 0.4f, transform.position.z);

        if (isReachedEnd || worldPath == null || worldPath.Count == 0)
            return;

        // 到达最后一个路径点
        if (currentWaypointIndex >= worldPath.Count)
        {
            rb.linearVelocity = Vector3.zero;
            isReachedEnd = true;
            Debug.Log("已到达终点，停止移动");
            return;
        }

        // 避障状态下跳过常规移动逻辑（由避障协程处理）
        if (isInCollisionAvoidance)
            return;

        // 移动到当前路径点（目标版本逻辑）
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
        rb.linearVelocity = new Vector3(moveDir.x, rb.linearVelocity.y, moveDir.z);
    }

    // 外部设置原始目标点的方法（供外部调用，如路径管理器）
    public void SetOriginalTargetPos(Vector3 targetPos)
    {
        originalTargetPos = targetPos;
        originalTargetPos.y = 0.05f; // 固定Y轴
        Debug.Log($"外部设置原始目标点：{originalTargetPos}");
        // 目标点变更后重新加载路径
        isPathLoaded = false;
        TryLoadPath();
    }

    // 可选：Gizmos绘制避障方向（调试用）
    private void OnDrawGizmos()
    {
        if (isInCollisionAvoidance)
        {
            Gizmos.color = Color.red;
            Gizmos.DrawRay(transform.position, transform.forward * 2f);
        }
    }
}