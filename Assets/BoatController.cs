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
    private Vector3 originalTargetPos;       // 新增：存储原始目标点（关键！）
    private bool isReplaningPath = false;     // 新增：标记是否正在重规划路径

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

        // 初始化原始目标点（需确保pathfinder已存储目标点，或从外部传入）
        if (pathfinder != null && pathfinder.targetWorldPos != Vector3.zero)
        {
            originalTargetPos = pathfinder.targetWorldPos;
            Debug.Log($"初始化原始目标点：{originalTargetPos}");
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

    // 碰撞处理：修复核心逻辑，保留原始目标点+触发精准重规划
    private void OnCollisionEnter(Collision collision)
    {
        if (collision.collider.CompareTag("USV") || collision.collider.CompareTag("Obstacle"))
        {
            Debug.LogError($"与{collision.collider.tag}发生碰撞！重新规划路径至原目标：{originalTargetPos}");
            StopMovement();
            worldPath.Clear(); // 仅清空路径，保留原始目标点
            isPathLoaded = false;

            // 避免重复重规划
            if (!isReplaningPath)
            {
                StartCoroutine(ReplanPathToOriginalTarget()); // 改用协程精准重规划
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

    // 核心修复：基于原始目标点的精准重规划协程
    private IEnumerator ReplanPathToOriginalTarget()
    {
        isReplaningPath = true;
        // 清空旧路径
        worldPath.Clear();
        currentWaypointIndex = 0;
        isPathLoaded = false;

        int retryCount = 0;
        int maxRetry = 3;
        bool replanSuccess = false;

        while (retryCount < maxRetry && !replanSuccess)
        {
            retryCount++;
            Debug.Log($"第{retryCount}次尝试重规划路径：当前位置→原目标点({originalTargetPos})");

            if (pathfinder != null && gridManager != null && originalTargetPos != Vector3.zero)
            {
                // 转换当前位置和原始目标点为栅格坐标
                Vector2Int currentGridPos = gridManager.WorldToGrid(transform.position);
                Vector2Int targetGridPos = gridManager.WorldToGrid(originalTargetPos);

                // 调用公开的FindPath方法（确保ImprovedAStar的FindPath为public）
                List<Vector2Int> newGridPath = pathfinder.FindPath(currentGridPos, targetGridPos);

                if (newGridPath != null && newGridPath.Count > 0)
                {
                    // 转换栅格路径为世界路径
                    worldPath.Clear();
                    foreach (var gridPos in newGridPath)
                    {
                        Vector3 worldPos = gridManager.GridToWorld(gridPos);
                        worldPos.y = 0.05f; // 固定Y轴高度
                        worldPath.Add(worldPos);
                    }
                    isPathLoaded = true;
                    replanSuccess = true;
                    Debug.Log($"路径重规划成功！新路径包含{worldPath.Count}个点");
                    // 修正朝向第一个路径点
                    FaceFirstWaypoint();
                }
                else
                {
                    Debug.LogWarning($"第{retryCount}次重规划失败，1秒后重试");
                    yield return new WaitForSeconds(1f);
                }
            }
            else
            {
                Debug.LogError("Pathfinder/GridManager未赋值，或原始目标点为空，无法重规划");
                yield break;
            }
        }

        // 重试耗尽仍失败的处理：偏移当前位置后再次尝试
        if (!replanSuccess)
        {
            Debug.LogError("重试耗尽，尝试偏移位置后重规划");
            Vector3 offsetPos = transform.position + Random.insideUnitSphere * 2f;
            offsetPos.y = 0.05f; // 固定Y轴
            Vector2Int offsetGridPos = gridManager.WorldToGrid(offsetPos);
            Vector2Int targetGridPos = gridManager.WorldToGrid(originalTargetPos);

            List<Vector2Int> newGridPath = pathfinder.FindPath(offsetGridPos, targetGridPos);
            if (newGridPath != null && newGridPath.Count > 0)
            {
                worldPath.Clear();
                foreach (var gridPos in newGridPath)
                {
                    worldPath.Add(gridManager.GridToWorld(gridPos));
                }
                isPathLoaded = true;
                replanSuccess = true;
                FaceFirstWaypoint();
            }
        }

        isReplaningPath = false;
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
                if (pathfinder.targetWorldPos != Vector3.zero)
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
            Vector3 worldPos = gridManager.GridToWorld(gridPos);
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
        if (worldPath.Count < 1) return;

        // 计算朝向第一个路径点的方向
        Vector3 targetDir = (worldPath[0] - transform.position).normalized;
        targetDir.y = 0; // 忽略Y轴
        // 直接设置朝向（跳过平滑转向，快速修正）
        transform.rotation = Quaternion.LookRotation(targetDir, Vector3.up);
        Debug.Log($"无人船朝向已修正：{transform.forward}");
    }

    // 移动逻辑（补充完整的FixedUpdate，确保碰撞后能沿新路径移动）
    void FixedUpdate()
    {
        // 固定Y轴高度（防止上下浮动）
        transform.position = new Vector3(transform.position.x, 0.4f, transform.position.z);

        // 路径无效时尝试重新加载（只在路径状态从有效变为无效时打印一次日志）
        if (isReachedEnd || worldPath == null || worldPath.Count == 0)
        {
            bool isPathInvalid = worldPath == null || worldPath.Count == 0;
            if (isPathInvalid && !wasPathInvalid)
            {
                Debug.LogWarning("路径无效，尝试重新加载...");
                wasPathInvalid = true;
                TryLoadPath();
            }
            return;
        }

        // 重置路径无效标记
        wasPathInvalid = false;

        // 到达终点判断
        if (currentWaypointIndex >= worldPath.Count)
        {
            if (!isReachedEnd)
            {
                Debug.Log("到达最终目标点！");
                StopMovement();
                isReachedEnd = true;
            }
            return;
        }

        // 移动到下一个路径点
        MoveToWaypoint();
    }

    // 核心移动逻辑：沿路径点移动
    private void MoveToWaypoint()
    {
        Vector3 targetWaypoint = worldPath[currentWaypointIndex];
        // 计算到目标路径点的方向（忽略Y轴）
        Vector3 direction = (targetWaypoint - transform.position).normalized;
        direction.y = 0;

        // 距离判断：是否到达当前路径点
        float distanceToWaypoint = Vector3.Distance(transform.position, targetWaypoint);
        if (distanceToWaypoint < waypointDistance)
        {
            // 切换到下一个路径点
            currentWaypointIndex++;
            return;
        }

        // 终点前减速逻辑
        float distanceToEnd = Vector3.Distance(transform.position, worldPath[worldPath.Count - 1]);
        float speedFactor = 1f;
        if (distanceToEnd < endPointSlowRange)
        {
            speedFactor = Mathf.Lerp(minEndSpeed / moveSpeed, 1f, distanceToEnd / endPointSlowRange);
        }
        currentSpeed = Mathf.Lerp(currentSpeed, moveSpeed * speedFactor, Time.fixedDeltaTime * rotationSpeed);

        // 平滑转向目标方向
        Quaternion targetRotation = Quaternion.LookRotation(direction);
        transform.rotation = Quaternion.Lerp(transform.rotation, targetRotation, Time.fixedDeltaTime * rotationSpeed);

        // 移动刚体
        rb.linearVelocity = transform.forward * currentSpeed;
    }

    // 外部设置原始目标点的方法（供外部调用，如路径管理器）
    public void SetOriginalTargetPos(Vector3 targetPos)
    {
        originalTargetPos = targetPos;
        originalTargetPos.y = 0.05f; // 固定Y轴
        Debug.Log($"外部设置原始目标点：{originalTargetPos}");
    }
}