using UnityEngine;
using System.Collections.Generic;

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
    public float waypointDistance = 1f;       // 路径点切换阈值
    public float endPointSlowRange = 2f;      // 终点前减速范围
    public float minEndSpeed = 0.5f;          // 终点前最小速度

    // 私有变量
    private List<Vector2Int> gridPath;        // 栅格坐标路径
    private List<Vector3> worldPath;          // 世界坐标路径
    private int currentWaypointIndex = 0;     // 当前路径点索引
    private Rigidbody rb;                     // 刚体组件
    private bool isReachedEnd = false;        // 是否到达终点
    private float currentSpeed = 0f;          // 当前速度（用于平滑过渡）
    private int 路径重试次数 = 0;            // 路径加载重试计数器
    private const int 最大重试次数 = 5;      // 最大重试次数

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

        // 初始加载路径
        TryLoadPath();
    }

    // 配置刚体物理参数
    private void ConfigureRigidbody()
    {
        rb.useGravity = false;                // 禁用重力（适用于水上运动）
        rb.interpolation = RigidbodyInterpolation.Interpolate; // 平滑刚体移动
        rb.collisionDetectionMode = CollisionDetectionMode.ContinuousDynamic; // 连续碰撞检测
        rb.linearDamping = 0.5f;              // 线性阻尼（阻力）
        rb.angularDamping = 2f;             // 角阻尼（旋转阻力）
    }

    // 碰撞处理
    private void OnCollisionEnter(Collision collision)
    {
        // 检测与其他无人船或障碍物的碰撞
        if (collision.collider.CompareTag("USV") || collision.collider.CompareTag("Obstacle"))
        {
            Debug.LogError($"与{collision.collider.tag}发生碰撞！暂停并重新规划路径");
            StopMovement();
            isReachedEnd = true;
            Invoke(nameof(ResumeMovement), 1f);  // 1秒后恢复移动
            if (pathfinder != null)
            {
                Invoke(nameof(ReplanPath), 1f);   // 1秒后重新规划路径
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
    private void TryLoadPath()
    {
        if (worldPath.Count > 0)
        {
            Debug.Log($"成功加载世界坐标路径，共{worldPath.Count}个点");
            // 可选：绘制世界坐标路径
            for (int i = 0; i < worldPath.Count - 1; i++)
            {
                Debug.DrawLine(worldPath[i], worldPath[i + 1], Color.green, 5f);
            }
        }

        // 检查必要引用
        if (pathfinder == null || gridManager == null)
        {
            string errorMsg = pathfinder == null ? "路径查找器未赋值！" : "网格管理器未赋值！";
            Debug.LogError(errorMsg);

            // 重试逻辑
            if (路径重试次数 < 最大重试次数)
            {
                路径重试次数++;
                Debug.LogWarning($"第{路径重试次数}次重试加载路径...");
                Invoke(nameof(TryLoadPath), 1f);
            }
            return;
        }

        // 检查路径是否有效
        if (pathfinder.path == null || pathfinder.path.Count == 0)
        {
            Debug.LogWarning($"路径数据为空，触发A*重新计算...（第{路径重试次数 + 1}次重试）");
            pathfinder.CalculatePathAfterDelay();
            路径重试次数++;

            // 重试或终止
            if (路径重试次数 < 最大重试次数)
                Invoke(nameof(TryLoadPath), 2f);
            else
                Debug.LogError("路径重试次数达到上限，A*路径仍为空！请检查障碍物配置或起点终点是否可达。");
            return;
        }

        // 转换路径坐标（栅格坐标 -> 世界坐标）
        gridPath = pathfinder.path;
        worldPath = new List<Vector3>();

        foreach (var gridPos in gridPath)
        {
            // 检查栅格坐标有效性
            if (!gridManager.IsValidGridPosition(gridPos))
            {
                Debug.LogError($"无效的栅格坐标：{gridPos}，路径转换失败");
                worldPath = null;
                Invoke(nameof(TryLoadPath), 1f);
                return;
            }

            // 转换为世界坐标
            Vector3 worldPos = gridManager.栅格转世界(gridPos);
            if (float.IsNaN(worldPos.x) || float.IsNaN(worldPos.z))
            {
                Debug.LogError($"栅格转世界坐标失败：{gridPos}");
                worldPath = null;
                Invoke(nameof(TryLoadPath), 1f);
                return;
            }

            // 固定Y轴高度（适配水面）
            worldPos.y = 0.05f;
            worldPath.Add(worldPos);
        }

        // 检查转换后的路径是否为空
        if (worldPath.Count == 0)
        {
            Debug.LogError("路径转换后为空，1秒后重试...");
            Invoke(nameof(TryLoadPath), 1f);
            return;
        }

        // 路径加载成功
        Debug.Log($"成功读取路径，共{worldPath.Count}个路径点");
        isReachedEnd = false;
        currentWaypointIndex = 0;
        路径重试次数 = 0;  // 重置重试计数器
    }

    // 物理更新（固定时间间隔）
    void FixedUpdate()
    {
        // 固定Y轴高度（防止上下浮动）
        transform.position = new Vector3(transform.position.x, 0.4f, transform.position.z);

        // 路径无效时尝试重新加载
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
            StopMovement();
            isReachedEnd = true;
            Debug.Log("已到达终点，停止移动");
            return;
        }

        // 移动到当前路径点
        Vector3 target = worldPath[currentWaypointIndex];
        Vector3 targetXZ = new Vector3(target.x, 0.4f, target.z);  // 忽略Y轴差异
        Vector3 currentXZ = new Vector3(transform.position.x, 0.4f, transform.position.z);
        float distance = Vector3.Distance(currentXZ, targetXZ);
        bool isLastWaypoint = (currentWaypointIndex == worldPath.Count - 1);
        float stopDistance = isLastWaypoint ? 0.5f : waypointDistance;  // 终点判断阈值更小

        // 到达当前路径点，切换到下一个
        if (distance <= stopDistance)
        {
            currentWaypointIndex++;
            if (currentWaypointIndex < worldPath.Count)
            {
                // 提前转向下一个路径点
                Vector3 nextTarget = worldPath[currentWaypointIndex];
                Vector3 nextTargetXZ = new Vector3(nextTarget.x, 0.4f, nextTarget.z);
                // 修改后（平滑插值）
                Quaternion nextRotation = Quaternion.LookRotation(nextTargetXZ - currentXZ);
                nextRotation = Quaternion.Euler(0, nextRotation.eulerAngles.y, 0);
                transform.rotation = Quaternion.Lerp(transform.rotation, nextRotation, rotationSpeed * Time.fixedDeltaTime * 2f);
            }
            return;
        }

        // 平滑转向目标方向
        Quaternion targetRotation = Quaternion.LookRotation(targetXZ - currentXZ);
        targetRotation = Quaternion.Euler(0, targetRotation.eulerAngles.y, 0);  // 只旋转Y轴
        transform.rotation = Quaternion.Lerp(transform.rotation, targetRotation, rotationSpeed * Time.fixedDeltaTime);

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
}