using UnityEngine;

/// <summary>
/// 无人船自主移动控制（手动优先+自动随机移动）- 适配GridManager栅格系统
/// 修复点：1. 替换过时的FindObjectOfType → FindFirstObjectByType 2. 强化栅格坐标转换健壮性
/// </summary>
[RequireComponent(typeof(Collider))] // 确保有碰撞体（栅格检测辅助）
public class USV_AutoMovement : MonoBehaviour
{
    [Header("移动基础设置")]
    [Tooltip("无人船手动移动速度")]
    public float usvMoveSpeed = 5f;
    [Tooltip("离水面高度偏移（固定Y轴）")]
    public float waterHeightOffset = 0.5f;

    [Header("按键控制配置")]
    public KeyCode forwardControlKey = KeyCode.W; // 前进
    public KeyCode backwardControlKey = KeyCode.S; // 后退
    public KeyCode leftControlKey = KeyCode.A; // 左移
    public KeyCode rightControlKey = KeyCode.D; // 右移

    [Header("自动移动配置")]
    [Tooltip("自动移动速度（建议低于手动速度）")]
    public float autoMoveSpeed = 3f;
    [Tooltip("随机方向更新间隔（秒）")]
    public float randomDirUpdateInterval = 1.5f;
    [Tooltip("是否开启障碍躲避（基于GridManager栅格）")]
    public bool enableObstacleAvoid = true;
    [Tooltip("是否限制水域边界（基于GridManager）")]
    public bool enableWaterBoundaryLimit = true;

    // 私有变量
    private Vector3 _randomMoveDir; // 当前随机移动方向
    private float _dirUpdateTimer; // 随机方向更新计时器
    private GridManager _gridManager; // 栅格管理器引用
    private bool _isGridReady; // 栅格就绪标记

    void Start()
    {
        // 初始化随机方向（避免初始帧无方向）
        _randomMoveDir = new Vector3(Random.Range(-1f, 1f), 0, Random.Range(-1f, 1f)).normalized;

        // 修复：替换过时的FindObjectOfType → FindFirstObjectByType（兼容Unity新版本）
        // 如需包含非激活对象，添加参数：FindObjectsInactive.Include
        _gridManager = Object.FindFirstObjectByType<GridManager>();

        if (_gridManager == null && (enableObstacleAvoid || enableWaterBoundaryLimit))
        {
            Debug.LogWarning($"[{name}] 未找到GridManager，障碍躲避/边界限制功能将失效！");
        }

        // 校验参数合法性
        ValidateParameters();
    }

    void Update()
    {
        // 更新栅格就绪状态（强化空值校验）
        _isGridReady = _gridManager != null && _gridManager.IsGridReady();

        Vector3 currentPosition = transform.position;
        float xAxisMovement = 0f;
        float zAxisMovement = 0f;

        // 1. 检测是否有手动输入（手动控制优先级最高）
        bool hasManualInput = CheckManualInput();

        if (hasManualInput)
        {
            // 2. 手动控制逻辑
            xAxisMovement = GetManualXMovement();
            zAxisMovement = GetManualZMovement();

            // 手动控制也限制水域边界
            if (enableWaterBoundaryLimit && _isGridReady)
            {
                LimitPositionToWaterBoundary(ref xAxisMovement, ref zAxisMovement, currentPosition);
            }
        }
        else
        {
            // 3. 无手动输入时，执行自动随机移动
            AutoRandomMovement(ref xAxisMovement, ref zAxisMovement);
        }

        // 更新位置并固定水面高度（限制NaN/无穷大值）
        UpdateUSVPosition(currentPosition, xAxisMovement, zAxisMovement);
    }

    /// <summary>
    /// 校验参数合法性，避免运行时异常
    /// </summary>
    private void ValidateParameters()
    {
        if (usvMoveSpeed < 0)
        {
            Debug.LogWarning($"[{name}] 手动移动速度不能为负，已重置为5", this);
            usvMoveSpeed = 5f;
        }
        if (autoMoveSpeed < 0)
        {
            Debug.LogWarning($"[{name}] 自动移动速度不能为负，已重置为3", this);
            autoMoveSpeed = 3f;
        }
        if (randomDirUpdateInterval <= 0)
        {
            Debug.LogWarning($"[{name}] 方向更新间隔必须大于0，已重置为1.5", this);
            randomDirUpdateInterval = 1.5f;
        }
    }

    /// <summary>
    /// 检测是否有手动输入
    /// </summary>
    private bool CheckManualInput()
    {
        return Input.GetKey(forwardControlKey) || Input.GetKey(backwardControlKey) ||
               Input.GetKey(leftControlKey) || Input.GetKey(rightControlKey);
    }

    /// <summary>
    /// 获取手动控制的X轴位移
    /// </summary>
    private float GetManualXMovement()
    {
        float x = 0f;
        if (Input.GetKey(leftControlKey)) x -= usvMoveSpeed * Time.deltaTime;
        if (Input.GetKey(rightControlKey)) x += usvMoveSpeed * Time.deltaTime;
        return x;
    }

    /// <summary>
    /// 获取手动控制的Z轴位移
    /// </summary>
    private float GetManualZMovement()
    {
        float z = 0f;
        if (Input.GetKey(forwardControlKey)) z += usvMoveSpeed * Time.deltaTime;
        if (Input.GetKey(backwardControlKey)) z -= usvMoveSpeed * Time.deltaTime;
        return z;
    }

    /// <summary>
    /// 自动随机移动逻辑（适配GridManager栅格障碍/边界）
    /// </summary>
    private void AutoRandomMovement(ref float xMove, ref float zMove)
    {
        // 定时更新随机移动方向
        _dirUpdateTimer += Time.deltaTime;
        if (_dirUpdateTimer >= randomDirUpdateInterval)
        {
            UpdateRandomMoveDir();
            _dirUpdateTimer = 0f;
        }

        // 障碍躲避逻辑（基于栅格）
        if (enableObstacleAvoid && _isGridReady)
        {
            AvoidObstaclesByGrid();
        }

        // 水域边界限制
        if (enableWaterBoundaryLimit && _isGridReady)
        {
            LimitDirectionToWaterBoundary();
        }

        // 应用自动移动位移（限制最大速度）
        xMove = Mathf.Clamp(_randomMoveDir.x * autoMoveSpeed * Time.deltaTime, -autoMoveSpeed * Time.deltaTime, autoMoveSpeed * Time.deltaTime);
        zMove = Mathf.Clamp(_randomMoveDir.z * autoMoveSpeed * Time.deltaTime, -autoMoveSpeed * Time.deltaTime, autoMoveSpeed * Time.deltaTime);
    }

    /// <summary>
    /// 更新随机移动方向（确保方向有效）
    /// </summary>
    private void UpdateRandomMoveDir()
    {
        Vector3 newDir = new Vector3(Random.Range(-1f, 1f), 0, Random.Range(-1f, 1f));
        // 避免零向量（防止无移动）
        _randomMoveDir = newDir.magnitude < 0.1f ? Vector3.forward : newDir.normalized;
    }

    /// <summary>
    /// 基于GridManager栅格的障碍躲避逻辑
    /// </summary>
    private void AvoidObstaclesByGrid()
    {
        int retryCount = 0;
        const int maxRetry = 5; // 最大重试次数，避免死循环
        bool isCurrentDirBlocked = IsDirectionBlockedByGrid(_randomMoveDir);

        // 检测到障碍则重试随机方向
        while (isCurrentDirBlocked && retryCount < maxRetry)
        {
            UpdateRandomMoveDir();
            isCurrentDirBlocked = IsDirectionBlockedByGrid(_randomMoveDir);
            retryCount++;
        }

        // 绘制调试射线
        Color rayColor = isCurrentDirBlocked ? Color.red : Color.green;
        Debug.DrawRay(transform.position, _randomMoveDir * 2f, rayColor, 0.1f);

        if (isCurrentDirBlocked)
        {
            Debug.LogWarning($"[{name}] 尝试{maxRetry}次仍检测到栅格障碍，停止自动移动", this);
            _randomMoveDir = Vector3.zero; // 停止移动，避免撞墙
        }
    }

    /// <summary>
    /// 检测指定方向的栅格是否被阻挡（强化坐标转换健壮性）
    /// </summary>
    private bool IsDirectionBlockedByGrid(Vector3 direction)
    {
        if (direction == Vector3.zero || !_isGridReady) return false;

        // 计算下一步的世界坐标
        Vector3 nextWorldPos = transform.position + direction * (autoMoveSpeed * Time.deltaTime * 1.5f);

        // 安全转换世界坐标到栅格坐标（增加空值/范围校验）
        Vector2Int nextGridPos;
        try
        {
            nextGridPos = _gridManager.WorldToGrid(nextWorldPos);
        }
        catch (System.Exception e)
        {
            Debug.LogError($"[{name}] 世界坐标转栅格坐标失败：{e.Message}", this);
            return false;
        }

        // 检查栅格是否可通行
        return !_gridManager.IsGridPassable(nextGridPos);
    }

    /// <summary>
    /// 限制移动方向在水域边界内
    /// </summary>
    private void LimitDirectionToWaterBoundary()
    {
        Vector3 futurePos = transform.position + _randomMoveDir * (autoMoveSpeed * Time.deltaTime * 2f);

        // 检测X轴边界（增加空值校验）
        if (futurePos.x < _gridManager.WaterMinX)
        {
            _randomMoveDir = new Vector3(Mathf.Abs(_randomMoveDir.x), 0, _randomMoveDir.z).normalized;
        }
        else if (futurePos.x > _gridManager.WaterMaxX)
        {
            _randomMoveDir = new Vector3(-Mathf.Abs(_randomMoveDir.x), 0, _randomMoveDir.z).normalized;
        }

        // 检测Z轴边界（增加空值校验）
        if (futurePos.z < _gridManager.WaterMinZ)
        {
            _randomMoveDir = new Vector3(_randomMoveDir.x, 0, Mathf.Abs(_randomMoveDir.z)).normalized;
        }
        else if (futurePos.z > _gridManager.WaterMaxZ)
        {
            _randomMoveDir = new Vector3(_randomMoveDir.x, 0, -Mathf.Abs(_randomMoveDir.z)).normalized;
        }
    }

    /// <summary>
    /// 限制手动移动的位置在水域边界内
    /// </summary>
    private void LimitPositionToWaterBoundary(ref float xMove, ref float zMove, Vector3 currentPos)
    {
        float futureX = currentPos.x + xMove;
        float futureZ = currentPos.z + zMove;

        // 修正X轴位移（增加边界值校验）
        if (futureX < _gridManager.WaterMinX)
        {
            xMove = Mathf.Max(_gridManager.WaterMinX - currentPos.x, -usvMoveSpeed * Time.deltaTime);
        }
        else if (futureX > _gridManager.WaterMaxX)
        {
            xMove = Mathf.Min(_gridManager.WaterMaxX - currentPos.x, usvMoveSpeed * Time.deltaTime);
        }

        // 修正Z轴位移（增加边界值校验）
        if (futureZ < _gridManager.WaterMinZ)
        {
            zMove = Mathf.Max(_gridManager.WaterMinZ - currentPos.z, -usvMoveSpeed * Time.deltaTime);
        }
        else if (futureZ > _gridManager.WaterMaxZ)
        {
            zMove = Mathf.Min(_gridManager.WaterMaxZ - currentPos.z, usvMoveSpeed * Time.deltaTime);
        }
    }

    /// <summary>
    /// 安全更新无人船位置（防止异常值）
    /// </summary>
    private void UpdateUSVPosition(Vector3 currentPos, float xMove, float zMove)
    {
        // 限制位移值，避免NaN/无穷大导致异常
        float newX = Mathf.Clamp(currentPos.x + xMove, -1000f, 1000f);
        float newZ = Mathf.Clamp(currentPos.z + zMove, -1000f, 1000f);

        // 最终边界兜底（即使栅格未就绪也限制）
        if (enableWaterBoundaryLimit && _isGridReady)
        {
            newX = Mathf.Clamp(newX, _gridManager.WaterMinX, _gridManager.WaterMaxX);
            newZ = Mathf.Clamp(newZ, _gridManager.WaterMinZ, _gridManager.WaterMaxZ);
        }

        // 固定Y轴高度，更新位置
        transform.position = new Vector3(newX, waterHeightOffset, newZ);
    }

    /// <summary>
    /// Gizmos绘制：辅助查看水域边界和障碍检测
    /// </summary>
    void OnDrawGizmosSelected()
    {
        // 绘制水域边界（如果GridManager存在）
        if (_gridManager != null && enableWaterBoundaryLimit)
        {
            Gizmos.color = new Color(0, 1, 0, 0.2f);
            Vector3 center = new Vector3(
                (_gridManager.WaterMinX + _gridManager.WaterMaxX) / 2,
                waterHeightOffset,
                (_gridManager.WaterMinZ + _gridManager.WaterMaxZ) / 2
            );
            Vector3 size = new Vector3(
                _gridManager.WaterMaxX - _gridManager.WaterMinX,
                0.1f,
                _gridManager.WaterMaxZ - _gridManager.WaterMinZ
            );
            Gizmos.DrawWireCube(center, size);
        }

        // 绘制自动移动方向
        if (enableObstacleAvoid)
        {
            Gizmos.color = Color.blue;
            Gizmos.DrawRay(transform.position, _randomMoveDir * 2f);
            Gizmos.DrawWireSphere(transform.position + _randomMoveDir * 2f, 0.2f);
        }
    }
}