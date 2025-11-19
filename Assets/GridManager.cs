using System.Collections.Generic;
using UnityEngine;
using UnityEditor;

internal struct Node
{
    public bool walkable;
    public Vector3 worldPosition;
    public int gridX;
    public int gridY;

    public Node(bool _walkable, Vector3 _worldPos, int _gridX, int _gridY)
    {
        walkable = _walkable;
        worldPosition = _worldPos;
        gridX = _gridX;
        gridY = _gridY;
    }
}

public class GridManager : MonoBehaviour
{
    [Header("栅格基础配置")]
    public float 栅格尺寸 = 1f;
    public Transform 水域平面;
    public LayerMask obstacleLayer;
    public Vector3 栅格原点;
    public int 栅格宽度; // 中文变量（与Agent的gridWidth对应）
    public int 栅格高度; // 中文变量（与Agent的gridHeight对应）

    [Header("初始化性能优化")]
    [Tooltip("每帧最大初始化节点数（值越小越流畅，默认100）")]
    public int 每帧初始化数量 = 100; // 优化：从200降至100，降低每帧开销

    [Header("障碍物检测配置")]
    [Tooltip("障碍物检测半径（值越小越精准，默认0.5f）")]
    public float obstacleCheckRadius = 0.5f;
    [Tooltip("检测半径偏移量（扩展/收缩检测范围，默认0f）")]
    public float radiusOffset = 0f;

    [Header("Gizmos显示设置（降低明显度）")]
    [Tooltip("栅格线高度（越低越不明显）")]
    public float 栅格线高度 = 0.1f;
    [Tooltip("障碍物显示高度")]
    public float 障碍物显示高度 = 0.2f;
    [Tooltip("栅格线颜色（alpha值越低越透明）")]
    public Color 栅格线颜色 = new Color(0.5f, 0.5f, 0.5f, 0.1f);
    [Tooltip("障碍物颜色")]
    public Color 障碍物颜色 = new Color(1f, 0f, 0f, 0.7f);

    // 适配Agent的英文属性调用（无需修改Agent逻辑）
    public int gridWidth => 栅格宽度;
    public int gridHeight => 栅格高度;

    private Node[,] 栅格地图;
    private int 初始化索引 = 0;
    private bool isInitializing = false;
    private bool isGridReady = false;
    private Collider[] 碰撞检测结果 = new Collider[1]; // 复用数组，减少GC
    private Vector2 水域大小缓存;
    private float 栅格半尺寸;

    void Start()
    {
        if (水域平面 == null)
        {
            Debug.LogError("GridManager：未赋值水域平面！");
            return;
        }

        栅格半尺寸 = 栅格尺寸 / 2f;
        计算水域大小();

        // 初始化栅格宽高（最小10x10，避免过小）
        栅格宽度 = Mathf.Max(10, Mathf.CeilToInt(水域大小缓存.x / 栅格尺寸));
        栅格高度 = Mathf.Max(10, Mathf.CeilToInt(水域大小缓存.y / 栅格尺寸));

        // 计算栅格原点（水域中心对齐）
        栅格原点 = 水域平面.position - new Vector3(水域大小缓存.x / 2, 0, 水域大小缓存.y / 2);
        栅格地图 = new Node[栅格宽度, 栅格高度];

        // 启动分帧初始化
        isInitializing = true;
        初始化索引 = 0;
        Debug.Log($"GridManager：开始分帧初始化，栅格参数：{栅格宽度}x{栅格高度}，每帧处理{每帧初始化数量}个节点");
    }

    void Update()
    {
        // 分帧处理栅格初始化，避免单帧卡顿
        if (isInitializing)
        {
            int 总节点数 = 栅格宽度 * 栅格高度;
            int 本次结束索引 = Mathf.Min(初始化索引 + 每帧初始化数量, 总节点数);

            // 批量初始化节点（减少循环内重复计算）
            while (初始化索引 < 本次结束索引)
            {
                int x = 初始化索引 / 栅格高度;
                int z = 初始化索引 % 栅格高度;

                // 计算节点世界坐标
                Vector3 节点位置 = 栅格原点 + new Vector3(
                    x * 栅格尺寸 + 栅格半尺寸,
                    水域平面.position.y,
                    z * 栅格尺寸 + 栅格半尺寸
                );

                // 初始化可通行节点（障碍物标记后续统一处理）
                栅格地图[x, z] = new Node(true, 节点位置, x, z);
                初始化索引++;
            }

            // 初始化完成后标记障碍物
            if (初始化索引 >= 总节点数)
            {
                isInitializing = false;
                isGridReady = true;
                标记障碍物();
                Debug.Log($"GridManager：分帧初始化完成，共{总节点数}个节点，已自动标记障碍物");
            }
        }
    }

    /// <summary>
    /// 验证栅格坐标是否有效（防止数组越界）
    /// </summary>
    public bool IsValidGridPosition(Vector2Int gridPos)
    {
        return gridPos.x >= 0 && gridPos.x < 栅格宽度 && gridPos.y >= 0 && gridPos.y < 栅格高度;
    }

    /// <summary>
    /// 检查栅格是否初始化完成
    /// </summary>
    public bool IsGridReady()
    {
        return isGridReady;
    }

    /// <summary>
    /// 批量标记障碍物（优化检测效率，减少GC）
    /// </summary>
    public void 标记障碍物(Camera 主相机 = null)
    {
        if (栅格地图 == null) return;

        float 实际检测半径 = 栅格半尺寸 + obstacleCheckRadius + radiusOffset;
        int 总节点数 = 栅格宽度 * 栅格高度;

        // 分批次检测（每批8个节点，平衡效率与开销）
        for (int i = 0; i < 总节点数; i += 8)
        {
            for (int j = 0; j < 8 && (i + j) < 总节点数; j++)
            {
                int 索引 = i + j;
                int x = 索引 / 栅格高度;
                int z = 索引 % 栅格高度;

                Node 节点 = 栅格地图[x, z];
                // 非分配式碰撞检测（复用数组，避免频繁创建）
                int 碰撞数量 = Physics.OverlapSphereNonAlloc(
                    节点.worldPosition,
                    实际检测半径,
                    碰撞检测结果,
                    obstacleLayer,
                    QueryTriggerInteraction.Ignore
                );
                节点.walkable = 碰撞数量 == 0;
                栅格地图[x, z] = 节点;
            }
        }
    }

    /// <summary>
    /// 重置栅格数据并重新标记障碍物
    /// </summary>
    public void 重置栅格()
    {
        重新初始化栅格数据();
        标记障碍物();
        Debug.Log("GridManager：栅格已重置并重新标记障碍物");
    }

    /// <summary>
    /// 适配RandomSpawnManager的初始化调用
    /// </summary>
    public void 初始化栅格()
    {
        重新初始化栅格数据();
        标记障碍物();
        Debug.Log("GridManager：栅格初始化完成（适配SpawnManager调用）");
    }

    /// <summary>
    /// 重新初始化栅格基础数据（全量同步，非分帧）
    /// </summary>
    private void 重新初始化栅格数据()
    {
        if (水域平面 == null) return;

        计算水域大小();
        int 新宽度 = Mathf.CeilToInt(水域大小缓存.x / 栅格尺寸);
        int 新高度 = Mathf.CeilToInt(水域大小缓存.y / 栅格尺寸);

        // 仅当尺寸变化时重建数组（减少内存分配）
        if (栅格地图 == null || 栅格地图.GetLength(0) != 新宽度 || 栅格地图.GetLength(1) != 新高度)
        {
            栅格地图 = new Node[新宽度, 新高度];
        }

        栅格宽度 = 新宽度;
        栅格高度 = 新高度;
        栅格原点 = 水域平面.position - new Vector3(水域大小缓存.x / 2, 0, 水域大小缓存.y / 2);
        栅格半尺寸 = 栅格尺寸 / 2f;

        // 全量初始化节点（重置时使用，非分帧）
        int 总节点数 = 栅格宽度 * 栅格高度;
        for (int i = 0; i < 总节点数; i++)
        {
            int x = i / 栅格高度;
            int z = i % 栅格高度;
            Vector3 节点位置 = 栅格原点 + new Vector3(
                x * 栅格尺寸 + 栅格半尺寸,
                水域平面.position.y,
                z * 栅格尺寸 + 栅格半尺寸
            );
            栅格地图[x, z] = new Node(true, 节点位置, x, z);
        }

        isGridReady = true;
    }

    /// <summary>
    /// 计算水域实际大小（基于缩放）
    /// </summary>
    private void 计算水域大小()
    {
        水域大小缓存 = new Vector2(
            水域平面.lossyScale.x * 10, // 假设水域原始尺寸为10x10（可根据实际模型调整）
            水域平面.lossyScale.z * 10
        );
    }

    /// <summary>
    /// 世界坐标转栅格坐标
    /// </summary>
    public Vector2Int 世界转栅格(Vector3 世界坐标)
    {
        if (!isGridReady) return Vector2Int.zero;

        Vector3 偏移 = 世界坐标 - 栅格原点;
        int x = Mathf.FloorToInt(偏移.x / 栅格尺寸);
        int z = Mathf.FloorToInt(偏移.z / 栅格尺寸);
        // 限制坐标在有效范围内
        x = Mathf.Clamp(x, 0, 栅格宽度 - 1);
        z = Mathf.Clamp(z, 0, 栅格高度 - 1);
        return new Vector2Int(x, z);
    }

    /// <summary>
    /// 栅格坐标转世界坐标
    /// </summary>
    public Vector3 栅格转世界(Vector2Int 栅格坐标)
    {
        if (!isGridReady) return Vector3.zero;

        int x = Mathf.Clamp(栅格坐标.x, 0, 栅格宽度 - 1);
        int z = Mathf.Clamp(栅格坐标.y, 0, 栅格高度 - 1);
        return 栅格原点 + new Vector3(
            x * 栅格尺寸 + 栅格半尺寸,
            水域平面.position.y,
            z * 栅格尺寸 + 栅格半尺寸
        );
    }

    /// <summary>
    /// 检查栅格是否可通行
    /// </summary>
    public bool 栅格是否可通行(Vector2Int 栅格坐标)
    {
        if (!isGridReady || 栅格地图 == null) return false;
        if (!IsValidGridPosition(栅格坐标)) return false;
        return 栅格地图[栅格坐标.x, 栅格坐标.y].walkable;
    }

    /// <summary>
    /// 编辑器菜单：强制刷新栅格和障碍物（右键调用）
    /// </summary>
    [ContextMenu("强制刷新栅格和障碍物")]
    public void 强制刷新栅格()
    {
        重新初始化栅格数据();
        标记障碍物();
        Debug.Log("GridManager：已强制刷新栅格和障碍物标记");
    }

    /// <summary>
    /// 编辑器菜单：定位到栅格中心（右键调用）
    /// </summary>
    [ContextMenu("定位到栅格原点")]
    public void 定位到栅格原点()
    {
        if (SceneView.lastActiveSceneView == null)
        {
            Debug.LogWarning("GridManager：未找到SceneView，无法定位");
            return;
        }

        Bounds 栅格范围 = new Bounds(
            栅格原点 + new Vector3(栅格宽度 * 栅格尺寸 / 2, 0, 栅格高度 * 栅格尺寸 / 2),
            new Vector3(栅格宽度 * 栅格尺寸, 10, 栅格高度 * 栅格尺寸)
        );
        SceneView.lastActiveSceneView.Frame(栅格范围);
        Debug.Log($"GridManager：已定位到栅格中心，坐标：{栅格范围.center}");
    }

    /// <summary>
    /// Gizmos绘制（场景视图可视化）
    /// </summary>
    private void OnDrawGizmos()
    {
        // 绘制水域范围提示（未赋值水域平面时）
        if (水域平面 == null)
        {
            Gizmos.color = Color.yellow;
            Gizmos.DrawWireCube(Vector3.zero, new Vector3(10, 0.1f, 10));
            return;
        }

        // 绘制水域边界
        计算水域大小();
        Gizmos.color = new Color(0.3f, 0.3f, 0.3f, 0.1f);
        Gizmos.DrawWireCube(水域平面.position, new Vector3(水域大小缓存.x, 0.1f, 水域大小缓存.y));

        // 栅格未初始化时不绘制细节
        if (!isGridReady || 栅格地图 == null) return;

        // 绘制栅格线
        Gizmos.color = 栅格线颜色;
        for (int x = 0; x <= 栅格宽度; x++)
        {
            Vector3 起点 = 栅格原点 + new Vector3(x * 栅格尺寸, 栅格线高度, 0);
            Vector3 终点 = 栅格原点 + new Vector3(x * 栅格尺寸, 栅格线高度, 栅格高度 * 栅格尺寸);
            Gizmos.DrawLine(起点, 终点);
        }
        for (int z = 0; z <= 栅格高度; z++)
        {
            Vector3 起点 = 栅格原点 + new Vector3(0, 栅格线高度, z * 栅格尺寸);
            Vector3 终点 = 栅格原点 + new Vector3(栅格宽度 * 栅格尺寸, 栅格线高度, z * 栅格尺寸);
            Gizmos.DrawLine(起点, 终点);
        }

        // 绘制障碍物
        Gizmos.color = 障碍物颜色;
        for (int x = 0; x < 栅格宽度; x++)
        {
            for (int z = 0; z < 栅格高度; z++)
            {
                if (!栅格地图[x, z].walkable)
                {
                    Vector3 障碍物中心 = 栅格转世界(new Vector2Int(x, z));
                    障碍物中心.y = 障碍物显示高度;
                    Gizmos.DrawCube(障碍物中心, new Vector3(栅格尺寸 * 0.8f, 0.2f, 栅格尺寸 * 0.8f));
                }
            }
        }

        // 绘制栅格整体边界
        Gizmos.color = new Color(0.5f, 0.5f, 0.5f, 0.05f);
        Gizmos.DrawWireCube(
            栅格原点 + new Vector3(栅格宽度 * 栅格尺寸 / 2, 栅格线高度, 栅格高度 * 栅格尺寸 / 2),
            new Vector3(栅格宽度 * 栅格尺寸, 0.1f, 栅格高度 * 栅格尺寸)
        );
        // 绘制A*规划的路径
        ImprovedAStar pathfinder = FindFirstObjectByType<ImprovedAStar>();
        if (pathfinder != null && pathfinder.path != null && pathfinder.path.Count > 1)
        {
            Gizmos.color = Color.cyan; // 路径颜色
            for (int i = 0; i < pathfinder.path.Count - 1; i++)
            {
                Vector3 start = 栅格转世界(pathfinder.path[i]);
                Vector3 end = 栅格转世界(pathfinder.path[i + 1]);
                start.y = 0.1f; // 稍微抬高显示
                end.y = 0.1f;
                Gizmos.DrawLine(start, end);

                // 绘制路径点
                Gizmos.DrawSphere(start, 栅格尺寸 * 0.3f);
            }
            // 绘制终点

            Gizmos.DrawSphere(栅格转世界(pathfinder.path[pathfinder.path.Count - 1]), 栅格尺寸 * 0.4f);
        }

    }
}