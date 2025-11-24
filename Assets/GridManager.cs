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
    [Header("水域挂载配置（二选一）")]
    [Tooltip("是否自动读取水域实际尺寸（优先Collider，再读Mesh）")]
    public bool autoReadWaterSize = true; // 自动读取开关
    [Tooltip("手动输入水域X轴长度（米），自动读取关闭时生效")]
    public float manualWaterSizeX = 100f;
    [Tooltip("手动输入水域Z轴长度（米），自动读取关闭时生效")]
    public float manualWaterSizeZ = 80f;

    [Header("栅格基础配置")]
    public float 栅格尺寸 = 1f;
    public Transform 水域平面; // 必须挂载水域GameObject（定位+读取用）
    public LayerMask obstacleLayer;
    public Vector3 栅格原点;
    public int 栅格宽度;
    public int 栅格高度;

    [Header("初始化性能优化")]
    public int 每帧初始化数量 = 100;

    [Header("障碍物检测配置")]
    public float obstacleCheckRadius = 0.5f;
    public float radiusOffset = 0f;

    [Header("Gizmos显示设置")]
    public float 栅格线高度 = 0.1f;
    public float 障碍物显示高度 = 0.2f;
    public Color 栅格线颜色 = new Color(0.5f, 0.5f, 0.5f, 0.1f);
    public Color 障碍物颜色 = new Color(1f, 0f, 0f, 0.7f);

    [Header("初始化超时保护")]
    public float initTimeout = 10f;

    public int gridWidth => 栅格宽度;
    public int gridHeight => 栅格高度;

    private Node[,] 栅格地图;
    private int 初始化索引 = 0;
    private bool isInitializing = false;
    private bool isGridReady = false;
    private Collider[] 碰撞检测结果 = new Collider[1];
    private Vector2 水域大小缓存;
    private float 栅格半尺寸;
    private float initTimer = 0f;

    void Start()
    {
        if (水域平面 == null)
        {
            Debug.LogError("GridManager：未赋值水域平面！");
            return;
        }

        栅格半尺寸 = 栅格尺寸 / 2f;
        计算水域大小(); // 核心优化：调用新的计算逻辑

        // 核心修改1：取消Mathf.CeilToInt，用Mathf.RoundToInt精准匹配，避免向上取整导致超出
        栅格宽度 = Mathf.RoundToInt(水域大小缓存.x / 栅格尺寸);
        栅格高度 = Mathf.RoundToInt(水域大小缓存.y / 栅格尺寸);

        // 核心修改2：强制限制栅格范围不超过水域尺寸（双重保险）
        栅格宽度 = Mathf.Clamp(栅格宽度, 10, Mathf.RoundToInt(水域大小缓存.x / 栅格尺寸));
        栅格高度 = Mathf.Clamp(栅格高度, 10, Mathf.RoundToInt(水域大小缓存.y / 栅格尺寸));

        // 栅格原点对齐水域中心（优化Y轴贴合水面）
        栅格原点 = 水域平面.position - new Vector3(水域大小缓存.x / 2, 0, 水域大小缓存.y / 2);
        栅格原点.y = 水域平面.position.y;

        栅格地图 = new Node[栅格宽度, 栅格高度];
        isInitializing = true;
        初始化索引 = 0;
        initTimer = 0f;
        Debug.Log($"GridManager：开始分帧初始化，水域尺寸：{水域大小缓存.x}x{水域大小缓存.y}，栅格参数：{栅格宽度}x{栅格高度}，每帧处理{每帧初始化数量}个节点");
    }
    void Update()
    {
        if (isInitializing)
        {
            initTimer += Time.deltaTime;
            if (initTimer > initTimeout)
            {
                isInitializing = false;
                isGridReady = true;
                标记障碍物();
                Debug.LogError($"GridManager：初始化超时（{initTimeout}秒），强制标记为就绪！");
                initTimer = 0f;
                return;
            }
        }

        if (isInitializing)
        {
            int 总节点数 = 栅格宽度 * 栅格高度;
            int 本次结束索引 = Mathf.Min(初始化索引 + 每帧初始化数量, 总节点数);

            while (初始化索引 < 本次结束索引)
            {
                int x = 初始化索引 / 栅格高度;
                int z = 初始化索引 % 栅格高度;

                Vector3 节点位置 = 栅格原点 + new Vector3(
                    x * 栅格尺寸 + 栅格半尺寸,
                    水域平面.position.y,
                    z * 栅格尺寸 + 栅格半尺寸
                );

                栅格地图[x, z] = new Node(true, 节点位置, x, z);
                初始化索引++;
            }

            if (初始化索引 >= 总节点数)
            {
                isInitializing = false;
                isGridReady = true;
                标记障碍物();
                Debug.Log($"GridManager：分帧初始化完成，共{总节点数}个节点，已自动标记障碍物");
                initTimer = 0f;
            }
        }
    }

    public bool IsValidGridPosition(Vector2Int gridPos)
    {
        return gridPos.x >= 0 && gridPos.x < 栅格宽度 && gridPos.y >= 0 && gridPos.y < 栅格高度;
    }

    public bool IsGridReady()
    {
        return isGridReady;
    }

    public void 标记障碍物(Camera 主相机 = null)
    {
        if (栅格地图 == null)
        {
            Debug.LogWarning("GridManager：标记障碍物失败，栅格地图未初始化！");
            return;
        }

        float 实际检测半径 = 栅格半尺寸 + obstacleCheckRadius + radiusOffset;
        int 总节点数 = 栅格宽度 * 栅格高度;
        int 障碍物数量 = 0;

        for (int i = 0; i < 总节点数; i += 8)
        {
            for (int j = 0; j < 8 && (i + j) < 总节点数; j++)
            {
                int 索引 = i + j;
                int x = 索引 / 栅格高度;
                int z = 索引 % 栅格高度;

                Node 节点 = 栅格地图[x, z];
                int 碰撞数量 = Physics.OverlapSphereNonAlloc(
                    节点.worldPosition,
                    实际检测半径,
                    碰撞检测结果,
                    obstacleLayer,
                    QueryTriggerInteraction.Ignore
                );

                if (碰撞数量 > 0)
                {
                    节点.walkable = false;
                    障碍物数量++;
                }
                else
                {
                    节点.walkable = true;
                }

                栅格地图[x, z] = 节点;
            }
        }

        Debug.Log($"GridManager：障碍物标记完成，共检测{总节点数}个节点，发现{障碍物数量}个障碍物节点");
    }

    public void 重置栅格()
    {
        重新初始化栅格数据();
        标记障碍物();
        Debug.Log("GridManager：栅格已重置并重新标记障碍物");
    }

    public void 初始化栅格()
    {
        重新初始化栅格数据();
        标记障碍物();
        Debug.Log("GridManager：栅格初始化完成（适配SpawnManager调用）");
    }

    private void 重新初始化栅格数据()
    {
        if (水域平面 == null)
        {
            Debug.LogError("GridManager：重新初始化失败，未赋值水域平面！");
            return;
        }

        计算水域大小(); // 重新计算水域大小
                  // 核心修改3：同步取消向上取整，精准匹配
        int 新宽度 = Mathf.RoundToInt(水域大小缓存.x / 栅格尺寸);
        int 新高度 = Mathf.RoundToInt(水域大小缓存.y / 栅格尺寸);

        // 强制限制新栅格范围不超水域
        新宽度 = Mathf.Clamp(新宽度, 10, Mathf.RoundToInt(水域大小缓存.x / 栅格尺寸));
        新高度 = Mathf.Clamp(新高度, 10, Mathf.RoundToInt(水域大小缓存.y / 栅格尺寸));

        if (栅格地图 == null || 栅格地图.GetLength(0) != 新宽度 || 栅格地图.GetLength(1) != 新高度)
        {
            栅格地图 = new Node[新宽度, 新高度];
        }

        栅格宽度 = 新宽度;
        栅格高度 = 新高度;
        栅格原点 = 水域平面.position - new Vector3(水域大小缓存.x / 2, 0, 水域大小缓存.y / 2);
        栅格原点.y = 水域平面.position.y; // 贴合水面
        栅格半尺寸 = 栅格尺寸 / 2f;

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
        initTimer = 0f;
        Debug.Log($"GridManager：重新初始化完成，水域尺寸：{水域大小缓存.x}x{水域大小缓存.y}，栅格参数：{栅格宽度}x{栅格高度}");
    }
    /// <summary>
    /// 核心优化：水域大小计算逻辑（自动读取/手动输入二选一）
    /// </summary>
    /// <summary>
    /// 核心优化：水域大小计算逻辑（自动读取/手动输入二选一）+ 日志防抖
    /// </summary>
    private void 计算水域大小()
    {
        if (水域平面 == null)
        {
            水域大小缓存 = Vector2.zero;
            Debug.LogError("GridManager：计算水域大小失败，未赋值水域平面！");
            return;
        }

        Vector2 newWaterSize; // 临时存储新计算的尺寸

        if (autoReadWaterSize)
        {
            // 自动读取模式：优先读Collider，再读Mesh，最后兜底
            newWaterSize = GetAutoWaterSize(水域平面.gameObject);
        }
        else
        {
            // 手动输入模式：直接使用设置的数值
            newWaterSize = new Vector2(manualWaterSizeX, manualWaterSizeZ);
        }

        // 核心修改：仅当新尺寸与旧尺寸差异超过0.01f（避免浮点误差）时，才更新并打印日志
        if (Mathf.Abs(newWaterSize.x - 水域大小缓存.x) > 0.01f || Mathf.Abs(newWaterSize.y - 水域大小缓存.y) > 0.01f)
        {
            水域大小缓存 = newWaterSize;
            Debug.Log($"GridManager：计算水域大小完成，尺寸：{水域大小缓存.x}x{水域大小缓存.y}（自动读取：{autoReadWaterSize}）");
        }
    }

    /// <summary>
    /// 新增：自动获取水域实际尺寸
    /// </summary>
    private Vector2 GetAutoWaterSize(GameObject waterObj)
    {
        // 方式1：读取Collider（推荐，水域通常有Collider用于碰撞检测）
        Collider waterCollider = waterObj.GetComponent<Collider>();
        if (waterCollider != null)
        {
            return new Vector2(
                Mathf.Abs(waterCollider.bounds.size.x),
                Mathf.Abs(waterCollider.bounds.size.z)
            );
        }

        // 方式2：读取MeshFilter（无Collider时使用，如纯可视化水域）
        MeshFilter waterMesh = waterObj.GetComponent<MeshFilter>();
        if (waterMesh != null && waterMesh.mesh != null)
        {
            return new Vector2(
                Mathf.Abs(waterMesh.mesh.bounds.size.x * waterObj.transform.lossyScale.x),
                Mathf.Abs(waterMesh.mesh.bounds.size.z * waterObj.transform.lossyScale.z)
            );
        }

        // 兜底：自动读取失败时，使用手动输入的默认值
        Debug.LogWarning("GridManager：自动读取水域尺寸失败，使用手动输入默认值");
        return new Vector2(manualWaterSizeX, manualWaterSizeZ);
    }

    public Vector2Int 世界转栅格(Vector3 世界坐标)
    {
        if (!isGridReady)
        {
            Debug.LogWarning("GridManager：世界转栅格失败，栅格未初始化完成！");
            return Vector2Int.zero;
        }

        Vector3 偏移 = 世界坐标 - 栅格原点;
        int x = Mathf.FloorToInt(偏移.x / 栅格尺寸);
        int z = Mathf.FloorToInt(偏移.z / 栅格尺寸);
        x = Mathf.Clamp(x, 0, 栅格宽度 - 1);
        z = Mathf.Clamp(z, 0, 栅格高度 - 1);
        return new Vector2Int(x, z);
    }

    public Vector3 栅格转世界(Vector2Int 栅格坐标)
    {
        if (!isGridReady)
        {
            Debug.LogWarning("GridManager：栅格转世界失败，栅格未初始化完成！");
            return Vector3.zero;
        }

        int x = Mathf.Clamp(栅格坐标.x, 0, 栅格宽度 - 1);
        int z = Mathf.Clamp(栅格坐标.y, 0, 栅格高度 - 1);
        return 栅格原点 + new Vector3(
            x * 栅格尺寸 + 栅格半尺寸,
            水域平面.position.y,
            z * 栅格尺寸 + 栅格半尺寸
        );
    }

    public bool 栅格是否可通行(Vector2Int 栅格坐标)
    {
        if (!isGridReady || 栅格地图 == null)
        {
            Debug.LogWarning("GridManager：检查栅格可通行性失败，栅格未初始化完成！");
            return false;
        }
        if (!IsValidGridPosition(栅格坐标))
        {
            Debug.LogWarning($"GridManager：栅格坐标{栅格坐标}无效，不可通行！");
            return false;
        }
        return 栅格地图[栅格坐标.x, 栅格坐标.y].walkable;
    }

    [ContextMenu("强制刷新栅格和障碍物")]
    public void 强制刷新栅格()
    {
        重新初始化栅格数据();
        标记障碍物();
        Debug.Log("GridManager：已强制刷新栅格和障碍物标记");
    }

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

    private void OnDrawGizmos()
    {
        if (水域平面 == null)
        {
            Gizmos.color = Color.yellow;
            Gizmos.DrawWireCube(Vector3.zero, new Vector3(10, 0.1f, 10));
            return;
        }

        计算水域大小(); // 实时更新水域Gizmos
                  // 绘制水域范围（灰色半透明）
        Gizmos.color = new Color(0.3f, 0.3f, 0.3f, 0.1f);
        Gizmos.DrawWireCube(水域平面.position, new Vector3(水域大小缓存.x, 0.1f, 水域大小缓存.y));

        if (!isGridReady || 栅格地图 == null) return;

        // 绘制栅格线（仅在水域范围内）
        Gizmos.color = 栅格线颜色;
        float gridMaxX = 栅格原点.x + 栅格宽度 * 栅格尺寸;
        float gridMaxZ = 栅格原点.z + 栅格高度 * 栅格尺寸;
        float waterMinX = 水域平面.position.x - 水域大小缓存.x / 2;
        float waterMaxX = 水域平面.position.x + 水域大小缓存.x / 2;
        float waterMinZ = 水域平面.position.z - 水域大小缓存.y / 2;
        float waterMaxZ = 水域平面.position.z + 水域大小缓存.y / 2;

        // 只绘制水域内的X方向栅格线
        for (int x = 0; x <= 栅格宽度; x++)
        {
            float lineX = 栅格原点.x + x * 栅格尺寸;
            // 仅当线在水域X范围内才绘制
            if (lineX >= waterMinX && lineX <= waterMaxX)
            {
                Vector3 起点 = new Vector3(lineX, 栅格线高度, Mathf.Max(栅格原点.z, waterMinZ));
                Vector3 终点 = new Vector3(lineX, 栅格线高度, Mathf.Min(gridMaxZ, waterMaxZ));
                Gizmos.DrawLine(起点, 终点);
            }
        }

        // 只绘制水域内的Z方向栅格线
        for (int z = 0; z <= 栅格高度; z++)
        {
            float lineZ = 栅格原点.z + z * 栅格尺寸;
            // 仅当线在水域Z范围内才绘制
            if (lineZ >= waterMinZ && lineZ <= waterMaxZ)
            {
                Vector3 起点 = new Vector3(Mathf.Max(栅格原点.x, waterMinX), 栅格线高度, lineZ);
                Vector3 终点 = new Vector3(Mathf.Min(gridMaxX, waterMaxX), 栅格线高度, lineZ);
                Gizmos.DrawLine(起点, 终点);
            }
        }

        // 绘制障碍物（仅水域内）
        Gizmos.color = 障碍物颜色;
        for (int x = 0; x < 栅格宽度; x++)
        {
            for (int z = 0; z < 栅格高度; z++)
            {
                if (!栅格地图[x, z].walkable)
                {
                    Vector3 障碍物中心 = 栅格转世界(new Vector2Int(x, z));
                    // 仅绘制水域内的障碍物
                    if (障碍物中心.x >= waterMinX && 障碍物中心.x <= waterMaxX &&
                        障碍物中心.z >= waterMinZ && 障碍物中心.z <= waterMaxZ)
                    {
                        障碍物中心.y = 障碍物显示高度;
                        Gizmos.DrawCube(障碍物中心, new Vector3(栅格尺寸 * 0.8f, 0.2f, 栅格尺寸 * 0.8f));
                    }
                }
            }
        }

        // 绘制栅格边界（贴合水域）
        Gizmos.color = new Color(0.5f, 0.5f, 0.5f, 0.05f);
        Gizmos.DrawWireCube(
            栅格原点 + new Vector3(栅格宽度 * 栅格尺寸 / 2, 栅格线高度, 栅格高度 * 栅格尺寸 / 2),
            new Vector3(栅格宽度 * 栅格尺寸, 0.1f, 栅格高度 * 栅格尺寸)
        );

        // 绘制路径（仅水域内）
        ImprovedAStar pathfinder = FindFirstObjectByType<ImprovedAStar>();
        if (pathfinder != null && pathfinder.path != null && pathfinder.path.Count > 1)
        {
            Gizmos.color = Color.cyan;
            for (int i = 0; i < pathfinder.path.Count - 1; i++)
            {
                Vector3 start = 栅格转世界(pathfinder.path[i]);
                Vector3 end = 栅格转世界(pathfinder.path[i + 1]);
                // 仅绘制水域内的路径段
                if (start.x >= waterMinX && start.x <= waterMaxX && start.z >= waterMinZ && start.z <= waterMaxZ &&
                    end.x >= waterMinX && end.x <= waterMaxX && end.z >= waterMinZ && end.z <= waterMaxZ)
                {
                    start.y = 0.1f;
                    end.y = 0.1f;
                    Gizmos.DrawLine(start, end);
                    Gizmos.DrawSphere(start, 栅格尺寸 * 0.3f);
                }
            }
            Vector3 finalPoint = 栅格转世界(pathfinder.path[pathfinder.path.Count - 1]);
            if (finalPoint.x >= waterMinX && finalPoint.x <= waterMaxX && finalPoint.z >= waterMinZ && finalPoint.z <= waterMaxZ)
            {
                finalPoint.y = 0.1f;
                Gizmos.DrawSphere(finalPoint, 栅格尺寸 * 0.4f);
            }
        }
    }
}