using System.Collections.Generic;
using UnityEngine;
using UnityEditor; // 需添加此命名空间以使用SceneView

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
    public float 栅格尺寸 = 1f;
    public Transform 水域平面;
    public LayerMask obstacleLayer;
    public Vector3 栅格原点;
    public int 栅格宽度;
    public int 栅格高度;
    private Node[,] 栅格地图;
    private int 初始化索引 = 0;
    private bool isInitializing = false;
    private bool isGridReady = false;
    private Collider[] 碰撞检测结果 = new Collider[1];
    private Vector2 水域大小缓存;

    private float 栅格半尺寸;
    private int 每帧初始化数量 = 200;

    [Header("Gizmos显示设置")]
    public float 栅格线高度 = 0.5f;
    public float 障碍物显示高度 = 0.6f;
    public Color 栅格线颜色 = new Color(0.8f, 0.8f, 0.8f, 1f);
    public Color 障碍物颜色 = new Color(1f, 0f, 0f, 1f);

    void Start()
    {
        if (水域平面 == null)
        {
            Debug.LogError("GridManager未赋值水域平面！");
            return;
        }
        栅格半尺寸 = 栅格尺寸 / 2f;
        计算水域大小();

        栅格宽度 = Mathf.Max(10, Mathf.CeilToInt(水域大小缓存.x / 栅格尺寸));
        栅格高度 = Mathf.Max(10, Mathf.CeilToInt(水域大小缓存.y / 栅格尺寸));

        栅格原点 = 水域平面.position - new Vector3(水域大小缓存.x / 2, 0, 水域大小缓存.y / 2);
        栅格地图 = new Node[栅格宽度, 栅格高度];

        isInitializing = true;
        初始化索引 = 0;
        Debug.Log($"栅格参数：宽度={栅格宽度}，高度={栅格高度}，尺寸={栅格尺寸}");
    }

    void Update()
    {
        if (isInitializing)
        {
            int 总节点数 = 栅格宽度 * 栅格高度;
            int 结束索引 = Mathf.Min(初始化索引 + 每帧初始化数量, 总节点数);

            while (初始化索引 < 结束索引 - 1)
            {
                int x1 = 初始化索引 / 栅格高度;
                int z1 = 初始化索引 % 栅格高度;
                Vector3 pos1 = 栅格原点 + new Vector3(
                    x1 * 栅格尺寸 + 栅格半尺寸,
                    水域平面.position.y,
                    z1 * 栅格尺寸 + 栅格半尺寸
                );
                栅格地图[x1, z1] = new Node(true, pos1, x1, z1);

                初始化索引++;
                int x2 = 初始化索引 / 栅格高度;
                int z2 = 初始化索引 % 栅格高度;
                Vector3 pos2 = 栅格原点 + new Vector3(
                    x2 * 栅格尺寸 + 栅格半尺寸,
                    水域平面.position.y,
                    z2 * 栅格尺寸 + 栅格半尺寸
                );
                栅格地图[x2, z2] = new Node(true, pos2, x2, z2);

                初始化索引++;
            }

            if (初始化索引 < 结束索引)
            {
                int x = 初始化索引 / 栅格高度;
                int z = 初始化索引 % 栅格高度;
                Vector3 pos = 栅格原点 + new Vector3(
                    x * 栅格尺寸 + 栅格半尺寸,
                    水域平面.position.y,
                    z * 栅格尺寸 + 栅格半尺寸
                );
                栅格地图[x, z] = new Node(true, pos, x, z);
                初始化索引++;
            }

            if (初始化索引 >= 总节点数)
            {
                isInitializing = false;
                isGridReady = true;
                标记障碍物();
                Debug.Log($"栅格分帧初始化完成：{栅格宽度}x{栅格高度}");
            }
        }
    }

    public bool IsGridReady()
    {
        return isGridReady;
    }

    public void 标记障碍物(Camera 主相机 = null)
    {
        float 检测半径 = 栅格半尺寸 + 0.5f;
        int 起始X = 0, 结束X = 栅格宽度;
        int 起始Z = 0, 结束Z = 栅格高度;

        for (int x = 起始X; x < 结束X; x += 4)
        {
            int 实际结束X = Mathf.Min(x + 4, 结束X);
            for (int z = 起始Z; z < 结束Z; z++)
            {
                for (int xInner = x; xInner < 实际结束X; xInner++)
                {
                    Node 节点 = 栅格地图[xInner, z];
                    int 碰撞数量 = Physics.OverlapSphereNonAlloc(
                        节点.worldPosition,
                        检测半径,
                        碰撞检测结果,
                        obstacleLayer,
                        QueryTriggerInteraction.Ignore
                    );
                    节点.walkable = 碰撞数量 == 0;
                    栅格地图[xInner, z] = 节点;
                }
            }
        }
    }

    public void 重置栅格()
    {
        重新初始化栅格数据();
        标记障碍物();
        Debug.Log("栅格已重置，重新标记障碍物完成");
    }

    private void 重新初始化栅格数据()
    {
        if (水域平面 == null) return;

        计算水域大小();
        int 新宽度 = Mathf.CeilToInt(水域大小缓存.x / 栅格尺寸);
        int 新高度 = Mathf.CeilToInt(水域大小缓存.y / 栅格尺寸);

        if (栅格地图 == null || 栅格地图.GetLength(0) != 新宽度 || 栅格地图.GetLength(1) != 新高度)
        {
            栅格地图 = new Node[新宽度, 新高度];
        }

        栅格宽度 = 新宽度;
        栅格高度 = 新高度;
        栅格原点 = 水域平面.position - new Vector3(水域大小缓存.x / 2, 0, 水域大小缓存.y / 2);
        栅格半尺寸 = 栅格尺寸 / 2f;

        for (int i = 0; i < 栅格宽度 * 栅格高度; i++)
        {
            int x = i / 栅格高度;
            int z = i % 栅格高度;
            Vector3 节点世界位置 = 栅格原点 + new Vector3(
                x * 栅格尺寸 + 栅格半尺寸,
                水域平面.position.y,
                z * 栅格尺寸 + 栅格半尺寸
            );
            栅格地图[x, z] = new Node(true, 节点世界位置, x, z);
        }
        isGridReady = true;
    }

    private void 计算水域大小()
    {
        水域大小缓存 = new Vector2(
            水域平面.lossyScale.x * 10,
            水域平面.lossyScale.z * 10
        );
    }

    public Vector2Int 世界转栅格(Vector3 世界坐标)
    {
        Vector3 偏移 = 世界坐标 - 栅格原点;
        int x = Mathf.FloorToInt(偏移.x / 栅格尺寸);
        int z = Mathf.FloorToInt(偏移.z / 栅格尺寸);
        x = Mathf.Clamp(x, 0, 栅格宽度 - 1);
        z = Mathf.Clamp(z, 0, 栅格高度 - 1);
        return new Vector2Int(x, z);
    }

    public Vector3 栅格转世界(Vector2Int 栅格坐标)
    {
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
        if (栅格坐标.x < 0 || 栅格坐标.x >= 栅格宽度 || 栅格坐标.y < 0 || 栅格坐标.y >= 栅格高度)
            return false;
        return 栅格地图[栅格坐标.x, 栅格坐标.y].walkable;
    }

    // 手动刷新按钮
    [ContextMenu("强制刷新栅格和障碍物")]
    public void 强制刷新栅格()
    {
        重新初始化栅格数据();
        标记障碍物();
        Debug.Log("已强制刷新栅格和障碍物标记");
    }

    // 定位到栅格原点的方法（添加在这里）
    [ContextMenu("定位到栅格原点")]
    public void 定位到栅格原点()
    {
        // 聚焦到栅格区域
        if (SceneView.lastActiveSceneView != null)
        {
            Bounds 栅格范围 = new Bounds(
                栅格原点 + new Vector3(栅格宽度 * 栅格尺寸 / 2, 0, 栅格高度 * 栅格尺寸 / 2),
                new Vector3(栅格宽度 * 栅格尺寸, 10, 栅格高度 * 栅格尺寸)
            );
            SceneView.lastActiveSceneView.Frame(栅格范围);
            Debug.Log($"已定位到栅格中心：{栅格范围.center}，范围：{栅格宽度 * 栅格尺寸}x{栅格高度 * 栅格尺寸}");
        }
        else
        {
            Debug.LogWarning("未找到SceneView，无法定位");
        }
    }

    private void OnDrawGizmos()
    {
        Debug.Log("GridManager的Gizmos正在绘制栅格"); // 新增日志
        if (水域平面 == null)
        {
            Gizmos.color = Color.yellow;
            Gizmos.DrawWireCube(Vector3.zero, new Vector3(10, 0.1f, 10));
            return;
        }

        Gizmos.color = Color.white;
        Gizmos.DrawWireCube(水域平面.position, new Vector3(水域大小缓存.x, 0.1f, 水域大小缓存.y));

        if (栅格地图 == null) return;

        Gizmos.color = 栅格线颜色;
        for (int x = 0; x <= 栅格宽度; x++)
        {
            Vector3 起点 = 栅格原点 + new Vector3(x * 栅格尺寸, 栅格线高度, 0);
            Vector3 终点 = 栅格原点 + new Vector3(x * 栅格尺寸, 栅格线高度, 栅格高度 * 栅格尺寸);
            Gizmos.DrawLine(起点, 终点);
            Gizmos.DrawLine(起点 + Vector3.right * 0.05f, 终点 + Vector3.right * 0.05f);
        }
        for (int z = 0; z <= 栅格高度; z++)
        {
            Vector3 起点 = 栅格原点 + new Vector3(0, 栅格线高度, z * 栅格尺寸);
            Vector3 终点 = 栅格原点 + new Vector3(栅格宽度 * 栅格尺寸, 栅格线高度, z * 栅格尺寸);
            Gizmos.DrawLine(起点, 终点);
            Gizmos.DrawLine(起点 + Vector3.forward * 0.05f, 终点 + Vector3.forward * 0.05f);
        }

        Gizmos.color = 障碍物颜色;
        for (int x = 0; x < 栅格宽度; x++)
        {
            for (int z = 0; z < 栅格高度; z++)
            {
                if (!栅格地图[x, z].walkable)
                {
                    Vector3 栅格中心 = 栅格转世界(new Vector2Int(x, z));
                    栅格中心.y = 障碍物显示高度;
                    Gizmos.DrawCube(栅格中心, new Vector3(栅格尺寸 * 0.8f, 0.2f, 栅格尺寸 * 0.8f));
                }
            }
        }

        // 栅格未就绪则不绘制
        if (!isGridReady) return;

        // 设置栅格线颜色（使用Inspector配置的颜色）
        Gizmos.color = 栅格线颜色;
        // 强制绘制栅格外框，确保能快速定位栅格范围
        Gizmos.DrawWireCube(
            栅格原点 + new Vector3(栅格宽度 / 2f, 栅格线高度, 栅格高度 / 2f),
            new Vector3(栅格宽度, 0.1f, 栅格高度)
        );

        // 绘制栅格网格线（逐行逐列绘制）
        for (int x = 0; x < 栅格宽度; x++)
        {
            for (int z = 0; z < 栅格高度; z++)
            {
                // 计算当前栅格起点
                Vector3 start = new Vector3(x * 栅格尺寸, 栅格线高度, z * 栅格尺寸) + 栅格原点;
                // 绘制水平方向线（X轴方向）
                Vector3 endHorizontal = new Vector3((x + 1) * 栅格尺寸, 栅格线高度, z * 栅格尺寸) + 栅格原点;
                Gizmos.DrawLine(start, endHorizontal);
                // 绘制垂直方向线（Z轴方向）
                Vector3 endVertical = new Vector3(x * 栅格尺寸, 栅格线高度, (z + 1) * 栅格尺寸) + 栅格原点;
                Gizmos.DrawLine(start, endVertical);
            }
        }

    }
}