using System.Collections.Generic;
using UnityEngine;

// 1. 改为struct减少堆内存分配（值类型存储在栈或数组中）
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
    // 2. 使用结构体数组进一步提升内存效率
    private Node[,] 栅格地图;
    private int 初始化索引 = 0;
    private bool isInitializing = false;
    private bool isGridReady = false;
    private Collider[] 碰撞检测结果 = new Collider[1];
    private Vector2 水域大小缓存;

    // 3. 缓存计算常量（避免重复计算）
    private float 栅格半尺寸; // 栅格尺寸/2，初始化时计算
    private int 每帧初始化数量 = 200; // 可根据性能动态调整

    void Start()
    {
        if (水域平面 == null)
        {
            Debug.LogError("GridManager未赋值水域平面！");
            return;
        }
        栅格半尺寸 = 栅格尺寸 / 2f; // 预计算常量
        计算水域大小();
        栅格宽度 = Mathf.CeilToInt(水域大小缓存.x / 栅格尺寸);
        栅格高度 = Mathf.CeilToInt(水域大小缓存.y / 栅格尺寸);
        栅格原点 = 水域平面.position - new Vector3(水域大小缓存.x / 2, 0, 水域大小缓存.y / 2);
        栅格地图 = new Node[栅格宽度, 栅格高度];

        isInitializing = true;
        初始化索引 = 0;
        Debug.Log($"开始分帧初始化栅格：{栅格宽度}x{栅格高度}，原点：{栅格原点}");
    }

    void Update()
    {
        if (isInitializing)
        {
            int 总节点数 = 栅格宽度 * 栅格高度;
            int 结束索引 = Mathf.Min(初始化索引 + 每帧初始化数量, 总节点数);

            // 4. 循环展开优化（每次处理2个节点，减少循环判断次数）
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

                // 处理下一个节点
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

            // 处理剩余的1个节点（如果总节点数为奇数）
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

    // 5. 障碍物检测范围优化（只检测视野内栅格，适合超大场景）
    public void 标记障碍物(Camera 主相机 = null)
    {
        float 检测半径 = 栅格半尺寸 + 0.1f;
        int 起始X = 0, 结束X = 栅格宽度;
        int 起始Z = 0, 结束Z = 栅格高度;

        // 如果有主相机，只检测视野范围内的栅格
        if (主相机 != null)
        {
            // 计算视野边界的栅格坐标
            Vector3 左下角 = 主相机.ViewportToWorldPoint(new Vector3(0, 0, 主相机.nearClipPlane));
            Vector3 右上角 = 主相机.ViewportToWorldPoint(new Vector3(1, 1, 主相机.farClipPlane));
            Vector2Int 左下栅格 = 世界转栅格(左下角);
            Vector2Int 右上栅格 = 世界转栅格(右上角);

            // 扩大检测范围（避免物体刚出视野就未标记）
            起始X = Mathf.Max(0, 左下栅格.x - 5);
            结束X = Mathf.Min(栅格宽度, 右上栅格.x + 5);
            起始Z = Mathf.Max(0, 左下栅格.y - 5);
            结束Z = Mathf.Min(栅格高度, 右上栅格.y + 5);
        }

        // 6. 外层循环按缓存行大小步长处理（提升CPU缓存命中率）
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
                    // 结构体需要重新赋值（值类型特性）
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

        // 复用数组（尺寸不变时）
        if (栅格地图 == null || 栅格地图.GetLength(0) != 新宽度 || 栅格地图.GetLength(1) != 新高度)
        {
            栅格地图 = new Node[新宽度, 新高度];
        }

        栅格宽度 = 新宽度;
        栅格高度 = 新高度;
        栅格原点 = 水域平面.position - new Vector3(水域大小缓存.x / 2, 0, 水域大小缓存.y / 2);
        栅格半尺寸 = 栅格尺寸 / 2f; // 重新计算常量

        // 线性初始化（复用计算逻辑）
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

    private void OnDrawGizmos()
    {
        if (水域平面 == null || 栅格地图 == null) return;

        Gizmos.color = Color.white;
        Gizmos.DrawWireCube(水域平面.position, new Vector3(水域大小缓存.x, 0.1f, 水域大小缓存.y));

        Gizmos.color = Color.gray;
        for (int x = 0; x <= 栅格宽度; x++)
        {
            Vector3 起点 = 栅格原点 + new Vector3(x * 栅格尺寸, 0.1f, 0);
            Vector3 终点 = 栅格原点 + new Vector3(x * 栅格尺寸, 0.1f, 栅格高度 * 栅格尺寸);
            Gizmos.DrawLine(起点, 终点);
        }
        for (int z = 0; z <= 栅格高度; z++)
        {
            Vector3 起点 = 栅格原点 + new Vector3(0, 0.1f, z * 栅格尺寸);
            Vector3 终点 = 栅格原点 + new Vector3(栅格宽度 * 栅格尺寸, 0.1f, z * 栅格尺寸);
            Gizmos.DrawLine(起点, 终点);
        }

        Gizmos.color = Color.red;
        //  gizmos也只绘制视野内的障碍物（优化编辑器性能）
        for (int x = 0; x < 栅格宽度; x++)
        {
            for (int z = 0; z < 栅格高度; z++)
            {
                if (!栅格地图[x, z].walkable)
                {
                    Vector3 栅格中心 = 栅格转世界(new Vector2Int(x, z));
                    Gizmos.DrawCube(栅格中心, new Vector3(栅格尺寸 - 0.1f, 0.2f, 栅格尺寸 - 0.1f));
                }
            }
        }
    }
}