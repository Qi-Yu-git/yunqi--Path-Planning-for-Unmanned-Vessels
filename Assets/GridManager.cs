using System.Collections.Generic;
using UnityEngine;

// 栅格节点类（仅内部使用）
internal class Node
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
    // 公开配置参数
    public float 栅格尺寸 = 1f;
    public Transform 水域平面;
    public LayerMask obstacleLayer;

    // 公开变量（供外部访问）
    public Vector3 栅格原点;
    public int 栅格宽度;
    public int 栅格高度;

    private Node[,] 栅格地图;

    void Start()
    {
        初始化栅格();
        标记障碍物();
        Debug.Log($"栅格初始化完成：{栅格宽度}×{栅格高度}，原点：{栅格原点}");
    }

    private void 初始化栅格()
    {
        if (水域平面 == null)
        {
            Debug.LogError("GridManager：未赋值水域平面！");
            return;
        }

        float 水域大小X = 水域平面.lossyScale.x * 10;
        float 水域大小Z = 水域平面.lossyScale.z * 10;

        栅格宽度 = Mathf.CeilToInt(水域大小X / 栅格尺寸);
        栅格高度 = Mathf.CeilToInt(水域大小Z / 栅格尺寸);

        栅格原点 = 水域平面.position - new Vector3(水域大小X / 2, 0, 水域大小Z / 2);

        栅格地图 = new Node[栅格宽度, 栅格高度];
        for (int x = 0; x < 栅格宽度; x++)
        {
            for (int z = 0; z < 栅格高度; z++)
            {
                Vector3 节点世界坐标 = 栅格原点 + new Vector3(
                    x * 栅格尺寸 + 栅格尺寸 / 2,
                    水域平面.position.y,
                    z * 栅格尺寸 + 栅格尺寸 / 2
                );
                栅格地图[x, z] = new Node(true, 节点世界坐标, x, z);
            }
        }
    }

    private void 标记障碍物()
    {
        for (int x = 0; x < 栅格宽度; x++)
        {
            for (int z = 0; z < 栅格高度; z++)
            {
                Node 节点 = 栅格地图[x, z];
                Collider[] 碰撞体 = Physics.OverlapSphere(
                    节点.worldPosition,
                    栅格尺寸 / 2 - 0.1f,
                    obstacleLayer
                );
                foreach (var 碰撞 in 碰撞体)
                {
                    if (碰撞.gameObject != 水域平面.gameObject)
                    {
                        节点.walkable = false;
                        break;
                    }
                }
            }
        }
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
            x * 栅格尺寸 + 栅格尺寸 / 2,
            水域平面.position.y,
            z * 栅格尺寸 + 栅格尺寸 / 2
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

        float 水域大小X = 水域平面.lossyScale.x * 10;
        float 水域大小Z = 水域平面.lossyScale.z * 10;
        Gizmos.color = Color.white;
        Gizmos.DrawWireCube(
            水域平面.position,
            new Vector3(水域大小X, 0.1f, 水域大小Z)
        );

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