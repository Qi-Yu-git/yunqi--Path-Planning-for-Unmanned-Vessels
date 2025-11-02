using System.Collections.Generic;
using UnityEngine;

public class ImprovedAStar : MonoBehaviour
{
    public GridManager gridManager;    // 拖拽栅格管理脚本对象
    public Transform startPos;        // 起点（无人艇初始位置）
    public Transform targetPos;       // 终点（目标位置）
    public List<Vector2Int> path;     // 开放访问权限，让BoatController可调用

    void Start()
    {
        Debug.Log("A*路径开始计算");
        Vector2Int startGrid = gridManager.世界转栅格(startPos.position);
        Vector2Int targetGrid = gridManager.世界转栅格(targetPos.position);
        path = FindPath(startGrid, targetGrid);
        Debug.Log("A*路径计算完成，路径点数量：" + (path?.Count ?? 0));

        // 绘制路径（调试用）
        if (path != null)
        {
            for (int i = 0; i < path.Count - 1; i++)
            {
                Debug.DrawLine(
                    gridManager.栅格转世界(path[i]),
                    gridManager.栅格转世界(path[i + 1]),
                    Color.green,
                    1000f
                );
            }
        }
    }

    // 开放访问权限，让BoatController可调用重新寻路
    public List<Vector2Int> FindPath(Vector2Int start, Vector2Int target)
    {
        List<Vector2Int> openList = new List<Vector2Int>();
        HashSet<Vector2Int> closedList = new HashSet<Vector2Int>();
        Dictionary<Vector2Int, Vector2Int> parentMap = new Dictionary<Vector2Int, Vector2Int>();
        Dictionary<Vector2Int, float> gCostMap = new Dictionary<Vector2Int, float>();
        Dictionary<Vector2Int, float> fCostMap = new Dictionary<Vector2Int, float>();

        openList.Add(start);
        gCostMap[start] = 0;
        fCostMap[start] = CalculateFCost(start, target);

        while (openList.Count > 0)
        {
            Vector2Int current = GetLowestFCostNode(openList, fCostMap);
            if (current == target)
            {
                return ReconstructPath(parentMap, current);
            }

            openList.Remove(current);
            closedList.Add(current);

            foreach (Vector2Int neighbor in GetNeighbors(current))
            {
                if (closedList.Contains(neighbor) || !gridManager.栅格是否可通行(neighbor))
                    continue;

                float newGCost = gCostMap[current] + CalculateDistance(current, neighbor) * gridManager.栅格尺寸;
                if (!gCostMap.ContainsKey(neighbor) || newGCost < gCostMap[neighbor])
                {
                    parentMap[neighbor] = current;
                    gCostMap[neighbor] = newGCost;
                    fCostMap[neighbor] = newGCost + CalculateFCost(neighbor, target);

                    if (!openList.Contains(neighbor))
                        openList.Add(neighbor);
                }
            }
        }
        return null; // 无路径
    }

    // 获取8邻域节点
    private List<Vector2Int> GetNeighbors(Vector2Int node)
    {
        return new List<Vector2Int>
        {
            new Vector2Int(node.x - 1, node.y - 1),
            new Vector2Int(node.x, node.y - 1),
            new Vector2Int(node.x + 1, node.y - 1),
            new Vector2Int(node.x - 1, node.y),
            new Vector2Int(node.x + 1, node.y),
            new Vector2Int(node.x - 1, node.y + 1),
            new Vector2Int(node.x, node.y + 1),
            new Vector2Int(node.x + 1, node.y + 1)
        };
    }

    // 计算F值（G+启发式H，H采用欧几里得距离）
    private float CalculateFCost(Vector2Int node, Vector2Int target)
    {
        return CalculateDistance(node, target) * gridManager.栅格尺寸;
    }

    // 计算欧几里得距离
    private float CalculateDistance(Vector2Int a, Vector2Int b)
    {
        int dx = a.x - b.x;
        int dy = a.y - b.y;
        return Mathf.Sqrt(dx * dx + dy * dy);
    }

    // 获取OpenList中F值最小的节点
    private Vector2Int GetLowestFCostNode(List<Vector2Int> openList, Dictionary<Vector2Int, float> fCostMap)
    {
        Vector2Int lowest = openList[0];
        for (int i = 1; i < openList.Count; i++)
        {
            if (fCostMap[openList[i]] < fCostMap[lowest])
                lowest = openList[i];
        }
        return lowest;
    }

    // 重构路径
    private List<Vector2Int> ReconstructPath(Dictionary<Vector2Int, Vector2Int> parentMap, Vector2Int end)
    {
        List<Vector2Int> path = new List<Vector2Int>();
        path.Add(end);
        while (parentMap.ContainsKey(path[0]))
        {
            path.Insert(0, parentMap[path[0]]);
        }
        // 移除第一个与起点重复的路径点
        if (path.Count > 0 && path[0] == gridManager.世界转栅格(startPos.position))
        {
            path.RemoveAt(0);
        }
        Debug.Log("路径点顺序：" + string.Join(", ", path));
        return path;
    }
}

