using System.Collections.Generic;
using UnityEngine;

public class ImprovedAStar : MonoBehaviour
{
    public GridManager gridManager;
    public Transform startPos;
    public Transform targetPos;
    public List<Vector2Int> path;

    void Start()
    {
        Debug.Log("A*路径开始计算");
        if (gridManager == null || startPos == null || targetPos == null)
        {
            Debug.LogError("ImprovedAStar：依赖项未完全赋值！");
            return;
        }

        // 强制修正起点世界坐标（约束到栅格内）
        Vector3 startWorldPos = startPos.position;
        float maxX = gridManager.栅格原点.x + gridManager.栅格宽度 * gridManager.栅格尺寸;
        float maxZ = gridManager.栅格原点.z + gridManager.栅格高度 * gridManager.栅格尺寸;
        startWorldPos.x = Mathf.Clamp(startWorldPos.x, gridManager.栅格原点.x + 0.5f, maxX - 0.5f);
        startWorldPos.z = Mathf.Clamp(startWorldPos.z, gridManager.栅格原点.z + 0.5f, maxZ - 0.5f);
        startPos.position = startWorldPos;

        // 转换并检查起点栅格
        Vector2Int startGrid = gridManager.世界转栅格(startWorldPos);
        if (!gridManager.栅格是否可通行(startGrid))
        {
            Debug.LogWarning($"起点栅格{startGrid}不可通行，扩大范围寻找可通行栅格...");
            startGrid = FindNearestWalkableGrid(startWorldPos);
            if (startGrid == new Vector2Int(-1, -1))
            {
                Debug.LogError("A*路径计算失败：扩大范围后仍无可用栅格！请手动调整无人船位置到水域中央");
                path = null;
                return;
            }
            startWorldPos = gridManager.栅格转世界(startGrid);
            startPos.position = startWorldPos;
            Debug.Log($"已自动修正起点到栅格{startGrid}");
        }

        // 修正终点坐标
        Vector3 targetWorldPos = targetPos.position;
        targetWorldPos.x = Mathf.Clamp(targetWorldPos.x, gridManager.栅格原点.x + 0.5f, maxX - 0.5f);
        targetWorldPos.z = Mathf.Clamp(targetWorldPos.z, gridManager.栅格原点.z + 0.5f, maxZ - 0.5f);
        targetPos.position = targetWorldPos;
        Vector2Int targetGrid = gridManager.世界转栅格(targetWorldPos);

        if (!gridManager.栅格是否可通行(targetGrid))
        {
            Debug.LogError($"A*路径计算失败：终点栅格{targetGrid}不可通行！");
            path = null;
            return;
        }

        // 生成路径
        path = FindPath(startGrid, targetGrid);
        Debug.Log($"A*路径计算完成，路径点数量：{path?.Count ?? 0}");

        // 绘制路径
        if (path != null && path.Count > 0)
        {
            for (int i = 0; i < path.Count - 1; i++)
            {
                Vector3 起点世界坐标 = gridManager.栅格转世界(path[i]);
                Vector3 终点世界坐标 = gridManager.栅格转世界(path[i + 1]);
                Debug.DrawLine(起点世界坐标, 终点世界坐标, Color.green, 1000f);
            }
        }
    }

    // 扩大搜索范围到5×5，确保找到可通行栅格
    private Vector2Int FindNearestWalkableGrid(Vector3 centerWorldPos)
    {
        for (int xOffset = -2; xOffset <= 2; xOffset++)
        {
            for (int zOffset = -2; zOffset <= 2; zOffset++)
            {
                if (xOffset == 0 && zOffset == 0)
                    continue;

                Vector3 offsetWorldPos = centerWorldPos + new Vector3(
                    xOffset * gridManager.栅格尺寸,
                    0,
                    zOffset * gridManager.栅格尺寸
                );
                Vector2Int offsetGrid = gridManager.世界转栅格(offsetWorldPos);
                if (gridManager.栅格是否可通行(offsetGrid))
                {
                    return offsetGrid;
                }
            }
        }
        return new Vector2Int(-1, -1);
    }

    // 核心A*寻路算法（完整实现）
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

        return null;
    }

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

    private float CalculateFCost(Vector2Int node, Vector2Int target)
    {
        int dx = Mathf.Abs(node.x - target.x);
        int dy = Mathf.Abs(node.y - target.y);
        return Mathf.Sqrt(dx * dx + dy * dy) * gridManager.栅格尺寸;
    }

    private float CalculateDistance(Vector2Int a, Vector2Int b)
    {
        int dx = a.x - b.x;
        int dy = a.y - b.y;
        return Mathf.Sqrt(dx * dx + dy * dy);
    }

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

    private List<Vector2Int> ReconstructPath(Dictionary<Vector2Int, Vector2Int> parentMap, Vector2Int end)
    {
        List<Vector2Int> path = new List<Vector2Int>();
        path.Add(end);
        Vector2Int current = end;

        while (parentMap.ContainsKey(current))
        {
            current = parentMap[current];
            path.Insert(0, current);
        }

        return path;
    }
}