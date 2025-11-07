using System;
using System.Collections.Generic;
using UnityEngine;

// 保持类名与原代码一致，解决CS0246错误
public class ImprovedAStar : MonoBehaviour
{
    // 常量定义
    private const float WATER_Y_HEIGHT = 0.05f;
    private const int NEIGHBOR_SEARCH_RANGE = 2;
    private const float DIAGONAL_COST = 1.41421356f; // 精确√2值
    private const float STRAIGHT_COST = 1f;

    // 静态邻居偏移量（8方向）
    private static readonly Vector2Int[] NeighborOffsets = new[]
    {
        new Vector2Int(-1, -1), new Vector2Int(0, -1), new Vector2Int(1, -1),
        new Vector2Int(-1, 0),                          new Vector2Int(1, 0),
        new Vector2Int(-1, 1),  new Vector2Int(0, 1), new Vector2Int(1, 1)
    };

    [SerializeField] private GridManager gridManager;
    [SerializeField] private Transform startPos;
    [SerializeField] private Transform targetPos;

    public List<Vector2Int> path;
    private List<Vector2Int> Path
    {
        get => path;
        set => path = value;
    }

    // 栅格参数缓存
    private int gridWidth;
    private int gridHeight;
    private float cellSize;
    private Vector3 gridOrigin;
    private float cellSizeHeuristic; // 预计算启发式系数

    // 复用集合（减少GC）
    private List<Vector2Int> neighborBuffer = new List<Vector2Int>(8);
    private BinaryHeapPriorityQueue openQueue = new BinaryHeapPriorityQueue(1024);
    private NodeData[,] nodeDataArray; // 复用节点数组
    private List<Vector2Int> pathBuffer = new List<Vector2Int>(256);

    private void Start()
    {
        Debug.Log("A*路径准备计算（等待目标点生成）");
        if (CheckDependencies())
        {
            CacheGridParameters();
            InitializeNodeDataArray();
            Invoke(nameof(CalculatePathAfterDelay), 0.5f);
        }
    }

    private void CacheGridParameters()
    {
        gridWidth = gridManager.栅格宽度;
        gridHeight = gridManager.栅格高度;
        cellSize = gridManager.栅格尺寸;
        gridOrigin = gridManager.栅格原点;
        cellSizeHeuristic = cellSize * 1.0001f; // 微小偏移确保启发式不高估
    }

    private void InitializeNodeDataArray()
    {
        nodeDataArray = new NodeData[gridWidth, gridHeight];
        // 预初始化所有节点（避免重复分配）
        for (int x = 0; x < gridWidth; x++)
        {
            for (int y = 0; y < gridHeight; y++)
            {
                nodeDataArray[x, y] = new NodeData(
                    float.MaxValue,
                    float.MaxValue,
                    new Vector2Int(-1, -1),
                    false,
                    false
                );
            }
        }
    }

    private bool CheckDependencies()
    {
        if (gridManager == null)
        {
            Debug.LogError("ImprovedAStar：GridManager未赋值！");
            return false;
        }
        if (startPos == null || targetPos == null)
        {
            Debug.LogError("ImprovedAStar：起点或目标点未赋值！");
            return false;
        }
        return true;
    }

    private void CalculatePathAfterDelay()
    {
        if (gridManager == null) return;

        Vector3 startWorldPos = ClampPositionToGrid(startPos.position);
        startPos.position = startWorldPos;

        Vector3 targetWorldPos = ClampPositionToGrid(targetPos.position);
        targetPos.position = targetWorldPos;

        Vector2Int startGrid = gridManager.世界转栅格(startWorldPos);
        Vector2Int targetGrid = gridManager.世界转栅格(targetWorldPos);

        startGrid = FindValidGrid(startGrid);
        if (startGrid.x == -1)
        {
            Debug.LogError("A*路径计算失败：扩大范围后仍无可用起点栅格！");
            Path = null;
            return;
        }

        startWorldPos = gridManager.栅格转世界(startGrid);
        startWorldPos.y = WATER_Y_HEIGHT;
        startPos.position = startWorldPos;
        Debug.Log($"已自动修正起点到栅格{startGrid}，位置：{startWorldPos}");

        if (!gridManager.栅格是否可通行(targetGrid))
        {
            Debug.LogError($"A*路径计算失败：终点栅格{targetGrid}不可通行！");
            Path = null;
            return;
        }

        Path = FindPath(startGrid, targetGrid);
        Debug.Log($"A*路径计算完成，目标点：{targetWorldPos}，路径点数量：{Path?.Count ?? 0}");

        DrawPath();
    }

    private Vector3 ClampPositionToGrid(Vector3 worldPos)
    {
        worldPos.y = WATER_Y_HEIGHT;
        float minX = gridOrigin.x + cellSize * 0.5f;
        float maxX = gridOrigin.x + (gridWidth - 1) * cellSize + cellSize * 0.5f;
        float minZ = gridOrigin.z + cellSize * 0.5f;
        float maxZ = gridOrigin.z + (gridHeight - 1) * cellSize + cellSize * 0.5f;

        worldPos.x = Mathf.Clamp(worldPos.x, minX, maxX);
        worldPos.z = Mathf.Clamp(worldPos.z, minZ, maxZ);
        return worldPos;
    }

    // 螺旋式搜索更高效的寻找可用栅格
    private Vector2Int FindValidGrid(Vector2Int originalGrid)
    {
        if (IsValidGrid(originalGrid) && gridManager.栅格是否可通行(originalGrid))
            return originalGrid;

        for (int layer = 1; layer <= NEIGHBOR_SEARCH_RANGE; layer++)
        {
            // 上边缘
            for (int x = -layer; x <= layer; x++)
            {
                var checkPos = new Vector2Int(originalGrid.x + x, originalGrid.y - layer);
                if (IsValidGrid(checkPos) && gridManager.栅格是否可通行(checkPos))
                    return checkPos;
            }

            // 右边缘
            for (int y = -layer + 1; y <= layer; y++)
            {
                var checkPos = new Vector2Int(originalGrid.x + layer, originalGrid.y + y);
                if (IsValidGrid(checkPos) && gridManager.栅格是否可通行(checkPos))
                    return checkPos;
            }

            // 下边缘
            for (int x = layer - 1; x >= -layer; x--)
            {
                var checkPos = new Vector2Int(originalGrid.x + x, originalGrid.y + layer);
                if (IsValidGrid(checkPos) && gridManager.栅格是否可通行(checkPos))
                    return checkPos;
            }

            // 左边缘
            for (int y = layer - 1; y > -layer; y--)
            {
                var checkPos = new Vector2Int(originalGrid.x - layer, originalGrid.y + y);
                if (IsValidGrid(checkPos) && gridManager.栅格是否可通行(checkPos))
                    return checkPos;
            }
        }

        return new Vector2Int(-1, -1);
    }

    private bool IsValidGrid(Vector2Int gridPos)
    {
        return gridPos.x >= 0 && gridPos.x < gridWidth &&
               gridPos.y >= 0 && gridPos.y < gridHeight;
    }

    private List<Vector2Int> FindPath(Vector2Int start, Vector2Int target)
    {
        ResetNodeData();
        openQueue.Clear();

        // 初始化起点
        nodeDataArray[start.x, start.y].GCost = 0;
        float hCost = CalculateHeuristic(start, target);
        nodeDataArray[start.x, start.y].FCost = hCost;
        nodeDataArray[start.x, start.y].InOpenSet = true;
        openQueue.Enqueue(start, hCost);

        while (openQueue.Count > 0)
        {
            Vector2Int current = openQueue.Dequeue();

            // 标记为已处理
            nodeDataArray[current.x, current.y].IsClosed = true;
            nodeDataArray[current.x, current.y].InOpenSet = false;

            // 到达目标
            if (current.Equals(target))
            {
                return ReconstructPath(target);
            }

            // 获取邻居
            neighborBuffer.Clear();
            GetNeighbors(current, neighborBuffer);

            foreach (Vector2Int neighbor in neighborBuffer)
            {
                if (nodeDataArray[neighbor.x, neighbor.y].IsClosed)
                    continue;

                if (!gridManager.栅格是否可通行(neighbor))
                    continue;

                // 计算新G值
                float newGCost = nodeDataArray[current.x, current.y].GCost +
                                CalculateDistance(current, neighbor) * cellSize;

                // 发现更优路径
                if (newGCost < nodeDataArray[neighbor.x, neighbor.y].GCost)
                {
                    nodeDataArray[neighbor.x, neighbor.y].GCost = newGCost;
                    float neighborHCost = CalculateHeuristic(neighbor, target);
                    float neighborFCost = newGCost + neighborHCost;
                    nodeDataArray[neighbor.x, neighbor.y].FCost = neighborFCost;
                    nodeDataArray[neighbor.x, neighbor.y].Parent = current;

                    if (nodeDataArray[neighbor.x, neighbor.y].InOpenSet)
                    {
                        openQueue.UpdatePriority(neighbor, neighborFCost);
                    }
                    else
                    {
                        nodeDataArray[neighbor.x, neighbor.y].InOpenSet = true;
                        openQueue.Enqueue(neighbor, neighborFCost);
                    }
                }
            }
        }

        return null; // 无路径
    }

    private void ResetNodeData()
    {
        for (int x = 0; x < gridWidth; x++)
        {
            for (int y = 0; y < gridHeight; y++)
            {
                nodeDataArray[x, y].GCost = float.MaxValue;
                nodeDataArray[x, y].FCost = float.MaxValue;
                nodeDataArray[x, y].Parent = new Vector2Int(-1, -1);
                nodeDataArray[x, y].IsClosed = false;
                nodeDataArray[x, y].InOpenSet = false;
            }
        }
    }

    private void GetNeighbors(Vector2Int node, List<Vector2Int> buffer)
    {
        foreach (var offset in NeighborOffsets)
        {
            int x = node.x + offset.x;
            int y = node.y + offset.y;
            if (x >= 0 && x < gridWidth && y >= 0 && y < gridHeight)
            {
                buffer.Add(new Vector2Int(x, y));
            }
        }
    }

    private float CalculateHeuristic(Vector2Int a, Vector2Int b)
    {
        int dx = Mathf.Abs(a.x - b.x);
        int dy = Mathf.Abs(a.y - b.y);
        // 优化的启发式：结合曼哈顿和对角线距离（更精确）
        return (dx + dy + (DIAGONAL_COST - 2) * Mathf.Min(dx, dy)) * cellSizeHeuristic;
    }

    private float CalculateDistance(Vector2Int a, Vector2Int b)
    {
        int dx = Mathf.Abs(a.x - b.x);
        int dy = Mathf.Abs(a.y - b.y);
        return dx == 0 || dy == 0 ? STRAIGHT_COST : DIAGONAL_COST;
    }

    private List<Vector2Int> ReconstructPath(Vector2Int end)
    {
        pathBuffer.Clear();
        Vector2Int current = end;

        while (current.x != -1)
        {
            pathBuffer.Add(current);
            current = nodeDataArray[current.x, current.y].Parent;
        }

        pathBuffer.Reverse();
        return new List<Vector2Int>(pathBuffer);
    }

    private void DrawPath()
    {
        if (Path == null || Path.Count < 2)
            return;

        for (int i = 0; i < Path.Count - 1; i++)
        {
            Vector3 from = gridManager.栅格转世界(Path[i]);
            Vector3 to = gridManager.栅格转世界(Path[i + 1]);
            from.y = to.y = WATER_Y_HEIGHT;
            Debug.DrawLine(from, to, Color.green, 1000f);
        }
    }

    private struct NodeData
    {
        public float GCost;
        public float FCost;
        public Vector2Int Parent;
        public bool IsClosed;
        public bool InOpenSet;

        public NodeData(float gCost, float fCost, Vector2Int parent, bool isClosed, bool inOpenSet)
        {
            GCost = gCost;
            FCost = fCost;
            Parent = parent;
            IsClosed = isClosed;
            InOpenSet = inOpenSet;
        }
    }

    // 二叉堆实现的优先级队列（O(log n)操作复杂度）
    private class BinaryHeapPriorityQueue
    {
        // 使用类而不是结构体解决CS1612错误
        private class HeapItem
        {
            public Vector2Int Node;
            public float Priority;
            public int Index;

            public HeapItem(Vector2Int node, float priority)
            {
                Node = node;
                Priority = priority;
                Index = -1;
            }
        }

        private readonly List<HeapItem> items;
        private readonly Dictionary<Vector2Int, HeapItem> nodeMap;
        private int count;

        public int Count => count;

        public BinaryHeapPriorityQueue(int initialCapacity)
        {
            items = new List<HeapItem>(initialCapacity);
            nodeMap = new Dictionary<Vector2Int, HeapItem>(initialCapacity);
            count = 0;
        }

        public void Enqueue(Vector2Int node, float priority)
        {
            if (nodeMap.TryGetValue(node, out var existing))
            {
                if (priority < existing.Priority)
                {
                    UpdatePriority(node, priority);
                }
                return;
            }

            var newItem = new HeapItem(node, priority) { Index = count };
            items.Add(newItem);
            nodeMap[node] = newItem;
            count++;
            BubbleUp(count - 1);
        }

        public Vector2Int Dequeue()
        {
            if (count == 0)
                throw new InvalidOperationException("队列已空");

            var topItem = items[0];
            count--;

            // 替换堆顶元素并更新索引（解决CS1612）
            var lastItem = items[count];
            lastItem.Index = 0;
            items[0] = lastItem;

            items.RemoveAt(count);
            nodeMap.Remove(topItem.Node);
            BubbleDown(0);

            return topItem.Node;
        }

        public void UpdatePriority(Vector2Int node, float newPriority)
        {
            if (!nodeMap.TryGetValue(node, out var item))
                return;

            int index = item.Index;
            item.Priority = newPriority;
            items[index] = item;
            nodeMap[node] = item;

            BubbleUp(index);
            BubbleDown(index);
        }

        public void Clear()
        {
            items.Clear();
            nodeMap.Clear();
            count = 0;
        }

        private void BubbleUp(int index)
        {
            while (index > 0)
            {
                int parentIndex = (index - 1) >> 1; // (index-1)/2
                if (items[parentIndex].Priority <= items[index].Priority)
                    break;

                Swap(index, parentIndex);
                index = parentIndex;
            }
        }

        private void BubbleDown(int index)
        {
            while (true)
            {
                int leftChild = (index << 1) + 1; // 2*index +1
                int rightChild = leftChild + 1;
                int smallest = index;

                if (leftChild < count && items[leftChild].Priority < items[smallest].Priority)
                    smallest = leftChild;

                if (rightChild < count && items[rightChild].Priority < items[smallest].Priority)
                    smallest = rightChild;

                if (smallest == index)
                    break;

                Swap(index, smallest);
                index = smallest;
            }
        }

        private void Swap(int i, int j)
        {
            // 交换元素并更新索引（解决CS1612）
            HeapItem temp = items[i];
            items[i] = items[j];
            items[j] = temp;

            items[i].Index = i;
            items[j].Index = j;

            // 更新字典映射
            nodeMap[items[i].Node] = items[i];
            nodeMap[items[j].Node] = items[j];
        }
    }
}