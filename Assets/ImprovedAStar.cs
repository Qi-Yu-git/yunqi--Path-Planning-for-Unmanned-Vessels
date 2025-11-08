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

    // 原私有方法修改为公开方法，允许外部脚本调用
    public void CalculatePathAfterDelay()
    {
        if (gridManager == null) return;

        Vector3 startWorldPos = ClampPositionToGrid(startPos.position);
        startPos.position = startWorldPos;
        Debug.Log($"起点原始世界坐标：{startPos.position}，限制后世界坐标：{startWorldPos}");

        Vector3 targetWorldPos = ClampPositionToGrid(targetPos.position);
        targetPos.position = targetWorldPos;
        Debug.Log($"目标原始世界坐标：{targetPos.position}，限制后世界坐标：{targetWorldPos}");

        Vector2Int startGrid = gridManager.世界转栅格(startWorldPos);
        Vector2Int targetGrid = gridManager.世界转栅格(targetWorldPos);
        Debug.Log($"起点栅格：{startGrid}，目标栅格：{targetGrid}");

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
        Debug.Log($"修正后起点栅格：{startGrid}，世界坐标：{startWorldPos}");

        if (!gridManager.栅格是否可通行(targetGrid))
        {
            Debug.LogError($"A*路径计算失败：终点栅格{targetGrid}不可通行！");
            Path = null;
            return;
        }

        Path = FindPath(startGrid, targetGrid);
        Debug.Log($"A*路径计算完成，目标点：{targetWorldPos}，路径点数量：{Path?.Count ?? 0}");


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

        // 添加日志：输出原始坐标、限制后的坐标、栅格边界
        Debug.Log($"Clamp前坐标：{worldPos}，Clamp后坐标：{worldPos}，边界[X: {minX}-{maxX}, Z: {minZ}-{maxZ}]");
        return worldPos;
    }

    // 螺旋式搜索更高效的寻找可用栅格
    private Vector2Int FindValidGrid(Vector2Int originalGrid)
    {
        Debug.Log($"原始栅格：{originalGrid}，是否有效且可通行：{IsValidGrid(originalGrid) && gridManager.栅格是否可通行(originalGrid)}");

        if (IsValidGrid(originalGrid) && gridManager.栅格是否可通行(originalGrid))
            return originalGrid;

        for (int layer = 1; layer <= NEIGHBOR_SEARCH_RANGE; layer++)
        {
            Debug.Log($"开始搜索第{layer}层栅格...");
            // 上边缘
            for (int x = -layer; x <= layer; x++)
            {
                var checkPos = new Vector2Int(originalGrid.x + x, originalGrid.y - layer);
                if (IsValidGrid(checkPos) && gridManager.栅格是否可通行(checkPos))
                {
                    Debug.Log($"找到有效栅格（上边缘）：{checkPos}");
                    return checkPos;
                }
            }

            // 右边缘
            for (int y = -layer + 1; y <= layer; y++)
            {
                var checkPos = new Vector2Int(originalGrid.x + layer, originalGrid.y + y);
                if (IsValidGrid(checkPos) && gridManager.栅格是否可通行(checkPos))
                {
                    Debug.Log($"找到有效栅格（右边缘）：{checkPos}");
                    return checkPos;
                }
            }

            // 下边缘
            for (int x = layer - 1; x >= -layer; x--)
            {
                var checkPos = new Vector2Int(originalGrid.x + x, originalGrid.y + layer);
                if (IsValidGrid(checkPos) && gridManager.栅格是否可通行(checkPos))
                {
                    Debug.Log($"找到有效栅格（下边缘）：{checkPos}");
                    return checkPos;
                }
            }

            // 左边缘
            for (int y = layer - 1; y > -layer; y--)
            {
                var checkPos = new Vector2Int(originalGrid.x - layer, originalGrid.y + y);
                if (IsValidGrid(checkPos) && gridManager.栅格是否可通行(checkPos))
                {
                    Debug.Log($"找到有效栅格（左边缘）：{checkPos}");
                    return checkPos;
                }
            }
        }
        Debug.LogError("未找到有效栅格！");
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
        int safetyCount = 0; // 防止死循环

        while (current.x != -1)
        {
            // 避免重复添加同一节点
            if (pathBuffer.Contains(current))
            {
                Debug.LogWarning("路径存在循环节点，已中断！");
                break;
            }

            pathBuffer.Add(current);
            current = nodeDataArray[current.x, current.y].Parent;

            // 安全检查：超过栅格总节点数则中断
            safetyCount++;
            if (safetyCount > gridWidth * gridHeight)
            {
                Debug.LogError("路径重构陷入死循环，已强制中断！");
                pathBuffer.Clear();
                return null;
            }
        }

        if (pathBuffer.Count == 0)
        {
            Debug.LogError("路径重构失败，无有效节点！");
            return null;
        }

        pathBuffer.Reverse();
        // 输出路径点详情，验证是否在栅格范围内
        Debug.Log($"路径重构完成，节点数：{pathBuffer.Count}，首节点：{pathBuffer[0]}，尾节点：{pathBuffer[pathBuffer.Count - 1]}");
        return new List<Vector2Int>(pathBuffer);
    }

    // 在ImprovedAStar中添加：
    [ContextMenu("聚焦路径")] // 右键菜单可直接调用
    public void FocusPath()
    {
        if (Path == null || Path.Count == 0 || gridManager == null)
        {
            Debug.LogWarning("无路径可聚焦！");
            return;
        }

        // 计算路径中心点
        Vector3 center = Vector3.zero;
        foreach (var gridPos in Path)
        {
            center += gridManager.栅格转世界(gridPos);
        }
        center /= Path.Count;

        // 聚焦到路径中心（适用于Scene视图）
        Camera sceneCamera = Camera.main;
        if (sceneCamera == null)
        {
            Debug.LogWarning("找不到主相机，无法自动聚焦");
            return;
        }
        sceneCamera.transform.position = center + new Vector3(0, 10, -10); // 路径上方视角
        sceneCamera.transform.LookAt(center);
        Debug.Log("已聚焦到路径中心");
    }

    private void DrawPath()
    {
        if (Path == null || Path.Count <= 1)
        {
            Debug.Log("路径为空或点数不足，无法绘制");
            return;
        }
        Debug.Log($"执行路径绘制，共{Path.Count}个点");
        // 可选：输出前3个路径点的世界坐标，验证转换是否正确
        for (int i = 0; i < Mathf.Min(3, Path.Count); i++)
        {
            Vector3 worldPos = gridManager.栅格转世界(Path[i]);
            Debug.Log($"路径点{i + 1} 栅格坐标：{Path[i]} → 世界坐标：{worldPos}");
        }
    }
    private void OnDrawGizmos()
    {
        // 1. 验证依赖
        if (gridManager == null || Path == null || Path.Count < 2)
            return;

        // 2. 绘制细绿色路径线
        Gizmos.color = new Color(0, 1, 0, 0.6f); // 半透明绿色，更柔和
        for (int i = 0; i < Path.Count - 1; i++)
        {
            Vector3 start = gridManager.栅格转世界(Path[i]);
            Vector3 end = gridManager.栅格转世界(Path[i + 1]);
            start.y = end.y = 0.1f; // 贴近水面高度，不突兀
            Gizmos.DrawLine(start, end);
        }

        // 3. 可选：轻微标记起点（避免完全隐形）
        Gizmos.color = new Color(0, 1, 0, 0.8f);
        Gizmos.DrawSphere(gridManager.栅格转世界(Path[0]), 0.2f);
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