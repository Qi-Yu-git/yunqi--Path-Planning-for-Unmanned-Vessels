using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

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

    // 在ImprovedAStar类中添加状态标记
    private bool isCalculatingPath = false;

    private void Start()
    {
        Debug.Log("A*路径准备计算（等待目标点生成）");
        if (CheckDependencies())
        {
            StartCoroutine(DelayCalculatePath(0.5f));
        }
    }

    // 延迟启动路径计算
    private IEnumerator DelayCalculatePath(float delayTime)
    {
        yield return new WaitForSeconds(delayTime);
        StartCoroutine(CalculatePathCoroutine());
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
        // 预初始化所有节点
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

    // 修改CalculatePathAfterDelay方法
    public void CalculatePathAfterDelay()
    {
        if (isCalculatingPath)
        {
            Debug.Log("路径计算已在进行中，忽略重复请求");
            return;
        }
        StartCoroutine(CalculatePathCoroutine());
    }


    // 协程版路径计算（带重试机制）
    private IEnumerator CalculatePathCoroutine()
    {
        int retryCount = 0;
        while (retryCount < 3)
        {
            if (gridManager == null)
            {
                Debug.LogError("A*路径计算失败：gridManager 未赋值！");
                path = null;
                yield break;
            }

            // 等待栅格初始化
            float waitTime = 0f;
            while (!gridManager.IsGridReady() && waitTime < 5f)
            {
                Debug.LogWarning($"A*等待栅格初始化...已等待{waitTime:F1}秒");
                waitTime += 0.5f;
                yield return new WaitForSeconds(0.5f);
            }

            if (!gridManager.IsGridReady())
            {
                Debug.LogError("A*路径计算失败：栅格初始化超时！");
                path = null;
                yield break;
            }

            CacheGridParameters();
            InitializeNodeDataArray();

            // 校验并修正起点/终点
            Vector3 startWorldPos = ClampPositionToGrid(startPos.position);
            Vector3 targetWorldPos = ClampPositionToGrid(targetPos.position);
            Vector2Int startGrid = gridManager.世界转栅格(startWorldPos);
            Vector2Int targetGrid = gridManager.世界转栅格(targetWorldPos);

            // 调用带搜索半径的FindValidGrid
            startGrid = FindValidGrid(startGrid, 5);
            targetGrid = FindValidGrid(targetGrid, 5);

            if (startGrid.x == -1 || targetGrid.x == -1)
            {
                Debug.LogError($"第{retryCount + 1}次重试：无法找到有效起点/终点！");
                retryCount++;
                yield return new WaitForSeconds(1f);
                continue;
            }

            // 更新起点/终点世界坐标
            startWorldPos = gridManager.栅格转世界(startGrid);
            startWorldPos.y = WATER_Y_HEIGHT;
            startPos.position = startWorldPos;
            targetWorldPos = gridManager.栅格转世界(targetGrid);
            targetWorldPos.y = WATER_Y_HEIGHT;
            targetPos.position = targetWorldPos;

            // 计算路径（包含简化）
            path = FindPath(startGrid, targetGrid);
            if (path != null && path.Count > 1)
            {
                
                Debug.Log($"路径计算成功，包含{path.Count}个点：{string.Join("->", path)}");

                yield break;
            }
            else
            {
                Debug.LogError("路径为空或只有一个点，无法显示");

                retryCount++;
                yield return new WaitForSeconds(1f);
            }

            // 当路径计算成功并返回时
            if (path != null && path.Count > 1)
            {
                Debug.Log($"路径计算成功，包含{path.Count}个点：{string.Join("->", path)}");
                isCalculatingPath = false; // 重置状态
                yield break;
            }

        }
        isCalculatingPath = false; // 循环结束后重置状态

        // 修复：使用Unity 6推荐的API替换过时的FindObjectOfType
        Debug.LogError("A*路径计算失败：3次重试后仍为空！尝试重新生成起点终点...");
        RandomSpawnManager spawnManager = FindFirstObjectByType<RandomSpawnManager>();
        if (spawnManager != null)
        {
            spawnManager.Regenerate();
            yield return new WaitForSeconds(0.3f);
            StartCoroutine(CalculatePathCoroutine()); // 重新计算路径
        }
        path = null;
    }

    // 限制坐标在栅格范围内
    private Vector3 ClampPositionToGrid(Vector3 worldPos)
    {
        worldPos.y = WATER_Y_HEIGHT;
        float minX = gridOrigin.x + cellSize * 0.5f;
        float maxX = gridOrigin.x + (gridWidth - 1) * cellSize + cellSize * 0.5f;
        float minZ = gridOrigin.z + cellSize * 0.5f;
        float maxZ = gridOrigin.z + (gridHeight - 1) * cellSize + cellSize * 0.5f;
        worldPos.x = Mathf.Clamp(worldPos.x, minX, maxX);
        worldPos.z = Mathf.Clamp(worldPos.z, minZ, maxZ);
        Debug.Log($"Clamp前坐标：{worldPos}，Clamp后坐标：{worldPos}，边界[X: {minX}-{maxX}, Z: {minZ}-{maxZ}]");
        return worldPos;
    }

    // ImprovedAStar.cs - FindValidGrid() 方法增强
    private Vector2Int FindValidGrid(Vector2Int originalGrid, int searchRange = NEIGHBOR_SEARCH_RANGE)
    {
        // 先检查原始栅格是否有效
        if (IsValidGrid(originalGrid) && gridManager.栅格是否可通行(originalGrid))
        {
            return originalGrid;
        }

        // 扩大搜索范围（从5→10），增加找到有效栅格的概率
        for (int range = 1; range <= searchRange; range++)
        {
            for (int x = -range; x <= range; x++)
            {
                for (int y = -range; y <= range; y++)
                {
                    if (Mathf.Abs(x) == range || Mathf.Abs(y) == range) // 只检查当前范围的边界
                    {
                        Vector2Int checkGrid = new Vector2Int(originalGrid.x + x, originalGrid.y + y);
                        if (IsValidGrid(checkGrid) && gridManager.栅格是否可通行(checkGrid))
                        {
                            Debug.Log($"在范围 {range} 找到有效栅格: {checkGrid}");
                            return checkGrid;
                        }
                    }
                }
            }
        }

        Debug.LogError($"未找到有效栅格，原始栅格: {originalGrid}");
        return new Vector2Int(-1, -1); // 无效标记
    }

    // 校验栅格是否在边界内
    private bool IsValidGrid(Vector2Int gridPos)
    {
        return gridPos.x >= 0 && gridPos.x < gridWidth &&
               gridPos.y >= 0 && gridPos.y < gridHeight;
    }

    // 核心A*寻路逻辑
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

            // 到达目标，重构并简化路径
            if (current.Equals(target))
            {
                List<Vector2Int> rawPath = ReconstructPath(target);
                return SimplifyPath(rawPath);
            }

            // 遍历邻居
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
                // 更新更优路径
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

    // 重置节点数据
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

    // 获取8方向邻居
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

    // 启发式函数（对角线距离）
    private float CalculateHeuristic(Vector2Int a, Vector2Int b)
    {
        int dx = Mathf.Abs(a.x - b.x);
        int dy = Mathf.Abs(a.y - b.y);
        return (dx + dy + (DIAGONAL_COST - 2) * Mathf.Min(dx, dy)) * cellSizeHeuristic;
    }

    // 计算节点间距离（直走/对角线）
    private float CalculateDistance(Vector2Int a, Vector2Int b)
    {
        int dx = Mathf.Abs(a.x - b.x);
        int dy = Mathf.Abs(a.y - b.y);
        return dx == 0 || dy == 0 ? STRAIGHT_COST : DIAGONAL_COST;
    }

    // 重构路径
    private List<Vector2Int> ReconstructPath(Vector2Int end)
    {
        pathBuffer.Clear();
        Vector2Int current = end;
        int safetyCount = 0;

        while (current.x != -1)
        {
            if (pathBuffer.Contains(current))
            {
                Debug.LogWarning("路径存在循环节点，已中断！");
                break;
            }
            pathBuffer.Add(current);
            current = nodeDataArray[current.x, current.y].Parent;

            // 防死循环
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
        Debug.Log($"路径重构完成，原始节点数：{pathBuffer.Count}，首节点：{pathBuffer[0]}，尾节点：{pathBuffer[pathBuffer.Count - 1]}");
        return new List<Vector2Int>(pathBuffer);
    }

    // 路径简化方法
    private List<Vector2Int> SimplifyPath(List<Vector2Int> path)
    {
        if (path == null || path.Count <= 2) return path;
        List<Vector2Int> simplified = new List<Vector2Int>();
        Vector2Int last = path[0];
        simplified.Add(last);
        Vector2Int directionPrev = path[1] - path[0];

        for (int i = 2; i < path.Count; i++)
        {
            Vector2Int directionCurr = path[i] - path[i - 1];
            // 优化：允许微小方向变化（比如方向向量点积>0.95，视为同一方向）
            // 关键修复：将 Vector2Int 转换为 Vector2（浮点向量）后再调用 normalized
            float dot = Vector2.Dot(((Vector2)directionPrev).normalized, ((Vector2)directionCurr).normalized);
            if (dot < 0.95f) // 方向变化较大时才保留节点
            {
                simplified.Add(path[i - 1]);
                directionPrev = directionCurr;
            }
        }
        simplified.Add(path[path.Count - 1]);
        Debug.Log($"路径简化完成：原始{path.Count}个点 → 简化后{simplified.Count}个点");
        return simplified;
    }
    // NodeData结构体定义
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

    // 二叉堆优先级队列实现
    private class BinaryHeapPriorityQueue
    {
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

            // 替换堆顶元素
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
            HeapItem temp = items[i];
            items[i] = items[j];
            items[j] = temp;

            items[i].Index = i;
            items[j].Index = j;

            nodeMap[items[i].Node] = items[i];
            nodeMap[items[j].Node] = items[j];
        }
    }
}