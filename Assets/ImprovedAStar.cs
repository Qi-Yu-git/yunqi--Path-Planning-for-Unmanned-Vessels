using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ImprovedAStar : MonoBehaviour
{
    [Header("闭环反馈参数 (决策层→规划层)")]
    [Tooltip("安全距离代价权重 (动态调整)")]
    public float safeCostWeight = 3.0f;

    [Tooltip("路径偏移修正")]
    public Vector2 pathOffset = Vector2.zero;

    [Tooltip("是否启用闭环反馈")]
    public bool enableClosedLoopFeedback = true;

    [Header("安全距离代价参数 (A*)")]
    [Tooltip("安全距离代价权重 C_safe")]
    public float safeCostWeightAStar = 3.0f;

    [Tooltip("衰减系数 λ")]
    public float safeCostLambda = 2.0f;

    [Tooltip("安全距离半径 r_safe (米)")]
    public float safeDistanceRadius = 2.0f;

    private DecisionFeedbackManager feedbackManager;

    public Vector3 targetWorldPos;
    private const float WATER_Y_HEIGHT = 0.05f;
    private const int NEIGHBOR_SEARCH_RANGE = 2;
    private const int SAFETY_SEARCH_RANGE = 2;
    private const float DIAGONAL_COST = 1.41421356f;
    private const float STRAIGHT_COST = 1f;

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

    private int gridWidth;
    private int gridHeight;
    private float cellSize;
    private Vector3 gridOrigin;
    private float cellSizeHeuristic;

    private List<Vector2Int> neighborBuffer = new List<Vector2Int>(8);
    private BinaryHeapPriorityQueue openQueue = new BinaryHeapPriorityQueue(1024);
    private NodeData[,] nodeDataArray;
    private List<Vector2Int> pathBuffer = new List<Vector2Int>(256);

    private bool isCalculatingPath = false;

    private void Start()
    {
        if (targetPos != null)
        {
            targetWorldPos = targetPos.position;
        }
        else
        {
            Debug.LogError("ImprovedAStar: 未设置targetPos，路径规划目标为空！");
            targetWorldPos = Vector3.zero;
        }

        Debug.Log("A*路径准备计算（等待目标点生成）");
        if (CheckDependencies())
        {
            StartCoroutine(DelayCalculatePath(0.5f));
        }

        if (enableClosedLoopFeedback)
        {
            feedbackManager = DecisionFeedbackManager.Instance;
            if (feedbackManager == null)
            {
                Debug.LogWarning("[A*] 未找到 DecisionFeedbackManager，闭环反馈禁用");
            }
        }
    }

    /// <summary>
    /// 设置路径偏移修正 (由反馈管理器调用)
    /// </summary>
    public void SetPathOffset(Vector2 offset)
    {
        pathOffset = offset;
    }

    private IEnumerator DelayCalculatePath(float delayTime)
    {
        yield return new WaitForSeconds(delayTime);
        StartCoroutine(CalculatePathCoroutine());
    }

    private void CacheGridParameters()
    {
        gridWidth = gridManager.gridWidth;
        gridHeight = gridManager.gridHeight;
        cellSize = gridManager.gridCellSize;
        gridOrigin = gridManager.gridOrigin;
        cellSizeHeuristic = cellSize * 1.0001f;
    }

    private void InitializeNodeDataArray()
    {
        nodeDataArray = new NodeData[gridWidth, gridHeight];
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

    public void CalculatePathAfterDelay()
    {
        if (isCalculatingPath)
        {
            Debug.Log("路径计算已在进行中，忽略重复请求");
            return;
        }
        StartCoroutine(CalculatePathCoroutine());
    }

    private IEnumerator CalculatePathCoroutine()
    {
        if (targetPos != null)
        {
            targetWorldPos = ClampPositionToGrid(targetPos.position);
        }
        else
        {
            Debug.LogError("A*路径计算失败：targetPos 未赋值！");
            path = null;
            yield break;
        }

        int retryCount = 0;
        while (retryCount < 3)
        {
            if (gridManager == null)
            {
                Debug.LogError("A*路径计算失败：gridManager 未赋值！");
                path = null;
                yield break;
            }

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

            Vector3 startWorldPos = ClampPositionToGrid(startPos.position);
            Vector3 targetWorldPos = ClampPositionToGrid(targetPos.position);
            Vector2Int startGrid = gridManager.WorldToGrid(startWorldPos);
            Vector2Int targetGrid = gridManager.WorldToGrid(targetWorldPos);

            startGrid = FindValidGrid(startGrid, 5);
            targetGrid = FindValidGrid(targetGrid, 5);

            if (startGrid.x == -1 || targetGrid.x == -1)
            {
                Debug.LogError($"第{retryCount + 1}次重试：无法找到有效起点/终点！");
                retryCount++;
                yield return new WaitForSeconds(1f);
                continue;
            }

            startWorldPos = gridManager.GridToWorld(startGrid);
            startWorldPos.y = WATER_Y_HEIGHT;
            startPos.position = startWorldPos;
            targetWorldPos = gridManager.GridToWorld(targetGrid);
            targetWorldPos.y = WATER_Y_HEIGHT;
            targetPos.position = targetWorldPos;

            path = FindPath(startGrid, targetGrid);
            if (path != null && path.Count > 1)
            {
                Debug.Log($"路径计算成功，包含{path.Count}个点");
                isCalculatingPath = false;
                yield break;
            }
            else
            {
                Debug.LogError("路径为空或只有一个点");
                retryCount++;
                yield return new WaitForSeconds(1f);
            }
        }

        isCalculatingPath = false;
        Debug.LogError("A*路径计算失败：3次重试后仍为空！尝试重新生成起点终点...");
        RandomSpawnManager spawnManager = FindFirstObjectByType<RandomSpawnManager>();
        if (spawnManager != null)
        {
            spawnManager.Regenerate();
            yield return new WaitForSeconds(0.3f);
            StartCoroutine(CalculatePathCoroutine());
        }
        path = null;
    }

    public Vector3 ClampPositionToGrid(Vector3 worldPos)
    {
        worldPos.y = WATER_Y_HEIGHT;

        float minX = gridOrigin.x + cellSize * 0.5f;
        float maxX = gridOrigin.x + (gridWidth - 1) * cellSize + cellSize * 0.5f;
        float minZ = gridOrigin.z + cellSize * 0.5f;
        float maxZ = gridOrigin.z + (gridHeight - 1) * cellSize + cellSize * 0.5f;

        float clampedX = Mathf.Clamp(worldPos.x, minX, maxX);
        float clampedZ = Mathf.Clamp(worldPos.z, minZ, maxZ);
        return new Vector3(clampedX, WATER_Y_HEIGHT, clampedZ);
    }

    private Vector2Int FindValidGrid(Vector2Int originalGrid, int searchRange = NEIGHBOR_SEARCH_RANGE)
    {
        if (IsValidGrid(originalGrid) && gridManager.IsGridPassable(originalGrid))
        {
            return originalGrid;
        }

        for (int range = 1; range <= searchRange; range++)
        {
            for (int x = -range; x <= range; x++)
            {
                for (int y = -range; y <= range; y++)
                {
                    if (Mathf.Abs(x) == range || Mathf.Abs(y) == range)
                    {
                        Vector2Int checkGrid = new Vector2Int(originalGrid.x + x, originalGrid.y + y);
                        if (IsValidGrid(checkGrid) && gridManager.IsGridPassable(checkGrid))
                        {
                            return checkGrid;
                        }
                    }
                }
            }
        }

        Debug.LogError($"未找到有效栅格，原始栅格: {originalGrid}");
        return new Vector2Int(-1, -1);
    }

    private bool IsValidGrid(Vector2Int gridPos)
    {
        return gridPos.x >= 0 && gridPos.x < gridWidth &&
               gridPos.y >= 0 && gridPos.y < gridHeight;
    }

    public List<Vector2Int> FindPath(Vector2Int start, Vector2Int target)
    {
        ResetNodeData();
        openQueue.Clear();

        // 如果有路径偏移，修正目标点
        Vector2Int adjustedTarget = target;
        if (pathOffset.magnitude > 0.1f)
        {
            int offsetX = Mathf.RoundToInt(pathOffset.x / cellSize);
            int offsetY = Mathf.RoundToInt(pathOffset.y / cellSize);
            adjustedTarget = new Vector2Int(
                Mathf.Clamp(target.x + offsetX, 0, gridWidth - 1),
                Mathf.Clamp(target.y + offsetY, 0, gridHeight - 1)
            );
            Debug.Log($"[A*] 应用路径偏移: {pathOffset} → 目标偏移至 {adjustedTarget}");
        }

        nodeDataArray[start.x, start.y].GCost = 0;
        float hCost = CalculateHeuristic(start, adjustedTarget);
        nodeDataArray[start.x, start.y].FCost = hCost;
        nodeDataArray[start.x, start.y].InOpenSet = true;
        openQueue.Enqueue(start, hCost);

        while (openQueue.Count > 0)
        {
            Vector2Int current = openQueue.Dequeue();
            nodeDataArray[current.x, current.y].IsClosed = true;
            nodeDataArray[current.x, current.y].InOpenSet = false;

            if (current.Equals(adjustedTarget))
            {
                List<Vector2Int> rawPath = ReconstructPath(adjustedTarget);
                return SimplifyPath(rawPath);
            }

            neighborBuffer.Clear();
            GetNeighbors(current, neighborBuffer);
            foreach (Vector2Int neighbor in neighborBuffer)
            {
                if (nodeDataArray[neighbor.x, neighbor.y].IsClosed)
                    continue;
                if (!gridManager.IsGridPassable(neighbor))
                    continue;

                // ====== 计算安全距离代价 (论文 4.2 节) ======
                float safetyPenalty = CalculateSafetyCost(neighbor);

                float newGCost = nodeDataArray[current.x, current.y].GCost +
                                CalculateDistance(current, neighbor) * cellSize +
                                safetyPenalty;

                if (newGCost < nodeDataArray[neighbor.x, neighbor.y].GCost)
                {
                    nodeDataArray[neighbor.x, neighbor.y].GCost = newGCost;
                    float neighborHCost = CalculateHeuristic(neighbor, adjustedTarget);
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
        return null;
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
        return (dx + dy + (DIAGONAL_COST - 2) * Mathf.Min(dx, dy)) * cellSizeHeuristic;
    }

    private float CalculateDistance(Vector2Int a, Vector2Int b)
    {
        int dx = Mathf.Abs(a.x - b.x);
        int dy = Mathf.Abs(a.y - b.y);
        return dx == 0 || dy == 0 ? STRAIGHT_COST : DIAGONAL_COST;
    }

    /// <summary>
    /// 计算安全距离代价 (论文 4.2 节)
    /// g(n) = g(parent) + d(parent,n) + C_safe * Σ exp(-λ * d(n,k))
    /// </summary>
    private float CalculateSafetyCost(Vector2Int gridPos)
    {
        float penalty = 0f;
        float currentWeight = safeCostWeightAStar;

        // 从反馈管理器获取动态权重 (闭环反馈)
        if (enableClosedLoopFeedback && feedbackManager != null)
        {
            currentWeight = 3.0f * feedbackManager.CurrentFeedback.safeDistanceMultiplier;
        }

        for (int dx = -SAFETY_SEARCH_RANGE; dx <= SAFETY_SEARCH_RANGE; dx++)
        {
            for (int dz = -SAFETY_SEARCH_RANGE; dz <= SAFETY_SEARCH_RANGE; dz++)
            {
                if (dx == 0 && dz == 0) continue;
                Vector2Int check = new Vector2Int(gridPos.x + dx, gridPos.y + dz);
                if (!IsValidGrid(check)) continue;
                if (gridManager.IsGridPassable(check)) continue; // 只对障碍物栅格计算代价

                float d = Mathf.Sqrt(dx * dx + dz * dz) * cellSize;
                if (d < safeDistanceRadius && d > 0.001f)
                {
                    penalty += currentWeight * Mathf.Exp(-safeCostLambda * d);
                }
            }
        }
        return penalty;
    }

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
        return new List<Vector2Int>(pathBuffer);
    }

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
            float dot = Vector2.Dot(((Vector2)directionPrev).normalized, ((Vector2)directionCurr).normalized);
            if (dot < 0.95f)
            {
                simplified.Add(path[i - 1]);
                directionPrev = directionCurr;
            }
        }
        simplified.Add(path[path.Count - 1]);
        return simplified;
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
                int parentIndex = (index - 1) >> 1;
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
                int leftChild = (index << 1) + 1;
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