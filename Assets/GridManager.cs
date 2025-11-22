using UnityEngine;
using System;

public class GridManager : MonoBehaviour
{
    public static GridManager Instance { get; private set; }

    public int gridWidth;
    public int gridHeight;
    public float cellSize;
    public Vector3 gridOrigin;
    private bool[,] isPassable;
    private bool isGridReady = false;



    // 中文属性映射（适配ImprovedAStar.cs中的中文调用）
    public int 栅格宽度 => gridWidth;
    public int 栅格高度 => gridHeight;
    public float 栅格尺寸 => cellSize;
    public Vector3 栅格原点 => gridOrigin;

    public bool IsGridReady() => isGridReady;
    public bool IsInitialized => isGridReady;

    private void Awake()
    {
        if (Instance == null)
        {
            Instance = this;
            InitializeGrid();
            Debug.Log("GridManager单例创建成功"); // 调试日志
        }
        else
        {
            Destroy(gameObject);
            Debug.LogWarning("检测到重复的GridManager，已自动销毁"); // 重复实例提示
        }
    }

    public void InitializeGrid()
    {
        isPassable = new bool[gridWidth, gridHeight];
        for (int x = 0; x < gridWidth; x++)
        {
            for (int y = 0; y < gridHeight; y++)
            {
                isPassable[x, y] = true; // 初始化为可通行
            }
        }
        isGridReady = true;
        Debug.Log("栅格初始化完成");
    }

    public Vector2Int 世界转栅格(Vector3 worldPos)
    {
        int x = Mathf.FloorToInt((worldPos.x - gridOrigin.x) / cellSize);
        int y = Mathf.FloorToInt((worldPos.z - gridOrigin.z) / cellSize);
        return new Vector2Int(x, y);
    }

    public Vector3 栅格转世界(Vector2Int gridPos)
    {
        return new Vector3(
            gridOrigin.x + gridPos.x * cellSize + cellSize / 2,
            0,
            gridOrigin.z + gridPos.y * cellSize + cellSize / 2
        );
    }

    public bool 栅格是否可通行(Vector2Int gridPos)
    {
        if (gridPos.x < 0 || gridPos.x >= gridWidth || gridPos.y < 0 || gridPos.y >= gridHeight)
            return false;

        return isPassable[gridPos.x, gridPos.y];
    }

    public bool IsValidGridPosition(Vector2Int gridPos)
    {
        return gridPos.x >= 0 && gridPos.x < gridWidth &&
               gridPos.y >= 0 && gridPos.y < gridHeight;
    }

    public void MarkAsObstacle(Vector2Int gridPos)
    {
        if (IsValidGridPosition(gridPos))
        {
            isPassable[gridPos.x, gridPos.y] = false;
        }
    }

    public void 强制刷新栅格()
    {
        // 重新检查所有栅格的通行性（根据实际场景障碍物）
        for (int x = 0; x < gridWidth; x++)
        {
            for (int y = 0; y < gridHeight; y++)
            {
                Vector3 worldPos = 栅格转世界(new Vector2Int(x, y));
                isPassable[x, y] = !Physics.CheckSphere(worldPos, cellSize / 2, LayerMask.GetMask("Obstacle"));
            }
        }
        Debug.Log("栅格已强制刷新");
    }

    public float GetDistanceToNearestObstacle(Vector2Int gridPos)
    {
        float minDistance = float.MaxValue;
        for (int x = 0; x < gridWidth; x++)
        {
            for (int y = 0; y < gridHeight; y++)
            {
                if (!isPassable[x, y])
                {
                    float distance = Vector2.Distance(new Vector2(gridPos.x, gridPos.y), new Vector2(x, y));
                    if (distance < minDistance)
                    {
                        minDistance = distance;
                    }
                }
            }
        }
        return minDistance * cellSize;
    }
}