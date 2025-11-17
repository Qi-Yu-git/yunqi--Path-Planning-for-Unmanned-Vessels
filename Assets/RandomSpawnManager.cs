using UnityEngine;
using System.Collections.Generic;

public class RandomSpawnManager : MonoBehaviour
{
    [Header("关联核心组件")]
    public GridManager gridManager;
    public Transform startPos; // 无人船起点
    public Transform targetPos; // 目标点
    public GameObject obstaclePrefab; // 障碍物预制体

    [Header("生成配置")]
    public int obstacleCount = 3; // 减少障碍物数量，降低初始难度
    public float obstacleY = 0.5f;
    public Vector2 fixedSpawnRangeX = new Vector2(-14.5f, 14.5f);
    public Vector2 fixedSpawnRangeZ = new Vector2(-9.5f, 9.5f);

    private List<Vector3> spawnedObstaclePositions = new List<Vector3>();
    private float xRangeSize;
    private float zRangeSize;
    private int maxRetryCount = 100; // 统一重试次数配置

    void Start()
    {
        // 预计算范围大小，避免重复计算
        xRangeSize = fixedSpawnRangeX.y - fixedSpawnRangeX.x;
        zRangeSize = fixedSpawnRangeZ.y - fixedSpawnRangeZ.x;

        SpawnRandomObstacles();

        if (gridManager != null)
            gridManager.初始化栅格(); // 修复：调用GridManager新增的“初始化栅格”方法

        SetRandomStartPos();
        SetRandomTargetPos();
    }

    // 生成随机位置（起点/目标点）
    public Vector3 GetRandomPos()
    {
        Vector3 randomSpawnPos;
        int tryCount = 0;
        do
        {
            // 使用更高效的随机位置计算方式
            float x = fixedSpawnRangeX.x + Random.value * xRangeSize;
            float z = fixedSpawnRangeZ.x + Random.value * zRangeSize;
            randomSpawnPos = new Vector3(x, 0.4f, z);
            tryCount++;
        } while (!IsValidSpawnPos(randomSpawnPos) && tryCount < maxRetryCount);
        return randomSpawnPos;
    }

    // 检查位置是否有效（使用缓存的Vector3避免重复创建）
    private bool IsValidSpawnPos(Vector3 pos)
    {
        // 1. 检查是否在水域范围内
        if (pos.x < fixedSpawnRangeX.x || pos.x > fixedSpawnRangeX.y ||
            pos.z < fixedSpawnRangeZ.x || pos.z > fixedSpawnRangeZ.y)
            return false;

        // 2. 检查栅格是否可通行
        if (gridManager != null)
        {
            Vector2Int gridPos = gridManager.世界转栅格(pos);
            if (!gridManager.栅格是否可通行(gridPos))
                return false;
        }

        return true;
    }

    // 生成障碍物
    private void SpawnRandomObstacles()
    {
        if (obstaclePrefab == null || gridManager == null)
        {
            Debug.LogError("缺少障碍物预制体或GridManager！");
            return;
        }
        spawnedObstaclePositions.Clear();
        // 预分配列表容量
        spawnedObstaclePositions.Capacity = obstacleCount;

        for (int i = 0; i < obstacleCount; i++)
        {
            Vector3 pos = GetObstacleRandomPos();
            Instantiate(obstaclePrefab, pos, Quaternion.identity, transform);
            spawnedObstaclePositions.Add(pos);
        }
    }

    // 获取障碍物随机位置
    private Vector3 GetObstacleRandomPos()
    {
        Vector3 pos = new Vector3(0, obstacleY, 0);
        int tryCount = 0;
        while (tryCount < maxRetryCount)
        {
            tryCount++;
            float x = fixedSpawnRangeX.x + Random.value * xRangeSize;
            float z = fixedSpawnRangeZ.x + Random.value * zRangeSize;
            pos.x = x;
            pos.z = z;
            if (IsObstaclePosValid(pos))
                break;
        }
        return pos;
    }

    // 检查障碍物位置是否有效
    private bool IsObstaclePosValid(Vector3 pos)
    {
        // 1. 检查是否在水域内
        if (pos.x < fixedSpawnRangeX.x || pos.x > fixedSpawnRangeX.y ||
            pos.z < fixedSpawnRangeZ.x || pos.z > fixedSpawnRangeZ.y)
            return false;

        // 2. 检查是否在栅格内
        Vector2Int gridPos = gridManager.世界转栅格(pos);
        if (gridPos.x < 0 || gridPos.x >= gridManager.gridWidth ||
            gridPos.y < 0 || gridPos.y >= gridManager.gridHeight)
            return false;

        // 3. 检查是否与其他障碍物重叠（使用平方距离，避免开方运算）
        const float minDistanceSquared = 9f; // 最小间距3米
        foreach (var p in spawnedObstaclePositions)
        {
            float dx = pos.x - p.x;
            float dz = pos.z - p.z;
            if (dx * dx + dz * dz < minDistanceSquared)
                return false;
        }

        return true;
    }

    // 设置起点位置
    private void SetRandomStartPos()
    {
        if (startPos != null)
        {
            startPos.position = GetRandomPos();
            startPos.gameObject.SetActive(true);
            Debug.Log($"无人船起点：{startPos.position}");
        }
        else
            Debug.LogError("未设置startPos！");
    }

    // 设置目标点位置
    private void SetRandomTargetPos()
    {
        if (targetPos != null && startPos != null)
        {
            Vector3 newTargetPos;
            int tryCount = 0;
            const float minDistanceSquared = 25f; // 起点与目标最小距离5米
            float dx, dz; // 声明在循环外部
            do
            {
                newTargetPos = GetRandomPos();
                tryCount++;
                // 平方距离比较，效率更高
                dx = newTargetPos.x - startPos.position.x;
                dz = newTargetPos.z - startPos.position.z;
            } while (dx * dx + dz * dz < minDistanceSquared && tryCount < maxRetryCount);

            targetPos.position = newTargetPos;
            Debug.Log($"目标点位置：{newTargetPos}");
        }
        else
            Debug.LogError("未设置targetPos或startPos！");
    }
}