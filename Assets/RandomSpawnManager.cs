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
    public int obstacleCount = 5;
    public float obstacleY = 0.5f;
    public Vector2 fixedSpawnRangeX = new Vector2(-14.5f, 14.5f);
    public Vector2 fixedSpawnRangeZ = new Vector2(-9.5f, 9.5f);

    private List<Vector3> spawnedObstaclePositions = new List<Vector3>();

    void Start()
    {
        SpawnRandomObstacles(); // 生成障碍物
        if (gridManager != null)
            gridManager.重置栅格(); // 重置栅格
        SetRandomStartPos(); // 生成起点
        SetRandomTargetPos(); // 生成目标点
    }

    // 生成随机位置（起点/目标点）
    public Vector3 GetRandomPos()
    {
        float x, z;
        Vector3 randomSpawnPos;
        bool goal_ok = false;

        do
        {
            x = Random.Range(fixedSpawnRangeX.x, fixedSpawnRangeX.y);
            z = Random.Range(fixedSpawnRangeZ.x, fixedSpawnRangeZ.y);
            randomSpawnPos = new Vector3(x, 0.4f, z); // 无人船可见高度
            goal_ok = Check_Pos(x, z);
        } while (!goal_ok);

        return randomSpawnPos;
    }

    // 检查位置是否有效
    public bool Check_Pos(float x, float z)
    {
        // 1. 检查是否在水域范围内
        if (x < fixedSpawnRangeX.x || x > fixedSpawnRangeX.y ||
            z < fixedSpawnRangeZ.x || z > fixedSpawnRangeZ.y)
            return false;

        // 2. 检查是否与障碍物重叠
        float checkRadius = 0.8f;
        foreach (var pos in spawnedObstaclePositions)
        {
            if (Vector3.Distance(new Vector3(x, 0, z), pos) < checkRadius)
                return false;
        }

        // 3. 检查栅格是否可通行
        if (gridManager != null)
        {
            Vector2Int gridPos = gridManager.世界转栅格(new Vector3(x, 0, z));
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
        bool valid = false;
        int tryCount = 0;

        while (!valid && tryCount < 100)
        {
            tryCount++;
            float x = Random.Range(fixedSpawnRangeX.x, fixedSpawnRangeX.y);
            float z = Random.Range(fixedSpawnRangeZ.x, fixedSpawnRangeZ.y);
            pos = new Vector3(x, obstacleY, z);

            valid = IsObstaclePosValid(pos);
        }

        return pos;
    }

    // 检查障碍物位置是否有效
    private bool IsObstaclePosValid(Vector3 pos)
    {
        // 检查是否在水域内
        bool inWater = pos.x >= fixedSpawnRangeX.x && pos.x <= fixedSpawnRangeX.y &&
                       pos.z >= fixedSpawnRangeZ.x && pos.z <= fixedSpawnRangeZ.y;

        // 检查是否在栅格内
        Vector2Int gridPos = gridManager.世界转栅格(pos);
        bool inGrid = gridPos.x >= 0 && gridPos.x < gridManager.栅格宽度 &&
                      gridPos.y >= 0 && gridPos.y < gridManager.栅格高度;

        // 检查是否与其他障碍物重叠
        bool noOverlap = true;
        foreach (var p in spawnedObstaclePositions)
        {
            if (Vector3.Distance(pos, p) < 1f)
            {
                noOverlap = false;
                break;
            }
        }

        return inWater && inGrid && noOverlap;
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
            Vector3 targetPos;
            int tryCount = 0;
            do
            {
                targetPos = GetRandomPos();
                tryCount++;
            } while (Vector3.Distance(targetPos, startPos.position) < 2f && tryCount < 50);

            this.targetPos.position = targetPos;
            Debug.Log($"目标点位置：{targetPos}");
        }
        else
            Debug.LogError("未设置targetPos或startPos！");
    }
}