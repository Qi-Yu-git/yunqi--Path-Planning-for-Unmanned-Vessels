using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class RandomSpawnManager : MonoBehaviour
{
    [SerializeField] private GridManager gridManager;
    [SerializeField] private Transform startPos;
    [SerializeField] private Transform targetPos;
    [SerializeField] private Vector2 spawnRangeX = new Vector2(-10, 10); // 随机X范围
    [SerializeField] private Vector2 spawnRangeZ = new Vector2(-10, 10); // 随机Z范围
    [SerializeField] private float minStartTargetDistance = 8f; // 起点终点最小距离
    [SerializeField] private int maxSinglePosRetries = 5; // 单个位置最大重试次数
    [SerializeField] private float retryOffsetRange = 2f; // 重试偏移范围

    // 礁石生成配置
    [Header("礁石生成参数")]
    [SerializeField] private GameObject rockPrefab; // 礁石预制体
    [SerializeField] private int minRockCount = 5;   // 最小礁石数量
    [SerializeField] private int maxRockCount = 15;  // 最大礁石数量
    [SerializeField] private float rockScaleMin = 0.8f; // 礁石最小缩放
    [SerializeField] private float rockScaleMax = 1.5f; // 礁石最大缩放
    [SerializeField] private LayerMask obstacleLayer; // 礁石所在层（需包含在GridManager的obstacleLayer中）
    [SerializeField] private float rockAvoidDistance = 3f; // 礁石与起点/终点的安全距离

    private List<GameObject> spawnedRocks = new List<GameObject>(); // 已生成礁石列表

    private void Start()
    {
        if (gridManager == null)
        {
            Debug.LogError("RandomSpawnManager：GridManager未赋值！");
            return;
        }
        // 等待栅格初始化后生成场景元素
        StartCoroutine(WaitForGridInitThenSetup());
    }

    private IEnumerator WaitForGridInitThenSetup()
    {
        while (!gridManager.IsGridReady())
        {
            yield return new WaitForSeconds(0.2f);
        }
        ClearExistingRocks(); // 清除旧礁石
        GenerateRandomRocks(); // 生成新礁石
        GenerateRandomStartAndTarget(); // 生成起点终点
    }

    // 生成随机礁石
    // RandomSpawnManager.cs 礁石生成逻辑修改
    private void GenerateRandomRocks()
    {
        int rockCount = Random.Range(minRockCount, maxRockCount + 1);
        int spawned = 0;
        int maxAttempts = rockCount * 2; // 减少最大尝试次数（原 3 倍，改为 2 倍）
        int attempts = 0;

        while (spawned < rockCount && attempts < maxAttempts)
        {
            attempts++;
            Vector3 rockPos = new Vector3(
                Random.Range(spawnRangeX.x, spawnRangeX.y),
                0.1f,
                Random.Range(spawnRangeZ.x, spawnRangeZ.y)
            );

            if (IsRockPosValid(rockPos))
            {
                // 实例化礁石的逻辑
                GameObject rock = Instantiate(rockPrefab, rockPos, Quaternion.Euler(-90f, Random.Range(0f, 360f), 0f));
                // ... 其他设置 ...
                spawnedRocks.Add(rock);
                spawned++;
            }
        }

        Debug.Log($"生成礁石完成：成功生成 {spawned}/{rockCount} 个，尝试次数 {attempts}");
        gridManager.强制刷新栅格();
    }

    // 校验礁石位置是否有效
    private bool IsRockPosValid(Vector3 rockPos)
    {
        // 1. 转换为栅格坐标
        Vector2Int gridPos = gridManager.世界转栅格(rockPos);
        // 2. 检查栅格有效性
        if (!gridManager.IsValidGridPosition(gridPos))
            return false;

        // 3. 检查是否与起点/终点过近
        if (Vector3.Distance(rockPos, startPos.position) < rockAvoidDistance ||
            Vector3.Distance(rockPos, targetPos.position) < rockAvoidDistance)
            return false;

        // 4. 检查是否与其他礁石过近
        foreach (var rock in spawnedRocks)
        {
            if (Vector3.Distance(rockPos, rock.transform.position) < 2f)
                return false;
        }

        // 5. 检查该位置是否可通行（确保不与已有障碍物重叠）
        return gridManager.栅格是否可通行(gridPos);
    }

    // 清除已生成的礁石
    private void ClearExistingRocks()
    {
        foreach (var rock in spawnedRocks)
        {
            if (rock != null)
                Destroy(rock);
        }
        spawnedRocks.Clear();
    }

    // 从LayerMask获取层级索引
    private int GetLayerFromMask(LayerMask mask)
    {
        int layer = 0;
        int maskValue = mask.value;
        while (maskValue > 1)
        {
            maskValue >>= 1;
            layer++;
        }
        return layer;
    }

    // 生成随机起点和终点（保持原有逻辑）
    public void GenerateRandomStartAndTarget()
    {
        // 生成有效起点
        Vector3 validStart = GenerateValidRandomPos();
        if (IsInvalidPos(validStart))
        {
            Debug.LogError("无法生成有效起点！");
            return;
        }

        // 生成有效终点（确保与起点距离）
        Vector3 validTarget = Vector3.zero;
        int targetRetry = 0;
        do
        {
            validTarget = GenerateValidRandomPos();
            targetRetry++;
        } while (IsInvalidPos(validTarget) ||
                 Vector3.Distance(validStart, validTarget) < minStartTargetDistance &&
                 targetRetry < maxSinglePosRetries * 2);

        if (IsInvalidPos(validTarget))
        {
            Debug.LogError("无法生成有效终点！");
            return;
        }

        // 赋值位置
        startPos.position = validStart;
        targetPos.position = validTarget;
        Debug.Log($"生成起点：{validStart}，终点：{validTarget}（距离：{Vector3.Distance(validStart, validTarget):F2}m）");
    }

    // 生成单个有效随机位置（保持原有逻辑）
    private Vector3 GenerateValidRandomPos()
    {
        Vector3 randomPos;
        int retryCount = 0;

        do
        {
            // 基础随机位置（在指定范围内）
            float x = Random.Range(spawnRangeX.x, spawnRangeX.y);
            float z = Random.Range(spawnRangeZ.x, spawnRangeZ.y);
            randomPos = new Vector3(x, 0.4f, z);

            // 若无效，小范围偏移重试
            if (retryCount > 0)
            {
                float offsetX = Random.Range(-retryOffsetRange, retryOffsetRange);
                float offsetZ = Random.Range(-retryOffsetRange, retryOffsetRange);
                randomPos.x += offsetX;
                randomPos.z += offsetZ;
                // 限制在范围内
                randomPos.x = Mathf.Clamp(randomPos.x, spawnRangeX.x, spawnRangeX.y);
                randomPos.z = Mathf.Clamp(randomPos.z, spawnRangeZ.x, spawnRangeZ.y);
            }

            retryCount++;
        }
        while (!IsPosValid(randomPos) && retryCount < maxSinglePosRetries);

        return retryCount < maxSinglePosRetries ? randomPos : Vector3.negativeInfinity;
    }

    // 校验位置有效性（保持原有逻辑）
    private bool IsPosValid(Vector3 worldPos)
    {
        Vector2Int gridPos = gridManager.世界转栅格(worldPos);
        if (!gridManager.IsValidGridPosition(gridPos) || !gridManager.栅格是否可通行(gridPos))
            return false;
        if (Physics.CheckSphere(worldPos, 0.5f, obstacleLayer))
            return false;
        return true;
    }

    private bool IsInvalidPos(Vector3 pos)
    {
        return pos == Vector3.negativeInfinity;
    }

    // 供外部调用（重新生成场景）
    public void Regenerate()
    {
        ClearExistingRocks();
        GenerateRandomRocks();
        GenerateRandomStartAndTarget();
    }

    // 清理礁石（避免场景残留）
    private void OnDestroy()
    {
        ClearExistingRocks();
    }
}