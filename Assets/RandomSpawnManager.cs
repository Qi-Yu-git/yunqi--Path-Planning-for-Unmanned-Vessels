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
    private int currentMinRockCount;
    private int currentMaxRockCount;
    private void Start()
    {
        currentMinRockCount = minRockCount;
        currentMaxRockCount = maxRockCount;

        if (gridManager == null)
        {
            Debug.LogError("RandomSpawnManager：GridManager未赋值！");
            return;
        }
        // 等待栅格初始化后生成场景元素
        StartCoroutine(WaitForGridInitThenSetup());
    }
    /// 供 Academy 调用，动态调整礁石生成数量范围
    /// </summary>
    public void SetRockCountRange(int newMin, int newMax)
    {
        currentMinRockCount = Mathf.Max(1, newMin); // 防止数量为0
        currentMaxRockCount = Mathf.Max(currentMinRockCount, newMax); // 确保最大值不小于最小值
        Debug.Log($"礁石数量范围更新为：{currentMinRockCount}-{currentMaxRockCount}");
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
    private void GenerateRandomRocks()
    {
        // 从 Academy 同步的范围中随机礁石数量
        int rockCount = Random.Range(currentMinRockCount, currentMaxRockCount + 1);
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
                GameObject newRock = Instantiate(rockPrefab, rockPos, Quaternion.Euler(-90f, Random.Range(0f, 360f), 0f));
                // 应用缩放
                float scale = Random.Range(rockScaleMin, rockScaleMax);
                newRock.transform.localScale = new Vector3(scale, scale, scale);
                // 设置礁石层级
                if (obstacleLayer.value != 0)
                {
                    int layerIndex = GetLayerFromMask(obstacleLayer);
                    newRock.layer = layerIndex;
                }
                spawnedRocks.Add(newRock);
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
        if (startPos != null && Vector3.Distance(rockPos, startPos.position) < rockAvoidDistance)
            return false;
        if (targetPos != null && Vector3.Distance(rockPos, targetPos.position) < rockAvoidDistance)
            return false;

        // 4. 检查是否与其他礁石过近
        foreach (var rock in spawnedRocks)
        {
            if (rock != null && Vector3.Distance(rockPos, rock.transform.position) < 2f)
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


    public void GenerateRandomStartAndTarget()
    {
        // 生成有效起点
        Vector3 validStart = GenerateValidRandomPos();
        if (IsInvalidPos(validStart))
        {
            Debug.LogError("无法生成有效起点！");
            return;
        }

        // 生成有效终点
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
        if (startPos != null)
            startPos.position = validStart;
        if (targetPos != null)
            targetPos.position = validTarget;
        Debug.Log($"起点：{validStart} 终点：{validTarget} 距离：{Vector3.Distance(validStart, validTarget):F2}m");
    }

    // 生成随机起点和终点（保持原有逻辑）
    // 在 RandomSpawnManager.cs 中修改 GenerateValidRandomPos 方法
    private Vector3 GenerateValidRandomPos()
    {
        Vector3 randomPos;
        int retryCount = 0;
        int maxAttempts = 50; // 增加最大尝试次数

        do
        {
            // 生成随机位置，在指定范围内
            float x = Random.Range(spawnRangeX.x, spawnRangeX.y);
            float z = Random.Range(spawnRangeZ.x, spawnRangeZ.y);
            randomPos = new Vector3(x, 0.4f, z);

            // 如果多次尝试失败，扩大搜索范围
            if (retryCount > maxAttempts / 2)
            {
                float expandRange = (retryCount - maxAttempts / 2) * 0.5f;
                x = Random.Range(spawnRangeX.x - expandRange, spawnRangeX.y + expandRange);
                z = Random.Range(spawnRangeZ.x - expandRange, spawnRangeZ.y + expandRange);
                randomPos = new Vector3(x, 0.4f, z);
            }

            retryCount++;
        } while (!IsPosValid(randomPos) && retryCount < maxAttempts);

        // 如果所有尝试都失败，返回一个默认的安全位置
        if (retryCount >= maxAttempts)
        {
            Debug.LogWarning("无法生成有效位置，使用默认位置");
            return new Vector3(0, 0.4f, 0); // 默认位置
        }

        return randomPos;
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