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
    [SerializeField] private float retryOffsetRange = 2f; // 重试偏移范围（小范围保证随机性）

    private void Start()
    {
        if (gridManager == null)
        {
            Debug.LogError("RandomSpawnManager：GridManager未赋值！");
            return;
        }
        // 等待栅格初始化
        StartCoroutine(WaitForGridInitThenSpawn());
    }

    private IEnumerator WaitForGridInitThenSpawn()
    {
        while (!gridManager.IsGridReady())
        {
            yield return new WaitForSeconds(0.2f);
        }
        GenerateRandomStartAndTarget();
    }

    // 生成随机起点和终点（动态校验+轻量重试）
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

    // 生成单个有效随机位置（核心轻量逻辑）
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

            // 若无效，小范围偏移重试（保持随机性）
            if (retryCount > 0)
            {
                float offsetX = Random.Range(-retryOffsetRange, retryOffsetRange);
                float offsetZ = Random.Range(-retryOffsetRange, retryOffsetRange);
                randomPos.x += offsetX;
                randomPos.z += offsetZ;
                // 限制在范围内，避免偏移出界
                randomPos.x = Mathf.Clamp(randomPos.x, spawnRangeX.x, spawnRangeX.y);
                randomPos.z = Mathf.Clamp(randomPos.z, spawnRangeZ.x, spawnRangeZ.y);
            }

            retryCount++;
        }
        // 校验条件：栅格可通行 + 远离障碍物（轻量校验）
        while (!IsPosValid(randomPos) && retryCount < maxSinglePosRetries);

        // 若多次重试仍无效，返回无效标记
        return retryCount < maxSinglePosRetries ? randomPos : Vector3.negativeInfinity;
    }

    // 轻量校验位置有效性（核心优化点）
    private bool IsPosValid(Vector3 worldPos)
    {
        // 1. 转换为栅格坐标
        Vector2Int gridPos = gridManager.世界转栅格(worldPos);
        // 2. 校验栅格是否有效且可通行（最关键）
        if (!gridManager.IsValidGridPosition(gridPos) || !gridManager.栅格是否可通行(gridPos))
            return false;
        // 3. 简单碰撞检测（只检测自身位置，不检测周围，降低消耗）
        if (Physics.CheckSphere(worldPos, 0.5f, LayerMask.GetMask("Obstacle")))
            return false;
        return true;
    }

    private bool IsInvalidPos(Vector3 pos)
    {
        return pos == Vector3.negativeInfinity;
    }

    // 供外部调用（重新生成路径时）
    public void Regenerate()
    {
        GenerateRandomStartAndTarget();
    }
}