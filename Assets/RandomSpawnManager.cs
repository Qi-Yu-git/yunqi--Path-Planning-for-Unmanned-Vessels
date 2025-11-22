using UnityEngine;
using System.Collections.Generic;
using System.Collections; // 添加IEnumerator所需的命名空间

public class RandomSpawnManager : MonoBehaviour
{
    public static RandomSpawnManager Instance { get; private set; }

    [Tooltip("岩石预制体")]
    public GameObject rockPrefab;
    [Tooltip("最大岩石数量")]
    public int maxRockCount = 50;
    [Tooltip("岩石生成范围")]
    public Vector2 spawnRange = new Vector2(50, 50);

    public Vector3 TargetPos { get; private set; }
    private GridManager gridManager;
    private List<Vector3> safePositions = new List<Vector3>();


    public void Regenerate()
    {
        ClearExistingRocks(); // 现在已实现该方法
        GenerateRandomRocks();
        GenerateRandomStartAndTarget();
    }

    private void Awake()
    {
        if (Instance == null)
        {
            Instance = this;
        }
        else
        {
            Destroy(gameObject);
        }
    }

    private void Start()
    {
        gridManager = Object.FindFirstObjectByType<GridManager>();
        StartCoroutine(WaitForGridInitThenSetup()); // 修正方法名
    }

    // 修复IEnumerator返回类型（移除泛型参数）
    private IEnumerator WaitForGridInitThenSetup()  // 正确的返回类型是IEnumerator而非IEnumerator<Something>
    {
        while (gridManager == null || !gridManager.IsGridReady())
        {
            yield return new WaitForSeconds(0.2f);
        }

        GenerateSafePositions();
        GenerateRandomRocks();
        GenerateRandomStartAndTarget();
    }

    // 添加缺失的ClearExistingRocks方法
    private void ClearExistingRocks()
    {
        GameObject[] existingRocks = GameObject.FindGameObjectsWithTag("Rock");
        foreach (var rock in existingRocks)
        {
            Destroy(rock);
        }
    }

    // 以下为原有代码（保持不变）
    private void GenerateSafePositions()
    {
        safePositions.Clear();
        for (int x = 0; x < gridManager.gridWidth; x++)
        {
            for (int y = 0; y < gridManager.gridHeight; y++)
            {
                Vector2Int gridPos = new Vector2Int(x, y);
                if (gridManager.栅格是否可通行(gridPos))
                {
                    safePositions.Add(gridManager.栅格转世界(gridPos));
                }
            }
        }
    }

    public void GenerateRandomRocks()
    {
        // 清除现有岩石
        GameObject[] existingRocks = GameObject.FindGameObjectsWithTag("Rock");
        foreach (var rock in existingRocks)
        {
            Destroy(rock);
        }

        if (gridManager == null || !gridManager.IsGridReady() || rockPrefab == null)
            return;

        int spawnCount = 0;
        int tryCount = 0;
        while (spawnCount < maxRockCount && tryCount < maxRockCount * 2)
        {
            Vector2Int randomGrid = new Vector2Int(
                Random.Range(5, gridManager.gridWidth - 5),
                Random.Range(5, gridManager.gridHeight - 5)
            );

            if (gridManager.栅格是否可通行(randomGrid))
            {
                Vector3 worldPos = gridManager.栅格转世界(randomGrid);
                worldPos.y = 0.1f;
                Instantiate(rockPrefab, worldPos, Quaternion.identity);

                // 标记栅格为障碍物
                gridManager.MarkAsObstacle(randomGrid);
                spawnCount++;
            }
            tryCount++;
        }
        Debug.Log($"岩石生成完成，成功生成 {spawnCount}/{maxRockCount} 个");
    }

    public void GenerateRandomStartAndTarget()
    {
        if (safePositions.Count == 0)
        {
            GenerateSafePositions();
        }

        Vector3 startPos = GetRandomSafePosition();
        Vector3 targetPos = GetRandomSafePosition();

        // 确保起点和终点距离足够
        while (Vector3.Distance(startPos, targetPos) < 5.0f && safePositions.Count > 1)
        {
            targetPos = GetRandomSafePosition();
        }

        TargetPos = targetPos;

        // 设置Agent位置
        var agent = Object.FindFirstObjectByType<USV_GlobalRLAgent>();
        if (agent != null)
        {
            agent.transform.position = startPos + new Vector3(0, 0.4f, 0);
        }

        Debug.Log($"起点：{startPos}，终点：{targetPos}，距离：{Vector3.Distance(startPos, targetPos):F2}m");
    }


    private Vector3 GetRandomSafePosition()
    {
        if (safePositions.Count == 0)
        {
            GenerateSafePositions();
        }
        return safePositions[Random.Range(0, safePositions.Count)];
    }
}