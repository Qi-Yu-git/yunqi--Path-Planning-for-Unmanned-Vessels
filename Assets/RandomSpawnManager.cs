using UnityEngine;
using System.Collections.Generic;
using System.Collections;

public class RandomSpawnManager : MonoBehaviour
{
    public static RandomSpawnManager Instance { get; private set; }

    [Tooltip("岩石预制体")]
    public GameObject rockPrefab;
    [Tooltip("最大岩石数量")]
    public int maxRockCount = 50;

    [Header("水域设置")]
    [Tooltip("挂载水域对象（如Plane），用于自动获取水域范围")]
    public Transform waterTransform; // 拖拽水域对象到这里

    public Vector3 TargetPos { get; private set; }
    private GridManager gridManager;
    private List<Vector3> safePositions = new List<Vector3>();

    // 自动计算的水域边界
    private float minWaterX;
    private float maxWaterX;
    private float minWaterZ;
    private float maxWaterZ;
    private float waterYHeight; // 水域高度

    public void Regenerate()
    {
        ClearExistingRocks();
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

        // 初始化水域边界
        if (waterTransform != null)
        {
            CalculateWaterBounds();
        }
        else
        {
            Debug.LogError("请挂载水域对象到 waterTransform！");
            // 默认值防止崩溃
            SetDefaultWaterBounds();
        }

        StartCoroutine(WaitForGridInitThenSetup());
    }

    // 计算水域边界（基于水域对象的尺寸和位置）
    private void CalculateWaterBounds()
    {
        // 获取水域对象的尺寸（假设是Plane，默认在XZ平面）
        Vector3 waterScale = waterTransform.localScale;
        Vector3 waterPos = waterTransform.position;

        // 计算边界（以水域中心为原点，向四周扩展）
        minWaterX = waterPos.x - waterScale.x * 5f; // Plane默认尺寸是10x10，所以乘以5
        maxWaterX = waterPos.x + waterScale.x * 5f;
        minWaterZ = waterPos.z - waterScale.z * 5f;
        maxWaterZ = waterPos.z + waterScale.z * 5f;
        waterYHeight = waterPos.y; // 水域自身的高度

        Debug.Log($"自动计算水域边界：X[{minWaterX:F2},{maxWaterX:F2}], Z[{minWaterZ:F2},{maxWaterZ:F2}], Y[{waterYHeight:F2}]");
    }

    // 设置默认水域边界（防止未挂载水域对象时崩溃）
    private void SetDefaultWaterBounds()
    {
        minWaterX = 0.5f;
        maxWaterX = 49.5f;
        minWaterZ = 0.5f;
        maxWaterZ = 99.5f;
        waterYHeight = 0f;
    }

    private IEnumerator WaitForGridInitThenSetup()
    {
        while (gridManager == null || !gridManager.IsGridReady())
        {
            yield return new WaitForSeconds(0.2f);
        }

        GenerateSafePositions();
        GenerateRandomRocks();
        GenerateRandomStartAndTarget();
    }

    private void ClearExistingRocks()
    {
        GameObject[] existingRocks = GameObject.FindGameObjectsWithTag("Rock");
        foreach (var rock in existingRocks)
        {
            Destroy(rock);
        }
    }

    // 在RandomSpawnManager.cs的GenerateSafePositions方法中修改
    private void GenerateSafePositions()
    {
        safePositions.Clear();
        if (gridManager == null)
        {
            Debug.LogError("GenerateSafePositions失败：GridManager未赋值");
            return;
        }
        if (!gridManager.IsGridReady())
        {
            Debug.LogError("GenerateSafePositions失败：栅格未就绪，等待初始化...");
            // 新增：等待栅格就绪后重试
            StartCoroutine(WaitForGridThenGenerateSafePositions());
            return;
        }

        // 正常生成安全位置的逻辑
        for (int x = 0; x < gridManager.gridWidth; x++)
        {
            for (int y = 0; y < gridManager.gridHeight; y++)
            {
                Vector2Int gridPos = new Vector2Int(x, y);
                if (gridManager.IsValidGridPosition(gridPos) && gridManager.栅格是否可通行(gridPos))
                {
                    Vector3 worldPos = gridManager.栅格转世界(gridPos);
                    if (worldPos.x >= minWaterX && worldPos.x <= maxWaterX &&
                        worldPos.z >= minWaterZ && worldPos.z <= maxWaterZ)
                    {
                        worldPos.y = waterYHeight + 0.05f;
                        safePositions.Add(worldPos);
                    }
                }
            }
        }
        Debug.Log($"生成安全位置完成：共{safePositions.Count}个");
    }

    // 新增：等待栅格就绪后重试生成安全位置
    private IEnumerator WaitForGridThenGenerateSafePositions()
    {
        while (gridManager == null || !gridManager.IsGridReady())
        {
            yield return new WaitForSeconds(0.5f);
        }
        GenerateSafePositions();
    }
    public void GenerateRandomRocks()
    {
        ClearExistingRocks();
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
                // 校验是否在水域内
                if (worldPos.x < minWaterX || worldPos.x > maxWaterX ||
                    worldPos.z < minWaterZ || worldPos.z > maxWaterZ)
                {
                    tryCount++;
                    continue;
                }

                worldPos.y = waterYHeight + 0.1f; // 岩石略高于水域
                Instantiate(rockPrefab, worldPos, Quaternion.identity);
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

        // 设置Agent位置（略高于水域）
        var agent = Object.FindFirstObjectByType<USV_GlobalRLAgent>();
        if (agent != null)
        {
            agent.transform.position = new Vector3(startPos.x, waterYHeight + 0.4f, startPos.z);
        }

        Debug.Log($"起点：{startPos}，终点：{targetPos}，距离：{Vector3.Distance(startPos, targetPos):F2}m");
    }

    public void ClearAllRocks()
    {
        GameObject[] rocks = GameObject.FindGameObjectsWithTag("Rock");
        foreach (var rock in rocks)
        {
            Destroy(rock);
        }
    }

    private Vector3 GetRandomSafePosition()
    {
        if (safePositions.Count == 0)
        {
            GenerateSafePositions();
            // 若仍为空，返回水域中心
            if (safePositions.Count == 0)
            {
                Vector3 centerPos = new Vector3(
                    (minWaterX + maxWaterX) / 2f,
                    waterYHeight + 0.05f,
                    (minWaterZ + maxWaterZ) / 2f
                );
                Debug.LogWarning("安全位置为空，返回水域中心：" + centerPos);
                return centerPos;
            }
        }

        return safePositions[Random.Range(0, safePositions.Count)];
    }

    // Gizmo绘制水域边界（方便调试）
    private void OnDrawGizmosSelected()
    {
        if (waterTransform != null)
        {
            Gizmos.color = Color.blue;
            Vector3 center = new Vector3(
                (minWaterX + maxWaterX) / 2f,
                waterYHeight,
                (minWaterZ + maxWaterZ) / 2f
            );
            Vector3 size = new Vector3(
                maxWaterX - minWaterX,
                0.1f,
                maxWaterZ - minWaterZ
            );
            Gizmos.DrawWireCube(center, size);
        }
    }
}