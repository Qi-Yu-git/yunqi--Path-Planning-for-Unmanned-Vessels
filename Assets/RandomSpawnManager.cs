using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class RandomSpawnManager : MonoBehaviour
{
    [SerializeField] private GridManager gridManager;
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
    public int currentMinRockCount;
    public int currentMaxRockCount;

    [Header("A* 路径关联")]
    public ImprovedAStar pathfinder; // 关联A*脚本
    public Transform boatTransform; // 关联无人船Transform
    public float spawnRange = 50f; // 生成范围（兼容原有大范围配置）

    // 动态生成的起点/目标点（不再通过Inspector赋值）
    private Transform dynamicStartPos;
    private Transform dynamicTargetPos;
    private GameObject markersParent; // 路径标记点父物体

    // 保留访问器方便后续扩展
    public int CurrentMinRockCount => currentMinRockCount;
    public int CurrentMaxRockCount => currentMaxRockCount;

    [Header("启动配置")]
    public bool spawnOnStart = true; // 开关：是否在启动时自动生成

    private void Start()
    {
        currentMinRockCount = minRockCount;
        currentMaxRockCount = maxRockCount;

        if (gridManager == null)
        {
            Debug.LogError("RandomSpawnManager：GridManager未赋值！");
            return;
        }

        if (pathfinder == null)
        {
            Debug.LogError("RandomSpawnManager：ImprovedAStar未赋值！");
            return;
        }

        if (boatTransform == null)
        {
            Debug.LogError("RandomSpawnManager：boatTransform未赋值！");
            return;
        }

        // 仅当spawnOnStart为true时，启动生成流程
        if (spawnOnStart)
        {
            StartCoroutine(WaitForGridInitThenSetup());
        }
        else
        {
            Debug.Log("RandomSpawnManager：已禁用启动时自动生成");
        }
    }

    /// <summary>
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
        float timeout = 10f; // 超时时间10秒
        float timer = 0f;
        while (gridManager == null || !gridManager.IsGridReady())
        {
            timer += Time.deltaTime;
            if (timer > timeout)
            {
                Debug.LogError("RandomSpawnManager：等待GridManager超时，尝试强制初始化栅格");
                if (gridManager != null)
                    gridManager.强制刷新栅格(); // 调用GridManager的强制刷新方法
                timer = 0f; // 重置计时器
            }
            Debug.LogWarning("RandomSpawnManager：等待GridManager初始化...");
            yield return new WaitForSeconds(0.5f);
        }

        // 等待栅格初始化完成
        while (!gridManager.IsGridReady())
        {
            Debug.LogWarning("RandomSpawnManager：等待栅格初始化...");
            yield return new WaitForSeconds(0.2f);
        }

        ClearExistingRocks(); // 清除旧礁石
        SpawnStartAndTarget(); // 生成动态起点/目标点（核心优化）
        GenerateRandomRocks(); // 生成新礁石（依赖起点/目标点位置做避障）
        Debug.Log("RandomSpawnManager：场景初始化完成！");
    }

    /// <summary>
    /// 生成动态起点和目标点（优化：统一管理父物体，修复DontSaveInEditor问题）
    /// </summary>
    void SpawnStartAndTarget()
    {
        // 1. 创建父物体统一管理路径标记点（避免层级混乱）
        markersParent = new GameObject("PathMarkers");
        markersParent.hideFlags = HideFlags.None; // 关键：允许编辑器正常保存
        DontDestroyOnLoad(markersParent); // 场景切换时自动销毁（Unity 2020+兼容）

        // 2. 生成起点（无人船当前位置）
        GameObject startObj = new GameObject("DynamicStartPos");
        startObj.transform.parent = markersParent.transform; // 绑定父物体
        startObj.hideFlags = HideFlags.None;
        startObj.transform.position = boatTransform.position;
        dynamicStartPos = startObj.transform;
        pathfinder.startPos = dynamicStartPos; // 赋值给A*脚本

        // 3. 生成目标点（随机位置，已通过GenerateValidRandomPos校验有效性）
        Vector3 validTargetPos = GenerateValidRandomPos();
        GameObject targetObj = new GameObject("DynamicTargetPos");
        targetObj.transform.parent = markersParent.transform; // 绑定父物体
        targetObj.hideFlags = HideFlags.None;
        targetObj.transform.position = validTargetPos;
        dynamicTargetPos = targetObj.transform;
        pathfinder.targetPos = dynamicTargetPos; // 赋值给A*脚本

        Debug.Log($"生成动态起点：{dynamicStartPos.position}，动态目标点：{dynamicTargetPos.position}");
    }

    // 生成随机礁石
    private void GenerateRandomRocks()
    {
        // 从当前范围中随机礁石数量
        int rockCount = Random.Range(currentMinRockCount, currentMaxRockCount + 1);
        int spawned = 0;
        int maxAttempts = rockCount * 2; // 合理尝试次数
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
                // 实例化礁石
                GameObject newRock = Instantiate(rockPrefab, rockPos, Quaternion.Euler(-90f, Random.Range(0f, 360f), 0f));
                // 应用随机缩放
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
        gridManager.强制刷新栅格(); // 刷新栅格障碍物数据
    }

    // 校验礁石位置是否有效
    private bool IsRockPosValid(Vector3 rockPos)
    {
        // 1. 转换为栅格坐标
        Vector2Int gridPos = gridManager.世界转栅格(rockPos);
        // 2. 检查栅格有效性
        if (!gridManager.IsValidGridPosition(gridPos))
            return false;

        // 3. 检查是否与动态起点/目标点过近
        if (dynamicStartPos != null && Vector3.Distance(rockPos, dynamicStartPos.position) < rockAvoidDistance)
            return false;
        if (dynamicTargetPos != null && Vector3.Distance(rockPos, dynamicTargetPos.position) < rockAvoidDistance)
            return false;

        // 4. 检查是否与其他礁石过近
        foreach (var rock in spawnedRocks)
        {
            if (rock != null && Vector3.Distance(rockPos, rock.transform.position) < 2f)
                return false;
        }

        // 5. 过滤非障碍物层级的碰撞体
        Collider[] colliders = Physics.OverlapSphere(rockPos, 0.5f);
        foreach (var col in colliders)
        {
            if (col.gameObject.layer == LayerMask.NameToLayer("Obstacle"))
            {
                return false; // 避免与已有障碍物重叠
            }
        }

        // 6. 检查该位置是否可通行
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

    /// <summary>
    /// 重新生成整个场景（含动态起点/目标点和礁石）
    /// </summary>
    public void Regenerate()
    {
        // 清除旧数据
        ClearExistingRocks();
        ClearDynamicMarkers();

        // 重新生成
        SpawnStartAndTarget();
        GenerateRandomRocks();
        gridManager.强制刷新栅格();
        Debug.Log("RandomSpawnManager：场景重新生成完成！");
    }

    /// <summary>
    /// 清除动态生成的起点/目标点
    /// </summary>
    private void ClearDynamicMarkers()
    {
        if (markersParent != null)
        {
            DestroyImmediate(markersParent);
            markersParent = null;
        }
        dynamicStartPos = null;
        dynamicTargetPos = null;
        pathfinder.startPos = null;
        pathfinder.targetPos = null;
    }

    // 生成随机有效位置（供目标点使用）
    private Vector3 GenerateValidRandomPos()
    {
        Vector3 randomPos;
        int retryCount = 0;
        int maxAttempts = 50;

        do
        {
            // 优先在小范围内生成，多次失败则扩大范围
            float x = retryCount > maxAttempts / 2
                ? Random.Range(spawnRangeX.x - 5f, spawnRangeX.y + 5f)
                : Random.Range(spawnRangeX.x, spawnRangeX.y);

            float z = retryCount > maxAttempts / 2
                ? Random.Range(spawnRangeZ.x - 5f, spawnRangeZ.y + 5f)
                : Random.Range(spawnRangeZ.x, spawnRangeZ.y);

            randomPos = new Vector3(x, 0.4f, z);
            retryCount++;
        } while (!IsPosValid(randomPos) && retryCount < maxAttempts);

        // 所有尝试失败时返回默认安全位置
        if (retryCount >= maxAttempts)
        {
            Debug.LogWarning("无法生成有效位置，使用默认位置");
            randomPos = new Vector3(0, 0.4f, 0);
        }

        return randomPos;
    }

    // 校验位置有效性
    private bool IsPosValid(Vector3 worldPos)
    {
        Vector2Int gridPos = gridManager.世界转栅格(worldPos);
        if (!gridManager.IsValidGridPosition(gridPos) || !gridManager.栅格是否可通行(gridPos))
            return false;

        if (Physics.CheckSphere(worldPos, 0.5f, obstacleLayer))
            return false;

        // 确保与起点（无人船位置）距离达标
        if (boatTransform != null && Vector3.Distance(worldPos, boatTransform.position) < minStartTargetDistance)
            return false;

        return true;
    }

    private bool IsInvalidPos(Vector3 pos)
    {
        return pos == Vector3.negativeInfinity;
    }

    // 场景退出时清理所有动态生成的对象（兼容旧版本Unity）
    private void OnDestroy()
    {
        ClearExistingRocks();
        ClearDynamicMarkers();
    }
}