using UnityEngine;
using System.Collections.Generic;
using System.Collections;

public class BoatController : MonoBehaviour
{
    public ImprovedAStar pathfinder;
    public GridManager gridManager;
    [Tooltip("直线运动速度")]
    public float moveSpeed = 1.5f;
    [Tooltip("转向速度")]
    public float rotationSpeed = 1f;
    [Tooltip("路径点切换距离")]
    public float waypointDistance = 1.5f;
    public float endPointSlowRange = 2f;
    public float minEndSpeed = 0.5f;
    public float arriveThreshold = 1f;

    private List<Vector2Int> gridPath;
    private List<Vector3> worldPath = new List<Vector3>();
    private int currentWaypointIndex = 0;
    private Rigidbody rb;
    private bool isReachedEnd = false;
    private float currentSpeed = 0f;
    private int 路径重试次数 = 0;
    private const int 最大重试次数 = 5;
 //   private bool wasPathInvalid = false;
    private bool isPathLoaded = false;
    private Vector3 currentTargetPos;

    void Start()
    {
        rb = GetComponent<Rigidbody>();
        if (rb == null)
        {
            Debug.LogError("无人船缺少Rigidbody组件！");
            return;
        }

        ConfigureRigidbody();

        if (pathfinder == null || gridManager == null)
        {
            Debug.LogError("请关联pathfinder和gridManager！");
            return;
        }

        StartCoroutine(WaitForGridInitThenLoadPath());
    }

    private IEnumerator WaitForGridInitThenLoadPath()
    {
        while (gridManager != null && !gridManager.IsGridReady())
        {
            Debug.Log("栅格未初始化，延迟重试路径加载...");
            yield return new WaitForSeconds(0.5f);
        }
        TryLoadPath();
    }

    private void ConfigureRigidbody()
    {
        rb.useGravity = false;
        rb.interpolation = RigidbodyInterpolation.Interpolate;
        rb.collisionDetectionMode = CollisionDetectionMode.ContinuousDynamic;
        rb.linearDamping = 0.5f;
        rb.angularDamping = 2f;
    }

    private void OnCollisionEnter(Collision collision)
    {
        if (collision.collider.CompareTag("USV") || collision.collider.CompareTag("Obstacle"))
        {
            Debug.LogError($"与{collision.collider.tag}发生碰撞！");
            StopMovement();
            isReachedEnd = true;
            worldPath.Clear();
            Invoke(nameof(ResumeMovement), 1f);
            if (pathfinder != null)
            {
                Invoke(nameof(ReplanPath), 1f);
            }

            // 通知Agent碰撞
            var agent = GetComponent<USV_GlobalRLAgent>();
            if (agent != null)
            {
                agent.AddReward(-50f);
                agent.EndEpisode();
            }
        }
    }

    private void StopMovement()
    {
        rb.linearVelocity = Vector3.zero;
        rb.angularVelocity = Vector3.zero;
        currentSpeed = 0f;
    }

    private void ResumeMovement()
    {
        currentWaypointIndex = Mathf.Max(0, currentWaypointIndex - 1);
        isReachedEnd = false;
        Debug.Log("恢复移动");
    }

    private void ReplanPath()
    {
        if (pathfinder != null)
        {
            Debug.Log("重新规划路径...");
            pathfinder.CalculatePathAfterDelay();
        }
    }

    public void TryLoadPath()
    {
        if (gridManager == null || !gridManager.IsGridReady())
        {
            Debug.Log("栅格未初始化，延迟重试...");
            Invoke(nameof(TryLoadPath), 0.5f);
            return;
        }

        var spawnManager = Object.FindFirstObjectByType<RandomSpawnManager>();
        if (spawnManager == null)
        {
            Debug.LogError("未找到RandomSpawnManager！");
            return;
        }
        Vector3 targetPos = spawnManager.TargetPos;

        if (pathfinder == null)
        {
            Debug.LogError("pathfinder未赋值！");
            return;
        }

        // 替换原方法调用
        pathfinder.CalculatePathAfterDelay();  // 原代码可能为pathfinder.CalculatePath();
        gridPath = pathfinder.path;

        if (gridPath == null || gridPath.Count == 0)
        {
            Debug.LogWarning($"路径数据为空，触发A*重新计算...（第{路径重试次数 + 1}次）");
            pathfinder.CalculatePathAfterDelay();
            路径重试次数++;
            float retryDelay = gridManager != null && !gridManager.IsGridReady() ? 2f : 1f;
            if (路径重试次数 < 最大重试次数)
                Invoke(nameof(TryLoadPath), retryDelay);
            else
                Debug.LogError("路径重试次数达到上限！");
            return;
        }

        worldPath.Clear();
        foreach (var gridPos in gridPath)
        {
            if (!gridManager.IsValidGridPosition(gridPos))
            {
                Debug.LogError($"无效栅格坐标：{gridPos}");
                worldPath.Clear();
                Invoke(nameof(TryLoadPath), 1f);
                return;
            }

            Vector3 worldPos = gridManager.栅格转世界(gridPos);
            if (float.IsNaN(worldPos.x) || float.IsNaN(worldPos.z))
            {
                Debug.LogError($"栅格转世界坐标失败：{gridPos}");
                worldPath.Clear();
                Invoke(nameof(TryLoadPath), 1f);
                return;
            }

            worldPos.y = 0.05f;
            worldPath.Add(worldPos);
        }

        if (worldPath.Count == 0)
        {
            Debug.LogError("路径转换后为空，重试...");
            Invoke(nameof(TryLoadPath), 1f);
            return;
        }

        Debug.Log($"成功读取路径，共{worldPath.Count}个路径点");
        isPathLoaded = true;
        isReachedEnd = false;
        currentWaypointIndex = 0;
        路径重试次数 = 0;
        FaceFirstWaypoint();
        currentTargetPos = worldPath[0];
    }

    private void FaceFirstWaypoint()
    {
        if (worldPath.Count < 2) return;

        Vector3 targetDir = (worldPath[1] - transform.position).normalized;
        transform.rotation = Quaternion.LookRotation(targetDir, Vector3.up);
        Debug.Log($"无人船朝向已修正：{transform.forward}");
    }

    void FixedUpdate()
    {
        transform.position = new Vector3(transform.position.x, 0.05f, transform.position.z);

        if (isReachedEnd || !isPathLoaded || worldPath.Count == 0)
            return;

        if (currentWaypointIndex >= worldPath.Count)
        {
            isReachedEnd = true;
            StopMovement();
            return;
        }

        currentTargetPos = worldPath[currentWaypointIndex];
        Vector3 currentXZ = new Vector3(transform.position.x, 0, transform.position.z);
        Vector3 targetXZ = new Vector3(currentTargetPos.x, 0, currentTargetPos.z);
        float distance = Vector3.Distance(currentXZ, targetXZ);

        // 到达终点判断
        if (currentWaypointIndex == worldPath.Count - 1 && distance < arriveThreshold)
        {
            Debug.Log("已到达终点，停止移动");
            StopMovement();
            isReachedEnd = true;

            // 通知Agent完成任务
            var agent = GetComponent<USV_GlobalRLAgent>();
            if (agent != null)
            {
                agent.SetReward(10.0f);
                agent.EndEpisode();
            }
            return;
        }

        float stopDistance = currentWaypointIndex == worldPath.Count - 1 ? 0.8f : waypointDistance;

        if (distance <= stopDistance)
        {
            if (currentWaypointIndex + 1 < worldPath.Count)
            {
                Vector3 nextTargetXZ = new Vector3(worldPath[currentWaypointIndex + 1].x, 0, worldPath[currentWaypointIndex + 1].z);
                float angleToNext = Vector3.Angle(transform.forward, nextTargetXZ - currentXZ);
                if (angleToNext < 90f)
                {
                    currentWaypointIndex++;
                }
            }
            else
            {
                currentWaypointIndex++;
            }
            return;
        }

        // 转向逻辑
        Vector3 targetDir = (targetXZ - currentXZ).normalized;
        float angle = Vector3.Angle(transform.forward, targetDir);
        float rotationSpeedMultiplier = angle > 90f ? 2f : 1f;
        float maxRotationDelta = 30f * Time.fixedDeltaTime * rotationSpeed * rotationSpeedMultiplier;

        Quaternion targetRot = Quaternion.LookRotation(targetDir, Vector3.up);
        transform.rotation = Quaternion.RotateTowards(transform.rotation, targetRot, maxRotationDelta);

        // 移动逻辑
        float targetSpeed = moveSpeed;
        if (currentWaypointIndex == worldPath.Count - 1 && distance < endPointSlowRange)
        {
            targetSpeed = Mathf.Lerp(minEndSpeed, moveSpeed, distance / endPointSlowRange);
        }

        currentSpeed = Mathf.Lerp(currentSpeed, targetSpeed, 0.1f);
        rb.linearVelocity = transform.forward * currentSpeed + new Vector3(0, rb.linearVelocity.y, 0);
    }
}