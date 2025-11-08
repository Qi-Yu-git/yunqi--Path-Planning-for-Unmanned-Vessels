using UnityEngine;
using System.Collections.Generic;
using System;

/// <summary>
/// 激光雷达实现（确保雷达射线可见）
/// </summary>
public class USVLidarSensorImpl : USVLidarSensor
{
    [Header("基础检测配置")]
    public float maxDetectionDistance = 20f; // 增大距离确保可见
    public LayerMask obstacleLayer;
    public float raycastHeight = 0.2f;

    [Header("位置同步")]
    public float offsetX = 0f;
    public float offsetZ = 0f;
    public bool useParentBinding = true;
    public Transform usvTransform;

    [Header("性能与可视化")]
    public int maxConcurrentRays = 30;
    public float scanInterval = 0.05f;
    public Color obstacleGizmoColor = Color.red;
    public Color rayColor = Color.blue; // 射线颜色（默认蓝色）
    [Tooltip("强制显示射线（即使未选中物体）")]
    public bool alwaysShowRays = true; // 新增：始终显示射线

    [Header("调试初始化")]
    [Tooltip("默认采样点数量（若外部未初始化则使用此值）")]
    public int defaultSampleCount = 36; // 确保至少有采样点

    private int _sampleCount;
    private float[] _distances;
    private Vector3[] _worldPositions;
    private Vector3[] _obstacleVelocities;
    private Dictionary<Collider, Vector3> _lastObstaclePositions = new Dictionary<Collider, Vector3>();

    private float _lastScanTime;
    private int _currentRayIndex;

    void Awake()
    {
        // 强制开启雷达显示开关
        alwaysShowRays = true;  // 确保Gizmos始终绘制雷达射线

        // 自动初始化（防止外部未调用Initialize）
        if (_sampleCount == 0)
        {
            Initialize(defaultSampleCount);
            Debug.LogWarning("激光雷达自动初始化，采样点数量：" + defaultSampleCount);
        }
    }

    void FixedUpdate()
    {
        if (!useParentBinding && usvTransform != null)
        {
            Vector3 targetPos = usvTransform.position;
            targetPos.x += offsetX;
            targetPos.z += offsetZ;
            transform.position = targetPos;
            transform.rotation = usvTransform.rotation;
        }
    }

    // 实现抽象方法：初始化
    public override void Initialize(int sampleCount)
    {
        _sampleCount = sampleCount;
        _distances = new float[sampleCount];
        _worldPositions = new Vector3[sampleCount];
        _obstacleVelocities = new Vector3[sampleCount];

        for (int i = 0; i < sampleCount; i++)
        {
            _distances[i] = maxDetectionDistance;
            _worldPositions[i] = transform.position + Quaternion.Euler(0, GetAngleByIndex(i), 0) * Vector3.forward * maxDetectionDistance;
            _obstacleVelocities[i] = Vector3.zero;
        }
    }

    void Update()
    {
        // 确保采样点已初始化
        if (_sampleCount == 0)
        {
            Initialize(defaultSampleCount);
            return;
        }

        if (!useParentBinding && usvTransform != null)
        {
            Vector3 targetPos = usvTransform.position;
            targetPos.x += offsetX;
            targetPos.z += offsetZ;
            transform.position = targetPos;
            transform.rotation = usvTransform.rotation;
        }

        if (Time.time - _lastScanTime < scanInterval) return;

        int raysToCast = Mathf.Min(maxConcurrentRays, _sampleCount - _currentRayIndex);
        for (int i = 0; i < raysToCast; i++)
        {
            CastSingleRay(_currentRayIndex + i);
        }

        _currentRayIndex += raysToCast;
        if (_currentRayIndex >= _sampleCount)
        {
            _currentRayIndex = 0;
            _lastScanTime = Time.time;
        }
    }

    private void CastSingleRay(int index)
    {
        float angle = GetAngleByIndex(index);
        Vector3 rayDirection = Quaternion.Euler(0, angle, 0) * transform.forward;
        Vector3 rayOrigin = transform.position + Vector3.up * raycastHeight;

        if (Physics.Raycast(rayOrigin, rayDirection, out RaycastHit hit, maxDetectionDistance, obstacleLayer))
        {
            _distances[index] = hit.distance;
            _worldPositions[index] = hit.point;

            if (_lastObstaclePositions.TryGetValue(hit.collider, out Vector3 lastPos))
            {
                _obstacleVelocities[index] = (hit.point - lastPos) / Time.deltaTime;
            }
            else
            {
                _obstacleVelocities[index] = Vector3.zero;
            }
            _lastObstaclePositions[hit.collider] = hit.point;
        }
        else
        {
            _distances[index] = maxDetectionDistance;
            _worldPositions[index] = rayOrigin + rayDirection * maxDetectionDistance;
            _obstacleVelocities[index] = Vector3.zero;
        }
    }

    // 修改USVLidarSensorImpl.cs中的GetAngleByIndex方法
    private float GetAngleByIndex(int index)
    {
        // 原逻辑可能导致角度计算错误，添加旋转偏移修正
        float angle = (float)index / _sampleCount * 360f;
        return angle - 180f;  // 确保雷达射线围绕船体360度均匀分布
    }

    public override void CompleteScan()
    {
        while (_currentRayIndex < _sampleCount)
        {
            int raysToCast = Mathf.Min(maxConcurrentRays, _sampleCount - _currentRayIndex);
            for (int i = 0; i < raysToCast; i++)
            {
                CastSingleRay(_currentRayIndex + i);
            }
            _currentRayIndex += raysToCast;
        }
        _currentRayIndex = 0;
        _lastScanTime = Time.time;
    }

    public override float[] GetDistances()
    {
        float[] result = new float[_distances.Length];
        Array.Copy(_distances, result, _distances.Length);
        return result;
    }

    public override Vector3[] GetWorldPositions()
    {
        Vector3[] result = new Vector3[_worldPositions.Length];
        Array.Copy(_worldPositions, result, _worldPositions.Length);
        return result;
    }

    public override Vector3[] GetObstacleVelocities()
    {
        Vector3[] result = new Vector3[_obstacleVelocities.Length];
        Array.Copy(_obstacleVelocities, result, _obstacleVelocities.Length);
        return result;
    }

    // 强化Gizmos显示逻辑
    private void OnDrawGizmos()
    {
        // 始终显示射线（即使未选中物体）
        if (alwaysShowRays)
        {
            DrawRaysGizmos();
        }
    }

    private void OnDrawGizmosSelected()
    {
        // 选中时也显示射线（确保至少一种方式可见）
        DrawRaysGizmos();
    }

    /// <summary>
    /// 单独的射线绘制方法，确保逻辑统一
    /// </summary>
    private void DrawRaysGizmos()
    {
        if (_sampleCount == 0)
        {
            // 未初始化时，用默认采样点绘制
            Gizmos.color = rayColor;
            Vector3 rayOrigin = transform.position + Vector3.up * raycastHeight;
            for (int i = 0; i < defaultSampleCount; i++)
            {
                float angle = (float)i / defaultSampleCount * 360f;
                Vector3 direction = Quaternion.Euler(0, angle, 0) * transform.forward;
                Gizmos.DrawLine(rayOrigin, rayOrigin + direction * maxDetectionDistance);
            }
            return;
        }

        // 已初始化时，绘制实际射线
        Gizmos.color = rayColor;
        Vector3 origin = transform.position + Vector3.up * raycastHeight;
        int step = Mathf.Max(1, _sampleCount / 32); // 绘制更多射线（32条）确保可见
        for (int i = 0; i < _sampleCount; i += step)
        {
            float angle = GetAngleByIndex(i);
            Vector3 direction = Quaternion.Euler(0, angle, 0) * transform.forward;
            Gizmos.DrawLine(origin, origin + direction * maxDetectionDistance);
        }

        // 绘制障碍标记
        Gizmos.color = obstacleGizmoColor;
        if (_worldPositions != null)
        {
            foreach (Vector3 pos in _worldPositions)
            {
                if (Vector3.Distance(origin, pos) < maxDetectionDistance - 0.1f)
                {
                    Gizmos.DrawSphere(pos, 0.15f);
                }
            }
        }
    }
}

/// <summary>
/// 激光雷达抽象基类
/// </summary>
public abstract class USVLidarSensor : MonoBehaviour
{
    public abstract void Initialize(int sampleCount);
    public abstract void CompleteScan();
    public abstract float[] GetDistances();
    public abstract Vector3[] GetWorldPositions();
    public abstract Vector3[] GetObstacleVelocities();
}