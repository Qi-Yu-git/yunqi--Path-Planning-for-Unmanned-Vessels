using UnityEngine;
using System.Collections.Generic;
using System;

/// <summary>
/// 雷达传感器实现类，处理扫描逻辑
/// </summary>
public class USVLidarSensorImpl : USVLidarSensor
{
    [Header("雷达基本参数")]
    public float maxDetectionDistance = 20f;
    public LayerMask obstacleLayer;
    public float raycastHeight = 0.2f;

    [Header("位置偏移设置")]
    public float offsetX = 0f;         // X轴偏移量
    public float offsetY = 0f;         // 新增Y轴偏移
    public float offsetZ = 0f;         // Z轴偏移量
    public bool useParentBinding = true;
    public Transform usvTransform;

    [Header("扫描参数")]
    public int maxConcurrentRays = 30;
    public float scanInterval = 0.05f;
    public Color obstacleGizmoColor = Color.red;

    [Header("传感器初始化")]
    public int defaultSampleCount = 36;

    private int _sampleCount;
    private float[] _distances;
    private Vector3[] _worldPositions;
    private Vector3[] _obstacleVelocities;
    private Dictionary<Collider, Vector3> _lastObstaclePositions = new Dictionary<Collider, Vector3>();
    private float _lastScanTime;
    private int _currentRayIndex;

    void Awake()
    {
        if (_sampleCount == 0)
        {
            Initialize(defaultSampleCount);
            Debug.LogWarning("雷达传感器自动初始化，采样数量为" + defaultSampleCount);
        }
    }

    void FixedUpdate()
    {
        UpdateRadarPosition();
    }

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
        if (_sampleCount == 0)
        {
            Initialize(defaultSampleCount);
            return;
        }

        if (usvTransform == null)
        {
            Debug.LogError("Usv Transform未赋值！");
            return;
        }

        UpdateRadarPosition();  // 确保在Update中也更新位置

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

      //  Debug.Log($"雷达位置：{transform.position}，载体位置：{usvTransform.position}");
    }

    // 新增统一的位置更新方法
    private void UpdateRadarPosition()
    {
        if (usvTransform == null) return;

        // 计算偏移后的位置（原逻辑保留）
        Vector3 offset = new Vector3(offsetX, offsetY, offsetZ);
        Vector3 worldOffset = usvTransform.TransformDirection(offset);
        Vector3 targetPos = usvTransform.position + worldOffset;
        targetPos.y = raycastHeight;

        // 更新雷达位置和旋转（关键：Y轴加180度翻转方向）
        transform.position = targetPos;
        // 在载体旋转基础上，Y轴额外旋转180度，修正方向相反问题
        transform.rotation = usvTransform.rotation * Quaternion.Euler(0, 180, 180);
    }
    private void CastSingleRay(int index)
    {
        float angle = GetAngleByIndex(index);
        Vector3 rayDirection = Quaternion.Euler(0, angle, 0) * transform.forward;
        Vector3 rayOrigin = transform.position;  // 已包含raycastHeight，无需再加

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

    private float GetAngleByIndex(int index)
    {
        float angle = (float)index / _sampleCount * 360f;
        return angle - 180f;
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
        // 确保数组已初始化，若未初始化则使用默认值初始化
        if (_distances == null)
        {
            Initialize(defaultSampleCount);
        }
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
}

public abstract class USVLidarSensor : MonoBehaviour
{
    public abstract void Initialize(int sampleCount);
    public abstract void CompleteScan();
    public abstract float[] GetDistances();
    public abstract Vector3[] GetWorldPositions();
    public abstract Vector3[] GetObstacleVelocities();
}