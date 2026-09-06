using UnityEngine;
using System;
using System.Collections;
using System.Collections.Generic;
using YoloV8Detection;
using USVGridSystem;
/// <summary>
/// 无人船自主移动控制（手动优先+自动随机移动）- 适配GridManager栅格系统
/// </summary>
[RequireComponent(typeof(Collider))]
public class USV_AutoMovement : MonoBehaviour
{
    [Header("移动基础设置")]
    [Tooltip("无人船手动移动速度")]
    public float usvMoveSpeed = 5f;
    [Tooltip("离水面高度偏移（固定Y轴）")]
    public float waterHeightOffset = 0.5f;

    [Header("按键控制配置")]
    public KeyCode forwardControlKey = KeyCode.W;
    public KeyCode backwardControlKey = KeyCode.S;
    public KeyCode leftControlKey = KeyCode.A;
    public KeyCode rightControlKey = KeyCode.D;

    [Header("自动移动配置")]
    [Tooltip("自动移动速度（建议低于手动速度）")]
    public float autoMoveSpeed = 3f;
    [Tooltip("随机方向更新间隔（秒）")]
    public float randomDirUpdateInterval = 1.5f;
    [Tooltip("是否开启障碍躲避（基于GridManager栅格）")]
    public bool enableObstacleAvoid = true;
    [Tooltip("是否限制水域边界（基于GridManager）")]
    public bool enableWaterBoundaryLimit = true;

    // 私有变量
    private Vector3 _randomMoveDir;
    private float _dirUpdateTimer;
    private GridManager _gridManager;
    private bool _isGridReady;

    // 日志配置
    private YoloV8Detection.YoloLogSettings _yoloLogSettings;
    private readonly YoloV8Detection.YoloLogSettings.LogModule _currentModule = YoloV8Detection.YoloLogSettings.LogModule.ObstacleBoat;

    void Start()
    {
        UpdateRandomMoveDir();

        _gridManager = UnityEngine.Object.FindFirstObjectByType<GridManager>();
        InitLogSettings();

        if (_gridManager == null && (enableObstacleAvoid || enableWaterBoundaryLimit))
        {
            LogWarn($"未找到GridManager，障碍躲避/边界限制功能将失效！");
        }

        ValidateParameters();

        if (_gridManager != null && enableObstacleAvoid)
        {
            StartCoroutine(CheckAndFixInitialPosition());
        }
    }

    // ✅ 正确：返回类型是 IEnumerator（非泛型），并且只定义一次
    private IEnumerator CheckAndFixInitialPosition()
    {
        yield return new WaitForSeconds(0.5f);

        if (_gridManager == null) yield break;

        Vector2Int gridPos = _gridManager.WorldToGrid(transform.position);
        if (!_gridManager.IsGridPassable(gridPos))
        {
            LogWarn($"初始位置在障碍物上，尝试移动到安全位置");

            for (int i = 0; i < 20; i++)
            {
                float angle = UnityEngine.Random.Range(0f, 360f) * Mathf.Deg2Rad;
                Vector3 offset = new Vector3(Mathf.Cos(angle), 0, Mathf.Sin(angle)) * 2f;
                Vector3 newPos = transform.position + offset;

                Vector2Int newGridPos = _gridManager.WorldToGrid(newPos);
                if (_gridManager.IsGridPassable(newGridPos))
                {
                    transform.position = new Vector3(newPos.x, waterHeightOffset, newPos.z);
                    LogInfo($"移动到安全位置: {transform.position}");
                    break;
                }
            }
        }
    }

    void Update()
    {
        _isGridReady = _gridManager != null && _gridManager.IsGridReady();

        Vector3 currentPosition = transform.position;
        float xAxisMovement = 0f;
        float zAxisMovement = 0f;

        bool hasManualInput = CheckManualInput();

        if (hasManualInput)
        {
            xAxisMovement = GetManualXMovement();
            zAxisMovement = GetManualZMovement();

            if (enableWaterBoundaryLimit && _isGridReady)
            {
                LimitPositionToWaterBoundary(ref xAxisMovement, ref zAxisMovement, currentPosition);
            }
        }
        else
        {
            AutoRandomMovement(ref xAxisMovement, ref zAxisMovement);
        }

        UpdateUSVPosition(currentPosition, xAxisMovement, zAxisMovement);
    }

    private void InitLogSettings()
    {
        _yoloLogSettings = YoloV8Detection.YoloLogSettings.Instance;

        if (_yoloLogSettings == null)
        {
            Debug.LogWarning("未找到 YoloLogSettings 单例，日志功能将失效！");
        }
    }

    #region 统一日志接口
    private void LogDebug(string message)
    {
        WriteLog(YoloV8Detection.YoloLogSettings.LogLevel.Debug, message);
    }

    private void LogInfo(string message)
    {
        WriteLog(YoloV8Detection.YoloLogSettings.LogLevel.Info, message);
    }

    private void LogWarn(string message)
    {
        WriteLog(YoloV8Detection.YoloLogSettings.LogLevel.Warn, message);
    }

    private void LogError(string message, Exception ex = null)
    {
        var fullMessage = ex == null ? message : $"{message}\n{ex}";
        WriteLog(YoloV8Detection.YoloLogSettings.LogLevel.Error, fullMessage);
    }

    private void LogFatal(string message, Exception ex = null)
    {
        var fullMessage = ex == null ? message : $"{message}\n{ex}";
        WriteLog(YoloV8Detection.YoloLogSettings.LogLevel.Fatal, fullMessage);
    }

    private void WriteLog(YoloV8Detection.YoloLogSettings.LogLevel level, string message)
    {
        if (_yoloLogSettings == null) return;
        if (!_yoloLogSettings.IsModuleEnabled(_currentModule)) return;

        var configLevel = _yoloLogSettings.GetModuleLogLevel(_currentModule);
        if (level < configLevel) return;

        string logContent = $"[{name}] {message}";
        switch (level)
        {
            case YoloV8Detection.YoloLogSettings.LogLevel.Debug:
                Debug.Log(logContent, this);
                break;
            case YoloV8Detection.YoloLogSettings.LogLevel.Info:
                Debug.Log(logContent, this);
                break;
            case YoloV8Detection.YoloLogSettings.LogLevel.Warn:
                Debug.LogWarning(logContent, this);
                break;
            case YoloV8Detection.YoloLogSettings.LogLevel.Error:
                Debug.LogError(logContent, this);
                break;
            case YoloV8Detection.YoloLogSettings.LogLevel.Fatal:
                Debug.LogError($"[FATAL] {logContent}", this);
                break;
            case YoloV8Detection.YoloLogSettings.LogLevel.None:
                break;
        }
    }
    #endregion

    private void ValidateParameters()
    {
        if (usvMoveSpeed < 0)
        {
            LogWarn($"手动移动速度不能为负，已重置为5");
            usvMoveSpeed = 5f;
        }
        if (autoMoveSpeed < 0)
        {
            LogWarn($"自动移动速度不能为负，已重置为3");
            autoMoveSpeed = 3f;
        }
        if (randomDirUpdateInterval <= 0)
        {
            LogWarn($"方向更新间隔必须大于0，已重置为1.5");
            randomDirUpdateInterval = 1.5f;
        }
    }

    private bool CheckManualInput()
    {
        return Input.GetKey(forwardControlKey) || Input.GetKey(backwardControlKey) ||
               Input.GetKey(leftControlKey) || Input.GetKey(rightControlKey);
    }

    private float GetManualXMovement()
    {
        float x = 0f;
        if (Input.GetKey(leftControlKey)) x -= usvMoveSpeed * Time.deltaTime;
        if (Input.GetKey(rightControlKey)) x += usvMoveSpeed * Time.deltaTime;
        return x;
    }

    private float GetManualZMovement()
    {
        float z = 0f;
        if (Input.GetKey(forwardControlKey)) z += usvMoveSpeed * Time.deltaTime;
        if (Input.GetKey(backwardControlKey)) z -= usvMoveSpeed * Time.deltaTime;
        return z;
    }

    private void AutoRandomMovement(ref float xMove, ref float zMove)
    {
        _dirUpdateTimer += Time.deltaTime;
        if (_dirUpdateTimer >= randomDirUpdateInterval)
        {
            UpdateRandomMoveDir();
            _dirUpdateTimer = 0f;
        }

        if (enableObstacleAvoid && _isGridReady)
        {
            AvoidObstaclesByGrid();
        }

        if (enableWaterBoundaryLimit && _isGridReady)
        {
            LimitDirectionToWaterBoundary();
        }

        xMove = Mathf.Clamp(_randomMoveDir.x * autoMoveSpeed * Time.deltaTime, -autoMoveSpeed * Time.deltaTime, autoMoveSpeed * Time.deltaTime);
        zMove = Mathf.Clamp(_randomMoveDir.z * autoMoveSpeed * Time.deltaTime, -autoMoveSpeed * Time.deltaTime, autoMoveSpeed * Time.deltaTime);
    }

    private void UpdateRandomMoveDir()
    {
        float angle = UnityEngine.Random.Range(0f, 360f) * Mathf.Deg2Rad;
        Vector3 newDir = new Vector3(Mathf.Cos(angle), 0, Mathf.Sin(angle));
        _randomMoveDir = newDir.normalized;
    }

    private void AvoidObstaclesByGrid()
    {
        int retryCount = 0;
        const int maxRetry = 5;
        bool isCurrentDirBlocked = IsDirectionBlockedByGrid(_randomMoveDir);

        while (isCurrentDirBlocked && retryCount < maxRetry)
        {
            UpdateRandomMoveDir();
            isCurrentDirBlocked = IsDirectionBlockedByGrid(_randomMoveDir);
            retryCount++;
        }

        if (isCurrentDirBlocked)
        {
            float angle = UnityEngine.Random.Range(0f, 360f) * Mathf.Deg2Rad;
            _randomMoveDir = new Vector3(Mathf.Cos(angle), 0, Mathf.Sin(angle)).normalized;
            LogWarn($"障碍物阻挡，强制生成新方向: {_randomMoveDir}");
        }
    }

    private bool IsDirectionBlockedByGrid(Vector3 direction)
    {
        if (direction == Vector3.zero || !_isGridReady) return false;

        Vector3 nextWorldPos = transform.position + direction * (autoMoveSpeed * Time.deltaTime * 1.5f);

        Vector2Int nextGridPos;
        try
        {
            nextGridPos = _gridManager.WorldToGrid(nextWorldPos);
        }
        catch (System.Exception e)
        {
            LogError($"世界坐标转栅格坐标失败：{e.Message}", e);
            return false;
        }

        return !_gridManager.IsGridPassable(nextGridPos);
    }

    private void LimitDirectionToWaterBoundary()
    {
        Vector3 futurePos = transform.position + _randomMoveDir * (autoMoveSpeed * Time.deltaTime * 2f);

        if (futurePos.x < _gridManager.WaterMinX)
        {
            _randomMoveDir = new Vector3(Mathf.Abs(_randomMoveDir.x), 0, _randomMoveDir.z).normalized;
        }
        else if (futurePos.x > _gridManager.WaterMaxX)
        {
            _randomMoveDir = new Vector3(-Mathf.Abs(_randomMoveDir.x), 0, _randomMoveDir.z).normalized;
        }

        if (futurePos.z < _gridManager.WaterMinZ)
        {
            _randomMoveDir = new Vector3(_randomMoveDir.x, 0, Mathf.Abs(_randomMoveDir.z)).normalized;
        }
        else if (futurePos.z > _gridManager.WaterMaxZ)
        {
            _randomMoveDir = new Vector3(_randomMoveDir.x, 0, -Mathf.Abs(_randomMoveDir.z)).normalized;
        }
    }

    private void LimitPositionToWaterBoundary(ref float xMove, ref float zMove, Vector3 currentPos)
    {
        float futureX = currentPos.x + xMove;
        float futureZ = currentPos.z + zMove;

        if (futureX < _gridManager.WaterMinX)
        {
            xMove = Mathf.Max(_gridManager.WaterMinX - currentPos.x, -usvMoveSpeed * Time.deltaTime);
        }
        else if (futureX > _gridManager.WaterMaxX)
        {
            xMove = Mathf.Min(_gridManager.WaterMaxX - currentPos.x, usvMoveSpeed * Time.deltaTime);
        }

        if (futureZ < _gridManager.WaterMinZ)
        {
            zMove = Mathf.Max(_gridManager.WaterMinZ - currentPos.z, -usvMoveSpeed * Time.deltaTime);
        }
        else if (futureZ > _gridManager.WaterMaxZ)
        {
            zMove = Mathf.Min(_gridManager.WaterMaxZ - currentPos.z, usvMoveSpeed * Time.deltaTime);
        }
    }

    private void UpdateUSVPosition(Vector3 currentPos, float xMove, float zMove)
    {
        float newX = Mathf.Clamp(currentPos.x + xMove, -1000f, 1000f);
        float newZ = Mathf.Clamp(currentPos.z + zMove, -1000f, 1000f);

        if (enableWaterBoundaryLimit && _isGridReady)
        {
            newX = Mathf.Clamp(newX, _gridManager.WaterMinX, _gridManager.WaterMaxX);
            newZ = Mathf.Clamp(newZ, _gridManager.WaterMinZ, _gridManager.WaterMaxZ);
        }

        transform.position = new Vector3(newX, waterHeightOffset, newZ);
    }

    void OnDrawGizmosSelected()
    {
        if (_gridManager != null && enableWaterBoundaryLimit)
        {
            Gizmos.color = new Color(0, 1, 0, 0.2f);
            Vector3 center = new Vector3(
                (_gridManager.WaterMinX + _gridManager.WaterMaxX) / 2,
                waterHeightOffset,
                (_gridManager.WaterMinZ + _gridManager.WaterMaxZ) / 2
            );
            Vector3 size = new Vector3(
                _gridManager.WaterMaxX - _gridManager.WaterMinX,
                0.1f,
                _gridManager.WaterMaxZ - _gridManager.WaterMinZ
            );
            Gizmos.DrawWireCube(center, size);
        }

        if (enableObstacleAvoid)
        {
            Gizmos.color = Color.blue;
            Gizmos.DrawRay(transform.position, _randomMoveDir * 2f);
            Gizmos.DrawWireSphere(transform.position + _randomMoveDir * 2f, 0.2f);
        }
    }
}