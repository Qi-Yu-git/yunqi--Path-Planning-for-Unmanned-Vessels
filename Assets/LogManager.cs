using System;
using System.IO;
using UnityEngine;
using System.Text;

/// <summary>
/// 日志级别枚举
/// </summary>
public enum LogLevel
{
    None,       // 关闭所有日志
    Error,      // 仅错误
    Warning,    // 错误+警告
    Info,       // 错误+警告+信息
    Debug       // 所有日志（兼容自定义Debug级）
}

/// <summary>
/// 全局日志管理类（单例）- 解决YOLO适配警告刷屏问题
/// </summary>
public class LogManager : MonoBehaviour
{
    // 单例实例
    public static LogManager Instance { get; private set; }

    // 日志配置
    [Header("日志配置")]
    [Tooltip("当前日志级别")] public LogLevel currentLogLevel = LogLevel.Warning;
    [Tooltip("是否持久化日志到文件")] public bool saveLogToFile = true;
    [Tooltip("日志文件保存路径（相对PersistentDataPath）")] public string logFilePath = "YoloDetectorLogs/";
    [Tooltip("是否让YOLO适配警告仅输出一次")] public bool yoloAdaptWarnOnce = true;

    // 标记：YOLO适配警告是否已输出过
    private bool _isYoloAdaptWarnOutput = false;
    // 日志文件流
    private StreamWriter _logWriter;
    private string _fullLogPath;

    private void Awake()
    {
        // 单例初始化
        if (Instance != null && Instance != this)
        {
            Destroy(gameObject);
            return;
        }
        Instance = this;
        DontDestroyOnLoad(gameObject);

        // 初始化日志文件
        InitLogFile();

        // 注册Unity日志回调
        Application.logMessageReceived += HandleLog;
    }

    private void OnDestroy()
    {
        // 释放日志回调和文件流
        Application.logMessageReceived -= HandleLog;
        _logWriter?.Flush();
        _logWriter?.Close();
    }

    /// <summary>
    /// 初始化日志文件
    /// </summary>
    private void InitLogFile()
    {
        if (!saveLogToFile) return;

        // 创建目录
        string dirPath = Path.Combine(Application.persistentDataPath, logFilePath);
        if (!Directory.Exists(dirPath))
        {
            Directory.CreateDirectory(dirPath);
        }

        // 生成日志文件名（按日期）
        string fileName = $"YoloDetector_{DateTime.Now:yyyyMMdd_HHmmss}.log";
        _fullLogPath = Path.Combine(dirPath, fileName);

        // 初始化文件流
        _logWriter = new StreamWriter(_fullLogPath, true, Encoding.UTF8);
        _logWriter.AutoFlush = true;
        LogInfo($"日志文件已创建：{_fullLogPath}");
    }

    /// <summary>
    /// 处理Unity日志回调（核心：控制YOLO警告仅输出一次）
    /// </summary>
    private void HandleLog(string logString, string stackTrace, LogType logType)
    {
        // ******** 核心逻辑：YOLO适配警告仅输出一次 ********
        bool isYoloAdaptWarn = logType == LogType.Warning
                             && logString.Contains("检测到模型输出84列（4坐标+80类别），自动适配COCO80类模式");

        if (isYoloAdaptWarn)
        {
            if (yoloAdaptWarnOnce && _isYoloAdaptWarnOutput)
            {
                return; // 已输出过，直接屏蔽
            }
            _isYoloAdaptWarnOutput = true; // 标记为已输出
        }

        // 按日志级别过滤（修正LogType.Debug不存在的问题）
        bool isLogAllowed = logType switch
        {
            LogType.Error => currentLogLevel >= LogLevel.Error,
            LogType.Warning => currentLogLevel >= LogLevel.Warning,
            LogType.Log => currentLogLevel >= LogLevel.Info,
            LogType.Assert => currentLogLevel >= LogLevel.Debug,  // 用Assert替代Debug
            LogType.Exception => currentLogLevel >= LogLevel.Debug,
            _ => true
        };

        if (!isLogAllowed) return;

        // 格式化日志内容
        string formattedLog = FormatLog(logString, stackTrace, logType);

        // 输出到控制台（修正LogType.Debug不存在的问题）
        switch (logType)
        {
            case LogType.Error:
                Debug.LogError(formattedLog);
                break;
            case LogType.Warning:
                Debug.LogWarning(formattedLog);
                break;
            case LogType.Assert:  // 用Assert承载自定义Debug级日志
                Debug.Log(formattedLog);
                break;
            default:
                Debug.Log(formattedLog);
                break;
        }

        // 持久化到文件
        if (saveLogToFile && _logWriter != null)
        {
            _logWriter.WriteLine(formattedLog);
        }
    }

    /// <summary>
    /// 格式化日志内容
    /// </summary>
    private string FormatLog(string logString, string stackTrace, LogType logType)
    {
        StringBuilder sb = new StringBuilder();
        sb.Append($"[{DateTime.Now:yyyy-MM-dd HH:mm:ss.fff}] ");
        sb.Append($"[{logType.ToString().ToUpper()}] ");
        sb.AppendLine(logString);

        // 非Info级别补充堆栈信息
        if (logType != LogType.Log && !string.IsNullOrEmpty(stackTrace))
        {
            sb.AppendLine("堆栈信息：");
            sb.AppendLine(stackTrace);
        }

        // 分隔符
        sb.AppendLine("----------------------------------------------------");
        return sb.ToString();
    }

    // 快捷日志方法（外部调用）
    public static void LogInfo(string message) => Instance?.HandleLog(message, "", LogType.Log);
    public static void LogWarning(string message) => Instance?.HandleLog(message, "", LogType.Warning);
    public static void LogError(string message, string stackTrace = "") => Instance?.HandleLog(message, stackTrace, LogType.Error);
    // 新增自定义Debug日志方法（映射到LogType.Assert）
    public static void LogDebug(string message) => Instance?.HandleLog(message, "", LogType.Assert);
}