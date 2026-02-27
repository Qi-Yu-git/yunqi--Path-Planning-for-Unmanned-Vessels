using UnityEngine;
using YoloV8Detection;
using System.Collections.Generic;
using System.Reflection;

#if UNITY_EDITOR
using UnityEditor;
#endif

namespace YoloV8Detection
{
    [DisallowMultipleComponent]
    [ExecuteInEditMode]
    public class YoloLogSettings : MonoBehaviour
    {
        #region 日志枚举定义（解决所有模块控制相关报错）
        /// <summary>
        /// 日志模块枚举（包含YoloV8Engine，适配YoloV8Engine.cs调用）
        /// </summary>
        public enum LogModule
        {
            YoloV8Engine,    // 核心引擎模块
            YoloDetector,    // 检测器模块
            ObstacleBoat,    // 障碍物船只模块
            OtherModule      // 其他扩展模块
        }

        /// <summary>
        /// 日志级别枚举（适配模块级日志控制）
        /// </summary>
        public enum LogLevel { None, Debug, Info, Warn, Error, Fatal }
        #endregion

        #region 全局+模块级配置字段
        [Header("=== 全局日志控制 ===")]
        [Tooltip("全局日志总开关（禁用后所有模块日志都关闭）")]
        public bool globalLogEnabled = true;

        [Tooltip("全局默认日志级别（模块未单独配置时使用）")]
        public LogLevel globalLogLevel = LogLevel.Info;

        [Header("=== 全局日志级别开关 ===")]
        [Tooltip("是否启用普通信息日志（所有Info级YOLO日志）")]
        public bool enableLogInfo = true;

        [Tooltip("是否启用警告日志（所有Warning级YOLO日志）")]
        public bool enableLogWarning = true;

        [Tooltip("是否启用错误日志（所有Error级YOLO日志）")]
        public bool enableLogError = true;

        [Header("=== 细分日志控制 ===")]
        [Tooltip("是否输出模型加载、维度转换等处理日志")]
        public bool logModelProcessing = false;

        [Tooltip("是否输出检测结果更新/详细NMS结果日志（含0个目标、帧尺寸等）")]
        public bool logDetectionResults = false;

        [Header("=== 日志过滤配置 ===")]
        [Tooltip("只输出指定类别的日志（为空则不限制）")]
        public List<string> logIncludedClasses = new List<string>();

        [Tooltip("排除指定类别的日志（优先级高于包含列表）")]
        public List<string> logExcludedClasses = new List<string>();
        #endregion

        #region 模块级配置缓存（核心：解决模块控制方法缺失）
        // 模块启用状态缓存
        private Dictionary<LogModule, bool> _moduleEnabled = new Dictionary<LogModule, bool>();
        // 模块日志级别缓存
        private Dictionary<LogModule, LogLevel> _moduleLogLevel = new Dictionary<LogModule, LogLevel>();
        #endregion

        #region 配置变更事件（解决原有OnSettingsChanged报错）
        /// <summary>
        /// 配置变更事件（供YoloV8Engine监听）
        /// </summary>
        public event System.Action OnSettingsChanged;

        /// <summary>
        /// 触发配置变更事件
        /// </summary>
        private void TriggerSettingsChanged()
        {
            OnSettingsChanged?.Invoke();
        }
        #endregion

        #region 单例实现
        private static readonly object _lock = new object();
        private static YoloLogSettings _instance;
        public static YoloLogSettings Instance
        {
            get
            {
                if (_instance == null)
                {
                    lock (_lock)
                    {
                        if (_instance == null)
                        {
                            _instance = UnityEngine.Object.FindFirstObjectByType<YoloLogSettings>();
                            if (_instance == null)
                            {
                                GameObject configObj = new GameObject("[YoloLogSettings]");
                                _instance = configObj.AddComponent<YoloLogSettings>();
                                DontDestroyOnLoad(configObj);
                                Debug.Log($"📌 自动创建YoloLogSettings实例（路径：{configObj.name}）");
                            }
                        }
                    }
                }
                return _instance;
            }
        }
        #endregion

        #region 核心补充：Log 方法（解决 CS0117 错误）
        /// <summary>
        /// 统一日志输出方法（带类别过滤，供YoloDetector调用）
        /// </summary>
        /// <param name="module">日志模块</param>
        /// <param name="level">日志级别</param>
        /// <param name="message">日志内容</param>
        /// <param name="className">检测类别名称（用于过滤）</param>
        public static void Log(LogModule module, LogLevel level, string message, string className = "")
        {
            if (!Instance.globalLogEnabled) return;

            // 检查模块是否启用
            if (!Instance.IsModuleEnabled(module)) return;

            // 检查日志级别
            LogLevel moduleLevel = Instance.GetModuleLogLevel(module);
            if (level < moduleLevel) return;

            // 检查类别过滤
            if (!string.IsNullOrEmpty(className))
            {
                // 排除列表优先级更高
                if (Instance.logExcludedClasses.Contains(className)) return;
                // 包含列表有内容时，只输出指定类别
                if (Instance.logIncludedClasses.Count > 0 && !Instance.logIncludedClasses.Contains(className)) return;
            }

            // 根据级别输出日志
            switch (level)
            {
                case LogLevel.Info when Instance.enableLogInfo:
                    Debug.Log($"[{module}] {message}");
                    break;
                case LogLevel.Warn when Instance.enableLogWarning:
                    Debug.LogWarning($"[{module}] {message}");
                    break;
                case LogLevel.Error when Instance.enableLogError:
                    Debug.LogError($"[{module}] {message}");
                    break;
                case LogLevel.Debug:
                    Debug.Log($"[DEBUG][{module}] {message}");
                    break;
                case LogLevel.Fatal:
                    Debug.LogError($"[FATAL][{module}] {message}");
                    break;
            }
        }

        /// <summary>
        /// 简化重载：默认Info级别+YoloDetector模块
        /// </summary>
        public static void Log(string message)
        {
            Log(LogModule.YoloDetector, LogLevel.Info, message);
        }

        /// <summary>
        /// 简化重载：指定级别+默认YoloDetector模块
        /// </summary>
        public static void Log(LogLevel level, string message)
        {
            Log(LogModule.YoloDetector, level, message);
        }
        #endregion

        #region 缺失的模块控制方法（核心修复CS1061错误）
        /// <summary>
        /// 检查指定模块是否启用日志（YoloV8Engine.cs调用）
        /// </summary>
        public bool IsModuleEnabled(LogModule module)
        {
            // 全局禁用则直接返回false
            if (!globalLogEnabled) return false;
            // 模块未配置则默认启用
            if (!_moduleEnabled.TryGetValue(module, out bool enabled))
            {
                _moduleEnabled[module] = true; // 初始化默认值
                return true;
            }
            return enabled;
        }

        /// <summary>
        /// 设置指定模块的启用状态（YoloV8Engine.cs调用）
        /// </summary>
        public void SetModuleEnabled(LogModule module, bool enabled)
        {
            _moduleEnabled[module] = enabled;
            TriggerSettingsChanged();
        }

        /// <summary>
        /// 获取指定模块的日志级别（YoloV8Engine.cs调用）
        /// </summary>
        public LogLevel GetModuleLogLevel(LogModule module)
        {
            // 模块未配置则返回全局级别
            if (!_moduleLogLevel.TryGetValue(module, out LogLevel level))
            {
                _moduleLogLevel[module] = globalLogLevel; // 初始化默认值
                return globalLogLevel;
            }
            return level;
        }

        /// <summary>
        /// 设置指定模块的日志级别（YoloV8Engine.cs调用）
        /// </summary>
        public void SetModuleLogLevel(LogModule module, LogLevel level)
        {
            _moduleLogLevel[module] = level;
            TriggerSettingsChanged();
        }

        /// <summary>
        /// 批量设置所有模块的启用状态
        /// </summary>
        public void SetAllModulesEnabled(bool enabled)
        {
            foreach (LogModule module in System.Enum.GetValues(typeof(LogModule)))
            {
                _moduleEnabled[module] = enabled;
            }
            TriggerSettingsChanged();
        }

        /// <summary>
        /// 批量设置所有模块的日志级别
        /// </summary>
        public void SetAllModulesLogLevel(LogLevel level)
        {
            globalLogLevel = level;
            foreach (LogModule module in System.Enum.GetValues(typeof(LogModule)))
            {
                _moduleLogLevel[module] = level;
            }
            TriggerSettingsChanged();
        }
        #endregion

        #region 原有缓存+同步逻辑（兼容原有代码）
        // 缓存当前配置（用于检测面板修改）
        private bool _lastGlobalLogEnabled;
        private LogLevel _lastGlobalLogLevel;
        private bool _lastEnableLogInfo;
        private bool _lastEnableLogWarning;
        private bool _lastEnableLogError;
        private bool _lastLogModelProcessing;
        private bool _lastLogDetectionResults;
        private List<string> _lastLogIncludedClasses = new List<string>();
        private List<string> _lastLogExcludedClasses = new List<string>();

        // 日志模块核心引用
        private YoloDetector _cachedDetector;
        private YoloV8Engine _cachedEngine;

        private void Awake()
        {
            if (_instance != null && _instance != this)
            {
#if UNITY_EDITOR
                Debug.LogWarning($"⚠️ 场景中已存在YoloLogSettings实例（{_instance.gameObject.name}），当前实例（{gameObject.name}）将被销毁！");
#endif
                DestroyImmediate(this);
                return;
            }
            _instance = this;
            DontDestroyOnLoad(gameObject);
            // 初始化模块配置缓存
            InitModuleConfig();
            // 【核心修改】默认关闭高频检测日志，解决刷屏
            logDetectionResults = false;
            logModelProcessing = false;
            enableLogInfo = false; // 关闭普通Info日志，只保留Warn/Error
                                   // 初始化缓存
            CacheCurrentSettings();
            // 预加载核心引用
            PreloadCoreReferences();
            // 启动时强制同步一次配置
            SyncSettingsToEngineImmediately();
        }


        /// <summary>
        /// 初始化模块配置默认值
        /// </summary>
        private void InitModuleConfig()
        {
            foreach (LogModule module in System.Enum.GetValues(typeof(LogModule)))
            {
                if (!_moduleEnabled.ContainsKey(module))
                    _moduleEnabled[module] = true;
                if (!_moduleLogLevel.ContainsKey(module))
                    _moduleLogLevel[module] = globalLogLevel;
            }
        }

        private void Start()
        {
            SyncSettingsToEngineImmediately();
            ResetUnityLogFilter();
        }

#if UNITY_EDITOR
        private void OnEnable()
        {
            EditorApplication.update += SyncSettingsInEditor;
            SyncSettingsToEngineImmediately();
        }

        private void OnDisable()
        {
            EditorApplication.update -= SyncSettingsInEditor;
        }

        private void SyncSettingsInEditor()
        {
            if (IsSettingsChanged())
            {
                SyncSettingsToEngineImmediately();
                CacheCurrentSettings();
                TriggerSettingsChanged();
            }
        }
#endif

        private void Update()
        {
            if (!Application.isPlaying) return;

            if (IsSettingsChanged())
            {
                SyncSettingsToEngineImmediately();
                CacheCurrentSettings();
                TriggerSettingsChanged();
            }
        }

        private void PreloadCoreReferences()
        {
            _cachedDetector = UnityEngine.Object.FindFirstObjectByType<YoloDetector>();
            if (_cachedDetector == null) return;

            var engineField = typeof(YoloDetector).GetField("_yoloEngine",
                BindingFlags.NonPublic | BindingFlags.Instance);
            if (engineField != null)
            {
                _cachedEngine = engineField.GetValue(_cachedDetector) as YoloV8Engine;
            }
        }

        private void ResetUnityLogFilter()
        {
            if (Debug.unityLogger != null)
            {
                Debug.unityLogger.logEnabled = true;
                Debug.unityLogger.filterLogType = LogType.Log | LogType.Warning | LogType.Error;
            }
        }

        public void SyncSettingsToEngineImmediately()
        {
            if (_cachedDetector == null)
            {
                _cachedDetector = UnityEngine.Object.FindFirstObjectByType<YoloDetector>();
                if (_cachedDetector == null)
                {
                    Debug.LogWarning("⚠️ 未找到YoloDetector组件，无法同步日志配置！");
                    return;
                }
            }

            if (_cachedEngine == null)
            {
                var engineField = typeof(YoloDetector).GetField("_yoloEngine",
                    BindingFlags.NonPublic | BindingFlags.Instance);
                if (engineField == null)
                {
                    Debug.LogError("❌ YoloDetector中未找到私有字段 _yoloEngine！请检查字段名是否匹配");
                    return;
                }
                _cachedEngine = engineField.GetValue(_cachedDetector) as YoloV8Engine;
            }

            if (_cachedEngine == null)
            {
                Debug.LogWarning("⚠️ YoloV8Engine实例为空（模型未加载？），配置将在引擎初始化后自动生效");
                return;
            }

            ApplySettingsToEngine(_cachedEngine);

            if (_cachedEngine.IsInitialized)
            {
                Debug.Log("🔄 YoloLogSettings配置已同步到YOLO引擎！");
            }
            else
            {
                Debug.Log("🔄 YoloLogSettings配置已暂存，引擎初始化后自动生效！");
            }
        }

        private void CacheCurrentSettings()
        {
            _lastGlobalLogEnabled = globalLogEnabled;
            _lastGlobalLogLevel = globalLogLevel;
            _lastEnableLogInfo = enableLogInfo;
            _lastEnableLogWarning = enableLogWarning;
            _lastEnableLogError = enableLogError;
            _lastLogModelProcessing = logModelProcessing;
            _lastLogDetectionResults = logDetectionResults;

            _lastLogIncludedClasses = new List<string>(logIncludedClasses);
            _lastLogExcludedClasses = new List<string>(logExcludedClasses);
        }

        private bool IsSettingsChanged()
        {
            if (_lastGlobalLogEnabled != globalLogEnabled) return true;
            if (_lastGlobalLogLevel != globalLogLevel) return true;
            if (_lastEnableLogInfo != enableLogInfo) return true;
            if (_lastEnableLogWarning != enableLogWarning) return true;
            if (_lastEnableLogError != enableLogError) return true;
            if (_lastLogModelProcessing != logModelProcessing) return true;
            if (_lastLogDetectionResults != logDetectionResults) return true;
            if (!ListEquals(_lastLogIncludedClasses, logIncludedClasses)) return true;
            if (!ListEquals(_lastLogExcludedClasses, logExcludedClasses)) return true;
            return false;
        }

        private bool ListEquals(List<string> a, List<string> b)
        {
            if (a == null || b == null) return a == b;
            if (a.Count != b.Count) return false;

            for (int i = 0; i < a.Count; i++)
            {
                if (a[i] != b[i]) return false;
            }
            return true;
        }

        public void ApplySettingsToEngine(YoloV8Engine engine)
        {
            if (engine == null)
            {
                Debug.LogWarning("❌ YOLO引擎实例为空，配置应用失败！");
                return;
            }

            try
            {
                // 同步核心日志开关
                SetEngineField(engine, "EnableLogInfo", enableLogInfo);
                SetEngineField(engine, "EnableLogWarning", enableLogWarning);
                SetEngineField(engine, "EnableLogError", enableLogError);

                // 同步细分日志控制
                SetEngineField(engine, "LogModelProcessing", logModelProcessing);
                SetEngineField(engine, "LogNmsResults", logDetectionResults);

                // 同步过滤配置
                SetEngineListField(engine, "LogIncludedClasses", logIncludedClasses);
                SetEngineListField(engine, "LogExcludedClasses", logExcludedClasses);

                // 触发引擎配置刷新
                InvokeEngineMethod(engine, "RefreshLogSettings");

                Debug.Log($"✅ YOLO日志配置应用成功 | Info:{enableLogInfo} | Warning:{enableLogWarning} | Error:{enableLogError}");
            }
            catch (System.Exception ex)
            {
                Debug.LogError($"❌ 应用日志配置时出错：{ex.Message}\n{ex.StackTrace}");
            }
        }

        #region 反射辅助方法
        private void SetEngineField(YoloV8Engine engine, string fieldName, object value)
        {
            var field = engine.GetType().GetField(fieldName,
                BindingFlags.Public | BindingFlags.NonPublic | BindingFlags.Instance);
            if (field == null)
            {
                Debug.LogWarning($"⚠️ 引擎中未找到字段 {fieldName}，跳过设置");
                return;
            }
            field.SetValue(engine, value);
        }

        private void SetEngineListField(YoloV8Engine engine, string fieldName, List<string> values)
        {
            var field = engine.GetType().GetField(fieldName,
                BindingFlags.Public | BindingFlags.NonPublic | BindingFlags.Instance);
            if (field == null)
            {
                Debug.LogWarning($"⚠️ 引擎中未找到列表字段 {fieldName}，跳过设置");
                return;
            }

            var list = field.GetValue(engine) as List<string>;
            if (list == null)
            {
                Debug.LogWarning($"⚠️ 列表字段 {fieldName} 类型不匹配，跳过设置");
                return;
            }

            list.Clear();
            list.AddRange(values);
        }

        private void InvokeEngineMethod(YoloV8Engine engine, string methodName)
        {
            var method = engine.GetType().GetMethod(methodName,
                BindingFlags.Public | BindingFlags.NonPublic | BindingFlags.Instance);
            if (method == null) return;
            method.Invoke(engine, null);
        }
        #endregion
        #endregion

        #region 上下文菜单
        [ContextMenu("重置为默认配置")]
        public void ResetToDefault()
        {
            globalLogEnabled = true;
            globalLogLevel = LogLevel.Info;
            enableLogInfo = true;
            enableLogWarning = true;
            enableLogError = true;
            logModelProcessing = false;
            logDetectionResults = false;
            logIncludedClasses.Clear();
            logExcludedClasses.Clear();

            // 重置模块配置
            InitModuleConfig();

            CacheCurrentSettings();
            SyncSettingsToEngineImmediately();
            TriggerSettingsChanged();
            Debug.Log("🔄 YOLO日志配置已重置为默认值");
        }

        [ContextMenu("禁用所有日志")]
        public void DisableAllLogs()
        {
            globalLogEnabled = false;
            enableLogInfo = false;
            enableLogWarning = false;
            enableLogError = false;
            logModelProcessing = false;
            logDetectionResults = false;

            CacheCurrentSettings();
            SyncSettingsToEngineImmediately();
            TriggerSettingsChanged();
            Debug.Log("🚫 所有YOLO日志已禁用");
        }

        [ContextMenu("启用所有日志")]
        public void EnableAllLogs()
        {
            globalLogEnabled = true;
            globalLogLevel = LogLevel.Debug;
            enableLogInfo = true;
            enableLogWarning = true;
            enableLogError = true;
            logModelProcessing = true;
            logDetectionResults = true;

            CacheCurrentSettings();
            SyncSettingsToEngineImmediately();
            TriggerSettingsChanged();
            Debug.Log("✅ 所有YOLO日志已启用");
        }

        [ContextMenu("强制同步配置到引擎")]
        public void ForceSyncSettings()
        {
            _cachedDetector = null;
            _cachedEngine = null;
            SyncSettingsToEngineImmediately();
            TriggerSettingsChanged();
        }

        [ContextMenu("打印当前配置")]
        public void PrintCurrentSettings()
        {
            string log = $"📋 当前YOLO日志配置：\n" +
                         $"全局开关：{globalLogEnabled} | 全局级别：{globalLogLevel}\n" +
                         $"Info日志：{enableLogInfo} | Warning日志：{enableLogWarning} | Error日志：{enableLogError}\n" +
                         $"模型处理日志：{logModelProcessing} | 检测结果日志：{logDetectionResults}\n" +
                         $"包含类别：{(logIncludedClasses.Count > 0 ? string.Join(",", logIncludedClasses) : "无")}\n" +
                         $"排除类别：{(logExcludedClasses.Count > 0 ? string.Join(",", logExcludedClasses) : "无")}";
            Debug.Log(log);
        }
        #endregion
    }
}