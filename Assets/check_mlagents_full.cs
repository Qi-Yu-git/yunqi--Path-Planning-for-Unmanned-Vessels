using System;
using System.Diagnostics;
using System.IO;
using UnityEngine;
using System.Reflection;

// 解决Debug命名冲突的别名
using Debug = UnityEngine.Debug;

public class MLAgentsChecker : MonoBehaviour
{
    [Header("Conda环境配置")]
    public string condaEnvNameOrPath = "mlagents-clean";
    public string condaScriptsPath = @"F:\ProgramData\anaconda3\Scripts";

    [Header("训练配置（核心！）")]
    public string workingDirectory = @"E:\Unity_data\PPUV-YOLOv8";
    public string trainerConfigPath = "Assets/config/trainer_config.yaml";
    public string resultsDirectory = "results";
    public int port = 5004;
    public bool useNoGraphics = true; // 新增：无图形化启动（解决无头服务器超时）

    [Header("调试配置")]
    public int commandTimeout = 60; // 普通命令超时时间（秒）
    public int trainerCheckTimeout = 60; // 延长至60秒，适配Unity<->Python连接
    public int unityConnectTimeout = 120; // Unity主动连接超时（秒）

    // 训练进程
    private Process _trainProcess;
    // 标记进程是否启动（核心修复：确保状态一致性）
    private bool _isTrainProcessStarted = false;
    // MLAgents版本标记（动态适配参数）
    private string _mlagentsVersion = "";
    // 自动生成的唯一RunID
    private string _uniqueRunId;
    // 连接状态标记（补充使用逻辑，消除CS0414警告）
    private bool _isUnityConnected = false;

    void Start()
    {
        // 生成唯一RunID（时间戳+随机数，避免重复）
        _uniqueRunId = $"run_{DateTime.Now:yyyyMMddHHmmss}_{UnityEngine.Random.Range(1000, 9999)}";
        Debug.Log($"📌 自动生成唯一RunID: {_uniqueRunId}");

        // 优先修复YoloLogSettings资源泄漏问题
        FixYoloLogSettingsRootIssue();

        // 适配1.x版本：检查Agent的Behavior配置（反射方式，兼容不同版本）
        CheckAgentBehaviorSettingsForMLAgents1x();

        // 基础环境检查（Conda/Python/MLAgents）
        CheckMLAgentsEnvironment();

        // 测试训练命令（适配不同MLAgents版本参数）
        TestTrainerCommand();

        // 适配1.x版本：启动后主动检查Unity与训练器的连接状态
        Invoke(nameof(CheckUnityTrainerConnectionForMLAgents1x), 5f);
    }

    #region 适配1.x版本：检查Agent Behavior配置（反射方式兼容）
    void CheckAgentBehaviorSettingsForMLAgents1x()
    {
        Debug.Log($"=== 检查Agent Behavior配置（适配ML-Agents 1.x） ===");

        // 修复：使用新版Unity API FindObjectsByType，解决Object命名冲突和泛型错误
        UnityEngine.Object[] agentObjects = UnityEngine.Object.FindObjectsByType<UnityEngine.MonoBehaviour>(FindObjectsSortMode.None);
        System.Collections.Generic.List<Component> agents = new System.Collections.Generic.List<Component>();

        // 筛选出Agent组件（反射方式，避免直接引用MLAgents类型）
        foreach (var obj in agentObjects)
        {
            try
            {
                if (obj.GetType().Name == "Agent" || obj.GetType().FullName.Contains("MLAgents.Agent"))
                {
                    agents.Add(obj as Component);
                }
            }
            catch
            {
                continue;
            }
        }

        if (agents.Count == 0)
        {
            Debug.LogWarning($"⚠️ 场景中未找到任何Agent组件，请确认Agent已正确添加");
            return;
        }

        bool hasInvalidBehavior = false;
        foreach (var agent in agents)
        {
            // 反射获取BehaviorParameters组件（兼容1.x/2.x）
            Component behaviorParams = null;
            try
            {
                // 先尝试2.x的名称
                behaviorParams = agent.GetComponent("BehaviorParameters");
                if (behaviorParams == null)
                {
                    // 再尝试1.x的旧名称/兼容方式
                    behaviorParams = agent.GetComponent("AgentParameters");
                }
            }
            catch
            {
                // 忽略反射错误
            }

            if (behaviorParams == null)
            {
                Debug.LogError($"❌ Agent {agent.name} 缺少Behavior/Agent Parameters组件");
                hasInvalidBehavior = true;
                continue;
            }

            // 反射检查BehaviorType（兼容1.x/2.x）
            try
            {
                var behaviorTypeProp = behaviorParams.GetType().GetProperty("BehaviorType");
                if (behaviorTypeProp != null)
                {
                    object behaviorTypeValue = behaviorTypeProp.GetValue(behaviorParams);
                    // 1.x版本中BehaviorType的Default值可能是"Default"或枚举值0
                    if (behaviorTypeValue.ToString() != "Default" && behaviorTypeValue.ToString() != "0")
                    {
                        Debug.LogError($"❌ Agent {agent.name} 的Behavior Type不是Default（当前：{behaviorTypeValue}）");
                        hasInvalidBehavior = true;
                    }
                }
                else
                {
                    // 1.x旧版本可能没有BehaviorType，检查Brain参数
                    var brainProp = behaviorParams.GetType().GetProperty("Brain");
                    if (brainProp != null && brainProp.GetValue(behaviorParams) == null)
                    {
                        Debug.LogError($"❌ Agent {agent.name} 的Brain参数未设置");
                        hasInvalidBehavior = true;
                    }
                }
            }
            catch (Exception e)
            {
                Debug.LogWarning($"⚠️ 检查Agent {agent.name} 的Behavior配置时出错: {e.Message}");
            }
        }

        if (hasInvalidBehavior)
        {
            Debug.LogError($"💡 解决方案：将所有Agent的Behavior Type设置为Default，或配置正确的Brain参数");
        }
        else
        {
            Debug.Log($"✅ 所有Agent的Behavior Parameters配置正确");
        }
    }
    #endregion

    #region 核心修复1：YoloLogSettings资源泄漏修复
    // 彻底修复YoloLogSettings的DontDestroyOnLoad导致的资源泄漏
    void FixYoloLogSettingsRootIssue()
    {
        try
        {
            GameObject yoloLogObj = GameObject.Find("YoloLogSettings");
            if (yoloLogObj != null)
            {
                // 编辑器模式：直接销毁
                if (!Application.isPlaying)
                {
                    DestroyImmediate(yoloLogObj);
                    Debug.Log("✅ 编辑器模式下已销毁YoloLogSettings，避免DontDestroyOnLoad错误");
                }
                // 运行时：重建对象并绑定到当前物体（随场景销毁）
                else
                {
                    yoloLogObj.transform.SetParent(null);
                    GameObject newObj = Instantiate(yoloLogObj);
                    Destroy(yoloLogObj);
                    newObj.name = "YoloLogSettings_Fixed";
                    newObj.transform.SetParent(transform); // 绑定到当前物体，随场景销毁
                    Debug.Log("✅ 运行时修复YoloLogSettings完成，已绑定到当前物体");
                }
            }
        }
        catch (Exception e)
        {
            Debug.LogWarning($"⚠️ 修复YoloLogSettings时出错: {e.Message}");
        }
    }
    #endregion

    #region 核心修复2：场景/应用退出时安全清理进程
    // 场景销毁时清理资源
    void OnDestroy()
    {
        // 补充使用_isUnityConnected，消除CS0414警告
        if (_isUnityConnected)
        {
            Debug.Log($"ℹ️ 训练连接状态：已连接，退出时终止进程");
        }
        else
        {
            Debug.Log($"ℹ️ 训练连接状态：未连接，退出时清理进程");
        }

        // 严格判断进程状态，避免空引用和重复释放
        if (_isTrainProcessStarted && _trainProcess != null)
        {
            try
            {
                if (!_trainProcess.HasExited)
                {
                    _trainProcess.Kill();
                    Debug.Log("✅ 已终止训练进程");
                }
            }
            catch (InvalidOperationException)
            {
                Debug.Log("ℹ️ 训练进程已释放，无需终止");
            }
            catch (Exception e)
            {
                Debug.LogWarning($"⚠️ 终止进程失败: {e.Message}");
            }
            finally
            {
                _trainProcess.Dispose();
                _trainProcess = null;
                _isTrainProcessStarted = false; // 重置标记
            }
        }

        // 强制清理YoloLogSettings
        if (Application.isPlaying)
        {
            GameObject yoloLogObj = GameObject.Find("YoloLogSettings_Fixed");
            if (yoloLogObj != null) Destroy(yoloLogObj);

            yoloLogObj = GameObject.Find("YoloLogSettings");
            if (yoloLogObj != null) Destroy(yoloLogObj);
        }
    }

    // 应用退出时最终清理
    void OnApplicationQuit()
    {
        if (_isTrainProcessStarted && _trainProcess != null)
        {
            try
            {
                if (!_trainProcess.HasExited) _trainProcess.Kill();
            }
            catch { }
            finally
            {
                _trainProcess.Dispose();
                _trainProcess = null;
                _isTrainProcessStarted = false;
            }
        }
    }
    #endregion

    #region 核心修复3：命令执行（解决同步/异步流冲突+超时问题）
    // 执行命令（返回：退出码、标准输出、错误输出）
    (int exitCode, string stdout, string stderr) RunCmd(string cmd, bool isTrainerCommand = false)
    {
        string fullCmd = BuildCondaCommand(cmd);
        if (string.IsNullOrEmpty(fullCmd))
        {
            return (-1, "", "Conda命令构建失败，请检查condaScriptsPath");
        }

        // 适配Windows/Linux shell
        string shell = Environment.OSVersion.Platform == PlatformID.Win32NT ? "cmd.exe" : "/bin/bash";
        string args = Environment.OSVersion.Platform == PlatformID.Win32NT
            ? $"/c {fullCmd}"
            : $"-c \"{fullCmd}\"";

        var processStartInfo = new ProcessStartInfo
        {
            FileName = shell,
            Arguments = args,
            WorkingDirectory = workingDirectory,
            RedirectStandardOutput = true,
            RedirectStandardError = true,
            UseShellExecute = false,
            CreateNoWindow = true,
            StandardOutputEncoding = System.Text.Encoding.UTF8,
            StandardErrorEncoding = System.Text.Encoding.UTF8,
            // 修复Python编码问题
            EnvironmentVariables = { { "PYTHONIOENCODING", "utf-8" } }
        };

        Process process = new Process { StartInfo = processStartInfo };
        try
        {
            Debug.Log($"📢 执行命令: {fullCmd} (工作目录: {workingDirectory})");
            process.Start();

            // 训练命令特殊处理（后台运行，避免流阻塞）
            if (isTrainerCommand)
            {
                _isTrainProcessStarted = true; // 先标记启动
                _trainProcess = process;

                // 超时等待：进程未退出=启动成功（后台运行）
                bool isAliveAfterWait = process.WaitForExit(trainerCheckTimeout * 1000);
                if (!isAliveAfterWait)
                {
                    Debug.Log($"✅ 训练进程启动成功！端口: {port} RunID: {_uniqueRunId} (后台运行中)");
                    return (0, "训练进程已启动（后台运行）", "");
                }
                // 进程立即退出=启动失败，读取错误信息
                else
                {
                    string stderr = process.StandardError.ReadToEnd();
                    string stdout = process.StandardOutput.ReadToEnd();
                    int exitCode = process.ExitCode;
                    process.Dispose();
                    _trainProcess = null;
                    _isTrainProcessStarted = false; // 重置标记
                    return (exitCode, stdout, stderr);
                }
            }
            // 普通命令：同步读取输出（延长超时）
            else
            {
                bool isCompleted = process.WaitForExit(commandTimeout * 1000);
                if (!isCompleted)
                {
                    process.Kill();
                    string errMsg = $"命令执行超时（{commandTimeout}秒）";
                    Debug.LogWarning($"⚠️ {errMsg}");
                    process.Dispose();
                    return (-2, "", errMsg);
                }

                string stdout = process.StandardOutput.ReadToEnd();
                string stderr = process.StandardError.ReadToEnd();

                if (!string.IsNullOrEmpty(stdout)) Debug.Log($"📝 命令输出:\n{stdout}");
                if (!string.IsNullOrEmpty(stderr)) Debug.Log($"⚠️ 命令错误输出:\n{stderr}");

                int exitCode = process.ExitCode;
                process.Dispose();
                return (exitCode, stdout, stderr);
            }
        }
        catch (Exception e)
        {
            if (process != null) process.Dispose();
            // 异常时重置训练进程标记
            if (isTrainerCommand)
            {
                _isTrainProcessStarted = false;
                _trainProcess = null;
            }
            string errMsg = $"执行命令异常: {e.Message}\n{e.StackTrace}";
            Debug.LogError($"❌ {errMsg}");
            return (-1, "", errMsg);
        }
    }

    // 构建Conda激活命令（适配Windows环境路径优先级）
    string BuildCondaCommand(string targetCmd)
    {
        if (Environment.OSVersion.Platform != PlatformID.Win32NT)
        {
            return $"source ~/.bashrc && conda activate {condaEnvNameOrPath} && {targetCmd}";
        }
        else
        {
            string activateBatPath = Path.Combine(condaScriptsPath, "activate.bat");
            if (!File.Exists(activateBatPath))
            {
                Debug.LogError($"❌ 找不到activate.bat: {activateBatPath}");
                return "";
            }

            // 修复Conda路径优先级问题：强制将conda脚本路径放到PATH最前面
            string condaPathFix = $"set PATH={condaScriptsPath};%PATH% && ";
            return $"{condaPathFix}\"{activateBatPath}\" \"{condaEnvNameOrPath}\" && {targetCmd}";
        }
    }
    #endregion

    #region 适配1.x版本：检查Unity与训练器的连接状态
    void CheckUnityTrainerConnectionForMLAgents1x()
    {
        if (!_isTrainProcessStarted)
        {
            Debug.LogWarning($"⚠️ 训练进程未启动，跳过连接检查");
            return;
        }

        Debug.Log($"=== 检查Unity与训练器连接状态（适配ML-Agents 1.x） ===");
        try
        {
            // 适配1.x版本：反射获取Academy的连接状态
            Type academyType = Type.GetType("Unity.MLAgents.Academy, Unity.MLAgents");
            // 原代码：检测Academy类（1.x）
            if (academyType == null)
            {
                // 替换为检测4.x的核心类 BehaviorParameters
                var behaviorParamsType = Type.GetType("Unity.MLAgents.Policies.BehaviorParameters, Unity.ML-Agents");
                if (behaviorParamsType == null)
                {
                    Debug.LogWarning($"⚠️ 未找到ML-Agents 4.x BehaviorParameters类，5秒后重试...");
                    Invoke(nameof(CheckUnityTrainerConnectionForMLAgents1x), 5f);
                    return;
                }
                // 如果找到4.x类，直接跳过重试（避免无限循环）
                return;
            }

            // 获取Academy实例
            PropertyInfo instanceProp = academyType.GetProperty("Instance", BindingFlags.Public | BindingFlags.Static);
            if (instanceProp == null)
            {
                Debug.LogWarning($"⚠️ Academy没有Instance属性，5秒后重试...");
                Invoke(nameof(CheckUnityTrainerConnectionForMLAgents1x), 5f);
                return;
            }
            object academyInstance = instanceProp.GetValue(null);

            if (academyInstance == null)
            {
                Debug.LogWarning($"⚠️ Academy未初始化，5秒后重试...");
                Invoke(nameof(CheckUnityTrainerConnectionForMLAgents1x), 5f);
                return;
            }

            // 检查连接状态（1.x版本的不同方式）
            bool isConnected = false;
            // 尝试1：检查是否有IsConnected方法
            MethodInfo isConnectedMethod = academyType.GetMethod("IsConnected", BindingFlags.Public | BindingFlags.Instance);
            if (isConnectedMethod != null)
            {
                isConnected = (bool)isConnectedMethod.Invoke(academyInstance, null);
            }
            else
            {
                // 尝试2：检查Academy的状态属性
                PropertyInfo statusProp = academyType.GetProperty("Status", BindingFlags.Public | BindingFlags.Instance);
                if (statusProp != null)
                {
                    object statusValue = statusProp.GetValue(academyInstance);
                    isConnected = statusValue.ToString() == "Running" || statusValue.ToString() == "Connected";
                }
                else
                {
                    // 无法直接检查，通过进程状态间接判断
                    isConnected = _trainProcess != null && !_trainProcess.HasExited;
                    Debug.LogWarning($"ℹ️ 无法直接检查连接状态，通过训练进程状态判断：{(isConnected ? "运行中" : "已退出")}");
                }
            }

            // 更新连接状态标记（消除CS0414警告）
            _isUnityConnected = isConnected;

            if (isConnected)
            {
                Debug.Log($"✅ Unity已成功连接到ML-Agents训练器（端口：{port}）");
            }
            else
            {
                Debug.LogWarning($"⚠️ Unity尚未连接到训练器，等待中...（超时剩余：{unityConnectTimeout}秒）");
                unityConnectTimeout -= 5;
                if (unityConnectTimeout > 0)
                {
                    Invoke(nameof(CheckUnityTrainerConnectionForMLAgents1x), 5f);
                }
                else
                {
                    Debug.LogError($"❌ Unity连接训练器超时！请检查：");
                    Debug.LogError($"1. 训练器进程是否正常运行（任务管理器查看python.exe）");
                    Debug.LogError($"2. 端口{port}是否被防火墙/杀毒软件拦截");
                    Debug.LogError($"3. ML-Agents版本是否与Unity包版本匹配");
                    Debug.LogError($"4. 是否添加了--no-graphics参数（无头环境必需）");
                }
            }
        }
        catch (Exception e)
        {
            Debug.LogError($"⚠️ 连接检查出错：{e.Message}");
            Invoke(nameof(CheckUnityTrainerConnectionForMLAgents1x), 5f);
        }
    }
    #endregion

    #region 环境检查与训练命令测试
    // 基础环境检查（Conda/Python/MLAgents版本）
    void CheckMLAgentsEnvironment()
    {
        Debug.Log($"=== 基础环境检查 ===");
        Debug.Log($"操作系统: {Environment.OSVersion.Platform}");
        Debug.Log($"Conda环境: {condaEnvNameOrPath}");
        Debug.Log($"工作目录: {workingDirectory}");

        // 1. 检查Conda
        var (condaCode, condaOut, condaErr) = RunCmd("conda --version");
        if (condaCode != 0)
        {
            Debug.LogError($"❌ Conda未找到: {condaErr}");
            return;
        }
        Debug.Log($"✅ Conda版本: {condaOut.Trim()}");

        // 2. 检查Python
        string pythonVersion = GetPythonVersion();
        Debug.Log($"Python版本: {pythonVersion}");

        // 3. 检查MLAgents版本（适配参数）
        var (pipCode, pipOut, pipErr) = RunCmd("pip show mlagents");
        _mlagentsVersion = ExtractVersionFromPipShow(pipOut);
        if (pipCode == 0 && !string.IsNullOrEmpty(_mlagentsVersion))
        {
            Debug.Log($"✅ MLAgents版本: {_mlagentsVersion}");
            // 适配1.x版本：简化版本匹配检查
            Debug.Log($"ℹ️ ML-Agents 1.x版本，自动使用--base-port参数");

            if (_mlagentsVersion.StartsWith("1."))
            {
                Debug.Log("ℹ️ 检测到旧版本MLAgents (1.x)，自动使用--base-port参数");
            }
        }
        else
        {
            Debug.LogWarning($"⚠️ 获取MLAgents版本失败: {pipErr}");
        }

        // 4. 跳过mlagents-learn --help检查（避免超时）
        Debug.Log("ℹ️ 跳过mlagents-learn --help检查，直接测试训练命令");
    }

    // 测试训练命令（核心：动态适配MLAgents版本参数 + 唯一RunID + --force + --no-graphics）
    void TestTrainerCommand()
    {
        Debug.Log($"\n=== 训练命令测试 ===");

        // 1. 检查配置文件存在性
        string fullConfigPath = Path.Combine(workingDirectory, trainerConfigPath);
        if (!File.Exists(fullConfigPath))
        {
            Debug.LogError($"❌ 训练配置文件不存在: {fullConfigPath}");
            return;
        }
        Debug.Log($"✅ 配置文件存在: {fullConfigPath}");

        // 2. 动态选择端口参数（1.x用--base-port，新版用--port）
        string portParam = _mlagentsVersion.StartsWith("1.") ? "--base-port" : "--port";

        // 核心修复：
        // 1. 添加--force参数解决重复RunID问题（1.x版本若报错可移除）
        // 2. 添加--no-graphics参数解决无头环境超时
        // 3. 使用唯一RunID
        string trainCmd =
            $"mlagents-learn \"{trainerConfigPath}\" " +
            $"--run-id {_uniqueRunId} " +
            $"--results-dir \"{resultsDirectory}\" " +
            $"{portParam} {port} " +
            $"--force"; // 1.x版本若报错"unrecognized arguments"，请删除此行

        // 新增：无图形化参数（解决无头服务器/后台运行超时）
        if (useNoGraphics)
        {
            trainCmd += " --no-graphics";
            Debug.Log("ℹ️ 已添加--no-graphics参数，适配无图形化环境");
        }

        // 3. 执行训练命令
        var (trainCode, trainOut, trainErr) = RunCmd(trainCmd, true);

        // 4. 结果分析与针对性错误处理
        if (trainCode == 0)
        {
            Debug.Log($"✅ 训练命令启动成功！RunID: {_uniqueRunId} {portParam}: {port}");
            Debug.Log($"ℹ️ Unity将尝试连接{portParam} {port}，请确保防火墙未拦截");
        }
        else if (trainCode == -2)
        {
            Debug.LogWarning($"⚠️ 训练命令启动超时，检查{portParam} {port}是否被占用或Conda环境是否正确");
        }
        else
        {
            Debug.LogError($"❌ 训练命令启动失败，错误码: {trainCode}");
            Debug.LogError($"错误详情: {trainErr}");

            // 针对性解决方案提示
            if (trainErr.Contains("Address already in use") || trainErr.Contains("端口") || trainErr.Contains("bind"))
            {
                Debug.LogError($"💡 解决方案：1. 更换端口（如{port + 1}）；2. 执行 netstat -ano | findstr {port} 找到PID后在任务管理器结束进程；3. 等待端口自动释放");
            }
            if (trainErr.Contains("No such file or directory") || trainErr.Contains("找不到文件") || trainErr.Contains("does not exist"))
            {
                Debug.LogError($"💡 解决方案：1. 检查配置文件路径是否正确（当前：{fullConfigPath}）；2. 使用绝对路径替换相对路径；3. 确认文件大小写与系统匹配");
            }
            if (trainErr.Contains("ModuleNotFoundError") || trainErr.Contains("ImportError"))
            {
                Debug.LogError($"💡 解决方案：1. 激活Conda环境执行 pip install --upgrade mlagents=={_mlagentsVersion}；2. 检查环境依赖是否完整（pip install -r requirements.txt）");
            }
            if (trainErr.Contains("unrecognized arguments") || trainErr.Contains("无效的参数"))
            {
                Debug.LogError($"💡 解决方案：1. MLAgents {_mlagentsVersion}不支持{portParam}/--force参数，移除后重试；2. 升级/降级MLAgents到匹配版本；3. 参考官方文档确认参数格式");
            }
            if (trainErr.Contains("Run ID") || trainErr.Contains("already exists"))
            {
                Debug.LogError($"💡 解决方案：已自动添加--force参数，若仍失败请手动删除{resultsDirectory}/{_uniqueRunId}目录后重试");
            }
            if (trainErr.Contains("encoding") || trainErr.Contains("编码"))
            {
                Debug.LogError($"💡 解决方案：已设置PYTHONIOENCODING=utf-8，若仍失败请检查系统区域设置为UTF-8，或在Conda环境执行 chcp 65001");
            }
            // 新增：超时错误针对性提示
            if (trainErr.Contains("UnityTimeOutException") || trainErr.Contains("took too long to respond"))
            {
                Debug.LogError($"💡 超时解决方案：");
                Debug.LogError($"1. 确保所有Agent的Behavior Type设置为Default或配置正确的Brain");
                Debug.LogError($"2. 启用--no-graphics参数（已自动添加）");
                Debug.LogError($"3. 延长超时时间（当前trainerCheckTimeout={trainerCheckTimeout}秒）");
                Debug.LogError($"4. 检查ML-Agents Python包与Unity包版本是否匹配");
                Debug.LogError($"5. 关闭Unity的Console窗口，减少图形渲染压力");
            }
        }
    }

    // 从pip show输出提取版本号
    string ExtractVersionFromPipShow(string pipOutput)
    {
        if (string.IsNullOrEmpty(pipOutput)) return "未知";
        foreach (var line in pipOutput.Split(new[] { Environment.NewLine }, StringSplitOptions.RemoveEmptyEntries))
        {
            if (line.StartsWith("Version:", StringComparison.OrdinalIgnoreCase))
            {
                return line.Split(':', StringSplitOptions.RemoveEmptyEntries)[1].Trim();
            }
        }
        return "解析失败";
    }

    // 获取Python版本
    string GetPythonVersion()
    {
        var (code, outStr, errStr) = RunCmd("python --version");
        return code == 0 ? outStr.Trim() : $"获取失败: {errStr}";
    }
    #endregion
}