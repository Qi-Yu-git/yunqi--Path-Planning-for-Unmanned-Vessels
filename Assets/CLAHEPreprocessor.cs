// ============================================================
// 文件名: CLAHEPreprocessor.cs
// 路径: Assets/Scripts/Perception/CLAHEPreprocessor.cs
// 描述: 对比度受限自适应直方图均衡化 (CLAHE)
// 论文: Section 2.3 - 感知层增强
// ============================================================

using UnityEngine;
using OpenCvSharp;
using System;
using System.IO;

/// <summary>
/// CLAHE 图像增强预处理器
/// 在 YOLO 检测前对图像进行对比度增强
/// </summary>
public class CLAHEPreprocessor : MonoBehaviour
{
    [Header("CLAHE 参数 (论文配置)")]
    [Tooltip("对比度限制阈值 (论文: 2.0)")]
    [Range(1.0f, 4.0f)]
    public float clipLimit = 2.0f;

    [Tooltip("分块网格大小 (论文: 8x8)")]
    [Range(4, 16)]
    public int tileGridSize = 8;

    [Tooltip("LAB 空间 L 通道亮度增益")]
    [Range(0.5f, 2.0f)]
    public float luminanceGain = 1.2f;

    [Header("调试选项")]
    [Tooltip("是否保存增强后的图像 (用于调试)")]
    public bool saveEnhancedImage = false;

    [Tooltip("增强图像保存路径")]
    public string savePath = "EnhancedImages/";

    private CLAHE clahe;
    private int frameCount = 0;

    void Start()
    {
        // 初始化 CLAHE
        clahe = Cv2.CreateCLAHE(clipLimit, new Size(tileGridSize, tileGridSize));
        Debug.Log($"[CLAHE] 初始化完成: clipLimit={clipLimit}, tileSize={tileGridSize}x{tileGridSize}");
    }

    /// <summary>
    /// 对输入图像应用 CLAHE 增强
    /// </summary>
    /// <param name="frame">输入 BGR 图像</param>
    /// <returns>增强后的 BGR 图像</returns>
    public Mat ApplyCLAHE(Mat frame)
    {
        if (frame == null || frame.Empty())
        {
            Debug.LogWarning("[CLAHE] 输入图像为空，跳过增强");
            return frame;
        }

        try
        {
            // Step 1: BGR → LAB 颜色空间
            Mat lab = new Mat();
            Cv2.CvtColor(frame, lab, ColorConversionCodes.BGR2Lab);

            // Step 2: 分离 L, A, B 通道
            Mat[] channels = Cv2.Split(lab);
            Mat lChannel = channels[0];
            Mat aChannel = channels[1];
            Mat bChannel = channels[2];

            // Step 3: 对 L 通道应用 CLAHE
            Mat enhancedL = new Mat();
            clahe.Apply(lChannel, enhancedL);

            // Step 4: 亮度增益 (可选)
            if (Mathf.Abs(luminanceGain - 1.0f) > 0.01f)
            {
                enhancedL.ConvertTo(enhancedL, MatType.CV_8UC1, luminanceGain, 0);
            }

            // Step 5: 合并增强后的 L 通道与原始 A, B 通道
            Mat[] enhancedChannels = new Mat[] { enhancedL, aChannel, bChannel };
            Mat enhancedLab = new Mat();
            Cv2.Merge(enhancedChannels, enhancedLab);

            // Step 6: LAB → BGR
            Mat result = new Mat();

            Cv2.CvtColor(enhancedLab, result, (ColorConversionCodes)70);

            // 释放中间 Mat
            lab.Dispose();
            lChannel.Dispose();
            aChannel.Dispose();
            bChannel.Dispose();
            enhancedL.Dispose();
            enhancedLab.Dispose();

            // 调试：保存增强图像
            if (saveEnhancedImage && frameCount % 30 == 0)
            {
                SaveEnhancedImage(result);
            }
            frameCount++;

            return result;
        }
        catch (Exception ex)
        {
            Debug.LogError($"[CLAHE] 增强失败: {ex.Message}\n{ex.StackTrace}");
            return frame; // 失败时返回原图
        }
    }

    /// <summary>
    /// 保存增强图像 (调试用)
    /// </summary>
    private void SaveEnhancedImage(Mat image)
    {
        try
        {
            string dir = Path.Combine(Application.persistentDataPath, savePath);
            if (!Directory.Exists(dir))
                Directory.CreateDirectory(dir);

            string filename = $"enhanced_{DateTime.Now:yyyyMMdd_HHmmss}_{frameCount}.png";
            string fullPath = Path.Combine(dir, filename);
            Cv2.ImWrite(fullPath, image);
            Debug.Log($"[CLAHE] 增强图像已保存: {fullPath}");
        }
        catch (Exception ex)
        {
            Debug.LogWarning($"[CLAHE] 保存图像失败: {ex.Message}");
        }
    }

    /// <summary>
    /// 批量增强 (用于多帧处理)
    /// </summary>
    public Mat[] ApplyCLAHEBatch(Mat[] frames)
    {
        if (frames == null || frames.Length == 0)
            return frames;

        Mat[] results = new Mat[frames.Length];
        for (int i = 0; i < frames.Length; i++)
        {
            results[i] = ApplyCLAHE(frames[i]);
        }
        return results;
    }

    private void OnDestroy()
    {
        clahe?.Dispose();
    }

    void OnValidate()
    {
        // 编辑器模式下实时更新 CLAHE 参数
        if (Application.isPlaying && clahe != null)
        {
            clahe = Cv2.CreateCLAHE(clipLimit, new Size(tileGridSize, tileGridSize));
        }
    }
}