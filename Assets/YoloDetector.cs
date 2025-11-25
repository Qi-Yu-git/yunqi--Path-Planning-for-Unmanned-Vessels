using UnityEngine;
using OpenCvSharp;
using System.Collections.Generic;

public class YoloDetector : MonoBehaviour
{
    [Header("配置")]
    public string modelPath; // 模型路径（如"StreamingAssets/yolov8n.onnx"）
    public float confidenceThreshold = 0.5f;
    public float iouThreshold = 0.4f;
    public WebCamTexture webCamTexture; // 可选：用于摄像头检测

    private YoloV8Engine yoloEngine;
    private Mat frameMat; // OpenCV图像格式

    void Start()
    {
        // 初始化YOLO引擎
        string fullModelPath = Application.streamingAssetsPath + "/" + modelPath;
        yoloEngine = new YoloV8Engine(fullModelPath, confidenceThreshold, iouThreshold);

        // 初始化摄像头（可选）
        if (webCamTexture == null)
        {
            WebCamDevice[] devices = WebCamTexture.devices;
            if (devices.Length > 0)
            {
                webCamTexture = new WebCamTexture(devices[0].name);
                webCamTexture.Play();
            }
        }
    }

    void Update()
    {
        if (webCamTexture != null && webCamTexture.isPlaying)
        {
            // 将摄像头纹理转换为OpenCV的Mat格式
            frameMat = WebCamTextureToMat(webCamTexture);

            // 执行检测
            List<YoloResult> results = yoloEngine.Detect(frameMat);

            // 处理检测结果（如绘制边界框、打印日志等）
            ProcessResults(results);

            // 释放Mat资源
            frameMat.Release();
        }
    }

    // WebCamTexture转OpenCV Mat
    private Mat WebCamTextureToMat(WebCamTexture texture)
    {
        Texture2D tex2D = new Texture2D(texture.width, texture.height);
        tex2D.SetPixels(texture.GetPixels());
        tex2D.Apply();
        byte[] bytes = tex2D.EncodeToPNG();
        Mat mat = Cv2.ImDecode(bytes, ImreadModes.Color);
        // 翻转图像（根据摄像头方向调整）
        Cv2.Flip(mat, mat, FlipMode.Y);
        Destroy(tex2D);
        return mat;
    }

    // 处理检测结果（示例：打印日志）
    private void ProcessResults(List<YoloResult> results)
    {
        foreach (var result in results)
        {
            Debug.Log($"检测到：{result.ClassName}，置信度：{result.Confidence:F2}，位置：{result.Rect}");
            // 如需在屏幕绘制边界框，可结合Unity的OnGUI或LineRenderer实现
        }
    }

    void OnDestroy()
    {
        // 释放资源
        yoloEngine?.Dispose();
        webCamTexture?.Stop();
    }
}