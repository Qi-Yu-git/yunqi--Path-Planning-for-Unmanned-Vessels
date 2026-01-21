using System;
using System.Collections.Generic;
using System.Linq;
using System.IO;
using OpenCvSharp;
using OpenCvSharp.Dnn;
using UnityEngine;
using UnityEngine.UI;
using TMPro;

// 解决 Rect 命名冲突
using OpenCvRect = OpenCvSharp.Rect;

namespace PPUV.YOLOv8
{
    /// <summary>
    /// 图像输入选择器（UI交互+摄像头/本地图片输入）
    /// </summary>
    public class ImageInputSelector : MonoBehaviour
    {
        // UI 元素（需在Inspector赋值）
        public Button cameraButton;
        public Button imageFileButton;
        public TextMeshProUGUI statusText;
        public RawImage displayImage;
        public Camera detectionCamera;  // 用于坐标转换的相机

        // YOLO 检测核心
        private YoloV8Engine yoloEngine;
        private string modelPath = Application.dataPath + "/Models/yolov8n.onnx";

        // 图像输入源相关
        private WebCamTexture webCamTexture;
        private Mat currentMat;
        private Texture2D outputTexture;
        private int selectedInputType = 0; // 0: 未选择, 1: 摄像头, 2: 本地图片

        // 障碍物数据
        private List<Vector3> detectedObstaclePositions = new List<Vector3>();
        private List<int> detectedObstacleIds = new List<int>();

        void Start()
        {
            // 初始化 UI 按钮事件
            cameraButton.onClick.AddListener(() => SelectInputType(1));
            imageFileButton.onClick.AddListener(() => SelectInputType(2));

            // 初始化显示纹理
            outputTexture = new Texture2D(1, 1);
            displayImage.texture = outputTexture;

            // 校验相机赋值
            if (detectionCamera == null)
            {
                detectionCamera = Camera.main;
                if (detectionCamera == null)
                    statusText.text = "警告：未指定检测相机，将使用主相机";
            }

            // 初始化 YOLO 引擎
            try
            {
                if (!File.Exists(modelPath))
                {
                    statusText.text = "模型文件不存在：" + modelPath;
                    return;
                }
                yoloEngine = new YoloV8Engine(modelPath);
                statusText.text = "YOLO 引擎初始化成功";
            }
            catch (Exception ex)
            {
                statusText.text = "YOLO 引擎初始化失败: " + ex.Message;
            }
        }

        /// <summary>
        /// 选择输入源（1=摄像头，2=本地图片）
        /// </summary>
        public void SelectInputType(int type)
        {
            selectedInputType = type;
            statusText.text = type == 1 ? "已选择：摄像头输入" : "已选择：本地图片输入";

            // 清理之前的资源
            if (webCamTexture != null)
            {
                webCamTexture.Stop();
                Destroy(webCamTexture);
            }
            currentMat?.Release();

            if (type == 1) InitCamera();
            else if (type == 2) LoadLocalImage();
        }

        /// <summary>
        /// 初始化摄像头
        /// </summary>
        void InitCamera()
        {
            WebCamDevice[] devices = WebCamTexture.devices;
            if (devices.Length == 0)
            {
                statusText.text = "未检测到摄像头";
                selectedInputType = 0;
                return;
            }
            webCamTexture = new WebCamTexture(devices[0].name, 1280, 720, 30);
            webCamTexture.Play();
            statusText.text = "摄像头已启动：" + devices[0].name;
        }

        /// <summary>
        /// 加载本地图片
        /// </summary>
        void LoadLocalImage()
        {
            string imagePath = Application.dataPath + "/Images/test.jpg";
            if (!File.Exists(imagePath))
            {
                statusText.text = "图片不存在：" + imagePath;
                return;
            }

            currentMat = Cv2.ImRead(imagePath);
            if (currentMat.Empty())
            {
                statusText.text = "图片加载失败：" + imagePath;
                return;
            }

            statusText.text = "已加载图片：" + Path.GetFileName(imagePath);
        }

        void Update()
        {
            if (selectedInputType == 0 || yoloEngine == null) return;

            // 摄像头实时处理
            if (selectedInputType == 1 && webCamTexture != null && webCamTexture.isPlaying && webCamTexture.didUpdateThisFrame)
            {
                currentMat?.Release();
                currentMat = WebCamTextureToMat(webCamTexture);
                ProcessImage();
            }
            // 本地图片处理（仅处理一次）
            else if (selectedInputType == 2 && currentMat != null && !currentMat.Empty())
            {
                ProcessImage();
                currentMat.Release();
                currentMat = null;
            }
        }

        /// <summary>
        /// WebCamTexture 转换为 OpenCV Mat（BGR格式）
        /// </summary>
        Mat WebCamTextureToMat(WebCamTexture texture)
        {
            Texture2D tempTexture = new Texture2D(texture.width, texture.height, TextureFormat.RGBA32, false);
            tempTexture.SetPixels32(texture.GetPixels32());
            tempTexture.Apply();

            byte[] rgbaBytes = tempTexture.GetRawTextureData();
            Mat rgbaMat = new Mat(texture.height, texture.width, MatType.CV_8UC4, rgbaBytes);
            Mat bgrMat = new Mat();
            Cv2.CvtColor(rgbaMat, bgrMat, ColorConversionCodes.RGBA2BGR);

            // 处理摄像头旋转
            if (texture.videoRotationAngle == 90) Cv2.Rotate(bgrMat, bgrMat, RotateFlags.Rotate90Clockwise);
            else if (texture.videoRotationAngle == 180) Cv2.Rotate(bgrMat, bgrMat, RotateFlags.Rotate180);
            else if (texture.videoRotationAngle == 270) Cv2.Rotate(bgrMat, bgrMat, RotateFlags.Rotate90Counterclockwise);

            Destroy(tempTexture);
            rgbaMat.Release();
            return bgrMat;
        }

        /// <summary>
        /// 处理图像（YOLO检测 + 绘制结果 + 显示 + 更新障碍物位置）
        /// </summary>
        // 在ImageInputSelector的ProcessImage方法中添加日志
        void ProcessImage()
        {
            if (currentMat == null || currentMat.Empty())
            {
                Debug.LogError("当前图像为空，未采集到有效画面");
                return;
            }
            Debug.Log("成功采集图像，开始YOLO检测");
            List<YoloResult> results = yoloEngine.Detect(currentMat);
            if (results.Count == 0)
            {
                Debug.LogWarning("YOLO检测无结果，可能是模型未识别到障碍物或图像预处理异常");
            }
            else
            {
                Debug.Log($"检测到 {results.Count} 个障碍物");
            }
            // 后续绘制和位置更新逻辑...
        }

        /// <summary>
        /// 在 Mat 上绘制检测结果
        /// </summary>
        void DrawDetectionResults(Mat mat, List<YoloResult> results)
        {
            foreach (var result in results)
            {
                OpenCvRect rect = new OpenCvRect(
                    (int)result.Rect.X, (int)result.Rect.Y,
                    (int)result.Rect.Width, (int)result.Rect.Height
                );
                Cv2.Rectangle(mat, rect, Scalar.Red, 2);

                string label = $"{result.ClassName} {result.Confidence:F2}";
                int fontHeight = 20;
                OpenCvRect labelRect = new OpenCvRect(
                    (int)result.Rect.X, (int)result.Rect.Y - fontHeight,
                    label.Length * 12, fontHeight
                );
                Cv2.Rectangle(mat, labelRect, Scalar.Black, -1);
                Cv2.PutText(
                    mat, label, new Point(labelRect.X, labelRect.Y + fontHeight - 5),
                    HersheyFonts.HersheySimplex, 0.5, Scalar.Green, 2
                );
            }
        }

        /// <summary>
        /// 将 Mat 显示到 RawImage
        /// </summary>
        void DisplayMat(Mat mat)
        {
            Mat rgbMat = new Mat();
            Cv2.CvtColor(mat, rgbMat, ColorConversionCodes.BGR2RGB);

            if (outputTexture.width != rgbMat.Cols || outputTexture.height != rgbMat.Rows)
                outputTexture.Reinitialize(rgbMat.Cols, rgbMat.Rows);

            outputTexture.LoadImage(rgbMat.ToBytes());
            outputTexture.Apply();
            rgbMat.Release();
        }

        /// <summary>
        /// 更新障碍物世界坐标和ID列表
        /// </summary>
        private void UpdateObstaclePositions(List<YoloResult> results)
        {
            detectedObstaclePositions.Clear();
            detectedObstacleIds.Clear();
            if (detectionCamera == null) return;

            foreach (var result in results)
            {
                if (IsObstacleClass(result.ClassName)) // 修正：访问当前类的方法
                {
                    Point2d imageCenter = new Point2d(
                        result.Rect.X + result.Rect.Width / 2,
                        result.Rect.Y + result.Rect.Height / 2
                    );
                    // 修正：访问当前类的方法和成员
                    Vector3 worldPos = ConvertImagePointToWorldPoint(imageCenter, currentMat);
                    detectedObstaclePositions.Add(worldPos);

                    // 生成唯一ID
                    int obstacleId = result.ClassId + (int)(Time.time * 1000);
                    detectedObstacleIds.Add(obstacleId);
                }
            }
        }

        /// <summary>
        /// 判断是否为需要避障的类别
        /// </summary>
        private bool IsObstacleClass(string className)
        {
            List<string> obstacleClasses = new List<string> { "car", "truck", "bus", "boat", "bicycle", "motorcycle" };
            return obstacleClasses.Contains(className);
        }

        /// <summary>
        /// 图像坐标（OpenCV）转换为世界坐标
        /// </summary>
        private Vector3 ConvertImagePointToWorldPoint(Point2d imagePoint, Mat imageMat)
        {
            float screenX = (float)(imagePoint.X / imageMat.Cols * Screen.width);
            float screenY = (float)(imagePoint.Y / imageMat.Rows * Screen.height);

            Vector3 screenPos = new Vector3(screenX, screenY, 5f);
            Vector3 worldPos = detectionCamera.ScreenToWorldPoint(screenPos);

            return worldPos;
        }

        /// <summary>
        /// 提供外部访问障碍物世界坐标的接口
        /// </summary>
        public List<Vector3> GetDetectedObstaclePositions()
        {
            return detectedObstaclePositions;
        }


        /// <summary>
        /// 提供外部访问障碍物ID的接口
        /// </summary>
        public List<int> GetDetectedObstacleIds()
        {
            return detectedObstacleIds;
        }

        /// <summary>
        /// 释放所有资源
        /// </summary>
        void OnDestroy()
        {
            webCamTexture?.Stop();
            Destroy(webCamTexture);
            currentMat?.Release();
            yoloEngine?.Dispose();
            Destroy(outputTexture);
        }
    }

    /// <summary>
    /// YOLOv8 检测核心引擎
    /// </summary>
    public class YoloV8Engine : IDisposable
    {
        private Net _net;
        private float _confidenceThreshold;
        private float _iouThreshold;
        private readonly List<string> _classNames = new List<string>
        {
            "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck",
            "boat", "traffic light", "fire hydrant", "stop sign", "parking meter", "bench",
            "bird", "cat", "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra",
            "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee",
            "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove",
            "skateboard", "surfboard", "tennis racket", "bottle", "wine glass", "cup",
            "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange",
            "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch",
            "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse",
            "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink",
            "refrigerator", "book", "clock", "vase", "scissors", "teddy bear", "hair drier",
            "toothbrush"
        };

        public YoloV8Engine(string modelPath, float confidenceThreshold = 0.5f, float iouThreshold = 0.4f)
        {
            _confidenceThreshold = confidenceThreshold;
            _iouThreshold = iouThreshold;

            _net = CvDnn.ReadNetFromOnnx(modelPath);
            if (_net.Empty())
                throw new InvalidOperationException($"模型加载失败，请检查路径：{modelPath}");

            _net.SetPreferableBackend(Backend.OPENCV);
            _net.SetPreferableTarget(Target.CPU);
        }

        public List<YoloResult> Detect(Mat frame)
        {
            if (frame.Empty())
                throw new ArgumentException("输入图像为空");

            int frameWidth = frame.Cols;
            int frameHeight = frame.Rows;

            Mat blob = CvDnn.BlobFromImage(
                frame, 1 / 255.0, new Size(640, 640),
                new Scalar(0, 0, 0), swapRB: false, crop: false
            );

            _net.SetInput(blob);
            string[] outputLayerNames = _net.GetUnconnectedOutLayersNames();
            Mat[] outputs = new Mat[outputLayerNames.Length];
            _net.Forward(outputs, outputLayerNames);

            List<YoloResult> results = ParseOutput(outputs[0], frameWidth, frameHeight);

            blob.Release();
            foreach (var output in outputs) output.Release();

            return results;
        }

        private List<YoloResult> ParseOutput(Mat output, int frameWidth, int frameHeight)
        {
            List<YoloResult> results = new List<YoloResult>();
            int rows = output.Rows;
            int cols = output.Cols;

            for (int i = 0; i < rows; i++)
            {
                float boxConfidence = output.At<float>(i, 4);
                if (boxConfidence < _confidenceThreshold) continue;

                int classId = -1;
                float maxClassScore = 0;
                for (int j = 5; j < cols; j++)
                {
                    float classScore = output.At<float>(i, j);
                    if (classScore > maxClassScore)
                    {
                        maxClassScore = classScore;
                        classId = j - 5;
                    }
                }

                if (maxClassScore < _confidenceThreshold || classId < 0) continue;

                float cx = output.At<float>(i, 0) * frameWidth;
                float cy = output.At<float>(i, 1) * frameHeight;
                float w = output.At<float>(i, 2) * frameWidth;
                float h = output.At<float>(i, 3) * frameHeight;
                float x1 = cx - w / 2;
                float y1 = cy - h / 2;

                results.Add(new YoloResult
                {
                    ClassId = classId,
                    ClassName = _classNames[classId],
                    Confidence = boxConfidence * maxClassScore,
                    Rect = new Rect2d(x1, y1, w, h)
                });
            }

            return ApplyNonMaxSuppression(results);
        }

        private List<YoloResult> ApplyNonMaxSuppression(List<YoloResult> results)
        {
            if (results.Count == 0) return results;

            float[] confidences = results.Select(r => r.Confidence).ToArray();
            Rect2d[] boxes = results.Select(r => r.Rect).ToArray();
            CvDnn.NMSBoxes(boxes, confidences, _confidenceThreshold, _iouThreshold, out int[] indices);

            return indices.Select(idx => results[idx]).ToList();
        }

        public void Dispose()
        {
            _net?.Dispose();
        }
    }

    /// <summary>
    /// 检测结果数据结构
    /// </summary>
    public class YoloResult
    {
        public int ClassId { get; set; }
        public string ClassName { get; set; }
        public float Confidence { get; set; }
        public Rect2d Rect { get; set; }
    }
}