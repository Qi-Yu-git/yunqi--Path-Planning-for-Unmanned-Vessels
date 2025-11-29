using OpenCvSharp;
using UnityEngine;

/// <summary>
/// YOLO目标检测结果数据结构
/// 存储单目标的检测信息
/// </summary>
public class YoloResult
{
    /// <summary>
    /// 类别ID（与类别名称列表索引对应）
    /// </summary>
    public int ClassId { get; set; }

    /// <summary>
    /// 类别名称（如"car"、"person"）
    /// </summary>
    public string ClassName { get; set; }

    /// <summary>
    /// 置信度（0-1之间）
    /// </summary>
    public float Confidence { get; set; }

    /// <summary>
    /// 边界框（图像坐标系，左上角为原点）
    /// </summary>
    public Rect2d Rect { get; set; }

    /// <summary>
    /// 目标中心点位置（图像坐标系）
    /// </summary>
    /// 
    // 添加追踪ID（如果模型支持）
    public int TrackId { get; set; }  // 新增这一行

    public Vector3 Position
    {
        get
        {
            // 计算边界框中心点作为位置
            return new Vector3(
                (float)(Rect.X + Rect.Width / 2),
                0,  // Y轴默认为0，可根据实际需求调整
                (float)(Rect.Y + Rect.Height / 2)
            );
        }
    }

    /// <summary>
    /// 转换为字符串（便于日志输出）
    /// </summary>
    public override string ToString()
    {
        return $"[{ClassId}] {ClassName} (置信度: {Confidence:F2}) " +
               $"位置: ({Rect.X:F1}, {Rect.Y:F1}, {Rect.Width:F1}, {Rect.Height:F1})";
    }
}