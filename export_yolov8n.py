from ultralytics import YOLO
import torch
import os

# 验证环境（和你的mlagents-clean配置匹配）
print(f"Torch版本: {torch.__version__}")
print(f"CUDA可用: {torch.cuda.is_available()}")
print(f"当前目录: {os.getcwd()}")

# 加载官方YOLOv8n预训练模型（已下载成功，直接读取本地）
model = YOLO("yolov8n.pt")
print("✅ YOLOv8n模型加载成功")

# 导出适配Unity+OpenCvSharp的ONNX模型（删除optimize=True，解决CUDA兼容问题）
export_path = model.export(
    format="onnx",
    opset=12,          # 适配OpenCvSharp，你的onnx1.15.0完美支持
    dynamic=False,     # 禁用动态维度，解决之前的加载形状错误
    simplify=True,     # 简化模型，移除冗余算子
    batch=1,           # 固定单批次，适配Unity单图推理
    imgsz=640,         # 固定640*640输入，和OpenCvSharp预处理一致
    device=0 if torch.cuda.is_available() else "cpu"  # 保留CUDA导出，删除optimize=True
)

# 导出完成提示
print(f"\n🎉 导出成功！")
print(f"📁 新ONNX文件路径: {export_path}")
print(f"💡 下一步执行copy命令即可移到Assets/Models")