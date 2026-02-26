from ultralytics import YOLO
import torch
import onnx
from onnxsim import simplify
import os

# ========== 环境适配 ==========
torch.cuda.is_available = lambda: False  # 强制CPU导出
device = "cpu"
print(f"📌 导出环境: PyTorch={torch.__version__}, 设备={device}")
print(f"当前目录: {os.getcwd()}")

# ========== 加载YOLOv8模型 ==========
model = YOLO("yolov8n.pt")
print("✅ YOLOv8n模型加载成功")

# ========== 导出基础ONNX（适配新版ultralytics） ==========
base_export_path = model.export(
    format="onnx",
    opset=12,          
    dynamic=False,     
    simplify=True,     
    batch=1,           
    imgsz=640,         
    device=device,     
    optimize=False,    
)

# ========== 二次深度简化（适配新版onnxsim 0.4.33） ==========
print("🔧 深度简化模型，移除Unity不支持的属性...")
onnx_model = onnx.load(base_export_path)
# 关键修改：移除enable_shape_inference等无效参数，适配新版onnxsim
simplified_model, check = simplify(
    onnx_model,
    input_shapes={"images": [1, 3, 640, 640]},
    skip_fuse_bn=False,
    # 移除enable_shape_inference、ignore_opset_check（新版参数已废弃）
)
assert check, "模型简化验证失败！"

# 保存最终兼容版（文件名和你原有代码一致）
final_export_path = base_export_path
onnx.save(simplified_model, final_export_path)

# ========== 导出完成提示 ==========
print(f"\n🎉 导出成功！")
print(f"📁 新ONNX文件路径: {final_export_path}")
print(f"💡 下一步执行copy命令即可移到Assets/Models")