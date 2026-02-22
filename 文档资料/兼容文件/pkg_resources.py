# 适配 setuptools 82.0.0 的 pkg_resources 兼容层（纯标准库，无额外依赖）
from importlib import metadata

# 核心：模拟 pkg_resources 的 Distribution 类（仅实现mlagents需要的属性）
class Distribution:
    def __init__(self, dist_name):
        try:
            # 从Python标准库获取包信息
            self._dist = metadata.distribution(dist_name)
            # 映射mlagents需要的核心属性
            self.version = self._dist.version
            self.project_name = self._dist.name
        except metadata.PackageNotFoundError:
            # 兼容mlagents的报错逻辑
            raise ImportError(f"Distribution '{dist_name}' not found")

# 核心：模拟 pkg_resources.get_distribution 方法（mlagents唯一调用的方法）
def get_distribution(dist_name):
    return Distribution(dist_name)

# 导出mlagents需要的符号（确保import后能直接调用）
__all__ = ["get_distribution", "Distribution"]
globals()["get_distribution"] = get_distribution
globals()["Distribution"] = Distribution