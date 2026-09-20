"""@brief ROS 2 启动文件测试。

@author susheng
@date 2026-08-28
"""

import importlib.util
from pathlib import Path


# [功能与联系] 执行此场景的输入构造和断言核验；用于回归验证，不参与正常航行控制。具体通过条件以函数内断言为准。
def test_launch_description_loads() -> None:
    launch_file = Path(__file__).parents[1] / "launch" / "river_chart.launch.py"
    spec = importlib.util.spec_from_file_location("river_chart_launch", launch_file)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    description = module.generate_launch_description()
    assert description is not None
