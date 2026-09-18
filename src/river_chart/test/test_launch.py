"""@brief ROS 2 启动文件测试。


@date 2026-08-28
"""

import importlib.util
from pathlib import Path


def test_launch_description_loads() -> None:
    launch_file = Path(__file__).parents[1] / "launch" / "river_chart.launch.py"
    spec = importlib.util.spec_from_file_location("river_chart_launch", launch_file)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    description = module.generate_launch_description()
    assert description is not None
