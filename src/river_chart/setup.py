"""@brief river_chart ROS 2 Python 功能包安装配置。

@author susheng
@date 2026-08-28
"""

from setuptools import find_packages, setup

package_name = "river_chart"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        # ament index 标记文件。
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml", "README.md", "FUNCTIONAL_GUIDE.md", "PARAMETERS.md"]),
        (f"share/{package_name}/launch", ["launch/river_chart.launch.py"]),
        (f"share/{package_name}/rviz", ["rviz/river_chart.rviz"]),
        (f"share/{package_name}/data", ["resource/ecdis_data.xml"]),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    description="Publish S-57 chart XML geometry as ROS 2 visualization markers",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "river_chart_node = river_chart.node:main",
        ],
    },
)
