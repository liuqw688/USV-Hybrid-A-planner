# 海图解析与发布

Python节点从S-57 XML提取几何，输出海图显示、原始陆地和航道数据。实际构建方式为ament_python。

入口、模块关系和当前运行方式见[项目说明](/home/l/work_ws/README.md)。

核心实现：river_chart/node.py：参数、栅格、Marker及航道发布；river_chart/parser.py：XML/WKT解析与坐标转换。

主要消息关系：`/chart_costmap、/chart_static_obstacles、/chart_lane_data`。

参数说明见[PARAMETERS.md](PARAMETERS.md)，代码导航见[FUNCTIONAL_GUIDE.md](FUNCTIONAL_GUIDE.md)。历史修复与测试资料保存在`/home/l/work_ws/docs/history/river_chart/`。
