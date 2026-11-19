# 海图解析与发布：代码导航

Python节点从S-57 XML提取几何，输出海图显示、原始陆地和航道数据。实际构建方式为ament_python。

river_chart/node.py：参数、栅格、Marker及航道发布；river_chart/parser.py：XML/WKT解析与坐标转换。

消息关系：`/chart_costmap、/chart_static_obstacles、/chart_lane_data`。

## 维护约定

- 运行配置以本包YAML和整链路launch为准。
- 地图安全、航道软偏好、显示图层分别处理；Marker不替代碰撞检查。
- 本船状态来自实测/仿真里程计，规划与控制使用相同六状态船模。
- 改动后运行本包与依赖链路测试；旧报告不代表新版本已通过。

详细项目关系见[PROJECT_CONTEXT.md](/home/l/work_ws/PROJECT_CONTEXT.md)。
