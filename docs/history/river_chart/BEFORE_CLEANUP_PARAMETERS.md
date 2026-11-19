# 参数作用及调节说明

本说明对应当前源码与配置，不承诺每个参数可在线更新。多数参数仅在构造时读取：编辑YAML后请停止旧实例并重新启动。launch内追加参数可能覆盖YAML；可用 `ros2 param dump /节点名` 核对运行值。

数值“增大/减小”指其他条件不变时的趋势，不是无限调大都有效。布尔值按开/关解释；话题和frame必须与生产者一致。不要同时降低安全半径、冗余及急停距离来追求直线效率。

## 无YAML的现有参数来源

当前节点由Python launch传入参数，没有现成加载的YAML。下表节点默认值与launch默认值分开，避免误以为launch运行是0.2m栅格。

| 参数 | 节点默认／launch默认 | 用途及调节影响 |
|---|---|---|
| input_path | 安装资源ecdis_data.xml | 换文件改变整张海图；需确认格式、坐标和航道要素 |
| frame_id | map／map；河道一键覆盖odom | 只改名字不做坐标变换；必须与TF和下游一致 |
| origin_longitude、origin_latitude | NaN自动推断 | 指定局部原点；变化会改变坐标，不宜任意改 |
| scale | 15／15 | 缩放几何坐标；增大通常缩小地图坐标距离，不只是外观 |
| line_width、point_size | 0.2、0.15 | 矢量线宽/点尺寸；增大更显眼，不改栅格安全 |
| fill_polygons | false | 开启填充显示，不改变规划可航行判定 |
| show_land_boundaries | false | 开启陆地边界显示，增加显示内容 |
| focus_navigation_features | true | 开启只聚焦航行要素；关闭显示更多类别 |
| publish_labels | false | 开启显示名称，增加文字负担 |
| publish_occupancy_grid | false | 开启可选显示图；不是规划图开关 |
| grid_resolution | 0.2／1.0米 | 可选显示图分辨率；增大更粗且省内存，减小更细耗资源 |
| max_grid_cells | 20000000／1000000 | 显示图格数上限；增大容纳更细图，减小可能自动加粗 |
| publish_planning_layers | true；launch不显式覆盖 | 关闭将缺少规划所需栅格输出 |
| planning_resolution | 0.2／1.0米 | 规划图分辨率；增大变粗降低细节，减小计算/内存增大；上限可能自动调整 |
| planning_max_cells | 20000000／650000 | 规划格数上限；增大更耗资源，减小可能加粗实际分辨率 |
| obstacle_inflation | 1米 | 规划基础图障碍处理尺寸；增大更保守、减小更贴近；不等于下游15米岸线膨胀 |
| channel_corridor_width | 20米 | 当前实现用作线段刷图半径；增大扩大低代价走廊，减小收窄；不决定左右航道 |
| publish_channel_direction | false | 开启方向Marker，不改变主航道分类 |
| channel_outside_cost | 60 | 基础图非走廊初值；增大外部代价高，减小更易离开；最终语义偏好还由manager定义 |
| reload_period | 0秒 | 0只发布一次；正数按秒重载；增大间隔更新慢，减小重复解析耗资源 |
| marker_topic、metadata_topic、occupancy_topic | chart_markers、chart_metadata、chart_occupancy | 更换输出名称须同步RViz和消费者 |
| future_obstacle_topic | future_obstacles | 预留PoseArray输入；当前没有独立障碍层完整接线 |
| dynamic_obstacle_radius | 1米 | 预留输入障碍刷图半径；增大占据范围大，减小可能过小；不能替代真实船半径 |
| rviz（launch项） | true | 是否启动显示界面；河道一键设false避免重复RViz |

示例：`ros2 launch river_chart river_chart.launch.py planning_resolution:=1.0 planning_max_cells:=650000 rviz:=true`。未公开为launch参数的节点项需通过节点启动参数或修改launch接线设置，不能假设所有项都可使用launch的:=覆盖。

## 跨包一致性检查

- 当前本船最高速度4m/s；小型无人船等效规划半径均为0.5m、安全冗余1m，因此普通中心硬净空为2m。Marker长1m宽1.5m只是显示外形。
- Hybrid在7m软域提前规划、2m硬净空由Hybrid负责；tracking_only=true的DWA只跟踪Path，不使用目标船距离停车。
- AIS式远程监视保持80m/40s不变；行动层24m/16s、紧急层8m/6s。风险圈不是感知截止范围，也不等于软/硬避让域。
- 地图尺寸、原点、分辨率与frame应一致；地图膨胀和独立障碍层不得混淆。
- 主航道、外部、对向代价是偏好，安全硬过滤优先；降低偏好不解除危险栅格阻挡。
