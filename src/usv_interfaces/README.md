# usv_interfaces

维护者：susheng（963338707@qq.com）  
文档更新：2026-11-03

## 功能

本包统一管理工程消息，不运行节点。仿真感知节点和真实传感器适配器使用相同的感知消息，
轨迹规划器和控制器使用相同的 B 样条消息。

## 消息

### `UsvBspline.msg`

| 字段 | 说明 |
| --- | --- |
| `order` | B 样条次数 |
| `traj_id` | 轨迹版本编号 |
| `start_time` | 样条参数 `knots[order]` 对应的 ROS 绝对时间。滚动切片必须使用共同的全局样条时间原点：若切片从全局参数 `t0` 开始，则填写 `global_epoch + t0`，不要每次发布都重置成当前时刻。 |
| `knots` | `x/y/phi` 共用的样条节点向量，秒 |
| `control_points` | 完整控制点 `[x, y, phi]`；位置单位 m，航向单位 rad |

节点向量长度必须为控制点数量加 `order + 1`。`x` 向东、`y` 向北，与 `/odom`
位置同属 ENU。`phi` 使用无人船导航角：北为 0，东为 `-pi/2`，西为 `+pi/2`，
南为 `±pi`。`phi` 是船体状态角，不是位置曲线的切线角；控制点角度应先连续展开。
控制器使用同一节点向量拟合三个分量，并从 `x/y/phi` 的样条导数按无人船运动学反算
`u/v/r`，其中 `u` 前向为正、`v` 右向为正。消息不携带终点语义，样条末端可以有非零
速度；若预测时间超过样条有效区间，控制器沿末端一阶导数外推参考。到点完成由末端导数
和实船状态推断，不需要额外的终点标记。控制器按有效起始时刻 `knots[order]` 求出样条
起点；该位置距当前有效 `/odom` 超过 2 m 时拒收新样条，并继续使用上一条有效参考。

### `ObstacleDetection.msg`

单个障碍物只包含四项：

| 字段 | 说明 |
| --- | --- |
| `label` | 标签，例如 `REEF` |
| `bearing_rad` | 相对船体水平角，rad；船首 0，右侧为正，左侧为负 |
| `range_m` | 障碍物重心距离，m |
| `radius_m` | 障碍物半径，m；不包含代价地图膨胀 |

### `ObstacleDetectionArray.msg`

```text
std_msgs/Header header
usv_interfaces/ObstacleDetection[] obstacles
```

`header.frame_id` 默认是 `usv_body_rfu`，表示右、前、上（RFU）本体坐标系。
`header.stamp` 是实际观测时刻，接收端使用该时刻的 TF 转换目标位置。
每帧是完整列表；空数组表示当前没有障碍，接收端应清除上一帧感知覆盖。

## 使用

```bash
colcon build --symlink-install --packages-select usv_interfaces
source install/setup.bash
ros2 interface show usv_interfaces/msg/ObstacleDetection
ros2 interface show usv_interfaces/msg/ObstacleDetectionArray
ros2 interface show usv_interfaces/msg/UsvBspline
```

该包没有独立启动文件。感知消息供 `channel_navigation_manager` 使用，B 样条消息供
轨迹发布器和 `usv_nmpc_controller` 使用。新增自定义消息也统一放在本包的 `msg/` 目录。
