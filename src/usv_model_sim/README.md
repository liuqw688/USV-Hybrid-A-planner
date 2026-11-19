# usv_model_sim

维护者：susheng（963338707@qq.com）  
文档更新：2026-11-04  

## 功能

`usv_model_sim` 只提供无人船仿真状态源：

- 根据 `/cmd` 对前向速度和偏航角速度进行动力学积分；
- 发布严格固定格式的 `/USV_state`；
- 订阅地图原点后才开始发布状态；
- 提供键盘控制脚本。

船体外观由 `usv_map` 负责显示；控制器只依赖 `/odom` 和 `/planning/bspline`，不依赖本包。
实船接入时可由 `usv_hardware` 替换本包并提供兼容的 `/cmd` 与 `/USV_state` 接口。

## 目录

```text
usv_model_sim/
├── config/usv_model.yaml       # 动力学、噪声和键盘参数
├── include/usv_model_sim/      # 仿真模型接口
├── src/                        # 动力学库和模型节点
├── scripts/keyboard_control.py
└── launch/usv_model.launch.py
```

## 配置

主配置：[config/usv_model.yaml](config/usv_model.yaml)

| 参数 | 默认值 | 说明 |
| --- | ---: | --- |
| `max_speed_mps` | 5.0 | 最大前向/后向速度，m/s |
| `max_yaw_rate_deg_s` | 15.0 | 最大偏航角速度，度/s |
| `initial_east_m` / `initial_north_m` | 0.0 / 0.0 | 仿真初始 ENU 位置，m |
| `position_noise_stddev_m` | 0.25 | 测量位置 ENU 高斯噪声标准差，m |
| `heading_noise_stddev_deg` | 1.0 | 航向测量独立零均值高斯白噪声标准差，度；0 关闭 |
| `noise_seed` | -1 | 仿真噪声随机种子；设为非负 32 位整数可复现实验，-1 使用随机种子 |
| `command_timeout_sec` | 0.5 | 控制超时后输入归零 |
| `update_rate_hz` | 20.0 | 动力学和状态发布频率 |
| `origin_topic` | `/fix` | 地图原点 |
| `cmd_topic` | `/cmd` | 控制输入 |
| `state_topic` | `/USV_state` | 状态输出 |

## 启动

```bash
ros2 launch usv_model_sim usv_model.launch.py
```

启动会运行 `usv_model_node`。模型收到有效 `/fix` 前不会发布 `/USV_state`。地图包提供船体外观，
运行地图 launch 时会依据 `/odom` 姿态显示。

键盘控制：

```bash
ros2 run usv_model_sim keyboard_control.py
```

| 按键 | 动作 |
| --- | --- |
| `w` / `s` | 前进 / 后退 |
| `a` / `d` | 左转 / 右转 |
| `q` / `e` | 前进并左转 / 前进并右转 |
| `1` / `2` | 增大 / 减小速度档位 |
| `3` / `4` | 增大 / 减小角速度档位 |
| 空格 | 控制量归零 |
| `t` | 归零并退出 |

## ROS 接口

### `/cmd`

类型为 `geometry_msgs/Twist`：`linear.x` 是前向速度 `u`（m/s），`angular.z` 是偏航角速度
（rad/s）。模型会按配置限幅，超过 `command_timeout_sec` 没有新指令时自动归零。

### `/USV_state`

类型为 `std_msgs/Float64MultiArray`，固定 19 个字段，下标从 0 开始：

| 下标 | 内容 | 单位/坐标 |
| --- | --- | --- |
| 0～2 | 姿态角 | rad，东北天导航系 |
| 3～5 | 导航系速度 | 东、北、天，m/s |
| 6～8 | 测量位置 | 纬度、经度、高度；纬经度 rad，高度 m |
| 9～11 | 本体速度 | 右、前、上（RFU），m/s |
| 12 | 偏航角速度 | rad/s |
| 13～15 | 位置真值 | 纬度、经度、高度；纬经度 rad，高度 m |
| 16 | 航向真值 | rad |
| 17 | 状态位 | 恒为 4 |
| 18 | 时间 | 累计仿真时间，s |

本体速度使用右前上坐标：`body_vel[0] = v` 为右向速度、`body_vel[1] = u` 为前向速度，
`body_vel[2] = 上向速度`。

航向范围为 `[-π, π]`：0 指北、`-π/2` 指东、`+π/2` 指西、±π 指南。模型使用：

```text
E_dot = -u*sin(psi) + v*cos(psi)
N_dot =  u*cos(psi) + v*sin(psi)
psi_dot = r
```

测量位置的东、北、天三个方向分别加入独立零均值高斯噪声。
航向测量字段 `[2]` 每帧独立加入标准差 1° 的零均值高斯白噪声，转换为弧度后归一化至 `[-π, π]`；
1° 是标准差，不是噪声幅值上限。动力学内部航向、真值字段 `[16]`、速度和横滚/俯仰保持原值。

## 测试

```bash
colcon test --packages-select usv_model_sim
colcon test-result --verbose
```
