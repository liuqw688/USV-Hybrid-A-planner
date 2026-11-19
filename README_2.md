# USV_systems

ROS 2 Jazzy 无人船工程，包含地图、六状态船体仿真、B 样条消息接口和 NMPC 跟踪控制器。正式控制器与仿真测试分别启动；测试脚本集中在 `usv_tracking_test` 包。

**规划器闭环仿真的简洁启动步骤见 [仿真测试快速启动](README_SIMULATION.md)**，包括地图、船模、NMPC、规划器及实际运行绘图。

## 功能包

| 包 | 职责 |
| --- | --- |
| `usv_interfaces` | 无人船 B 样条及感知消息定义 |
| `usv_map` | 海图、原点、里程计转换与 RViz 显示 |
| `usv_model_sim` | 六状态船体动力学仿真及键盘控制 |
| `usv_nmpc_controller` | 正式 NMPC 控制节点、配置及单元测试 |
| `usv_tracking_test` | 测试轨迹发布、仿真组合启动及闭环路线回归 |
| `usv_planner` | 六状态动力学 A*、二维海图避障、位姿 B 样条规划 |
| `usv_map_tools` | 海图转换及航道、泊位、礁石、禁航区编辑 |
| `channel_navigation_manager` | 航道代价及模拟感知 |

## 编译与终端环境

```bash
cd /home/sus/USV_systems
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

以下四步按顺序分别在四个终端运行，参考发布脚本最后启动。每个新终端先加载相同环境：

```bash
source /opt/ros/jazzy/setup.bash
source /home/sus/USV_systems/install/setup.bash
```

### 1. 地图与里程计

```bash
ros2 launch usv_map usv_map.launch.py \
  rviz:=true \
  channel_navigation:=false \
  simulate_perception:=false
```

以上是纯跟踪测试的地图启动方式：启动静态海图、原点 `/fix`、里程计转换、船体显示 TF 和 RViz，关闭动态航道代价与模拟感知。里程计节点收到 `/fix` 和 `/USV_state` 后发布 `/odom`；船体状态由第 2 步的仿真器提供，历史轨迹发布到 `/USV_path`。

启动参数使用 `参数名:=值` 写在命令末尾；不写时采用下表默认值。

| 启动参数 | 默认值 | 取值及作用 |
| --- | --- | --- |
| `input_path` | `""`（空字符串） | 海图 XML 路径，建议使用绝对路径。非空时覆盖 `usv_map.yaml` 的同名项；两处都为空时加载包内 `data/suiku.xml` |
| `rviz` | `true` | `true` 启动 RViz 窗口；`false` 关闭窗口，地图、里程计和 TF 仍运行 |
| `channel_navigation` | `true` | `true` 启动航道代价模块，发布动态 `/channel_costmap`；`false` 关闭整个航道模块及其模拟感知。静态 `/chart_costmap` 仍发布 |
| `simulate_perception` | `true` | 仅在 `channel_navigation=true` 时有效；`true` 启动模拟障碍感知；`false` 只关闭模拟感知，航道模块继续运行，可接收外部 `/obstacle_detections` |
| `output` | `screen` | `screen` 将节点输出显示在终端；`log` 将节点输出写入 ROS 日志文件 |

**只写 `simulate_perception:=false` 时，`channel_navigation` 仍为默认的 `true`，因此航道代价模块仍会启动。** 直接运行不带参数的命令，则 RViz、航道模块和模拟感知全部开启。

其他常用组合：

```bash
# 无图形界面的纯跟踪测试。
ros2 launch usv_map usv_map.launch.py \
  rviz:=false channel_navigation:=false simulate_perception:=false

# 启用动态航道代价及模拟障碍感知。
ros2 launch usv_map usv_map.launch.py \
  rviz:=true channel_navigation:=true simulate_perception:=true

# 启用动态航道代价，障碍检测由外部感知节点提供。
ros2 launch usv_map usv_map.launch.py \
  rviz:=true channel_navigation:=true simulate_perception:=false
```

地图原点、栅格分辨率、里程计周期等在 [usv_map.yaml](src/usv_map/config/usv_map.yaml) 中修改；卫星图层在 [water_map.yaml](src/usv_map/config/water_map.yaml) 中修改；航道模块另读 [channel_navigation.yaml](src/channel_navigation_manager/config/channel_navigation.yaml)。这些 YAML 配置不是上述启动参数，例如不能直接追加 `planning_resolution:=1.0` 来修改分辨率。各项当前值、单位和配置生效方法见 [地图包配置说明](src/usv_map/README.md#配置)。查看启动参数：

```bash
ros2 launch usv_map usv_map.launch.py --show-args
```

### 2. USV 动力学仿真

```bash
ros2 launch usv_model_sim usv_model.launch.py
```

仿真器收到地图原点 `/fix` 后开始发布 `/USV_state`，订阅 `/cmd` 驱动船体。默认初始位置为 ENU `(0, 0)`，船首向东；位置、初始航向和测量噪声在 `src/usv_model_sim/config/usv_model.yaml` 配置。需要使用其他配置时：

```bash
ros2 launch usv_model_sim usv_model.launch.py model_config:=/绝对路径/usv_model.yaml
```

### 3. NMPC 控制器

```bash
ros2 launch usv_nmpc_controller usv_nmpc.launch.py
```

此入口只启动 `usv_nmpc_node`。控制器接收 `/odom` 和 `/planning/bspline`，输出 `/cmd`；缺少有效输入时保持等待或停车。它不启动地图、仿真器或测试发布器，可直接对接真实船体状态源和局部规划器。指定控制参数：

```bash
ros2 launch usv_nmpc_controller usv_nmpc.launch.py config_file:=/绝对路径/usv_nmpc.yaml
```

### 4. Python 脚本发布测试局部路径

```bash
USV_NMPC_PUBLISH_RATE_MODE=fixed python3 /home/sus/USV_systems/src/usv_tracking_test/test/numeric_routes/route_shapes.py
```

脚本只负责参考发布，不启动地图、仿真器或控制器。收到 `/odom` 后，以当前六状态为初值，用船体模型生成完整路线并拟合一次全局时间 B 样条；路线为 R7 圆、直线 200 m、R10 圆、直线 200 m、R15 圆、直线 200 m。生成全局样条需要一段计算时间，随后按当前船位投影，以约 10 Hz 发布前方最多 50 m 的局部样条片段到 `/planning/bspline`，并发布 `/planning/bspline_path` 供 RViz 显示。控制点为 `[x, y, phi]`，各片段沿用同一全局时间基准。末段按剩余长度缩短。

测试 1～10 Hz 随机更新时，替换上一步命令：

```bash
USV_NMPC_PUBLISH_RATE_MODE=random \
USV_NMPC_PUBLISH_RATE_MIN_HZ=1 \
USV_NMPC_PUBLISH_RATE_MAX_HZ=10 \
python3 /home/sus/USV_systems/src/usv_tracking_test/test/numeric_routes/route_shapes.py
```

同一次测试只运行一个参考发布器。局部规划器接入时，停止该 Python 脚本，由规划器发布同一消息接口；详见 [NMPC 接口约定](src/usv_nmpc_controller/README.md)。

## 话题连接与检查

```text
地图 /fix ──────────────────────────> USV 仿真器
USV 仿真器 /USV_state ──> 里程计转换 /odom ──> NMPC
                                  └───────> Python 测试参考发布器
Python 或局部规划器 /planning/bspline ──────> NMPC /cmd ──> USV 仿真器
```

```bash
ros2 node list
ros2 topic hz /odom
ros2 topic hz /planning/bspline
ros2 topic echo /nmpc_tracking_state --qos-durability transient_local
ros2 topic echo /cmd
```

正常运行后应进入 `TRACKING`。四个终端用 `Ctrl+C` 分别停止。测试与其他仿真同时运行时，各测试终端应设置相同且独立的 `ROS_DOMAIN_ID`，避免话题互相干扰。

## 可选：组合测试启动

```bash
ros2 launch usv_tracking_test usv_tracking.launch.py
```

该入口默认只组合启动地图和动力学仿真，替代前述终端 1、2；NMPC 和 Python 参考脚本仍按第 3、4 步分别启动。也可追加 `start_controller:=true` 同时启动 NMPC，随后独立运行 Python 脚本。使用四终端方式时，无需再运行组合入口。

默认不提前发布参考，避免控制器晚启动时，旧时间样条的当前参考点已离船超过 2 m 而被拒收。重启测试时，先停止参考发布脚本，待控制器就绪后重新启动脚本，重新建立路线时间基准。

## 路线回归与测试

批量测试会自行启动隔离的仿真和控制器，输出放在源码工作空间之外：

```bash
cd /home/sus/USV_systems
OUT_DIR=/tmp/usv_systems_runs/rolling_composite
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python3 src/usv_tracking_test/test/numeric_routes/run_suite.py "$OUT_DIR" --parallel 2
python3 src/usv_tracking_test/test/numeric_routes/analyze_rolling_suite.py "$OUT_DIR"
```

默认测试初始航向为 0°、30°、−60°、−90°、180°。参数、测量与图表说明见 [测试包说明](src/usv_tracking_test/README.md)。执行工程检查：

```bash
colcon test
colcon test-result --verbose
```

## 详细说明

- [代码注释规范](COMMENT_STYLE.md)
- [地图与里程计](src/usv_map/README.md)
- [地图编辑工具](src/usv_map_tools/README.md)
- [动力学仿真](src/usv_model_sim/README.md)
- [消息接口](src/usv_interfaces/README.md)
- [NMPC 控制器](src/usv_nmpc_controller/README.md)
- [独立测试与路线回归](src/usv_tracking_test/README.md)
- [无人船局部规划器](src/usv_planner/README.md)
- [航道代价与感知](src/channel_navigation_manager/README.md)
