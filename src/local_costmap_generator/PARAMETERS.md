# 河岸代价地图：当前参数

以下默认值直接对应当前YAML；调节目的与单位见YAML中的相邻注释。修改后重启节点。

## local_costmap.yaml

| 参数 | 当前值 |
|---|---|
| `input_topic` | `/chart_costmap` |
| `output_topic` | `/local_costmap` |
| `occupied_threshold` | `90` |
| `robot_radius` | `3.0` |
| `inflation_radius` | `15.0` |
| `minimum_cost` | `1` |
| `high_cost_threshold` | `50` |
| `decay_factor` | `3.0` |

## 航道裁剪与独立障碍

| 参数 | 默认配置 |
|---|---|
| `lane_data_topic` | `/chart_lane_data` |
| `clip_shore_soft_in_channel` | `true` |
| `obstacle_output_topic` | `/channel/obstacle_costmap` |

只裁剪岸边和航道外高代价源的软梯度，真实障碍100、未知-1和3m硬缓冲99不清除。关闭裁剪可用于对比；无法读取有效同坐标系航道数据时默认保留完整膨胀。独立障碍层在MAIN、OPPOSITE和航道外均生效。

`obstacle_lethal_threshold: 85` 与Hybrid/MPC独立障碍禁止阈值一致；合并到local_costmap时达到该值的独立障碍膨胀格映射为99，使全局A*不能将其选成后级拒绝的航点。调整时必须同步三端阈值。
