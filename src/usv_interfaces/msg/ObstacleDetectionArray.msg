# 每帧为完整观测快照；空数组表示本帧无障碍，接收端必须清除上一帧感知覆盖。
# header.frame_id 为 RFU 传感器坐标系，默认 usv_body_rfu；方位与距离均为水平量。
# header.stamp 为观测时刻，需存在该时刻到地图坐标系的 TF；不能用发送时刻替代。
std_msgs/Header header
usv_interfaces/ObstacleDetection[] obstacles
