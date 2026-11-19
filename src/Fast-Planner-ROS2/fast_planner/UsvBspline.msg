# B 样条次数（例如三次样条为 3）和轨迹版本编号。
int32 order
int64 traj_id
# 轨迹开始执行的 ROS 时间。
builtin_interfaces/Time start_time
# 位置样条节点向量（秒）和 ENU 位置控制点（米）。
float64[] knots
geometry_msgs/Point[] pos_pts
# 可选航向控制点（弧度）及其均匀节点间隔（秒）。
float64[] yaw_pts
float64 yaw_dt
