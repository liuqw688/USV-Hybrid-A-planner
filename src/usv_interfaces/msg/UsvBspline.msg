# B 样条次数（例如三次样条为 3）和轨迹版本编号。
int32 order
int64 traj_id
# knots[order] 对应的 ROS 绝对时间。滚动切片需映射回同一全局样条时间原点，
# 不应每次发布都重置为当前时刻。
builtin_interfaces/Time start_time
# x、y、phi 共用的 B 样条节点向量，单位秒。
float64[] knots
# 位姿样条控制点 [x, y, phi]；不是曲线上的插值状态，也不是六维动力学状态。
usv_interfaces/UsvBsplineControlPoint[] control_points
