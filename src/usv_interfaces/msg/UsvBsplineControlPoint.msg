# ENU 世界坐标中的无人船 B 样条控制点。
# x、y 单位 m，其中 x 向东、y 向北；phi 单位 rad。
# phi 是导航航向角：0 指北，-pi/2 指东，+pi/2 指西，±pi 指南。
# 发布端应对 phi 做连续展开，避免在 ±pi 处发生跳变。
float64 x
float64 y
float64 phi
