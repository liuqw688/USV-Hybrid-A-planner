# 单个目标，仅发布标签、水平相对方位、水平距离和大小；不发布轮廓点。
string label                     # 语义标签，例如 REEF（暗礁）。
float64 bearing_rad              # 重心方位，rad，范围 [-pi, pi]；船首 0，右正左负。
float64 range_m                  # 传感器到目标重心的水平距离，m；不是最近边缘距离。
float64 radius_m                 # 大小按半径解释，m，必须大于 0；不包含代价地图的额外膨胀。
