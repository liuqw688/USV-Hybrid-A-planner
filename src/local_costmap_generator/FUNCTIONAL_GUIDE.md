# 局部膨胀代码导航

`src/local_costmap_node.cpp` 接收原图与静态航道JSON，按旋转栅格原点转换并扫描线填充航道掩膜，发布两张对齐地图。

`include/local_costmap_generator/inflation.hpp` 标记占用连通分量，分别计算所有硬障碍、独立障碍、高代价源距离场；保留硬缓冲，裁剪航道内岸边软梯度，输出独立障碍膨胀。

`test/test_inflation.cpp` 验证来源分离、独立小岛、未知、零膨胀半径及无航道数据的保守回退。
