# scan_planner

独立的 ROS 2 学习包，源码位于 `src/main.cpp`，可执行程序名为 `scan_planner_odom`。

在工作空间根目录编译：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select scan_planner
source install/setup.bash
ros2 run scan_planner scan_planner_odom
```

驱动可以单独编译：

```bash
colcon build --packages-select odin_ros_driver
```

节点订阅 `odin1/odometry` 和 `/odin1/cloud_slam`，通过话题接收驱动或回放数据，不依赖驱动包参与编译。现有 `start_odometry.sh` 仍用于启动驱动。

当前节点转发里程计为 TF，并将输入点云转换为 PCL 点云，执行预处理、RANSAC 地面平面分割及结果发布。坐标处理和几何结果的当前限制见[工作空间 README](../../README.md#学习时需要注意的实现边界)。

## 点云预处理

`include/scan_planner/cloud_preprocessor.hpp` 提供可复用的 `CloudPreprocessor`，依次执行：

1. 去除 NaN/Inf 和距离范围外的点；
2. Statistical Outlier Removal 离群点滤除；
3. 体素降采样；
4. 根据里程计姿态消除 roll/pitch，使 Z 轴与重力方向对齐。

节点会将结果发布到 `/cloud/downsampled`。参数可在启动时覆盖：

```bash
ros2 run scan_planner scan_planner_odom --ros-args \
  -p cloud.voxel_size:=0.03 -p cloud.min_range:=0.5 \
  -p cloud.max_range:=8.0 -p cloud.sor_mean_k:=16
```

其他 C++ 节点可链接本包并包含头文件后直接调用：

```cpp
scan_planner::CloudPreprocessor processor;
auto result = processor.process(input_cloud, imu_orientation);
```

也可以单独调用某一步：

```cpp
auto valid = processor.filterFiniteAndRange(input_cloud);
auto clean = processor.removeOutliers(*valid);
auto small = processor.voxelDownsample(*clean);
auto level = processor.alignToGravity(*small, imu_orientation);
```

## 地面与障碍物分离

`CloudCut` 使用 RANSAC 平面模型估计地面，并将点云分为地面和障碍物：

```cpp
scan_planner::CloudCut cutter;
auto result = cutter.separate(level_cloud);
// result.ground、result.obstacles：两类点云
// result.point_heights、point_normals、point_slopes_deg：与输入点索引对应的几何信息
```

节点发布 `/cloud/ground` 和 `/cloud/obstacles`。RANSAC 参数可通过
`cloud_cut.distance_threshold`、`cloud_cut.max_iterations` 和
`cloud_cut.max_ground_slope_deg` 调整。

包的许可证暂标为 `TODO`，由项目作者决定。
