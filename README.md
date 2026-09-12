# tang_nav3D_study

一个围绕三维导航感知的个人学习项目，主要包含 **Odin 设备的 ROS 驱动**和基于 **PCL 的点云快速处理实验**。通过实际点云数据学习 ROS 2 通信、坐标变换、滤波、降采样和地面分割，逐步探索楼梯等几何结构的识别。

当前重点是打通“接收点云 → 预处理 → 地面与非地面分离 → 可视化”的流程。“快速处理”指通过滤波和降采样减少后续计算量，项目暂未提供处理速度的基准测试。

## 项目组成

| 模块 | 内容 |
| --- | --- |
| [`odin_ros_driver`](src/odin_ros_driver/README.md) | Odin 传感器接入、点云与里程计等数据发布，以及建图、重定位和图像相关示例 |
| [`scan_planner`](src/scan_planner/README.md) | 个人学习包，实现点云预处理、RANSAC 平面分割及结果发布 |

仓库中的设备和驱动名称为 **Odin**，对应包名 `odin_ros_driver`。

```text
tang_nav3D_study/
├── src/
│   ├── odin_ros_driver/          # Odin 驱动、SDK、配置和启动文件
│   └── scan_planner/
│       ├── include/scan_planner/
│       │   ├── cloud_preprocessor.hpp
│       │   └── cloud_cut.hpp
│       └── src/
│           ├── main.cpp         # ROS 2 节点、话题和 TF
│           ├── cloud.cpp        # 点云预处理
│           └── cloud_cut.cpp    # 地面平面估计与点云分离
├── build/                      # 编译生成
├── install/                    # 安装空间，编译生成
└── log/                        # 构建日志，编译生成
```

本地地图可放在 `maps/` 下；该目录被 Git 忽略，不随仓库分发。当前学习节点从 ROS 话题接收数据，尚未提供直接读取 PCD 文件的运行入口。

## 当前处理流程

```text
Odin 驱动或 rosbag 回放
    ├── /odin1/odometry → 保存最新姿态、转发 TF
    └── /odin1/cloud_slam
            → 去除 NaN/Inf、距离范围裁剪
            → 统计离群点滤波（SOR）
            → 体素降采样（VoxelGrid）
            → 基于姿态的 roll/pitch 校正实验
            ├── /cloud/downsampled
            └── RANSAC 平面拟合、坡度检查
                    ├── /cloud/ground
                    └── /cloud/obstacles
```

- `CloudPreprocessor` 封装预处理流程，也可以分别调用各个步骤。
- `CloudCut` 拟合一个候选平面，检查其倾角，再分离内点和剩余点；拟合失败或倾角超限时，输入点归入非地面结果。
- 地面与障碍物话题用于观察几何分割结果；楼梯识别和完整导航规划仍是后续学习内容。

## 环境与依赖

本文命令按 **Ubuntu 22.04 / ROS 2 Humble** 环境编写。

- 构建工具：`colcon`、CMake、支持 C++17 的编译器。
- 点云处理：PCL、Eigen、`pcl_conversions`。
- ROS 2：`rclcpp`、`sensor_msgs`、`nav_msgs`、`geometry_msgs`、`tf2_ros`。
- 驱动还涉及 OpenCV、yaml-cpp、libusb 等依赖，以及设备 USB 权限配置。

首次接入设备时，请先阅读[驱动说明](src/odin_ros_driver/README.md)，核对 SDK/固件要求、依赖和设备配置。`scan_planner` 通过话题使用数据，可以独立于驱动编译。

## 编译

以下命令均在工作空间根目录执行。

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select odin_ros_driver scan_planner
source install/setup.bash
```

仅学习点云处理、使用已有发布节点或 rosbag 数据时，可以只编译：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select scan_planner
source install/setup.bash
```

## 运行

### 1. 启动 Odin 驱动

设备连接和依赖配置完成后，在第一个终端执行：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run odin_ros_driver host_sdk_sample --ros-args \
  -p "config_file:=$PWD/src/odin_ros_driver/config/control_command.yaml"
```

该命令使用源码目录中的[驱动配置](src/odin_ros_driver/config/control_command.yaml)。其中 `custom_map_mode` 的含义为：

| 值 | 模式 |
| --- | --- |
| `0` | 里程计模式 |
| `1` | SLAM 建图模式，当前仓库配置值 |
| `2` | 重定位模式，需要配置已有地图的绝对路径 |

配置中的 `sendcloudslam` 控制 SLAM 点云发布。更完整的驱动演示可使用以下命令，它还会启动深度、重投影、图像叠加和 RViz2 相关节点，使用安装目录中的配置：

```bash
ros2 launch odin_ros_driver odin1_ros2.launch.py
```

### 2. 启动点云处理节点

在第二个终端执行：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run scan_planner scan_planner_odom
```

也可以使用 rosbag 回放提供输入；需保证点云与里程计话题名称、消息类型及坐标含义匹配。

### 3. 查看结果

```bash
ros2 topic list
ros2 topic hz /cloud/downsampled
rviz2
```

在 RViz2 中添加 `PointCloud2` 显示，分别选择以下话题。输入位于 `odom` 系时，Fixed Frame 可设为 `odom`；使用前请同时核对下文的坐标处理说明。

| 话题 | 消息类型 | 用途 |
| --- | --- | --- |
| `/odin1/odometry` | `nav_msgs/msg/Odometry` | 输入里程计和姿态，默认无命名空间时 |
| `/odin1/cloud_slam` | `sensor_msgs/msg/PointCloud2` | 输入点云 |
| `/cloud/downsampled` | `sensor_msgs/msg/PointCloud2` | 预处理后的点云 |
| `/cloud/ground` | `sensor_msgs/msg/PointCloud2` | 通过坡度检查的候选地面平面内点 |
| `/cloud/obstacles` | `sensor_msgs/msg/PointCloud2` | 剩余点或分割失败时的输入点 |

`ros2 topic hz` 用于观察话题接收频率，不代表单帧算法耗时。

## 参数调整

参数在节点启动时读取，可以通过 `--ros-args -p` 覆盖：

```bash
ros2 run scan_planner scan_planner_odom --ros-args \
  -p cloud.voxel_size:=0.03 \
  -p cloud.min_range:=0.5 \
  -p cloud.max_range:=8.0 \
  -p cloud.sor_mean_k:=16 \
  -p cloud_cut.distance_threshold:=0.08
```

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `cloud.voxel_size` | `0.03` | 体素边长，m |
| `cloud.min_range` | `0.5` | 到输入坐标原点的最小保留距离，m |
| `cloud.max_range` | `8.0` | 到输入坐标原点的最大保留距离，m |
| `cloud.sor_mean_k` | `16` | 统计离群点滤波的邻居数量 |
| `cloud.sor_stddev` | `1.0` | 离群点阈值的标准差倍数 |
| `cloud_cut.distance_threshold` | `0.08` | 平面内点距离阈值，m |
| `cloud_cut.max_iterations` | `100` | RANSAC 最大迭代次数 |
| `cloud_cut.max_ground_slope_deg` | `20.0` | 候选平面允许的最大倾角，度 |

## 学习时需要注意的实现边界

- **坐标处理**：距离裁剪使用输入点的 `x²+y²+z²`，因此参考的是输入坐标原点，并不自动跟随机器人位置。节点使用最新里程计姿态进行校正，尚未实现点云与姿态的时间同步；校正后仍沿用输入消息的坐标系标识。坐标转换和重力对齐需要继续验证，不能仅凭 RViz 显示正常判断正确性。
- **平面与语义**：当前 RANSAC 只估计一个平面，再做倾角检查，没有地面高度或多平面关系约束；水平桌面也可能成为候选。`/cloud/obstacles` 表示非地面分割结果，并不意味着完成了物体类别识别。
- **几何结果**：`point_heights` 是点到估计平面的有符号法向距离，不是统一朝上的竖直离地高度；`point_normals` 和 `point_slopes_deg` 为各点重复填写同一个候选平面的法向量和倾角，尚未估计逐点邻域特征。使用时需检查 `GroundPlaneResult::valid`。

## 后续学习方向

- [ ] 增加离线 PCD 读取与分割结果保存。
- [ ] 验证坐标变换、重力对齐与消息时间同步。
- [ ] 学习 KD-tree 邻域搜索、局部法向量估计和区域生长。
- [ ] 根据踏面高差、间距和排列关系区分楼梯与普通地面。
- [ ] 记录处理耗时、点数变化和分割效果，补充可重复的测试数据。

## 来源与许可

Odin 驱动包含第三方代码与 SDK，来源、许可及使用说明见[驱动文档](src/odin_ros_driver/README.md)和对应文件。个人学习包 `scan_planner` 的 `package.xml` 中许可证目前仍标为 `TODO`，尚未统一确定项目许可证。
