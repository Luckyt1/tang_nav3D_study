# tang_nav3D_study

一个围绕三维导航的个人学习项目，包含 **Odin 驱动、FreeDOM 建图、BTC/ICP 重定位和 ScanPlanner 导航规划**，同时保留基于 PCL 的点云处理实验。

当前链路为“点云与里程计 → 建图/重定位 → 全局路径 → 局部避障轨迹 → RViz 预览”。速度控制器可以独立启用，实车参数和跟踪效果仍需在实际平台验证。

## 项目组成

| 模块 | 内容 |
| --- | --- |
| [`map`](src/map/README.md) | FreeDOM 建图、子图和 BTC 描述子、地图快照保存 |
| [`planner`](src/planner/README.md) | OctoMap 全局规划、ScanPlanner 局部规划、B 样条优化与控制；包含规划消息和原点云学习节点 |
| [`relocalization`](src/relocalization/README.md) | BTC 粗定位、ICP 精定位、连续跟踪与 `map → odom` 发布 |
| [`odin_ros_driver`](src/odin_ros_driver/README.md) | Odin 接入、点云与里程计发布、设备 SDK 和图像工具 |

`src` 和 colcon 均只包含以上四个模块。算法成为所属模块的内部库，
不再单独作为 ROS 包。共享 FreeDOM/BTC 由 `map` 导出，`relocalization` 链接复用。

```text
src/
├── map/
│   ├── src/map_node.cpp          # ROS 接口、main 和启动
│   └── third_party/              # mapping / freedom / btc
├── planner/
│   ├── src/main.cpp              # 唯一入口源文件，调用运行库
│   ├── include/planner/planner.h  # 节点创建接口
│   ├── msg/                      # Bspline、DataDisp
│   ├── launch/                   # navigation.launch.py
│   └── third_party/              # 规划运行库、占据图、搜索、优化、轨迹等
├── relocalization/
│   ├── src/relocalization_node.cpp # 定位节点及 main
│   └── third_party/localization/ # 粗定位、精定位、跟踪
└── odin_ros_driver/
    ├── src/driver_node.cpp       # main、设备初始化、ROS 接口与关闭
    └── third_party/              # odin_sdk / cloud_processing
```

`planner/src` 仅保留 `main.cpp`，通过 `planner.h` 创建运行库中的节点；
具体实现和算法均在 `planner/third_party`。同一入口源文件构建原有四个程序，
各自保持独立进程。其余模块的入口仍位于对应节点 `.cpp`，算法头文件按库保留。

布局调整后，ROS 包名 `scan_planner` 统一为 `planner`，全局规划节点也由
`planner` 提供。原 `scan_planner_msgs/msg/Bspline`、`DataDisp` 改为
`planner/msg/Bspline`、`DataDisp`；外部脚本和旧 rosbag 的类型引用需要相应更新。

本地地图放在 `maps/` 下；该目录被 Git 忽略，不随仓库分发。导航读取快照中的静态 PCD 和 BTC 数据，原有点云学习节点继续从 ROS 话题接收数据。

## 原有点云学习流程

```text
Odin 驱动或 rosbag 回放
    ├── /odin1/odometry → 转发 TF
    └── /odin1/cloud_slam
            → 去除 NaN/Inf、距离范围裁剪
            → 统计离群点滤波（SOR）
            → 体素降采样（VoxelGrid）
            → 保留输入坐标系与时间戳
            ├── /cloud/downsampled
            └── RANSAC 平面拟合、坡度检查
                    ├── /cloud/ground
                    └── /cloud/obstacles
```

- `CloudPreprocessor` 封装预处理流程，也可以分别调用各个步骤。
- `CloudCut` 拟合一个候选平面，检查其倾角，再分离内点和剩余点；拟合失败或倾角超限时，输入点归入非地面结果。
- 地面与障碍物话题用于观察几何分割结果；楼梯通行性识别及步态控制尚未实现，导航链路见下方运行说明。

## 环境与依赖

本文命令按 **Ubuntu 22.04 / ROS 2 Humble** 环境编写。

- 构建工具：`colcon`、CMake、支持 C++17 的编译器。
- 点云处理：PCL、Eigen、`pcl_conversions`。
- ROS 2：`rclcpp`、`sensor_msgs`、`nav_msgs`、`geometry_msgs`、`tf2_ros`。
- 驱动还涉及 OpenCV、yaml-cpp、libusb 等依赖，以及设备 USB 权限配置。

首次接入设备时，请先阅读[驱动说明](src/odin_ros_driver/README.md)，核对 SDK/固件要求、依赖和设备配置。`planner` 通过话题使用数据，点云学习节点也可接收 rosbag 回放。

## 编译

以下命令均在工作空间根目录执行。

```bash
source /opt/ros/humble/setup.bash
colcon build --executor sequential
source install/setup.bash
```

显式构建规划模块及其工作区依赖：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-up-to planner
source install/setup.bash
```

## 运行

### 一键建图或重定位

编译好对应的 `map` 或 `relocalization` 包后，在桌面终端运行：

```bash
./start_mapping.sh
```

选择 `1` 建图、`2` 重定位或 `3` 导航预览，脚本会同时启动 Odin 里程计、所选处理节点和 RViz2（需安装 `ros-humble-rviz2`）。建图模式使用 `config/mapping.rviz`，在 `odom` 系显示静态地图和子图位姿；重定位使用 `config/relocalization.rviz`，在 `map` 系显示已保存地图、BTC 候选位姿、ICP 对齐点云和随里程计更新的跟踪位姿。精定位成功时日志显示 `ICP refine: success=true`；有效期内独立发布 `map -> odom` 和 `/relocalization/tracked_pose`，默认最高 50 Hz，实际取决于里程计 TF 更新率。详细接口和过期状态见 [`src/relocalization/README.md`](src/relocalization/README.md)。

模式 `3` 使用同一 BTC 快照启动定位和 ScanPlanner/Octo 全局规划；定位有效后在 RViz 用 **2D Goal Pose** 指定目标，显示全局路径、局部轨迹和障碍图，默认不输出速度。尺寸、地面高度和安装角沿用用户在 `3D_nav` 中已调好的配置，详见 [规划模块说明](src/planner/README.md)。

RViz 与节点使用相同的 `ROS_DOMAIN_ID`（默认 `13`），日志位于同一 tmux 会话的 `rviz` 窗口。运行 `./stop_mapping.sh` 会一起关闭节点和 RViz；停止前需先保存建图结果。

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
| `0` | 里程计模式，当前仓库默认值 |
| `1` | SLAM 建图模式，需要手动开启 |
| `2` | 重定位模式，需要配置已有地图的绝对路径 |

默认使用 Odin 提供里程计和点云，不启用设备端 SLAM 建图及地图保存；数据录制也保持关闭（`recorddata: 0`）。需要保存设备地图时，先将 `custom_map_mode` 改为 `1` 并重启驱动，再手动调用 `save_map`。

配置中的 `sendcloudslam` 控制 SLAM 点云发布。更完整的驱动演示可使用以下命令，它还会启动深度、重投影、图像叠加和 RViz2 相关节点，使用安装目录中的配置：

```bash
ros2 launch odin_ros_driver odin1_ros2.launch.py
```

### 2. 启动点云处理节点

在第二个终端执行：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run planner scan_planner_odom
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
ros2 run planner scan_planner_odom --ros-args \
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

- **坐标处理**：`/odin1/cloud_slam` 已在 `odom` 系中，节点预处理不再使用设备姿态旋转点云，输出保留输入坐标系与时间戳。距离裁剪使用输入点的 `x²+y²+z²`，参考的是输入坐标原点，并不自动跟随机器人位置。需要接入其他坐标系时，应先明确坐标变换关系。
- **平面与语义**：当前 RANSAC 只估计一个平面，再做倾角检查，没有地面高度或多平面关系约束；水平桌面也可能成为候选。`/cloud/obstacles` 表示非地面分割结果，并不意味着完成了物体类别识别。
- **几何结果**：`point_heights` 是点到估计平面的有符号法向距离，不是统一朝上的竖直离地高度；`point_normals` 和 `point_slopes_deg` 为各点重复填写同一个候选平面的法向量和倾角，尚未估计逐点邻域特征。使用时需检查 `GroundPlaneResult::valid`。

## 后续学习方向

- [ ] 增加离线 PCD 读取与分割结果保存。
- [ ] 验证坐标变换、重力对齐与消息时间同步。
- [ ] 学习 KD-tree 邻域搜索、局部法向量估计和区域生长。
- [ ] 根据踏面高差、间距和排列关系区分楼梯与普通地面。
- [ ] 记录处理耗时、点数变化和分割效果，补充可重复的测试数据。

## 来源与许可

Odin 驱动包含第三方代码与 SDK，来源、许可及使用说明见[驱动文档](src/odin_ros_driver/README.md)和对应文件。个人学习包 `planner` 的 `package.xml` 中许可证目前仍标为 `TODO`，尚未统一确定项目许可证。
