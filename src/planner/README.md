# planner

本包位于 `src/planner`，ROS 包名为 `planner`。包含从相邻 `3D_nav` 移植的规划与控制节点，以及原有点云学习节点 `scan_planner_odom`。移植来源及适配见 [SOURCE_NOTICE.md](SOURCE_NOTICE.md)。

## 代码入口

`src` 中只有一个 `main.cpp`，负责 ROS 初始化、通过 `planner.h` 创建节点、
运行执行器和关闭 ROS。规划、控制和点云处理实现作为库从 `third_party` 链接进来。

```text
planner/
├── src/main.cpp                    # 唯一入口源文件，管理 ROS 生命周期
├── include/planner/planner.h        # 创建节点的统一接口
└── third_party/
    ├── planner_runtime/            # 规划调度、控制器、ROS 适配节点实现
    │   ├── src/
    │   └── include/plan_manage/
    ├── plan_env/                   # 占据图
    ├── path_searching/              # 路径搜索
    ├── bspline_opt/                 # B 样条优化
    ├── traj_utils/                  # 轨迹工具
    ├── OctoPlanner3D-ROS2/          # 全局规划算法
    └── cloud_processing/           # 点云算法
```

CMake 用这个 `main.cpp` 构建原有四个程序，在编译时通过 `PLANNER_NODE_FACTORY`
选择创建函数，并链接对应运行库。各程序继续独立运行，全局规划与控制器有各自的执行器。

| 可执行名 | `planner.h` 接口 | 链接的运行库 |
| --- | --- | --- |
| `scan_planner_node` | `createLocalPlannerNode()` | `local_planner_runtime` |
| `octo_global_planner_node` | `createGlobalPlannerNode()` | `global_planner_runtime` |
| `closed_loop_controller` | `createControllerNode()` | `controller_runtime` |
| `scan_planner_odom` | `createCloudPreprocessorNode()` | `cloud_preprocessor_runtime` |

`planner_runtime` 是本项目移植和适配后的 ROS 集成代码；来源见其中的 `NOTICE.md`。
局部规划节点持有 FSM，确保其状态、订阅和定时器覆盖节点整个运行期间。

## 导航预览

在工作区根目录编译完整链路：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-up-to odin_ros_driver planner
source install/setup.bash
./start_mapping.sh
```

选择 **3）导航预览**，选择已保存的 BTC 地图。脚本启动里程计、现有 BTC/ICP 定位、全局规划、局部规划和 RViz。定位有效后，用 RViz 的 **2D Goal Pose** 在地图中指定目标；黄色为全局路径，蓝色为局部引导路径，Marker 显示优化轨迹。该入口默认不启动速度控制器。

规划使用 `static_map.pcd`，定位使用同一快照的 BTC 数据；不用设备内置的 BIN 重定位地图。单独接入已运行的里程计和定位时：

```bash
ros2 launch planner navigation.launch.py \
  map_directory:='/完整路径/maps/某次快照' start_relocalization:=false
```

`navigation.launch.py` 不启动雷达驱动。`start_relocalization` 默认 true，`start_rviz` 默认 true，`start_controller` 默认 false。`odin_scan_planner.launch.py` 是此入口的别名，参数也使用 `map_directory`。

ROS 包名现在是 `planner`。原 `scan_planner_msgs/msg/Bspline` 与
`scan_planner_msgs/msg/DataDisp` 已合并为 `planner/msg/Bspline` 和
`planner/msg/DataDisp`；Python 代码使用 `from planner.msg import Bspline`。

## 已移植的功能

| 模块 | 功能 |
| --- | --- |
| `octo_global_planner_node` | PCD 转 OctoMap、地面支撑搜索、全局路径及可视化 |
| `third_party/plan_env` | 实时点云的三维滑动占据图、射线清障、机器人碰撞膨胀 |
| `third_party/path_searching` / `third_party/bspline_opt` | 局部 A*、B 样条优化、碰撞及动力约束检查 |
| `scan_planner_node` | 跟随全局路径、周期/碰撞重规划、规划失败停车 |
| `closed_loop_controller` | 轨迹跟踪，输出车体系 `Twist`；独立启用 |

当前局部 A* 是 XY 八邻域搜索，默认按里程计高度规划；它没有完整的坡度、台阶通行评估或步态控制。三维占据表示和全局地面支撑搜索不等于已经验证可上下楼梯。

## 坐标与定位有效期

`/odin1/cloud_slam` 保持 `odom` 坐标；局部栅格、引导路径和 B 样条均在 `odom`。RViz 目标与静态地图在 `map`：

```text
/goal_pose (map)
  → relocalization_bridge → /navigation/map_goal
  → Octo 全局规划 → /map/initial_path (map)
  → relocalization_bridge → /initial_path (odom)
  → scan_planner_node → /planning/bspline (odom)
```

桥接同时检查 `/relocalization/tracking_status` 与 TF。定位失效或状态心跳超过 0.5 秒时，清空局部路径并发布 `navigation/enabled=false`。动态 TF 使用 best-effort，静态外参仍使用 reliable/transient-local。控制器独立检查里程计时间戳前进和使能心跳；失效后清除旧轨迹，恢复需要新轨迹。

`base_link_odometry.py` 把 IMU 安装旋转应用到 base_link 姿态与车体系速度，当前模型假设两者原点重合。FSM 再把车体系速度旋转到 odom 系，作为规划初速度。

## 机器人参数

沿用用户在 `3D_nav` 中已调好的参数：`robot_radius=0.3`、`body_height=0.35`、`ground_height=-1.1`，安装俯仰角 `mount_pitch=0.7853981633974483`（45°），roll/yaw 均为 0。地面高度是 **odom 系中的 Z**，安装角单位为弧度。规划与控制器 YAML 参数与来源保持一致。

菜单入口可通过 `ROBOT_RADIUS`、`BODY_HEIGHT`、`GROUND_HEIGHT`、`MOUNT_ROLL`、`MOUNT_PITCH`、`MOUNT_YAW` 环境变量覆盖。直接 launch 使用同名小写参数。控制增益、横向速度能力等在 `config/controllers.yaml`；轮式差速底盘应将 `max_vy` 设为 0。需要实际速度输出时，显式设置 `start_controller:=true`，并指定底盘接收的 `cmd_vel_topic`。

## 验证

```bash
ROS_DOMAIN_ID=206 ROS_LOCALHOST_ONLY=1 colcon test --executor sequential --packages-select planner
colcon test-result --verbose
```

测试包含占据图更新、搜索、样条边界、定位过期清路径、控制器失联停车，以及使用合成地面运行完整规划 launch。测试运行在隔离 ROS domain，不启动硬件。实车跟踪和楼梯通行尚需实际平台验证。

## 原有点云学习节点

在工作空间根目录编译：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-up-to planner
source install/setup.bash
ros2 run planner scan_planner_odom
```

驱动可以单独编译：

```bash
colcon build --packages-select odin_ros_driver
```

节点订阅 `odin1/odometry` 和 `/odin1/cloud_slam`，通过话题接收驱动或回放数据；导航的运行依赖由包清单统一声明。现有 `start_odometry.sh` 仍用于启动驱动。

当前节点转发里程计为 TF，并将输入点云转换为 PCL 点云，执行预处理、RANSAC 地面平面分割及结果发布。坐标处理和几何结果的当前限制见[工作空间 README](../../README.md#学习时需要注意的实现边界)。

## 点云预处理

`third_party/cloud_processing/include/scan_planner/cloud_preprocessor.hpp` 提供可复用的 `CloudPreprocessor`，依次执行：

1. 去除 NaN/Inf 和距离范围外的点；
2. Statistical Outlier Removal 离群点滤除；
3. 体素降采样；

节点输入 `/odin1/cloud_slam` 已在 `odom` 坐标系中，调用 `process(input_cloud)` 保持点云方向，不再传入设备姿态进行二次旋转。输出保留输入消息的坐标系与时间戳。

节点会将结果发布到 `/cloud/downsampled`。参数可在启动时覆盖：

```bash
ros2 run planner scan_planner_odom --ros-args \
  -p cloud.voxel_size:=0.03 -p cloud.min_range:=0.5 \
  -p cloud.max_range:=8.0 -p cloud.sor_mean_k:=16
```

其他 C++ 节点可链接本包并包含头文件后直接调用：

```cpp
scan_planner::CloudPreprocessor processor;
auto result = processor.process(input_cloud);
```

也可以单独调用某一步：

```cpp
auto valid = processor.filterFiniteAndRange(input_cloud);
auto clean = processor.removeOutliers(*valid);
auto small = processor.voxelDownsample(*clean);
```

## 地面与障碍物分离

`CloudCut` 使用 RANSAC 平面模型估计地面，并将点云分为地面和障碍物：

```cpp
scan_planner::CloudCut cutter;
auto result = cutter.separate(processed_cloud);
// result.ground、result.obstacles：两类点云
// result.point_heights、point_normals、point_slopes_deg：与输入点索引对应的几何信息
```

节点发布 `/cloud/ground` 和 `/cloud/obstacles`。RANSAC 参数可通过
`cloud_cut.distance_threshold`、`cloud_cut.max_iterations` 和
`cloud_cut.max_ground_slope_deg` 调整。

原有点云学习算法已整理到 `third_party/cloud_processing`；文件的许可证暂标为 `TODO`，由项目作者决定；移植部分保留其来源许可，见 `SOURCE_NOTICE.md`。

## 点云方向回归检查

在工作空间根目录编译后运行：

```bash
source /opt/ros/humble/setup.bash
python3 src/planner/test/test_cloud_frame.py \
  install/planner/lib/planner/scan_planner_odom
```

测试使用本机独立 ROS domain 启动处理节点，模拟航向过零及 roll/pitch 变化，验证 `odom` 系点云的坐标、frame_id 和时间戳不随设备姿态改变；不需要连接 Odin 设备。
