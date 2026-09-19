# 移植来源

2026-09-18 从相邻工作区 `../3D_nav` 移植；其 Git HEAD 为
`06f7ea2b2bcd931d27e83456dafed02743fed635`。来源包含未提交的修复和新增测试，
复制的是当时工作区内容，不能仅用该 commit 重建完全相同的源码。

对应关系：

- `3D_nav/src/scan_planner` → 本项目 `src/planner`，ROS 包名改为 `planner`。
- `plan_env`、`path_searching`、`bspline_opt`、`traj_utils` 移入 `src/planner/third_party`，`scan_planner_msgs` 消息移入 `src/planner/msg`。
- `bxi_octo_global_planner` ROS 节点移入 `planner` 包，`OctoPlanner3D-ROS2` 移入 `src/planner/third_party`。

保留源文件中的作者和许可声明。SCAN-Planner 各包的原 manifest 声明为 Apache-2.0；
OctoPlanner3D 的 LICENSE 及 OctoMap 头文件中的独立许可声明随源码保留。
原有点云学习文件保留其原先尚待作者确定的 `TODO` 许可标记。
OctoMap 沿用来源自带的 x86-64 共享库，安装时移除来源机器的运行路径；
迁移至 ARM 等其他架构时，需要提供对应架构的兼容库。

本项目适配：原点云处理算法整理到 `src/planner/third_party/cloud_processing`，
保留 `scan_planner_odom`；使用本项目 BTC/ICP 快照和定位状态；
动态 TF 采用 best-effort；定位失效取消路径并禁用控制；控制器检查里程计、使能心跳；
修正规划初速度的坐标变换。设备 BIN 地图启动方式替换为 `navigation.launch.py`，
旧的 launch 名称作为新入口的别名。安装角、机器人尺寸和地面高度由运行参数指定。

2026-09-19 按用户确认，恢复原项目已调好的 45° 安装俯仰角；规划与控制器
YAML 参数保持来源值，启动脚本、launch 与里程计适配器采用相同默认安装角。

集成验证中修复了 OctoPlanner3D 对单层地面地图的搜索边界：使用已有高度余量
允许搜索地面上方的自由格，保留真实占据范围、地面支撑和碰撞检查。
RViz 目标与起点均按配置的距离上限投影到可通行地面层，再执行连通性检查。
同时修复轨迹工具 `getMeanVel()` 缺少返回值，并补充对应回归测试。

规划入口整理：`src` 仅保留公共 `main.cpp`，原六个节点/调度实现及
`plan_manage` 头文件归入 `third_party/planner_runtime`。该目录为本项目
适配的集成代码；`include/planner/planner.h` 提供节点创建接口，运行库静态链接。
