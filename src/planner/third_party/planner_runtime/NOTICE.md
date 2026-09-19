# 规划运行库

本目录收纳本项目移植和适配后的 ROS 集成代码，并非未修改的上游快照。
它与 `plan_env`、`path_searching`、`bspline_opt`、`traj_utils`、
`OctoPlanner3D-ROS2` 和 `cloud_processing` 一起由 planner 构建为内部库。

- `scan_replan_fsm.cpp`、`planner_manager.cpp`、`include/plan_manage`：
  原 `planner/src` 和 `planner/include` 中的 SCAN-Planner 规划调度实现。
- `scan_planner_node.cpp`：持有规划 FSM 的 ROS 节点及创建接口。
- `closed_loop_controller.cpp`：原轨迹控制节点及创建接口。
- `octo_global_planner_node.cpp`：原 Octo 全局规划 ROS 适配节点及创建接口。
- `cloud_preprocessor_node.cpp`：原点云学习节点及创建接口。

源码和许可来源见模块根目录 `SOURCE_NOTICE.md`。算法与参数未因移动而更改。
这些库不定义 `main()`；唯一入口位于 `planner/src/main.cpp`，通过
`planner/include/planner/planner.h` 创建所需节点并管理 ROS 生命周期。
