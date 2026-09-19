# 四模块布局整理计划

## planner 单一入口与库边界（当前要求）

用户进一步明确 `planner/src` 只应有一个 `main.cpp`，通过头文件调用库。
本轮仅整理 planner，不重新改动其他三个模块。

- 将当前 6 个节点/调度 `.cpp` 和 `plan_manage` 头移入
  `third_party/planner_runtime`；说明该目录为本项目适配的 ROS 集成实现。
- `include/planner/planner.h` 声明节点创建接口，库负责持有算法与节点状态。
  `src/main.cpp` 统一负责 ROS 初始化、执行器、异常报告和关闭。
- 同一个 main 源文件继续构建原有 4 个可执行程序，各自链接对应运行库。
  CMake 选择对应节点工厂；进程、节点名、参数、话题和 launch 命令保持原状。
- 局部规划 FSM 由节点持有，在节点基类销毁前析构，保证回调和资源生命周期。
- 先跑规划回归基线；迁移测试引用，执行独立干净构建、全部规划回归、
  安装依赖和启动菜单检查。`src` 文件清单须严格为 `main.cpp`。
- 不修改算法与参数，不改已有失效保护分支，不增加依赖或 runtime 兜底路径。

以下记录此前两步整理，入口相关约定已由本节取代。

## 后续入口简化（已完成的前一步）

用户确认仅转发一次的 main 文件没有必要。保留四模块和 third_party 布局，
将 7 个运行入口直接改为所在节点 `.cpp` 中的全局 `main()`，移除 7 个薄入口
及 4 个仅声明 run 函数的头文件；算法接口头继续保留。

先运行当前回归基线，再调整节点入口、CMake 源文件列表与头文件安装规则。
同步当前 README，清除 install 中删除的 4 个旧入口头，编译四包并重跑回归。
既有白盒测试通过重命名 main 包含节点源码，检查其继续正常链接。
不修改算法、参数、异常处理、失效保护、执行器和线程数；不涉及 fallback 分支。
本轮验收为无单次转发入口和 run 声明残留、四包构建及已有测试通过。

以下为首次整理时的计划，其中薄 main 和公开 run 入口要求已由上述要求取代。

目标：`src` 顶层只保留 `map`、`planner`、`relocalization`、`odin_ros_driver`，
colcon 也只发现这四个包。算法按使用方收进模块的 `third_party`，模块通过一个
公开 `.h` 入口供轻量 `main` 调用；算法内部必要的头文件继续保留。

## 行为锁定

修改源码前运行现有 map、relocalization、ScanPlanner 与其算法包的全部测试。
保留粗定位/ICP/高频跟踪、占据图、全局/局部规划、控制器保护及旧点云节点。
保留节点名、话题名、坐标语义、45° 安装标定、所有已调参数与默认导航预览模式。
不启动真实硬件。驱动仅验证构建与入口符号。

## 分类与实施顺序

1. 合并边界：原 `plan_env`、`path_searching`、`bspline_opt`、`traj_utils` 变成
   `planner/third_party` 下的内部 CMake 库；OctoPlanner3D 同样移入此处。
   原全局规划 ROS 节点纳入 planner，原独立消息包的消息移至 `planner/msg`。
2. planner 的 ROS 包名统一为 `planner`，消息类型统一为 `planner/msg/*`。
   现有节点可执行名保持不变，更新 launch、Python/C++ 引用、脚本和测试。
   旧的 `scan_planner`、`scan_planner_msgs`、`bxi_octo_global_planner` 不留空壳包。
3. map/relocalization 的处理算法移至各自 `third_party`，共享 FreeDOM/BTC 仍由
   map 导出，relocalization 链接复用，不复制两份算法。
4. 每个模块增加 `include/<模块>/<模块>.h`，声明运行入口函数；现有启动逻辑
   移至实现文件，轻量 main 调用入口。驱动的 SDK 和点云算法归入 third_party，保留初始化/关闭顺序。
5. 保留算法许可与来源记录，移除多余 ROS 包清单及独立包构建文件。
   更新根目录脚本和文档。删除构建残留前先保存旧产物到临时归档。

## 明确不改的逻辑

现有空路径停车、定位失效门禁、BTC 旋转重试、TF 等待与地图转换重试属于已有
测试覆盖的兼容/失效处理。本轮只调整路径、链接及入口，不改变这些分支。
不增加新依赖，不把多节点强行合成一个进程，不改变轮式底盘的速度限制。

## 验收

- colcon 只识别指定的四个包；src 顶层目录与目标一致。
- 从只有 ROS 系统环境的独立构建/安装目录完整构建，不能借用旧包的 install。
- map/relocalization/planner 的全部已有回归迁移后通过，包括目标→全局路径→
  局部运动轨迹、定位失效撤销、控制器断流停车、原点云坐标回归。
- 每个 main 经模块入口头文件调用；无生产代码 include `.cpp`。
- 编译安装后的常规 `./start_mapping.sh` 能定位四包内的新节点位置。
- 明确记录 ROS 包名与 Bspline 消息类型变更，外部脚本/旧 rosbag 需要使用新类型。

## 分工与复核

planner 合并与 map/relocalization 整理由独立实现者负责；主线程负责驱动入口、
启动脚本、整体文档及独立构建验收。独立只读复核检查布局与功能边界。
