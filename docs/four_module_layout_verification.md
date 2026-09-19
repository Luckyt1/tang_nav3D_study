# 四模块布局验收

日期：2026-09-19。实施计划见 [four_module_layout_plan.md](four_module_layout_plan.md)，
目录及运行方式见 [README](../README.md)。

## 结构与入口

- `src` 顶层、`package.xml` 清单和 `colcon list` 均只有 `map`、`planner`、
  `relocalization`、`odin_ros_driver`。
- `planner/src` 只有 `main.cpp`，通过 `include/planner/planner.h` 创建库中的节点。
  6 个节点/调度实现和 `plan_manage` 头位于 `third_party/planner_runtime`。
  其余三个模块的 main 与对应 ROS 节点实现在同一 `.cpp` 中。
- planner 四个程序编译同一个 main 源文件，分别链接具名工厂和静态运行库。
  节点、FSM 和执行器在关闭 ROS 前依次释放；原进程划分保留。
- 算法集中到对应模块 `third_party`；map 导出的 FreeDOM/BTC 库仍供
  relocalization 复用。规划内部库显式静态链接。
- 生产代码没有 `#include` `.cpp`；现有白盒测试的直接源码包含方式保留。
- ROS 包名统一为 `planner`；规划消息为 `planner/msg/Bspline` 和
  `planner/msg/DataDisp`。节点名、可执行名和话题名保留。

## 验证结果

| 检查 | 结果 |
| --- | --- |
| 重构前回归 | 原 8 个相关包通过；另手动补跑原点云坐标回归通过 |
| 全新构建 | 仅加载 `/opt/ros/humble` 环境，map / driver / relocalization 与 planner 分两批全部构建通过 |
| 后续入口合并 | 修改前回归通过，合并后四包重新构建通过，再次回归通过 |
| planner 库化 | 再次归档并清空 planner 的 build/install；全新构建通过，19 组规划回归复测全通过 |
| CTest | map 4/4、relocalization 4/4、planner 19/19 通过 |
| colcon 汇总 | `182 tests, 0 errors, 0 failures, 0 skipped` |
| 安装环境 | 新 shell 仅发现本工作区四包，无旧算法包/旧消息包索引 |
| 消息 | Python 从新 install 导入 `planner.msg`，Bspline / DataDisp 的 C typesupport 均可加载 |
| 动态链接 | 10 个已安装 C++ 可执行文件的 ldd 无缺失库，不依赖 build 或旧归档目录 |
| 全局规划依赖 | 已安装 OctoMap 从 `install/planner/lib/planner` 加载 |
| 启动菜单 | 3 个模式的环境/节点检查和命令生成通过；拦截所有 tmux 操作，未启动硬件 |
| 静态检查 | Python 语法、shell 语法、四包 XML、唯一入口清单、工厂头安装、`git diff --check` 均通过 |

入口合并及后续库化均经独立只读复核。库化后的 FSM、manager 和内部头文件
逐字节保持一致；原算法、配置、launch、脚本及消息哈希未变。两个白盒测试
仅更新源码包含路径、移除旧 main 重命名宏。安装后的 10 个 C++ 可执行文件
依赖检查及三个启动菜单模拟检查再次通过。

库化后首轮 19 组测试中，点云坐标测试在等待 ROS 节点发现时超时一次；
未修改代码或放宽断言，单独复测通过，随后完整 19 组复测通过。
这次超时未能复现，原因尚未确认，首轮失败日志保留。

规划集成回归覆盖目标到全局路径和局部运动轨迹、定位失效撤销路径、
控制器断流停车、启动参数及点云坐标保持。测试在独立 ROS 域执行。

算法文件移动前后逐字节核对；规划和控制器 YAML 与原 `3D_nav` 配置一致，
驱动 `control_command.yaml` 保留用户原有修改。map 的单线程和重定位的
两个执行线程、回调组及 TF 监听线程保持原样。

本次构建平台为 x86-64 / ROS 2 Humble；未做真实硬件运行和 ROS 1 构建。
编译中仍有既有第三方代码及 CMake 的警告，本次没有修改其算法逻辑。

## 本次日志及可恢复归档

- 构建：`/tmp/four_module_build.log`、`/tmp/four_module_planner_build.log`。
- 回归：`/tmp/four_module_tests.log`、`/tmp/four_module_planner_tests.log`。
- 入口合并基线、构建、回归：`/tmp/main_cleanup_baseline.log`、
  `/tmp/main_cleanup_build.log`、`/tmp/main_cleanup_tests.log`。
- 库化基线、构建、首轮与复测：`/tmp/planner_library_baseline.log`、
  `/tmp/planner_library_build.log`、`/tmp/planner_library_tests.log`、
  `/tmp/planner_library_retest.log`。
- 启动命令：`/tmp/four_module_startup.QZEO8d/commands.log`。
- 合并前入口与构建文件备份：`/tmp/main_entry_cleanup_before_qvq4mjlr/`。
- 库化前源码及 planner build/install：`/tmp/planner_library_before_e6t51vum/`。
- 旧 build/install：`/tmp/four_module_previous_20260919_155425/`。
- 原规划子包：`/tmp/tang_nav3d_planner_layout_archive/`。

这些 `/tmp` 文件为本机临时证据及备份，不是运行依赖，系统清理后可能消失。
