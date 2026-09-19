# Odin SDK 与点云算法

这些文件来自本模块原有的 Manifold Tech Odin 驱动，未改变算法实现：

- `odin_sdk/include`：原 `include/lidar_api*.h`。
- `odin_sdk/lib`：原 `lib/liblydHostApi_{amd,arm}.a`，分别对应两种平台。
- `cloud_processing`：原 `src` / `include` 中的点云深度转换、重投影、
  彩色点云渲染和多项式相机模型。

保留各文件原有版权声明；许可证与 SDK 使用约束见模块根目录的
`LICENSE` 和 `README.md`。SDK 为上游预编译二进制，本次未重新编译。
ROS 节点、设备连接和生命周期管理位于模块 `src` 中。
