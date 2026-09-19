# map：Odin 点云接入 FreeDOM

本包的 ROS 2 建图节点与 `main()` 位于同一文件，处理算法集中放在 `third_party`：

- `src/map_node.cpp`：ROS 参数、点云与 TF 输入、处理器调用、结果发布和 `main()`；保持单线程顺序更新地图。
- `third_party/mapping/src/freedom.cpp`：保留跨帧状态的 FreeDOM 处理器，接入官方算法并提取当前静态地图。
- `third_party/mapping/src/submap.cpp`：根据运动选择子图锚点，记录位姿，从最新静态地图裁剪局部子图，并保存地图、子图、位姿和 BTC 描述子。

处理器接口位于 `third_party/mapping/include/map/`。安装后仍使用 `map/freedom.hpp` 和 `map/submap.hpp`，并通过原有 `map::freedom_processing`、`map::submap_processing` 等 CMake 目标供重定位模块复用。`third_party/freedom` 保存固定版本的官方核心算法、许可证和移植说明；没有启动 ROS 1 节点或要求 ROS 1 桥接。

`third_party/btc` 为 [官方 BTC](https://github.com/hku-mars/btc_descriptor/tree/742af157036144edad9a8350330c0158ea1a40d5) 的提取/建索引核心，已移除 ROS 1、Ceres、GTSAM 和 TBB 依赖。业务接入、保存和读回均在 `third_party/mapping/src/submap.cpp`。上游在该版本没有许可证文件，`package.xml` 的 license 为 TODO；本包的 MIT 声明不覆盖这部分源码，具体来源和修改见 `third_party/btc/NOTICE.md`。

## 编译与运行

在工作空间根目录：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select map --parallel-workers 1 --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 run map map_node
```

运行前需启动 Odin 驱动或回放包含点云和 TF 的 rosbag。节点名为 `freedom_map`。

也可以在工作空间根目录运行 `./start_mapping.sh`，输入 `1`（或直接回车）启动建图，布局为左侧 Odin 纯里程计、右上 `map_node`、右下地图保存控制台。默认聚焦保存窗格，输入 **`s` 并回车**即可保存，响应会显示成功状态和保存目录；可以重复保存，输入 `q` 只退出保存控制台。输入 `2` 则选择已保存的 BTC 地图，以两个窗格启动里程计和重定位。脚本复用根目录的 `start_odometry.sh` 和 `save_map.sh`，默认会话名为 `indoor_slam`、`ROS_DOMAIN_ID=13`（可通过同名环境变量覆盖），模式和地图检查通过后会清理同名旧会话。切换模式前请先保存建图成果。按 `Ctrl+B` 后按 `D` 可退出界面并保留节点运行，使用 `tmux attach -t indoor_slam` 返回。

停止时，在另一个终端或退出 tmux 界面后运行 `./stop_mapping.sh`。脚本先向各窗格发送 `Ctrl+C`，等待 2 秒后关闭该会话；会话已停止时可重复执行。若启动时指定了自定义会话名，停止时也使用相同名称，例如 `SESSION=my_mapping ./stop_mapping.sh`。

| 输入/输出 | 默认值 | 含义 |
| --- | --- | --- |
| 点云输入 | `/odin1/cloud_slam` | `sensor_msgs/msg/PointCloud2`，需要 FLOAT32 的 x/y/z 字段 |
| 位姿输入 | `/tf`、`/tf_static` | 查询点云时刻的 `odom -> lidar` 完整变换 |
| 点云输出 | `/map/static_cloud` | `sensor_msgs/msg/PointCloud2`，当前静态地图，坐标系 `odom` |
| 当前子图 | `/map/submap_cloud` | `sensor_msgs/msg/PointCloud2`，子图局部坐标，每次地图更新后替换 |
| 切换时的上一张子图 | `/map/submap_completed` | `sensor_msgs/msg/PointCloud2`，新子图创建时发布上一张的快照 |
| 全部子图位姿 | `/map/submap_poses` | `geometry_msgs/msg/PoseArray`，建图坐标系下的锚点，数组下标就是子图 ID |
| 保存服务 | `/map/save_map` | `std_srvs/srv/Trigger`，保存当前完整地图、全部子图、位姿和 BTC 描述子 |

RViz 的 Fixed Frame 设为 `odom`，添加 `/map/static_cloud`。输出是多帧累积、允许后续修订的完整静态地图，不能再次逐帧追加累积。输出保留触发本次更新的点云时间戳，仅包含 XYZ。

当前功能包括静态地图、运动子图、BTC 特征提取与文件保存；尚未接入在线 BTC 查询、配准和重定位节点。现有 planner 点云分支继续独立运行，可保留当前动态障碍物观测。

## 根据运动划分子图

`src/map_node.cpp` 在 `freedom_->process()` 后调用 `submaps_->update(static_map, map_from_sensor, stamp)`：

1. 首次在雷达周围裁剪出非空静态点云时，创建 `0` 号子图。
2. 相对当前子图创建时的雷达位置，直线位移达到 **10 m** 才创建下一张子图，不是累计行驶里程。原地旋转继续更新同一张子图，不单独触发新建。
3. 每张子图的锚点固定为创建时的 `T_odom_lidar`，记录 ID、创建时间戳和 `T_odom_submap`。后续机器人移动不会改变这个锚点。
4. 从最新完整静态地图中提取锚点周围 **5 m 球形范围**的点，再用 `T_odom_submap.inverse()` 转成子图局部坐标。每次替换点云，不追加旧点，因此当前子图能反映 FreeDOM 后续的动态残影清理。
5. 切换时先用最新地图提取上一张子图，发布到 `/map/submap_completed`，再发布新子图。当前位置裁剪为空时暂缓新建；已有子图被清空时仍发布空点云，清除旧显示。

当前默认移动间隔为 10 m、裁剪半径为 5 m：相邻锚点恰好相距 10 m 时，球形范围仅相切，没有正体积的空间重叠；间距更大时存在间隙。需要相邻子图重叠时，应配合调整 `submap.radius`，使实际锚点间距小于裁剪直径；实际重叠点数仍取决于场景和可见性。里程计跳变或两帧之间运动过大时不会插值补建子图。子图位姿继承里程计误差，本模块不做回环或位姿优化。

球形裁剪范围与雷达朝向无关，因此已移除旧的 `submap.rotation_threshold_deg` 参数，避免原地每转 30° 生成重复子图。位移仍依据 TF 中的雷达位置；如果原地旋转时里程计位置漂移超过位移阈值，仍会触发新建，需要检查位姿输入。

子图坐标系依次为 `map_submap_0`、`map_submap_1` 等，同时广播固定 TF `odom -> map_submap_<id>`。`map_frame` 自定义时，父坐标系和位姿消息也随之改变。点云消息时间戳是本次地图更新时刻；锚点创建时间保存在 `SubmapPose::stamp_ns` 及对应静态 TF 中。`PoseArray` 的消息时间是最近一次新增锚点的时刻。

RViz 的 Fixed Frame 设为 `odom`，用 PointCloud2 显示 `/map/submap_cloud`，用 PoseArray 显示 `/map/submap_poses`。子图话题使用 reliable、transient local、深度 1，晚订阅者可获取最新消息；它们不是历史点云队列。也可以检查：

```bash
ros2 topic echo /map/submap_poses --once --qos-durability transient_local
```

位姿在节点内存中记录，重启后 ID 从 0 开始；调用保存服务后才写入磁盘。历史子图不缓存点云，可以调用 `SubmapBuilder::extract(latest_static_map, id)` 按固定锚点重新提取。`/map/submap_completed` 只是切换时的快照，后续清理不会重新发布历史快照，最后一张也不会在退出时转成 completed。保存服务会基于同一份最新静态地图重新提取全部子图，包括当前 active，避免保存早期残影或漏掉最后一张。

## 保存地图

通过 `start_mapping.sh` 选择建图模式后，可直接在右下窗格输入 `s` 并回车保存。也可以保持 `map_node` 运行，在另一个终端进入工作区，调用保存服务。脚本默认 ROS domain 为 13；自定义过 domain 时使用相同值：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=13
ros2 service call /map/save_map std_srvs/srv/Trigger "{}"
```

服务返回 `success: true` 时，`message` 是本次保存目录的绝对路径。无有效地图或写入失败时返回 `success: false` 和原因。确认保存成功后，再执行 `./stop_mapping.sh`；停止或重启不会自动保存。

默认根目录是 `maps/`，相对节点启动时的工作目录。使用根目录的 `start_mapping.sh` 启动时，地图保存在本工作区的 `maps/` 下。每次保存新建一个带 UTC 时间和 Unix 纳秒值的独立目录，不覆盖以前的结果：

```text
maps/<本次保存时间>/
├── static_map.pcd       # 最新完整静态地图，map_frame 坐标系（默认 odom）
├── submaps/
│   ├── 000000.pcd      # 对应 map_submap_0 的局部坐标
│   └── 000001.pcd
├── btc/               # btc.enabled=true 时生成
│   ├── 000000.btc     # 同编号子图的二进制特征、三角描述子与平面
│   ├── 000001.btc
│   ├── manifest.csv   # 各子图的特征数量、状态和文件路径
│   └── index.csv      # 三角形边长桶 -> 子图 ID、三角形序号
├── poses.csv
└── metadata.yaml
```

PCD 为 binary XYZ 格式。`poses.csv` 的列依次为 `id,stamp_ns,tx,ty,tz,qx,qy,qz,qw,point_count,pcd_file`，位姿表示 `T_map_submap`，即 `p_map = R * p_submap + t`。时间戳为锚点创建时的 ROS 纳秒时间，PCD 路径相对于本次保存目录。若某张历史子图被清理为空，保留其 CSV 行和原始 ID，`point_count=0`、`pcd_file` 为空，不生成空 PCD。

`metadata.yaml` 记录格式版本、坐标系、最新地图的 ROS 时间戳、保存的系统时间、点数、锚点数量、实际写出的子图数量，以及本次使用的 FreeDOM、子图和 BTC 参数。建图中可以多次调用服务，每次都使用调用时最新完成处理的地图；返回后继续建图。

保存服务与点云处理串行执行，保证地图、子图、位姿和描述子对应同一份快照。BTC 提取和写盘期间点云处理会暂停，大地图保存可能耗时，建议机器人停稳后保存。保存目录中的 `.incomplete` 标记会在全部文件写完后删除；若程序被强制终止而留下该标记，应忽略该目录。正常写入失败会尝试清理本次目录，已有地图不受影响。

自定义保存根目录：

```bash
ros2 run map map_node --ros-args -p save.directory:=/absolute/path/to/maps
```

## BTC 描述子

默认 `btc.enabled=true`。调用 `/map/save_map` 时，`SubmapBuilder::save()` 为每个锚点（包括最后一张 active）重新裁剪当前静态地图，再调用 `extractBtcFeatures()`。流程为官方 BTC 的体素平面检测 → 平面投影 → 二进制占据特征 → 非极大值抑制 → 三角形组合。XYZ 输入转为 XYZI 时将 intensity 置零；BTC 提取使用几何信息。普通点云回调不进行 BTC 提取，避免每帧重复计算整套描述子。

默认特征参数采用官方 `config_indoor.yaml`，尚未针对 Odin 实测调优。`btc.voxel_size=0.5 m` 用于检测平面，不修改保存的 PCD，也不替代 FreeDOM 的 5 cm 静态地图体素。默认允许三角边长 1～30 m；实际能形成的边长受子图范围约束。保持现有的 10 m 锚点间隔、5 m 裁剪半径；提取描述子不会增加子图之间的重叠。

### 文件和状态

- `btc/000000.btc`：本工程自定义的版本化文本格式 `MAP_BTC 1`，不是上游已有文件格式。保存子图 ID、全部二进制特征（局部坐标、占据位数组及其置位数量）、全部 BTC（三边、中心、三个顶点及二进制特征）和用于后续几何验证的平面点/法向量。数值采用足够精度的文本，不直接序列化 C++ 内存。
- `btc/manifest.csv`：`id,status,binary_count,triangle_count,plane_count,descriptor_file`。路径相对于保存目录；空子图也保留 ID 和一个空特征文件。
- `btc/index.csv`：`bucket_x,bucket_y,bucket_z,submap_id,triangle_id`。每行指向 `.btc` 中从 0 开始的一条三角描述子，可按前三列分桶检索。桶值与官方 `AddBtcDescs` 一致：`int(triangle_[axis] + 0.5)`，其中 `triangle_` 已按 `triangle_resolution` 缩放，并非米制原始边长。
- `metadata.yaml` 的 `btc` 段保存来源版本、格式版本、文件路径、总特征数量与全部提取参数。以后提取查询点云时应使用同一组参数。

`status=ready` 表示生成了非零三角描述子；`no_triangles` 表示子图有点但特征不足；`empty` 表示该锚点的点云已被清空。无三角描述子不影响 PCD 保存成功，不能把 `success: true` 单独理解为已经有可检索特征。平面或几何特征很少的场景允许零描述子，应查看清单中的数量；有描述子也不等于已经验证重定位成功。

`map_processing::loadBtcFeatures(path)` 可以读回 `.btc`，得到原始二进制、三角和几何平面数据；将 `features.triangles` 传给 `BtcDescManager::AddBtcDescs()` 可重建内存桶索引。当前未提供在线查询/匹配服务，仍需下一阶段接入候选检索和位姿配准。

### 参数

下面参数均以 `btc.` 为前缀，在节点启动时读取：

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `enabled` | `true` | 保存时提取 BTC；设为 false 只保存点云和位姿 |
| `useful_corner_num` | `500` | 最多保留的二进制关键点数 |
| `voxel_size` / `voxel_init_num` | `0.5` / `10` | 平面检测体素边长及点数阈值 |
| `plane_detection_threshold` | `0.01` | 平面协方差最小特征值阈值 |
| `plane_merge_normal_threshold` / `plane_merge_distance_threshold` | `0.1` / `0.3` | 平面合并的法向和距离阈值 |
| `projection_plane_num` | `2` | 投影平面数 |
| `projection_resolution` / `projection_height_increment` | `0.2` / `0.1` | 投影像素大小和高度分箱间隔，米 |
| `projection_distance_min` / `projection_distance_max` | `-1.0` / `4.0` | 到投影平面的有符号距离范围，米 |
| `summary_min_threshold` | `6` | 特征最少占据的高度箱数量 |
| `line_filter_enabled` | `false` | 是否启用直线特征过滤 |
| `descriptor_near_num` | `15` | 构造三角形时的邻居数上限 |
| `descriptor_min_length` / `descriptor_max_length` | `1.0` / `30.0` | 三角形边长范围，米 |
| `non_max_suppression_radius` | `1.0` | 关键点抑制邻域半径，米 |
| `triangle_resolution` | `0.2` | 三角边长归一化尺度，米 |

参数校验限制高度箱数量在 1～255，邻居数 3～100、关键点上限 3～5000，避免无效计算和无界配置。BTC 文件与 PCD 一起处于 `.incomplete` 保存事务内；提取/写入失败时整批保存失败，不留下宣称成功的半套数据。

## 回调中的坐标与时间处理

1. 按输入时间戳查询 `T_odom_lidar`，其中已包含 `odom -> imu -> lidar` 外参链。
2. `cloud_slam` 已在 `odom` 中，先用逆变换还原为雷达系点云；如果输入本来就在 `lidar` 系中则不变换。
3. 调用 `freedom_->process(sensor_cloud, map_from_sensor)`。入口先在雷达坐标系中丢弃非有限点及三维距离 `sqrt(x²+y²+z²) <= sensor.min_range` 的近点（默认 0.2 m，包含边界），再交给 FreeDOM 按量程处理，在共同坐标系中维护多帧自由空间与静态地图。此过滤针对每帧输入，不会随着机器人移动将历史地图中靠近当前雷达的静态点删除。
4. 将返回的静态地图转换为 ROS 点云并发布。
5. 将同一份静态地图和雷达位姿传给子图处理器，发布当前子图、切换快照与锚点位姿。

TF 使用独立线程接收，处理器由单线程回调依次调用。缺少点云时刻 TF 的帧会跳过，不会使用最新位姿代替。重复、倒序、零时间戳和格式错误的点云会跳过。重新从头回放 rosbag 时需要重启本节点，以清除历史状态。

`/map/submap_cloud` 在每次成功处理点云后发布，频率受输入频率、TF 等待、处理时间和消息传输影响，没有固定 6 Hz 限频。若输入约 10 Hz 而输出只有 6～8 Hz，先查看建图日志是否出现 `Skip cloud: timestamped TF unavailable`。点云时间略晚于最新里程计 TF 时，需要等下一帧 TF 才能插值；100 ms 上限可能被一个 10 Hz 更新周期和调度抖动耗尽。因此默认 `tf_timeout=0.2`，保留原始时间戳查询。这个值是等待上限，不是每帧固定休眠；TF 已就绪时立即继续，确实缺失超过上限的帧仍跳过。该参数在启动时读取，已运行实例需下次启动才使用新默认值。

驱动的 `use_host_ros_time: 1` 使用接收时间，接近的时间戳不保证观测真正同步。本包不改变驱动配置；真实效果验证前应检查采样时间关系与 TF。对 `cloud_slam` 还应确认设备输出满足逐帧测量假设。使用 `cloud_raw` 时，本节点不会根据逐点 `offset_time` 做运动补偿，应在上游完成必要的去畸变。

## 启动参数

参数在启动时读取。示例：

```bash
ros2 run map map_node --ros-args \
  -p input_topic:=/odin1/cloud_slam \
  -p output_topic:=/map/static_cloud \
  -p map_frame:=odom \
  -p sensor_frame:=lidar \
  -p sensor.max_range:=8.0 \
  -p freedom.sub_voxel_size:=0.05 \
  -p freedom.counts_to_free:=3 \
  -p submap.translation_threshold:=10.0 \
  -p submap.radius:=5.0
```

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `input_topic` | `/odin1/cloud_slam` | 点云输入，不能与输出同名 |
| `output_topic` | `/map/static_cloud` | 静态地图输出 |
| `map_frame` | `odom` | 连续、不跳变的建图参考系 |
| `sensor_frame` | `lidar` | 雷达坐标系 |
| `tf_timeout` | `0.2` | 单次 TF 查询最多等待秒数，范围 0～1；给 10 Hz 里程计的下一帧 TF 留出到达余量 |
| `sensor.min_range` | `0.2` | 相对当前雷达的三维距离，输入点距离 ≤ 此值时丢弃，米 |
| `sensor.max_range` | `8.0` | 最大距离，米 |
| `sensor.min_z` | `-4.0` | 上游算法使用的相对雷达高度下限，米 |
| `sensor.max_z` | `4.0` | 上游算法使用的相对雷达高度上限，米 |
| `freedom.sub_voxel_size` | `0.05` | 静态地图子体素边长，米 |
| `freedom.counts_to_free` | `3` | 确认自由空间的观测次数参数 |
| `freedom.counts_to_revert` | `20` | 持续占据后撤销自由空间判断的次数参数 |
| `freedom.num_threads` | `2` | FreeDOM 核心线程数 |
| `submap.translation_threshold` | `10.0` | 相对当前锚点的直线位移阈值，米，正有限值 |
| `submap.radius` | `5.0` | 以固定锚点为中心的球形裁剪半径，米，正有限值 |
| `save.directory` | `maps` | 保存根目录，允许绝对路径；相对路径以节点启动工作目录为基准 |

默认采用上游室内配置的多分辨率层级：`voxel_depth=2`、`block_depth=5`。静态地图子体素为 5 cm 时，自由空间体素为 20 cm，块为 1.6 m。这些设置与普通 VoxelGrid 降采样是不同参数。

当前封装限制最大量程不超过 30 m、高度区间跨度不超过 30 m、子体素边长在 0.02～0.5 m、线程数在 1～8，以约束密集工作网格的分配规模。最小量程非负、上下限有序；观测次数为正，撤销次数不小于确认次数。需要更大范围时应先评估内存，而不是直接放宽参数。

与上游室内楼梯配置一致，当前关闭射线增强：Odin 的视场与无回波模型尚未标定，先使用真实观测射线。扫描动态去除、自由空间估计和历史地图清理仍使用官方算法。默认保留整张累计地图，运行时间和覆盖范围增大时内存与整图发布开销会增长。子图裁剪每帧遍历最新静态地图，当前未建立额外空间索引。

## 验证

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --packages-select map --parallel-workers 1 --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select map --event-handlers console_direct+
colcon test-result --test-result-base build/map --verbose
```

C++ 测试覆盖多帧自由空间、动态残影清理及子图位移阈值、原地旋转与小幅位置抖动、局部坐标、地图修订，并读回保存的 PCD/CSV 检查历史重裁剪、当前子图、重复保存及失败处理。ROS 集成测试用合成点云和 TF 检查消息坐标、时间、输入拒绝、地图持续累积、子图发布和保存服务。真实 Odin 数据下的动态去除效果、子图尺度与实时性需要 rosbag 或设备验证。

`test_btc` 使用合成房间/立柱验证非零 BTC 提取、空/稀疏点云、非法参数、描述子保存读回、索引对应关系、最新地图重提取、禁用 BTC 及提取失败时的目录清理。可选真实子图检查不依赖固定私有文件：

```bash
BTC_SMOKE_PCD=/absolute/path/to/submaps/000000.pcd \
  build/map/test_btc --gtest_filter=BtcExtraction.OptionalLocalSubmapFixtureProducesFiniteFeatures
```

该检查使用默认室内参数，要求指定文件能读且生成非零三角描述子；未设置环境变量时跳过这一项。通过仅说明该子图能提取特征，不代表重定位精度已验证。
