# BTC 粗定位与 ICP 精定位

本包使用累积的实时点云完成 BTC 粗定位，再以粗位姿为初值做 ICP 精配准。通过质量检查后更新 `map -> odom` 修正；在修正有效期内，独立地跟随最新里程计 TF 持续发布该修正与地图中的雷达位姿。匹配频率和位姿输出频率分开。

## 文件结构

- `src/relocalization_node.cpp`：ROS 参数、点云订阅、按点云时间戳查询 TF、发布结果和 `main()`；保留两个执行线程与原回调组划分。
- `third_party/localization/src/relocalization.cpp`：快照加载、查询点云累积、BTC 检索和三角形几何验证。
- `third_party/localization/src/fine_registration.cpp`：独立的 PCL ICP 精配准和质量检查，由节点在每次粗定位查询后调用；复用同一份累积点云。
- `third_party/localization/src/pose_tracking.cpp`：保存通过检查的地图修正，结合最新里程计传播位姿，检查定位与里程计有效期。
- `third_party/localization/include/relocalization/`：上述算法的 `.hpp` 接口，仍使用 `relocalization/relocalization.hpp`、`relocalization/fine_registration.hpp` 和 `relocalization/pose_tracking.hpp` 的包含路径。
- `msg/`：粗定位候选与精定位结果、质量诊断。

直接复用 `map` 导出的 FreeDOM、子图和 BTC 库；两端使用同一份描述子提取实现。

## 编译与启动

已编译后，在工作空间根目录运行 `./start_mapping.sh`，输入 `2` 即可启动
**Odin 里程计 + BTC 重定位**；输入 `1` 启动建图。脚本从 `maps/` 中选择包含
BTC 描述子且没有 `.incomplete` 标记的地图：只有一份时自动使用，多份时输入编号，
直接回车默认选择目录名排序最后的一份（本工程目录按保存时间命名）。
没有可用地图时会提示退出，不关闭原有会话。

其他位置的地图可用 `MAP_DIR='/完整地图目录' ./start_mapping.sh` 指定，再输入 `2`。
两种模式都用 `./stop_mapping.sh` 停止。切换模式会重建同名 tmux 会话，建图成果需要先通过
`/map/save_map` 保存。

手动启动方式如下。

在工作空间根目录执行：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-up-to relocalization --parallel-workers 1
source install/setup.bash
export ROS_DOMAIN_ID=13
ros2 run relocalization relocalization_node --ros-args \
  -p map.directory:="/绝对路径/maps/某次保存目录"
```

运行时需要里程计提供 `/odin1/cloud_slam` 和同一时间戳的 `odom -> lidar` TF；也支持点云先经过其他坐标系的 TF。节点会把点云还原到雷达坐标，再交给处理器。

`map.directory` 指向**一次保存的快照目录**，不是上层 `maps/`。目录需要包含：

```text
metadata.yaml
poses.csv
static_map.pcd
submaps/*.pcd
btc/manifest.csv
btc/*.btc
```

建图时通过 `ros2 service call /map/save_map std_srvs/srv/Trigger '{}'` 保存，响应中的路径可直接使用。旧的、不含 BTC 的快照会被明确拒绝，需要使用当前 `map` 节点、开启 `btc.enabled` 后重新保存。加载过程不会修改地图文件。

新快照记录 `btc.extraction_revision: 2`。已有 BTC 快照若缺少该字段（或为 `1`），加载时先校验原始描述子和清单，再使用保存的子图 PCD 与提取参数在内存中重新生成描述子；无需重新建图，也不会覆盖原文件。无法识别的提取版本会被拒绝。

## 处理流程

1. 启动时读取子图锚点、BTC 文件及保存的预处理参数，从三角形描述子重建内存索引；读取并发布完整静态地图，子图 PCD 按需读取。
2. 默认每新增 10 个有效点云帧查询一次。使用建图时的近点范围、FreeDOM 体素尺寸和动态滤除参数处理点云，保留附近的历史观测，让转动扫描的不同视角能够累计。
3. 以最后一帧雷达位置为中心，按保存的 `submap.radius` 裁剪；将点云转换到最后一帧雷达局部坐标，提取 BTC。
4. 三角形边长检索、二进制描述子相似度筛选、子图投票，再用刚体变换验证顶点一致性。按支持数和误差排序，最多输出 5 个候选。
   若没有候选，且当前雷达坐标轴与里程计坐标轴不同，则仅用里程计旋转将查询点云转到另一组参考轴，再提取和检索一次，缓解雷达倾斜造成的体素与投影差异。两次使用相同的匹配阈值，输出位姿仍表示最后一帧雷达在地图中的姿态；不会把里程计位置当作地图位置。两次均失败时，诊断保留内点数、投票数、三角形数依次较优的那次尝试。
5. 查询后只重置本轮帧计数，保留 FreeDOM 局部历史。局部地图由上游 FreeDOM 按块随雷达移动裁剪，范围取保存的传感器最大距离和高度范围，查询点云仍按子图半径裁剪。本轮凑够 10 帧所用的时间超过 3 秒时，仅重置本轮计数。只有连续 30 秒没有收到带有效 TF 的输入时，才在下一帧清除历史；短暂 TF 延迟不会丢失之前扫描的结构。
6. `src/relocalization_node.cpp` 把本轮查询交给 `FineRegistration`。查询点云与候选子图降采样到 10 cm；默认最多精化前 3 个粗候选，每个候选最多迭代 100 次，满足收敛条件时提前结束。目标子图降采样结果缓存在内存中。
7. ICP 使用 `submap_from_query` 初值。达到迭代上限而未收敛会被拒绝；随后用最近邻距离独立计算内点数、重叠率和 RMSE，并检查相对粗位姿的修正幅度。从通过检查的候选中优先选择重叠率最高、其次 RMSE 最低的一个。
8. 点云匹配和跟踪定时器使用独立的互斥回调组，由两个执行线程处理，耗时的 BTC/ICP 不占用跟踪回调线程。定时器默认以 50 Hz 读取最新 `odom -> lidar` TF；仅当里程计时间戳前进且修正仍有效时，发布 `map -> odom` 和 `tracked_pose`。不为重复的旧里程计数据换上新时间戳。

动态 `/tf` 使用 `best_effort` 订阅，保留 100 条接收队列和 TF 历史，避免旧消息丢失后的可靠重传等待阻塞后续位姿。`/tf_static` 仍使用 `reliable + transient_local`，确保后启动的节点能收到静态外参。丢帧时仍严格按点云时间戳查询 TF；等待超时会跳过该帧，不以当前位姿替代历史位姿。里程计真正停更时仍触发超时保护。

因此查询结果频率低于输入点云频率；10 Hz 输入、每 10 帧查询时，理想查询频率约 1 Hz，实际还受特征提取与检索耗时影响。`accumulated_frames` 表示本轮新增有效帧数，点云可以包含之前的局部历史。特征不足或没有匹配时也会发布空候选结果，不会沿用旧候选。动态滤除需要多次观测，不保证消除所有行人点。

## 输出话题与坐标

| 话题 | 类型 | 内容 |
| --- | --- | --- |
| `/relocalization/global_map` | `sensor_msgs/msg/PointCloud2` | 保存的完整地图，位于 `map_frame`（默认 `map`），启动时发布并保留供晚加入的订阅者读取 |
| `/relocalization/candidates` | `relocalization/msg/CandidateArray` | 子图 ID、票数、内点数、二进制相似度、顶点 RMSE、粗位姿 |
| `/relocalization/candidate_poses` | `geometry_msgs/msg/PoseArray` | 候选雷达粗位姿，供 RViz 查看；无匹配时为空 |
| `/relocalization/query_cloud` | `sensor_msgs/msg/PointCloud2` | 当前查询点云，发布在本次运行的 `odom` 坐标系 |
| `/relocalization/fine_result` | `relocalization/msg/FineResult` | 每轮精定位是否成功、失败原因、重叠率、内点 RMSE、修正幅度和精位姿 |
| `/relocalization/pose` | `geometry_msgs/msg/PoseStamped` | 仅成功时发布最后一帧雷达在 `map` 中的精位姿 |
| `/relocalization/tracked_pose` | `geometry_msgs/msg/PoseStamped` | 有效期内由地图修正与最新里程计合成的高频位姿；时间戳来自里程计 |
| `/relocalization/tracking_status` | `relocalization/msg/TrackingStatus` | 跟踪是否有效、失效原因、最近定位时间和年龄；停止输出位姿时仍更新状态 |
| `/relocalization/aligned_cloud` | `sensor_msgs/msg/PointCloud2` | 成功时发布变换到 `map` 的完整查询点云；失败时发布空云清除上一轮显示 |
| `/tf` | `tf2_msgs/msg/TFMessage` | 跟踪有效期间，随新里程计时间戳发布 `map -> odom`；`fine.publish_tf:=false` 仅关闭 TF，不关闭 tracked pose |

`CandidateArray.status` 为 `candidates`、`no_descriptors` 或 `no_match`。`votes`、`inliers` 按不同查询三角形计数，RMSE 单位为米，不是定位置信概率。

失败时同时查看 `failure_reason`：

| 原因 | 含义 |
| --- | --- |
| `no_descriptors` | 没有提取出三角形 |
| `insufficient_triangles` | 查询三角形数量低于投票/内点最低要求 |
| `insufficient_votes` | 边长与二进制相似度筛选后，没有子图达到投票门槛 |
| `geometry_rejected` | 有子图达到投票门槛，但刚体几何验证失败 |

`best_votes / required_votes` 和 `best_inliers / required_inliers` 给出实际支持数及门槛。
两种最大支持数可能来自不同子图；未进入几何验证时 `best_inliers` 为 0。有候选时 `failure_reason` 为空。
`best_length_votes` 是仅经过边长筛选的最大票数；若它较高但 `best_votes` 很低，说明主要被二进制相似度筛掉。

`query` 表示查询窗口**最后一帧的雷达坐标系**；消息时间戳也是该帧时间戳。

```text
submap_from_query = T_submap_query
map_from_query   = T_map_submap × T_submap_query
odom_from_query  = 当前里程计中的最后一帧雷达位姿
map_from_odom    = 精配准后的 map_from_query × inverse(odom_from_query)
tracked_pose     = 最近有效的 map_from_odom × 当前 odom_from_sensor
```

这里的 `map` 是保存地图的坐标系，默认输出名称为 `map`。即使建图快照里叫 `odom`，也不能把它当成本次重定位启动后的实时 `odom`：两次运行原点可能不同。改变 `map_frame` 参数只改变名称，不变换地图数值。首次精配准成功之前，不发布地图到里程计的 TF。

### 判断精定位结果

日志出现 `ICP refine: success=true`，或 `/relocalization/fine_result` 的 `success: true`，表示本轮精配准通过检查。`candidates>0` 仅表示粗定位候选。可运行：

```bash
ros2 topic echo /relocalization/fine_result --once
```

`overlap` 的分母是全部降采样查询点数；`inliers` 是距离目标最近点不超过 `fine.inlier_distance` 的点数。`rmse` 为这些内点的距离均方根，单位米，无内点时为 `inf`。它是点云配准残差，不是绝对定位误差或置信概率。

常见失败原因包括 `no_candidates`、`insufficient_query_points`、`insufficient_target_points`、`icp_not_converged`、`insufficient_inliers`、`low_overlap`、`high_rmse`、`excessive_translation_correction`、`excessive_rotation_correction`。`disabled` 表示关闭了精配准；`refinement_error` 的具体异常见同轮终端日志。失败消息中的位姿不可使用。

`fine_result` 和 `pose` 仍表示离散的精配准结果。`tracked_pose` 则使用最近有效修正结合最新里程计传播，频率上限为 `tracking.publish_rate` 与里程计 TF 更新频率两者中的较低值。匹配失败不会立即丢掉仍有效的修正，也不会给它续期；这段时间的传播不是新的一次 ICP 匹配。RViz 的绿色箭头默认显示 `tracked_pose`。

默认修正有效期为 5 秒，从触发该轮查询的点云开始处理时计时，并同时检查最新里程计时间与查询时间的差值，取较大的年龄。计算太慢、返回时已经过期的结果不会激活跟踪。连续 1 秒没有新的里程计时间戳也会停止发布位姿和 TF；反复收到相同 TF 不会延长新鲜度。时钟暂停时仍用单调时钟检查过期。观察到里程计时间倒退会清空修正、重置点云累积时间线，并拒绝此前尚未算完的结果；上游里程计原点必须保持连续；若时间戳继续递增却重置了原点，当前逻辑无法自动识别。

使用 `/relocalization/tracking_status` 的 `valid` 判断当前传播是否有效。状态包含 `waiting_for_fix`、`tracking`、`fix_expired`、`odometry_stale`、`odometry_reset`、`waiting_for_odometry`、`odometry_unavailable`。`fix_stamp` 保留最近一次采用的查询时间，不会随高频输出刷新。停止广播不会撤销 tf2 已缓存的历史变换，因此不能只凭“存在 TF”判断当前有效性。重复结构或单平面环境仍可能产生歧义，几何质量门槛不保证位置唯一。

```bash
ros2 topic hz /relocalization/tracked_pose
ros2 topic echo /relocalization/tracking_status --once
```

### 查看完整地图

完整地图从 `metadata.yaml` 的 `static_map_file`（本工程保存为 `static_map.pcd`）读取，
**不依赖实时点云、TF 或 BTC 匹配成功**。它包含保存时的完整地图，不只是各子图覆盖的区域。
启动日志会显示发布话题、坐标系和点数；文件缺失、损坏或点数与元数据不一致时会明确报错。

RViz 设置：

- **Fixed Frame**：`map`（若设置了 `map_frame`，使用对应名称）。
- 添加 **PointCloud2**，话题选择 `/relocalization/global_map`。
- **Durability Policy**：`Transient Local`，**Reliability Policy**：`Reliable`。
- **Color Transformer**：`FlatColor` 或 `AxisColor`；地图包含 XYZ，没有 intensity。

全局地图只发布一次并保留最后一条消息，因此不需要用 `topic hz` 判断是否正常。可在任意时刻检查点数：

```bash
ros2 topic echo /relocalization/global_map sensor_msgs/msg/PointCloud2 \
  --qos-durability transient_local --qos-reliability reliable --once --field width
```

`/map/static_cloud` 是建图节点的实时输出；选择重定位模式时，全局地图使用上面的 `/relocalization/global_map`。
同一 `map` 坐标系下可以同时显示完整地图和 `/relocalization/candidate_poses`，检查粗位姿落在哪个区域。

## 常用参数

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `map.directory` | 必填 | 保存快照路径 |
| `input_topic` | `/odin1/cloud_slam` | 里程计点云 |
| `global_map_topic` | `/relocalization/global_map` | 已保存完整地图的发布话题 |
| `odom_frame` / `sensor_frame` / `map_frame` | `odom` / `lidar` / `map` | 当前里程计、雷达、保存地图输出坐标名称 |
| `tf_timeout` | `0.2` | 等待点云时间戳 TF 的最长秒数 |
| `query.frames` | `10` | 每次查询之间新增的有效帧数，局部历史跨查询保留 |
| `query.max_duration` | `3.0` | 本轮新增帧的最大时间跨度，超过时只重置帧计数，秒 |
| `query.history_timeout` | `30.0` | 两次有效输入之间的最大间隔，超过时清除局部历史，秒 |
| `search.top_k` | `5` | 最大候选数 |
| `search.min_votes` / `search.min_inliers` | `5` / `4` | 最少检索票数、几何验证内点数 |
| `search.length_tolerance` | `0.02` | 三角形边长向量的相对误差阈值 |
| `search.binary_similarity` | `0.7` | 顶点二进制相似度均值阈值 |
| `search.max_vertex_error` | `0.5` | 几何验证顶点误差阈值，米 |
| `search.max_hypotheses` | `64` | 最多验证的候选子图数及每张子图的变换假设数 |
| `search.max_matches_per_candidate` | `2000` | 每张候选子图保留的匹配对上限 |
| `fine.enabled` / `fine.publish_tf` | `true` / `true` | 启用精配准 / 跟踪有效时广播 `map -> odom` |
| `fine.voxel_size` | `0.1` | ICP 输入降采样体素边长，米 |
| `fine.max_correspondence_distance` | `0.75` | ICP 对应点的最大距离，米 |
| `fine.max_iterations` / `fine.max_candidates` | `100` / `3` | 每候选迭代上限（收敛后提前结束） / 最多精化候选数 |
| `fine.min_points` | `50` | 降采样源、目标和最终内点的最少点数 |
| `fine.inlier_distance` | `0.25` | 最终质量评估的内点距离门槛，米 |
| `fine.min_overlap` / `fine.max_rmse` | `0.6` / `0.15` | 最少内点比例 / 内点 RMSE 上限（米） |
| `fine.max_translation_correction` | `1.0` | 相对粗位姿允许修正的最大平移距离，米 |
| `fine.max_rotation_correction_deg` | `30.0` | 相对粗位姿允许修正的最大旋转角，度 |
| `tracking.publish_rate` | `50.0` | 跟踪 TF/位姿最大发布频率，Hz；里程计不更新时不重复发布 |
| `tracking.fix_timeout` | `5.0` | 最近精定位结果的有效期，秒 |
| `tracking.odom_timeout` | `1.0` | 里程计时间戳允许停止前进的最长时间，秒 |

查询半径、FreeDOM 和 BTC 提取参数从 `metadata.yaml` 读取，避免单独设置导致建图与查询不一致。

## 验证

```bash
colcon test --packages-select relocalization --event-handlers console_direct+
colcon test-result --test-result-base build/relocalization/test_results --verbose
```

测试覆盖快照加载、已知刚体变换下的粗/精位姿与坐标组合、查询窗口、异常输入、质量拒绝、跟踪传播、时间戳去重、过期、迟到结果、时钟倒退隔离及 ROS 点云/TF/结果发布。实际定位误差仍需带真值的现场数据验证。

BTC 第三方来源和许可说明见 `../map/third_party/btc/NOTICE.md`；本包没有复制第二份第三方实现。
