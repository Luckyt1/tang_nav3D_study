#ifndef MAP_SUBMAP_HPP_
#define MAP_SUBMAP_HPP_

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <vector>

#include <btc/btc.h>

#include "map/freedom.hpp"

namespace map_processing {

// Descriptor file syntax is versioned separately from the extraction algorithm.
inline constexpr int kBtcExtractionRevision = 2;

struct SubmapConfig {
  double translation_threshold{10.0};
  double radius{5.0};
};

struct SubmapPose {
  std::size_t id;
  std::int64_t stamp_ns;
  // 将子图局部坐标变换到建图坐标系；固定为创建时的雷达位姿。
  Eigen::Isometry3d map_from_submap;
};

struct Submap {
  SubmapPose pose;
  Cloud cloud;  // 子图局部坐标，每次从最新静态地图重新裁剪。
};

struct SubmapUpdate {
  bool created{false};
  std::optional<Submap> active;
  // 切换子图时，使用本次最新地图裁剪上一张；不是不可变的最终地图。
  std::optional<Submap> completed;
};

// 默认值来自官方 config_indoor.yaml。只配置特征提取，不启用回环/重定位。
struct BtcConfig {
  bool enabled{true};
  int useful_corner_num{500};
  double plane_detection_threshold{0.01};
  double plane_merge_normal_threshold{0.1};
  double plane_merge_distance_threshold{0.3};
  double voxel_size{0.5};
  int voxel_init_num{10};
  int projection_plane_num{2};
  double projection_resolution{0.2};
  double projection_height_increment{0.1};
  double projection_distance_min{-1.0};
  double projection_distance_max{4.0};
  int summary_min_threshold{6};
  bool line_filter_enabled{false};
  int descriptor_near_num{15};
  double descriptor_min_length{1.0};
  double descriptor_max_length{30.0};
  double non_max_suppression_radius{1.0};
  double triangle_resolution{0.2};
};

struct BtcFeatures {
  std::uint64_t submap_id{0};
  std::vector<BinaryDescriptor> binary;
  std::vector<BTC> triangles;
  pcl::PointCloud<pcl::PointXYZINormal> planes;
};

void validateBtcConfig(const BtcConfig& config);
BtcFeatures extractBtcFeatures(const Cloud& local_cloud, std::uint64_t submap_id,
                              const BtcConfig& config = {});
// 读回本工程版本化 .btc 文件，可将 triangles 交给 AddBtcDescs 重建内存索引。
BtcFeatures loadBtcFeatures(const std::filesystem::path& file);

struct MapSaveOptions {
  std::filesystem::path directory{"maps"};
  std::string map_frame{"odom"};
  std::string sensor_frame{"lidar"};
  FreedomConfig freedom;
  BtcConfig btc;
};

class SubmapBuilder {
 public:
  explicit SubmapBuilder(const SubmapConfig& config = {});

  SubmapUpdate update(const Cloud& static_map, const Eigen::Isometry3d& map_from_sensor,
                      std::int64_t stamp_ns);
  const std::vector<SubmapPose>& poses() const { return poses_; }

  // 只存锚点，历史子图也可由最新整图重新提取，应用 FreeDOM 的后续清理结果。
  Cloud extract(const Cloud& static_map, std::size_t id) const;

  // 同一份最新整图导出全部锚点（包括 active），返回本次保存的绝对目录。
  // 每次新建目录；失败时抛异常并清理本次写入，不覆盖以前的地图。
  std::filesystem::path save(const Cloud& latest_static_map,
                             const MapSaveOptions& options = {}) const;

 private:
  Cloud crop(const Cloud& static_map, const Eigen::Isometry3d& map_from_submap) const;

  SubmapConfig config_;
  std::vector<SubmapPose> poses_;
  std::int64_t last_stamp_ns_{0};
};

}  // namespace map_processing

#endif  // MAP_SUBMAP_HPP_
