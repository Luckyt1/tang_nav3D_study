#ifndef MAP_FREEDOM_HPP_
#define MAP_FREEDOM_HPP_

#include <memory>

#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace map_processing {

using Cloud = pcl::PointCloud<pcl::PointXYZ>;

struct FreedomConfig {
  // 为了限制 FreeDOM 内部 dense raycast grid 内存，配置会在构造时校验：
  // sensor_max_range <= 30m，z span <= 30m，sub_voxel_size 在 [0.02, 0.5]。
  double sensor_min_range{0.2};  // 丢弃距当前雷达原点 <= 此距离的输入点，单位米。
  double sensor_max_range{8.0};
  double sensor_min_z{-4.0};
  double sensor_max_z{4.0};
  double sub_voxel_size{0.05};
  int counts_to_free{3};
  int counts_to_revert{20};
  int num_threads{2};
};

class FreedomProcessor {
 public:
  // 建图保留完整地图；重定位可启用以雷达为中心的有界局部历史。
  explicit FreedomProcessor(const FreedomConfig& config = {}, bool local_map = false);
  ~FreedomProcessor();

  FreedomProcessor(const FreedomProcessor&) = delete;
  FreedomProcessor& operator=(const FreedomProcessor&) = delete;

  Cloud process(const Cloud& sensor_cloud, const Eigen::Isometry3d& map_from_sensor);

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace map_processing

#endif  // MAP_FREEDOM_HPP_
