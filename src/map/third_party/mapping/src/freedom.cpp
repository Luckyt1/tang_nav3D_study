#include "map/freedom.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>

#include <pcl/common/point_tests.h>

#include "freedom/freedom.h"

namespace map_processing {
namespace {

bool finite(double value) { return std::isfinite(value); }

void require(bool condition, const std::string& message) {
  if (!condition) {
    throw std::invalid_argument(message);
  }
}

void validateConfig(const FreedomConfig& config) {
  require(finite(config.sensor_min_range) && config.sensor_min_range >= 0.0,
          "FreedomConfig.sensor_min_range must be finite and non-negative");
  require(finite(config.sensor_max_range) &&
              config.sensor_max_range > config.sensor_min_range,
          "FreedomConfig.sensor_max_range must be finite and greater than min_range");
  require(config.sensor_max_range <= 30.0,
          "FreedomConfig.sensor_max_range is capped at 30 m to bound dense raycast memory");
  require(finite(config.sensor_min_z) && finite(config.sensor_max_z) &&
              config.sensor_min_z < config.sensor_max_z,
          "FreedomConfig sensor z limits must be finite and ordered");
  require((config.sensor_max_z - config.sensor_min_z) <= 30.0,
          "FreedomConfig z span is capped at 30 m to bound dense raycast memory");
  require(finite(config.sub_voxel_size) && config.sub_voxel_size >= 0.02 &&
              config.sub_voxel_size <= 0.5,
          "FreedomConfig.sub_voxel_size must be finite and in [0.02, 0.5]");
  require(config.counts_to_free > 0, "FreedomConfig.counts_to_free must be positive");
  require(config.counts_to_revert >= config.counts_to_free,
          "FreedomConfig.counts_to_revert must be >= counts_to_free");
  require(config.num_threads > 0 && config.num_threads <= 8,
          "FreedomConfig.num_threads must be in [1, 8]");
}

void validateTransform(const Eigen::Isometry3d& map_from_sensor) {
  const Eigen::Matrix4d matrix = map_from_sensor.matrix();
  require(matrix.allFinite(), "map_from_sensor must contain only finite values");

  const Eigen::Matrix3d rotation = map_from_sensor.linear();
  const double orthonormal_error =
      (rotation.transpose() * rotation - Eigen::Matrix3d::Identity()).norm();
  require(orthonormal_error < 1e-6, "map_from_sensor rotation must be orthonormal");
  require(std::abs(rotation.determinant() - 1.0) < 1e-6,
          "map_from_sensor rotation determinant must be 1");
}

Cloud filterInputCloud(const Cloud& cloud, double min_range) {
  Cloud filtered;
  filtered.reserve(cloud.size());
  // PCL 的 XYZ 是 float；阈值使用相同精度，保证 0.2F 这样的边界点也被排除。
  const float min_distance = static_cast<float>(min_range);
  for (const auto& point : cloud.points) {
    if (pcl::isFinite(point) && point.getVector3fMap().norm() > min_distance) {
      filtered.push_back(point);
    }
  }
  filtered.width = static_cast<std::uint32_t>(filtered.size());
  filtered.height = 1;
  filtered.is_dense = true;
  return filtered;
}

freedom::FreeDOM::Config toFreedomConfig(const FreedomConfig& config, bool local_map) {
  const unsigned int threads = static_cast<unsigned int>(config.num_threads);

  freedom::FreeDOM::Config freedom_config{};
  freedom_config.sensor_min_range = config.sensor_min_range;
  freedom_config.sensor_max_range = config.sensor_max_range;
  freedom_config.sensor_min_z = config.sensor_min_z;
  freedom_config.sensor_max_z = config.sensor_max_z;
  freedom_config.sub_voxel_size = config.sub_voxel_size;
  freedom_config.voxel_depth = 2;
  freedom_config.block_depth = 5;

  // 使用上游空间裁剪限制重定位历史，建图仍保留完整地图。
  freedom_config.enable_local_map = local_map;
  freedom_config.local_map_range = config.sensor_max_range;
  freedom_config.local_map_min_z = config.sensor_min_z;
  freedom_config.local_map_max_z = config.sensor_max_z;

  // Odin 的真实视场还没标定，先按 indoor_stairs 的保守设置关闭增强射线。
  freedom_config.raycast_max_range = config.sensor_max_range;
  freedom_config.raycast_min_z = config.sensor_min_z;
  freedom_config.raycast_max_z = config.sensor_max_z;
  freedom_config.counts_to_free = static_cast<unsigned int>(config.counts_to_free);
  freedom_config.counts_to_revert = static_cast<unsigned int>(config.counts_to_revert);
  freedom_config.conservative_connectivity = 26;
  freedom_config.aggressive_connectivity = 124;
  freedom_config.enable_raycast_enhancement = false;

  freedom_config.lidar_horizon_fov = 0.0;
  freedom_config.lidar_vertical_fov_upper = 0.0;
  freedom_config.lidar_vertical_fov_lower = 0.0;
  freedom_config.depth_image_vertical_lines = 0;
  freedom_config.depth_image_min_range = config.sensor_min_range;
  freedom_config.max_raycast_enhancement_range = config.sensor_max_range;
  freedom_config.raycast_enhancement_depth_margin = 0.0;
  freedom_config.inpaint_size = 0;
  freedom_config.erosion_size = 0;
  freedom_config.min_raycast_enhancement_area = 0.0;
  freedom_config.depth_image_top_margin = 0.0;
  freedom_config.learn_fov = false;
  freedom_config.enable_fov_mask = false;
  freedom_config.fov_mask_path.clear();
  freedom_config.num_threads = threads;
  return freedom_config;
}

Cloud extractStaticCloud(const freedom::MRMap& map) {
  Cloud output;

  const unsigned int block_idx_size = map.getVoxel2blockMultiples();
  const unsigned int voxel_num = map.getVoxel2blockMultiplesCubed();
  const unsigned int sub_voxel_num = map.getSubvoxel2voxelMultiplesCubed();
  const double block_size = map.getBlockSize();
  const double voxel_size = map.getVoxelSize();

  for (const auto& block_pair : map.get_static_blocks()) {
    const freedom::PointBias block_bias = block_size * block_pair.first.cast<double>();
    const freedom::StaticBlock& static_block = block_pair.second;

    freedom::Index local_voxel_idx(0, 0, 0);
    for (unsigned int voxel = 0; voxel < voxel_num; ++voxel) {
      if (!static_block.is_static_voxel_allocated(voxel)) {
        freedom::incrementIdx(local_voxel_idx, block_idx_size);
        continue;
      }

      const freedom::StaticVoxel& static_voxel = static_block.getStaticVoxel(voxel);
      const freedom::PointBias voxel_bias = voxel_size * local_voxel_idx.cast<double>();

      for (unsigned int sub_voxel = 0; sub_voxel < sub_voxel_num; ++sub_voxel) {
        const bool occupied = static_voxel.scan_in_subvoxel[sub_voxel] !=
                              freedom::StaticVoxel::NOT_A_SCAN;
        const bool static_point =
            occupied && static_voxel.dynamic_level[sub_voxel] <= freedom::DynamicLevel::STATIC;
        if (static_point) {
          const freedom::Point point =
              block_bias + voxel_bias + static_voxel.points[sub_voxel].cast<double>();
          output.emplace_back(static_cast<float>(point.x()), static_cast<float>(point.y()),
                              static_cast<float>(point.z()));
        }
      }
      freedom::incrementIdx(local_voxel_idx, block_idx_size);
    }
  }

  output.width = static_cast<std::uint32_t>(output.size());
  output.height = 1;
  output.is_dense = true;
  return output;
}

}  // namespace

struct FreedomProcessor::Impl {
  Impl(const FreedomConfig& config, bool local_map) : min_range(config.sensor_min_range) {
    validateConfig(config);
    processor.set_params(toFreedomConfig(config, local_map));
    processor.set_map_removal_callback([this](const freedom::MRMap& map) {
      // FreeDOM 官方回调在 map removal + static integration 后触发，这里取完整静态地图。
      latest_static_map = extractStaticCloud(map);
    });
  }

  freedom::FreeDOM processor;
  Cloud latest_static_map;
  double min_range;
};

FreedomProcessor::FreedomProcessor(const FreedomConfig& config, bool local_map)
    : impl_(std::make_unique<Impl>(config, local_map)) {}

FreedomProcessor::~FreedomProcessor() = default;

Cloud FreedomProcessor::process(const Cloud& sensor_cloud,
                                const Eigen::Isometry3d& map_from_sensor) {
  validateTransform(map_from_sensor);

  // 在传感器坐标系中先移除近点，避免机器人自身回波进入静态地图和后续子图/BTC。
  const Cloud filtered = filterInputCloud(sensor_cloud, impl_->min_range);
  if (filtered.empty()) {
    return impl_->latest_static_map;
  }

  // 输入点云是传感器坐标系；官方算法负责用 map_from_sensor 变到 map frame。
  impl_->processor.pointcloud_integrate(filtered, map_from_sensor);
  return impl_->latest_static_map;
}

}  // namespace map_processing
