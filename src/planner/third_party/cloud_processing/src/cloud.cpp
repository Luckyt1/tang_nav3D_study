#include "scan_planner/cloud_preprocessor.hpp"

#include <cmath>
#include <stdexcept>

#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>

namespace scan_planner
{

CloudPreprocessor::CloudPreprocessor(CloudPreprocessorConfig config)
{
  setConfig(config);
}

void CloudPreprocessor::setConfig(const CloudPreprocessorConfig & config)
{
  // 这些检查可以尽早发现参数写反、单位错误等问题，避免 PCL 产生无意义结果。
  if (config.voxel_size <= 0.0F || config.min_range < 0.0F ||
      config.max_range <= config.min_range || config.sor_mean_k < 1 ||
      config.sor_stddev <= 0.0) {
    throw std::invalid_argument("invalid point cloud preprocessing configuration");
  }
  config_ = config;
}

CloudPreprocessor::CloudPtr CloudPreprocessor::process(
  const Cloud & input, const Eigen::Quaternionf & orientation) const
{
  const auto finite = filterFiniteAndRange(input);
  const auto filtered = removeOutliers(*finite);
  const auto downsampled = voxelDownsample(*filtered);
  return alignToGravity(*downsampled, orientation);
}

CloudPreprocessor::CloudPtr CloudPreprocessor::filterFiniteAndRange(const Cloud & input) const
{
  // 去除 NaN/Inf，并按点到传感器原点的三维距离裁剪范围。
  // 使用距离平方比较，避免对每个点调用 sqrt。
  CloudPtr finite(new Cloud);
  finite->reserve(input.size());
  const float min_range_sq = config_.min_range * config_.min_range;
  const float max_range_sq = config_.max_range * config_.max_range;
  for (const auto & point : input.points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
      continue;
    }
    const float range_sq = point.x * point.x + point.y * point.y + point.z * point.z;
    if (range_sq >= min_range_sq && range_sq <= max_range_sq) {
      finite->push_back(point);
    }
  }
  finite->width = static_cast<std::uint32_t>(finite->size());
  finite->height = 1;
  finite->is_dense = true;
  return finite;
}

CloudPreprocessor::CloudPtr CloudPreprocessor::removeOutliers(const Cloud & input) const
{
  // 统计离群点滤波。点数太少时跳过该步骤，避免邻域不足。
  CloudPtr filtered(new Cloud);
  if (input.size() >= static_cast<std::size_t>(config_.sor_mean_k + 1)) {
    pcl::StatisticalOutlierRemoval<Point> sor;
    sor.setInputCloud(input.makeShared());
    sor.setMeanK(config_.sor_mean_k);
    sor.setStddevMulThresh(config_.sor_stddev);
    sor.filter(*filtered);
  } else {
    filtered = input.makeShared();
  }
  return filtered;
}

CloudPreprocessor::CloudPtr CloudPreprocessor::voxelDownsample(const Cloud & input) const
{
  // 体素降采样。每个体素只保留一个代表点，降低后续算法计算量。
  CloudPtr downsampled(new Cloud);
  pcl::VoxelGrid<Point> voxel;
  voxel.setInputCloud(input.makeShared());
  voxel.setLeafSize(config_.voxel_size, config_.voxel_size, config_.voxel_size);
  voxel.filter(*downsampled);
  return downsampled;
}

CloudPreprocessor::CloudPtr CloudPreprocessor::alignToGravity(
  const Cloud & input, const Eigen::Quaternionf & orientation) const
{
  // 姿态校正：只消除横滚和俯仰，保留航向角，使 Z 轴对齐重力方向。
  const Eigen::Vector3f euler = orientation.normalized().toRotationMatrix().eulerAngles(2, 1, 0);
  const Eigen::AngleAxisf yaw(euler[0], Eigen::Vector3f::UnitZ());
  const Eigen::AngleAxisf pitch(euler[1], Eigen::Vector3f::UnitY());
  const Eigen::AngleAxisf roll(euler[2], Eigen::Vector3f::UnitX());
  (void)yaw;  // yaw is intentionally preserved in the gravity-aligned frame.
  Eigen::Affine3f level_transform = Eigen::Affine3f::Identity();
  level_transform.linear() = (pitch * roll).inverse().toRotationMatrix();

  CloudPtr output(new Cloud);
  pcl::transformPointCloud(input, *output, level_transform);
  return output;
}

}  // namespace scan_planner
