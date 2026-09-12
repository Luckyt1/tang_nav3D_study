#include "scan_planner/cloud_cut.hpp"

#include <cmath>
#include <algorithm>
#include <stdexcept>

#include <pcl/ModelCoefficients.h>
#include <pcl/PointIndices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

namespace scan_planner
{

CloudCut::CloudCut(GroundSeparatorConfig config)
: config_(config)
{
  if (config_.distance_threshold <= 0.0F || config_.max_iterations < 1 ||
      config_.max_ground_slope_deg <= 0.0F || config_.max_ground_slope_deg >= 90.0F) {
    throw std::invalid_argument("invalid ground separator configuration");
  }
}

GroundPlaneResult CloudCut::separate(const Cloud & input) const
{
  GroundPlaneResult result;
  if (input.size() < 3U) {
    result.obstacles = input.makeShared();
    return result;
  }

  pcl::SACSegmentation<pcl::PointXYZ> segmentation;
  segmentation.setOptimizeCoefficients(true);
  segmentation.setModelType(pcl::SACMODEL_PLANE);
  segmentation.setMethodType(pcl::SAC_RANSAC);
  segmentation.setDistanceThreshold(config_.distance_threshold);
  segmentation.setMaxIterations(config_.max_iterations);
  segmentation.setInputCloud(input.makeShared());

  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  segmentation.segment(*inliers, *coefficients);
  if (inliers->indices.size() < 3U || coefficients->values.size() < 4U) {
    result.obstacles = input.makeShared();
    return result;
  }

  Eigen::Vector3f normal(coefficients->values[0], coefficients->values[1], coefficients->values[2]);
  const float normal_length = normal.norm();
  if (normal_length <= 1e-6F) {
    result.obstacles = input.makeShared();
    return result;
  }
  normal.normalize();
  constexpr float kPi = 3.14159265358979323846F;
  const float slope = std::acos(std::clamp(std::abs(normal.z()), 0.0F, 1.0F)) * 180.0F / kPi;
  result.slope_deg = slope;
  result.plane = Eigen::Vector4f(normal.x(), normal.y(), normal.z(), coefficients->values[3] / normal_length);
  result.point_heights.reserve(input.size());
  result.point_normals.reserve(input.size());
  result.point_slopes_deg.reserve(input.size());
  for (const auto & point : input.points) {
    result.point_heights.push_back(
      result.plane.x() * point.x + result.plane.y() * point.y +
      result.plane.z() * point.z + result.plane.w());
    result.point_normals.push_back(normal);
    result.point_slopes_deg.push_back(slope);
  }
  if (slope > config_.max_ground_slope_deg) {
    result.obstacles = input.makeShared();
    return result;
  }

  pcl::ExtractIndices<pcl::PointXYZ> extractor;
  extractor.setInputCloud(input.makeShared());
  extractor.setIndices(inliers);
  extractor.setNegative(false);
  extractor.filter(*result.ground);
  extractor.setNegative(true);
  extractor.filter(*result.obstacles);
  result.valid = true;
  return result;
}

}  // namespace scan_planner
