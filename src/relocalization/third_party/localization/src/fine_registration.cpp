#include "relocalization/fine_registration.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/icp.h>

namespace relocalization_processing {
namespace {

void require(bool condition, const std::string& message) {
  if (!condition) {
    throw std::invalid_argument(message);
  }
}

bool rigid(const Eigen::Matrix4d& matrix, double tolerance = 1e-6) {
  if (!matrix.allFinite() ||
      !matrix.row(3).isApprox(Eigen::RowVector4d(0.0, 0.0, 0.0, 1.0), tolerance)) {
    return false;
  }
  const Eigen::Matrix3d rotation = matrix.topLeftCorner<3, 3>();
  return (rotation.transpose() * rotation).isApprox(Eigen::Matrix3d::Identity(), tolerance) &&
         std::abs(rotation.determinant() - 1.0) <= tolerance;
}

Cloud::Ptr downsample(const Cloud& cloud, double voxel_size) {
  for (const auto& point : cloud) {
    require(point.getVector3fMap().allFinite(), "fine registration cloud must be finite");
  }
  auto result = pcl::make_shared<Cloud>();
  if (!cloud.empty()) {
    pcl::VoxelGrid<pcl::PointXYZ> filter;
    const float leaf = static_cast<float>(voxel_size);
    filter.setLeafSize(leaf, leaf, leaf);
    filter.setInputCloud(cloud.makeShared());
    filter.filter(*result);
    for (const auto& point : *result) {
      require(point.getVector3fMap().allFinite(), "downsampled fine registration cloud must be finite");
    }
  }
  return result;
}

FineResult refineCandidate(const Cloud::ConstPtr& source, const Cloud::ConstPtr& target,
                           const Candidate& candidate, const Eigen::Isometry3d& odom_from_query,
                           const FineConfig& config) {
  FineResult result;
  result.submap_id = candidate.submap_id;
  result.query_points = source->size();
  result.target_points = target->size();
  if (target->size() < config.min_points) {
    result.failure_reason = "insufficient_target_points";
    return result;
  }

  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setInputSource(source);
  icp.setInputTarget(target);
  icp.setMaxCorrespondenceDistance(config.max_correspondence_distance);
  icp.setMaximumIterations(config.max_iterations);
  icp.setTransformationEpsilon(1e-8);
  icp.setEuclideanFitnessEpsilon(1e-6);
  icp.getConvergeCriteria()->setFailureAfterMaximumIterations(true);
  Cloud aligned;
  const Eigen::Matrix4f initial = candidate.submap_from_query.matrix().cast<float>();
  require(initial.allFinite(), "BTC initial transform exceeds float range");
  icp.align(aligned, initial);
  result.converged = icp.hasConverged();
  const Eigen::Matrix4d final = icp.getFinalTransformation().cast<double>();
  // ICP accumulates float rotations; allow rounding error, then restore an exact rotation.
  if (!rigid(final, 1e-3)) {
    result.failure_reason = "invalid_icp_transform";
    return result;
  }
  Eigen::Isometry3d submap_from_query = Eigen::Isometry3d::Identity();
  submap_from_query.linear() =
    Eigen::Quaterniond(Eigen::Matrix3d(final.topLeftCorner<3, 3>())).normalized().toRotationMatrix();
  submap_from_query.translation() = final.topRightCorner<3, 1>();
  result.translation_correction =
    (submap_from_query.translation() - candidate.submap_from_query.translation()).norm();
  result.rotation_correction = Eigen::AngleAxisd(
    submap_from_query.linear() * candidate.submap_from_query.linear().transpose()).angle();

  pcl::KdTreeFLANN<pcl::PointXYZ> tree;
  tree.setInputCloud(target);
  std::vector<int> indices(1);
  std::vector<float> squared_distances(1);
  const double max_squared_distance = config.inlier_distance * config.inlier_distance;
  double squared_error_sum = 0.0;
  for (const auto& point : *source) {
    const Eigen::Vector3d transformed =
      submap_from_query * point.getVector3fMap().cast<double>();
    const pcl::PointXYZ aligned_point(static_cast<float>(transformed.x()),
                                      static_cast<float>(transformed.y()),
                                      static_cast<float>(transformed.z()));
    if (!aligned_point.getVector3fMap().allFinite()) {
      result.failure_reason = "invalid_icp_transform";
      return result;
    }
    if (tree.nearestKSearch(aligned_point, 1, indices, squared_distances) == 1 &&
        std::isfinite(squared_distances[0]) && squared_distances[0] <= max_squared_distance) {
      ++result.inliers;
      squared_error_sum += squared_distances[0];
    }
  }
  result.overlap = static_cast<double>(result.inliers) / result.query_points;
  if (result.inliers > 0) {
    result.rmse = std::sqrt(squared_error_sum / result.inliers);
  }

  if (!result.converged) {
    result.failure_reason = "icp_not_converged";
  } else if (result.inliers < config.min_points) {
    result.failure_reason = "insufficient_inliers";
  } else if (result.overlap < config.min_overlap) {
    result.failure_reason = "low_overlap";
  } else if (result.rmse > config.max_rmse) {
    result.failure_reason = "high_rmse";
  } else if (result.translation_correction > config.max_translation_correction) {
    result.failure_reason = "excessive_translation_correction";
  } else if (result.rotation_correction > config.max_rotation_correction) {
    result.failure_reason = "excessive_rotation_correction";
  } else {
    const Eigen::Isometry3d map_from_submap =
      candidate.map_from_query * candidate.submap_from_query.inverse();
    const Eigen::Isometry3d map_from_query = map_from_submap * submap_from_query;
    const Eigen::Isometry3d map_from_odom = map_from_query * odom_from_query.inverse();
    if (!rigid(map_from_query.matrix()) || !rigid(map_from_odom.matrix())) {
      result.failure_reason = "invalid_pose_composition";
      return result;
    }
    result.success = true;
    result.submap_from_query = submap_from_query;
    result.map_from_query = map_from_query;
    result.map_from_odom = map_from_odom;
  }
  return result;
}

}  // namespace

FineRegistration::FineRegistration(const MapDatabase& database, const FineConfig& config)
    : database_(database), config_(config) {
  const auto positive = [](double value) { return std::isfinite(value) && value > 0.0; };
  require(positive(config.voxel_size) &&
            std::isfinite(static_cast<float>(config.voxel_size)) &&
            std::isfinite(1.0F / static_cast<float>(config.voxel_size)),
          "FineConfig.voxel_size must be positive and representable as a float leaf size");
  require(positive(config.max_correspondence_distance),
          "FineConfig.max_correspondence_distance must be finite and positive");
  require(config.max_iterations > 0, "FineConfig.max_iterations must be positive");
  require(config.max_candidates > 0 && config.max_candidates <= 50,
          "FineConfig.max_candidates must be in [1, 50]");
  require(config.min_points >= 3, "FineConfig.min_points must be at least 3");
  require(positive(config.inlier_distance), "FineConfig.inlier_distance must be finite and positive");
  require(positive(config.min_overlap) && config.min_overlap <= 1.0,
          "FineConfig.min_overlap must be in (0, 1]");
  require(positive(config.max_rmse), "FineConfig.max_rmse must be finite and positive");
  require(positive(config.max_translation_correction),
          "FineConfig.max_translation_correction must be finite and positive");
  require(positive(config.max_rotation_correction) && config.max_rotation_correction <= std::acos(-1.0),
          "FineConfig.max_rotation_correction must be in (0, pi]");
}

FineResult FineRegistration::refine(const QueryResult& query) {
  require(rigid(query.odom_from_query.matrix()), "query odom_from_query must be a finite rigid pose");
  const auto source = downsample(query.query_cloud, config_.voxel_size);
  FineResult best;
  best.query_points = source->size();
  if (query.candidates.empty()) {
    best.failure_reason = "no_candidates";
    return best;
  }
  if (source->size() < config_.min_points) {
    best.failure_reason = "insufficient_query_points";
    return best;
  }

  const auto count = std::min(query.candidates.size(), config_.max_candidates);
  for (std::size_t i = 0; i < count; ++i) {
    const auto& candidate = query.candidates[i];
    require(rigid(candidate.submap_from_query.matrix()), "BTC submap_from_query must be a finite rigid pose");
    require(rigid(candidate.map_from_query.matrix()), "BTC map_from_query must be a finite rigid pose");
    auto found = targets_.find(candidate.submap_id);
    if (found == targets_.end()) {
      found = targets_.emplace(candidate.submap_id,
        downsample(database_.loadSubmap(candidate.submap_id), config_.voxel_size)).first;
    }
    const auto attempt = refineCandidate(source, found->second, candidate, query.odom_from_query, config_);
    if (i == 0 || (attempt.success && !best.success) ||
        (attempt.success == best.success &&
         (attempt.overlap > best.overlap ||
          (attempt.overlap == best.overlap && attempt.rmse < best.rmse)))) {
      best = attempt;
    }
  }
  return best;
}

}  // namespace relocalization_processing
