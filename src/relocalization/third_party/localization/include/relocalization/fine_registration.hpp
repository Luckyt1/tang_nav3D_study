#ifndef RELOCALIZATION_FINE_REGISTRATION_HPP_
#define RELOCALIZATION_FINE_REGISTRATION_HPP_

#include <limits>
#include <unordered_map>

#include "relocalization/relocalization.hpp"

namespace relocalization_processing {

struct FineConfig {
  double voxel_size{0.1};
  double max_correspondence_distance{0.75};
  int max_iterations{100};
  std::size_t max_candidates{3};
  std::size_t min_points{50};
  double inlier_distance{0.25};
  double min_overlap{0.6};
  double max_rmse{0.15};
  double max_translation_correction{1.0};
  double max_rotation_correction{0.5235987755982988};  // radians: 30 degrees
};

struct FineResult {
  bool success{false};
  bool converged{false};
  std::uint64_t submap_id{0};
  std::size_t query_points{0};
  std::size_t target_points{0};
  std::size_t inliers{0};
  double overlap{0.0};
  double rmse{std::numeric_limits<double>::infinity()};
  double translation_correction{0.0};
  double rotation_correction{0.0};
  // Transforms are valid only when success is true; query means last sensor frame.
  Eigen::Isometry3d submap_from_query{Eigen::Isometry3d::Identity()};
  Eigen::Isometry3d map_from_query{Eigen::Isometry3d::Identity()};
  Eigen::Isometry3d map_from_odom{Eigen::Isometry3d::Identity()};
  std::string failure_reason;
};

// Reuses the accumulated BTC query cloud and saved submap; no ROS dependency.
// The database must outlive this object. Targets are downsampled and cached once.
class FineRegistration {
 public:
  explicit FineRegistration(const MapDatabase& database, const FineConfig& config = {});
  FineResult refine(const QueryResult& query);

 private:
  const MapDatabase& database_;
  FineConfig config_;
  std::unordered_map<std::uint64_t, Cloud::ConstPtr> targets_;
};

}  // namespace relocalization_processing

#endif  // RELOCALIZATION_FINE_REGISTRATION_HPP_
