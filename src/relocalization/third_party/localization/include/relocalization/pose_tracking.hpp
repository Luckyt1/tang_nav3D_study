#ifndef RELOCALIZATION_POSE_TRACKING_HPP_
#define RELOCALIZATION_POSE_TRACKING_HPP_

#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <Eigen/Geometry>

namespace relocalization_processing {

struct TrackingConfig {
  double fix_timeout{5.0};
  double odom_timeout{1.0};
};

struct TrackingResult {
  bool valid{false};
  bool publish{false};  // Only a new odometry timestamp may publish a pose/TF.
  std::string status{"waiting_for_fix"};
  std::int64_t stamp_ns{0};
  std::int64_t fix_stamp_ns{0};
  double fix_age{std::numeric_limits<double>::infinity()};
  Eigen::Isometry3d map_from_odom{Eigen::Isometry3d::Identity()};
  Eigen::Isometry3d map_from_sensor{Eigen::Isometry3d::Identity()};
};

// Caller serializes access. Steady timestamps are monotonic nanoseconds, independent of ROS time.
class PoseTracking {
 public:
  explicit PoseTracking(const TrackingConfig& config = {});
  std::uint64_t generation() const { return generation_; }
  bool updateFix(const Eigen::Isometry3d& map_from_odom, std::int64_t query_stamp_ns,
                 std::int64_t query_started_steady_ns, std::int64_t steady_now_ns,
                 std::uint64_t query_generation);
  TrackingResult updateOdometry(const Eigen::Isometry3d& odom_from_sensor,
                                std::int64_t stamp_ns, std::int64_t steady_now_ns);
  TrackingResult unavailable(std::int64_t steady_now_ns) const;

 private:
  struct Fix {
    Eigen::Isometry3d map_from_odom;
    std::int64_t stamp_ns;
    std::int64_t query_started_steady_ns;
  };
  TrackingConfig config_;
  std::optional<Fix> fix_;
  std::uint64_t generation_{0};
  std::int64_t odom_stamp_ns_{0};
  std::int64_t odom_received_steady_ns_{0};
  std::int64_t published_stamp_ns_{0};
};

}  // namespace relocalization_processing
#endif  // RELOCALIZATION_POSE_TRACKING_HPP_
