#include "relocalization/pose_tracking.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace relocalization_processing {
namespace {

void validatePose(const Eigen::Isometry3d& pose) {
  const Eigen::Matrix4d matrix = pose.matrix();
  const Eigen::Matrix3d rotation = pose.linear();
  if (!matrix.allFinite() ||
      !matrix.row(3).isApprox(Eigen::RowVector4d(0.0, 0.0, 0.0, 1.0), 1e-6) ||
      !(rotation.transpose() * rotation).isApprox(Eigen::Matrix3d::Identity(), 1e-6) ||
      std::abs(rotation.determinant() - 1.0) > 1e-6) {
    throw std::invalid_argument("tracking pose must be a finite rigid transform");
  }
}

void validateTime(std::int64_t stamp_ns, std::int64_t steady_now_ns) {
  if (stamp_ns <= 0 || steady_now_ns < 0) {
    throw std::invalid_argument("tracking requires a positive data stamp and nonnegative steady time");
  }
}

double elapsed(std::int64_t now_ns, std::int64_t then_ns) {
  return std::max(0.0, static_cast<double>(now_ns - then_ns) / 1e9);
}

}  // namespace

PoseTracking::PoseTracking(const TrackingConfig& config) : config_(config) {
  if (!std::isfinite(config.fix_timeout) || config.fix_timeout <= 0.0 ||
      !std::isfinite(config.odom_timeout) || config.odom_timeout <= 0.0) {
    throw std::invalid_argument("tracking timeouts must be finite and positive");
  }
}

bool PoseTracking::updateFix(const Eigen::Isometry3d& map_from_odom,
                             std::int64_t query_stamp_ns, std::int64_t query_started_steady_ns,
                             std::int64_t steady_now_ns,
                             std::uint64_t query_generation) {
  validatePose(map_from_odom);
  validateTime(query_stamp_ns, steady_now_ns);
  if (query_started_steady_ns < 0 || query_started_steady_ns > steady_now_ns) {
    throw std::invalid_argument("tracking query start must be nonnegative and no later than completion");
  }
  if (query_generation != generation_ ||
      (fix_ && query_stamp_ns <= fix_->stamp_ns) ||
      elapsed(steady_now_ns, query_started_steady_ns) > config_.fix_timeout ||
      elapsed(odom_stamp_ns_, query_stamp_ns) > config_.fix_timeout) {
    return false;
  }
  fix_ = Fix{map_from_odom, query_stamp_ns, query_started_steady_ns};
  return true;
}

TrackingResult PoseTracking::updateOdometry(const Eigen::Isometry3d& odom_from_sensor,
                                            std::int64_t stamp_ns,
                                            std::int64_t steady_now_ns) {
  validatePose(odom_from_sensor);
  validateTime(stamp_ns, steady_now_ns);
  TrackingResult result;
  result.stamp_ns = stamp_ns;
  if (stamp_ns < odom_stamp_ns_) {
    fix_.reset();
    ++generation_;
    published_stamp_ns_ = 0;
    odom_stamp_ns_ = stamp_ns;
    odom_received_steady_ns_ = steady_now_ns;
    result.status = "odometry_reset";
    return result;
  }
  if (stamp_ns > odom_stamp_ns_) {
    odom_stamp_ns_ = stamp_ns;
    odom_received_steady_ns_ = steady_now_ns;
  }
  if (!fix_) {
    return result;
  }
  result.fix_stamp_ns = fix_->stamp_ns;
  result.fix_age = std::max(elapsed(stamp_ns, fix_->stamp_ns),
                            elapsed(steady_now_ns, fix_->query_started_steady_ns));
  if (stamp_ns < fix_->stamp_ns) {
    result.status = "waiting_for_odometry";
    return result;
  }
  if (elapsed(steady_now_ns, odom_received_steady_ns_) > config_.odom_timeout) {
    result.status = "odometry_stale";
    return result;
  }
  if (result.fix_age > config_.fix_timeout) {
    result.status = "fix_expired";
    return result;
  }
  const Eigen::Isometry3d map_from_sensor = fix_->map_from_odom * odom_from_sensor;
  validatePose(map_from_sensor);
  result.valid = true;
  result.status = "tracking";
  result.map_from_odom = fix_->map_from_odom;
  result.map_from_sensor = map_from_sensor;
  result.publish = stamp_ns > published_stamp_ns_;
  if (result.publish) {
    published_stamp_ns_ = stamp_ns;
  }
  return result;
}

TrackingResult PoseTracking::unavailable(std::int64_t steady_now_ns) const {
  if (steady_now_ns < 0) {
    throw std::invalid_argument("tracking steady time must be nonnegative");
  }
  TrackingResult result;
  result.stamp_ns = odom_stamp_ns_;
  result.status = "odometry_unavailable";
  if (fix_) {
    result.fix_stamp_ns = fix_->stamp_ns;
    result.fix_age = std::max(elapsed(odom_stamp_ns_, fix_->stamp_ns),
                              elapsed(steady_now_ns, fix_->query_started_steady_ns));
  }
  return result;
}

}  // namespace relocalization_processing
