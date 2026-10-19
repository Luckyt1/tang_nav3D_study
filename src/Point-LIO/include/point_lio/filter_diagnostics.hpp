// SPDX-License-Identifier: BSD-3-Clause
#ifndef POINT_LIO_FILTER_DIAGNOSTICS_HPP
#define POINT_LIO_FILTER_DIAGNOSTICS_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <limits>
#include <ostream>
#include <string>

namespace point_lio
{

// Streaming statistics of magnitudes, not signed or net motion. No sample storage.
struct MagnitudeStatistics
{
  std::size_t count = 0;
  std::size_t invalid = 0;
  double sum = 0.0;
  double squared_sum = 0.0;
  double maximum = 0.0;

  void add(double value)
  {
    if (!std::isfinite(value)) {
      ++invalid;
      return;
    }
    value = std::abs(value);
    ++count;
    sum += value;
    squared_sum += value * value;
    maximum = std::max(maximum, value);
  }

  double rms() const
  {
    return count ? std::sqrt(squared_sum / count) : std::numeric_limits<double>::quiet_NaN();
  }

  void write(std::ostream & out) const
  {
    out << ',' << count << ',' << invalid << ',' << sum << ',' << rms() << ','
        << (count ? maximum : std::numeric_limits<double>::quiet_NaN());
  }

  static void writeHeader(std::ostream & out, const char * name, const char * unit)
  {
    out << ',' << name << "_count," << name << "_invalid," << name << "_sum_" << unit
        << ',' << name << "_rms_" << unit << ',' << name << "_max_" << unit;
  }
};

struct UpdateStatistics
{
  std::size_t attempts = 0;
  std::size_t failures = 0;
  MagnitudeStatistics position;
  MagnitudeStatistics rotation;

  void write(std::ostream & out) const
  {
    out << ',' << attempts << ',' << failures;
    position.write(out);
    rotation.write(out);
  }

  static void writeHeader(std::ostream & out, const char * name)
  {
    out << ',' << name << "_updates," << name << "_failures";
    const std::string position_name = std::string(name) + "_position_correction";
    const std::string rotation_name = std::string(name) + "_rotation_correction";
    MagnitudeStatistics::writeHeader(out, position_name.c_str(), "m");
    MagnitudeStatistics::writeHeader(out, rotation_name.c_str(), "rad");
  }
};

enum class UpdateKind { Imu, Lidar };

struct ImuCallbackStatistics
{
  std::size_t received = 0;
  std::size_t accepted = 0;
  std::size_t gaps_over_7p5ms = 0;
  std::size_t backwards = 0;
  std::size_t duplicates = 0;
  double first_stamp = 0.0;
  double last_stamp = 0.0;
  double maximum_gap = 0.0;

  void record(double stamp)
  {
    if (received == 0) {
      first_stamp = stamp;
    } else {
      const double dt = stamp - last_stamp;
      if (dt > 0.0075) ++gaps_over_7p5ms;
      if (dt < 0.0) ++backwards;
      if (dt == 0.0) ++duplicates;
      maximum_gap = std::max(maximum_gap, dt);
    }
    ++received;
    last_stamp = stamp;
  }
};

struct FrameDiagnostics
{
  // Lifetime counters survive frame reset: spin_some() receives data before the
  // current frame begins, so resetting these there would lose the evidence.
  ImuCallbackStatistics imu_callbacks;
  UpdateStatistics imu;
  UpdateStatistics lidar;
  MagnitudeStatistics gyro_residual;
  MagnitudeStatistics accel_residual;
  MagnitudeStatistics lidar_residual;
  std::size_t saturated_imu_axes = 0;
  std::size_t negative_state_dt = 0;
  std::size_t negative_covariance_dt = 0;
  bool lidar_residual_recorded = false;

  void resetFrame()
  {
    const auto callbacks = imu_callbacks;
    *this = FrameDiagnostics{};
    imu_callbacks = callbacks;
  }

  template <typename Derived>
  void recordLidarResiduals(const Eigen::MatrixBase<Derived> & residual)
  {
    // The iterated filter can evaluate this model repeatedly for the same group.
    // Count only its first linearization, before any correction in that update.
    if (lidar_residual_recorded) return;
    lidar_residual_recorded = true;
    for (Eigen::Index i = 0; i < residual.size(); ++i) lidar_residual.add(residual(i));
  }

  void recordPrediction(double dt, bool state, bool covariance)
  {
    if (dt < 0.0) {
      if (state) ++negative_state_dt;
      if (covariance) ++negative_covariance_dt;
    }
  }

  static void writeHeader(std::ostream & out)
  {
    out << "frame_start_s,frame_end_s,state_time_s,use_imu_as_input,downsampled_points";
    UpdateStatistics::writeHeader(out, "imu");
    UpdateStatistics::writeHeader(out, "lidar");
    MagnitudeStatistics::writeHeader(out, "imu_gyro_residual_norm", "rad_s");
    MagnitudeStatistics::writeHeader(out, "imu_accel_residual_norm", "m_s2");
    MagnitudeStatistics::writeHeader(out, "lidar_prior_residual", "m");
    out << ",saturated_imu_axes,negative_state_dt,negative_covariance_dt"
        << ",imu_callbacks_total,imu_accepted_total,imu_callback_gaps_over_7p5ms_total"
        << ",imu_callback_max_gap_s,imu_callback_backwards_total,imu_callback_duplicates_total"
        << ",imu_callback_first_stamp_s,imu_callback_last_stamp_s"
        << ",pos_x_m,pos_y_m,pos_z_m,quat_x,quat_y,quat_z,quat_w"
        << ",vel_x_m_s,vel_y_m_s,vel_z_m_s,bg_x_rad_s,bg_y_rad_s,bg_z_rad_s"
        << ",ba_x_m_s2,ba_y_m_s2,ba_z_m_s2\n";
  }

  template <typename State>
  void writeRow(
    std::ostream & out, double start, double end, double state_time, bool imu_as_input,
    std::size_t point_count, const State & state) const
  {
    out << std::setprecision(17) << start << ',' << end << ',' << state_time << ','
        << imu_as_input << ',' << point_count;
    imu.write(out);
    lidar.write(out);
    gyro_residual.write(out);
    accel_residual.write(out);
    lidar_residual.write(out);
    out << ',' << saturated_imu_axes << ',' << negative_state_dt << ',' << negative_covariance_dt;
    out << ',' << imu_callbacks.received << ',' << imu_callbacks.accepted
        << ',' << imu_callbacks.gaps_over_7p5ms << ',' << imu_callbacks.maximum_gap
        << ',' << imu_callbacks.backwards << ',' << imu_callbacks.duplicates
        << ',' << imu_callbacks.first_stamp << ',' << imu_callbacks.last_stamp;
    for (int i = 0; i < 3; ++i) out << ',' << state.pos(i);
    const auto q = state.rot.quaternion();
    out << ',' << q.x() << ',' << q.y() << ',' << q.z() << ',' << q.w();
    for (int i = 0; i < 3; ++i) out << ',' << state.vel(i);
    for (int i = 0; i < 3; ++i) out << ',' << state.bg(i);
    for (int i = 0; i < 3; ++i) out << ',' << state.ba(i);
    out << '\n';
  }
};

template <typename State, typename Update>
bool observeUpdate(FrameDiagnostics * diagnostics, UpdateKind kind, const State & state, Update update)
{
  // In normal operation, do not copy the state or measurement matrices.
  if (diagnostics == nullptr) return update();
  auto & statistics = kind == UpdateKind::Imu ? diagnostics->imu : diagnostics->lidar;
  if (kind == UpdateKind::Lidar) diagnostics->lidar_residual_recorded = false;
  ++statistics.attempts;
  const Eigen::Vector3d position = state.pos;
  const Eigen::Quaterniond rotation = state.rot.quaternion();
  const bool accepted = update();
  if (!accepted) {
    ++statistics.failures;
    return false;
  }
  statistics.position.add((state.pos - position).norm());
  statistics.rotation.add(rotation.angularDistance(state.rot.quaternion()));
  return true;
}

}  // namespace point_lio
#endif  // POINT_LIO_FILTER_DIAGNOSTICS_HPP
