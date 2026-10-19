#include "IMU_Processing.h"

#include <stdexcept>

const bool time_list(PointType & x, PointType & y) { return (x.curvature < y.curvature); };

void ImuProcess::set_gyr_cov(const V3D & scaler) { cov_gyr_scale = scaler; }

void ImuProcess::set_acc_cov(const V3D & scaler) { cov_vel_scale = scaler; }

ImuProcess::ImuProcess()
: b_first_frame_(true), imu_need_init_(true), logger(rclcpp::get_logger("ImuProcess"))
{
  imu_en = true;
  init_iter_num = 0;
  mean_acc = V3D(0, 0, 0.0);
  mean_gyr = V3D(0, 0, 0);
  after_imu_init_ = false;
  state_cov.setIdentity();
}

ImuProcess::~ImuProcess() {}

void ImuProcess::Reset()
{
  RCLCPP_WARN(logger, "reset ImuProcess");
  b_first_frame_ = true;
  ResetInitializationWindow();
  have_init_stamp_ = false;
  last_init_stamp_ns_ = 0;
  imu_need_init_ = true;
  after_imu_init_ = false;

  time_last_scan = 0.0;
}

void ImuProcess::Set_init(const std::vector<double> & configured_gravity, Eigen::Matrix3d & rot)
{
  const auto valid_direction = [](const Eigen::Vector3d & value) {
    return value.allFinite() && std::isfinite(value.norm()) && value.norm() > 1e-9;
  };
  if (!valid_direction(gravity_)) {
    throw std::invalid_argument("mapping.gravity must be finite and nonzero");
  }
  Eigen::Vector3d tmp_gravity;
  if (imu_en) {
    if (!valid_direction(mean_acc)) {
      throw std::invalid_argument("mean IMU acceleration must be finite and nonzero");
    }
    tmp_gravity = -mean_acc.normalized() * gravity_.norm();
  } else {
    if (configured_gravity.size() != 3) {
      throw std::invalid_argument("mapping.gravity_init must contain three values when IMU is disabled");
    }
    tmp_gravity = Eigen::Vector3d(
      configured_gravity[0], configured_gravity[1], configured_gravity[2]);
    if (!valid_direction(tmp_gravity)) {
      throw std::invalid_argument("mapping.gravity_init must be finite and nonzero");
    }
  }
  rot = Eigen::Quaterniond::FromTwoVectors(tmp_gravity, gravity_).normalized().toRotationMatrix();

  // Accelerometers constrain roll/pitch but not yaw.  The upside-down MID360
  // starts near the anti-parallel singularity, where tiny startup
  // acceleration used to choose either 0 or 180 degrees of yaw.  Pin that
  // unobservable gauge to zero while preserving the gravity alignment.
  const Eigen::Vector3d world_gravity = gravity_.normalized();
  if (std::abs(world_gravity.z()) > 1.0 - 1e-6) {
    const double yaw = std::atan2(rot(1, 0), rot(0, 0));
    rot =
      Eigen::AngleAxisd(-yaw * world_gravity.z(), world_gravity).toRotationMatrix() * rot;
  }
  RCLCPP_INFO(logger, "Gravity initialization: source=%s, body gravity=[%.4f, %.4f, %.4f] m/s^2",
    imu_en ? "IMU mean acceleration" : "mapping.gravity_init",
    tmp_gravity.x(), tmp_gravity.y(), tmp_gravity.z());
}

void ImuProcess::ResetInitializationWindow()
{
  mean_acc.setZero();
  mean_gyr.setZero();
  init_iter_num = 0;
  init_start_stamp_ns_ = 0;
}

void ImuProcess::IMU_init(const MeasureGroup & meas)
{
  if (!std::isfinite(acc_norm) || acc_norm <= 0.0) {
    throw std::invalid_argument("mapping.acc_norm must be finite and positive");
  }
  b_first_frame_ = false;

  for (const auto & imu : meas.imu) {
    const int64_t stamp_ns = static_cast<int64_t>(imu->header.stamp.sec) * 1000000000LL +
      imu->header.stamp.nanosec;
    // sync_packages may pass a cumulative batch. Never count old samples twice;
    // an actual stream restart is handled by the caller's ImuProcess::Reset().
    if (have_init_stamp_ && stamp_ns <= last_init_stamp_ns_) continue;
    if (have_init_stamp_ && stamp_ns - last_init_stamp_ns_ > 100000000LL) {
      ResetInitializationWindow();
    }
    have_init_stamp_ = true;
    last_init_stamp_ns_ = stamp_ns;

    const auto & imu_acc = imu->linear_acceleration;
    const auto & gyr_acc = imu->angular_velocity;
    const V3D cur_acc(imu_acc.x, imu_acc.y, imu_acc.z);
    const V3D cur_gyr(gyr_acc.x, gyr_acc.y, gyr_acc.z);
    // Allow small nonzero gyro bias, but reject rotation/vibration and changes
    // in acceleration direction. Acceleration thresholds are independent of units.
    const bool quiet = cur_acc.allFinite() && cur_gyr.allFinite() &&
      std::abs(cur_acc.norm() / acc_norm - 1.0) <= 0.15 && cur_gyr.norm() <= 0.05 &&
      (init_iter_num == 0 ||
       ((cur_acc - mean_acc).norm() / acc_norm <= 0.05 &&
        (cur_gyr - mean_gyr).norm() <= 0.02));
    if (!quiet) {
      ResetInitializationWindow();
      continue;
    }
    if (init_iter_num == 0) init_start_stamp_ns_ = stamp_ns;
    ++init_iter_num;
    mean_acc += (cur_acc - mean_acc) / init_iter_num;
    mean_gyr += (cur_gyr - mean_gyr) / init_iter_num;
  }
}

/**
 * @brief 处理IMU和激光雷达数据的函数
 * @param meas 测量组，包含IMU和激光雷达数据
 * @param cur_pcl_un_ 未处理的点云指针，用于存储处理后的点云数据
 */
void ImuProcess::Process(const MeasureGroup & meas, PointCloudXYZI::Ptr cur_pcl_un_)
{
  // 检查IMU是否启用
  if (imu_en) {
    // 如果IMU数据为空，则直接返回
    if (meas.imu.empty()) return;

    // 如果需要初始化IMU
    if (imu_need_init_) {
      IMU_init(meas);
      if (init_iter_num >= MAX_INI_COUNT &&
          last_init_stamp_ns_ - init_start_stamp_ns_ >= 500000000LL) {
        RCLCPP_INFO(logger,
          "IMU initialized from %d stationary samples: gyro bias=[%.6f, %.6f, %.6f] rad/s",
          init_iter_num, mean_gyr.x(), mean_gyr.y(), mean_gyr.z());
        imu_need_init_ = false;
        *cur_pcl_un_ = *(meas.lidar);
      }
      return;
    }
    if (!after_imu_init_) after_imu_init_ = true;
    *cur_pcl_un_ = *(meas.lidar);
    return;
  } else {
    *cur_pcl_un_ = *(meas.lidar);
    return;
  }
}
