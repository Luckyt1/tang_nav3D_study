#pragma once
#include <math.h>

#include <cmath>
#include <cstdint>
#include <vector>
// #include <deque>
// #include <mutex>
// #include <thread>
#include <csignal>
#include <rclcpp/rclcpp.hpp>
// #include <so3_math.h>
#include <Eigen/Eigen>
// #include "Estimator.h"
#include <common_lib.h>
#include <pcl/common/io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Eigen>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

/// *************Preconfiguration

#define MAX_INI_COUNT (100)
const bool time_list(PointType & x, PointType & y);  // {return (x.curvature < y.curvature);};

/// *************IMU Process and undistortion
class ImuProcess
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ImuProcess();
  ~ImuProcess();

  void Reset();
  void Process(const MeasureGroup & meas, PointCloudXYZI::Ptr pcl_un_);
  void set_gyr_cov(const V3D & scaler);
  void set_acc_cov(const V3D & scaler);
  void Set_init(const std::vector<double> & configured_gravity, Eigen::Matrix3d & rot);
  const V3D & initial_gyro_bias() const { return mean_gyr; }

  MD(12, 12) state_cov = MD(12, 12)::Identity();
  int lidar_type;
  V3D gravity_;
  bool imu_en;
  // Acceleration magnitude at rest, in the incoming IMU's units (g or m/s^2).
  double acc_norm = 1.0;
  V3D mean_acc;
  bool imu_need_init_ = true;
  bool after_imu_init_ = false;
  bool b_first_frame_ = true;
  double time_last_scan = 0.0;
  V3D cov_gyr_scale = V3D(0.0001, 0.0001, 0.0001);
  V3D cov_vel_scale = V3D(0.0001, 0.0001, 0.0001);

private:
  void IMU_init(const MeasureGroup & meas);
  void ResetInitializationWindow();
  V3D mean_gyr;
  int init_iter_num = 0;
  int64_t init_start_stamp_ns_ = 0;
  int64_t last_init_stamp_ns_ = 0;
  bool have_init_stamp_ = false;
  rclcpp::Logger logger;
};
