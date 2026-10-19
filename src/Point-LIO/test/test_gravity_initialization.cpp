#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <stdexcept>

#include "IMU_Processing.h"

namespace {

void appendImuSamples(
  MeasureGroup & measurements, int count, const Eigen::Vector3d & gyro,
  const Eigen::Vector3d & acceleration = Eigen::Vector3d(0, 0, 1),
  int64_t first_stamp_ns = 1000000000LL, int64_t interval_ns = 5000000LL)
{
  for (int i = 0; i < count; ++i) {
    auto sample = std::make_shared<sensor_msgs::msg::Imu>();
    const int64_t stamp = first_stamp_ns + i * interval_ns;
    sample->header.stamp.sec = static_cast<int32_t>(stamp / 1000000000LL);
    sample->header.stamp.nanosec = static_cast<uint32_t>(stamp % 1000000000LL);
    sample->angular_velocity.x = gyro.x();
    sample->angular_velocity.y = gyro.y();
    sample->angular_velocity.z = gyro.z();
    sample->linear_acceleration.x = acceleration.x();
    sample->linear_acceleration.y = acceleration.y();
    sample->linear_acceleration.z = acceleration.z();
    measurements.imu.push_back(sample);
  }
}

Eigen::Matrix3d initializeGravity(Eigen::Vector3d measured_gravity)
{
  ImuProcess process;
  process.gravity_ = Eigen::Vector3d(0.0, 0.0, -9.81);
  process.mean_acc = -measured_gravity;
  Eigen::Matrix3d rotation;
  // Fixed mount configuration must not override a measured IMU direction.
  process.Set_init({0.0, 0.0, 9.81}, rotation);
  EXPECT_TRUE(rotation.allFinite());
  EXPECT_TRUE((rotation.transpose() * rotation).isApprox(Eigen::Matrix3d::Identity(), 1e-9));
  EXPECT_NEAR(rotation.determinant(), 1.0, 1e-9);
  EXPECT_TRUE((rotation * measured_gravity.normalized()).isApprox(
    process.gravity_.normalized(), 1e-6));
  return rotation;
}

TEST(GravityInitialization, UprightMountRemainsLevel)
{
  EXPECT_TRUE(initializeGravity({0.0, 0.0, -9.81}).isApprox(Eigen::Matrix3d::Identity(), 1e-9));
}

TEST(GravityInitialization, UpsideDownMountMatchesRollCorrection)
{
  const auto rotation = initializeGravity({0.0, 0.0, 9.81});
  const Eigen::Matrix3d mounting =
    Eigen::AngleAxisd(std::acos(-1.0), Eigen::Vector3d::UnitX()).toRotationMatrix();
  EXPECT_TRUE((rotation * mounting.transpose()).isApprox(Eigen::Matrix3d::Identity(), 1e-9));
}

TEST(GravityInitialization, StartupNoiseDoesNotFlipUpsideDownYaw)
{
  for (double x : {-0.01, 0.0, 0.01}) {
    for (double y : {-0.01, 0.0, 0.01}) {
      const auto rotation = initializeGravity({x, y, 9.81});
      EXPECT_NEAR(std::atan2(rotation(1, 0), rotation(0, 0)), 0.0, 1e-9);
    }
  }
}

TEST(GravityInitialization, RecordedStaticImuTiltIsAlignedAfterAveraging)
{
  ImuProcess process;
  process.gravity_ = Eigen::Vector3d(0.0, 0.0, -9.81);
  // Actual stationary MID360S mean, in the driver's native acceleration units.
  const Eigen::Vector3d acceleration(-0.1010430629, 0.0070568519, -0.9925465157);
  MeasureGroup measurements;
  for (int i = 0; i < 120; ++i) {
    auto sample = std::make_shared<sensor_msgs::msg::Imu>();
    sample->header.stamp.sec = 1;
    sample->header.stamp.nanosec = i * 5000000;
    const double perturbation = (i % 2 == 0 ? 1.0 : -1.0) * 0.002;
    sample->linear_acceleration.x = acceleration.x() + perturbation;
    sample->linear_acceleration.y = acceleration.y();
    sample->linear_acceleration.z = acceleration.z() - perturbation;
    measurements.imu.push_back(sample);
  }
  auto cloud = std::make_shared<PointCloudXYZI>();
  process.Process(measurements, cloud);
  ASSERT_FALSE(process.imu_need_init_);
  ASSERT_TRUE(process.mean_acc.isApprox(acceleration, 1e-9));

  Eigen::Matrix3d rotation;
  process.Set_init({0.0, 0.0, 9.81}, rotation);
  EXPECT_TRUE((rotation * -acceleration.normalized()).isApprox(
    process.gravity_.normalized(), 1e-9));
  EXPECT_TRUE((rotation.transpose() * rotation).isApprox(Eigen::Matrix3d::Identity(), 1e-9));
  EXPECT_NEAR(rotation.determinant(), 1.0, 1e-9);
  EXPECT_NEAR(std::atan2(rotation(1, 0), rotation(0, 0)), 0.0, 1e-9);
  const Eigen::Matrix3d fixed_mount =
    Eigen::AngleAxisd(std::acos(-1.0), Eigen::Vector3d::UnitX()).toRotationMatrix();
  EXPECT_GT(Eigen::AngleAxisd(rotation * fixed_mount.transpose()).angle(), 5.0 * M_PI / 180.0);
}

TEST(GravityInitialization, EnabledImuDoesNotRequireFallbackConfiguration)
{
  ImuProcess process;
  process.gravity_ = Eigen::Vector3d(0.0, 0.0, -9.81);
  process.mean_acc = Eigen::Vector3d(0.0, 0.0, 1.0);
  Eigen::Matrix3d rotation;
  process.Set_init({}, rotation);
  EXPECT_TRUE(rotation.isApprox(Eigen::Matrix3d::Identity(), 1e-9));
  process.Set_init({std::numeric_limits<double>::quiet_NaN()}, rotation);
  EXPECT_TRUE(rotation.isApprox(Eigen::Matrix3d::Identity(), 1e-9));
}

TEST(GravityInitialization, DisabledImuUsesConfiguredDirection)
{
  ImuProcess process;
  process.imu_en = false;
  process.gravity_ = Eigen::Vector3d(0.0, 0.0, -9.81);
  process.mean_acc.setConstant(std::numeric_limits<double>::quiet_NaN());
  const Eigen::Vector3d fallback(1.0, 0.2, 9.81);
  Eigen::Matrix3d rotation;
  process.Set_init({fallback.x(), fallback.y(), fallback.z()}, rotation);
  EXPECT_TRUE((rotation * fallback.normalized()).isApprox(process.gravity_.normalized(), 1e-9));
}

TEST(GravityInitialization, InvalidDirectionsCannotProduceAnInitialPose)
{
  ImuProcess process;
  process.gravity_ = Eigen::Vector3d(0.0, 0.0, -9.81);
  Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
  for (const auto & value : {Eigen::Vector3d::Zero().eval(),
                            Eigen::Vector3d(std::numeric_limits<double>::quiet_NaN(), 0, 1),
                            Eigen::Vector3d(std::numeric_limits<double>::infinity(), 0, 1)}) {
    process.mean_acc = value;
    EXPECT_THROW(process.Set_init({0, 0, 9.81}, rotation), std::invalid_argument);
    EXPECT_TRUE(rotation.isApprox(Eigen::Matrix3d::Identity()));
  }
  process.imu_en = false;
  for (const auto & value : std::vector<std::vector<double>>{
         {}, {0, 1}, {0, 0, 0}, {0, 0, 1, 2},
         {0, 0, std::numeric_limits<double>::quiet_NaN()}}) {
    EXPECT_THROW(process.Set_init(value, rotation), std::invalid_argument);
  }
  process.gravity_.setZero();
  EXPECT_THROW(process.Set_init({0, 0, 9.81}, rotation), std::invalid_argument);
}

TEST(GyroBiasInitialization, QuietNonzeroBiasIsAcceptedInBothAccelerationUnits)
{
  const Eigen::Vector3d bias(-0.006, 0.001, -0.0055);
  for (const double acceleration_scale : {1.0, 9.81}) {
    ImuProcess process;
    process.acc_norm = acceleration_scale;
    MeasureGroup measurements;
    appendImuSamples(measurements, 120, bias, {0, 0, -acceleration_scale});
    measurements.lidar->push_back(PointType());
    auto cloud = std::make_shared<PointCloudXYZI>();
    process.Process(measurements, cloud);
    ASSERT_FALSE(process.imu_need_init_);
    EXPECT_TRUE(process.initial_gyro_bias().isApprox(bias, 1e-12));
    EXPECT_EQ(cloud->size(), 1U);
  }
}

TEST(GyroBiasInitialization, RepeatedCumulativeBatchesDoNotCountAsNewSamples)
{
  ImuProcess process;
  MeasureGroup measurements;
  const Eigen::Vector3d bias(0.006, -0.002, 0.003);
  appendImuSamples(measurements, 20, bias);
  auto cloud = std::make_shared<PointCloudXYZI>();
  for (int i = 0; i < 10; ++i) {
    process.Process(measurements, cloud);
    EXPECT_TRUE(process.imu_need_init_);
  }
  // Duplicate/out-of-order history must not contaminate the new quiet window.
  appendImuSamples(measurements, 20, {0.8, 0, 0});
  appendImuSamples(measurements, 100, bias, {0, 0, 1}, 1100000000LL);
  process.Process(measurements, cloud);
  ASSERT_FALSE(process.imu_need_init_);
  EXPECT_TRUE(process.initial_gyro_bias().isApprox(bias, 1e-12));
}

TEST(GyroBiasInitialization, FreshFrameBatchesPreserveTheInitializationWindow)
{
  ImuProcess process;
  auto cloud = std::make_shared<PointCloudXYZI>();
  MeasureGroup measurements;
  const Eigen::Vector3d bias(-0.006, 0.001, -0.0055);
  for (int frame = 0; frame < 6; ++frame) {
    measurements.imu.clear();
    appendImuSamples(measurements, 20, bias, {0, 0, 1}, 1000000000LL + frame * 100000000LL);
    process.Process(measurements, cloud);
    EXPECT_EQ(process.imu_need_init_, frame < 5);
  }
  EXPECT_TRUE(process.initial_gyro_bias().isApprox(bias, 1e-12));
}

TEST(GyroBiasInitialization, RequiresBothEnoughSamplesAndElapsedTime)
{
  ImuProcess process;
  auto cloud = std::make_shared<PointCloudXYZI>();
  MeasureGroup measurements;
  appendImuSamples(measurements, 120, {0, 0, 0}, {0, 0, 1}, 1000000000LL, 1000000LL);
  process.Process(measurements, cloud);
  EXPECT_TRUE(process.imu_need_init_);
  appendImuSamples(measurements, 381, {0, 0, 0}, {0, 0, 1}, 1120000000LL, 1000000LL);
  process.Process(measurements, cloud);
  EXPECT_FALSE(process.imu_need_init_);

  process.Reset();
  measurements.imu.clear();
  appendImuSamples(measurements, 60, {0, 0, 0}, {0, 0, 1}, 1000000000LL, 10000000LL);
  process.Process(measurements, cloud);
  EXPECT_TRUE(process.imu_need_init_);
}

TEST(GyroBiasInitialization, RotationDiscardsPriorWindowAndWaitsForFreshQuietData)
{
  ImuProcess process;
  auto cloud = std::make_shared<PointCloudXYZI>();
  MeasureGroup measurements;
  appendImuSamples(measurements, 60, {-0.01, 0, 0});
  appendImuSamples(measurements, 20, {0.2, 0, 0}, {0, 0, 1}, 1300000000LL);
  const Eigen::Vector3d bias(0.006, 0.001, -0.005);
  appendImuSamples(measurements, 60, bias, {0, 0, 1}, 1400000000LL);
  process.Process(measurements, cloud);
  ASSERT_TRUE(process.imu_need_init_);
  appendImuSamples(measurements, 60, bias, {0, 0, 1}, 1700000000LL);
  process.Process(measurements, cloud);
  ASSERT_FALSE(process.imu_need_init_);
  EXPECT_TRUE(process.initial_gyro_bias().isApprox(bias, 1e-12));
}

TEST(GyroBiasInitialization, AccelerationMotionAndInvalidValuesResetWindow)
{
  for (const Eigen::Vector3d disturbance : {
         Eigen::Vector3d(0, 0, 1.3), Eigen::Vector3d(0.2, 0, 1),
         Eigen::Vector3d(std::numeric_limits<double>::quiet_NaN(), 0, 1)}) {
    ImuProcess process;
    auto cloud = std::make_shared<PointCloudXYZI>();
    MeasureGroup measurements;
    appendImuSamples(measurements, 60, {0, 0, 0});
    appendImuSamples(measurements, 1, {0, 0, 0}, disturbance, 1300000000LL);
    appendImuSamples(measurements, 60, {0, 0, 0}, {0, 0, 1}, 1305000000LL);
    process.Process(measurements, cloud);
    EXPECT_TRUE(process.imu_need_init_);
    appendImuSamples(measurements, 60, {0, 0, 0}, {0, 0, 1}, 1605000000LL);
    process.Process(measurements, cloud);
    EXPECT_FALSE(process.imu_need_init_);
  }
}

TEST(GyroBiasInitialization, LongGapStartsNewWindow)
{
  ImuProcess process;
  auto cloud = std::make_shared<PointCloudXYZI>();
  MeasureGroup measurements;
  appendImuSamples(measurements, 60, {0.01, 0, 0});
  const Eigen::Vector3d bias(-0.006, 0.001, -0.005);
  appendImuSamples(measurements, 60, bias, {0, 0, 1}, 2000000000LL);
  process.Process(measurements, cloud);
  ASSERT_TRUE(process.imu_need_init_);
  appendImuSamples(measurements, 60, bias, {0, 0, 1}, 2300000000LL);
  process.Process(measurements, cloud);
  ASSERT_FALSE(process.imu_need_init_);
  EXPECT_TRUE(process.initial_gyro_bias().isApprox(bias, 1e-12));
}

TEST(GyroBiasInitialization, ResetClearsBiasAndAcceptsRestartedTimestamps)
{
  ImuProcess process;
  auto cloud = std::make_shared<PointCloudXYZI>();
  MeasureGroup measurements;
  appendImuSamples(measurements, 120, {0.01, 0, 0});
  process.Process(measurements, cloud);
  ASSERT_FALSE(process.imu_need_init_);
  process.after_imu_init_ = true;
  process.Reset();
  EXPECT_TRUE(process.imu_need_init_);
  EXPECT_FALSE(process.after_imu_init_);
  EXPECT_TRUE(process.initial_gyro_bias().isZero());
  measurements.imu.clear();
  const Eigen::Vector3d bias(-0.005, 0.002, 0);
  appendImuSamples(measurements, 120, bias, {0, 0, 1}, 0);
  process.Process(measurements, cloud);
  ASSERT_FALSE(process.imu_need_init_);
  EXPECT_TRUE(process.initial_gyro_bias().isApprox(bias, 1e-12));
}

TEST(GyroBiasInitialization, DisabledImuCopiesCloudWithoutSamplesOrValidImuScale)
{
  ImuProcess process;
  process.imu_en = false;
  process.acc_norm = 0.0;
  MeasureGroup measurements;
  measurements.lidar->push_back(PointType());
  auto cloud = std::make_shared<PointCloudXYZI>();
  EXPECT_NO_THROW(process.Process(measurements, cloud));
  EXPECT_EQ(cloud->size(), 1U);
  EXPECT_TRUE(process.initial_gyro_bias().isZero());
}

}  // namespace
