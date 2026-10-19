#include <gtest/gtest.h>

#include "point_lio/filter_diagnostics.hpp"
#include "point_lio/manifold_state.hpp"
#include "point_lio/narrow_esikf.hpp"

#include <limits>
#include <sstream>
#include <string>

namespace
{
using Filter = point_lio::esikf::NarrowESIKF<point_lio::OutputState, 30, point_lio::Input>;

void imuMeasurement(
  point_lio::OutputState &, point_lio::esikf::MeasurementData<double> & measurement)
{
  measurement.z_IMU.setConstant(0.3);
  measurement.R_IMU.setOnes();
}

std::size_t fields(const std::string & row)
{
  return 1 + std::count(row.begin(), row.end(), ',');
}
}  // namespace

TEST(FilterDiagnostics, MagnitudesAreNotSignedOrNetCorrections)
{
  point_lio::MagnitudeStatistics statistics;
  EXPECT_TRUE(std::isnan(statistics.rms()));
  statistics.add(-3.0);
  statistics.add(4.0);
  statistics.add(std::numeric_limits<double>::quiet_NaN());
  EXPECT_EQ(statistics.count, 2U);
  EXPECT_EQ(statistics.invalid, 1U);
  EXPECT_DOUBLE_EQ(statistics.sum, 7.0);
  EXPECT_DOUBLE_EQ(statistics.rms(), std::sqrt(12.5));
  EXPECT_DOUBLE_EQ(statistics.maximum, 4.0);
}

TEST(FilterDiagnostics, OpposingUpdatesRemainVisibleAndFailuresAreSeparate)
{
  point_lio::FrameDiagnostics diagnostics;
  point_lio::OutputState state;
  for (double correction : {0.1, -0.1}) {
    ASSERT_TRUE(point_lio::observeUpdate(
      &diagnostics, point_lio::UpdateKind::Imu, state, [&] {
        state.pos.x() += correction;
        state.rot.retract(Eigen::Vector3d(0.0, 0.0, correction));
        return true;
      }));
  }
  EXPECT_FALSE(point_lio::observeUpdate(
    &diagnostics, point_lio::UpdateKind::Imu, state, [] { return false; }));
  EXPECT_EQ(diagnostics.imu.attempts, 3U);
  EXPECT_EQ(diagnostics.imu.failures, 1U);
  EXPECT_EQ(diagnostics.imu.position.count, 2U);
  EXPECT_NEAR(diagnostics.imu.position.sum, 0.2, 1e-12);
  EXPECT_NEAR(diagnostics.imu.rotation.rms(), 0.1, 1e-12);
  EXPECT_NEAR(state.pos.norm(), 0.0, 1e-12);
  EXPECT_NEAR(state.rot.quaternion().angularDistance(Eigen::Quaterniond::Identity()), 0.0, 1e-12);
  EXPECT_EQ(diagnostics.lidar.attempts, 0U);
}

TEST(FilterDiagnostics, LidarIterationsCountOnlyTheFirstResidualPerUpdate)
{
  point_lio::FrameDiagnostics diagnostics;
  point_lio::InputState state;
  for (int update = 0; update < 2; ++update) {
    ASSERT_TRUE(point_lio::observeUpdate(
      &diagnostics, point_lio::UpdateKind::Lidar, state, [&] {
        diagnostics.recordLidarResiduals(Eigen::Vector2d(-0.03, 0.04));
        diagnostics.recordLidarResiduals(Eigen::Vector2d(0.001, 0.002));
        return true;
      }));
  }
  EXPECT_EQ(diagnostics.lidar_residual.count, 4U);
  EXPECT_NEAR(diagnostics.lidar_residual.rms(), std::sqrt(0.00125), 1e-12);
  EXPECT_DOUBLE_EQ(diagnostics.lidar_residual.maximum, 0.04);
  EXPECT_EQ(diagnostics.lidar.attempts, 2U);
}

TEST(FilterDiagnostics, CountsOnlyActuallyRequestedNegativePropagation)
{
  point_lio::FrameDiagnostics diagnostics;
  diagnostics.recordPrediction(-0.001, true, false);
  diagnostics.recordPrediction(-0.002, false, true);
  diagnostics.recordPrediction(0.0, true, true);
  diagnostics.recordPrediction(0.01, true, true);
  EXPECT_EQ(diagnostics.negative_state_dt, 1U);
  EXPECT_EQ(diagnostics.negative_covariance_dt, 1U);
  diagnostics = point_lio::FrameDiagnostics{};
  EXPECT_EQ(diagnostics.negative_state_dt, 0U);
  EXPECT_EQ(diagnostics.negative_covariance_dt, 0U);
}

TEST(FilterDiagnostics, DisabledDiagnosticsNeverReadPose)
{
  struct UnreadableRotation
  {
    Eigen::Quaterniond quaternion() const
    {
      ADD_FAILURE() << "Disabled diagnostics must not snapshot pose";
      return Eigen::Quaterniond::Identity();
    }
  };
  struct State
  {
    Eigen::Vector3d pos = Eigen::Vector3d::Zero();
    UnreadableRotation rot;
  } state;
  int calls = 0;
  EXPECT_TRUE(point_lio::observeUpdate(
    nullptr, point_lio::UpdateKind::Lidar, state, [&] { ++calls; return true; }));
  EXPECT_EQ(calls, 1);
}

TEST(FilterDiagnostics, CallbackTotalsSurviveFrameResetAndRevealMissingIntervals)
{
  point_lio::FrameDiagnostics diagnostics;
  diagnostics.imu_callbacks.record(1.0);
  diagnostics.imu_callbacks.record(1.005);
  ++diagnostics.imu_callbacks.accepted;
  diagnostics.imu.attempts = 2;
  diagnostics.resetFrame();
  EXPECT_EQ(diagnostics.imu.attempts, 0U);
  EXPECT_EQ(diagnostics.imu_callbacks.received, 2U);
  EXPECT_EQ(diagnostics.imu_callbacks.accepted, 1U);
  diagnostics.imu_callbacks.record(1.025);
  diagnostics.imu_callbacks.record(1.025);
  diagnostics.imu_callbacks.record(1.020);
  EXPECT_EQ(diagnostics.imu_callbacks.gaps_over_7p5ms, 1U);
  EXPECT_EQ(diagnostics.imu_callbacks.duplicates, 1U);
  EXPECT_EQ(diagnostics.imu_callbacks.backwards, 1U);
  EXPECT_NEAR(diagnostics.imu_callbacks.maximum_gap, 0.020, 1e-12);
  EXPECT_DOUBLE_EQ(diagnostics.imu_callbacks.first_stamp, 1.0);
  EXPECT_DOUBLE_EQ(diagnostics.imu_callbacks.last_stamp, 1.020);
}

TEST(FilterDiagnostics, CsvMatchesSchemaAndSupportsBothStateModes)
{
  point_lio::FrameDiagnostics diagnostics;
  std::ostringstream output;
  point_lio::FrameDiagnostics::writeHeader(output);
  diagnostics.writeRow(output, 1.0, 1.1, 1.09, false, 42, point_lio::OutputState{});
  diagnostics.writeRow(output, 2.0, 2.1, 2.09, true, 43, point_lio::InputState{});
  std::istringstream input(output.str());
  std::string header, row;
  std::getline(input, header);
  ASSERT_NE(header.find("imu_gyro_residual_norm_rms_rad_s"), std::string::npos);
  ASSERT_NE(header.find("lidar_prior_residual_rms_m"), std::string::npos);
  for (int mode = 0; mode < 2; ++mode) {
    ASSERT_TRUE(static_cast<bool>(std::getline(input, row)));
    EXPECT_EQ(fields(header), fields(row));
    EXPECT_NE(row.find("nan"), std::string::npos);  // No samples are not a zero residual.
  }
}

TEST(FilterDiagnostics, ImuReturnReportsMissingModelWithoutChangingState)
{
  Filter filter;
  filter.x_.pos = Eigen::Vector3d(1.0, 2.0, 3.0);
  const auto covariance = filter.P_;
  EXPECT_FALSE(filter.update_iterated_dyn_share_IMU());
  EXPECT_TRUE(filter.P_.isApprox(covariance, 0.0));
  EXPECT_TRUE(filter.x_.pos.isApprox(Eigen::Vector3d(1.0, 2.0, 3.0), 0.0));
}

TEST(FilterDiagnostics, ImuReturnReportsFailedSolveWithoutChangingState)
{
  Filter filter;
  filter.init_dyn_share_modified_3h(nullptr, nullptr, nullptr, imuMeasurement);
  filter.P_(0, 0) = std::numeric_limits<double>::quiet_NaN();
  const auto previous = filter.x_;
  EXPECT_FALSE(filter.update_iterated_dyn_share_IMU());
  EXPECT_TRUE(filter.x_.pos.isApprox(previous.pos, 0.0));
  EXPECT_TRUE(filter.x_.omg.isApprox(previous.omg, 0.0));
  EXPECT_TRUE(filter.x_.bg.isApprox(previous.bg, 0.0));
  EXPECT_TRUE(std::isnan(filter.P_(0, 0)));
}

TEST(FilterDiagnostics, DiagnosticsPreserveSuccessfulImuUpdateAndCovariance)
{
  Filter plain, observed;
  plain.init_dyn_share_modified_3h(nullptr, nullptr, nullptr, imuMeasurement);
  observed.init_dyn_share_modified_3h(nullptr, nullptr, nullptr, imuMeasurement);
  point_lio::FrameDiagnostics diagnostics;
  ASSERT_TRUE(plain.update_iterated_dyn_share_IMU());
  ASSERT_TRUE(point_lio::observeUpdate(
    &diagnostics, point_lio::UpdateKind::Imu, observed.x_,
    [&] { return observed.update_iterated_dyn_share_IMU(); }));
  EXPECT_TRUE(plain.P_.isApprox(observed.P_, 0.0));
  EXPECT_TRUE(plain.x_.omg.isApprox(observed.x_.omg, 0.0));
  EXPECT_TRUE(plain.x_.bg.isApprox(observed.x_.bg, 0.0));
  EXPECT_TRUE(plain.x_.omg.isApprox(Eigen::Vector3d::Constant(0.1), 1e-12));
  EXPECT_TRUE(plain.x_.bg.isApprox(Eigen::Vector3d::Constant(0.1), 1e-12));
  EXPECT_EQ(diagnostics.imu.attempts, 1U);
  EXPECT_EQ(diagnostics.imu.failures, 0U);
}
