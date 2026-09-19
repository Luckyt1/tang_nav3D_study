#include "relocalization/pose_tracking.hpp"

#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>

#include <gtest/gtest.h>

namespace {

using relocalization_processing::PoseTracking;
using relocalization_processing::TrackingConfig;

constexpr std::int64_t kSecond = 1000000000;
constexpr std::int64_t kSteady = 100 * kSecond;

Eigen::Isometry3d pose(double x, double y, double z, double yaw, double pitch = 0.0) {
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.translation() = Eigen::Vector3d(x, y, z);
  result.linear() =
    (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
     Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())).toRotationMatrix();
  return result;
}

const Eigen::Isometry3d kMapFromOdom = pose(5.0, -2.0, 0.6, 1.1, -0.2);
const Eigen::Isometry3d kOdomFromSensor = pose(1.0, 2.0, 0.3, -0.5, 0.3);

TEST(PoseTracking, DoesNotPublishBeforeFirstFix) {
  PoseTracking tracker;
  const auto result = tracker.updateOdometry(kOdomFromSensor, 10 * kSecond, kSteady);
  EXPECT_FALSE(result.valid);
  EXPECT_FALSE(result.publish);
  EXPECT_FALSE(tracker.unavailable(kSteady).valid);
}

TEST(PoseTracking, CombinesFixedMapCorrectionWithEachNewOdometryPose) {
  PoseTracking tracker;
  ASSERT_TRUE(tracker.updateFix(kMapFromOdom, 10 * kSecond, kSteady, kSteady, tracker.generation()));
  const auto first = tracker.updateOdometry(kOdomFromSensor, 10 * kSecond, kSteady);
  ASSERT_TRUE(first.valid);
  EXPECT_TRUE(first.publish);
  EXPECT_EQ(first.stamp_ns, 10 * kSecond);
  EXPECT_EQ(first.fix_stamp_ns, 10 * kSecond);
  EXPECT_TRUE(first.map_from_odom.matrix().isApprox(kMapFromOdom.matrix(), 1e-12));
  EXPECT_TRUE(first.map_from_sensor.matrix().isApprox(
    (kMapFromOdom * kOdomFromSensor).matrix(), 1e-12));

  const auto moved_odom = pose(1.4, 2.3, 0.7, -0.2, 0.4);
  const auto moved = tracker.updateOdometry(moved_odom, 11 * kSecond, kSteady + kSecond / 2);
  ASSERT_TRUE(moved.valid);
  EXPECT_TRUE(moved.publish);
  EXPECT_EQ(moved.stamp_ns, 11 * kSecond);
  EXPECT_EQ(moved.fix_stamp_ns, 10 * kSecond);
  EXPECT_NEAR(moved.fix_age, 1.0, 1e-12);
  EXPECT_TRUE(moved.map_from_sensor.matrix().isApprox(
    (kMapFromOdom * moved_odom).matrix(), 1e-12));
  EXPECT_FALSE(moved.map_from_sensor.matrix().isApprox(first.map_from_sensor.matrix(), 1e-3));
}

TEST(PoseTracking, PublishesEachOdometryTimestampOnceEvenAfterAnUpdatedFix) {
  PoseTracking tracker;
  ASSERT_TRUE(tracker.updateFix(kMapFromOdom, 9 * kSecond, kSteady, kSteady, tracker.generation()));
  ASSERT_TRUE(tracker.updateOdometry(kOdomFromSensor, 10 * kSecond, kSteady).publish);
  const auto duplicate = tracker.updateOdometry(
    kOdomFromSensor, 10 * kSecond, kSteady + kSecond / 10);
  EXPECT_TRUE(duplicate.valid);
  EXPECT_FALSE(duplicate.publish);
  EXPECT_EQ(duplicate.stamp_ns, 10 * kSecond);

  ASSERT_TRUE(tracker.updateFix(pose(5.1, -2.1, 0.6, 1.12), 10 * kSecond,
                                kSteady + kSecond / 5,
                                kSteady + kSecond / 5, tracker.generation()));
  const auto new_fix_same_odom = tracker.updateOdometry(
    kOdomFromSensor, 10 * kSecond, kSteady + kSecond / 4);
  EXPECT_TRUE(new_fix_same_odom.valid);
  EXPECT_FALSE(new_fix_same_odom.publish);
  EXPECT_EQ(new_fix_same_odom.stamp_ns, 10 * kSecond);
  EXPECT_TRUE(tracker.updateOdometry(
    kOdomFromSensor, 11 * kSecond, kSteady + kSecond / 2).publish);
}

TEST(PoseTracking, DuplicateOdometryDoesNotRefreshItsOneSecondLifetime) {
  PoseTracking tracker;
  ASSERT_TRUE(tracker.updateFix(kMapFromOdom, 10 * kSecond, kSteady, kSteady, tracker.generation()));
  ASSERT_TRUE(tracker.updateOdometry(kOdomFromSensor, 10 * kSecond, kSteady).valid);
  EXPECT_TRUE(tracker.updateOdometry(
    kOdomFromSensor, 10 * kSecond, kSteady + 9 * kSecond / 10).valid);

  const auto stale = tracker.updateOdometry(
    kOdomFromSensor, 10 * kSecond, kSteady + 11 * kSecond / 10);
  EXPECT_FALSE(stale.valid);
  EXPECT_FALSE(stale.publish);
  EXPECT_TRUE(tracker.updateOdometry(
    kOdomFromSensor, 10 * kSecond + kSecond / 10, kSteady + 12 * kSecond / 10).valid);
}

TEST(PoseTracking, FixExpiresWhenEitherDataAgeOrSteadyAgeExceedsFiveSeconds) {
  PoseTracking data_age;
  ASSERT_TRUE(data_age.updateFix(kMapFromOdom, 10 * kSecond, kSteady, kSteady, data_age.generation()));
  EXPECT_TRUE(data_age.updateOdometry(
    kOdomFromSensor, 14 * kSecond + 9 * kSecond / 10, kSteady + kSecond / 10).valid);
  const auto old_data = data_age.updateOdometry(
    kOdomFromSensor, 15 * kSecond + kSecond / 10, kSteady + kSecond / 5);
  EXPECT_FALSE(old_data.valid);
  EXPECT_FALSE(old_data.publish);
  EXPECT_NEAR(old_data.fix_age, 5.1, 1e-12);

  PoseTracking steady_age;
  ASSERT_TRUE(steady_age.updateFix(kMapFromOdom, 10 * kSecond, kSteady, kSteady,
                                   steady_age.generation()));
  EXPECT_TRUE(steady_age.updateOdometry(
    kOdomFromSensor, 10 * kSecond + kSecond / 10, kSteady + 49 * kSecond / 10).valid);
  const auto old_steady = steady_age.updateOdometry(
    kOdomFromSensor, 10 * kSecond + kSecond / 5, kSteady + 51 * kSecond / 10);
  EXPECT_FALSE(old_steady.valid);
  EXPECT_FALSE(old_steady.publish);
  EXPECT_NEAR(old_steady.fix_age, 5.1, 1e-12);
}

TEST(PoseTracking, OldAndDuplicateFixesCannotExtendValidityButANewFixRestoresIt) {
  PoseTracking tracker;
  const auto generation = tracker.generation();
  ASSERT_TRUE(tracker.updateFix(kMapFromOdom, 10 * kSecond, kSteady, kSteady, generation));
  ASSERT_TRUE(tracker.updateOdometry(kOdomFromSensor, 10 * kSecond, kSteady).valid);
  EXPECT_FALSE(tracker.updateFix(kMapFromOdom, 10 * kSecond,
                                 kSteady + 4 * kSecond, kSteady + 4 * kSecond, generation));
  EXPECT_FALSE(tracker.updateFix(kMapFromOdom, 9 * kSecond,
                                 kSteady + 4 * kSecond, kSteady + 4 * kSecond, generation));
  EXPECT_FALSE(tracker.updateOdometry(
    kOdomFromSensor, 10 * kSecond + kSecond / 10, kSteady + 51 * kSecond / 10).valid);

  ASSERT_TRUE(tracker.updateFix(kMapFromOdom, 11 * kSecond,
                                kSteady + 52 * kSecond / 10,
                                kSteady + 52 * kSecond / 10, generation));
  const auto recovered = tracker.updateOdometry(
    kOdomFromSensor, 11 * kSecond, kSteady + 53 * kSecond / 10);
  EXPECT_TRUE(recovered.valid);
  EXPECT_TRUE(recovered.publish);
  EXPECT_EQ(recovered.fix_stamp_ns, 11 * kSecond);
}

TEST(PoseTracking, OdometryRollbackInvalidatesFixAndRejectsPreviousGenerationResults) {
  PoseTracking tracker;
  const auto original_generation = tracker.generation();
  ASSERT_TRUE(tracker.updateFix(kMapFromOdom, 20 * kSecond, kSteady, kSteady, original_generation));
  ASSERT_TRUE(tracker.updateOdometry(kOdomFromSensor, 20 * kSecond, kSteady).valid);
  const auto reset = tracker.updateOdometry(
    kOdomFromSensor, 19 * kSecond, kSteady + kSecond / 10);
  EXPECT_FALSE(reset.valid);
  EXPECT_FALSE(reset.publish);
  EXPECT_EQ(tracker.generation(), original_generation + 1);
  EXPECT_FALSE(tracker.updateFix(kMapFromOdom, 20 * kSecond,
                                 kSteady + kSecond / 5,
                                 kSteady + kSecond / 5, original_generation));
  EXPECT_FALSE(tracker.updateOdometry(
    kOdomFromSensor, 20 * kSecond, kSteady + 3 * kSecond / 10).valid);

  ASSERT_TRUE(tracker.updateFix(kMapFromOdom, 20 * kSecond,
                                kSteady + 4 * kSecond / 10,
                                kSteady + 4 * kSecond / 10, tracker.generation()));
  const auto recovered = tracker.updateOdometry(
    kOdomFromSensor, 20 * kSecond + kSecond / 10, kSteady + kSecond / 2);
  EXPECT_TRUE(recovered.valid);
  EXPECT_TRUE(recovered.publish);
}

TEST(PoseTracking, FutureFixWaitsUntilOdometryReachesItsTimestamp) {
  PoseTracking tracker;
  EXPECT_FALSE(tracker.updateOdometry(kOdomFromSensor, 10 * kSecond, kSteady).valid);
  ASSERT_TRUE(tracker.updateFix(kMapFromOdom, 12 * kSecond,
                                kSteady + kSecond / 10,
                                kSteady + kSecond / 10, tracker.generation()));
  const auto before_fix = tracker.updateOdometry(
    kOdomFromSensor, 11 * kSecond, kSteady + kSecond / 5);
  EXPECT_FALSE(before_fix.valid);
  EXPECT_FALSE(before_fix.publish);
  const auto caught_up = tracker.updateOdometry(
    kOdomFromSensor, 12 * kSecond, kSteady + 3 * kSecond / 10);
  EXPECT_TRUE(caught_up.valid);
  EXPECT_TRUE(caught_up.publish);
  EXPECT_EQ(caught_up.stamp_ns, 12 * kSecond);
}

TEST(PoseTracking, RejectsLateComputationAndMeasuresAcceptedFixLifetimeFromQueryStart) {
  PoseTracking late;
  EXPECT_FALSE(late.updateFix(kMapFromOdom, 10 * kSecond, kSteady,
                              kSteady + 51 * kSecond / 10, late.generation()));
  EXPECT_FALSE(late.updateOdometry(
    kOdomFromSensor, 10 * kSecond, kSteady + 52 * kSecond / 10).valid);

  PoseTracking slow;
  ASSERT_TRUE(slow.updateFix(kMapFromOdom, 10 * kSecond, kSteady,
                             kSteady + 4 * kSecond, slow.generation()));
  const auto accepted = slow.updateOdometry(
    kOdomFromSensor, 10 * kSecond, kSteady + 4 * kSecond);
  EXPECT_TRUE(accepted.valid);
  EXPECT_NEAR(accepted.fix_age, 4.0, 1e-12);
  const auto expired = slow.updateOdometry(
    kOdomFromSensor, 10 * kSecond + kSecond / 10, kSteady + 51 * kSecond / 10);
  EXPECT_FALSE(expired.valid);
  EXPECT_FALSE(expired.publish);
  EXPECT_NEAR(expired.fix_age, 5.1, 1e-12);
}

TEST(PoseTracking, MissingTransformDoesNotRefreshOdometryOrFixFreshness) {
  PoseTracking tracker;
  ASSERT_TRUE(tracker.updateFix(kMapFromOdom, 10 * kSecond, kSteady, kSteady, tracker.generation()));
  ASSERT_TRUE(tracker.updateOdometry(kOdomFromSensor, 10 * kSecond, kSteady).valid);
  const auto missing = tracker.unavailable(kSteady + 9 * kSecond / 10);
  EXPECT_FALSE(missing.valid);
  EXPECT_FALSE(missing.publish);
  EXPECT_FALSE(tracker.updateOdometry(
    kOdomFromSensor, 10 * kSecond, kSteady + 11 * kSecond / 10).valid);
  EXPECT_TRUE(tracker.updateOdometry(
    kOdomFromSensor, 11 * kSecond, kSteady + 12 * kSecond / 10).valid);
  EXPECT_FALSE(tracker.unavailable(kSteady + 49 * kSecond / 10).valid);
  EXPECT_FALSE(tracker.updateOdometry(
    kOdomFromSensor, 11 * kSecond + kSecond / 10, kSteady + 51 * kSecond / 10).valid);
}

TEST(PoseTracking, MissingTransformPreservesLastOdometryStampAndDataAge) {
  PoseTracking tracker;
  ASSERT_TRUE(tracker.updateFix(kMapFromOdom, 10 * kSecond, kSteady, kSteady, 0));
  ASSERT_TRUE(tracker.updateOdometry(kOdomFromSensor, 14 * kSecond, kSteady).valid);
  const auto missing = tracker.unavailable(kSteady + kSecond);
  EXPECT_FALSE(missing.valid);
  EXPECT_EQ(missing.stamp_ns, 14 * kSecond);
  EXPECT_EQ(missing.fix_stamp_ns, 10 * kSecond);
  EXPECT_DOUBLE_EQ(missing.fix_age, 4.0);
}

TEST(PoseTracking, RejectsInvalidConfiguration) {
  for (const double invalid : {0.0, -1.0, std::numeric_limits<double>::infinity(),
                               std::numeric_limits<double>::quiet_NaN()}) {
    TrackingConfig config;
    config.fix_timeout = invalid;
    EXPECT_THROW({ PoseTracking tracker(config); }, std::invalid_argument);
    config = TrackingConfig{};
    config.odom_timeout = invalid;
    EXPECT_THROW({ PoseTracking tracker(config); }, std::invalid_argument);
  }
}

TEST(PoseTracking, RejectsInvalidPosesAndTimestampsWithoutInstallingAFix) {
  PoseTracking tracker;
  auto invalid = kMapFromOdom;
  invalid.translation().x() = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(tracker.updateFix(invalid, 10 * kSecond, kSteady, kSteady, tracker.generation()),
               std::invalid_argument);
  invalid = kMapFromOdom;
  invalid.linear().col(0) *= -1.0;
  EXPECT_THROW(tracker.updateFix(invalid, 10 * kSecond, kSteady, kSteady, tracker.generation()),
               std::invalid_argument);
  invalid = kOdomFromSensor;
  invalid.linear() *= 2.0;
  EXPECT_THROW(tracker.updateOdometry(invalid, 10 * kSecond, kSteady), std::invalid_argument);
  invalid = kOdomFromSensor;
  invalid.matrix()(3, 0) = 0.1;
  EXPECT_THROW(tracker.updateOdometry(invalid, 10 * kSecond, kSteady), std::invalid_argument);
  EXPECT_THROW(tracker.updateFix(kMapFromOdom, 0, kSteady, kSteady, tracker.generation()),
               std::invalid_argument);
  EXPECT_THROW(tracker.updateOdometry(kOdomFromSensor, -1, kSteady), std::invalid_argument);
  EXPECT_THROW(tracker.updateFix(kMapFromOdom, 10 * kSecond, -1, -1, tracker.generation()),
               std::invalid_argument);
  EXPECT_THROW(tracker.updateFix(kMapFromOdom, 10 * kSecond, -1, kSteady,
                                 tracker.generation()), std::invalid_argument);
  EXPECT_THROW(tracker.updateFix(kMapFromOdom, 10 * kSecond, kSteady + 1, kSteady,
                                 tracker.generation()), std::invalid_argument);
  EXPECT_THROW(tracker.unavailable(-1), std::invalid_argument);
  EXPECT_FALSE(tracker.updateOdometry(kOdomFromSensor, 10 * kSecond, kSteady).valid);
  EXPECT_EQ(tracker.generation(), 0U);
}

}  // namespace
