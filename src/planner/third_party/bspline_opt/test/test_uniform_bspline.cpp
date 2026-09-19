#include <gtest/gtest.h>

#include <algorithm>
#include <Eigen/Core>
#include <bspline_opt/uniform_bspline.h>

TEST(UniformBspline, EvaluatesLinearControlPoints)
{
  Eigen::MatrixXd points = Eigen::MatrixXd::Zero(3, 6);
  for (int i = 0; i < points.cols(); ++i) points(0, i) = static_cast<double>(i);

  scan_planner::UniformBspline spline(points, 3, 1.0);
  EXPECT_NEAR(spline.evaluateDeBoorT(0.0).x(), 1.0, 1e-9);
  EXPECT_NEAR(spline.evaluateDeBoorT(2.0).x(), 3.0, 1e-9);
  EXPECT_NEAR(spline.evaluateDeBoorT(2.0).y(), 0.0, 1e-9);
}

TEST(UniformBspline, DerivativeMatchesLinearSlope)
{
  Eigen::MatrixXd points = Eigen::MatrixXd::Zero(3, 6);
  for (int i = 0; i < points.cols(); ++i) points(0, i) = static_cast<double>(i);

  auto derivative = scan_planner::UniformBspline(points, 3, 0.5).getDerivative();
  const Eigen::Vector3d velocity = derivative.evaluateDeBoorT(0.75);
  EXPECT_NEAR(velocity.x(), 2.0, 1e-9);
  EXPECT_NEAR(velocity.y(), 0.0, 1e-9);
  EXPECT_NEAR(velocity.z(), 0.0, 1e-9);
}

TEST(UniformBspline, FeasibilityUsesVelocityNormAcrossAxes)
{
  Eigen::MatrixXd points = Eigen::MatrixXd::Zero(3, 6);
  for (int i = 0; i < points.cols(); ++i)
    points.col(i) = Eigen::Vector3d(0.3, 0.3, 0.0) * i;
  scan_planner::UniformBspline spline(points, 3, 1.0);
  spline.setPhysicalLimits(0.35, 0.5, 0.0);
  double ratio = 0.0;
  EXPECT_FALSE(spline.checkFeasibility(ratio, false));
  EXPECT_GT(ratio, 1.0);
}

TEST(UniformBspline, ParameterizationPreservesBothEndpointStates)
{
  std::vector<Eigen::Vector3d> samples;
  for (int i = 0; i < 7; ++i)
    samples.emplace_back(0.03 * i * i, 0.02 * (i % 2), 0.05);
  const std::vector<Eigen::Vector3d> derivatives = {
      {0.1, 0.02, 0.0}, {0.35, 0.0, 0.0}, {0.03, 0.0, 0.0}, {0.0, 0.0, 0.0}};
  Eigen::MatrixXd controls;
  scan_planner::UniformBspline::parameterizeToBspline(0.5, samples, derivatives, controls);
  scan_planner::UniformBspline spline(controls, 3, 0.5);
  auto velocity = spline.getDerivative();
  auto acceleration = velocity.getDerivative();
  for (int end = 0; end < 2; ++end)
  {
    const double t = end == 0 ? 0.0 : spline.getTimeSum();
    const auto &position = end == 0 ? samples.front() : samples.back();
    EXPECT_LT((spline.evaluateDeBoorT(t) - position).norm(), 1e-9);
    EXPECT_LT((velocity.evaluateDeBoorT(t) - derivatives[end]).norm(), 1e-9);
    EXPECT_LT((acceleration.evaluateDeBoorT(t) - derivatives[end + 2]).norm(), 1e-9);
  }
}

TEST(UniformBspline, CollisionCheckStepUsesMotionInsteadOfEndpointDistance)
{
  Eigen::MatrixXd points = Eigen::MatrixXd::Zero(3, 8);
  points.row(0) << 0.0, 0.0, 5.0, -5.0, -5.0, 5.0, 0.0, 0.0;

  scan_planner::UniformBspline spline(points, 3, 1.0);
  const double duration = spline.getTimeSum();
  const double endpoint_distance =
      (spline.evaluateDeBoorT(duration) - spline.evaluateDeBoorT(0.0)).norm();
  const double old_step =
      std::max(0.01, duration / std::max(1.0, endpoint_distance / 0.5));
  const double new_step = spline.getCollisionCheckTimeStep(0.5);

  EXPECT_NEAR(endpoint_distance, 0.0, 1e-9);
  EXPECT_NEAR(old_step, duration, 1e-9);
  EXPECT_LT(new_step, 0.05);
}

TEST(UniformBspline, CollisionCheckStepHandlesShiftedKnots)
{
  Eigen::MatrixXd points = Eigen::MatrixXd::Zero(3, 8);
  points.row(0) << 0.0, 0.0, 5.0, -5.0, -5.0, 5.0, 0.0, 0.0;

  scan_planner::UniformBspline spline(points, 3, 1.0);
  Eigen::VectorXd shifted_knots = spline.getKnot().array() + 10.0;
  spline.setKnot(shifted_knots);

  EXPECT_NEAR(spline.getTimeSum(), 5.0, 1e-9);
  EXPECT_LT(spline.getCollisionCheckTimeStep(0.5), 0.05);
}
