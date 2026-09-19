#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <traj_utils/polynomial_traj.h>

TEST(PolynomialTraj, PiecewiseLinearReferenceStaysInsideWaypointBounds)
{
  Eigen::MatrixXd points(3, 58);
  for (int i = 0; i < points.cols(); ++i)
    points.col(i) << 0.1 * i, 0.4 * std::sin(0.35 * i), 0.0;

  auto trajectory = PolynomialTraj::piecewiseLinearTraj(points, 0.35);
  trajectory.init();

  const double duration = trajectory.getTimeSum();
  ASSERT_GT(duration, 0.0);
  for (double t = 0.0; t <= duration; t += 0.02)
  {
    const Eigen::Vector3d position = trajectory.evaluate(std::min(t, duration));
    EXPECT_TRUE(position.allFinite());
    EXPECT_GE(position.x(), points.row(0).minCoeff() - 1e-9);
    EXPECT_LE(position.x(), points.row(0).maxCoeff() + 1e-9);
    EXPECT_GE(position.y(), points.row(1).minCoeff() - 1e-9);
    EXPECT_LE(position.y(), points.row(1).maxCoeff() + 1e-9);
    EXPECT_LE(trajectory.evaluateVel(std::min(t, duration)).norm(), 0.35 + 1e-9);
  }

  EXPECT_TRUE(trajectory.evaluate(0.0).isApprox(points.col(0), 1e-9));
  EXPECT_TRUE(trajectory.evaluate(duration).isApprox(points.col(points.cols() - 1), 1e-9));
}

TEST(PolynomialTraj, MeanVelocityReturnsStraightLineAverageSpeed)
{
  Eigen::MatrixXd points(3, 2);
  points.col(0) << 0.0, 0.0, 0.0;
  points.col(1) << 1.0, 0.0, 0.0;

  auto trajectory = PolynomialTraj::piecewiseLinearTraj(points, 0.5);
  trajectory.init();
  trajectory.getTraj();
  const double sampled_length = trajectory.getLength();

  EXPECT_NEAR(trajectory.getMeanVel(), sampled_length / trajectory.getTimeSum(), 1e-12);
  EXPECT_NEAR(trajectory.getMeanVel(), 0.5, 1e-2);
}
