#include <gtest/gtest.h>
#include <Eigen/Eigen>
#include <algorithm>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <iostream>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <vector>
#include <thread>
#include <functional>
#include <limits>
#include <stdexcept>
#include <visualization_msgs/msg/marker.hpp>
#include <cv_bridge/cv_bridge.h>
#include <random>
#include <queue>
#include <tuple>
#include <unordered_map>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>
#define private public
#include <plan_env/grid_map.h>
#undef private
#include <bspline_opt/bspline_optimizer.h>
#include <planner/msg/bspline.hpp>
#include <planner/msg/data_disp.hpp>
#include <traj_utils/planning_visualization.h>
#include <plan_manage/plan_container.hpp>

// Exercise the real FSM callbacks and publication gates without hardware.
#define private public
#include <plan_manage/scan_replan_fsm.h>
#undef private

#define private public
#include "../third_party/planner_runtime/src/closed_loop_controller.cpp"
#undef private

namespace scan_planner
{
class PlanningSafety : public testing::Test
{
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  void SetUp() override
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides({
      {"fsm.navi_mode", 3}, {"fsm.use_odom_z", true},
      {"fsm.enable_completion_driven_replan", false},
      {"fsm.thresh_replan", 1.0}, {"fsm.thresh_no_replan", 0.1},
      {"manager.max_vel", 0.35}, {"manager.max_acc", 0.5},
      {"manager.control_points_distance", 0.2}, {"manager.planning_horizon", 0.5},
      {"optimization.max_vel", 0.35}, {"optimization.max_acc", 0.5},
      {"optimization.lambda_smooth", 1.0}, {"optimization.lambda_collision", 1.0},
      {"optimization.lambda_feasibility", 0.1}, {"optimization.lambda_fitness", 1.0},
      {"optimization.dist0", 0.2}, {"fsm.planning_horizon", 0.5},
      {"grid_map.need_extrinsic", false},
      {"grid_map.resolution", 0.1}, {"grid_map.sliding_map_size_x", 2.0},
      {"grid_map.sliding_map_size_y", 2.0}, {"grid_map.sliding_map_size_z", 2.0},
      {"grid_map.double_cylinder_radius", 0.1},
      {"grid_map.obstacles_inflation_z_up", 0.1},
      {"grid_map.obstacles_inflation_z_down", 0.1},
      {"grid_map.ground_height", -1.0}, {"grid_map.map_sliding_en", false},
      {"grid_map.p_hit", 0.85}, {"grid_map.p_miss", 0.30},
      {"grid_map.p_min", 0.12}, {"grid_map.p_max", 0.98},
      {"grid_map.p_occ", 0.80}, {"grid_map.max_ray_length", 1.0}});
    node = std::make_shared<rclcpp::Node>("planning_safety_test", options);
    fsm.init(node.get());
    auto odom = std::make_shared<nav_msgs::msg::Odometry>();
    odom->pose.pose.position.x = 0.05;
    odom->pose.pose.position.y = 0.05;
    odom->pose.pose.position.z = 0.05;
    odom->pose.pose.orientation.w = 1.0;
    fsm.odometryCallback(odom);
    fsm.end_pt_ = Eigen::Vector3d(0.75, 0.05, 0.05);
  }

  UniformBspline movingSpline(double vx, double vy = 0.0)
  {
    Eigen::MatrixXd points = Eigen::MatrixXd::Zero(3, 6);
    for (int i = 0; i < points.cols(); ++i)
      points.col(i) = fsm.odom_pos_ + Eigen::Vector3d(vx, vy, 0.0) * (i - 1);
    return UniformBspline(points, 3, 1.0);
  }

  planner::msg::Bspline makeTrajectoryMsg(UniformBspline spline, int64_t id = 1)
  {
    planner::msg::Bspline trajectory;
    trajectory.order = 3;
    trajectory.traj_id = id;
    auto points = spline.getControlPoint();
    for (int i = 0; i < points.cols(); ++i)
    {
      geometry_msgs::msg::Point point;
      point.x = points(0, i);
      point.y = points(1, i);
      point.z = points(2, i);
      trajectory.pos_pts.push_back(point);
    }
    auto knots = spline.getKnot();
    trajectory.knots.assign(knots.data(), knots.data() + knots.size());
    return trajectory;
  }

  nav_msgs::msg::Odometry makeOdom(const rclcpp::Time &stamp, double yaw = 0.0)
  {
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.pose.pose.position.x = 0.05;
    odom.pose.pose.position.y = 0.05;
    odom.pose.pose.position.z = 0.05;
    odom.pose.pose.orientation.z = std::sin(yaw / 2.0);
    odom.pose.pose.orientation.w = std::cos(yaw / 2.0);
    return odom;
  }

  std::shared_ptr<rclcpp::Node> node;
  SCANReplanFSM fsm;
};

TEST_F(PlanningSafety, OccupiedMeasuredStartIsNeverMoved)
{
  fsm.planner_manager_->grid_map_->setOccupied(fsm.odom_pos_);
  fsm.setStartStateFromOdomOrCurrentTraj();
  EXPECT_TRUE(fsm.start_pt_.isApprox(fsm.odom_pos_));
}

TEST_F(PlanningSafety, EmptyGlobalPathCancelsOldTaskAndHoldsPosition)
{
  fsm.active_waypoints_ = {fsm.end_pt_};
  fsm.have_target_ = fsm.have_new_target_ = fsm.trigger_ = true;
  fsm.exec_state_ = SCANReplanFSM::EXEC_TRAJ;
  fsm.planner_manager_->updateTrajInfo(movingSpline(0.2), node->now());
  fsm.pathCallback(std::make_shared<nav_msgs::msg::Path>());
  EXPECT_TRUE(fsm.active_waypoints_.empty());
  EXPECT_FALSE(fsm.have_target_);
  EXPECT_FALSE(fsm.have_new_target_);
  EXPECT_FALSE(fsm.trigger_);
  EXPECT_EQ(fsm.exec_state_, SCANReplanFSM::WAIT_TARGET);
  auto &traj = fsm.planner_manager_->local_data_.position_traj_;
  EXPECT_TRUE(traj.evaluateDeBoorT(traj.getTimeSum()).isApprox(fsm.odom_pos_));
  EXPECT_NEAR(traj.getDerivative().evaluateDeBoorT(0.0).norm(), 0.0, 1e-9);
}

TEST_F(PlanningSafety, UnusableReplacementRouteCancelsOldTask)
{
  fsm.have_target_ = fsm.have_new_target_ = fsm.trigger_ = true;
  fsm.exec_state_ = SCANReplanFSM::EXEC_TRAJ;
  fsm.planner_manager_->updateTrajInfo(movingSpline(0.2), node->now());
  for (double x = -0.95; x < 1.0; x += 0.1)
    for (double y = -0.95; y < 1.0; y += 0.1)
      for (double z = -0.45; z < 0.6; z += 0.1)
        fsm.planner_manager_->grid_map_->setOccupied(Eigen::Vector3d(x, y, z));
  auto route = std::make_shared<nav_msgs::msg::Path>();
  geometry_msgs::msg::PoseStamped goal;
  goal.pose.position.x = fsm.end_pt_.x();
  goal.pose.position.y = fsm.end_pt_.y();
  goal.pose.position.z = fsm.end_pt_.z();
  route->poses.push_back(goal);
  fsm.pathCallback(route);
  EXPECT_FALSE(fsm.have_target_);
  EXPECT_EQ(fsm.exec_state_, SCANReplanFSM::WAIT_TARGET);
  auto velocity = fsm.planner_manager_->local_data_.position_traj_.getDerivative();
  EXPECT_NEAR(velocity.evaluateDeBoorT(0.0).norm(), 0.0, 1e-9);
}

TEST_F(PlanningSafety, MissingMapObservationStopsAnActiveTrajectory)
{
  fsm.have_target_ = fsm.trigger_ = true;
  fsm.exec_state_ = SCANReplanFSM::EXEC_TRAJ;
  fsm.planner_manager_->updateTrajInfo(movingSpline(0.2), node->now());
  fsm.execFSMCallback();
  auto &traj = fsm.planner_manager_->local_data_.position_traj_;
  EXPECT_NEAR(traj.getDerivative().evaluateDeBoorT(0.0).norm(), 0.0, 1e-9);
  EXPECT_TRUE(fsm.have_target_);  // Fresh observations may resume this task.
}

TEST_F(PlanningSafety, FreshObservationResumesPausedTargetButNotCancelledTarget)
{
  fsm.have_target_ = fsm.trigger_ = true;
  fsm.exec_state_ = SCANReplanFSM::EXEC_TRAJ;
  fsm.planner_manager_->updateTrajInfo(movingSpline(0.2), node->now());
  auto &map = *fsm.planner_manager_->grid_map_;
  map.last_observation_ = std::chrono::steady_clock::now() - std::chrono::seconds(2);
  fsm.checkCollisionCallback();
  EXPECT_TRUE(fsm.waiting_for_map_);
  map.last_observation_ = std::chrono::steady_clock::now();
  ASSERT_TRUE(fsm.ensureFreshMap());
  EXPECT_EQ(fsm.exec_state_, SCANReplanFSM::GEN_NEW_TRAJ);

  map.last_observation_ -= std::chrono::seconds(2);
  ASSERT_FALSE(fsm.ensureFreshMap());
  fsm.pathCallback(std::make_shared<nav_msgs::msg::Path>());
  map.last_observation_ = std::chrono::steady_clock::now();
  ASSERT_TRUE(fsm.ensureFreshMap());
  EXPECT_FALSE(fsm.have_target_);
  EXPECT_EQ(fsm.exec_state_, SCANReplanFSM::WAIT_TARGET);
}

TEST_F(PlanningSafety, PublicationGateRejectsStaleMapAndOccupiedStart)
{
  auto &manager = *fsm.planner_manager_;
  auto &map = *manager.grid_map_;
  map.last_observation_ = std::chrono::steady_clock::now();
  EXPECT_TRUE(manager.checkTrajectoryCollisionFree(movingSpline(0.2)));
  map.setOccupied(fsm.odom_pos_);
  EXPECT_FALSE(manager.reboundReplan(fsm.odom_pos_, Eigen::Vector3d::Zero(),
      Eigen::Vector3d::Zero(), fsm.end_pt_, Eigen::Vector3d::Zero(), true, false));
  EXPECT_EQ(manager.local_data_.traj_id_, 0);
  map.last_observation_ -= std::chrono::seconds(2);
  map.setOccupancy(fsm.odom_pos_, 0);
  EXPECT_FALSE(manager.checkTrajectoryCollisionFree(movingSpline(0.2)));
}

TEST_F(PlanningSafety, PublishedVelocityMustRespectConfiguredNormLimit)
{
  auto &manager = *fsm.planner_manager_;
  EXPECT_TRUE(manager.checkDynamicFeasibility(movingSpline(0.3)));
  EXPECT_FALSE(manager.checkDynamicFeasibility(movingSpline(0.5)));
  EXPECT_FALSE(manager.checkDynamicFeasibility(movingSpline(0.3, 0.3)));
}

TEST_F(PlanningSafety, PublishedAccelerationMustRespectConfiguredLimit)
{
  Eigen::MatrixXd points = Eigen::MatrixXd::Zero(3, 6);
  for (int i = 0; i < points.cols(); ++i)
    points(0, i) = 0.5 * 0.8 * std::pow(i * 0.1, 2);
  EXPECT_FALSE(fsm.planner_manager_->checkDynamicFeasibility(UniformBspline(points, 3, 0.1)));
}

TEST_F(PlanningSafety, MovingStartCanReachCruiseSpeedWithinStrictLimits)
{
  auto &manager = *fsm.planner_manager_;
  manager.grid_map_->last_observation_ = std::chrono::steady_clock::now();
  ASSERT_TRUE(manager.reboundReplan(fsm.odom_pos_, Eigen::Vector3d(0.1, 0.0, 0.0),
      Eigen::Vector3d::Zero(), Eigen::Vector3d(0.575, 0.05, 0.05),
      Eigen::Vector3d(0.35, 0.0, 0.0), true, false));
  EXPECT_TRUE(manager.checkDynamicFeasibility(manager.local_data_.position_traj_));
  EXPECT_TRUE(manager.checkTrajectoryCollisionFree(manager.local_data_.position_traj_));
  EXPECT_LT((manager.local_data_.position_traj_.evaluateDeBoorT(0.0) - fsm.odom_pos_).norm(), 1e-9);
  EXPECT_LT((manager.local_data_.velocity_traj_.evaluateDeBoorT(0.0) -
      Eigen::Vector3d(0.1, 0.0, 0.0)).norm(), 1e-9);
}

TEST_F(PlanningSafety, ControllerOdomTimeoutClearsTrajectoryUntilFreshOdomAndNewTrajectory)
{
  using namespace std::chrono_literals;
  rclcpp::NodeOptions options;
  options.parameter_overrides({{"odom_timeout", 0.05}});
  auto controller = std::make_shared<ClosedLoopController>(options);
  const auto trajectory = makeTrajectoryMsg(movingSpline(0.2), 301);
  const auto stamp = node->now();

  controller->odomCallback(std::make_shared<nav_msgs::msg::Odometry>(makeOdom(stamp)));
  controller->bsplineCallback(std::make_shared<planner::msg::Bspline>(trajectory));
  ASSERT_TRUE(controller->have_odom_);
  ASSERT_TRUE(controller->receive_traj_);

  std::this_thread::sleep_for(80ms);
  controller->cmdCallback();
  EXPECT_FALSE(controller->have_odom_);
  EXPECT_FALSE(controller->receive_traj_);

  controller->odomCallback(std::make_shared<nav_msgs::msg::Odometry>(makeOdom(stamp)));
  EXPECT_FALSE(controller->have_odom_) << "same odom stamp must not refresh controller freshness";

  controller->odomCallback(std::make_shared<nav_msgs::msg::Odometry>(
      makeOdom(stamp + rclcpp::Duration::from_seconds(0.1))));
  EXPECT_TRUE(controller->have_odom_);
  EXPECT_FALSE(controller->receive_traj_) << "recovering odom alone must not resume an old trajectory";

  controller->bsplineCallback(std::make_shared<planner::msg::Bspline>(trajectory));
  EXPECT_TRUE(controller->receive_traj_);
}

TEST_F(PlanningSafety, ControllerNavigationEnableGateClearsAndRejectsTrajectories)
{
  using namespace std::chrono_literals;
  rclcpp::NodeOptions options;
  options.parameter_overrides({
      {"require_navigation_enable", true},
      {"navigation_timeout", 0.05},
      {"odom_timeout", 1.0}});
  auto controller = std::make_shared<ClosedLoopController>(options);
  const auto trajectory = makeTrajectoryMsg(movingSpline(0.2), 302);
  auto enabled = std::make_shared<std_msgs::msg::Bool>();
  auto disabled = std::make_shared<std_msgs::msg::Bool>();
  enabled->data = true;
  disabled->data = false;

  controller->odomCallback(std::make_shared<nav_msgs::msg::Odometry>(makeOdom(node->now())));
  controller->bsplineCallback(std::make_shared<planner::msg::Bspline>(trajectory));
  EXPECT_FALSE(controller->receive_traj_) << "disabled navigation must reject new trajectories";

  controller->navigationEnableCallback(enabled);
  controller->bsplineCallback(std::make_shared<planner::msg::Bspline>(trajectory));
  ASSERT_TRUE(controller->receive_traj_);

  controller->navigationEnableCallback(disabled);
  EXPECT_FALSE(controller->receive_traj_);
  controller->bsplineCallback(std::make_shared<planner::msg::Bspline>(trajectory));
  EXPECT_FALSE(controller->receive_traj_);

  controller->navigationEnableCallback(enabled);
  controller->bsplineCallback(std::make_shared<planner::msg::Bspline>(trajectory));
  ASSERT_TRUE(controller->receive_traj_);
  controller->odomCallback(std::make_shared<nav_msgs::msg::Odometry>(
      makeOdom(node->now() + rclcpp::Duration::from_seconds(2.0))));
  ASSERT_TRUE(controller->have_odom_);
  const auto accepted_stamp = controller->last_odom_stamp_;
  controller->navigationEnableCallback(enabled);
  EXPECT_TRUE(controller->receive_traj_) << "periodic true heartbeats must not reset active state";
  EXPECT_TRUE(controller->have_odom_);
  EXPECT_EQ(controller->last_odom_stamp_.nanoseconds(), accepted_stamp.nanoseconds());
  std::this_thread::sleep_for(80ms);
  controller->cmdCallback();
  EXPECT_FALSE(controller->receive_traj_) << "stale navigation heartbeat must clear the active trajectory";
}

TEST_F(PlanningSafety, ControllerNavigationRecoveryResetsOdomEpochButNotRandomReordering)
{
  using namespace std::chrono_literals;
  rclcpp::NodeOptions options;
  options.parameter_overrides({
      {"require_navigation_enable", true},
      {"navigation_timeout", 0.05},
      {"odom_timeout", 1.0}});
  auto controller = std::make_shared<ClosedLoopController>(options);
  const auto trajectory = makeTrajectoryMsg(movingSpline(0.2), 303);
  auto enabled = std::make_shared<std_msgs::msg::Bool>();
  auto disabled = std::make_shared<std_msgs::msg::Bool>();
  enabled->data = true;
  disabled->data = false;

  controller->navigationEnableCallback(enabled);
  const auto high_stamp = node->now() + rclcpp::Duration::from_seconds(10.0);
  controller->odomCallback(std::make_shared<nav_msgs::msg::Odometry>(makeOdom(high_stamp)));
  ASSERT_TRUE(controller->have_odom_);
  controller->odomCallback(std::make_shared<nav_msgs::msg::Odometry>(
      makeOdom(high_stamp - rclcpp::Duration::from_seconds(1.0))));
  EXPECT_EQ(controller->last_odom_stamp_.nanoseconds(), high_stamp.nanoseconds())
      << "single out-of-order odom must not reset the accepted epoch";

  controller->bsplineCallback(std::make_shared<planner::msg::Bspline>(trajectory));
  ASSERT_TRUE(controller->receive_traj_);
  controller->navigationEnableCallback(disabled);
  EXPECT_FALSE(controller->receive_traj_);
  EXPECT_FALSE(controller->have_odom_);

  controller->navigationEnableCallback(enabled);
  EXPECT_FALSE(controller->receive_traj_);
  const auto low_stamp = node->now() + rclcpp::Duration::from_seconds(1.0);
  controller->odomCallback(std::make_shared<nav_msgs::msg::Odometry>(makeOdom(low_stamp)));
  EXPECT_TRUE(controller->have_odom_) << "gated recovery must allow a lower timestamp epoch";
  EXPECT_EQ(controller->last_odom_stamp_.nanoseconds(), low_stamp.nanoseconds());
  EXPECT_FALSE(controller->receive_traj_) << "new odom after reset still requires a new trajectory";
  controller->bsplineCallback(std::make_shared<planner::msg::Bspline>(trajectory));
  EXPECT_TRUE(controller->receive_traj_);

  std::this_thread::sleep_for(80ms);
  controller->navigationEnableCallback(enabled);
  EXPECT_FALSE(controller->have_odom_) << "heartbeat-stale recovery must reset odom epoch";
  controller->odomCallback(std::make_shared<nav_msgs::msg::Odometry>(makeOdom(low_stamp)));
  EXPECT_TRUE(controller->have_odom_);
}

TEST_F(PlanningSafety, ControllerRejectsInvalidTimeoutParameters)
{
  rclcpp::NodeOptions negative_odom;
  negative_odom.parameter_overrides({{"odom_timeout", -0.1}});
  EXPECT_THROW(std::make_shared<ClosedLoopController>(negative_odom), std::invalid_argument);

  rclcpp::NodeOptions zero_navigation;
  zero_navigation.parameter_overrides({{"navigation_timeout", 0.0}});
  EXPECT_THROW(std::make_shared<ClosedLoopController>(zero_navigation), std::invalid_argument);

  rclcpp::NodeOptions nan_odom;
  nan_odom.parameter_overrides({{"odom_timeout", std::numeric_limits<double>::quiet_NaN()}});
  EXPECT_THROW(std::make_shared<ClosedLoopController>(nan_odom), std::invalid_argument);
}

TEST_F(PlanningSafety, FsmRotatesBodyTwistIntoOdomFrameAndRejectsInvalidOrientation)
{
  auto odom = makeOdom(node->now(), M_PI / 2.0);
  odom.twist.twist.linear.x = 1.0;
  odom.twist.twist.linear.y = 0.0;
  odom.twist.twist.linear.z = 0.2;
  fsm.odometryCallback(std::make_shared<nav_msgs::msg::Odometry>(odom));
  EXPECT_NEAR(fsm.odom_vel_.x(), 0.0, 1e-9);
  EXPECT_NEAR(fsm.odom_vel_.y(), 1.0, 1e-9);
  EXPECT_NEAR(fsm.odom_vel_.z(), 0.2, 1e-9);

  odom.pose.pose.orientation.w = 0.0;
  odom.pose.pose.orientation.z = 0.0;
  odom.twist.twist.linear.x = 3.0;
  fsm.odometryCallback(std::make_shared<nav_msgs::msg::Odometry>(odom));
  EXPECT_NEAR(fsm.odom_vel_.x(), 0.0, 1e-9);
  EXPECT_NEAR(fsm.odom_vel_.y(), 1.0, 1e-9);
}

TEST_F(PlanningSafety, ControllerTurnsBeforeFollowingAPathBehindTheRobot)
{
  using namespace std::chrono_literals;
  auto probe = std::make_shared<rclcpp::Node>("turn_before_translation_test");
  auto controller = std::make_shared<ClosedLoopController>();
  auto odom_pub = probe->create_publisher<nav_msgs::msg::Odometry>("body_pose", 10);
  auto trajectory_pub = probe->create_publisher<planner::msg::Bspline>("planning/bspline", 10);
  std::vector<geometry_msgs::msg::Twist> commands;
  bool frozen = false;
  auto cmd_sub = probe->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10,
      [&](geometry_msgs::msg::Twist::ConstSharedPtr msg) { commands.push_back(*msg); });
  auto frozen_sub = probe->create_subscription<std_msgs::msg::Bool>("planning/go2_execution_frozen", 10,
      [&](std_msgs::msg::Bool::ConstSharedPtr msg) { frozen = msg->data; });
  nav_msgs::msg::Odometry odom;
  odom.pose.pose.position.x = odom.pose.pose.position.y = odom.pose.pose.position.z = 0.05;
  odom.pose.pose.orientation.w = 1.0;
  auto spline = movingSpline(-0.2);
  planner::msg::Bspline trajectory;
  trajectory.order = 3;
  trajectory.traj_id = 101;
  auto points = spline.getControlPoint();
  for (int i = 0; i < points.cols(); ++i)
  {
    geometry_msgs::msg::Point point;
    point.x = points(0, i);
    point.y = points(1, i);
    point.z = points(2, i);
    trajectory.pos_pts.push_back(point);
  }
  auto knots = spline.getKnot();
  trajectory.knots.assign(knots.data(), knots.data() + knots.size());
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(probe);
  executor.add_node(controller);
  const auto drive = [&](const std::function<bool()> &done) {
    const auto deadline = std::chrono::steady_clock::now() + 2s;
    while (std::chrono::steady_clock::now() < deadline)
    {
      odom.header.stamp = probe->now();
      odom_pub->publish(odom);
      executor.spin_some();
      if (done()) return true;
      std::this_thread::sleep_for(10ms);
    }
    return done();
  };
  ASSERT_TRUE(drive([&] { return trajectory_pub->get_subscription_count() > 0; }));
  trajectory_pub->publish(trajectory);
  ASSERT_TRUE(drive([&] { return frozen; }));
  commands.clear();
  ASSERT_TRUE(drive([&] { return commands.size() >= 20; }));
  for (const auto &cmd : commands)
  {
    EXPECT_DOUBLE_EQ(cmd.linear.x, 0.0);
    EXPECT_DOUBLE_EQ(cmd.linear.y, 0.0);
    EXPECT_GT(std::abs(cmd.angular.z), 0.1);
  }
  // After measured heading aligns with the route, translation may resume.
  odom.pose.pose.orientation.w = 0.0;
  odom.pose.pose.orientation.z = 1.0;
  ASSERT_TRUE(drive([&] {
    return !frozen && !commands.empty() && commands.back().linear.x > 0.02;
  }));
  EXPECT_NEAR(commands.back().angular.z, 0.0, 1e-6);
}

TEST_F(PlanningSafety, MessageChainStopsResumesAndCancels)
{
  using namespace std::chrono_literals;
  rclcpp::NodeOptions options;
  options.parameter_overrides({{"max_vx", 0.35}, {"max_vy", 0.4}});
  auto controller = std::make_shared<ClosedLoopController>(options);
  auto body_pub = node->create_publisher<nav_msgs::msg::Odometry>("body_pose", 10);
  auto sensor_pub = node->create_publisher<nav_msgs::msg::Odometry>("sensor_pose", 10);
  auto cloud_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>("cloud", 10);
  auto path_pub = node->create_publisher<nav_msgs::msg::Path>("initial_path", rclcpp::QoS(1).reliable().transient_local());
  std::vector<geometry_msgs::msg::Twist> commands;
  auto sub = node->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10,
      [&](geometry_msgs::msg::Twist::ConstSharedPtr msg) { commands.push_back(*msg); });
  nav_msgs::msg::Odometry odom;
  odom.header.frame_id = "odom";
  odom.child_frame_id = "base_link";
  odom.pose.pose.position.x = odom.pose.pose.position.y = odom.pose.pose.position.z = 0.05;
  odom.pose.pose.orientation.w = 1.0;
  pcl::PointCloud<pcl::PointXYZ> points;
  points.push_back(pcl::PointXYZ(0.75, 0.75, 0.05));
  sensor_msgs::msg::PointCloud2 cloud;
  pcl::toROSMsg(points, cloud);
  cloud.header.frame_id = "odom";

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(controller);
  const auto drive = [&](std::chrono::milliseconds duration, bool observations,
                         const std::function<bool()> &done) {
    const auto deadline = std::chrono::steady_clock::now() + duration;
    auto next_publish = std::chrono::steady_clock::time_point{};
    while (std::chrono::steady_clock::now() < deadline)
    {
      if (std::chrono::steady_clock::now() >= next_publish)
      {
        odom.header.stamp = node->now();
        body_pub->publish(odom);
        sensor_pub->publish(odom);
        if (observations)
        {
          cloud.header.stamp = odom.header.stamp;
          cloud_pub->publish(cloud);
        }
        next_publish = std::chrono::steady_clock::now() + 50ms;
      }
      executor.spin_some();
      if (done()) return true;
      std::this_thread::sleep_for(5ms);
    }
    return done();
  };
  const auto moving = [&] { return !commands.empty() && commands.back().linear.x > 0.02; };
  const auto held = [&] {
    return commands.size() >= 10 &&
      std::all_of(commands.end() - 10, commands.end(), [](const auto &cmd) {
        return std::abs(cmd.linear.x) + std::abs(cmd.linear.y) + std::abs(cmd.angular.z) < 1e-6;
      });
  };
  ASSERT_TRUE(drive(3s, true, [&] { return fsm.planner_manager_->grid_map_->hasRecentObservation(); }));
  nav_msgs::msg::Path route;
  route.header.frame_id = "odom";
  geometry_msgs::msg::PoseStamped goal;
  goal.pose.position.x = 0.75;
  goal.pose.position.y = goal.pose.position.z = 0.05;
  goal.pose.orientation.w = 1.0;
  route.poses.push_back(goal);
  path_pub->publish(route);
  ASSERT_TRUE(drive(8s, true, moving)) << "Fresh observations must permit a feasible trajectory";
  commands.clear();
  ASSERT_TRUE(drive(3s, false, held)) << "Loss of observations must stop controller output";
  ASSERT_TRUE(drive(8s, true, moving)) << "A paused target must resume on fresh observations";
  path_pub->publish(nav_msgs::msg::Path{});
  commands.clear();
  ASSERT_TRUE(drive(2s, true, held)) << "Empty path must stop the controller";
  commands.clear();
  drive(500ms, true, [] { return false; });
  EXPECT_TRUE(held());
  EXPECT_FALSE(fsm.have_target_);
  executor.remove_node(controller);
  executor.remove_node(node);
}
}  // namespace scan_planner
