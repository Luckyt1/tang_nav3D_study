#include "../third_party/planner_runtime/src/octo_global_planner_node.cpp"

#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

using namespace std::chrono_literals;

class BxiOctoGlobalPlannerNodeTestAccess
{
public:
  static void installEmptyMap(const std::shared_ptr<BxiOctoGlobalPlannerNode> & node)
  {
    node->octree_ = std::make_shared<octomap::OcTree>(0.1);
    node->planner_->setOctomap(node->octree_);
    node->map_ready_ = true;
    if (node->map_retry_timer_) {
      node->map_retry_timer_->cancel();
    }
    if (node->path_timer_) {
      node->path_timer_->cancel();
    }
  }

  static void installMap(
    const std::shared_ptr<BxiOctoGlobalPlannerNode> & node,
    const std::shared_ptr<octomap::OcTree> & map)
  {
    node->octree_ = map;
    node->planner_->setOctomap(node->octree_);
    node->map_ready_ = true;
    if (node->map_retry_timer_) {
      node->map_retry_timer_->cancel();
    }
    if (node->path_timer_) {
      node->path_timer_->cancel();
    }
  }

  static void cancelTimers(const std::shared_ptr<BxiOctoGlobalPlannerNode> & node)
  {
    if (node->map_retry_timer_) {
      node->map_retry_timer_->cancel();
    }
    if (node->path_timer_) {
      node->path_timer_->cancel();
    }
  }

  static bool pending(const std::shared_ptr<BxiOctoGlobalPlannerNode> & node)
  {
    return node->pending_plan_;
  }

  static void tryPendingPlan(const std::shared_ptr<BxiOctoGlobalPlannerNode> & node)
  {
    node->tryPendingPlan();
  }

  static void onGoal(
    const std::shared_ptr<BxiOctoGlobalPlannerNode> & node,
    const geometry_msgs::msg::PoseStamped::SharedPtr & msg)
  {
    node->onGoal(msg);
  }

  static void onOdom(
    const std::shared_ptr<BxiOctoGlobalPlannerNode> & node,
    const nav_msgs::msg::Odometry::SharedPtr & msg)
  {
    node->onOdom(msg);
  }
};

double cellCenter(double resolution, int cell)
{
  return (static_cast<double>(cell) + 0.5) * resolution;
}

std::shared_ptr<octomap::OcTree> makeSingleLayerFloor()
{
  constexpr double resolution = 0.2;
  constexpr int ground_z = -5;
  auto map = std::make_shared<octomap::OcTree>(resolution);
  for (int x = -2; x <= 12; ++x) {
    for (int y = -2; y <= 2; ++y) {
      map->updateNode(
        octomap::point3d(
          cellCenter(resolution, x),
          cellCenter(resolution, y),
          cellCenter(resolution, ground_z)),
        true);
    }
  }
  map->updateInnerOccupancy();
  return map;
}

std::shared_ptr<BxiOctoGlobalPlannerNode> makePlannerNodeWithLimits(
  double ground_snap_xy_radius_m,
  double max_goal_snap_distance)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("input_pcd", "/tmp/octo_global_planner_replan_gate_missing.pcd");
  options.append_parameter_override("path_topic", "/test_octo_global_planner_initial_path");
  options.append_parameter_override("debug_path_topic", "/test_octo_global_planner_debug_path");
  options.append_parameter_override("ground_path_topic", "/test_octo_global_planner_ground_path");
  options.append_parameter_override("publish_map", false);
  options.append_parameter_override("trusted_ground_enabled", false);
  options.append_parameter_override("path_publish_period", 10.0);
  options.append_parameter_override("ground_snap_xy_radius_m", ground_snap_xy_radius_m);
  options.append_parameter_override("ground_snap_vertical_range_m", 2.0);
  options.append_parameter_override("max_goal_snap_distance", max_goal_snap_distance);
  return std::make_shared<BxiOctoGlobalPlannerNode>(options);
}

std::shared_ptr<BxiOctoGlobalPlannerNode> makePlannerNode()
{
  return makePlannerNodeWithLimits(0.35, 0.8);
}

geometry_msgs::msg::PoseStamped::SharedPtr makeGoal(double x, double z = 0.0)
{
  auto msg = std::make_shared<geometry_msgs::msg::PoseStamped>();
  msg->header.frame_id = "world";
  msg->pose.position.x = x;
  msg->pose.position.z = z;
  msg->pose.orientation.w = 1.0;
  return msg;
}

nav_msgs::msg::Odometry::SharedPtr makeOdom(double x, double z = 0.0)
{
  auto msg = std::make_shared<nav_msgs::msg::Odometry>();
  msg->pose.pose.position.x = x;
  msg->pose.pose.position.z = z;
  msg->pose.pose.orientation.w = 1.0;
  return msg;
}

void spinFor(rclcpp::Executor & executor, std::chrono::milliseconds duration)
{
  const auto deadline = std::chrono::steady_clock::now() + duration;
  while (std::chrono::steady_clock::now() < deadline) {
    executor.spin_some();
    std::this_thread::sleep_for(5ms);
  }
}

bool spinUntilPathCount(
  rclcpp::Executor & executor,
  const std::vector<nav_msgs::msg::Path> & paths,
  std::size_t expected)
{
  const auto deadline = std::chrono::steady_clock::now() + 1s;
  while (std::chrono::steady_clock::now() < deadline) {
    executor.spin_some();
    if (paths.size() >= expected) {
      return true;
    }
    std::this_thread::sleep_for(5ms);
  }
  return paths.size() >= expected;
}

int runReplanGateScenario()
{
  auto planner_node = makePlannerNode();
  BxiOctoGlobalPlannerNodeTestAccess::cancelTimers(planner_node);

  auto observer = std::make_shared<rclcpp::Node>("octo_global_planner_replan_gate_observer");
  std::vector<nav_msgs::msg::Path> paths;
  auto sub = observer->create_subscription<nav_msgs::msg::Path>(
    "/test_octo_global_planner_initial_path",
    rclcpp::QoS(10).reliable().transient_local(),
    [&paths](const nav_msgs::msg::Path::SharedPtr msg) {
      paths.push_back(*msg);
    });
  (void)sub;

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(planner_node);
  executor.add_node(observer);
  spinFor(executor, 100ms);

  BxiOctoGlobalPlannerNodeTestAccess::onGoal(planner_node, makeGoal(1.0));
  spinFor(executor, 100ms);
  if (!paths.empty() || !BxiOctoGlobalPlannerNodeTestAccess::pending(planner_node)) {
    return 1;
  }

  BxiOctoGlobalPlannerNodeTestAccess::onOdom(planner_node, makeOdom(0.0));
  spinFor(executor, 100ms);
  if (!paths.empty() || !BxiOctoGlobalPlannerNodeTestAccess::pending(planner_node)) {
    return 2;
  }

  BxiOctoGlobalPlannerNodeTestAccess::installEmptyMap(planner_node);
  BxiOctoGlobalPlannerNodeTestAccess::tryPendingPlan(planner_node);
  if (!spinUntilPathCount(executor, paths, 1) || !paths.back().poses.empty()) {
    return 3;
  }

  for (int i = 0; i < 5; ++i) {
    BxiOctoGlobalPlannerNodeTestAccess::onOdom(planner_node, makeOdom(0.1 * i));
  }
  spinFor(executor, 100ms);
  if (paths.size() != 1) {
    return 4;
  }

  BxiOctoGlobalPlannerNodeTestAccess::onGoal(planner_node, makeGoal(2.0));
  if (!spinUntilPathCount(executor, paths, 2) || !paths.back().poses.empty()) {
    return 5;
  }
  return 0;
}

int runGoalProjectionScenario()
{
  auto planner_node = makePlannerNode();
  BxiOctoGlobalPlannerNodeTestAccess::installMap(planner_node, makeSingleLayerFloor());

  auto observer = std::make_shared<rclcpp::Node>("octo_global_planner_goal_projection_observer");
  std::vector<nav_msgs::msg::Path> paths;
  auto sub = observer->create_subscription<nav_msgs::msg::Path>(
    "/test_octo_global_planner_initial_path",
    rclcpp::QoS(10).reliable().transient_local(),
    [&paths](const nav_msgs::msg::Path::SharedPtr msg) {
      paths.push_back(*msg);
    });
  (void)sub;

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(planner_node);
  executor.add_node(observer);
  spinFor(executor, 100ms);

  BxiOctoGlobalPlannerNodeTestAccess::onOdom(planner_node, makeOdom(0.1, 0.3));
  BxiOctoGlobalPlannerNodeTestAccess::onGoal(planner_node, makeGoal(1.3, 0.3));
  if (!spinUntilPathCount(executor, paths, 1) || paths.back().poses.size() < 2) {
    return 1;
  }

  BxiOctoGlobalPlannerNodeTestAccess::onGoal(planner_node, makeGoal(1.3, 3.0));
  if (!spinUntilPathCount(executor, paths, 2) || !paths.back().poses.empty()) {
    return 2;
  }

  return 0;
}

int runFinalGoalSnapLimitScenario()
{
  auto planner_node = makePlannerNodeWithLimits(0.8, 0.2);
  BxiOctoGlobalPlannerNodeTestAccess::installMap(planner_node, makeSingleLayerFloor());

  auto observer = std::make_shared<rclcpp::Node>("octo_global_planner_goal_snap_limit_observer");
  std::vector<nav_msgs::msg::Path> paths;
  auto sub = observer->create_subscription<nav_msgs::msg::Path>(
    "/test_octo_global_planner_initial_path",
    rclcpp::QoS(10).reliable().transient_local(),
    [&paths](const nav_msgs::msg::Path::SharedPtr msg) {
      paths.push_back(*msg);
    });
  (void)sub;

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(planner_node);
  executor.add_node(observer);
  spinFor(executor, 100ms);

  BxiOctoGlobalPlannerNodeTestAccess::onOdom(planner_node, makeOdom(0.1, 0.3));
  BxiOctoGlobalPlannerNodeTestAccess::onGoal(planner_node, makeGoal(2.9, 0.3));
  if (!spinUntilPathCount(executor, paths, 1) || !paths.back().poses.empty()) {
    return 1;
  }

  return 0;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int result = runReplanGateScenario();
  if (result == 0) {
    result = 100 + runGoalProjectionScenario();
    if (result == 100) {
      result = 0;
    }
  }
  if (result == 0) {
    result = 200 + runFinalGoalSnapLimitScenario();
    if (result == 200) {
      result = 0;
    }
  }
  rclcpp::shutdown();
  return result;
}
