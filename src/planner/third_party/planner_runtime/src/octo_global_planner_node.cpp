#include "planner/planner.h"

#include "global_planner.h"
#include "pcd2octomap_converter.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace
{

// Small message builders keep the marker/path publishing code compact.
geometry_msgs::msg::Point makePoint(double x, double y, double z)
{
  geometry_msgs::msg::Point point;
  point.x = x;
  point.y = y;
  point.z = z;
  return point;
}

std_msgs::msg::ColorRGBA makeColor(float r, float g, float b, float a)
{
  std_msgs::msg::ColorRGBA color;
  color.r = r;
  color.g = g;
  color.b = b;
  color.a = a;
  return color;
}

std_msgs::msg::ColorRGBA heightColor(double normalized_height, float alpha)
{
  const float t = static_cast<float>(std::clamp(normalized_height, 0.0, 1.0));
  constexpr float stops[][3] = {
    {0.10F, 0.20F, 0.85F},  // Low: blue.
    {0.00F, 0.75F, 0.85F},  // Mid-low: cyan.
    {0.20F, 0.85F, 0.25F},  // Mid-high: green.
    {0.98F, 0.78F, 0.08F},  // High: yellow.
    {0.90F, 0.12F, 0.08F},  // Highest: red.
  };
  constexpr int segment_count = static_cast<int>(std::size(stops)) - 1;
  const float scaled = t * static_cast<float>(segment_count);
  const int segment = std::min(
    segment_count - 1, static_cast<int>(std::floor(scaled)));
  const float local_t = scaled - static_cast<float>(segment);
  const float red = stops[segment][0] +
    (stops[segment + 1][0] - stops[segment][0]) * local_t;
  const float green = stops[segment][1] +
    (stops[segment + 1][1] - stops[segment][1]) * local_t;
  const float blue = stops[segment][2] +
    (stops[segment + 1][2] - stops[segment][2]) * local_t;
  return makeColor(red, green, blue, alpha);
}

std::string defaultTrustedGroundPosesFile(const std::string & input_pcd)
{
  constexpr char cloud_suffix[] = "_cloud.ply";
  constexpr char ply_suffix[] = ".ply";
  if (input_pcd.size() >= std::size(cloud_suffix) - 1 &&
      input_pcd.compare(
        input_pcd.size() - (std::size(cloud_suffix) - 1),
        std::size(cloud_suffix) - 1,
        cloud_suffix) == 0)
  {
    return input_pcd.substr(0, input_pcd.size() - (std::size(cloud_suffix) - 1)) +
      "_poses.txt";
  }
  if (input_pcd.size() >= std::size(ply_suffix) - 1 &&
      input_pcd.compare(
        input_pcd.size() - (std::size(ply_suffix) - 1),
        std::size(ply_suffix) - 1,
        ply_suffix) == 0)
  {
    return input_pcd.substr(0, input_pcd.size() - (std::size(ply_suffix) - 1)) +
      "_poses.txt";
  }
  return input_pcd + "_poses.txt";
}

bool loadTrustedGroundPoses(
  const std::string & poses_file,
  std::vector<global_planner::PointPose> & poses)
{
  std::ifstream input(poses_file);
  if (!input) {
    return false;
  }

  poses.clear();
  std::string line;
  while (std::getline(input, line)) {
    if (line.empty() || line.front() == '#') {
      continue;
    }

    // rtabmap-export --poses --poses_format 11:
    // timestamp x y z qx qy qz qw id
    std::istringstream fields(line);
    double timestamp = 0.0;
    global_planner::PointPose pose{};
    if (!(fields >> timestamp >> pose.x >> pose.y >> pose.z) ||
        !std::isfinite(pose.x) || !std::isfinite(pose.y) || !std::isfinite(pose.z))
    {
      continue;
    }
    poses.push_back(pose);
  }
  return true;
}

}  // namespace

class BxiOctoGlobalPlannerNode : public rclcpp::Node
{
public:
  explicit BxiOctoGlobalPlannerNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("bxi_octo_global_planner", options)
  {
    // Runtime parameters describe the static map source, planner constraints,
    // topic names, and RViz visualization behavior.
    frame_id_ = declare_parameter<std::string>("frame_id", "world");
    input_pcd_ = declare_parameter<std::string>("input_pcd", "");
    const std::string output_bt = declare_parameter<std::string>("output_bt", "/tmp/bxi_octo_global_map.bt");
    const double cloud_scale = declare_parameter<double>("cloud_scale", 1.0);
    const double octomap_resolution = declare_parameter<double>("octomap_resolution", 0.20);
    const int min_points_per_voxel = declare_parameter<int>("min_points_per_voxel", 2);
    const int min_cluster_voxels = declare_parameter<int>("min_cluster_voxels", 2);
    const double robot_radius = declare_parameter<double>("robot_radius", 0.30);
    const bool require_ground_support = declare_parameter<bool>("require_ground_support", true);
    const bool strict_direct_ground_support = declare_parameter<bool>("strict_direct_ground_support", false);
    const int ground_support_xy_radius_cells = declare_parameter<int>("ground_support_xy_radius_cells", 1);
    const int ground_support_depth_cells = declare_parameter<int>("ground_support_depth_cells", 1);
    const int snap_search_radius_cells = declare_parameter<int>("snap_search_radius_cells", 20);
    max_goal_snap_distance_ = declare_parameter<double>("max_goal_snap_distance", 0.45);
    const int max_iterations = declare_parameter<int>("max_iterations", 1000000);
    const bool enable_preblocked_costmap = declare_parameter<bool>("enable_preblocked_costmap", true);
    const int preblocked_costmap_radius_cells = declare_parameter<int>("preblocked_costmap_radius_cells", 3);
    const double preblocked_costmap_weight = declare_parameter<double>("preblocked_costmap_weight", 1.0);
    const bool enable_clearance_cost = declare_parameter<bool>("enable_clearance_cost", true);
    const int clearance_cost_radius_cells = declare_parameter<int>("clearance_cost_radius_cells", 4);
    const double clearance_cost_weight = declare_parameter<double>("clearance_cost_weight", 0.8);
    const double vertical_search_padding_below = declare_parameter<double>("vertical_search_padding_below", 1.0);
    const double vertical_search_padding_above = declare_parameter<double>("vertical_search_padding_above", 0.6);
    const std::string odom_topic = declare_parameter<std::string>("odom_topic", "/simulation/base_footprint/pose");
    const std::string goal_topic = declare_parameter<std::string>("goal_topic", "/move_base_simple/goal");
    const std::string path_topic = declare_parameter<std::string>("path_topic", "/initial_path");
    const std::string debug_path_topic = declare_parameter<std::string>("debug_path_topic", "/octo_global_path");
    const std::string ground_path_topic = declare_parameter<std::string>("ground_path_topic", "/octo_ground_path");
    const std::string map_topic = declare_parameter<std::string>("map_marker_topic", "/octo_occupied_map");
    const std::string start_marker_topic = declare_parameter<std::string>("start_marker_topic", "/octo_start_marker");
    const std::string goal_marker_topic = declare_parameter<std::string>("goal_marker_topic", "/octo_goal_marker");
    start_z_offset_ = declare_parameter<double>("start_z_offset", 0.0);
    goal_z_offset_ = declare_parameter<double>("goal_z_offset", 0.0);
    path_z_offset_ = declare_parameter<double>("path_z_offset", 0.0);
    const bool trusted_ground_enabled =
      declare_parameter<bool>("trusted_ground_enabled", true);
    const std::string trusted_ground_poses_file_parameter =
      declare_parameter<std::string>("trusted_ground_poses_file", "");
    const double trusted_ground_z_offset =
      declare_parameter<double>("trusted_ground_z_offset", start_z_offset_);
    const double trusted_ground_radius_m =
      declare_parameter<double>("trusted_ground_radius_m", robot_radius);
    const double trusted_ground_max_segment_m =
      declare_parameter<double>("trusted_ground_max_segment_m", 0.75);
    ground_snap_xy_radius_m_ = declare_parameter<double>("ground_snap_xy_radius_m", 0.35);
    ground_snap_vertical_range_m_ = declare_parameter<double>("ground_snap_vertical_range_m", 2.0);
    enforce_path_ground_clearance_ = declare_parameter<bool>("enforce_path_ground_clearance", false);
    path_min_ground_clearance_ = declare_parameter<double>("path_min_ground_clearance", 0.75);
    path_ground_search_depth_ = declare_parameter<double>("path_ground_search_depth", 1.50);
    path_ground_xy_radius_cells_ = declare_parameter<int>("path_ground_xy_radius_cells", 1);
    map_alpha_ = declare_parameter<double>("map_alpha", 0.72);
    publish_map_ = declare_parameter<bool>("publish_map", true);
    path_publish_period_ = declare_parameter<double>("path_publish_period", 0.5);

    // Use transient-local publishers so RViz or downstream nodes can receive
    // the latest path/map immediately after they subscribe.
    const auto latched_qos = rclcpp::QoS(1).transient_local().reliable();
    path_pub_ = create_publisher<nav_msgs::msg::Path>(path_topic, latched_qos);
    debug_path_pub_ = create_publisher<nav_msgs::msg::Path>(debug_path_topic, latched_qos);
    ground_path_pub_ = create_publisher<nav_msgs::msg::Path>(ground_path_topic, latched_qos);
    map_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(map_topic, latched_qos);
    start_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(start_marker_topic, latched_qos);
    goal_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(goal_marker_topic, latched_qos);
    path_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(std::max(0.1, path_publish_period_))),
      std::bind(&BxiOctoGlobalPlannerNode::republishLastPath, this));

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic,
      rclcpp::SensorDataQoS(),
      std::bind(&BxiOctoGlobalPlannerNode::onOdom, this, std::placeholders::_1));
    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      goal_topic,
      rclcpp::QoS(10),
      std::bind(&BxiOctoGlobalPlannerNode::onGoal, this, std::placeholders::_1));

    // The converter and planner implementations come from
    // ../third_party/OctoPlanner3D-ROS2 and are linked in CMakeLists.txt.
    converter_ = std::make_shared<pcd2octomap::Pcd2OctomapConverter>();
    converter_->setInputPcdFile(input_pcd_);
    converter_->setOutputBtFile(output_bt);
    converter_->setInputScale(cloud_scale);
    converter_->setResolution(octomap_resolution);
    converter_->setMinPointsPerVoxel(min_points_per_voxel);
    converter_->setMinClusterVoxels(min_cluster_voxels);
    planner_ = std::make_shared<global_planner::GlobalPlanner>();
    planner_->setRobotRadius(robot_radius);
    planner_->setRequireGroundSupport(require_ground_support);
    planner_->setGroundSupportParams(
      strict_direct_ground_support,
      ground_support_xy_radius_cells,
      ground_support_depth_cells);
    planner_->setSnapSearchRadiusCells(snap_search_radius_cells);
    planner_->setMaxGoalSnapDistance(max_goal_snap_distance_);
    planner_->setMaxIterations(max_iterations);
    planner_->setPreblockedCostmapEnabled(enable_preblocked_costmap);
    planner_->setPreblockedCostmapParams(preblocked_costmap_radius_cells, preblocked_costmap_weight);
    planner_->setClearanceCostParams(enable_clearance_cost, clearance_cost_radius_cells, clearance_cost_weight);
    planner_->setVerticalSearchPadding(vertical_search_padding_below, vertical_search_padding_above);

    if (trusted_ground_enabled) {
      const std::string trusted_ground_poses_file =
        trusted_ground_poses_file_parameter.empty()
        ? defaultTrustedGroundPosesFile(input_pcd_)
        : trusted_ground_poses_file_parameter;
      std::vector<global_planner::PointPose> trusted_ground_poses;
      if (loadTrustedGroundPoses(trusted_ground_poses_file, trusted_ground_poses)) {
        planner_->setTrustedGroundCorridor(
          trusted_ground_poses,
          trusted_ground_z_offset,
          trusted_ground_radius_m,
          trusted_ground_max_segment_m);
        RCLCPP_INFO(
          get_logger(),
          "Trusted ground corridor: poses=%zu source=%s z_offset=%.3f radius=%.3f max_segment=%.3f",
          trusted_ground_poses.size(),
          trusted_ground_poses_file.c_str(),
          trusted_ground_z_offset,
          trusted_ground_radius_m,
          trusted_ground_max_segment_m);
      } else {
        RCLCPP_WARN(
          get_logger(),
          "Trusted ground corridor disabled: cannot read poses file %s. "
          "Continuing with PLY ground support only.",
          trusted_ground_poses_file.c_str());
      }
    }

    RCLCPP_INFO(
      get_logger(),
      "Building OctoMap from %s (cloud_scale=%.6f, resolution=%.3f, robot_radius=%.3f)",
      input_pcd_.c_str(),
      cloud_scale,
      octomap_resolution,
      robot_radius);
    if (!buildMap()) {
      // PCD files may become available shortly after launch (or the first
      // write to /tmp may fail transiently). Keep the node alive and retry
      // instead of permanently rejecting every goal.
      map_retry_timer_ = create_wall_timer(
        std::chrono::seconds(2),
        std::bind(&BxiOctoGlobalPlannerNode::retryBuildMap, this));
    }

    if (map_ready_) {
      RCLCPP_INFO(
        get_logger(),
        "Octo global planner ready: odom=%s goal=%s path=%s ground_path=%s",
        odom_topic.c_str(),
        goal_topic.c_str(),
        path_topic.c_str(),
        ground_path_topic.c_str());
    } else {
      RCLCPP_WARN(
        get_logger(),
        "Octo global planner started without a map; retrying PCD conversion in 2 seconds.");
    }
  }

private:
  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    odom_ = msg;
    tryPendingPlan();
  }

  void onGoal(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    if (msg->header.frame_id.empty() || msg->header.frame_id != frame_id_) {
      RCLCPP_ERROR(
        get_logger(), "Rejecting goal in frame '%s'; expected '%s'.",
        msg->header.frame_id.c_str(), frame_id_.c_str());
      return;
    }
    // A new goal triggers planning with the most recent odometry pose.
    goal_ = msg;
    pending_plan_ = true;
    tryPendingPlan();
  }

  void tryPendingPlan()
  {
    if (pending_plan_ && map_ready_ && odom_ && goal_) {
      pending_plan_ = false;
      planIfReady();
      return;
    }
    if (!goal_ || !pending_plan_) {
      return;
    }
    if (!map_ready_ || !planner_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "OctoMap is not ready; goal cached while the map is built.");
      return;
    }
    if (!odom_) {
      RCLCPP_WARN(get_logger(), "No odometry yet; goal cached.");
    }
  }

  void planIfReady()
  {
    if (!map_ready_ || !planner_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "OctoMap is not ready; goal cached while the map is built.");
      return;
    }
    if (!odom_) {
      RCLCPP_WARN(get_logger(), "No odometry yet; goal cached.");
      return;
    }
    if (!goal_) {
      return;
    }

    // The localization topic carries an elevated base pose, while the map
    // traversability graph lives one cell above ground support. Project both
    // inputs onto that graph before A*, then restore the measured base height
    // only when publishing the route for downstream navigation.
    global_planner::PointPose start_input;
    start_input.x = odom_->pose.pose.position.x;
    start_input.y = odom_->pose.pose.position.y;
    start_input.z = odom_->pose.pose.position.z + start_z_offset_;

    global_planner::PointPose goal_input;
    goal_input.x = goal_->pose.position.x;
    goal_input.y = goal_->pose.position.y;
    goal_input.z = goal_->pose.position.z + goal_z_offset_;

    global_planner::PointPose start;
    if (!planner_->snapToTraversableGroundCell(
          start_input, ground_snap_xy_radius_m_, ground_snap_vertical_range_m_, start)) {
      RCLCPP_ERROR(
        get_logger(),
        "No traversable ground cell near base input [%.2f %.2f %.2f] (xy=%.2fm, z=%.2fm).",
        start_input.x, start_input.y, start_input.z,
        ground_snap_xy_radius_m_, ground_snap_vertical_range_m_);
      publishPaths({}, 0.0);
      return;
    }

    global_planner::PointPose goal;
    if (!planner_->snapToTraversableGroundCell(
          goal_input, ground_snap_xy_radius_m_, ground_snap_vertical_range_m_, goal)) {
      RCLCPP_ERROR(
        get_logger(),
        "No traversable ground cell near goal input [%.2f %.2f %.2f] (xy=%.2fm, z=%.2fm).",
        goal_input.x, goal_input.y, goal_input.z,
        ground_snap_xy_radius_m_, ground_snap_vertical_range_m_);
      publishPaths({}, 0.0);
      return;
    }

    RCLCPP_INFO(
      get_logger(),
      "Planning ground-supported path: base_input=[%.2f %.2f %.2f] start=[%.2f %.2f %.2f] "
      "goal_input=[%.2f %.2f %.2f] goal=[%.2f %.2f %.2f]",
      start_input.x,
      start_input.y,
      start_input.z,
      start.x,
      start.y,
      start.z,
      goal_input.x,
      goal_input.y,
      goal_input.z,
      goal.x,
      goal.y,
      goal.z);

    planner_->makePlan(start, goal);

    // getPlannerResults returns the last path generated by makePlan().
    std::vector<global_planner::PointPose> path;
    planner_->getPlannerResults(path);
    if (path.empty()) {
      RCLCPP_ERROR(get_logger(), "OctoPlanner3D returned an empty path.");
      publishPaths(path, 0.0);
      return;
    }

    const double final_goal_xy_snap = std::hypot(
      path.back().x - goal_input.x,
      path.back().y - goal_input.y);
    if (max_goal_snap_distance_ > 0.0 && final_goal_xy_snap > max_goal_snap_distance_) {
      RCLCPP_ERROR(
        get_logger(),
        "Final goal XY snap to [%.2f %.2f %.2f] is %.2fm, exceeding the %.2fm limit.",
        path.back().x,
        path.back().y,
        path.back().z,
        final_goal_xy_snap,
        max_goal_snap_distance_);
      publishPaths({}, 0.0);
      return;
    }

    // The OctoMap route is a connected sequence of support cells, while the
    // local planner controls bxi_base_link. Keep that consumer-facing route
    // on one base-height plane: RGB-D surface noise can otherwise make a
    // flat floor look like abrupt vertical steps between adjacent voxels.
    const double base_path_z = odom_->pose.pose.position.z + path_z_offset_;
    publishPoseMarker(
      makeBasePoint(path.front(), base_path_z),
      "octo_start",
      makeColor(0.10F, 0.90F, 0.20F, 1.0F),
      start_marker_pub_);
    publishPoseMarker(
      makeBasePoint(path.back(), base_path_z),
      "octo_goal",
      makeColor(0.95F, 0.25F, 0.15F, 1.0F),
      goal_marker_pub_);
    publishPaths(path, base_path_z);
    RCLCPP_INFO(
      get_logger(),
      "Published %zu ground-connected poses; base route height is %.3fm.",
      path.size(),
      base_path_z);
  }

  bool buildMap()
  {
    if (!converter_ || !planner_) {
      RCLCPP_ERROR(get_logger(), "OctoMap build unavailable: converter or planner is null.");
      return false;
    }

    RCLCPP_INFO(
      get_logger(),
      "Building OctoMap from %s (retry=%s)",
      input_pcd_.c_str(), map_ready_ ? "no" : "yes");
    if (!converter_->convert()) {
      RCLCPP_ERROR(
        get_logger(),
        "Failed to build OctoMap from '%s'; will retry in 2 seconds.",
        input_pcd_.c_str());
      return false;
    }

    octree_ = converter_->getOctomap();
    if (!octree_) {
      RCLCPP_ERROR(get_logger(), "OctoMap conversion returned a null tree; will retry.");
      return false;
    }

    planner_->setOctomap(octree_);
    RCLCPP_INFO(
      get_logger(),
      "Trusted ground support cells active: %zu",
      planner_->trustedGroundSupportCellCount());
    map_ready_ = true;

    double min_x = 0.0;
    double min_y = 0.0;
    double min_z = 0.0;
    double max_x = 0.0;
    double max_y = 0.0;
    double max_z = 0.0;
    octree_->getMetricMin(min_x, min_y, min_z);
    octree_->getMetricMax(max_x, max_y, max_z);
    RCLCPP_INFO(
      get_logger(),
      "OctoMap occupied bounds: min=[%.2f %.2f %.2f] max=[%.2f %.2f %.2f]",
      min_x, min_y, min_z, max_x, max_y, max_z);

    if (publish_map_) {
      publishMap();
    }
    if (map_retry_timer_) {
      map_retry_timer_->cancel();
    }
    tryPendingPlan();
    return true;
  }

  void retryBuildMap()
  {
    if (map_ready_) {
      if (map_retry_timer_) {
        map_retry_timer_->cancel();
      }
      return;
    }
    buildMap();
  }

  global_planner::PointPose makeBasePoint(
    const global_planner::PointPose & point,
    double base_path_z) const
  {
    return global_planner::PointPose{point.x, point.y, base_path_z};
  }

  void publishPaths(const std::vector<global_planner::PointPose> & path, double base_path_z)
  {
    nav_msgs::msg::Path ground_msg;
    ground_msg.header.frame_id = frame_id_;
    ground_msg.header.stamp = now();
    ground_msg.poses.reserve(path.size());

    nav_msgs::msg::Path base_msg;
    base_msg.header = ground_msg.header;
    base_msg.poses.reserve(path.size());

    for (const auto & point : path) {
      const double ground_z = groundPathZ(point);
      geometry_msgs::msg::PoseStamped ground_pose;
      ground_pose.header = ground_msg.header;
      ground_pose.pose.position = makePoint(point.x, point.y, ground_z);
      ground_pose.pose.orientation.w = 1.0;
      ground_msg.poses.push_back(ground_pose);

      geometry_msgs::msg::PoseStamped base_pose = ground_pose;
      base_pose.header = base_msg.header;
      base_pose.pose.position.z = base_path_z;
      base_msg.poses.push_back(base_pose);
    }

    ground_path_pub_->publish(ground_msg);
    path_pub_->publish(base_msg);
    debug_path_pub_->publish(base_msg);

    if (path.empty()) {
      has_last_path_ = false;
      return;
    }
    last_ground_path_ = std::move(ground_msg);
    last_base_path_ = std::move(base_msg);
    has_last_path_ = true;
  }

  void republishLastPath()
  {
    if (!has_last_path_) {
      return;
    }

    const auto stamp = now();
    last_ground_path_.header.stamp = stamp;
    last_base_path_.header.stamp = stamp;
    for (auto & pose : last_ground_path_.poses) {
      pose.header.stamp = stamp;
    }
    for (auto & pose : last_base_path_.poses) {
      pose.header.stamp = stamp;
    }
    ground_path_pub_->publish(last_ground_path_);
    // /initial_path is an event consumed by the local planner. Repeating it
    // would reset that planner to waypoint zero on every RViz refresh.
    debug_path_pub_->publish(last_base_path_);
  }

  double groundPathZ(const global_planner::PointPose & point) const
  {
    double support_z = 0.0;
    if (findHighestOccupiedBelow(point.x, point.y, point.z, support_z)) {
      return support_z;
    }
    return point.z;
  }

  bool findHighestOccupiedBelow(double x, double y, double z, double & support_z) const
  {
    if (!octree_) {
      return false;
    }

    const double resolution = octree_->getResolution();
    const int xy_radius = std::max(0, path_ground_xy_radius_cells_);
    const int depth_cells = std::max(
      1, static_cast<int>(std::ceil(path_ground_search_depth_ / resolution)));
    const int start_z_index = static_cast<int>(std::floor(z / resolution));
    const int base_x_index = static_cast<int>(std::floor(x / resolution));
    const int base_y_index = static_cast<int>(std::floor(y / resolution));

    // Search downward first and optionally across neighboring XY cells. The
    // first occupied layer is the highest nearby support surface.
    bool found = false;
    double best_z = -std::numeric_limits<double>::infinity();
    for (int dz = 0; dz <= depth_cells; ++dz) {
      const int z_index = start_z_index - dz;
      for (int dx = -xy_radius; dx <= xy_radius; ++dx) {
        for (int dy = -xy_radius; dy <= xy_radius; ++dy) {
          const double query_x = (static_cast<double>(base_x_index + dx) + 0.5) * resolution;
          const double query_y = (static_cast<double>(base_y_index + dy) + 0.5) * resolution;
          const double query_z = (static_cast<double>(z_index) + 0.5) * resolution;
          const octomap::OcTreeNode * node = octree_->search(query_x, query_y, query_z);
          if (!node || !octree_->isNodeOccupied(node)) {
            continue;
          }
          if (query_z > best_z) {
            best_z = query_z;
            found = true;
          }
        }
      }
      if (found) {
        support_z = best_z;
        return true;
      }
    }

    return false;
  }

  void publishMap()
  {
    if (!octree_ || !map_pub_) {
      return;
    }

    // RViz CUBE_LIST markers require a single cube scale per marker, so group
    // occupied OctoMap leaves by voxel size. Per-point colors retain height.
    const float alpha = static_cast<float>(std::clamp(map_alpha_, 0.05, 1.0));
    double min_x = 0.0;
    double min_y = 0.0;
    double min_z = 0.0;
    double max_x = 0.0;
    double max_y = 0.0;
    double max_z = 0.0;
    octree_->getMetricMin(min_x, min_y, min_z);
    octree_->getMetricMax(max_x, max_y, max_z);
    const double height_range = std::max(1e-6, max_z - min_z);
    std::unordered_map<double, visualization_msgs::msg::Marker> markers_by_size;
    for (auto it = octree_->begin_leafs(); it != octree_->end_leafs(); ++it) {
      if (!octree_->isNodeOccupied(*it)) {
        continue;
      }

      const double size = it.getSize();
      auto marker_it = markers_by_size.find(size);
      if (marker_it == markers_by_size.end()) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = frame_id_;
        marker.ns = "octo_occupied_voxels";
        marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = size;
        marker.scale.y = size;
        marker.scale.z = size;
        marker.color = makeColor(1.0F, 1.0F, 1.0F, alpha);
        marker_it = markers_by_size.emplace(size, std::move(marker)).first;
      }
      marker_it->second.points.push_back(makePoint(it.getX(), it.getY(), it.getZ()));
      marker_it->second.colors.push_back(
        heightColor((it.getZ() - min_z) / height_range, alpha));
    }

    visualization_msgs::msg::MarkerArray array;
    visualization_msgs::msg::Marker cleanup;
    // Delete old markers before publishing the current occupied voxel groups.
    cleanup.header.frame_id = frame_id_;
    cleanup.header.stamp = now();
    cleanup.ns = "octo_occupied_voxels_cleanup";
    cleanup.id = 0;
    cleanup.action = visualization_msgs::msg::Marker::DELETEALL;
    array.markers.push_back(cleanup);

    int id = 0;
    for (auto & entry : markers_by_size) {
      auto & marker = entry.second;
      marker.header.stamp = now();
      marker.id = id++;
      array.markers.push_back(marker);
    }

    map_pub_->publish(array);
  }

  void publishPoseMarker(
    const global_planner::PointPose & pose,
    const std::string & ns,
    const std_msgs::msg::ColorRGBA & color,
    const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr & publisher)
  {
    // Start and goal are shown as simple spheres to make planning inputs easy
    // to verify in RViz.
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id_;
    marker.header.stamp = now();
    marker.ns = ns;
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position = makePoint(pose.x, pose.y, pose.z);
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.35;
    marker.scale.y = 0.35;
    marker.scale.z = 0.35;
    marker.color = color;
    publisher->publish(marker);
  }

  std::string frame_id_;
  std::string input_pcd_;
  double start_z_offset_ = 0.0;
  double goal_z_offset_ = 0.0;
  double path_z_offset_ = 0.0;
  double max_goal_snap_distance_ = 0.45;
  double ground_snap_xy_radius_m_ = 0.35;
  double ground_snap_vertical_range_m_ = 2.0;
  bool enforce_path_ground_clearance_ = false;
  double path_min_ground_clearance_ = 0.75;
  double path_ground_search_depth_ = 1.50;
  int path_ground_xy_radius_cells_ = 1;
  double map_alpha_ = 0.72;
  double path_publish_period_ = 0.5;
  bool publish_map_ = true;
  bool map_ready_ = false;
  bool has_last_path_ = false;
  bool pending_plan_ = false;

  nav_msgs::msg::Odometry::SharedPtr odom_;
  geometry_msgs::msg::PoseStamped::SharedPtr goal_;
  std::shared_ptr<pcd2octomap::Pcd2OctomapConverter> converter_;
  std::shared_ptr<global_planner::GlobalPlanner> planner_;
  std::shared_ptr<octomap::OcTree> octree_;
  nav_msgs::msg::Path last_ground_path_;
  nav_msgs::msg::Path last_base_path_;

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr debug_path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr ground_path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr map_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr start_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr goal_marker_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::TimerBase::SharedPtr path_timer_;
  rclcpp::TimerBase::SharedPtr map_retry_timer_;

  friend class BxiOctoGlobalPlannerNodeTestAccess;
};

std::shared_ptr<rclcpp::Node> planner::createGlobalPlannerNode()
{
  return std::make_shared<BxiOctoGlobalPlannerNode>();
}
