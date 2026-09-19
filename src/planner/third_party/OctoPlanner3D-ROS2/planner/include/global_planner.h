/**
 * @file      octo_planner/include/global_planner.h
 * @brief     3D A star Planner
 * @author    juchunyu <juchunyu@qq.com>
 * @date      2026-05-31 12:00:01 
 * @copyright Copyright (c) 2025-2026 Institute of Robotics Planning and Control (IRPC). 
 *            All rights reserved.
 */
#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <queue>

#include "octomap/OcTree.h"

namespace global_planner
{

struct GridIndex
{
  int x;
  int y;
  int z;

  bool operator==(const GridIndex & other) const
  {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct GridIndexHash
{
  std::size_t operator()(const GridIndex & k) const
  {
    const std::size_t h1 = std::hash<int>{}(k.x);
    const std::size_t h2 = std::hash<int>{}(k.y);
    const std::size_t h3 = std::hash<int>{}(k.z);
    return h1 ^ (h2 << 1) ^ (h3 << 2);
  }
};

struct QueueNode
{
  GridIndex idx;
  double f;
  double g;
};

struct QueueNodeCompare
{
  bool operator()(const QueueNode & a, const QueueNode & b) const
  {
    return a.f > b.f;
  }
};

struct PointPose
{
    double x;
    double y;
    double z;
};

class GlobalPlanner
{
public:
  GlobalPlanner();
  
  ~GlobalPlanner();

  void setOctomap(std::shared_ptr<octomap::OcTree> map);

  // Add a verified walking corridor from optimized base poses. The corridor is
  // only used as virtual ground support for RGB-D blind spots; all occupancy
  // and robot-footprint collision checks still come from the OctoMap.
  void setTrustedGroundCorridor(
    const std::vector<PointPose> & base_poses,
    double ground_z_offset,
    double corridor_radius_m,
    double max_segment_length_m);

  std::size_t trustedGroundSupportCellCount() const
  {
    return trusted_ground_support_cells_.size();
  }

  void makePlan(const PointPose start,const PointPose goal);

  void getPlannerResults(std::vector<PointPose>& plannerResults);

  // Project an arbitrary pose (for example, the elevated base_link pose or a
  // clicked ground goal) onto the nearest collision-free cell supported by
  // the ground. This keeps the global search on the connected walking layer.
  bool snapToTraversableGroundCell(
    const PointPose & seed,
    double horizontal_radius_m,
    double vertical_range_m,
    PointPose & out) const;

  void setRobotRadius(double robot_radius) { robot_radius_ = robot_radius; }

  void setRequireGroundSupport(bool require_ground_support)
  {
    require_ground_support_ = require_ground_support;
  }

  void setGroundSupportParams(
    bool strict_direct_ground_support,
    int support_xy_radius_cells,
    int support_depth_cells)
  {
    strict_direct_ground_support_ = strict_direct_ground_support;
    ground_support_xy_radius_cells_ = support_xy_radius_cells;
    ground_support_depth_cells_ = support_depth_cells;
  }

  void setSnapSearchRadiusCells(int radius_cells)
  {
    snap_search_radius_cells_ = radius_cells;
  }

  void setMaxGoalSnapDistance(double distance_m)
  {
    max_goal_snap_distance_m_ = std::max(0.0, distance_m);
  }

  void setMaxIterations(int max_iterations)
  {
    max_iterations_ = max_iterations;
  }

  void setPreblockedCostmapEnabled(bool enabled)
  {
    enable_preblocked_costmap_ = enabled;
  }


  void setPreblockedCostmapParams(int radius_cells, double weight)
  {
    preblocked_costmap_radius_cells_ = radius_cells;
    preblocked_costmap_weight_ = weight;
  }

  void setClearanceCostParams(bool enabled, int radius_cells, double weight)
  {
    enable_clearance_cost_ = enabled;
    clearance_cost_radius_cells_ = radius_cells;
    clearance_cost_weight_ = weight;
  }

  void setVerticalSearchPadding(double below, double above)
  {
    vertical_search_padding_below_ = below;
    vertical_search_padding_above_ = above;
  }

private:

  void fillBounds(PointPose & min_bound,PointPose & max_bound) const;

  void onGoalPose(const PointPose goal);

  void tryPlan();

  GridIndex worldToGrid(double x, double y, double z) const;

  octomap::point3d gridToWorld(const GridIndex & idx) const;

  bool isInsideMetricBounds(const GridIndex & idx) const;

  bool isInsidePlanningBounds(const GridIndex & idx) const;

  bool hasGroundSupport(
    const GridIndex & idx,
    bool strict_direct_ground_support,
    int support_xy_radius_cells,
    int support_depth_cells) const;

  bool isOccupiedCell(const GridIndex & idx) const;

  bool hasNonOccupiedNeighborSameLevel(const GridIndex & idx) const;

  bool hasSameLevelNeighborWithOccupiedBelow(const GridIndex & idx) const;

  bool hasSameLevelNeighborWithOccupiedAbove(const GridIndex & idx) const;

  void rebuildPreblockedCells();

//   void onExternalPreblockedMarker(const visualization_msgs::msg::Marker::SharedPtr msg);

  void rebuildPreblockedCostmap();

  void rebuildClearanceCostmap();

  void rebuildTraversableComponents();

  bool findNearestCellInComponent(
    const GridIndex & seed, int radius_cells, int component_id, GridIndex & out) const;

  double getPreblockedCost(const GridIndex & idx) const;

  double getClearanceCost(const GridIndex & idx) const;

//   void publishCellSetMarker(
//     const std::unordered_set<GridIndex, GridIndexHash> & cells,
//     const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr & publisher,
//     const std::string & ns,
//     float r_color,
//     float g_color,
//     float b_color,
//     float a_color) const;

  void publishPreblockedCellsMarker();

  void publishRiskCostCloud() const;

  void rebuildDerivedLayers();

  void rebuildTrustedGroundSupport();

  bool isCellTraversable(
    const GridIndex & idx,
    double robot_radius,
    bool require_ground_support,
    bool strict_direct_ground_support,
    int support_xy_radius_cells,
    int support_depth_cells) const;

  bool findNearestFreeCell(
    const GridIndex & seed,
    double robot_radius,
    int radius_cells,
    bool require_ground_support,
    bool strict_direct_ground_support,
    int support_xy_radius_cells,
    int support_depth_cells,
    GridIndex & out) const;

  std::vector<GridIndex> make26Directions() const;

  std::vector<GridIndex> reconstructPath(
    const std::unordered_map<GridIndex, GridIndex, GridIndexHash> & came_from,
    GridIndex current) const;

  bool startPlan();

  void publishPath(
    const std::vector<GridIndex> & cells,
    const std::string & frame_id);

  double euclidean(const GridIndex & a, const GridIndex & b)
  {
    const double dx = static_cast<double>(a.x - b.x);
    const double dy = static_cast<double>(a.y - b.y);
    const double dz = static_cast<double>(a.z - b.z);
    return std::sqrt(dx * dx + dy * dy + dz * dz);
  }


private:
 
  std::string source_world_file_;

  double robot_radius_ = 0.30;
  int max_iterations_ = 800000;
  int snap_search_radius_cells_ = 12;
  double max_goal_snap_distance_m_ = 0.45;
  bool require_ground_support_ = true;
  bool strict_direct_ground_support_ = false;
  int ground_support_xy_radius_cells_ = 1;
  int ground_support_depth_cells_ = 1;
  bool enable_preblocked_costmap_ = true;
  int preblocked_costmap_radius_cells_ = 3;
  double preblocked_costmap_weight_ = 2.5;
  bool enable_clearance_cost_ = true;
  int clearance_cost_radius_cells_ = 4;
  double clearance_cost_weight_ = 0.8;
  double vertical_search_padding_below_ = 1.0;
  double vertical_search_padding_above_ = 0.6;
  bool lowest_traversable_only_ = false;

  bool map_ready_ = false;
  bool has_start_ = false;
  bool has_goal_ = false;
  bool has_goal_pose_ = false;
  bool planning_in_progress_ = false;

  std::uint64_t plan_seq_ = 0;
  std::uint64_t last_success_seq_ = 0;
  std::uint64_t last_octomap_hash_ = 0;

  PointPose start_point_;
  PointPose goal_point_;
  PointPose goal_pose_;

  std::vector<PointPose> planner_results_;

  std::shared_ptr<octomap::OcTree> octree_;

  std::vector<PointPose> trusted_ground_base_poses_;
  std::unordered_set<GridIndex, GridIndexHash> trusted_ground_support_cells_;
  double trusted_ground_z_offset_ = 0.0;
  double trusted_ground_radius_m_ = 0.0;
  double trusted_ground_max_segment_length_m_ = 0.0;

  std::unordered_set<GridIndex, GridIndexHash> traversable_cells_;
  std::unordered_map<GridIndex, int, GridIndexHash> traversable_component_ids_;
  std::unordered_set<GridIndex, GridIndexHash> preblocked_cells_;
  std::unordered_set<GridIndex, GridIndexHash> external_preblocked_cells_;
  std::unordered_map<GridIndex, double, GridIndexHash> preblocked_costmap_;
  std::unordered_map<GridIndex, double, GridIndexHash> clearance_costmap_;
};

}  // namespace global_planner
