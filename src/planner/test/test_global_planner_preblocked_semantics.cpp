#define private public
#include "global_planner.h"
#undef private

#include <memory>
#include <vector>

namespace
{

constexpr double kResolution = 0.1;

double center(int cell)
{
  return (static_cast<double>(cell) + 0.5) * kResolution;
}

std::shared_ptr<octomap::OcTree> makeSupportedCorridor(int first_x, int last_x)
{
  auto map = std::make_shared<octomap::OcTree>(kResolution);
  for (int x = first_x; x <= last_x; ++x) {
    map->updateNode(octomap::point3d(center(x), center(0), center(0)), true);
  }
  map->updateNode(octomap::point3d(center(first_x), center(0), center(4)), true);
  map->updateInnerOccupancy();
  return map;
}

double centerFor(double resolution, int cell)
{
  return (static_cast<double>(cell) + 0.5) * resolution;
}

global_planner::GlobalPlanner makeCorridorPlanner(
  bool preblocked_costmap_enabled = true)
{
  global_planner::GlobalPlanner planner;
  planner.setRobotRadius(0.0);
  planner.setRequireGroundSupport(true);
  planner.setGroundSupportParams(true, 0, 1);
  planner.setSnapSearchRadiusCells(0);
  planner.setMaxGoalSnapDistance(10.0);
  planner.setMaxIterations(2000);
  planner.setPreblockedCostmapEnabled(preblocked_costmap_enabled);
  planner.setPreblockedCostmapParams(2, 1.0);
  planner.setClearanceCostParams(false, 0, 0.0);
  planner.setVerticalSearchPadding(0.0, 0.0);
  planner.setOctomap(makeSupportedCorridor(0, 4));
  return planner;
}

bool planCorridor(global_planner::GlobalPlanner & planner, int start_x, int goal_x)
{
  planner.makePlan(
    global_planner::PointPose{center(start_x), center(0), center(1)},
    global_planner::PointPose{center(goal_x), center(0), center(1)});
  std::vector<global_planner::PointPose> path;
  planner.getPlannerResults(path);
  return !path.empty();
}

bool pathContainsCell(
  const global_planner::GlobalPlanner & planner,
  const std::vector<global_planner::PointPose> & path,
  const global_planner::GridIndex & cell)
{
  for (const auto & pose : path) {
    if (planner.worldToGrid(pose.x, pose.y, pose.z) == cell) {
      return true;
    }
  }
  return false;
}

int automaticPreblockedCellsAreCostOnly()
{
  auto planner = makeCorridorPlanner();
  const global_planner::GridIndex risk_cell{2, 0, 1};
  planner.preblocked_cells_.insert(risk_cell);
  planner.rebuildDerivedLayers();
  planner.rebuildTraversableComponents();
  planner.rebuildPreblockedCostmap();

  if (planner.traversable_cells_.find(risk_cell) == planner.traversable_cells_.end()) {
    return 1;
  }
  if (planner.getPreblockedCost(risk_cell) <= 0.0) {
    return 2;
  }

  planner.makePlan(
    global_planner::PointPose{center(0), center(0), center(1)},
    global_planner::PointPose{center(4), center(0), center(1)});
  std::vector<global_planner::PointPose> path;
  planner.getPlannerResults(path);
  if (path.empty()) {
    return 3;
  }
  return pathContainsCell(planner, path, risk_cell) ? 0 : 4;
}

int explicitPreblockedCellsStayHard()
{
  for (const bool cost_enabled : {true, false}) {
    auto planner = makeCorridorPlanner(cost_enabled);
    const global_planner::GridIndex forbidden{2, 0, 1};
    planner.external_preblocked_cells_.insert(forbidden);
    planner.rebuildPreblockedCells();
    planner.rebuildDerivedLayers();
    planner.rebuildTraversableComponents();
    planner.rebuildPreblockedCostmap();

    if (planner.isCellTraversable(forbidden, 0.0, true, true, 0, 1)) {
      return cost_enabled ? 10 : 20;
    }
    if (planCorridor(planner, 0, 4)) {
      return cost_enabled ? 11 : 21;
    }
  }
  return 0;
}

int measuredObstacleAndUnsupportedCellsStayForbidden()
{
  auto obstacle_map = makeSupportedCorridor(0, 4);
  obstacle_map->updateNode(octomap::point3d(center(2), center(0), center(1)), true);
  obstacle_map->updateInnerOccupancy();

  global_planner::GlobalPlanner obstacle_planner;
  obstacle_planner.setRobotRadius(0.0);
  obstacle_planner.setRequireGroundSupport(true);
  obstacle_planner.setGroundSupportParams(true, 0, 1);
  obstacle_planner.setOctomap(obstacle_map);
  if (obstacle_planner.isCellTraversable({2, 0, 1}, 0.0, true, true, 0, 1)) {
    return 1;
  }

  auto unsupported_map = std::make_shared<octomap::OcTree>(kResolution);
  unsupported_map->updateNode(octomap::point3d(center(0), center(0), center(4)), true);
  unsupported_map->updateNode(octomap::point3d(center(4), center(0), center(0)), true);
  unsupported_map->updateInnerOccupancy();

  global_planner::GlobalPlanner unsupported_planner;
  unsupported_planner.setRobotRadius(0.0);
  unsupported_planner.setRequireGroundSupport(true);
  unsupported_planner.setGroundSupportParams(true, 0, 1);
  unsupported_planner.setOctomap(unsupported_map);
  return unsupported_planner.isCellTraversable({0, 0, 1}, 0.0, true, true, 0, 1) ? 2 : 0;
}

int singleLayerGroundBuildsPlanningLayerButKeepsSafetyChecks()
{
  constexpr double resolution = 0.2;
  constexpr int first_x = -4;
  constexpr int last_x = 8;
  constexpr int first_y = -3;
  constexpr int last_y = 3;
  constexpr int ground_z = -5;
  constexpr int walk_z = ground_z + 1;

  auto make_floor = [&] {
    auto map = std::make_shared<octomap::OcTree>(resolution);
    for (int x = first_x; x <= last_x; ++x) {
      for (int y = first_y; y <= last_y; ++y) {
        map->updateNode(
          octomap::point3d(
            centerFor(resolution, x),
            centerFor(resolution, y),
            centerFor(resolution, ground_z)),
          true);
      }
    }
    map->updateInnerOccupancy();
    return map;
  };

  global_planner::GlobalPlanner planner;
  planner.setRobotRadius(0.3);
  planner.setRequireGroundSupport(true);
  planner.setGroundSupportParams(false, 1, 1);
  planner.setSnapSearchRadiusCells(20);
  planner.setMaxGoalSnapDistance(10.0);
  planner.setMaxIterations(5000);
  planner.setPreblockedCostmapEnabled(true);
  planner.setPreblockedCostmapParams(3, 1.0);
  planner.setClearanceCostParams(false, 0, 0.0);
  planner.setVerticalSearchPadding(1.0, 0.6);
  planner.setOctomap(make_floor());

  const global_planner::GridIndex walk_cell{0, 0, walk_z};
  if (!planner.isInsidePlanningBounds(walk_cell)) {
    return 1;
  }
  if (planner.isInsideMetricBounds(walk_cell)) {
    return 2;
  }
  if (planner.traversable_cells_.find(walk_cell) == planner.traversable_cells_.end()) {
    return 3;
  }

  global_planner::PointPose snapped{};
  if (!planner.snapToTraversableGroundCell(
        global_planner::PointPose{
          centerFor(resolution, 0), centerFor(resolution, 0), 0.3},
        0.35, 2.0, snapped))
  {
    return 4;
  }

  planner.makePlan(
    global_planner::PointPose{
      centerFor(resolution, 0), centerFor(resolution, 0), centerFor(resolution, walk_z)},
    global_planner::PointPose{
      centerFor(resolution, 6), centerFor(resolution, 0), centerFor(resolution, walk_z)});
  std::vector<global_planner::PointPose> path;
  planner.getPlannerResults(path);
  if (path.size() < 2) {
    return 5;
  }

  auto unsupported_map = std::make_shared<octomap::OcTree>(resolution);
  for (const auto & corner : std::vector<std::pair<int, int>>{
      {first_x, first_y}, {first_x, last_y}, {last_x, first_y}, {last_x, last_y}})
  {
    unsupported_map->updateNode(
      octomap::point3d(
        centerFor(resolution, corner.first),
        centerFor(resolution, corner.second),
        centerFor(resolution, ground_z)),
      true);
  }
  unsupported_map->updateInnerOccupancy();

  global_planner::GlobalPlanner unsupported_planner;
  unsupported_planner.setRobotRadius(0.3);
  unsupported_planner.setRequireGroundSupport(true);
  unsupported_planner.setGroundSupportParams(false, 1, 1);
  unsupported_planner.setVerticalSearchPadding(1.0, 0.6);
  unsupported_planner.setOctomap(unsupported_map);
  if (unsupported_planner.isCellTraversable(walk_cell, 0.3, true, false, 1, 1)) {
    return 6;
  }

  auto obstacle_map = make_floor();
  obstacle_map->updateNode(
    octomap::point3d(
      centerFor(resolution, 0),
      centerFor(resolution, 0),
      centerFor(resolution, walk_z)),
    true);
  obstacle_map->updateInnerOccupancy();

  global_planner::GlobalPlanner obstacle_planner;
  obstacle_planner.setRobotRadius(0.3);
  obstacle_planner.setRequireGroundSupport(true);
  obstacle_planner.setGroundSupportParams(false, 1, 1);
  obstacle_planner.setVerticalSearchPadding(1.0, 0.6);
  obstacle_planner.setOctomap(obstacle_map);
  return obstacle_planner.isCellTraversable(walk_cell, 0.3, true, false, 1, 1) ? 7 : 0;
}

}  // namespace

int main()
{
  if (const int result = automaticPreblockedCellsAreCostOnly(); result != 0) {
    return result;
  }
  if (const int result = explicitPreblockedCellsStayHard(); result != 0) {
    return 100 + result;
  }
  if (const int result = measuredObstacleAndUnsupportedCellsStayForbidden(); result != 0) {
    return 200 + result;
  }
  if (const int result = singleLayerGroundBuildsPlanningLayerButKeepsSafetyChecks(); result != 0) {
    return 400 + result;
  }

  auto planner = makeCorridorPlanner();
  return planCorridor(planner, 4, 0) ? 0 : 300;
}
