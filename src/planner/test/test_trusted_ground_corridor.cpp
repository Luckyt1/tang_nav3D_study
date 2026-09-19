#include <memory>
#include <vector>

#include "global_planner.h"

namespace
{

std::shared_ptr<octomap::OcTree> makeMap(bool block_corridor)
{
  auto map = std::make_shared<octomap::OcTree>(0.1);
  // Distant floor samples establish map bounds without adding support below
  // the test robot. The upper sample also gives the free walking cell a valid
  // metric Z range.
  for (const auto & point : std::vector<octomap::point3d>{
         {-0.45F, -0.45F, 0.05F}, {-0.45F, 0.45F, 0.05F},
         {0.45F, -0.45F, 0.05F}, {0.45F, 0.45F, 0.45F}})
  {
    map->updateNode(point, true);
  }
  if (block_corridor) {
    map->updateNode(octomap::point3d(0.05F, 0.05F, 0.15F), true);
  }
  map->updateInnerOccupancy();
  return map;
}

bool canSnapWithTrustedCorridor(bool block_corridor)
{
  global_planner::GlobalPlanner planner;
  planner.setRobotRadius(0.0);
  planner.setOctomap(makeMap(block_corridor));

  const global_planner::PointPose base_pose{0.0, 0.0, 0.10};
  global_planner::PointPose snapped{};
  if (planner.snapToTraversableGroundCell(base_pose, 0.0, 0.0, snapped)) {
    return false;  // The local RGB-D floor is intentionally absent.
  }

  planner.setTrustedGroundCorridor({base_pose}, -0.10, 0.0, 0.75);
  return planner.snapToTraversableGroundCell(base_pose, 0.0, 0.0, snapped);
}

}  // namespace

int main()
{
  if (!canSnapWithTrustedCorridor(false)) {
    return 1;
  }
  // A trusted history never overrides a measured obstacle at the body cell.
  if (canSnapWithTrustedCorridor(true)) {
    return 2;
  }
  return 0;
}
