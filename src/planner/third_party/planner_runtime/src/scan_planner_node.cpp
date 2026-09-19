#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <plan_manage/scan_replan_fsm.h>
#include "planner/planner.h"

namespace
{
class LocalPlannerNode final : public rclcpp::Node
{
public:
  explicit LocalPlannerNode(const rclcpp::NodeOptions & options)
  : rclcpp::Node("scan_planner_node", options)
  {
    planner_.init(this);
  }

private:
  // Keep callbacks and planning state alive until this node leaves the executor.
  scan_planner::SCANReplanFSM planner_;
};
}  // namespace

std::shared_ptr<rclcpp::Node> planner::createLocalPlannerNode()
{
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  return std::make_shared<LocalPlannerNode>(options);
}
