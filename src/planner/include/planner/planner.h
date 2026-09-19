#pragma once

#include <memory>

namespace rclcpp
{
class Node;
}

namespace planner
{
std::shared_ptr<rclcpp::Node> createLocalPlannerNode();
std::shared_ptr<rclcpp::Node> createGlobalPlannerNode();
std::shared_ptr<rclcpp::Node> createControllerNode();
std::shared_ptr<rclcpp::Node> createCloudPreprocessorNode();
}  // namespace planner
