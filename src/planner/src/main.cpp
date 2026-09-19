#include <exception>

#include <rclcpp/rclcpp.hpp>

#include "planner/planner.h"

// CMake selects the library factory for each executable built from this file.
#ifndef PLANNER_NODE_FACTORY
#error "PLANNER_NODE_FACTORY must select a factory declared in planner/planner.h"
#endif

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int result = 0;
  try
  {
    auto node = PLANNER_NODE_FACTORY();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
  }
  catch (const std::exception & error)
  {
    RCLCPP_FATAL(rclcpp::get_logger("planner"), "Failed to run planner node: %s", error.what());
    result = 1;
  }
  rclcpp::shutdown();
  return result;
}
