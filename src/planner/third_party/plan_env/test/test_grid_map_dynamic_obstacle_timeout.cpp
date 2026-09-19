#include <chrono>
#include <memory>
#include <thread>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include "plan_env/grid_map.h"

using namespace std::chrono_literals;

TEST(GridMapDynamicObstacleTimeout, MissingObservationsPreserveOccupancyAndInflation)
{
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("grid_map_timeout_test");
  node->declare_parameter("grid_map.resolution", 0.1);
  node->declare_parameter("grid_map.sliding_map_size_x", 2.0);
  node->declare_parameter("grid_map.sliding_map_size_y", 2.0);
  node->declare_parameter("grid_map.sliding_map_size_z", 2.0);
  node->declare_parameter("grid_map.obstacles_inflation_z_up", 0.1);
  node->declare_parameter("grid_map.obstacles_inflation_z_down", 0.1);
  node->declare_parameter("grid_map.double_cylinder_radius", 0.2);
  node->declare_parameter("grid_map.map_sliding_en", false);
  node->declare_parameter("grid_map.p_hit", 0.85);
  node->declare_parameter("grid_map.p_miss", 0.30);
  node->declare_parameter("grid_map.p_min", 0.12);
  node->declare_parameter("grid_map.p_max", 0.98);
  node->declare_parameter("grid_map.p_occ", 0.80);
  node->declare_parameter("grid_map.max_ray_length", 1.0);
  node->declare_parameter("grid_map.ground_height", -1.0);
  node->declare_parameter("grid_map.occ_interval_ms", 20);
  node->declare_parameter("grid_map.vis_interval_ms", 10000);
  node->declare_parameter("grid_map.dynamic_obstacle_timeout_s", 0.15);

  GridMap map;
  map.initMap(node.get());
  const Eigen::Vector3d obstacle(0.2, 0.0, 0.0);
  map.setOccupied(obstacle);
  EXPECT_EQ(map.getOccupancy(obstacle), 1);
  EXPECT_EQ(map.getInflateOccupancy(obstacle, 0.0), 1);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  const auto spin_for = [&](std::chrono::milliseconds duration) {
    const auto deadline = std::chrono::steady_clock::now() + duration;
    while (std::chrono::steady_clock::now() < deadline)
    {
      executor.spin_some();
      std::this_thread::sleep_for(5ms);
    }
  };

  spin_for(100ms);
  map.setOccupied(obstacle);
  spin_for(100ms);
  EXPECT_EQ(map.getOccupancy(obstacle), 1);

  const auto deadline = std::chrono::steady_clock::now() + 300ms;
  while (map.getOccupancy(obstacle) != 0 && std::chrono::steady_clock::now() < deadline)
  {
    executor.spin_some();
    std::this_thread::sleep_for(5ms);
  }
  EXPECT_EQ(map.getOccupancy(obstacle), 1);
  EXPECT_EQ(map.getInflateOccupancy(obstacle, 0.0), 1);

  executor.remove_node(node);
  rclcpp::shutdown();
}
