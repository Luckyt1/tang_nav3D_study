#include <algorithm>
#include <chrono>
#include <cmath>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <gtest/gtest.h>
#include <iostream>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>
#include <nav_msgs/msg/odometry.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <queue>
#include <random>
#include <rclcpp/rclcpp.hpp>
#include <rmw/qos_profiles.h>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tuple>
#include <unordered_map>
#include <visualization_msgs/msg/marker.hpp>

#define private public
#include "plan_env/grid_map.h"
#undef private

namespace
{
void declareGridInitParameters(const std::shared_ptr<rclcpp::Node>& node)
{
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
  node->declare_parameter("grid_map.occ_interval_ms", 10000);
  node->declare_parameter("grid_map.vis_interval_ms", 10000);
}

GridMap makeSlidingMap()
{
  GridMap map;
  map.mp_.resolution_ = 1.0;
  map.mp_.resolution_inv_ = 1.0;
  map.mp_.map_voxel_num_ = Eigen::Vector3i(10, 10, 3);
  map.mp_.map_origin_idx_ = Eigen::Vector3i::Zero();
  map.mp_.map_sliding_en_ = true;
  map.mp_.map_sliding_thresh_vox_ = 1;
  map.mp_.min_occupancy_log_ = 0.0;
  map.mp_.clamp_min_log_ = -1.0;
  map.mp_.clamp_max_log_ = 1.0;
  map.mp_.unknown_flag_ = 0.01;
  map.mp_.ground_height_ = -100.0;
  map.mp_.ground_filter_height_ = 0.0;
  map.mp_.dynamic_obstacle_timeout_s_ = 0.0;
  map.mp_.double_cylinder_radius_ = 2.0;
  map.mp_.double_cylinder_offset_ = 0.0;
  map.mp_.obstacles_inflation_z_up = 0.0;
  map.mp_.obstacles_inflation_z_down = 0.0;

  const int buffer_size = map.mp_.map_voxel_num_.prod();
  map.md_.occupancy_buffer_.assign(buffer_size, map.mp_.clamp_min_log_ - map.mp_.unknown_flag_);
  map.md_.occupancy_buffer_inflate_.assign(buffer_size, 0);
  map.md_.occupancy_buffer_inflate_cnt_.assign(buffer_size, 0);
  map.md_.count_hit_.assign(buffer_size, 0);
  map.md_.count_hit_and_miss_.assign(buffer_size, 0);
  map.md_.flag_rayend_.assign(buffer_size, -1);
  map.md_.flag_traverse_.assign(buffer_size, -1);

  map.updateMapBoundaryFromIndex();
  map.rebuildInflationOffsets();
  map.md_.local_bound_min_ = map.mp_.map_bound_min_idx_;
  map.md_.local_bound_max_ = map.mp_.map_bound_max_idx_;
  return map;
}

int inflationCount(GridMap& map, const Eigen::Vector3d& pos)
{
  Eigen::Vector3i id;
  map.posToIndex(pos, id);
  return map.md_.occupancy_buffer_inflate_cnt_[map.toAddress(id)];
}
}  // namespace

TEST(GridMapLidarExtrinsic, UsesDefaultAndParameterOverride)
{
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("grid_map_lidar_extrinsic_test");
  declareGridInitParameters(node);
  node->declare_parameter("grid_map.lidar_extrinsic_translation",
                          std::vector<double>{0.1, -0.2, 0.3});
  node->declare_parameter("grid_map.lidar_extrinsic_rotation",
                          std::vector<double>{0.0, -1.0, 0.0,
                                              1.0, 0.0, 0.0,
                                              0.0, 0.0, 1.0});

  GridMap map;
  map.initMap(node.get());
  EXPECT_DOUBLE_EQ(map.mp_.lidar_extrinsic_(0, 3), 0.1);
  EXPECT_DOUBLE_EQ(map.mp_.lidar_extrinsic_(1, 3), -0.2);
  EXPECT_DOUBLE_EQ(map.mp_.lidar_extrinsic_(2, 3), 0.3);
  EXPECT_DOUBLE_EQ(map.mp_.lidar_extrinsic_(0, 1), -1.0);
  EXPECT_DOUBLE_EQ(map.mp_.lidar_extrinsic_(1, 0), 1.0);

  rclcpp::shutdown();
}

TEST(GridMapLidarExtrinsic, RejectsInvalidRotation)
{
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("grid_map_bad_lidar_extrinsic_test");
  declareGridInitParameters(node);
  node->declare_parameter("grid_map.lidar_extrinsic_rotation",
                          std::vector<double>{1.0, 0.0, 0.0,
                                              0.0, 2.0, 0.0,
                                              0.0, 0.0, 1.0});

  GridMap map;
  EXPECT_THROW(map.initMap(node.get()), std::invalid_argument);

  rclcpp::shutdown();
}

TEST(GridMapSlidingInflation, KeepsInflationAcrossPositiveXSlide)
{
  auto map = makeSlidingMap();
  const Eigen::Vector3d obstacle(4.5, 0.5, 0.5);
  const Eigen::Vector3d inflated_neighbor(5.5, 0.5, 0.5);

  map.setOccupied(obstacle);
  map.updateSlidingMap(Eigen::Vector3d(1.5, 0.5, 0.5));

  EXPECT_EQ(map.getOccupancy(obstacle), 1);
  EXPECT_EQ(map.getInflateOccupancy(inflated_neighbor, 0.0), 1);
  EXPECT_EQ(inflationCount(map, inflated_neighbor), 1);

  map.setOccupied(obstacle);
  EXPECT_EQ(inflationCount(map, inflated_neighbor), 1);
}

TEST(GridMapSlidingInflation, KeepsInflationAcrossNegativeXSlide)
{
  auto map = makeSlidingMap();
  const Eigen::Vector3d obstacle(-4.5, 0.5, 0.5);
  const Eigen::Vector3d inflated_neighbor(-5.5, 0.5, 0.5);

  map.setOccupied(obstacle);
  map.updateSlidingMap(Eigen::Vector3d(-1.5, 0.5, 0.5));

  EXPECT_EQ(map.getOccupancy(obstacle), 1);
  EXPECT_EQ(map.getInflateOccupancy(inflated_neighbor, 0.0), 1);
  EXPECT_EQ(inflationCount(map, inflated_neighbor), 1);
}

TEST(GridMapSlidingInflation, KeepsInflationAcrossMultiAxisSlide)
{
  auto map = makeSlidingMap();
  const Eigen::Vector3d obstacle(4.5, 4.5, 0.5);
  const Eigen::Vector3d inflated_neighbor(5.5, 5.5, 0.5);

  map.setOccupied(obstacle);
  map.updateSlidingMap(Eigen::Vector3d(1.5, 1.5, 0.5));

  EXPECT_EQ(map.getOccupancy(obstacle), 1);
  EXPECT_EQ(map.getInflateOccupancy(inflated_neighbor, 0.0), 1);
  EXPECT_EQ(inflationCount(map, inflated_neighbor), 1);
}

TEST(GridMapSlidingInflation, ClearsInflationForCellsLeavingMap)
{
  auto map = makeSlidingMap();
  const Eigen::Vector3d obstacle(-4.5, 0.5, 0.5);
  const Eigen::Vector3d entering_cell(5.5, 0.5, 0.5);

  map.setOccupied(obstacle);
  map.updateSlidingMap(Eigen::Vector3d(1.5, 0.5, 0.5));

  EXPECT_EQ(map.getOccupancy(obstacle), -1);
  EXPECT_EQ(map.getInflateOccupancy(entering_cell, 0.0), 0);
  EXPECT_EQ(inflationCount(map, entering_cell), 0);
}

TEST(GridMapSlidingInflation, FreeRayEvidenceClearsObstacleAndInflation)
{
  auto map = makeSlidingMap();
  map.mp_.map_sliding_en_ = false;
  map.mp_.max_ray_length_ = 10.0;
  map.mp_.local_update_range_ = Eigen::Vector3d::Constant(10.0);
  map.mp_.prob_hit_log_ = 1.0;
  map.mp_.prob_miss_log_ = -0.5;
  map.md_.ray_pos_ = Eigen::Vector3d(0.5, 0.5, 0.5);
  map.md_.raycast_num_ = 0;
  const Eigen::Vector3d obstacle(2.5, 0.5, 0.5);
  map.setOccupied(obstacle);
  ASSERT_EQ(map.getInflateOccupancy(obstacle, 0.0), 1);
  // Returns beyond the old obstacle are positive evidence that its cell is free.
  map.md_.proj_points_ = {Eigen::Vector3d(4.5, 0.5, 0.5)};
  map.md_.proj_points_cnt = 1;
  for (int i = 0; i < 4; ++i)
    map.raycastProcess();
  EXPECT_EQ(map.getOccupancy(obstacle), 0);
  // The new endpoint is itself inflated; check the old obstacle's opposite side.
  EXPECT_EQ(map.getInflateOccupancy(Eigen::Vector3d(1.5, 0.5, 0.5), 0.0), 0);
}

TEST(GridMapSlidingInflation, FreshnessExpiresWithoutDeletingOccupancy)
{
  auto map = makeSlidingMap();
  const Eigen::Vector3d obstacle(2.5, 0.5, 0.5);
  map.setOccupied(obstacle);
  EXPECT_FALSE(map.hasRecentObservation());
  map.last_observation_ = std::chrono::steady_clock::now();
  EXPECT_TRUE(map.hasRecentObservation());
  map.last_observation_ -= std::chrono::seconds(2);
  EXPECT_FALSE(map.hasRecentObservation());
  EXPECT_EQ(map.getOccupancy(obstacle), 1);
  EXPECT_EQ(map.getInflateOccupancy(obstacle, 0.0), 1);
}

TEST(GridMapSlidingInflation, FullResetRequiresNewObservation)
{
  auto map = makeSlidingMap();
  map.last_observation_ = std::chrono::steady_clock::now();
  ASSERT_TRUE(map.hasRecentObservation());
  map.resetAllMapData();
  EXPECT_FALSE(map.hasRecentObservation());
}
