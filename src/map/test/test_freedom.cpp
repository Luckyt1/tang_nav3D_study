#include "map/freedom.hpp"

#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <new>
#include <stdexcept>

#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include "freedom/depth_image.h"

namespace {

using map_processing::Cloud;
using map_processing::FreedomConfig;
using map_processing::FreedomProcessor;

Cloud makeCloud(std::initializer_list<Eigen::Vector3f> points) {
  Cloud cloud;
  for (const auto& point : points) {
    cloud.emplace_back(point.x(), point.y(), point.z());
  }
  cloud.width = static_cast<std::uint32_t>(cloud.size());
  cloud.height = 1;
  cloud.is_dense = true;
  return cloud;
}

void addPlane(Cloud& cloud, float x, float y_min, float y_max, float z_min, float z_max,
              float step) {
  for (float y = y_min; y <= y_max + 1e-4F; y += step) {
    for (float z = z_min; z <= z_max + 1e-4F; z += step) {
      cloud.emplace_back(x, y, z);
    }
  }
  cloud.width = static_cast<std::uint32_t>(cloud.size());
  cloud.height = 1;
  cloud.is_dense = true;
}

Cloud makeWall(float x = 4.0F) {
  Cloud cloud;
  // FreeDOM 只有在目标体素的 26 邻域都获得 free 证据后才删除旧障碍。
  // 墙面需要覆盖得比障碍更宽，后续射线才会穿过障碍周围完整邻域。
  addPlane(cloud, x, -1.20F, 1.20F, -1.00F, 1.00F, 0.05F);
  return cloud;
}

Cloud makeObstacle(float x = 2.0F) {
  Cloud cloud;
  addPlane(cloud, x, -0.20F, 0.20F, -0.20F, 0.20F, 0.05F);
  return cloud;
}

Cloud merged(const Cloud& a, const Cloud& b) {
  Cloud out = a;
  out.insert(out.end(), b.begin(), b.end());
  out.width = static_cast<std::uint32_t>(out.size());
  out.height = 1;
  out.is_dense = true;
  return out;
}

FreedomConfig testConfig() {
  FreedomConfig config;
  config.sensor_max_range = 6.0;
  config.sensor_min_z = -2.0;
  config.sensor_max_z = 2.0;
  config.counts_to_free = 1;
  config.counts_to_revert = 20;
  config.num_threads = 2;
  return config;
}

std::size_t countNear(const Cloud& cloud, const Eigen::Vector3f& center, float radius) {
  const float radius_sq = radius * radius;
  std::size_t count = 0;
  for (const auto& point : cloud) {
    const Eigen::Vector3f delta(point.x - center.x(), point.y - center.y(), point.z - center.z());
    if (delta.squaredNorm() <= radius_sq) {
      ++count;
    }
  }
  return count;
}

}  // namespace

TEST(FreeDOMCore, DisabledEnhancementDestructionDoesNotDependOnPreviousMemory) {
  // 关闭射线增强时 DepthImage 不调用 set_params；复用的非零内存也必须安全析构。
  alignas(freedom::DepthImage) unsigned char storage[sizeof(freedom::DepthImage)];
  std::memset(storage, 0xA5, sizeof(storage));
  auto* image = new (storage) freedom::DepthImage;
  EXPECT_TRUE(image->get_enhanced_pointcloud().empty());
  image->~DepthImage();
}

TEST(FreedomProcessor, RejectsInvalidConfig) {
  FreedomConfig config = testConfig();
  config.sensor_max_range = config.sensor_min_range;
  EXPECT_THROW(FreedomProcessor processor(config), std::invalid_argument);

  config = testConfig();
  config.num_threads = 0;
  EXPECT_THROW(FreedomProcessor processor(config), std::invalid_argument);

  config = testConfig();
  config.sensor_max_range = 100.0;
  EXPECT_THROW(FreedomProcessor processor(config), std::invalid_argument);
}

TEST(FreedomProcessor, LocalHistoryEvictsDistantPointsWhileMappingKeepsThem) {
  FreedomProcessor local(testConfig(), true);
  FreedomProcessor mapping(testConfig());
  const auto scan = makeCloud({Eigen::Vector3f(1.0F, 0.0F, 0.0F)});
  const auto origin = Eigen::Isometry3d::Identity();
  ASSERT_FALSE(local.process(scan, origin).empty());
  ASSERT_FALSE(mapping.process(scan, origin).empty());
  auto moved = Eigen::Isometry3d::Identity();
  moved.translation().x() = 30.0;
  const auto local_cloud = local.process(scan, moved);
  const auto global_cloud = mapping.process(scan, moved);
  EXPECT_EQ(countNear(local_cloud, Eigen::Vector3f(1.0F, 0.0F, 0.0F), 0.1F), 0U);
  EXPECT_EQ(countNear(global_cloud, Eigen::Vector3f(1.0F, 0.0F, 0.0F), 0.1F), 1U);
  EXPECT_EQ(countNear(local_cloud, Eigen::Vector3f(31.0F, 0.0F, 0.0F), 0.1F), 1U);
}

TEST(FreedomProcessor, FiltersInvalidPointsAndRejectsInvalidTransform) {
  FreedomProcessor processor(testConfig());
  Eigen::Isometry3d identity = Eigen::Isometry3d::Identity();

  Cloud all_invalid = makeCloud({Eigen::Vector3f(1.0F, 0.0F, 0.0F)});
  all_invalid.points[0].x = std::numeric_limits<float>::quiet_NaN();
  EXPECT_TRUE(processor.process(all_invalid, identity).empty());

  Cloud mixed = makeWall();
  mixed.points[0].x = std::numeric_limits<float>::infinity();
  const Cloud map = processor.process(mixed, identity);
  EXPECT_GT(countNear(map, Eigen::Vector3f(4.0F, 0.0F, 0.0F), 0.35F), 10U);

  Cloud valid_cloud = makeCloud({Eigen::Vector3f(1.0F, 0.0F, 0.0F)});
  Eigen::Isometry3d scaled = Eigen::Isometry3d::Identity();
  scaled.linear()(0, 0) = 2.0;
  EXPECT_THROW(processor.process(valid_cloud, scaled), std::invalid_argument);
}

TEST(FreedomProcessor, EmptyOrAllInvalidCloudKeepsLastSnapshot) {
  FreedomProcessor processor(testConfig());
  const Eigen::Isometry3d identity = Eigen::Isometry3d::Identity();

  const Cloud map = processor.process(makeWall(), identity);
  ASSERT_GT(map.size(), 10U);

  Cloud invalid = makeCloud({Eigen::Vector3f(1.0F, 0.0F, 0.0F)});
  invalid.points[0].z = std::numeric_limits<float>::quiet_NaN();
  const Cloud after_invalid = processor.process(invalid, identity);
  EXPECT_EQ(after_invalid.size(), map.size());

  const Cloud after_empty = processor.process(Cloud{}, identity);
  EXPECT_EQ(after_empty.size(), map.size());

  const Cloud valid_again = processor.process(makeWall(5.0F), identity);
  EXPECT_GT(countNear(valid_again, Eigen::Vector3f(5.0F, 0.0F, 0.0F), 0.35F), 10U);
}

TEST(FreedomProcessor, RemovesNearPointsIncludingBoundaryRelativeToSensor) {
  FreedomProcessor processor(testConfig());
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation() = Eigen::Vector3d(5.0, -3.0, 1.0);
  const Cloud near = makeCloud({
    Eigen::Vector3f::Zero(), Eigen::Vector3f(0.1F, 0.0F, 0.0F),
    Eigen::Vector3f(0.0F, -0.199F, 0.0F), Eigen::Vector3f(0.0F, 0.0F, 0.2F),
    Eigen::Vector3f(-0.2F, 0.0F, 0.0F)});
  EXPECT_TRUE(processor.process(near, pose).empty());

  const Cloud far = makeCloud({Eigen::Vector3f(0.21F, 0.0F, 0.0F),
                               Eigen::Vector3f(0.0F, -0.4F, 0.0F)});
  const Cloud map = processor.process(merged(near, far), pose);
  ASSERT_EQ(map.size(), 2U);
  EXPECT_EQ(countNear(map, Eigen::Vector3f(5.21F, -3.0F, 1.0F), 0.001F), 1U);
  EXPECT_EQ(countNear(map, Eigen::Vector3f(5.0F, -3.4F, 1.0F), 0.001F), 1U);

  // 全部输入被滤除时保留先前地图，不把机器人周围的历史静态地图挖空。
  const Cloud after_near = processor.process(near, pose);
  EXPECT_EQ(after_near.size(), map.size());
}

TEST(FreedomProcessor, NearPointFilterUsesConfiguredMinimumRange) {
  auto config = testConfig();
  config.sensor_min_range = 0.5;
  FreedomProcessor processor(config);
  const Cloud input = makeCloud({Eigen::Vector3f(0.49F, 0.0F, 0.0F),
                                 Eigen::Vector3f(0.5F, 0.0F, 0.0F),
                                 Eigen::Vector3f(0.51F, 0.0F, 0.0F)});
  const Cloud map = processor.process(input, Eigen::Isometry3d::Identity());
  ASSERT_EQ(map.size(), 1U);
  EXPECT_EQ(countNear(map, Eigen::Vector3f(0.51F, 0.0F, 0.0F), 0.001F), 1U);
}

TEST(FreedomProcessor, MaintainsStaticMapAndRemovesDynamicObjects) {
  FreedomProcessor processor(testConfig());
  const Eigen::Isometry3d identity = Eigen::Isometry3d::Identity();

  const Cloud wall = makeWall();
  const Cloud obstacle = makeObstacle();
  Cloud map = processor.process(merged(wall, obstacle), identity);

  ASSERT_GT(countNear(map, Eigen::Vector3f(4.0F, 0.0F, 0.0F), 0.35F), 10U);
  ASSERT_GT(countNear(map, Eigen::Vector3f(2.0F, 0.0F, 0.0F), 0.35F), 5U);

  for (int i = 0; i < 4; ++i) {
    map = processor.process(wall, identity);
  }

  EXPECT_GT(countNear(map, Eigen::Vector3f(4.0F, 0.0F, 0.0F), 0.35F), 10U);
  EXPECT_EQ(countNear(map, Eigen::Vector3f(2.0F, 0.0F, 0.0F), 0.35F), 0U);

  map = processor.process(merged(wall, obstacle), identity);
  EXPECT_GT(countNear(map, Eigen::Vector3f(4.0F, 0.0F, 0.0F), 0.35F), 10U);
  EXPECT_EQ(countNear(map, Eigen::Vector3f(2.0F, 0.0F, 0.0F), 0.35F), 0U);
}

TEST(FreedomProcessor, UsesSensorPoseToWriteMapFramePoints) {
  FreedomProcessor processor(testConfig());
  Eigen::Isometry3d map_from_sensor = Eigen::Isometry3d::Identity();
  map_from_sensor.translation() = Eigen::Vector3d(1.0, -0.5, 0.25);

  const Cloud map = processor.process(makeWall(2.0F), map_from_sensor);

  EXPECT_GT(countNear(map, Eigen::Vector3f(3.0F, -0.5F, 0.25F), 0.45F), 10U);
  EXPECT_EQ(countNear(map, Eigen::Vector3f(2.0F, 0.0F, 0.0F), 0.35F), 0U);
}
