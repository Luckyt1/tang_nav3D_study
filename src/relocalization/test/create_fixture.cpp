#include "map/submap.hpp"

#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <initializer_list>
#include <iostream>
#include <stdexcept>
#include <string>
#include <system_error>

#include <Eigen/Geometry>

namespace {

using map_processing::BtcConfig;
using map_processing::Cloud;
using map_processing::MapSaveOptions;
using map_processing::SubmapBuilder;
using map_processing::SubmapConfig;

Cloud makeSparseCloud() {
  Cloud cloud;
  cloud.emplace_back(1.0F, 0.0F, 0.0F);
  cloud.emplace_back(1.4F, 0.2F, 0.1F);
  cloud.emplace_back(1.8F, -0.2F, 0.0F);
  cloud.width = static_cast<std::uint32_t>(cloud.size());
  cloud.height = 1;
  cloud.is_dense = true;
  return cloud;
}

Cloud makeRoomCloud() {
  Cloud cloud;
  for (float x = -2.5F; x <= 2.5F + 1e-4F; x += 0.08F) {
    for (float y = -2.5F; y <= 2.5F + 1e-4F; y += 0.08F) {
      cloud.emplace_back(x, y, 0.0F);
      cloud.emplace_back(x, y, 2.0F);
    }
  }
  for (float z = 0.0F; z <= 2.0F + 1e-4F; z += 0.08F) {
    for (float d = -2.5F; d <= 2.5F + 1e-4F; d += 0.08F) {
      cloud.emplace_back(d, -2.5F, z);
      cloud.emplace_back(d, 2.5F, z);
      cloud.emplace_back(-2.5F, d, z);
      cloud.emplace_back(2.5F, d, z);
    }
  }
  for (const auto& post : {Eigen::Vector3f(-1.2F, -0.8F, 1.8F),
                          Eigen::Vector3f(1.1F, -0.6F, 1.6F),
                          Eigen::Vector3f(-0.3F, 1.0F, 1.7F),
                          Eigen::Vector3f(1.2F, 1.1F, 1.5F)}) {
    for (float z = 0.0F; z <= post.z() + 1e-4F; z += 0.06F) {
      for (float d = -0.16F; d <= 0.16F + 1e-4F; d += 0.04F) {
        cloud.emplace_back(post.x() + d, post.y() - 0.16F, z);
        cloud.emplace_back(post.x() + d, post.y() + 0.16F, z);
        cloud.emplace_back(post.x() - 0.16F, post.y() + d, z);
        cloud.emplace_back(post.x() + 0.16F, post.y() + d, z);
      }
    }
  }
  cloud.width = static_cast<std::uint32_t>(cloud.size());
  cloud.height = 1;
  cloud.is_dense = true;
  return cloud;
}

SubmapConfig submapConfig() {
  SubmapConfig config;
  config.translation_threshold = 10.0;
  config.radius = 2.0;
  return config;
}

BtcConfig btcConfig() {
  BtcConfig config;
  config.useful_corner_num = 30;
  config.voxel_size = 0.2;
  config.voxel_init_num = 3;
  config.projection_plane_num = 1;
  config.projection_resolution = 0.1;
  config.projection_height_increment = 0.1;
  config.projection_distance_min = -1.0;
  config.projection_distance_max = 3.0;
  config.summary_min_threshold = 3;
  config.descriptor_near_num = 3;
  config.descriptor_min_length = 0.2;
  config.descriptor_max_length = 8.0;
  config.non_max_suppression_radius = 0.15;
  config.triangle_resolution = 0.2;
  return config;
}

MapSaveOptions saveOptions(const std::filesystem::path& root) {
  MapSaveOptions options;
  options.directory = root;
  options.map_frame = "fixture_map";
  options.sensor_frame = "fixture_lidar";
  options.freedom.sensor_min_range = 0.0;
  options.freedom.sensor_max_range = 10.0;
  options.freedom.sensor_min_z = -4.0;
  options.freedom.sensor_max_z = 4.0;
  options.freedom.sub_voxel_size = 0.05;
  options.freedom.counts_to_free = 999;
  options.freedom.counts_to_revert = 999;
  options.freedom.num_threads = 1;
  options.btc = btcConfig();
  return options;
}

}  // namespace

int main(int argc, char** argv) {
  try {
    const std::filesystem::path root =
      argc > 1 ? std::filesystem::path(argv[1]) :
                 std::filesystem::temp_directory_path() /
                   ("relocalization_fixture_" +
                    std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    std::filesystem::create_directories(root);

    const bool room = argc > 2 && std::string(argv[2]) == "room";
    if (argc > 2 && !room) {
      throw std::invalid_argument("fixture mode must be room when specified");
    }
    Cloud cloud = room ? makeRoomCloud() : makeSparseCloud();
    auto config = submapConfig();
    auto options = saveOptions(root);
    Eigen::Isometry3d map_from_submap = Eigen::Isometry3d::Identity();
    if (room) {
      config.radius = 8.0;
      options.btc.useful_corner_num = 300;
      options.btc.projection_plane_num = 4;
      options.btc.descriptor_near_num = 8;
      map_from_submap.translation() = Eigen::Vector3d(4.0, -2.0, 0.5);
      map_from_submap.linear() =
        Eigen::AngleAxisd(0.6, Eigen::Vector3d::UnitZ()).toRotationMatrix();
      // Keep the exact sensor-local input available to the Python ROS test.
      std::ofstream query(root / "query.xyz");
      query.exceptions(std::ios::badbit | std::ios::failbit);
      query << std::setprecision(9);
      for (auto& point : cloud) {
        query << point.x << ' ' << point.y << ' ' << point.z << '\n';
        const Eigen::Vector3d mapped =
          map_from_submap * Eigen::Vector3d(point.x, point.y, point.z);
        point.x = static_cast<float>(mapped.x());
        point.y = static_cast<float>(mapped.y());
        point.z = static_cast<float>(mapped.z());
      }
    }
    SubmapBuilder builder(config);
    if (!builder.update(cloud, map_from_submap, 1000000000).created) {
      throw std::runtime_error("fixture submap was not created");
    }
    const auto snapshot = builder.save(cloud, options);

    if (!room) {
      std::cout << "query_xyz 1.0 0.0 0.0\n";
      std::cout << "query_xyz 1.4 0.2 0.1\n";
      std::cout << "query_xyz 1.8 -0.2 0.0\n";
    }
    std::cout << snapshot.string() << '\n';
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
