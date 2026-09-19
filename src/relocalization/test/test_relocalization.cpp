#include "relocalization/relocalization.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <initializer_list>
#include <iostream>
#include <iterator>
#include <limits>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

#include <Eigen/Geometry>
#include <gtest/gtest.h>
#include <pcl/io/pcd_io.h>

namespace {

using map_processing::BtcConfig;
using map_processing::BtcFeatures;
using map_processing::Cloud;
using map_processing::FreedomConfig;
using map_processing::MapSaveOptions;
using map_processing::SubmapBuilder;
using map_processing::SubmapConfig;
using map_processing::extractBtcFeatures;
using map_processing::loadBtcFeatures;
using relocalization_processing::MapDatabase;
using relocalization_processing::QueryConfig;
using relocalization_processing::RelocalizationProcessor;
using relocalization_processing::SearchConfig;

constexpr double kTolerance = 1e-5;

class TempDirectory {
 public:
  TempDirectory()
  : path_(std::filesystem::temp_directory_path() /
          ("relocalization_test_" +
           std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()))) {
    std::filesystem::create_directories(path_);
  }

  ~TempDirectory() {
    std::error_code error;
    std::filesystem::remove_all(path_, error);
  }

  const std::filesystem::path& path() const { return path_; }

 private:
  std::filesystem::path path_;
};

Eigen::Isometry3d makePose(double x = 0.0, double y = 0.0, double z = 0.0,
                           double yaw_deg = 0.0) {
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation() = Eigen::Vector3d(x, y, z);
  pose.linear() =
    Eigen::AngleAxisd(yaw_deg * M_PI / 180.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  return pose;
}

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

void finalizeCloud(Cloud& cloud) {
  cloud.width = static_cast<std::uint32_t>(cloud.size());
  cloud.height = 1;
  cloud.is_dense = true;
}

void addBoxSurface(Cloud& cloud, float center_x, float center_y, float half_extent, float height) {
  for (float x = center_x - half_extent; x <= center_x + half_extent + 1e-4F; x += 0.08F) {
    for (float y = center_y - half_extent; y <= center_y + half_extent + 1e-4F; y += 0.08F) {
      cloud.emplace_back(x, y, 0.0F);
      cloud.emplace_back(x, y, height);
    }
  }
  for (float z = 0.0F; z <= height + 1e-4F; z += 0.08F) {
    for (float x = center_x - half_extent; x <= center_x + half_extent + 1e-4F; x += 0.08F) {
      cloud.emplace_back(x, center_y - half_extent, z);
      cloud.emplace_back(x, center_y + half_extent, z);
    }
    for (float y = center_y - half_extent; y <= center_y + half_extent + 1e-4F; y += 0.08F) {
      cloud.emplace_back(center_x - half_extent, y, z);
      cloud.emplace_back(center_x + half_extent, y, z);
    }
  }
}

void addSquarePost(Cloud& cloud, float center_x, float center_y, float half_width, float height) {
  for (float z = 0.0F; z <= height + 1e-4F; z += 0.06F) {
    for (float d = -half_width; d <= half_width + 1e-4F; d += 0.04F) {
      cloud.emplace_back(center_x + d, center_y - half_width, z);
      cloud.emplace_back(center_x + d, center_y + half_width, z);
      cloud.emplace_back(center_x - half_width, center_y + d, z);
      cloud.emplace_back(center_x + half_width, center_y + d, z);
    }
  }
}

Cloud makeRoomWithPosts() {
  Cloud cloud;
  addBoxSurface(cloud, 0.0F, 0.0F, 2.5F, 2.0F);
  addSquarePost(cloud, -1.2F, -0.8F, 0.16F, 1.8F);
  addSquarePost(cloud, 1.1F, -0.6F, 0.16F, 1.6F);
  addSquarePost(cloud, -0.3F, 1.0F, 0.16F, 1.7F);
  addSquarePost(cloud, 1.2F, 1.1F, 0.16F, 1.5F);
  finalizeCloud(cloud);
  return cloud;
}

Cloud makeShiftedRoomWithExtraPost() {
  Cloud cloud = makeRoomWithPosts();
  addSquarePost(cloud, 0.35F, -1.5F, 0.14F, 1.4F);
  finalizeCloud(cloud);
  return cloud;
}

BtcConfig testBtcConfig() {
  BtcConfig config;
  config.useful_corner_num = 300;
  config.voxel_size = 0.2;
  config.voxel_init_num = 3;
  config.projection_plane_num = 4;
  config.projection_resolution = 0.1;
  config.projection_height_increment = 0.1;
  config.projection_distance_min = -1.0;
  config.projection_distance_max = 3.0;
  config.summary_min_threshold = 3;
  config.descriptor_near_num = 8;
  config.descriptor_min_length = 0.2;
  config.descriptor_max_length = 8.0;
  config.non_max_suppression_radius = 0.15;
  config.triangle_resolution = 0.2;
  return config;
}

SearchConfig permissiveSearchConfig() {
  SearchConfig config;
  config.top_k = 5;
  config.min_votes = 1;
  config.min_inliers = 1;
  config.max_hypotheses = 128;
  config.max_matches_per_candidate = 4000;
  config.length_tolerance = 0.05;
  config.binary_similarity = 0.5;
  config.max_vertex_error = 0.35;
  return config;
}

SubmapConfig testSubmapConfig(double radius = 8.0) {
  SubmapConfig config;
  config.translation_threshold = 10.0;
  config.radius = radius;
  return config;
}

MapSaveOptions saveOptions(const TempDirectory& temp, BtcConfig btc = testBtcConfig(),
                           double radius = 8.0) {
  MapSaveOptions options;
  options.directory = temp.path();
  options.map_frame = "test_map";
  options.sensor_frame = "test_lidar";
  options.freedom.sensor_min_range = 0.37;
  options.freedom.sensor_max_range = 11.0;
  options.freedom.sensor_min_z = -2.5;
  options.freedom.sensor_max_z = 3.25;
  options.freedom.sub_voxel_size = 0.12;
  options.freedom.counts_to_free = 7;
  options.freedom.counts_to_revert = 31;
  options.freedom.num_threads = 1;
  options.btc = btc;
  (void)radius;
  return options;
}

Cloud transformCloud(const Cloud& cloud, const Eigen::Isometry3d& target_from_source) {
  Cloud transformed;
  transformed.reserve(cloud.size());
  for (const auto& point : cloud) {
    const Eigen::Vector3d source(point.x, point.y, point.z);
    const Eigen::Vector3d target = target_from_source * source;
    transformed.emplace_back(static_cast<float>(target.x()), static_cast<float>(target.y()),
                             static_cast<float>(target.z()));
  }
  finalizeCloud(transformed);
  return transformed;
}

Eigen::Vector3d transformVector(const Eigen::Isometry3d& target_from_source,
                                const Eigen::Vector3d& source) {
  return target_from_source * source;
}

BinaryDescriptor transformBinary(const BinaryDescriptor& binary,
                                 const Eigen::Isometry3d& target_from_source) {
  BinaryDescriptor transformed = binary;
  transformed.location_ = transformVector(target_from_source, binary.location_);
  return transformed;
}

BTC transformTriangle(const BTC& triangle, const Eigen::Isometry3d& target_from_source) {
  BTC transformed = triangle;
  transformed.center_ = transformVector(target_from_source, triangle.center_);
  transformed.binary_A_ = transformBinary(triangle.binary_A_, target_from_source);
  transformed.binary_B_ = transformBinary(triangle.binary_B_, target_from_source);
  transformed.binary_C_ = transformBinary(triangle.binary_C_, target_from_source);
  return transformed;
}

pcl::PointXYZINormal transformPlane(const pcl::PointXYZINormal& plane,
                                    const Eigen::Isometry3d& target_from_source) {
  pcl::PointXYZINormal transformed = plane;
  const Eigen::Vector3d center = target_from_source * Eigen::Vector3d(plane.x, plane.y, plane.z);
  const Eigen::Vector3d normal =
    target_from_source.linear() * Eigen::Vector3d(plane.normal_x, plane.normal_y, plane.normal_z);
  transformed.x = static_cast<float>(center.x());
  transformed.y = static_cast<float>(center.y());
  transformed.z = static_cast<float>(center.z());
  transformed.normal_x = static_cast<float>(normal.x());
  transformed.normal_y = static_cast<float>(normal.y());
  transformed.normal_z = static_cast<float>(normal.z());
  return transformed;
}

BtcFeatures transformFeatures(const BtcFeatures& features,
                              const Eigen::Isometry3d& target_from_source) {
  BtcFeatures transformed;
  transformed.submap_id = features.submap_id;
  transformed.binary.reserve(features.binary.size());
  for (const auto& binary : features.binary) {
    transformed.binary.push_back(transformBinary(binary, target_from_source));
  }
  transformed.triangles.reserve(features.triangles.size());
  for (const auto& triangle : features.triangles) {
    transformed.triangles.push_back(transformTriangle(triangle, target_from_source));
  }
  transformed.planes.reserve(features.planes.size());
  for (const auto& plane : features.planes) {
    transformed.planes.push_back(transformPlane(plane, target_from_source));
  }
  return transformed;
}

std::filesystem::path saveSingleSubmapSnapshot(const TempDirectory& temp, const Cloud& local_cloud,
                                               const Eigen::Isometry3d& map_from_submap,
                                               double radius = 8.0,
                                               BtcConfig btc = testBtcConfig()) {
  SubmapBuilder builder(testSubmapConfig(radius));
  const Cloud map_cloud = transformCloud(local_cloud, map_from_submap);
  EXPECT_TRUE(builder.update(map_cloud, map_from_submap, 100).created);
  return builder.save(map_cloud, saveOptions(temp, btc, radius));
}

std::filesystem::path saveTwoSubmapSnapshot(const TempDirectory& temp) {
  const Eigen::Isometry3d first_anchor = makePose(0.0, 0.0, 0.0);
  const Eigen::Isometry3d second_anchor = makePose(12.0, 0.0, 0.0, 15.0);
  const Cloud first = transformCloud(makeRoomWithPosts(), first_anchor);
  const Cloud second = transformCloud(makeShiftedRoomWithExtraPost(), second_anchor);
  Cloud map_cloud = first;
  map_cloud.insert(map_cloud.end(), second.begin(), second.end());
  finalizeCloud(map_cloud);

  SubmapBuilder builder(testSubmapConfig());
  EXPECT_TRUE(builder.update(map_cloud, first_anchor, 100).created);
  EXPECT_TRUE(builder.update(map_cloud, second_anchor, 200).created);
  return builder.save(map_cloud, saveOptions(temp));
}

std::string readText(const std::filesystem::path& path) {
  std::ifstream stream(path);
  return std::string(std::istreambuf_iterator<char>(stream), std::istreambuf_iterator<char>());
}

void writeText(const std::filesystem::path& path, const std::string& text) {
  std::ofstream stream(path);
  stream << text;
}

std::string withExtractionRevision(std::string metadata, const std::string& revision) {
  const auto offset = metadata.find("  extraction_revision: ");
  if (offset == std::string::npos) {
    throw std::runtime_error("saved metadata is missing extraction_revision");
  }
  const auto length = metadata.find('\n', offset) - offset + 1;
  metadata.replace(offset, length,
                   revision.empty() ? "" : "  extraction_revision: " + revision + "\n");
  return metadata;
}

std::vector<std::string> splitCsvLine(const std::string& line) {
  std::vector<std::string> fields;
  std::size_t start = 0;
  while (start <= line.size()) {
    const std::size_t comma = line.find(',', start);
    if (comma == std::string::npos) {
      fields.push_back(line.substr(start));
      break;
    }
    fields.push_back(line.substr(start, comma - start));
    start = comma + 1;
  }
  return fields;
}

std::vector<std::vector<std::string>> readCsv(const std::filesystem::path& path) {
  std::ifstream stream(path);
  std::vector<std::vector<std::string>> rows;
  std::string line;
  while (std::getline(stream, line)) {
    rows.push_back(splitCsvLine(line));
  }
  return rows;
}

std::string joinCsvLine(const std::vector<std::string>& fields) {
  std::ostringstream line;
  for (std::size_t i = 0; i < fields.size(); ++i) {
    if (i != 0) {
      line << ',';
    }
    line << fields[i];
  }
  return line.str();
}

void writeCsv(const std::filesystem::path& path, const std::vector<std::vector<std::string>>& rows) {
  std::ofstream stream(path);
  for (const auto& row : rows) {
    stream << joinCsvLine(row) << '\n';
  }
}

void expectVectorNear(const Eigen::Vector3d& actual, const Eigen::Vector3d& expected,
                      double tolerance = kTolerance) {
  EXPECT_NEAR(actual.x(), expected.x(), tolerance);
  EXPECT_NEAR(actual.y(), expected.y(), tolerance);
  EXPECT_NEAR(actual.z(), expected.z(), tolerance);
}

void expectPoseNear(const Eigen::Isometry3d& actual, const Eigen::Isometry3d& expected,
                    double tolerance = kTolerance) {
  expectVectorNear(actual.translation(), expected.translation(), tolerance);
  EXPECT_TRUE(actual.linear().isApprox(expected.linear(), tolerance))
    << "actual:\n" << actual.linear() << "\nexpected:\n" << expected.linear();
}

void expectCloudPointNear(const Cloud& cloud, std::size_t index, const Eigen::Vector3f& expected,
                          float tolerance = 1e-4F) {
  ASSERT_LT(index, cloud.size());
  EXPECT_NEAR(cloud[index].x, expected.x(), tolerance);
  EXPECT_NEAR(cloud[index].y, expected.y(), tolerance);
  EXPECT_NEAR(cloud[index].z, expected.z(), tolerance);
}

}  // namespace

TEST(MapDatabase, LoadsSavedMetadataParametersInsteadOfDefaults) {
  TempDirectory temp;
  auto btc = testBtcConfig();
  btc.useful_corner_num = 123;
  btc.voxel_size = 0.3;
  btc.descriptor_near_num = 9;
  btc.triangle_resolution = 0.25;
  const double radius = 6.5;
  const auto snapshot = saveSingleSubmapSnapshot(temp, makeRoomWithPosts(), makePose(), radius, btc);

  const MapDatabase database(snapshot, permissiveSearchConfig());

  EXPECT_EQ(database.size(), 1U);
  EXPECT_EQ(database.mapFrame(), "test_map");
  EXPECT_NEAR(database.radius(), radius, kTolerance);
  EXPECT_EQ(database.btcConfig().useful_corner_num, 123);
  EXPECT_NEAR(database.btcConfig().voxel_size, 0.3, kTolerance);
  EXPECT_EQ(database.btcConfig().descriptor_near_num, 9);
  EXPECT_NEAR(database.btcConfig().triangle_resolution, 0.25, kTolerance);
  EXPECT_NEAR(database.freedomConfig().sensor_min_range, 0.37, kTolerance);
  EXPECT_NEAR(database.freedomConfig().sensor_max_range, 11.0, kTolerance);
  EXPECT_NEAR(database.freedomConfig().sub_voxel_size, 0.12, kTolerance);
  EXPECT_EQ(database.freedomConfig().counts_to_free, 7);
  EXPECT_EQ(database.freedomConfig().counts_to_revert, 31);
  EXPECT_EQ(database.freedomConfig().num_threads, 1);
}

TEST(MapDatabase, LoadsSubmapPointCloudById) {
  TempDirectory temp;
  const auto snapshot = saveSingleSubmapSnapshot(temp, makeCloud({Eigen::Vector3f(1.0F, 2.0F, 0.5F),
                                                                  Eigen::Vector3f(-1.0F, 0.5F, 1.0F)}),
                                                 makePose(3.0, -1.0, 0.0, 30.0));
  const MapDatabase database(snapshot, permissiveSearchConfig());

  const Cloud submap = database.loadSubmap(0);

  ASSERT_EQ(submap.size(), 2U);
  expectCloudPointNear(submap, 0, Eigen::Vector3f(1.0F, 2.0F, 0.5F));
  expectCloudPointNear(submap, 1, Eigen::Vector3f(-1.0F, 0.5F, 1.0F));
  EXPECT_THROW(database.loadSubmap(99), std::out_of_range);
}

TEST(MapDatabase, RebuildsOnlyLegacyFeaturesFromPcdWithoutChangingSnapshot) {
  TempDirectory temp;
  const auto snapshot = saveSingleSubmapSnapshot(temp, makeRoomWithPosts(), makePose());
  const auto metadata = readText(snapshot / "metadata.yaml");
  ASSERT_NE(metadata.find("  extraction_revision: " +
                         std::to_string(map_processing::kBtcExtractionRevision) + "\n"),
            std::string::npos);

  Cloud saved_cloud;
  ASSERT_EQ(pcl::io::loadPCDFile((snapshot / "submaps/000000.pcd").string(), saved_cloud), 0);
  const auto query = extractBtcFeatures(saved_cloud, 123, testBtcConfig());
  ASSERT_GE(query.triangles.size(), 5U);

  // A valid older extraction may have found no features in a non-empty submap.
  writeText(snapshot / "btc/000000.btc",
            "MAP_BTC 1\nsubmap_id 0\nbinary_count 0\ntriangle_count 0\nplane_count 0\nend\n");
  writeText(snapshot / "btc/manifest.csv",
            "id,status,binary_count,triangle_count,plane_count,descriptor_file\n"
            "0,no_triangles,0,0,0,btc/000000.btc\n");

  for (const std::string revision : {"", "1", "2"}) {
    SCOPED_TRACE("extraction_revision=" + revision);
    writeText(snapshot / "metadata.yaml", withExtractionRevision(metadata, revision));
    std::vector<std::pair<std::filesystem::path, std::string>> files_before;
    for (const auto& entry : std::filesystem::recursive_directory_iterator(snapshot)) {
      if (entry.is_regular_file()) {
        files_before.emplace_back(entry.path(), readText(entry.path()));
      }
    }

    const MapDatabase database(snapshot);
    const auto candidates = database.search(query);
    if (revision == "2") {
      EXPECT_TRUE(candidates.empty());
    } else {
      ASSERT_FALSE(candidates.empty());
      EXPECT_EQ(candidates.front().submap_id, 0U);
      expectPoseNear(candidates.front().submap_from_query, Eigen::Isometry3d::Identity(), 0.01);
    }
    for (const auto& [path, contents] : files_before) {
      EXPECT_EQ(readText(path), contents) << path;
    }
  }
}

TEST(MapDatabase, RejectsInvalidOrFutureExtractionRevision) {
  TempDirectory temp;
  const auto snapshot = saveSingleSubmapSnapshot(
    temp, makeCloud({Eigen::Vector3f(1.0F, 0.0F, 0.0F)}), makePose());
  const auto metadata = readText(snapshot / "metadata.yaml");
  for (const std::string revision : {"0", "-1", "3", "2.5", "bogus"}) {
    SCOPED_TRACE(revision);
    writeText(snapshot / "metadata.yaml", withExtractionRevision(metadata, revision));
    EXPECT_THROW(MapDatabase database(snapshot), std::runtime_error);
  }
}

TEST(MapDatabase, ValidatesLegacyManifestBeforeRebuildingFeatures) {
  TempDirectory temp;
  const auto snapshot = saveSingleSubmapSnapshot(
    temp, makeCloud({Eigen::Vector3f(1.0F, 0.0F, 0.0F)}), makePose());
  writeText(snapshot / "metadata.yaml",
            withExtractionRevision(readText(snapshot / "metadata.yaml"), ""));
  auto manifest = readCsv(snapshot / "btc/manifest.csv");
  ASSERT_EQ(manifest.size(), 2U);
  manifest[1][2] = "999";
  writeCsv(snapshot / "btc/manifest.csv", manifest);
  EXPECT_THROW(MapDatabase database(snapshot), std::runtime_error);
}

TEST(MapDatabase, RejectsInvalidLegacySubmapPcdBeforeRebuildingFeatures) {
  TempDirectory temp;
  Cloud cloud = makeCloud({Eigen::Vector3f(1.0F, 0.0F, 0.0F),
                           Eigen::Vector3f(2.0F, 0.0F, 0.0F)});
  const auto snapshot = saveSingleSubmapSnapshot(temp, cloud, makePose());
  writeText(snapshot / "metadata.yaml",
            withExtractionRevision(readText(snapshot / "metadata.yaml"), ""));
  const auto file = (snapshot / "submaps/000000.pcd").string();

  Cloud wrong_count = makeCloud({Eigen::Vector3f(1.0F, 0.0F, 0.0F)});
  ASSERT_EQ(pcl::io::savePCDFileBinary(file, wrong_count), 0);
  EXPECT_THROW(MapDatabase database(snapshot), std::runtime_error);

  cloud[0].x = std::numeric_limits<float>::quiet_NaN();
  cloud.is_dense = false;
  ASSERT_EQ(pcl::io::savePCDFileBinary(file, cloud), 0);
  EXPECT_THROW(MapDatabase database(snapshot), std::invalid_argument);
}

TEST(MapDatabase, LoadsWholeMapInSavedCoordinatesIncludingPointsOutsideSubmaps) {
  TempDirectory temp;
  const auto local = makeCloud({Eigen::Vector3f(1.0F, 0.0F, 0.5F),
                                Eigen::Vector3f(20.0F, 0.0F, 1.0F)});
  const auto anchor = makePose(3.0, -1.0, 0.0, 30.0);
  const auto snapshot = saveSingleSubmapSnapshot(temp, local, anchor, 5.0);
  // metadata 中的路径是读取依据，不能硬编码文件名或用子图拼接代替完整地图。
  std::filesystem::rename(snapshot / "static_map.pcd", snapshot / "whole map.pcd");
  auto metadata = readText(snapshot / "metadata.yaml");
  const auto offset = metadata.find("static_map_file: static_map.pcd");
  ASSERT_NE(offset, std::string::npos);
  metadata.replace(offset, std::string("static_map_file: static_map.pcd").size(),
                   "static_map_file: \"whole map.pcd\"");
  writeText(snapshot / "metadata.yaml", metadata);

  const MapDatabase database(snapshot);
  const auto full = database.loadGlobalMap();
  const auto expected = transformCloud(local, anchor);
  ASSERT_EQ(full.size(), 2U);
  EXPECT_EQ(database.loadSubmap(0).size(), 1U);
  for (std::size_t i = 0; i < expected.size(); ++i) {
    expectCloudPointNear(full, i, expected[i].getVector3fMap());
  }
}

TEST(MapDatabase, ReportsMissingOrMismatchedGlobalMap) {
  TempDirectory temp;
  const auto cloud = makeCloud({Eigen::Vector3f(1.0F, 0.0F, 0.0F)});
  const auto snapshot = saveSingleSubmapSnapshot(temp, cloud, makePose());
  const MapDatabase database(snapshot);
  Cloud wrong_cloud = cloud;
  wrong_cloud.emplace_back(2.0F, 0.0F, 0.0F);
  ASSERT_EQ(pcl::io::savePCDFileBinary((snapshot / "static_map.pcd").string(), wrong_cloud), 0);
  EXPECT_THROW(database.loadGlobalMap(), std::runtime_error);
  std::filesystem::remove(snapshot / "static_map.pcd");
  EXPECT_THROW(database.loadGlobalMap(), std::runtime_error);
}

TEST(MapDatabase, RetrievesCorrectSubmapAndCoarsePoseForRigidlyTransformedBtcQuery) {
  TempDirectory temp;
  const Eigen::Isometry3d map_from_submap = makePose(4.0, -2.0, 0.3, 35.0);
  const auto snapshot = saveSingleSubmapSnapshot(temp, makeRoomWithPosts(), map_from_submap);
  const auto stored = loadBtcFeatures(snapshot / "btc" / "000000.btc");
  ASSERT_FALSE(stored.triangles.empty());
  const Eigen::Isometry3d query_from_submap = makePose(0.45, -0.25, 0.1, 20.0);
  const BtcFeatures query = transformFeatures(stored, query_from_submap);
  const MapDatabase database(snapshot, permissiveSearchConfig());

  const auto candidates = database.search(query);

  ASSERT_FALSE(candidates.empty());
  EXPECT_EQ(candidates.front().submap_id, 0U);
  EXPECT_GT(candidates.front().votes, 0U);
  EXPECT_GT(candidates.front().inliers, 0U);
  EXPECT_GE(candidates.front().mean_binary_similarity, 0.5);
  EXPECT_LE(candidates.front().vertex_rmse, 0.35);
  const Eigen::Isometry3d expected_submap_from_query = query_from_submap.inverse();
  expectPoseNear(candidates.front().submap_from_query, expected_submap_from_query, 0.08);
  expectPoseNear(candidates.front().map_from_query,
                 map_from_submap * expected_submap_from_query, 0.08);
}

TEST(MapDatabase, ReturnsEmptyCandidatesForEmptyQueryFeatures) {
  TempDirectory temp;
  const auto snapshot = saveTwoSubmapSnapshot(temp);
  const MapDatabase database(snapshot, permissiveSearchConfig());

  BtcFeatures empty_query;
  empty_query.submap_id = 999;
  const auto candidates = database.search(empty_query);

  EXPECT_TRUE(candidates.empty());
}

TEST(MapDatabase, RetrievesSecondSubmapWithDefaultSearchThresholds) {
  TempDirectory temp;
  const auto snapshot = saveTwoSubmapSnapshot(temp);
  const auto second = loadBtcFeatures(snapshot / "btc" / "000001.btc");
  ASSERT_GE(second.triangles.size(), 5U);
  const MapDatabase database(snapshot);

  const auto candidates = database.search(second);

  ASSERT_FALSE(candidates.empty());
  EXPECT_EQ(candidates.front().submap_id, 1U);
  EXPECT_GE(candidates.front().votes, 5U);
  EXPECT_GE(candidates.front().inliers, 4U);
  expectPoseNear(candidates.front().submap_from_query, Eigen::Isometry3d::Identity(), 0.08);
}

TEST(MapDatabase, ReturnsEmptyCandidatesWhenVoteThresholdIsNotReached) {
  TempDirectory temp;
  const auto snapshot = saveSingleSubmapSnapshot(temp, makeRoomWithPosts(), makePose());
  SearchConfig strict_search = permissiveSearchConfig();
  strict_search.min_votes = std::numeric_limits<std::size_t>::max() / 2U;
  const auto stored = loadBtcFeatures(snapshot / "btc" / "000000.btc");
  ASSERT_FALSE(stored.triangles.empty());
  const MapDatabase database(snapshot, strict_search);

  const auto candidates = database.search(stored);

  EXPECT_TRUE(candidates.empty());
}

TEST(MapDatabase, RejectsIncompleteSnapshotDirectory) {
  TempDirectory temp;
  const auto snapshot = saveSingleSubmapSnapshot(temp, makeRoomWithPosts(), makePose());
  writeText(snapshot / ".incomplete", "interrupted save\n");

  EXPECT_THROW(MapDatabase database(snapshot, permissiveSearchConfig()), std::runtime_error);
}

TEST(MapDatabase, RejectsMissingBtcDescriptorFile) {
  TempDirectory temp;
  const auto snapshot = saveSingleSubmapSnapshot(temp, makeRoomWithPosts(), makePose());
  std::filesystem::remove(snapshot / "btc" / "000000.btc");

  EXPECT_THROW(MapDatabase database(snapshot, permissiveSearchConfig()), std::runtime_error);
}

TEST(MapDatabase, RejectsCorruptedPoseRowId) {
  TempDirectory temp;
  const auto snapshot = saveSingleSubmapSnapshot(temp, makeRoomWithPosts(), makePose());
  auto poses = readCsv(snapshot / "poses.csv");
  ASSERT_GE(poses.size(), 2U);
  ASSERT_FALSE(poses[1].empty());
  poses[1][0] = "bad_id";
  writeCsv(snapshot / "poses.csv", poses);

  EXPECT_THROW(MapDatabase database(snapshot, permissiveSearchConfig()), std::runtime_error);
}

TEST(MapDatabase, RejectsZeroQuaternionInPoseCsv) {
  TempDirectory temp;
  const auto snapshot = saveSingleSubmapSnapshot(temp, makeRoomWithPosts(), makePose());
  auto poses = readCsv(snapshot / "poses.csv");
  ASSERT_GE(poses.size(), 2U);
  ASSERT_GE(poses[1].size(), 9U);
  poses[1][5] = "0";
  poses[1][6] = "0";
  poses[1][7] = "0";
  poses[1][8] = "0";
  writeCsv(snapshot / "poses.csv", poses);

  EXPECT_THROW(MapDatabase database(snapshot, permissiveSearchConfig()), std::runtime_error);
}

TEST(MapDatabase, RejectsMissingSnapshotPath) {
  TempDirectory temp;

  EXPECT_THROW(MapDatabase database(temp.path() / "does_not_exist", permissiveSearchConfig()),
               std::runtime_error);
}

TEST(RelocalizationProcessor, WaitsUntilConfiguredFrameCountBeforeProducingQuery) {
  TempDirectory temp;
  const auto snapshot = saveSingleSubmapSnapshot(temp, makeRoomWithPosts(), makePose());
  QueryConfig query;
  query.frames_per_query = 3;
  query.max_duration = 10.0;
  RelocalizationProcessor processor(snapshot, query, permissiveSearchConfig());
  const Cloud cloud = makeCloud({Eigen::Vector3f(0.0F, 0.0F, 0.0F)});

  EXPECT_FALSE(processor.process(cloud, makePose(), 100).has_value());
  EXPECT_FALSE(processor.process(cloud, makePose(), 200).has_value());
  const auto result = processor.process(cloud, makePose(), 300);

  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->stamp_ns, 300);
  EXPECT_EQ(result->frame_count, 3U);
}

TEST(RelocalizationProcessor, ProducesCandidateForExactRoomQueryPointCloud) {
  TempDirectory temp;
  const Cloud room = makeRoomWithPosts();
  const auto snapshot = saveSingleSubmapSnapshot(temp, room, makePose());
  QueryConfig query;
  query.frames_per_query = 1;
  query.max_duration = 10.0;
  RelocalizationProcessor processor(snapshot, query, permissiveSearchConfig());

  const auto result = processor.process(room, makePose(), 100);

  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->stamp_ns, 100);
  EXPECT_EQ(result->frame_count, 1U);
  EXPECT_FALSE(result->query_cloud.empty());
  EXPECT_GT(result->binary_count, 0U);
  EXPECT_GT(result->triangle_count, 0U);
  ASSERT_FALSE(result->candidates.empty());
  EXPECT_EQ(result->candidates.front().submap_id, 0U);
  expectPoseNear(result->candidates.front().submap_from_query, Eigen::Isometry3d::Identity(), 0.08);
  expectPoseNear(result->candidates.front().map_from_query, Eigen::Isometry3d::Identity(), 0.08);
}

TEST(RelocalizationProcessor, OrientationRetryPreservesSensorLocalCandidatePoses) {
  TempDirectory temp;
  const Cloud room = makeRoomWithPosts();
  const auto map_from_submap = makePose(4.0, -2.0, 0.5, 30.0);
  const auto snapshot = saveSingleSubmapSnapshot(temp, room, map_from_submap);
  auto submap_from_sensor = makePose(0.2, -0.1, 0.1);
  submap_from_sensor.linear() =
    (Eigen::AngleAxisd(20.0 * M_PI / 180.0, Eigen::Vector3d::UnitZ()) *
     Eigen::AngleAxisd(25.0 * M_PI / 180.0, Eigen::Vector3d::UnitY()) *
     Eigen::AngleAxisd(30.0 * M_PI / 180.0, Eigen::Vector3d::UnitX()))
      .toRotationMatrix();
  const auto input = transformCloud(room, submap_from_sensor.inverse());
  auto odom_from_sensor = submap_from_sensor;
  // Odom and map origins differ: normalization must use orientation alone.
  odom_from_sensor.translation() = Eigen::Vector3d(7.0, -4.0, 2.0);
  QueryConfig query;
  query.frames_per_query = 1;
  RelocalizationProcessor processor(snapshot, query);

  const auto result = processor.process(input, odom_from_sensor, 1000000000);

  ASSERT_TRUE(result);
  EXPECT_TRUE(result->used_orientation_retry);
  expectPoseNear(result->odom_from_query, odom_from_sensor, 1e-9);
  ASSERT_FALSE(result->candidates.empty());
  const auto& candidate = result->candidates.front();
  EXPECT_EQ(candidate.submap_id, 0U);
  const auto expect_sensor_pose = [](const Eigen::Isometry3d& actual,
                                      const Eigen::Isometry3d& expected) {
    EXPECT_LT((actual.translation() - expected.translation()).norm(), 0.1);
    const double angle_error =
      Eigen::AngleAxisd(actual.linear() * expected.linear().transpose()).angle();
    EXPECT_LT(angle_error, 3.0 * M_PI / 180.0);
  };
  expect_sensor_pose(candidate.submap_from_query, submap_from_sensor);
  expect_sensor_pose(candidate.map_from_query, map_from_submap * submap_from_sensor);
}

TEST(RelocalizationProcessor, QueriesEveryConfiguredNumberOfFrames) {
  TempDirectory temp;
  const Cloud room = makeRoomWithPosts();
  const auto snapshot = saveSingleSubmapSnapshot(temp, room, makePose());
  QueryConfig query;
  query.frames_per_query = 2;
  query.max_duration = 10.0;
  RelocalizationProcessor processor(snapshot, query, permissiveSearchConfig());

  EXPECT_FALSE(processor.process(room, makePose(), 100).has_value());
  ASSERT_TRUE(processor.process(room, makePose(), 200).has_value());
  EXPECT_FALSE(processor.process(room, makePose(), 300).has_value());
}

TEST(RelocalizationProcessor, RetainsDifferentViewsAcrossQueriesButDropsStaleHistory) {
  TempDirectory temp;
  const Cloud first = makeCloud({Eigen::Vector3f(1.0F, 0.0F, 0.0F)});
  const Cloud second = makeCloud({Eigen::Vector3f(0.0F, 2.0F, 0.0F)});
  const auto snapshot = saveSingleSubmapSnapshot(temp, first, makePose());
  QueryConfig query;
  query.frames_per_query = 1;
  query.max_duration = 0.5;
  query.history_timeout = 2.0;
  RelocalizationProcessor processor(snapshot, query);
  ASSERT_TRUE(processor.process(first, makePose(), 1000000000));
  const auto combined = processor.process(second, makePose(), 1100000000);
  ASSERT_TRUE(combined);
  ASSERT_EQ(combined->query_cloud.size(), 2U);
  EXPECT_EQ(combined->frame_count, 1U);  // New frames since the last query.
  EXPECT_EQ(combined->failure_reason, "no_descriptors");

  const auto after_short_gap = processor.process(second, makePose(), 2000000000);
  ASSERT_TRUE(after_short_gap);
  EXPECT_EQ(after_short_gap->query_cloud.size(), 2U);

  const auto restarted = processor.process(second, makePose(), 4100000000);
  ASSERT_TRUE(restarted);
  ASSERT_EQ(restarted->query_cloud.size(), 1U);
  EXPECT_NEAR(restarted->query_cloud.front().y, 2.0, 1e-5);
}

TEST(MapDatabase, ReportsVoteAndGeometryRejectionsAndResetsDiagnostics) {
  TempDirectory temp;
  const auto snapshot = saveSingleSubmapSnapshot(temp, makeRoomWithPosts(), makePose());
  const auto features = loadBtcFeatures(snapshot / "btc" / "000000.btc");
  ASSERT_GT(features.triangles.size(), 0U);
  SearchConfig config;
  config.min_votes = features.triangles.size() + 1;
  relocalization_processing::SearchDiagnostics diagnostics;
  MapDatabase votes_db(snapshot, config);
  EXPECT_TRUE(votes_db.search(features, diagnostics).empty());
  EXPECT_EQ(diagnostics.best_votes, features.triangles.size());
  EXPECT_EQ(diagnostics.best_length_votes, features.triangles.size());
  EXPECT_EQ(diagnostics.best_inliers, 0U);
  EXPECT_EQ(diagnostics.required_votes, config.min_votes);

  config.min_votes = 1;
  config.min_inliers = features.triangles.size() + 1;
  MapDatabase geometry_db(snapshot, config);
  EXPECT_TRUE(geometry_db.search(features, diagnostics).empty());
  EXPECT_GT(diagnostics.best_inliers, 0U);
  EXPECT_LT(diagnostics.best_inliers, diagnostics.required_inliers);
  EXPECT_TRUE(geometry_db.search(BtcFeatures{}, diagnostics).empty());
  EXPECT_EQ(diagnostics.best_votes, 0U);
  EXPECT_EQ(diagnostics.best_length_votes, 0U);
  EXPECT_EQ(diagnostics.best_inliers, 0U);
}

TEST(RelocalizationProcessor, CropsAccumulatedQueryAroundLatestSensorPose) {
  TempDirectory temp;
  const auto snapshot = saveSingleSubmapSnapshot(temp, makeRoomWithPosts(), makePose(), 1.0);
  QueryConfig query;
  query.frames_per_query = 2;
  query.max_duration = 10.0;
  RelocalizationProcessor processor(snapshot, query, permissiveSearchConfig());
  const Cloud old_near_first_pose = makeCloud({Eigen::Vector3f(0.6F, 0.0F, 0.0F)});

  EXPECT_FALSE(processor.process(old_near_first_pose, makePose(0.0, 0.0, 0.0), 100).has_value());
  const auto nearby_result = processor.process(Cloud{}, makePose(0.7, 0.0, 0.0), 200);
  ASSERT_TRUE(nearby_result.has_value());
  ASSERT_FALSE(nearby_result->query_cloud.empty());

  EXPECT_FALSE(processor.process(old_near_first_pose, makePose(0.0, 0.0, 0.0), 300).has_value());
  const auto result = processor.process(Cloud{}, makePose(5.0, 0.0, 0.0), 400);

  ASSERT_TRUE(result.has_value());
  EXPECT_TRUE(result->query_cloud.empty());
  EXPECT_TRUE(result->candidates.empty());
}

TEST(RelocalizationProcessor, InvalidPoseDoesNotPolluteAccumulationWindow) {
  TempDirectory temp;
  const auto snapshot = saveSingleSubmapSnapshot(temp, makeRoomWithPosts(), makePose());
  QueryConfig query;
  query.frames_per_query = 2;
  query.max_duration = 10.0;
  RelocalizationProcessor processor(snapshot, query, permissiveSearchConfig());
  Eigen::Isometry3d invalid_pose = Eigen::Isometry3d::Identity();
  invalid_pose.translation().x() = std::numeric_limits<double>::quiet_NaN();

  EXPECT_THROW(processor.process(makeRoomWithPosts(), invalid_pose, 100), std::invalid_argument);
  EXPECT_FALSE(processor.process(makeRoomWithPosts(), makePose(), 200).has_value());
}

TEST(RelocalizationProcessor, InvalidTimestampDoesNotPolluteAccumulationWindow) {
  TempDirectory temp;
  const auto snapshot = saveSingleSubmapSnapshot(temp, makeRoomWithPosts(), makePose());
  QueryConfig query;
  query.frames_per_query = 2;
  query.max_duration = 10.0;
  RelocalizationProcessor processor(snapshot, query, permissiveSearchConfig());

  EXPECT_THROW(processor.process(makeRoomWithPosts(), makePose(), 0), std::invalid_argument);
  EXPECT_FALSE(processor.process(makeRoomWithPosts(), makePose(), 100).has_value());
}

TEST(RelocalizationProcessor, ExpiredBatchRetainsPreviouslyScannedHistory) {
  TempDirectory temp;
  const auto snapshot = saveSingleSubmapSnapshot(
    temp, makeCloud({Eigen::Vector3f(0.6F, 0.0F, 0.0F)}), makePose());
  QueryConfig query;
  query.frames_per_query = 2;
  query.max_duration = 0.5;
  RelocalizationProcessor processor(snapshot, query);
  const Cloud first = makeCloud({Eigen::Vector3f(0.6F, 0.0F, 0.0F)});
  const Cloud second = makeCloud({Eigen::Vector3f(1.8F, 0.0F, 0.0F)});

  EXPECT_FALSE(processor.process(first, makePose(), 1000000000));
  EXPECT_FALSE(processor.process(second, makePose(), 2000000000));
  const auto result = processor.process(second, makePose(), 2100000000);
  ASSERT_TRUE(result);
  EXPECT_EQ(result->frame_count, 2U);
  ASSERT_EQ(result->query_cloud.size(), 2U);
  EXPECT_TRUE(std::any_of(result->query_cloud.begin(), result->query_cloud.end(),
    [](const auto& point) { return point.x < 1.0F; }));
}

TEST(RelocalizationProcessor, OptionalRealPcdRetrievesWithDefaultParameters) {
  const char* path = std::getenv("BTC_SMOKE_PCD");
  if (!path || !*path) {
    GTEST_SKIP() << "set BTC_SMOKE_PCD to a real local submap PCD";
  }
  Cloud cloud;
  ASSERT_EQ(pcl::io::loadPCDFile(path, cloud), 0);
  ASSERT_FALSE(cloud.empty());
  TempDirectory temp;
  SubmapBuilder builder;
  ASSERT_TRUE(builder.update(cloud, makePose(), 100).created);
  MapSaveOptions options;
  options.directory = temp.path();
  const auto snapshot = builder.save(cloud, options);
  QueryConfig query;
  query.frames_per_query = 1;
  for (const double yaw : {0.0, 5.0, 30.0, 60.0, 90.0, 135.0, 180.0}) {
    SCOPED_TRACE("yaw=" + std::to_string(yaw));
    const Eigen::Isometry3d map_from_query = makePose(0.0, 0.0, 0.0, yaw);
    const Cloud input = transformCloud(cloud, map_from_query.inverse());
    RelocalizationProcessor processor(snapshot, query);
    const auto result = processor.process(input, makePose(), 1000000000);
    ASSERT_TRUE(result);
    EXPECT_GT(result->triangle_count, 0U);
    ASSERT_FALSE(result->candidates.empty());
    const auto& candidate = result->candidates.front();
    EXPECT_EQ(candidate.submap_id, 0U);
    const double translation_error =
      (candidate.map_from_query.translation() - map_from_query.translation()).norm();
    const double angle_error = Eigen::AngleAxisd(
      candidate.map_from_query.linear() * map_from_query.linear().transpose()).angle();
    EXPECT_LE(translation_error, 0.25);
    EXPECT_LE(angle_error, 5.0 * M_PI / 180.0);
    std::cout << "Real PCD query: yaw=" << yaw
              << ", points=" << result->query_cloud.size()
              << ", triangles=" << result->triangle_count
              << ", inliers=" << candidate.inliers
              << ", rmse=" << candidate.vertex_rmse
              << ", translation_error=" << translation_error
              << ", angle_error_deg=" << angle_error * 180.0 / M_PI << '\n';
  }
}

TEST(RelocalizationProcessor, ResetStartsNewTimelineWithoutKeepingOldPoints) {
  TempDirectory temp;
  const auto cloud = makeCloud({Eigen::Vector3f(1.0F, 0.0F, 0.0F)});
  const auto snapshot = saveSingleSubmapSnapshot(temp, cloud, makePose());
  QueryConfig query;
  query.frames_per_query = 1;
  RelocalizationProcessor processor(snapshot, query);
  ASSERT_TRUE(processor.process(cloud, makePose(), 10000000000LL));
  processor.reset();
  const auto result = processor.process(
    makeCloud({Eigen::Vector3f(2.0F, 0.0F, 0.0F)}), makePose(), 1000000000LL);
  ASSERT_TRUE(result);
  EXPECT_EQ(processor.database().size(), 1U);
  ASSERT_EQ(result->query_cloud.size(), 1U);
  EXPECT_GT(result->query_cloud.front().x, 1.5F);
}
