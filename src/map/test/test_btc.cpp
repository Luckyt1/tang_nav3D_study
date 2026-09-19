#include "map/submap.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <initializer_list>
#include <iterator>
#include <limits>
#include <optional>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <system_error>
#include <vector>

#include <Eigen/Geometry>
#include <gtest/gtest.h>
#include <pcl/io/pcd_io.h>

namespace {

using map_processing::BtcConfig;
using map_processing::BtcFeatures;
using map_processing::Cloud;
using map_processing::MapSaveOptions;
using map_processing::SubmapBuilder;
using map_processing::SubmapConfig;
using map_processing::extractBtcFeatures;
using map_processing::loadBtcFeatures;
using map_processing::validateBtcConfig;

constexpr double kTolerance = 1e-9;

class TempDirectory {
 public:
  TempDirectory()
  : path_(std::filesystem::temp_directory_path() /
          ("btc_test_" +
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

Eigen::Isometry3d makePose(double x = 0.0, double y = 0.0, double z = 0.0) {
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation() = Eigen::Vector3d(x, y, z);
  return pose;
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

SubmapConfig testSubmapConfig() {
  SubmapConfig config;
  config.translation_threshold = 10.0;
  config.radius = 8.0;
  return config;
}

MapSaveOptions saveOptions(const TempDirectory& temp, BtcConfig btc = testBtcConfig()) {
  MapSaveOptions options;
  options.directory = temp.path();
  options.btc = btc;
  return options;
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
  cloud.width = static_cast<std::uint32_t>(cloud.size());
  cloud.height = 1;
  cloud.is_dense = true;
  return cloud;
}

std::string readText(const std::filesystem::path& path) {
  std::ifstream stream(path);
  return std::string(std::istreambuf_iterator<char>(stream), std::istreambuf_iterator<char>());
}

void writeText(const std::filesystem::path& path, const std::string& text) {
  std::ofstream stream(path);
  stream << text;
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

std::size_t countDirectories(const std::filesystem::path& path) {
  std::size_t count = 0;
  for (const auto& entry : std::filesystem::directory_iterator(path)) {
    if (entry.is_directory()) {
      ++count;
    }
  }
  return count;
}

std::uint64_t parseUnsigned(const std::string& text) {
  return static_cast<std::uint64_t>(std::stoull(text));
}

std::size_t countOccupiedBits(const BinaryDescriptor& descriptor) {
  return static_cast<std::size_t>(
    std::count(descriptor.occupy_array_.begin(), descriptor.occupy_array_.end(), true));
}

void expectVectorNear(const Eigen::Vector3d& actual, const Eigen::Vector3d& expected) {
  EXPECT_NEAR(actual.x(), expected.x(), kTolerance);
  EXPECT_NEAR(actual.y(), expected.y(), kTolerance);
  EXPECT_NEAR(actual.z(), expected.z(), kTolerance);
}

void expectBinaryDescriptorNear(const BinaryDescriptor& actual, const BinaryDescriptor& expected) {
  expectVectorNear(actual.location_, expected.location_);
  EXPECT_EQ(actual.summary_, expected.summary_);
  ASSERT_EQ(actual.occupy_array_.size(), expected.occupy_array_.size());
  for (std::size_t i = 0; i < actual.occupy_array_.size(); ++i) {
    EXPECT_EQ(actual.occupy_array_[i], expected.occupy_array_[i]);
  }
}

void expectBinaryDescriptorsNear(const std::vector<BinaryDescriptor>& actual,
                                 const std::vector<BinaryDescriptor>& expected) {
  ASSERT_EQ(actual.size(), expected.size());
  for (std::size_t i = 0; i < actual.size(); ++i) {
    expectBinaryDescriptorNear(actual[i], expected[i]);
  }
}

void expectTriangleNear(const BTC& actual, const BTC& expected) {
  expectVectorNear(actual.triangle_, expected.triangle_);
  expectVectorNear(actual.angle_, expected.angle_);
  expectVectorNear(actual.center_, expected.center_);
  EXPECT_EQ(actual.frame_number_, expected.frame_number_);
  expectBinaryDescriptorNear(actual.binary_A_, expected.binary_A_);
  expectBinaryDescriptorNear(actual.binary_B_, expected.binary_B_);
  expectBinaryDescriptorNear(actual.binary_C_, expected.binary_C_);
}

void expectTrianglesNear(const std::vector<BTC>& actual, const std::vector<BTC>& expected) {
  ASSERT_EQ(actual.size(), expected.size());
  for (std::size_t i = 0; i < actual.size(); ++i) {
    expectTriangleNear(actual[i], expected[i]);
  }
}

void expectPlanesNear(const pcl::PointCloud<pcl::PointXYZINormal>& actual,
                      const pcl::PointCloud<pcl::PointXYZINormal>& expected) {
  ASSERT_EQ(actual.size(), expected.size());
  for (std::size_t i = 0; i < actual.size(); ++i) {
    EXPECT_NEAR(actual[i].x, expected[i].x, kTolerance);
    EXPECT_NEAR(actual[i].y, expected[i].y, kTolerance);
    EXPECT_NEAR(actual[i].z, expected[i].z, kTolerance);
    EXPECT_NEAR(actual[i].intensity, expected[i].intensity, kTolerance);
    EXPECT_NEAR(actual[i].normal_x, expected[i].normal_x, kTolerance);
    EXPECT_NEAR(actual[i].normal_y, expected[i].normal_y, kTolerance);
    EXPECT_NEAR(actual[i].normal_z, expected[i].normal_z, kTolerance);
    EXPECT_NEAR(actual[i].curvature, expected[i].curvature, kTolerance);
  }
}

void expectFeaturesNear(const BtcFeatures& actual, const BtcFeatures& expected) {
  EXPECT_EQ(actual.submap_id, expected.submap_id);
  expectBinaryDescriptorsNear(actual.binary, expected.binary);
  expectTrianglesNear(actual.triangles, expected.triangles);
  expectPlanesNear(actual.planes, expected.planes);
}

void expectBinaryDescriptorsAreFinite(const std::vector<BinaryDescriptor>& descriptors) {
  for (const auto& descriptor : descriptors) {
    EXPECT_TRUE(descriptor.location_.allFinite());
    EXPECT_EQ(countOccupiedBits(descriptor), static_cast<std::size_t>(descriptor.summary_));
  }
}

void expectTrianglesAreFinite(const std::vector<BTC>& triangles) {
  for (const auto& triangle : triangles) {
    EXPECT_TRUE(triangle.triangle_.allFinite());
    EXPECT_TRUE(triangle.angle_.allFinite());
    EXPECT_TRUE(triangle.center_.allFinite());
    EXPECT_GT(triangle.triangle_.minCoeff(), 0.0);
    EXPECT_LE(triangle.triangle_[0], triangle.triangle_[1] + kTolerance);
    EXPECT_LE(triangle.triangle_[1], triangle.triangle_[2] + kTolerance);
    expectBinaryDescriptorsAreFinite(
      {triangle.binary_A_, triangle.binary_B_, triangle.binary_C_});
  }
}

void expectPlanesAreFinite(const pcl::PointCloud<pcl::PointXYZINormal>& planes) {
  for (const auto& plane : planes) {
    EXPECT_TRUE(plane.getVector3fMap().allFinite());
    EXPECT_TRUE(plane.getNormalVector3fMap().allFinite());
    EXPECT_TRUE(std::isfinite(plane.intensity));
    EXPECT_TRUE(std::isfinite(plane.curvature));
  }
}

void expectFeaturesAreFinite(const BtcFeatures& features) {
  expectBinaryDescriptorsAreFinite(features.binary);
  expectTrianglesAreFinite(features.triangles);
  expectPlanesAreFinite(features.planes);
}

std::optional<std::filesystem::path> optionalLocalSubmapFixture() {
  const char* path = std::getenv("BTC_SMOKE_PCD");
  if (path == nullptr || std::string(path).empty()) {
    return std::nullopt;
  }
  return std::filesystem::path(path);
}

}  // namespace

TEST(BtcExtraction, ReturnsEmptyFeaturesForEmptyCloudAndPreservesSubmapId) {
  const auto features = extractBtcFeatures(Cloud{}, 70000, testBtcConfig());

  EXPECT_EQ(features.submap_id, 70000U);
  EXPECT_TRUE(features.binary.empty());
  EXPECT_TRUE(features.triangles.empty());
  EXPECT_TRUE(features.planes.empty());
}

TEST(BtcExtraction, HandlesTooSmallCloudsWithoutCrashing) {
  const auto config = testBtcConfig();

  for (const auto& cloud : {
         makeCloud({Eigen::Vector3f(0.0F, 0.0F, 0.0F)}),
         makeCloud({Eigen::Vector3f(0.0F, 0.0F, 0.0F), Eigen::Vector3f(0.1F, 0.0F, 0.0F)}),
         makeCloud({Eigen::Vector3f(0.0F, 0.0F, 0.0F), Eigen::Vector3f(0.1F, 0.0F, 0.0F),
                    Eigen::Vector3f(0.2F, 0.0F, 0.0F)}),
       }) {
    const auto features = extractBtcFeatures(cloud, 3, config);

    EXPECT_EQ(features.submap_id, 3U);
    EXPECT_TRUE(features.triangles.empty());
    expectFeaturesAreFinite(features);
  }
}

TEST(BtcExtraction, RejectsNonFinitePoints) {
  auto cloud = makeCloud({Eigen::Vector3f(0.0F, 0.0F, 0.0F)});
  cloud.push_back(pcl::PointXYZ(std::numeric_limits<float>::quiet_NaN(), 0.0F, 0.0F));
  cloud.width = static_cast<std::uint32_t>(cloud.size());

  EXPECT_THROW(extractBtcFeatures(cloud, 0, testBtcConfig()), std::invalid_argument);
}

TEST(BtcExtraction, RejectsInvalidConfigurationValues) {
  EXPECT_NO_THROW(validateBtcConfig(testBtcConfig()));

  auto config = testBtcConfig();
  config.useful_corner_num = 2;
  EXPECT_THROW(validateBtcConfig(config), std::invalid_argument);

  config = testBtcConfig();
  config.voxel_size = 0.0;
  EXPECT_THROW(validateBtcConfig(config), std::invalid_argument);

  config = testBtcConfig();
  config.projection_distance_min = config.projection_distance_max;
  EXPECT_THROW(validateBtcConfig(config), std::invalid_argument);

  config = testBtcConfig();
  config.summary_min_threshold = 255;
  EXPECT_THROW(validateBtcConfig(config), std::invalid_argument);

  config = testBtcConfig();
  config.triangle_resolution = std::numeric_limits<double>::infinity();
  EXPECT_THROW(validateBtcConfig(config), std::invalid_argument);
}

TEST(BtcExtraction, GeneratesFiniteTrianglesForSyntheticRoomWithPosts) {
  const auto features = extractBtcFeatures(makeRoomWithPosts(), 70000, testBtcConfig());

  EXPECT_EQ(features.submap_id, 70000U);
  EXPECT_FALSE(features.binary.empty());
  EXPECT_FALSE(features.planes.empty());
  ASSERT_FALSE(features.triangles.empty());
  for (const auto& triangle : features.triangles) {
    EXPECT_EQ(triangle.frame_number_, 70000U);
  }
  expectFeaturesAreFinite(features);
}

TEST(BtcExtraction, ShufflingInputPreservesSyntheticRoomFeatures) {
  const auto cloud = makeRoomWithPosts();
  const auto config = testBtcConfig();
  const auto expected = extractBtcFeatures(cloud, 70000, config);
  ASSERT_FALSE(expected.binary.empty());
  ASSERT_FALSE(expected.triangles.empty());

  for (const auto seed : {7U, 42U, 2026U}) {
    SCOPED_TRACE("shuffle_seed=" + std::to_string(seed));
    auto shuffled = cloud;
    std::mt19937 generator(seed);
    std::shuffle(shuffled.begin(), shuffled.end(), generator);

    expectFeaturesNear(extractBtcFeatures(shuffled, 70000, config), expected);
  }
}

TEST(BtcExtraction, EqualScoreNearbyCornersKeepOneStableRepresentative) {
  auto cloud = makeCloud({Eigen::Vector3f(-3.0F, -3.0F, 0.0F)});
  for (const auto& center : {Eigen::Vector2f(-1.0F, -1.0F),
                             Eigen::Vector2f(1.0F, -1.0F),
                             Eigen::Vector2f(0.0F, 1.0F)}) {
    for (int height = 0; height < 15; ++height) {
      cloud.emplace_back(center.x(), center.y(), 0.025F + 0.1F * height);
    }
  }
  cloud.width = static_cast<std::uint32_t>(cloud.size());
  auto config = testBtcConfig();
  // Isolate corner suppression using the extractor's horizontal-plane fallback.
  config.voxel_init_num = 10000;
  config.projection_plane_num = 1;
  config.non_max_suppression_radius = 0.01;
  const auto separated = extractBtcFeatures(cloud, 0, config);
  ASSERT_GT(separated.binary.size(), 1U);
  for (const auto& corner : separated.binary) {
    ASSERT_EQ(corner.summary_, separated.binary.front().summary_);
  }

  config.non_max_suppression_radius = 10.0;
  const auto suppressed = extractBtcFeatures(cloud, 0, config);
  ASSERT_EQ(suppressed.binary.size(), 1U);
  EXPECT_EQ(suppressed.binary.front().summary_, separated.binary.front().summary_);

  std::reverse(cloud.begin(), cloud.end());
  expectBinaryDescriptorsNear(extractBtcFeatures(cloud, 0, config).binary,
                              suppressed.binary);
}

TEST(BtcExtraction, OptionalLocalSubmapShufflePreservesFeatures) {
  const auto maybe_path = optionalLocalSubmapFixture();
  if (!maybe_path.has_value()) {
    GTEST_SKIP() << "set BTC_SMOKE_PCD to run the optional real PCD shuffle regression";
  }
  ASSERT_TRUE(std::filesystem::is_regular_file(*maybe_path)) << *maybe_path;
  Cloud cloud;
  ASSERT_EQ(pcl::io::loadPCDFile<pcl::PointXYZ>(maybe_path->string(), cloud), 0)
    << *maybe_path;
  const auto expected = extractBtcFeatures(cloud, 0, BtcConfig{});
  ASSERT_FALSE(expected.binary.empty());
  ASSERT_FALSE(expected.triangles.empty());

  std::mt19937 generator(42U);
  std::shuffle(cloud.begin(), cloud.end(), generator);

  expectFeaturesNear(extractBtcFeatures(cloud, 0, BtcConfig{}), expected);
}

TEST(BtcExtraction, OptionalLocalSubmapFixtureProducesFiniteFeatures) {
  const auto maybe_path = optionalLocalSubmapFixture();
  if (!maybe_path.has_value()) {
    GTEST_SKIP() << "set BTC_SMOKE_PCD to run the optional real PCD BTC smoke test";
  }
  ASSERT_TRUE(std::filesystem::exists(*maybe_path)) << *maybe_path;
  Cloud cloud;
  ASSERT_EQ(pcl::io::loadPCDFile<pcl::PointXYZ>(maybe_path->string(), cloud), 0)
    << *maybe_path;

  const auto features = extractBtcFeatures(cloud, 0, BtcConfig{});

  std::cout << "BTC_SMOKE_PCD counts: binary=" << features.binary.size()
            << " triangles=" << features.triangles.size()
            << " planes=" << features.planes.size() << '\n';
  SCOPED_TRACE("binary_count=" + std::to_string(features.binary.size()) +
               " triangle_count=" + std::to_string(features.triangles.size()) +
               " plane_count=" + std::to_string(features.planes.size()));
  EXPECT_FALSE(features.triangles.empty());
  expectFeaturesAreFinite(features);
}

TEST(BtcLoad, PreservesLargeSubmapIdsAboveUint16Range) {
  TempDirectory temp;
  const auto path = temp.path() / "large_id.btc";
  writeText(path,
            "MAP_BTC 1\n"
            "submap_id 70000\n"
            "binary_count 0\n"
            "triangle_count 0\n"
            "plane_count 0\n"
            "end\n");

  const auto features = loadBtcFeatures(path);

  EXPECT_EQ(features.submap_id, 70000U);
  EXPECT_TRUE(features.binary.empty());
  EXPECT_TRUE(features.triangles.empty());
  EXPECT_TRUE(features.planes.empty());
}

TEST(BtcLoad, RejectsUnsupportedFormatVersion) {
  TempDirectory temp;
  const auto path = temp.path() / "bad_version.btc";
  writeText(path,
            "MAP_BTC 2\n"
            "submap_id 0\n"
            "binary_count 0\n"
            "triangle_count 0\n"
            "plane_count 0\n"
            "end\n");

  EXPECT_THROW(loadBtcFeatures(path), std::runtime_error);
}

TEST(BtcLoad, RejectsTruncatedFiles) {
  TempDirectory temp;
  const auto path = temp.path() / "truncated.btc";
  writeText(path,
            "MAP_BTC 1\n"
            "submap_id 0\n"
            "binary_count 1\n"
            "binary 0 0 0 0 0 -\n");

  EXPECT_THROW(loadBtcFeatures(path), std::runtime_error);
}

TEST(BtcLoad, RejectsOccupancySummaryMismatch) {
  TempDirectory temp;
  const auto path = temp.path() / "summary_mismatch.btc";
  writeText(path,
            "MAP_BTC 1\n"
            "submap_id 0\n"
            "binary_count 1\n"
            "binary 0 0 0 1 3 101\n"
            "triangle_count 0\n"
            "plane_count 0\n"
            "end\n");

  EXPECT_THROW(loadBtcFeatures(path), std::runtime_error);
}

TEST(BtcSave, WritesLoadableDescriptorsManifestAndIndex) {
  TempDirectory temp;
  SubmapBuilder builder(testSubmapConfig());
  const auto cloud = makeRoomWithPosts();
  const auto btc_config = testBtcConfig();
  const auto expected = extractBtcFeatures(cloud, 0, btc_config);
  ASSERT_TRUE(builder.update(cloud, makePose(), 1).created);

  const auto snapshot = builder.save(cloud, saveOptions(temp, btc_config));

  const auto manifest = readCsv(snapshot / "btc" / "manifest.csv");
  ASSERT_EQ(manifest.size(), 2U);
  EXPECT_EQ(manifest[0], (std::vector<std::string>{
                           "id", "status", "binary_count", "triangle_count", "plane_count",
                           "descriptor_file"}));
  EXPECT_EQ(manifest[1][0], "0");
  EXPECT_EQ(manifest[1][1], "ready");

  const auto descriptor_path = snapshot / manifest[1][5];
  ASSERT_TRUE(std::filesystem::is_regular_file(descriptor_path));
  const auto features = loadBtcFeatures(descriptor_path);

  EXPECT_EQ(features.submap_id, 0U);
  EXPECT_EQ(parseUnsigned(manifest[1][2]), features.binary.size());
  EXPECT_EQ(parseUnsigned(manifest[1][3]), features.triangles.size());
  EXPECT_EQ(parseUnsigned(manifest[1][4]), features.planes.size());
  ASSERT_FALSE(features.triangles.empty());
  expectFeaturesAreFinite(features);
  expectFeaturesNear(features, expected);

  const auto index = readCsv(snapshot / "btc" / "index.csv");
  ASSERT_EQ(index.size(), features.triangles.size() + 1U);
  EXPECT_EQ(index[0], (std::vector<std::string>{
                        "bucket_x", "bucket_y", "bucket_z", "submap_id", "triangle_id"}));
  BtcDescManager manager;
  manager.AddBtcDescs(features.triangles);
  for (std::size_t i = 0; i < features.triangles.size(); ++i) {
    const auto& row = index[i + 1U];
    ASSERT_EQ(row.size(), 5U);
    const auto bucket_x = static_cast<std::int64_t>(features.triangles[i].triangle_[0] + 0.5);
    const auto bucket_y = static_cast<std::int64_t>(features.triangles[i].triangle_[1] + 0.5);
    const auto bucket_z = static_cast<std::int64_t>(features.triangles[i].triangle_[2] + 0.5);
    EXPECT_EQ(std::stoll(row[0]), bucket_x);
    EXPECT_EQ(std::stoll(row[1]), bucket_y);
    EXPECT_EQ(std::stoll(row[2]), bucket_z);
    EXPECT_EQ(row[3], "0");
    EXPECT_EQ(parseUnsigned(row[4]), i);
    const auto bucket = manager.data_base_.find(BTC_LOC(bucket_x, bucket_y, bucket_z));
    ASSERT_NE(bucket, manager.data_base_.end());
    EXPECT_FALSE(bucket->second.empty());
  }

  const auto metadata = readText(snapshot / "metadata.yaml");
  EXPECT_NE(metadata.find("btc:\n  enabled: true"), std::string::npos);
  EXPECT_NE(metadata.find("  descriptor_format: MAP_BTC"), std::string::npos);
  EXPECT_NE(metadata.find("  manifest_file: btc/manifest.csv"), std::string::npos);
  EXPECT_NE(metadata.find("  index_file: btc/index.csv"), std::string::npos);
}

TEST(BtcSave, WritesEmptyDescriptorForEmptyHistoricalSubmap) {
  TempDirectory temp;
  SubmapConfig submap_config = testSubmapConfig();
  submap_config.translation_threshold = 2.0;
  submap_config.radius = 0.75;
  SubmapBuilder builder(submap_config);
  ASSERT_TRUE(builder.update(makeCloud({Eigen::Vector3f(0.0F, 0.0F, 0.0F)}), makePose(), 1)
                .created);
  ASSERT_TRUE(builder.update(makeCloud({Eigen::Vector3f(2.0F, 0.0F, 0.0F)}),
                             makePose(2.0, 0.0, 0.0), 2)
                .created);

  const auto snapshot =
    builder.save(makeCloud({Eigen::Vector3f(2.0F, 0.0F, 0.0F)}), saveOptions(temp));

  const auto manifest = readCsv(snapshot / "btc" / "manifest.csv");
  ASSERT_EQ(manifest.size(), 3U);
  EXPECT_EQ(manifest[1][0], "0");
  EXPECT_EQ(manifest[1][1], "empty");
  EXPECT_EQ(manifest[1][2], "0");
  EXPECT_EQ(manifest[1][3], "0");
  EXPECT_EQ(manifest[1][4], "0");
  EXPECT_EQ(manifest[1][5], "btc/000000.btc");
  EXPECT_TRUE(std::filesystem::is_regular_file(snapshot / "btc" / "000000.btc"));

  const auto features = loadBtcFeatures(snapshot / "btc" / "000000.btc");
  EXPECT_EQ(features.submap_id, 0U);
  EXPECT_TRUE(features.binary.empty());
  EXPECT_TRUE(features.triangles.empty());
  EXPECT_TRUE(features.planes.empty());
}

TEST(BtcSave, UsesLatestCloudInsteadOfCachedActiveSubmapForBtcStatus) {
  TempDirectory temp;
  SubmapConfig submap_config = testSubmapConfig();
  submap_config.radius = 0.75;
  SubmapBuilder builder(submap_config);
  ASSERT_TRUE(builder.update(makeCloud({Eigen::Vector3f(0.0F, 0.0F, 0.0F)}), makePose(), 1)
                .created);

  const auto snapshot =
    builder.save(makeCloud({Eigen::Vector3f(5.0F, 0.0F, 0.0F)}), saveOptions(temp));

  const auto manifest = readCsv(snapshot / "btc" / "manifest.csv");
  ASSERT_EQ(manifest.size(), 2U);
  EXPECT_EQ(manifest[1][0], "0");
  EXPECT_EQ(manifest[1][1], "empty");
  const auto features = loadBtcFeatures(snapshot / "btc" / "000000.btc");
  EXPECT_TRUE(features.binary.empty());
  EXPECT_TRUE(features.triangles.empty());
  EXPECT_TRUE(features.planes.empty());
}

TEST(BtcSave, DoesNotCreateBtcDirectoryWhenDisabled) {
  TempDirectory temp;
  SubmapBuilder builder(testSubmapConfig());
  ASSERT_TRUE(builder.update(makeCloud({Eigen::Vector3f(0.0F, 0.0F, 0.0F)}), makePose(), 1)
                .created);
  auto btc = testBtcConfig();
  btc.enabled = false;

  const auto snapshot =
    builder.save(makeCloud({Eigen::Vector3f(0.0F, 0.0F, 0.0F)}), saveOptions(temp, btc));

  EXPECT_FALSE(std::filesystem::exists(snapshot / "btc"));
  const auto metadata = readText(snapshot / "metadata.yaml");
  EXPECT_NE(metadata.find("btc:\n  enabled: false"), std::string::npos);
  EXPECT_NE(metadata.find("  manifest_file: null"), std::string::npos);
  EXPECT_NE(metadata.find("  index_file: null"), std::string::npos);
}

TEST(BtcSave, CleansIncompleteSnapshotWhenBtcExtractionFails) {
  TempDirectory temp;
  writeText(temp.path() / "root_sentinel.txt", "keep root file\n");
  std::filesystem::create_directory(temp.path() / "old_snapshot");
  writeText(temp.path() / "old_snapshot" / "sentinel.txt", "keep old snapshot\n");

  SubmapConfig submap_config;
  submap_config.translation_threshold = 10.0;
  submap_config.radius = 100000.0;
  SubmapBuilder builder(submap_config);
  const auto sparse_huge_span = makeCloud({
    Eigen::Vector3f(0.0F, 0.0F, 0.0F),
    Eigen::Vector3f(10000.0F, 10000.0F, 0.0F),
    Eigen::Vector3f(20000.0F, 20000.0F, 0.0F),
    Eigen::Vector3f(30000.0F, 30000.0F, 0.0F),
    Eigen::Vector3f(40000.0F, 40000.0F, 0.0F),
    Eigen::Vector3f(50000.0F, 50000.0F, 0.0F),
    Eigen::Vector3f(60000.0F, 60000.0F, 0.0F),
  });
  ASSERT_TRUE(builder.update(sparse_huge_span, makePose(), 1).created);

  EXPECT_THROW(builder.save(sparse_huge_span, saveOptions(temp)), std::runtime_error);

  EXPECT_TRUE(std::filesystem::is_regular_file(temp.path() / "root_sentinel.txt"));
  EXPECT_TRUE(std::filesystem::is_regular_file(temp.path() / "old_snapshot" / "sentinel.txt"));
  EXPECT_EQ(countDirectories(temp.path()), 1U);
}
