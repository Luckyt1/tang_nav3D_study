#include "map/submap.hpp"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <initializer_list>
#include <iterator>
#include <limits>
#include <stdexcept>
#include <string>
#include <system_error>
#include <vector>

#include <Eigen/Geometry>
#include <gtest/gtest.h>
#include <pcl/io/pcd_io.h>

namespace {

using map_processing::Cloud;
using map_processing::MapSaveOptions;
using map_processing::SubmapBuilder;
using map_processing::SubmapConfig;

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

Eigen::Isometry3d makePose(double x, double y, double z, double yaw_deg = 0.0) {
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation() = Eigen::Vector3d(x, y, z);
  pose.linear() =
    Eigen::AngleAxisd(yaw_deg * M_PI / 180.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  return pose;
}

Eigen::Vector3f pointAt(const Cloud& cloud, std::size_t index = 0) {
  const auto& point = cloud.at(index);
  return Eigen::Vector3f(point.x, point.y, point.z);
}

void expectPointNear(const Eigen::Vector3f& actual, const Eigen::Vector3f& expected,
                     float tolerance = 1e-4F) {
  EXPECT_NEAR(actual.x(), expected.x(), tolerance);
  EXPECT_NEAR(actual.y(), expected.y(), tolerance);
  EXPECT_NEAR(actual.z(), expected.z(), tolerance);
}

SubmapConfig testConfig() {
  SubmapConfig config;
  config.translation_threshold = 2.0;
  config.radius = 5.0;
  return config;
}

class TempDirectory {
 public:
  TempDirectory()
  : path_(std::filesystem::temp_directory_path() /
          ("submap_builder_save_test_" +
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

std::string readText(const std::filesystem::path& path) {
  std::ifstream stream(path);
  return std::string(std::istreambuf_iterator<char>(stream), std::istreambuf_iterator<char>());
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

Cloud readPcd(const std::filesystem::path& path) {
  Cloud cloud;
  EXPECT_EQ(pcl::io::loadPCDFile<pcl::PointXYZ>(path.string(), cloud), 0);
  return cloud;
}

std::size_t countDirectories(const std::filesystem::path& path) {
  if (!std::filesystem::exists(path) || !std::filesystem::is_directory(path)) {
    return 0;
  }
  std::size_t count = 0;
  for (const auto& entry : std::filesystem::directory_iterator(path)) {
    if (entry.is_directory()) {
      ++count;
    }
  }
  return count;
}

}  // namespace

TEST(SubmapBuilder, RejectsInvalidConfiguration) {
  SubmapConfig config = testConfig();
  config.translation_threshold = 0.0;
  EXPECT_THROW(SubmapBuilder builder(config), std::invalid_argument);

  config = testConfig();
  config.radius = -1.0;
  EXPECT_THROW(SubmapBuilder builder(config), std::invalid_argument);

  config = testConfig();
  config.radius = std::numeric_limits<double>::infinity();
  EXPECT_THROW(SubmapBuilder builder(config), std::invalid_argument);
}

TEST(SubmapBuilder, CreatesInitialSubmapWhenFirstFrameHasPointsInsideRadius) {
  SubmapBuilder builder(testConfig());

  const auto update =
    builder.update(makeCloud({Eigen::Vector3f(1.0F, 0.0F, 0.0F)}), makePose(0.0, 0.0, 0.0), 1);

  ASSERT_TRUE(update.created);
  ASSERT_TRUE(update.active.has_value());
  EXPECT_FALSE(update.completed.has_value());
  EXPECT_EQ(update.active->pose.id, 0U);
  EXPECT_EQ(update.active->pose.stamp_ns, 1);
  EXPECT_EQ(builder.poses().size(), 1U);
  ASSERT_EQ(update.active->cloud.size(), 1U);
  expectPointNear(pointAt(update.active->cloud), Eigen::Vector3f(1.0F, 0.0F, 0.0F));
}

TEST(SubmapBuilder, DoesNotCreateInitialSubmapWhenNoPointsAreInsideRadius) {
  SubmapBuilder builder(testConfig());

  const auto update =
    builder.update(makeCloud({Eigen::Vector3f(5.1F, 0.0F, 0.0F)}), makePose(0.0, 0.0, 0.0), 1);

  EXPECT_FALSE(update.created);
  EXPECT_FALSE(update.active.has_value());
  EXPECT_FALSE(update.completed.has_value());
  EXPECT_TRUE(builder.poses().empty());
}

TEST(SubmapBuilder, IncludesPointsOnSphericalRadiusBoundary) {
  SubmapBuilder builder(testConfig());

  const auto update =
    builder.update(makeCloud({Eigen::Vector3f(3.0F, 4.0F, 0.0F)}), makePose(0.0, 0.0, 0.0), 1);

  ASSERT_TRUE(update.active.has_value());
  ASSERT_EQ(update.active->cloud.size(), 1U);
  expectPointNear(pointAt(update.active->cloud), Eigen::Vector3f(3.0F, 4.0F, 0.0F));
}

TEST(SubmapBuilder, EmitsEmptyActiveCloudWhenLatestMapBecomesEmpty) {
  SubmapBuilder builder(testConfig());
  ASSERT_TRUE(builder.update(makeCloud({Eigen::Vector3f(1.0F, 0.0F, 0.0F)}),
                             makePose(0.0, 0.0, 0.0), 1)
                .active.has_value());

  const auto update = builder.update(Cloud{}, makePose(0.5, 0.0, 0.0), 2);

  EXPECT_FALSE(update.created);
  ASSERT_TRUE(update.active.has_value());
  EXPECT_EQ(update.active->pose.id, 0U);
  EXPECT_TRUE(update.active->cloud.empty());
}

TEST(SubmapBuilder, CreatesNewSubmapWhenTranslationThresholdIsReached) {
  SubmapBuilder builder(testConfig());
  ASSERT_TRUE(builder.update(makeCloud({Eigen::Vector3f(1.0F, 0.0F, 0.0F)}),
                             makePose(0.0, 0.0, 0.0), 1)
                .created);

  const auto update =
    builder.update(makeCloud({Eigen::Vector3f(2.5F, 0.0F, 0.0F)}), makePose(2.0, 0.0, 0.0), 2);

  ASSERT_TRUE(update.created);
  ASSERT_TRUE(update.active.has_value());
  ASSERT_TRUE(update.completed.has_value());
  EXPECT_EQ(update.active->pose.id, 1U);
  EXPECT_EQ(update.completed->pose.id, 0U);
  EXPECT_EQ(builder.poses().size(), 2U);
}

TEST(SubmapBuilder, RotationInPlaceKeepsUpdatingOneSubmap) {
  SubmapBuilder builder(testConfig());
  ASSERT_TRUE(builder.update(makeCloud({Eigen::Vector3f(1.0F, 0.0F, 0.0F)}),
                             makePose(0.0, 0.0, 0.0), 1)
                .created);

  // 两整圈旋转，跨过 +/-180 度；新观测继续更新点云，锚点数量不能增长。
  for (int step = 1; step <= 24; ++step) {
    const float x = 1.0F + 0.05F * step;
    const auto update = builder.update(makeCloud({Eigen::Vector3f(x, 0.0F, 0.0F)}),
                                       makePose(0.0, 0.0, 0.0, step * 30.0), step + 1);
    EXPECT_FALSE(update.created);
    EXPECT_FALSE(update.completed.has_value());
    ASSERT_TRUE(update.active.has_value());
    EXPECT_EQ(update.active->pose.id, 0U);
    EXPECT_EQ(builder.poses().size(), 1U);
    EXPECT_TRUE(update.active->pose.map_from_submap.isApprox(makePose(0.0, 0.0, 0.0)));
    ASSERT_EQ(update.active->cloud.size(), 1U);
    expectPointNear(pointAt(update.active->cloud), Eigen::Vector3f(x, 0.0F, 0.0F));
  }
}

TEST(SubmapBuilder, RotationWithSmallPositionJitterWaitsForTranslationThreshold) {
  SubmapBuilder builder(testConfig());
  const auto map = makeCloud({Eigen::Vector3f(1.0F, 0.0F, 0.0F)});
  ASSERT_TRUE(builder.update(map, makePose(0.0, 0.0, 0.0), 1).created);

  for (int step = 1; step <= 12; ++step) {
    const double angle = step * M_PI / 6.0;
    const auto update = builder.update(
      map, makePose(0.3 * std::sin(angle), 0.3 * (1.0 - std::cos(angle)), 0.0, step * 30.0),
      step + 1);
    EXPECT_FALSE(update.created);
    ASSERT_TRUE(update.active.has_value());
    EXPECT_EQ(update.active->pose.id, 0U);
  }

  const auto moved = builder.update(map, makePose(2.0, 0.0, 0.0, 90.0), 14);
  EXPECT_TRUE(moved.created);
  ASSERT_TRUE(moved.active.has_value());
  EXPECT_EQ(moved.active->pose.id, 1U);
  EXPECT_EQ(builder.poses().size(), 2U);
}

TEST(SubmapBuilder, KeepsPreviousActiveSubmapWhenTriggeredPoseHasNoPoints) {
  SubmapBuilder builder(testConfig());
  ASSERT_TRUE(builder.update(makeCloud({Eigen::Vector3f(0.5F, 0.0F, 0.0F)}),
                             makePose(0.0, 0.0, 0.0), 1)
                .created);

  const auto update =
    builder.update(makeCloud({Eigen::Vector3f(0.5F, 0.0F, 0.0F)}), makePose(10.0, 0.0, 0.0), 2);

  EXPECT_FALSE(update.created);
  ASSERT_TRUE(update.active.has_value());
  EXPECT_EQ(update.active->pose.id, 0U);
  EXPECT_EQ(builder.poses().size(), 1U);
  ASSERT_EQ(update.active->cloud.size(), 1U);
  expectPointNear(pointAt(update.active->cloud), Eigen::Vector3f(0.5F, 0.0F, 0.0F));
}

TEST(SubmapBuilder, ConvertsPointsIntoAnchorLocalCoordinates) {
  SubmapBuilder builder(testConfig());
  const Eigen::Isometry3d anchor = makePose(1.0, 2.0, 0.0, 90.0);
  const Eigen::Vector3d local_point(1.0, 0.0, 0.5);
  const Eigen::Vector3d map_point = anchor * local_point;

  const auto update =
    builder.update(makeCloud({map_point.cast<float>()}), anchor, 1);

  ASSERT_TRUE(update.active.has_value());
  ASSERT_EQ(update.active->cloud.size(), 1U);
  expectPointNear(pointAt(update.active->cloud), local_point.cast<float>());
}

TEST(SubmapBuilder, ReplacesActiveCloudFromLatestMapInsteadOfAccumulatingFrames) {
  SubmapBuilder builder(testConfig());
  ASSERT_TRUE(builder.update(makeCloud({Eigen::Vector3f(1.0F, 0.0F, 0.0F)}),
                             makePose(0.0, 0.0, 0.0), 1)
                .created);

  const auto update =
    builder.update(makeCloud({Eigen::Vector3f(1.5F, 0.0F, 0.0F)}), makePose(0.1, 0.0, 0.0), 2);

  ASSERT_TRUE(update.active.has_value());
  ASSERT_EQ(update.active->cloud.size(), 1U);
  expectPointNear(pointAt(update.active->cloud), Eigen::Vector3f(1.5F, 0.0F, 0.0F));
}

TEST(SubmapBuilder, CompletesOldSubmapFromLatestMapWhenCreatingNextSubmap) {
  SubmapBuilder builder(testConfig());
  ASSERT_TRUE(builder.update(makeCloud({Eigen::Vector3f(1.0F, 0.0F, 0.0F)}),
                             makePose(0.0, 0.0, 0.0), 1)
                .created);

  const auto update =
    builder.update(makeCloud({Eigen::Vector3f(1.5F, 0.0F, 0.0F), Eigen::Vector3f(2.5F, 0.0F, 0.0F)}),
                   makePose(2.0, 0.0, 0.0), 2);

  ASSERT_TRUE(update.completed.has_value());
  ASSERT_EQ(update.completed->cloud.size(), 2U);
  expectPointNear(pointAt(update.completed->cloud, 0), Eigen::Vector3f(1.5F, 0.0F, 0.0F));
  expectPointNear(pointAt(update.completed->cloud, 1), Eigen::Vector3f(2.5F, 0.0F, 0.0F));
}

TEST(SubmapBuilder, ExtractSkipsInvalidPointsAndUsesStoredAnchorPose) {
  SubmapBuilder builder(testConfig());
  ASSERT_TRUE(builder.update(makeCloud({Eigen::Vector3f(1.0F, 0.0F, 0.0F)}),
                             makePose(1.0, 0.0, 0.0), 1)
                .created);

  Cloud latest_map = makeCloud({Eigen::Vector3f(2.0F, 0.0F, 0.0F), Eigen::Vector3f(3.0F, 0.0F, 0.0F)});
  latest_map.points[1].x = std::numeric_limits<float>::quiet_NaN();
  const Cloud extracted = builder.extract(latest_map, 0);

  ASSERT_EQ(extracted.size(), 1U);
  expectPointNear(pointAt(extracted), Eigen::Vector3f(1.0F, 0.0F, 0.0F));
}

TEST(SubmapBuilder, ThrowsForUnknownSubmapId) {
  SubmapBuilder builder(testConfig());

  EXPECT_THROW(builder.extract(Cloud{}, 42), std::out_of_range);
}

TEST(SubmapBuilder, RejectsNonIncreasingTimestampsWithoutChangingState) {
  SubmapBuilder builder(testConfig());
  ASSERT_TRUE(builder.update(makeCloud({Eigen::Vector3f(1.0F, 0.0F, 0.0F)}),
                             makePose(0.0, 0.0, 0.0), 10)
                .created);

  EXPECT_THROW(builder.update(Cloud{}, makePose(0.0, 0.0, 0.0), 10), std::invalid_argument);
  EXPECT_EQ(builder.poses().size(), 1U);

  const auto update =
    builder.update(makeCloud({Eigen::Vector3f(1.5F, 0.0F, 0.0F)}), makePose(0.0, 0.0, 0.0), 11);
  ASSERT_TRUE(update.active.has_value());
  EXPECT_EQ(update.active->pose.id, 0U);
  ASSERT_EQ(update.active->cloud.size(), 1U);
  expectPointNear(pointAt(update.active->cloud), Eigen::Vector3f(1.5F, 0.0F, 0.0F));
}

TEST(SubmapBuilder, RejectsInvalidPoseWithoutChangingState) {
  SubmapBuilder builder(testConfig());
  ASSERT_TRUE(builder.update(makeCloud({Eigen::Vector3f(1.0F, 0.0F, 0.0F)}),
                             makePose(0.0, 0.0, 0.0), 1)
                .created);
  Eigen::Isometry3d invalid_pose = makePose(0.0, 0.0, 0.0);
  invalid_pose.linear()(0, 0) = 2.0;

  EXPECT_THROW(builder.update(makeCloud({Eigen::Vector3f(3.0F, 0.0F, 0.0F)}), invalid_pose, 2),
               std::invalid_argument);
  EXPECT_EQ(builder.poses().size(), 1U);

  const auto update =
    builder.update(makeCloud({Eigen::Vector3f(1.5F, 0.0F, 0.0F)}), makePose(0.0, 0.0, 0.0), 2);
  ASSERT_TRUE(update.active.has_value());
  EXPECT_EQ(update.active->pose.id, 0U);
  ASSERT_EQ(update.active->cloud.size(), 1U);
  expectPointNear(pointAt(update.active->cloud), Eigen::Vector3f(1.5F, 0.0F, 0.0F));
}

TEST(SubmapBuilder, SaveWritesSnapshotFilesWithLatestSubmapCrops) {
  SubmapConfig config = testConfig();
  config.radius = 1.0;
  SubmapBuilder builder(config);
  ASSERT_TRUE(builder.update(makeCloud({Eigen::Vector3f(0.5F, 0.0F, 0.0F)}),
                             makePose(0.0, 0.0, 0.0), 100)
                .created);
  const Cloud latest_map =
    makeCloud({Eigen::Vector3f(0.75F, 0.0F, 0.0F), Eigen::Vector3f(2.0F, 0.25F, 0.0F)});
  ASSERT_TRUE(builder.update(latest_map, makePose(2.0, 0.0, 0.0, 90.0), 200).created);
  TempDirectory temp;
  MapSaveOptions options;
  options.directory = temp.path() / "maps";
  options.map_frame = "map";
  options.sensor_frame = "mid360";
  options.freedom.sensor_max_range = 7.5;
  options.freedom.counts_to_free = 4;

  const std::filesystem::path snapshot = builder.save(latest_map, options);

  EXPECT_TRUE(snapshot.is_absolute());
  EXPECT_TRUE(std::filesystem::exists(snapshot / "static_map.pcd"));
  EXPECT_TRUE(std::filesystem::exists(snapshot / "submaps" / "000000.pcd"));
  EXPECT_TRUE(std::filesystem::exists(snapshot / "submaps" / "000001.pcd"));
  EXPECT_TRUE(std::filesystem::exists(snapshot / "poses.csv"));
  EXPECT_TRUE(std::filesystem::exists(snapshot / "metadata.yaml"));
  EXPECT_FALSE(std::filesystem::exists(snapshot / ".incomplete"));

  const Cloud static_map = readPcd(snapshot / "static_map.pcd");
  ASSERT_EQ(static_map.size(), 2U);
  expectPointNear(pointAt(static_map, 0), Eigen::Vector3f(0.75F, 0.0F, 0.0F));
  expectPointNear(pointAt(static_map, 1), Eigen::Vector3f(2.0F, 0.25F, 0.0F));

  const Cloud first_submap = readPcd(snapshot / "submaps" / "000000.pcd");
  ASSERT_EQ(first_submap.size(), 1U);
  expectPointNear(pointAt(first_submap), Eigen::Vector3f(0.75F, 0.0F, 0.0F));
  const Cloud active_submap = readPcd(snapshot / "submaps" / "000001.pcd");
  ASSERT_EQ(active_submap.size(), 1U);
  expectPointNear(pointAt(active_submap), Eigen::Vector3f(0.25F, 0.0F, 0.0F));

  const auto rows = readCsv(snapshot / "poses.csv");
  ASSERT_EQ(rows.size(), 3U);
  EXPECT_EQ(rows[0], (std::vector<std::string>{"id", "stamp_ns", "tx", "ty", "tz", "qx", "qy",
                                               "qz", "qw", "point_count", "pcd_file"}));
  ASSERT_EQ(rows[1].size(), 11U);
  EXPECT_EQ(rows[1][0], "0");
  EXPECT_EQ(rows[1][1], "100");
  EXPECT_EQ(rows[1][9], "1");
  EXPECT_EQ(rows[1][10], "submaps/000000.pcd");
  ASSERT_EQ(rows[2].size(), 11U);
  EXPECT_EQ(rows[2][0], "1");
  EXPECT_EQ(rows[2][1], "200");
  EXPECT_DOUBLE_EQ(std::stod(rows[2][2]), 2.0);
  EXPECT_DOUBLE_EQ(std::stod(rows[2][5]), 0.0);
  EXPECT_DOUBLE_EQ(std::stod(rows[2][6]), 0.0);
  EXPECT_NEAR(std::stod(rows[2][7]), std::sqrt(0.5), 1e-12);
  EXPECT_NEAR(std::stod(rows[2][8]), std::sqrt(0.5), 1e-12);
  EXPECT_EQ(rows[2][9], "1");
  EXPECT_EQ(rows[2][10], "submaps/000001.pcd");

  const std::string metadata = readText(snapshot / "metadata.yaml");
  EXPECT_NE(metadata.find("format_version: 1"), std::string::npos);
  EXPECT_NE(metadata.find("map_frame: \"map\""), std::string::npos);
  EXPECT_NE(metadata.find("sensor_frame: \"mid360\""), std::string::npos);
  EXPECT_NE(metadata.find("snapshot_stamp_ns: 200"), std::string::npos);
  EXPECT_NE(metadata.find("anchor_count: 2"), std::string::npos);
  EXPECT_NE(metadata.find("saved_submap_count: 2"), std::string::npos);
  EXPECT_NE(metadata.find("static_point_count: 2"), std::string::npos);
  EXPECT_NE(metadata.find("translation_threshold"), std::string::npos);
  EXPECT_NE(metadata.find("radius"), std::string::npos);
  EXPECT_NE(metadata.find("sensor:\n"), std::string::npos);
  EXPECT_NE(metadata.find("  max_range: 7.5\n"), std::string::npos);
  EXPECT_NE(metadata.find("  counts_to_free: 4\n"), std::string::npos);
}

TEST(SubmapBuilder, SaveCreatesIndependentSnapshotDirectories) {
  SubmapBuilder builder(testConfig());
  ASSERT_TRUE(builder.update(makeCloud({Eigen::Vector3f(1.0F, 0.0F, 0.0F)}),
                             makePose(0.0, 0.0, 0.0), 10)
                .created);
  TempDirectory temp;
  MapSaveOptions options;
  options.directory = temp.path() / "maps";

  const std::filesystem::path first =
    builder.save(makeCloud({Eigen::Vector3f(1.0F, 0.0F, 0.0F)}), options);
  const std::filesystem::path second =
    builder.save(makeCloud({Eigen::Vector3f(2.0F, 0.0F, 0.0F)}), options);

  EXPECT_NE(first, second);
  EXPECT_TRUE(std::filesystem::exists(first / "static_map.pcd"));
  EXPECT_TRUE(std::filesystem::exists(second / "static_map.pcd"));
  ASSERT_EQ(readPcd(first / "static_map.pcd").size(), 1U);
  const Cloud second_map = readPcd(second / "static_map.pcd");
  ASSERT_EQ(second_map.size(), 1U);
  expectPointNear(pointAt(second_map), Eigen::Vector3f(2.0F, 0.0F, 0.0F));
}

TEST(SubmapBuilder, SaveKeepsEmptyAnchorInCsvAndOmitsEmptySubmapPcd) {
  SubmapConfig config = testConfig();
  config.radius = 1.0;
  SubmapBuilder builder(config);
  ASSERT_TRUE(builder.update(makeCloud({Eigen::Vector3f(0.5F, 0.0F, 0.0F)}),
                             makePose(0.0, 0.0, 0.0), 10)
                .created);
  const Cloud active_only = makeCloud({Eigen::Vector3f(2.25F, 0.0F, 0.0F)});
  ASSERT_TRUE(builder.update(active_only, makePose(2.0, 0.0, 0.0), 20).created);
  TempDirectory temp;
  MapSaveOptions options;
  options.directory = temp.path() / "maps";

  const std::filesystem::path snapshot = builder.save(active_only, options);

  EXPECT_FALSE(std::filesystem::exists(snapshot / "submaps" / "000000.pcd"));
  EXPECT_TRUE(std::filesystem::exists(snapshot / "submaps" / "000001.pcd"));
  const auto rows = readCsv(snapshot / "poses.csv");
  ASSERT_EQ(rows.size(), 3U);
  ASSERT_EQ(rows[1].size(), 11U);
  EXPECT_EQ(rows[1][0], "0");
  EXPECT_EQ(rows[1][9], "0");
  EXPECT_EQ(rows[1][10], "");
  ASSERT_EQ(rows[2].size(), 11U);
  EXPECT_EQ(rows[2][0], "1");
  EXPECT_EQ(rows[2][9], "1");
  EXPECT_EQ(rows[2][10], "submaps/000001.pcd");
}

TEST(SubmapBuilder, SaveRejectsBeforeFirstUpdateWithoutCreatingSnapshot) {
  SubmapBuilder builder(testConfig());
  TempDirectory temp;
  MapSaveOptions options;
  options.directory = temp.path() / "maps";

  EXPECT_THROW(builder.save(makeCloud({Eigen::Vector3f(1.0F, 0.0F, 0.0F)}), options),
               std::runtime_error);
  EXPECT_EQ(countDirectories(options.directory), 0U);
}

TEST(SubmapBuilder, SaveRejectsEmptyStaticMapWithoutCreatingSnapshot) {
  SubmapBuilder builder(testConfig());
  ASSERT_TRUE(builder.update(makeCloud({Eigen::Vector3f(1.0F, 0.0F, 0.0F)}),
                             makePose(0.0, 0.0, 0.0), 10)
                .created);
  TempDirectory temp;
  MapSaveOptions options;
  options.directory = temp.path() / "maps";

  EXPECT_THROW(builder.save(Cloud{}, options), std::runtime_error);
  EXPECT_EQ(countDirectories(options.directory), 0U);
}

TEST(SubmapBuilder, SaveFailureLeavesExistingOutputRootFileUntouched) {
  SubmapBuilder builder(testConfig());
  ASSERT_TRUE(builder.update(makeCloud({Eigen::Vector3f(1.0F, 0.0F, 0.0F)}),
                             makePose(0.0, 0.0, 0.0), 10)
                .created);
  TempDirectory temp;
  const std::filesystem::path output_file = temp.path() / "maps";
  {
    std::ofstream stream(output_file);
    stream << "already a file";
  }
  MapSaveOptions options;
  options.directory = output_file;

  EXPECT_THROW(builder.save(makeCloud({Eigen::Vector3f(1.0F, 0.0F, 0.0F)}), options),
               std::runtime_error);
  EXPECT_TRUE(std::filesystem::is_regular_file(output_file));
  EXPECT_EQ(readText(output_file), "already a file");
}
