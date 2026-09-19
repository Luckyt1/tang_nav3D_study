#include "relocalization/fine_registration.hpp"

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

#include <Eigen/Geometry>
#include <gtest/gtest.h>
#include <pcl/io/pcd_io.h>

namespace {

using map_processing::Cloud;
using relocalization_processing::Candidate;
using relocalization_processing::FineConfig;
using relocalization_processing::FineRegistration;
using relocalization_processing::MapDatabase;
using relocalization_processing::QueryResult;

class TempDirectory {
 public:
  TempDirectory()
  : path_(std::filesystem::temp_directory_path() /
          ("fine_registration_test_" + std::to_string(
            std::chrono::steady_clock::now().time_since_epoch().count()))) {
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

Eigen::Isometry3d pose(const Eigen::Vector3d& translation, double yaw_degrees = 0.0,
                       double pitch_degrees = 0.0, double roll_degrees = 0.0) {
  const double radians = M_PI / 180.0;
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.translation() = translation;
  result.linear() =
    (Eigen::AngleAxisd(yaw_degrees * radians, Eigen::Vector3d::UnitZ()) *
     Eigen::AngleAxisd(pitch_degrees * radians, Eigen::Vector3d::UnitY()) *
     Eigen::AngleAxisd(roll_degrees * radians, Eigen::Vector3d::UnitX()))
      .toRotationMatrix();
  return result;
}

Cloud transformCloud(const Cloud& cloud, const Eigen::Isometry3d& transform) {
  Cloud result;
  result.reserve(cloud.size());
  for (const auto& point : cloud) {
    const Eigen::Vector3d transformed =
      transform * Eigen::Vector3d(point.x, point.y, point.z);
    result.emplace_back(transformed.x(), transformed.y(), transformed.z());
  }
  result.width = static_cast<std::uint32_t>(result.size());
  result.height = 1;
  result.is_dense = true;
  return result;
}

void addBox(Cloud& cloud, const Eigen::Vector3d& center, const Eigen::Vector3d& size) {
  // Different face dimensions and tangential offsets avoid a repeated lattice.
  for (int normal_axis = 0; normal_axis < 3; ++normal_axis) {
    const int u_axis = (normal_axis + 1) % 3;
    const int v_axis = (normal_axis + 2) % 3;
    const int u_steps = static_cast<int>(std::ceil(size[u_axis] / 0.09));
    const int v_steps = static_cast<int>(std::ceil(size[v_axis] / 0.09));
    for (const double side : {-1.0, 1.0}) {
      for (int u = 1; u < u_steps; ++u) {
        for (int v = 1; v < v_steps; ++v) {
          Eigen::Vector3d point = center;
          point[normal_axis] += side * size[normal_axis] / 2.0;
          point[u_axis] += size[u_axis] * (static_cast<double>(u) / u_steps - 0.5) +
                           0.012 * std::sin(2.3 * u + 1.7 * v);
          point[v_axis] += size[v_axis] * (static_cast<double>(v) / v_steps - 0.5) +
                           0.012 * std::cos(1.1 * u - 2.7 * v);
          cloud.emplace_back(point.x(), point.y(), point.z());
        }
      }
    }
  }
}

Cloud makeAsymmetricScene() {
  Cloud cloud;
  addBox(cloud, Eigen::Vector3d(-1.4, -0.7, 0.8), Eigen::Vector3d(0.7, 0.9, 1.6));
  addBox(cloud, Eigen::Vector3d(0.5, 0.6, 1.1), Eigen::Vector3d(1.1, 0.5, 0.9));
  addBox(cloud, Eigen::Vector3d(1.6, -1.2, 0.5), Eigen::Vector3d(0.6, 0.8, 1.0));
  cloud.width = static_cast<std::uint32_t>(cloud.size());
  cloud.height = 1;
  cloud.is_dense = true;
  return cloud;
}

std::filesystem::path saveSnapshot(const TempDirectory& temp, const Cloud& cloud,
                                   const Eigen::Isometry3d& map_from_submap) {
  map_processing::SubmapConfig submaps;
  submaps.radius = 8.0;
  map_processing::SubmapBuilder builder(submaps);
  const auto map_cloud = transformCloud(cloud, map_from_submap);
  if (!builder.update(map_cloud, map_from_submap, 100).created) {
    throw std::runtime_error("failed to create test submap");
  }
  map_processing::MapSaveOptions options;
  options.directory = temp.path();
  return builder.save(map_cloud, options);
}

double translationError(const Eigen::Isometry3d& actual, const Eigen::Isometry3d& expected) {
  return (actual.translation() - expected.translation()).norm();
}

double rotationError(const Eigen::Isometry3d& actual, const Eigen::Isometry3d& expected) {
  return Eigen::AngleAxisd(actual.linear() * expected.linear().transpose()).angle();
}

class FineRegistrationTest : public ::testing::Test {
 protected:
  void SetUp() override {
    scene = makeAsymmetricScene();
    database = std::make_unique<MapDatabase>(saveSnapshot(temp, scene, map_from_submap));
    config.voxel_size = 0.035;
    config.inlier_distance = 0.08;
    config.min_overlap = 0.95;
    config.max_rmse = 0.025;
  }

  QueryResult makeQuery(bool perturb = true) const {
    QueryResult query;
    query.query_cloud = transformCloud(scene, submap_from_query.inverse());
    query.odom_from_query = pose(Eigen::Vector3d(11.0, -6.0, 2.0), 67.0, -7.0, 3.0);
    Candidate candidate;
    candidate.submap_id = 0;
    candidate.submap_from_query = submap_from_query;
    if (perturb) {
      candidate.submap_from_query =
        submap_from_query * pose(Eigen::Vector3d(0.035, -0.025, 0.02), 1.2, -0.6, 0.4);
    }
    candidate.map_from_query = map_from_submap * candidate.submap_from_query;
    query.candidates.push_back(candidate);
    return query;
  }

  TempDirectory temp;
  Cloud scene;
  const Eigen::Isometry3d map_from_submap = pose(Eigen::Vector3d(5.0, -3.0, 1.0), 37.0);
  const Eigen::Isometry3d submap_from_query =
    pose(Eigen::Vector3d(0.3, -0.4, 0.6), -18.0, 10.0, 4.0);
  std::unique_ptr<MapDatabase> database;
  FineConfig config;
};

TEST_F(FineRegistrationTest, RefinesPerturbedPoseAndComposesMapAndOdomTransforms) {
  const auto query = makeQuery();
  const auto coarse = query.candidates.front().submap_from_query;
  FineRegistration registration(*database, config);

  const auto result = registration.refine(query);

  ASSERT_TRUE(result.success) << result.failure_reason;
  EXPECT_TRUE(result.converged);
  EXPECT_TRUE(result.failure_reason.empty());
  EXPECT_EQ(result.submap_id, 0U);
  EXPECT_GE(result.inliers, config.min_points);
  EXPECT_GE(result.overlap, config.min_overlap);
  EXPECT_LE(result.rmse, config.max_rmse);
  EXPECT_LT(translationError(result.submap_from_query, submap_from_query),
            translationError(coarse, submap_from_query) * 0.4);
  EXPECT_LT(rotationError(result.submap_from_query, submap_from_query),
            rotationError(coarse, submap_from_query) * 0.4);
  EXPECT_LT(translationError(result.submap_from_query, submap_from_query), 0.015);
  EXPECT_LT(rotationError(result.submap_from_query, submap_from_query), 0.4 * M_PI / 180.0);

  const Eigen::Isometry3d expected_map_from_query = map_from_submap * submap_from_query;
  const Eigen::Isometry3d expected_map_from_odom =
    expected_map_from_query * query.odom_from_query.inverse();
  EXPECT_TRUE(result.map_from_query.matrix().isApprox(
    (map_from_submap * result.submap_from_query).matrix(), 1e-5));
  EXPECT_TRUE((result.map_from_odom * query.odom_from_query).matrix().isApprox(
    result.map_from_query.matrix(), 1e-5));
  EXPECT_LT(translationError(result.map_from_query, expected_map_from_query), 0.015);
  EXPECT_LT(translationError(result.map_from_odom, expected_map_from_odom), 0.08);

  // A cached target must produce the same registration on the next query.
  const auto repeated = registration.refine(query);
  ASSERT_TRUE(repeated.success) << repeated.failure_reason;
  EXPECT_TRUE(repeated.map_from_odom.matrix().isApprox(result.map_from_odom.matrix(), 1e-6));
}

TEST_F(FineRegistrationTest, RejectsMissingCandidates) {
  auto query = makeQuery();
  query.candidates.clear();
  const auto result = FineRegistration(*database, config).refine(query);
  EXPECT_FALSE(result.success);
  EXPECT_FALSE(result.failure_reason.empty());
}

TEST_F(FineRegistrationTest, RejectsSparseQuery) {
  auto query = makeQuery();
  query.query_cloud.resize(5);
  const auto result = FineRegistration(*database, config).refine(query);
  EXPECT_FALSE(result.success);
  EXPECT_FALSE(result.failure_reason.empty());
}

TEST_F(FineRegistrationTest, RejectsSparseTarget) {
  TempDirectory sparse_temp;
  auto sparse_scene = scene;
  sparse_scene.resize(5);
  MapDatabase sparse_database(saveSnapshot(sparse_temp, sparse_scene, map_from_submap));
  const auto result = FineRegistration(sparse_database, config).refine(makeQuery());
  EXPECT_FALSE(result.success);
  EXPECT_FALSE(result.failure_reason.empty());
}

TEST_F(FineRegistrationTest, RejectsCloudWithNoOverlap) {
  auto query = makeQuery(false);
  query.query_cloud = transformCloud(
    query.query_cloud, pose(Eigen::Vector3d(20.0, -15.0, 10.0)));
  const auto result = FineRegistration(*database, config).refine(query);
  EXPECT_FALSE(result.success);
  EXPECT_FALSE(result.failure_reason.empty());
}

TEST_F(FineRegistrationTest, RejectsLowOverlapDespiteAnAccuratelyAlignedSubset) {
  auto query = makeQuery(false);
  const auto displaced = transformCloud(
    query.query_cloud, pose(Eigen::Vector3d(20.0, -15.0, 10.0)));
  query.query_cloud.insert(query.query_cloud.end(), displaced.begin(), displaced.end());
  const auto result = FineRegistration(*database, config).refine(query);
  EXPECT_FALSE(result.success);
  EXPECT_GT(result.inliers, config.min_points);
  EXPECT_LT(result.overlap, config.min_overlap);
  EXPECT_FALSE(result.failure_reason.empty());
}

TEST_F(FineRegistrationTest, RejectsHighResidualEvenWhenMostPointsOverlap) {
  auto query = makeQuery(false);
  for (std::size_t index = 0; index < query.query_cloud.size(); ++index) {
    auto& point = query.query_cloud[index];
    point.x += static_cast<float>(0.015 * std::sin(1.3 * index));
    point.y += static_cast<float>(0.015 * std::cos(2.1 * index));
    point.z += static_cast<float>(0.015 * std::sin(0.7 * index));
  }
  config.max_rmse = 0.001;
  const auto result = FineRegistration(*database, config).refine(query);
  EXPECT_FALSE(result.success);
  EXPECT_TRUE(result.converged);
  EXPECT_GE(result.overlap, config.min_overlap);
  EXPECT_GT(result.rmse, config.max_rmse);
  EXPECT_EQ(result.failure_reason, "high_rmse");
}

TEST_F(FineRegistrationTest, RejectsExcessiveTranslationOrRotationCorrections) {
  const auto query = makeQuery();
  auto translation_limit = config;
  translation_limit.max_translation_correction = 0.005;
  const auto translation = FineRegistration(*database, translation_limit).refine(query);
  EXPECT_FALSE(translation.success);
  EXPECT_GT(translation.translation_correction, translation_limit.max_translation_correction);
  EXPECT_FALSE(translation.failure_reason.empty());

  auto rotation_limit = config;
  rotation_limit.max_rotation_correction = 0.001;
  const auto rotation = FineRegistration(*database, rotation_limit).refine(query);
  EXPECT_FALSE(rotation.success);
  EXPECT_GT(rotation.rotation_correction, rotation_limit.max_rotation_correction);
  EXPECT_FALSE(rotation.failure_reason.empty());
}

TEST_F(FineRegistrationTest, DoesNotAcceptIterationLimitAsConvergence) {
  config.max_iterations = 1;
  const auto result = FineRegistration(*database, config).refine(makeQuery());
  EXPECT_FALSE(result.success);
  EXPECT_FALSE(result.converged);
  EXPECT_FALSE(result.failure_reason.empty());
}

TEST_F(FineRegistrationTest, RejectsNonFiniteOrNonRigidInputPoses) {
  auto query = makeQuery();
  query.odom_from_query.translation().x() = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(FineRegistration(*database, config).refine(query), std::invalid_argument);

  query = makeQuery();
  query.candidates.front().submap_from_query.linear() *= 2.0;
  EXPECT_THROW(FineRegistration(*database, config).refine(query), std::invalid_argument);

  query = makeQuery();
  query.candidates.front().map_from_query.linear().col(0) *= -1.0;
  EXPECT_THROW(FineRegistration(*database, config).refine(query), std::invalid_argument);

  query = makeQuery();
  query.odom_from_query.matrix()(3, 0) = 0.1;
  EXPECT_THROW(FineRegistration(*database, config).refine(query), std::invalid_argument);
}

TEST_F(FineRegistrationTest, RejectsInvalidConfiguration) {
  std::vector<std::pair<std::string, FineConfig>> invalid;
  const auto add = [&](const std::string& name, auto change) {
    auto value = config;
    change(value);
    invalid.emplace_back(name, value);
  };
  add("zero voxel", [](auto& value) { value.voxel_size = 0.0; });
  add("negative correspondence", [](auto& value) { value.max_correspondence_distance = -1.0; });
  add("zero iterations", [](auto& value) { value.max_iterations = 0; });
  add("zero candidates", [](auto& value) { value.max_candidates = 0; });
  add("zero minimum points", [](auto& value) { value.min_points = 0; });
  add("zero inlier distance", [](auto& value) { value.inlier_distance = 0.0; });
  add("excessive overlap", [](auto& value) { value.min_overlap = 1.1; });
  add("negative RMSE", [](auto& value) { value.max_rmse = -0.1; });
  add("negative translation", [](auto& value) { value.max_translation_correction = -0.1; });
  add("negative rotation", [](auto& value) { value.max_rotation_correction = -0.1; });
  add("NaN overlap", [](auto& value) {
    value.min_overlap = std::numeric_limits<double>::quiet_NaN();
  });
  add("infinite correspondence", [](auto& value) {
    value.max_correspondence_distance = std::numeric_limits<double>::infinity();
  });
  for (const auto& [name, value] : invalid) {
    SCOPED_TRACE(name);
    EXPECT_THROW(FineRegistration(*database, value), std::invalid_argument);
  }
}

TEST(FineRegistration, OptionalRealPcdRefinesPerturbedPose) {
  const char* path = std::getenv("BTC_SMOKE_PCD");
  if (path == nullptr || *path == '\0') {
    GTEST_SKIP() << "set BTC_SMOKE_PCD to a real local submap PCD";
  }
  Cloud cloud;
  ASSERT_EQ(pcl::io::loadPCDFile(path, cloud), 0);
  ASSERT_FALSE(cloud.empty());
  TempDirectory temp;
  const auto map_from_submap = pose(Eigen::Vector3d(3.0, -1.0, 0.5), 20.0);
  MapDatabase database(saveSnapshot(temp, cloud, map_from_submap));
  const auto submap_from_query = pose(Eigen::Vector3d(0.1, -0.1, 0.0), -15.0, 4.0, 2.0);
  QueryResult query;
  query.query_cloud = transformCloud(database.loadSubmap(0), submap_from_query.inverse());
  query.odom_from_query = pose(Eigen::Vector3d(7.0, 2.0, 1.0), 30.0);
  Candidate candidate;
  candidate.submap_from_query =
    submap_from_query * pose(Eigen::Vector3d(0.04, -0.03, 0.02), 1.5);
  candidate.map_from_query = map_from_submap * candidate.submap_from_query;
  query.candidates.push_back(candidate);

  const auto result = FineRegistration(database).refine(query);

  ASSERT_TRUE(result.success) << result.failure_reason;
  EXPECT_LT(translationError(result.submap_from_query, submap_from_query), 0.03);
  EXPECT_LT(rotationError(result.submap_from_query, submap_from_query), 0.5 * M_PI / 180.0);
  EXPECT_TRUE((result.map_from_odom * query.odom_from_query).matrix().isApprox(
    result.map_from_query.matrix(), 1e-5));
}

}  // namespace
