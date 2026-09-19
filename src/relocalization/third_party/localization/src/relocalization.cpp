#include "relocalization/relocalization.hpp"

#include <algorithm>
#include <array>
#include <cerrno>
#include <charconv>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <limits>
#include <locale>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <string>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#include <opencv2/core/persistence.hpp>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>

namespace relocalization_processing {
namespace {

struct BucketKey {
  std::int64_t x{0};
  std::int64_t y{0};
  std::int64_t z{0};

  bool operator==(const BucketKey& other) const {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct BucketHash {
  std::size_t operator()(const BucketKey& key) const {
    std::size_t seed = 1469598103934665603ULL;
    const auto mix = [&seed](std::int64_t value) {
      seed ^= static_cast<std::uint64_t>(value) + 0x9e3779b97f4a7c15ULL +
              (seed << 6) + (seed >> 2);
    };
    mix(key.x);
    mix(key.y);
    mix(key.z);
    return seed;
  }
};

struct Match {
  std::size_t query_index{0};
  std::size_t map_index{0};
  const BTC* query{nullptr};
  const BTC* map{nullptr};
  double similarity{0.0};
};

struct PoseRow {
  std::uint64_t id{0};
  std::int64_t stamp_ns{0};
  Eigen::Isometry3d map_from_submap{Eigen::Isometry3d::Identity()};
  std::size_t point_count{0};
  std::filesystem::path pcd_file;
};

struct SubmapRecord {
  PoseRow pose;
  map_processing::BtcFeatures features;
  std::filesystem::path descriptor_file;
  mutable std::optional<Cloud> cached_cloud;
};

bool finite(double value) { return std::isfinite(value); }

void require(bool condition, const std::string& message) {
  if (!condition) {
    throw std::invalid_argument(message);
  }
}

void validateRigid(const Eigen::Isometry3d& pose, const std::string& name) {
  const Eigen::Matrix4d matrix = pose.matrix();
  require(matrix.allFinite(), name + " must contain only finite values");
  require(matrix.row(3).isApprox(Eigen::RowVector4d(0.0, 0.0, 0.0, 1.0)),
          name + " must be an isometry");
  const Eigen::Matrix3d rotation = pose.linear();
  require((rotation.transpose() * rotation).isApprox(Eigen::Matrix3d::Identity(), 1e-6),
          name + " rotation must be orthonormal");
  require(std::abs(rotation.determinant() - 1.0) < 1e-6,
          name + " rotation determinant must be 1");
}

void validateSearchConfig(const SearchConfig& config) {
  require(config.top_k > 0, "SearchConfig.top_k must be positive");
  require(config.min_votes > 0, "SearchConfig.min_votes must be positive");
  require(config.min_inliers > 0, "SearchConfig.min_inliers must be positive");
  require(config.max_hypotheses > 0, "SearchConfig.max_hypotheses must be positive");
  require(config.max_matches_per_candidate > 0,
          "SearchConfig.max_matches_per_candidate must be positive");
  require(finite(config.length_tolerance) && config.length_tolerance > 0.0 &&
              config.length_tolerance < 1.0,
          "SearchConfig.length_tolerance must be finite and in (0, 1)");
  require(finite(config.binary_similarity) && config.binary_similarity >= 0.0 &&
              config.binary_similarity <= 1.0,
          "SearchConfig.binary_similarity must be finite and in [0, 1]");
  require(finite(config.max_vertex_error) && config.max_vertex_error > 0.0,
          "SearchConfig.max_vertex_error must be positive and finite");
}

void validateQueryConfig(const QueryConfig& config) {
  require(config.frames_per_query > 0, "QueryConfig.frames_per_query must be positive");
  require(finite(config.max_duration) && config.max_duration > 0.0,
          "QueryConfig.max_duration must be positive and finite");
  require(finite(config.history_timeout) && config.history_timeout > 0.0,
          "QueryConfig.history_timeout must be positive and finite");
}

template <typename UInt>
UInt parseUnsigned(const std::string& value, const std::string& field) {
  UInt parsed{};
  const auto result = std::from_chars(value.data(), value.data() + value.size(), parsed);
  if (result.ec != std::errc{} || result.ptr != value.data() + value.size()) {
    throw std::runtime_error("invalid unsigned integer in " + field + ": " + value);
  }
  return parsed;
}

std::int64_t parseInt64(const std::string& value, const std::string& field) {
  std::int64_t parsed{};
  const auto result = std::from_chars(value.data(), value.data() + value.size(), parsed);
  if (result.ec != std::errc{} || result.ptr != value.data() + value.size()) {
    throw std::runtime_error("invalid integer in " + field + ": " + value);
  }
  return parsed;
}

double parseDouble(const std::string& value, const std::string& field) {
  if (value.empty()) {
    throw std::runtime_error("invalid finite double in " + field + ": " + value);
  }
  char* end = nullptr;
  errno = 0;
  const double parsed = std::strtod(value.c_str(), &end);
  if (errno != 0 || end != value.c_str() + value.size() || !std::isfinite(parsed)) {
    throw std::runtime_error("invalid finite double in " + field + ": " + value);
  }
  return parsed;
}

std::vector<std::string> splitCsvLine(const std::string& line) {
  std::vector<std::string> fields;
  std::string field;
  std::istringstream stream(line);
  while (std::getline(stream, field, ',')) {
    fields.push_back(field);
  }
  if (!line.empty() && line.back() == ',') {
    fields.emplace_back();
  }
  return fields;
}

std::filesystem::path safeRelativePath(const std::string& value, const std::string& field,
                                       bool allow_empty = false) {
  if (value.empty()) {
    if (allow_empty) {
      return {};
    }
    throw std::runtime_error(field + " must not be empty");
  }
  std::filesystem::path path(value);
  if (path.is_absolute()) {
    throw std::runtime_error(field + " must be relative: " + value);
  }
  const auto normalized = path.lexically_normal();
  for (const auto& part : normalized) {
    if (part == "..") {
      throw std::runtime_error(field + " must stay inside the snapshot: " + value);
    }
  }
  return normalized;
}

std::string readTextFile(const std::filesystem::path& path) {
  std::ifstream stream(path);
  if (!stream) {
    throw std::runtime_error("cannot open file: " + path.string());
  }
  std::ostringstream buffer;
  buffer << stream.rdbuf();
  return buffer.str();
}

std::string quoteOpenCvUnsafeMetadataScalars(const std::string& text) {
  std::istringstream input(text);
  std::ostringstream output;
  std::string line;
  while (std::getline(input, line)) {
    const auto key = line.find("upstream_commit:");
    if (key != std::string::npos) {
      const auto value_begin = line.find_first_not_of(" \t", key + std::string("upstream_commit:").size());
      if (value_begin != std::string::npos && line[value_begin] != '"' && line[value_begin] != '\'') {
        line.insert(line.size(), 1, '"');
        line.insert(value_begin, 1, '"');
      }
    }
    output << line << '\n';
  }
  return output.str();
}

std::string nodeString(const cv::FileNode& node, const std::string& field) {
  if (node.empty()) {
    throw std::runtime_error("missing metadata field: " + field);
  }
  if (node.isString()) {
    return static_cast<std::string>(node);
  }
  if (node.isInt()) {
    return std::to_string(static_cast<int>(node));
  }
  if (node.isReal()) {
    std::ostringstream stream;
    stream.imbue(std::locale::classic());
    stream << std::setprecision(17) << static_cast<double>(node);
    return stream.str();
  }
  throw std::runtime_error("unsupported metadata field type: " + field);
}

bool nodeBool(const cv::FileNode& node, const std::string& field) {
  const std::string value = nodeString(node, field);
  if (value == "true" || value == "1") {
    return true;
  }
  if (value == "false" || value == "0") {
    return false;
  }
  throw std::runtime_error("invalid boolean metadata field: " + field);
}

int nodeInt(const cv::FileNode& node, const std::string& field) {
  const auto value = parseInt64(nodeString(node, field), field);
  if (value < std::numeric_limits<int>::min() || value > std::numeric_limits<int>::max()) {
    throw std::runtime_error("metadata integer out of range: " + field);
  }
  return static_cast<int>(value);
}

double nodeDouble(const cv::FileNode& node, const std::string& field) {
  return parseDouble(nodeString(node, field), field);
}

void validateQuaternionOrder(const cv::FileNode& node) {
  if (node.empty()) {
    return;
  }
  if (!node.isSeq() || node.size() != 4) {
    throw std::runtime_error("unsupported quaternion_order; expected [qx,qy,qz,qw]");
  }
  const std::array<std::string, 4> expected{"qx", "qy", "qz", "qw"};
  for (int i = 0; i < 4; ++i) {
    if (nodeString(node[i], "quaternion_order") != expected[static_cast<std::size_t>(i)]) {
      throw std::runtime_error("unsupported quaternion_order; expected [qx,qy,qz,qw]");
    }
  }
}

std::filesystem::path metadataPath(const cv::FileStorage& fs, const std::string& field,
                                   bool allow_null = false) {
  const auto node = fs[field];
  if (node.empty()) {
    throw std::runtime_error("missing metadata field: " + field);
  }
  if (allow_null && node.isNone()) {
    return {};
  }
  const std::string value = nodeString(node, field);
  if (allow_null && value == "null") {
    return {};
  }
  return safeRelativePath(value, field);
}

struct Metadata {
  std::string map_frame;
  std::filesystem::path poses_file;
  std::filesystem::path static_map_file;
  std::size_t static_point_count{0};
  double radius{0.0};
  map_processing::FreedomConfig freedom;
  map_processing::BtcConfig btc;
  int btc_extraction_revision{1};
  std::filesystem::path manifest_file;
};

Metadata loadMetadata(const std::filesystem::path& snapshot) {
  const auto text =
    "%YAML:1.0\n" + quoteOpenCvUnsafeMetadataScalars(readTextFile(snapshot / "metadata.yaml"));
  cv::FileStorage fs(text, cv::FileStorage::READ | cv::FileStorage::MEMORY);
  if (!fs.isOpened()) {
    throw std::runtime_error("cannot parse metadata.yaml");
  }
  if (nodeInt(fs["format_version"], "format_version") != 1) {
    throw std::runtime_error("unsupported map snapshot format_version");
  }

  Metadata metadata;
  metadata.map_frame = nodeString(fs["map_frame"], "map_frame");
  if (metadata.map_frame.empty()) {
    throw std::runtime_error("metadata map_frame must not be empty");
  }
  if (nodeString(fs["submap_coordinates"], "submap_coordinates") != "local") {
    throw std::runtime_error("unsupported submap_coordinates; expected local");
  }
  if (nodeString(fs["pose_convention"], "pose_convention") != "T_map_submap") {
    throw std::runtime_error("unsupported pose_convention; expected T_map_submap");
  }
  validateQuaternionOrder(fs["quaternion_order"]);
  metadata.poses_file = metadataPath(fs, "poses_file");
  metadata.static_map_file = metadataPath(fs, "static_map_file");
  metadata.static_point_count = parseUnsigned<std::size_t>(
    nodeString(fs["static_point_count"], "static_point_count"), "static_point_count");

  const auto submap = fs["submap"];
  if (submap.empty()) {
    throw std::runtime_error("missing metadata submap section");
  }
  metadata.radius = nodeDouble(submap["radius"], "submap.radius");
  if (!finite(metadata.radius) || metadata.radius <= 0.0) {
    throw std::runtime_error("metadata submap.radius must be positive and finite");
  }

  const auto sensor = fs["sensor"];
  const auto freedom = fs["freedom"];
  if (sensor.empty() || freedom.empty()) {
    throw std::runtime_error("missing metadata sensor/freedom section");
  }
  metadata.freedom.sensor_min_range = nodeDouble(sensor["min_range"], "sensor.min_range");
  metadata.freedom.sensor_max_range = nodeDouble(sensor["max_range"], "sensor.max_range");
  metadata.freedom.sensor_min_z = nodeDouble(sensor["min_z"], "sensor.min_z");
  metadata.freedom.sensor_max_z = nodeDouble(sensor["max_z"], "sensor.max_z");
  metadata.freedom.sub_voxel_size = nodeDouble(freedom["sub_voxel_size"], "freedom.sub_voxel_size");
  metadata.freedom.counts_to_free = nodeInt(freedom["counts_to_free"], "freedom.counts_to_free");
  metadata.freedom.counts_to_revert =
    nodeInt(freedom["counts_to_revert"], "freedom.counts_to_revert");
  metadata.freedom.num_threads = nodeInt(freedom["num_threads"], "freedom.num_threads");
  if (nodeBool(freedom["raycast_enhancement"], "freedom.raycast_enhancement")) {
    throw std::runtime_error("unsupported freedom.raycast_enhancement=true in saved map");
  }

  const auto btc = fs["btc"];
  if (btc.empty() || !nodeBool(btc["enabled"], "btc.enabled")) {
    throw std::runtime_error(
      "snapshot BTC descriptors are missing or disabled; save the map again with btc.enabled=true");
  }
  if (nodeString(btc["descriptor_format"], "btc.descriptor_format") != "MAP_BTC" ||
      nodeInt(btc["descriptor_format_version"], "btc.descriptor_format_version") != 1) {
    throw std::runtime_error(
      "unsupported BTC descriptor format; save the map again with the current map package");
  }
  if (nodeString(btc["coordinates"], "btc.coordinates") != "submap_local") {
    throw std::runtime_error("unsupported BTC coordinates; expected submap_local");
  }
  if (!btc["extraction_revision"].empty()) {
    metadata.btc_extraction_revision =
      nodeInt(btc["extraction_revision"], "btc.extraction_revision");
  }
  if (metadata.btc_extraction_revision < 1 ||
      metadata.btc_extraction_revision > map_processing::kBtcExtractionRevision) {
    throw std::runtime_error("unsupported BTC extraction_revision");
  }
  metadata.manifest_file = safeRelativePath(
    nodeString(btc["manifest_file"], "btc.manifest_file"), "btc.manifest_file");
  const auto index_file = safeRelativePath(
    nodeString(btc["index_file"], "btc.index_file"), "btc.index_file");
  (void)index_file;

  metadata.btc.enabled = true;
  metadata.btc.useful_corner_num = nodeInt(btc["useful_corner_num"], "btc.useful_corner_num");
  metadata.btc.plane_detection_threshold =
    nodeDouble(btc["plane_detection_threshold"], "btc.plane_detection_threshold");
  metadata.btc.plane_merge_normal_threshold =
    nodeDouble(btc["plane_merge_normal_threshold"], "btc.plane_merge_normal_threshold");
  metadata.btc.plane_merge_distance_threshold =
    nodeDouble(btc["plane_merge_distance_threshold"], "btc.plane_merge_distance_threshold");
  metadata.btc.voxel_size = nodeDouble(btc["voxel_size"], "btc.voxel_size");
  metadata.btc.voxel_init_num = nodeInt(btc["voxel_init_num"], "btc.voxel_init_num");
  metadata.btc.projection_plane_num =
    nodeInt(btc["projection_plane_num"], "btc.projection_plane_num");
  metadata.btc.projection_resolution =
    nodeDouble(btc["projection_resolution"], "btc.projection_resolution");
  metadata.btc.projection_height_increment =
    nodeDouble(btc["projection_height_increment"], "btc.projection_height_increment");
  metadata.btc.projection_distance_min =
    nodeDouble(btc["projection_distance_min"], "btc.projection_distance_min");
  metadata.btc.projection_distance_max =
    nodeDouble(btc["projection_distance_max"], "btc.projection_distance_max");
  metadata.btc.summary_min_threshold =
    nodeInt(btc["summary_min_threshold"], "btc.summary_min_threshold");
  metadata.btc.line_filter_enabled =
    nodeBool(btc["line_filter_enabled"], "btc.line_filter_enabled");
  metadata.btc.descriptor_near_num =
    nodeInt(btc["descriptor_near_num"], "btc.descriptor_near_num");
  metadata.btc.descriptor_min_length =
    nodeDouble(btc["descriptor_min_length"], "btc.descriptor_min_length");
  metadata.btc.descriptor_max_length =
    nodeDouble(btc["descriptor_max_length"], "btc.descriptor_max_length");
  metadata.btc.non_max_suppression_radius =
    nodeDouble(btc["non_max_suppression_radius"], "btc.non_max_suppression_radius");
  metadata.btc.triangle_resolution =
    nodeDouble(btc["triangle_resolution"], "btc.triangle_resolution");

  map_processing::FreedomProcessor validator(metadata.freedom);
  (void)validator;
  map_processing::validateBtcConfig(metadata.btc);
  return metadata;
}

std::unordered_map<std::uint64_t, PoseRow> loadPoses(
  const std::filesystem::path& snapshot, const std::filesystem::path& relative_path) {
  std::ifstream stream(snapshot / relative_path);
  stream.imbue(std::locale::classic());
  if (!stream) {
    throw std::runtime_error("cannot open poses.csv");
  }
  std::string line;
  if (!std::getline(stream, line) ||
      line != "id,stamp_ns,tx,ty,tz,qx,qy,qz,qw,point_count,pcd_file") {
    throw std::runtime_error("unsupported poses.csv header");
  }
  std::unordered_map<std::uint64_t, PoseRow> poses;
  while (std::getline(stream, line)) {
    if (line.empty()) {
      continue;
    }
    const auto fields = splitCsvLine(line);
    if (fields.size() != 11) {
      throw std::runtime_error("invalid poses.csv row");
    }
    PoseRow row;
    row.id = parseUnsigned<std::uint64_t>(fields[0], "poses.id");
    row.stamp_ns = parseInt64(fields[1], "poses.stamp_ns");
    if (row.stamp_ns <= 0) {
      throw std::runtime_error("poses.csv stamp_ns must be positive");
    }
    const Eigen::Vector3d translation(parseDouble(fields[2], "poses.tx"),
                                      parseDouble(fields[3], "poses.ty"),
                                      parseDouble(fields[4], "poses.tz"));
    Eigen::Quaterniond q(parseDouble(fields[8], "poses.qw"),
                         parseDouble(fields[5], "poses.qx"),
                         parseDouble(fields[6], "poses.qy"),
                         parseDouble(fields[7], "poses.qz"));
    if (!q.coeffs().allFinite() || q.norm() < 1e-9) {
      throw std::runtime_error("poses.csv quaternion must be finite and nonzero");
    }
    q.normalize();
    row.map_from_submap = Eigen::Isometry3d::Identity();
    row.map_from_submap.translation() = translation;
    row.map_from_submap.linear() = q.toRotationMatrix();
    validateRigid(row.map_from_submap, "poses.csv map_from_submap");
    row.point_count = parseUnsigned<std::size_t>(fields[9], "poses.point_count");
    row.pcd_file = safeRelativePath(fields[10], "poses.pcd_file", true);
    if (row.point_count == 0 && !row.pcd_file.empty()) {
      throw std::runtime_error("poses.csv empty submap must not reference a PCD file");
    }
    if (row.point_count > 0 && row.pcd_file.empty()) {
      throw std::runtime_error("poses.csv non-empty submap must reference a PCD file");
    }
    if (!row.pcd_file.empty() && !std::filesystem::is_regular_file(snapshot / row.pcd_file)) {
      throw std::runtime_error("missing submap PCD: " + row.pcd_file.string());
    }
    if (!poses.emplace(row.id, std::move(row)).second) {
      throw std::runtime_error("duplicate submap id in poses.csv");
    }
  }
  return poses;
}

BucketKey bucketFor(const BTC& triangle) {
  return BucketKey{static_cast<std::int64_t>(triangle.triangle_[0] + 0.5),
                   static_cast<std::int64_t>(triangle.triangle_[1] + 0.5),
                   static_cast<std::int64_t>(triangle.triangle_[2] + 0.5)};
}

bool sameBitShape(const BinaryDescriptor& lhs, const BinaryDescriptor& rhs) {
  return lhs.occupy_array_.size() == rhs.occupy_array_.size();
}

double descriptorSimilarity(const BTC& query, const BTC& map) {
  if (!sameBitShape(query.binary_A_, map.binary_A_) ||
      !sameBitShape(query.binary_B_, map.binary_B_) ||
      !sameBitShape(query.binary_C_, map.binary_C_)) {
    return 0.0;
  }
  return (binary_similarity(query.binary_A_, map.binary_A_) +
          binary_similarity(query.binary_B_, map.binary_B_) +
          binary_similarity(query.binary_C_, map.binary_C_)) / 3.0;
}

Eigen::Isometry3d solveSubmapFromQuery(const BTC& query, const BTC& map) {
  Eigen::Matrix3d src = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d ref = Eigen::Matrix3d::Zero();
  src.col(0) = query.binary_A_.location_ - query.center_;
  src.col(1) = query.binary_B_.location_ - query.center_;
  src.col(2) = query.binary_C_.location_ - query.center_;
  ref.col(0) = map.binary_A_.location_ - map.center_;
  ref.col(1) = map.binary_B_.location_ - map.center_;
  ref.col(2) = map.binary_C_.location_ - map.center_;
  const Eigen::Matrix3d covariance = src * ref.transpose();
  // Same Kabsch/SVD recovery as upstream BTC triangle_solver, restricted to query->submap.
  Eigen::JacobiSVD<Eigen::Matrix3d> svd(covariance, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix3d rotation = svd.matrixV() * svd.matrixU().transpose();
  if (rotation.determinant() < 0.0) {
    Eigen::Matrix3d correction = Eigen::Matrix3d::Identity();
    correction(2, 2) = -1.0;
    rotation = svd.matrixV() * correction * svd.matrixU().transpose();
  }
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.linear() = rotation;
  transform.translation() = -rotation * query.center_ + map.center_;
  validateRigid(transform, "candidate submap_from_query");
  return transform;
}

std::optional<double> verifiedVertexMeanSquared(const Eigen::Isometry3d& submap_from_query,
                                                const BTC& query, const BTC& map,
                                                double max_vertex_error) {
  const Eigen::Vector3d a =
    submap_from_query * query.binary_A_.location_ - map.binary_A_.location_;
  const Eigen::Vector3d b =
    submap_from_query * query.binary_B_.location_ - map.binary_B_.location_;
  const Eigen::Vector3d c =
    submap_from_query * query.binary_C_.location_ - map.binary_C_.location_;
  if (a.norm() > max_vertex_error || b.norm() > max_vertex_error || c.norm() > max_vertex_error) {
    return std::nullopt;
  }
  return (a.squaredNorm() + b.squaredNorm() + c.squaredNorm()) / 3.0;
}

std::vector<std::size_t> evenlySampledIndices(std::size_t count, std::size_t limit) {
  const std::size_t sample_count = std::min(count, limit);
  std::vector<std::size_t> indices;
  indices.reserve(sample_count);
  if (sample_count == 0) {
    return indices;
  }
  if (sample_count == 1) {
    indices.push_back(0);
    return indices;
  }
  for (std::size_t i = 0; i < sample_count; ++i) {
    indices.push_back((i * (count - 1)) / (sample_count - 1));
  }
  indices.erase(std::unique(indices.begin(), indices.end()), indices.end());
  return indices;
}

Cloud cropAroundLastSensor(const Cloud& map_cloud, const Eigen::Isometry3d& local_from_last,
                           double radius) {
  Cloud query;
  const Eigen::Isometry3d last_from_local = local_from_last.inverse();
  const Eigen::Vector3d origin = local_from_last.translation();
  for (const auto& point : map_cloud) {
    const Eigen::Vector3d local_point(point.x, point.y, point.z);
    if (!local_point.allFinite() || (local_point - origin).norm() > radius) {
      continue;
    }
    const Eigen::Vector3d query_point = last_from_local * local_point;
    query.emplace_back(static_cast<float>(query_point.x()),
                       static_cast<float>(query_point.y()),
                       static_cast<float>(query_point.z()));
  }
  query.width = static_cast<std::uint32_t>(query.size());
  query.height = 1;
  query.is_dense = true;
  return query;
}

void validateCloud(const Cloud& cloud) {
  for (const auto& point : cloud) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
      throw std::invalid_argument("input cloud contains a non-finite point");
    }
  }
}

}  // namespace

struct MapDatabase::Impl {
  Impl(const Impl& other)
      : root(other.root), search(other.search), metadata(other.metadata),
        records(other.records) {
    rebuildLookups();
  }

  Impl(const std::filesystem::path& directory, const SearchConfig& search_config)
      : search(search_config) {
    validateSearchConfig(search);
    if (directory.empty()) {
      throw std::invalid_argument("map snapshot directory must not be empty");
    }
    root = std::filesystem::absolute(directory).lexically_normal();
    if (!std::filesystem::is_directory(root)) {
      throw std::runtime_error("map snapshot directory does not exist: " + root.string());
    }
    if (std::filesystem::exists(root / ".incomplete")) {
      throw std::runtime_error("map snapshot is incomplete: " + root.string());
    }
    metadata = loadMetadata(root);
    auto pose_rows = loadPoses(root, metadata.poses_file);
    loadManifest(std::move(pose_rows));
  }

  void rebuildLookups() {
    record_by_id.clear();
    index.clear();
    for (std::size_t record_index = 0; record_index < records.size(); ++record_index) {
      auto& record = records[record_index];
      record_by_id.emplace(record.pose.id, record_index);
      for (std::size_t i = 0; i < record.features.triangles.size(); ++i) {
        const auto& triangle = record.features.triangles[i];
        index[bucketFor(triangle)].push_back(Match{0, i, nullptr, &triangle, 0.0});
      }
    }
  }

  const Cloud& loadSubmapCloud(const SubmapRecord& record) const {
    if (!record.cached_cloud.has_value()) {
      Cloud cloud;
      if (record.pose.point_count > 0) {
        if (pcl::io::loadPCDFile((root / record.pose.pcd_file).string(), cloud) != 0) {
          throw std::runtime_error("failed to load submap PCD: " + record.pose.pcd_file.string());
        }
        if (cloud.size() != record.pose.point_count) {
          throw std::runtime_error("submap PCD point count does not match poses.csv");
        }
        validateCloud(cloud);
      }
      record.cached_cloud = std::move(cloud);
    }
    return *record.cached_cloud;
  }

  void loadManifest(std::unordered_map<std::uint64_t, PoseRow> pose_rows) {
    const auto manifest_path = root / metadata.manifest_file;
    if (!std::filesystem::is_regular_file(manifest_path)) {
      throw std::runtime_error(
        "snapshot BTC manifest is missing; save the map again with btc.enabled=true");
    }
    std::ifstream stream(manifest_path);
    stream.imbue(std::locale::classic());
    std::string line;
    if (!std::getline(stream, line) ||
        line != "id,status,binary_count,triangle_count,plane_count,descriptor_file") {
      throw std::runtime_error("unsupported btc/manifest.csv header");
    }
    std::unordered_set<std::uint64_t> seen;
    while (std::getline(stream, line)) {
      if (line.empty()) {
        continue;
      }
      const auto fields = splitCsvLine(line);
      if (fields.size() != 6) {
        throw std::runtime_error("invalid BTC manifest row");
      }
      const auto id = parseUnsigned<std::uint64_t>(fields[0], "manifest.id");
      const auto pose_it = pose_rows.find(id);
      if (pose_it == pose_rows.end()) {
        throw std::runtime_error("BTC manifest references an unknown submap id");
      }
      if (!seen.insert(id).second) {
        throw std::runtime_error("duplicate submap id in BTC manifest");
      }
      const std::string& status = fields[1];
      const auto binary_count = parseUnsigned<std::size_t>(fields[2], "manifest.binary_count");
      const auto triangle_count = parseUnsigned<std::size_t>(fields[3], "manifest.triangle_count");
      const auto plane_count = parseUnsigned<std::size_t>(fields[4], "manifest.plane_count");
      const auto descriptor_file = safeRelativePath(fields[5], "manifest.descriptor_file");
      if (!std::filesystem::is_regular_file(root / descriptor_file)) {
        throw std::runtime_error("missing BTC descriptor file: " + descriptor_file.string());
      }
      auto features = map_processing::loadBtcFeatures(root / descriptor_file);
      if (features.submap_id != id || features.binary.size() != binary_count ||
          features.triangles.size() != triangle_count || features.planes.size() != plane_count) {
        throw std::runtime_error("BTC manifest counts do not match descriptor file");
      }
      if (status == "empty") {
        if (pose_it->second.point_count != 0 || binary_count != 0 || triangle_count != 0) {
          throw std::runtime_error("BTC empty status does not match pose/counts");
        }
      } else if (status == "no_triangles") {
        if (pose_it->second.point_count == 0 || triangle_count != 0) {
          throw std::runtime_error("BTC no_triangles status does not match pose/counts");
        }
      } else if (status == "ready") {
        if (pose_it->second.point_count == 0 || triangle_count == 0) {
          throw std::runtime_error("BTC ready status does not match pose/counts");
        }
      } else {
        throw std::runtime_error("unsupported BTC manifest status: " + status);
      }

      records.push_back(SubmapRecord{std::move(pose_it->second), std::move(features),
                                     descriptor_file, std::nullopt});
    }
    if (seen.size() != pose_rows.size()) {
      throw std::runtime_error("BTC manifest and poses.csv submap ID sets differ");
    }
    if (metadata.btc_extraction_revision < map_processing::kBtcExtractionRevision) {
      // Validate the original snapshot first, then upgrade only the in-memory features.
      for (auto& record : records) {
        record.features = map_processing::extractBtcFeatures(
          loadSubmapCloud(record), record.pose.id, metadata.btc);
      }
    }
    rebuildLookups();
  }

  std::vector<Candidate> searchFeatures(const map_processing::BtcFeatures& query,
                                         SearchDiagnostics& diagnostics) const {
    diagnostics = SearchDiagnostics{0, 0, search.min_votes, search.min_inliers};
    if (query.triangles.empty()) {
      return {};
    }
    std::unordered_map<std::uint64_t, std::vector<Match>> matches_by_submap;
    std::unordered_map<std::uint64_t, std::unordered_set<std::size_t>> votes_by_submap;
    std::unordered_map<std::uint64_t, std::unordered_set<std::size_t>> length_votes_by_submap;
    for (std::size_t qi = 0; qi < query.triangles.size(); ++qi) {
      const auto& q = query.triangles[qi];
      const auto base = bucketFor(q);
      for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
          for (int dz = -1; dz <= 1; ++dz) {
            const auto found = index.find(BucketKey{base.x + dx, base.y + dy, base.z + dz});
            if (found == index.end()) {
              continue;
            }
            for (const auto& stored : found->second) {
              const BTC& m = *stored.map;
              const double length_error = (q.triangle_ - m.triangle_).norm();
              if (length_error >= q.triangle_.norm() * search.length_tolerance) {
                continue;
              }
              auto& length_votes = length_votes_by_submap[m.frame_number_];
              length_votes.insert(qi);
              diagnostics.best_length_votes = std::max(
                diagnostics.best_length_votes, length_votes.size());
              const double similarity = descriptorSimilarity(q, m);
              if (similarity < search.binary_similarity) {
                continue;
              }
              auto& matches = matches_by_submap[m.frame_number_];
              if (matches.size() < search.max_matches_per_candidate) {
                matches.push_back(Match{qi, stored.map_index, &q, &m, similarity});
              }
              votes_by_submap[m.frame_number_].insert(qi);
            }
          }
        }
      }
    }

    std::vector<std::pair<std::uint64_t, std::size_t>> voted_ids;
    voted_ids.reserve(votes_by_submap.size());
    for (const auto& [id, votes] : votes_by_submap) {
      diagnostics.best_votes = std::max(diagnostics.best_votes, votes.size());
      if (votes.size() >= search.min_votes) {
        voted_ids.emplace_back(id, votes.size());
      }
    }
    std::sort(voted_ids.begin(), voted_ids.end(), [](const auto& lhs, const auto& rhs) {
      if (lhs.second != rhs.second) {
        return lhs.second > rhs.second;
      }
      return lhs.first < rhs.first;
    });
    if (voted_ids.size() > search.max_hypotheses) {
      voted_ids.resize(search.max_hypotheses);
    }

    std::vector<Candidate> candidates;
    for (const auto& [id, votes] : voted_ids) {
      const auto record_it = record_by_id.find(id);
      if (record_it == record_by_id.end()) {
        continue;
      }
      const auto& record = records[record_it->second];
      const auto& matches = matches_by_submap[id];
      Candidate best;
      best.submap_id = id;
      best.votes = votes;
      double best_similarity = -1.0;
      for (const std::size_t seed_index :
           evenlySampledIndices(matches.size(), search.max_hypotheses)) {
        const auto& seed = matches[seed_index];
        Eigen::Isometry3d submap_from_query;
        try {
          submap_from_query = solveSubmapFromQuery(*seed.query, *seed.map);
        } catch (const std::exception&) {
          continue;
        }
        std::unordered_map<std::size_t, std::pair<double, double>> best_inlier_by_query;
        for (const auto& match : matches) {
          const auto error_squared = verifiedVertexMeanSquared(
            submap_from_query, *match.query, *match.map, search.max_vertex_error);
          if (error_squared.has_value()) {
            auto [it, inserted] = best_inlier_by_query.emplace(
              match.query_index, std::make_pair(*error_squared, match.similarity));
            if (!inserted && *error_squared < it->second.first) {
              it->second = std::make_pair(*error_squared, match.similarity);
            }
          }
        }
        const auto inliers = best_inlier_by_query.size();
        diagnostics.best_inliers = std::max(diagnostics.best_inliers, inliers);
        if (inliers < search.min_inliers) {
          continue;
        }
        double squared_error_sum = 0.0;
        double similarity_sum = 0.0;
        for (const auto& [query_index, error_and_similarity] : best_inlier_by_query) {
          (void)query_index;
          squared_error_sum += error_and_similarity.first;
          similarity_sum += error_and_similarity.second;
        }
        const double rmse = std::sqrt(squared_error_sum / static_cast<double>(inliers));
        const double mean_similarity = similarity_sum / static_cast<double>(inliers);
        if (best.inliers == 0 || inliers > best.inliers ||
            (inliers == best.inliers &&
             (mean_similarity > best_similarity ||
              (mean_similarity == best_similarity && rmse < best.vertex_rmse)))) {
          best.inliers = inliers;
          best.mean_binary_similarity = mean_similarity;
          best.vertex_rmse = rmse;
          best.submap_from_query = submap_from_query;
          best.map_from_query = record.pose.map_from_submap * submap_from_query;
          best_similarity = mean_similarity;
        }
      }
      if (best.inliers >= search.min_inliers) {
        validateRigid(best.submap_from_query, "candidate submap_from_query");
        validateRigid(best.map_from_query, "candidate map_from_query");
        candidates.push_back(best);
      }
    }

    std::sort(candidates.begin(), candidates.end(), [](const Candidate& lhs, const Candidate& rhs) {
      if (lhs.inliers != rhs.inliers) {
        return lhs.inliers > rhs.inliers;
      }
      if (lhs.mean_binary_similarity != rhs.mean_binary_similarity) {
        return lhs.mean_binary_similarity > rhs.mean_binary_similarity;
      }
      if (lhs.vertex_rmse != rhs.vertex_rmse) {
        return lhs.vertex_rmse < rhs.vertex_rmse;
      }
      return lhs.submap_id < rhs.submap_id;
    });
    if (candidates.size() > search.top_k) {
      candidates.resize(search.top_k);
    }
    return candidates;
  }

  std::filesystem::path root;
  SearchConfig search;
  Metadata metadata;
  std::vector<SubmapRecord> records;
  std::unordered_map<std::uint64_t, std::size_t> record_by_id;
  std::unordered_map<BucketKey, std::vector<Match>, BucketHash> index;
};

MapDatabase::MapDatabase(const std::filesystem::path& snapshot_directory,
                         const SearchConfig& search)
    : impl_(std::make_unique<Impl>(snapshot_directory, search)) {}

MapDatabase::~MapDatabase() = default;
MapDatabase::MapDatabase(const MapDatabase& other)
    : impl_(std::make_unique<Impl>(*other.impl_)) {}
MapDatabase& MapDatabase::operator=(const MapDatabase& other) {
  if (this != &other) {
    impl_ = std::make_unique<Impl>(*other.impl_);
  }
  return *this;
}
MapDatabase::MapDatabase(MapDatabase&&) noexcept = default;
MapDatabase& MapDatabase::operator=(MapDatabase&&) noexcept = default;

const map_processing::BtcConfig& MapDatabase::btcConfig() const { return impl_->metadata.btc; }
const map_processing::FreedomConfig& MapDatabase::freedomConfig() const {
  return impl_->metadata.freedom;
}
double MapDatabase::radius() const { return impl_->metadata.radius; }
const std::string& MapDatabase::mapFrame() const { return impl_->metadata.map_frame; }
std::size_t MapDatabase::size() const { return impl_->records.size(); }

std::vector<Candidate> MapDatabase::search(const map_processing::BtcFeatures& query) const {
  SearchDiagnostics diagnostics;
  return search(query, diagnostics);
}

std::vector<Candidate> MapDatabase::search(const map_processing::BtcFeatures& query,
                                           SearchDiagnostics& diagnostics) const {
  return impl_->searchFeatures(query, diagnostics);
}

Cloud MapDatabase::loadGlobalMap() const {
  const auto file = impl_->root / impl_->metadata.static_map_file;
  Cloud cloud;
  if (pcl::io::loadPCDFile(file.string(), cloud) != 0) {
    throw std::runtime_error("failed to load global map PCD: " + file.string());
  }
  if (cloud.size() != impl_->metadata.static_point_count) {
    throw std::runtime_error("global map PCD point count does not match metadata.yaml");
  }
  validateCloud(cloud);
  return cloud;
}

Cloud MapDatabase::loadSubmap(std::uint64_t submap_id) const {
  const auto record_it = impl_->record_by_id.find(submap_id);
  if (record_it == impl_->record_by_id.end()) {
    throw std::out_of_range("unknown submap id");
  }
  return impl_->loadSubmapCloud(impl_->records[record_it->second]);
}

struct RelocalizationProcessor::Impl {
  Impl(const std::filesystem::path& snapshot_directory, const QueryConfig& query_config,
       const SearchConfig& search_config)
      : database(snapshot_directory, search_config), query(query_config),
        freedom(std::make_unique<map_processing::FreedomProcessor>(database.freedomConfig(), true)) {
    validateQueryConfig(query);
  }

  void resetWindow() {
    freedom = std::make_unique<map_processing::FreedomProcessor>(database.freedomConfig(), true);
    frame_count = 0;
    first_stamp_ns = 0;
    latest_static_map.clear();
  }

  MapDatabase database;
  QueryConfig query;
  std::unique_ptr<map_processing::FreedomProcessor> freedom;
  std::size_t frame_count{0};
  std::int64_t first_stamp_ns{0};
  std::int64_t last_stamp_ns{0};
  Cloud latest_static_map;
};

RelocalizationProcessor::RelocalizationProcessor(const std::filesystem::path& snapshot_directory,
                                                 const QueryConfig& query,
                                                 const SearchConfig& search)
    : impl_(std::make_unique<Impl>(snapshot_directory, query, search)) {}

RelocalizationProcessor::~RelocalizationProcessor() = default;
RelocalizationProcessor::RelocalizationProcessor(RelocalizationProcessor&&) noexcept = default;
RelocalizationProcessor& RelocalizationProcessor::operator=(RelocalizationProcessor&&) noexcept =
  default;

std::optional<QueryResult> RelocalizationProcessor::process(
  const Cloud& sensor_cloud, const Eigen::Isometry3d& odom_from_sensor, std::int64_t stamp_ns) {
  validateRigid(odom_from_sensor, "odom_from_sensor");
  validateCloud(sensor_cloud);
  if (stamp_ns <= 0 || stamp_ns <= impl_->last_stamp_ns) {
    throw std::invalid_argument("relocalization requires increasing positive stamps");
  }

  // 短暂丢帧/TF 延迟只影响本轮查询节奏；不能丢弃已扫描的其他朝向。
  if (impl_->last_stamp_ns > 0 &&
      static_cast<double>(stamp_ns - impl_->last_stamp_ns) * 1e-9 > impl_->query.history_timeout) {
    impl_->resetWindow();
  } else if (impl_->frame_count > 0 &&
      static_cast<double>(stamp_ns - impl_->first_stamp_ns) * 1e-9 > impl_->query.max_duration) {
    impl_->frame_count = 0;
    impl_->first_stamp_ns = 0;
  }
  if (impl_->frame_count == 0) {
    impl_->first_stamp_ns = stamp_ns;
  }

  impl_->latest_static_map = impl_->freedom->process(sensor_cloud, odom_from_sensor);
  ++impl_->frame_count;
  impl_->last_stamp_ns = stamp_ns;

  if (impl_->frame_count < impl_->query.frames_per_query) {
    return std::nullopt;
  }

  QueryResult result;
  result.stamp_ns = stamp_ns;
  result.frame_count = impl_->frame_count;
  result.odom_from_query = odom_from_sensor;
  result.query_cloud = cropAroundLastSensor(
    impl_->latest_static_map, odom_from_sensor, impl_->database.radius());
  // 查询频率与历史寿命分开：每 N 帧查询，但保留附近不同朝向的观测。
  // FreeDOM local map 会随雷达移动移除远处的历史，避免地图无限增长。
  impl_->frame_count = 0;
  impl_->first_stamp_ns = 0;

  const auto features = map_processing::extractBtcFeatures(
    result.query_cloud, static_cast<std::uint64_t>(stamp_ns), impl_->database.btcConfig());
  result.binary_count = features.binary.size();
  result.triangle_count = features.triangles.size();
  result.candidates = impl_->database.search(features, result.diagnostics);
  if (result.candidates.empty() &&
      !odom_from_sensor.linear().isApprox(Eigen::Matrix3d::Identity(), 1e-6)) {
    // BTC voxel/projection grids are orientation-sensitive. Retry once using
    // odometry axes, without assuming any map position or relaxing verification.
    Eigen::Isometry3d aligned_from_query = Eigen::Isometry3d::Identity();
    aligned_from_query.linear() = odom_from_sensor.linear();
    Cloud aligned_cloud;
    pcl::transformPointCloud(result.query_cloud, aligned_cloud, aligned_from_query.matrix());
    const auto aligned_features = map_processing::extractBtcFeatures(
      aligned_cloud, static_cast<std::uint64_t>(stamp_ns), impl_->database.btcConfig());
    SearchDiagnostics aligned_diagnostics;
    auto aligned_candidates = impl_->database.search(aligned_features, aligned_diagnostics);
    const auto quality = [](const SearchDiagnostics& diagnostics, std::size_t triangles) {
      return std::make_tuple(diagnostics.best_inliers, diagnostics.best_votes, triangles);
    };
    if (!aligned_candidates.empty() ||
        quality(aligned_diagnostics, aligned_features.triangles.size()) >
        quality(result.diagnostics, result.triangle_count)) {
      for (auto& candidate : aligned_candidates) {
        // Search estimated T_map_aligned; expose T_map_sensor as usual.
        candidate.submap_from_query = candidate.submap_from_query * aligned_from_query;
        candidate.map_from_query = candidate.map_from_query * aligned_from_query;
      }
      result.binary_count = aligned_features.binary.size();
      result.triangle_count = aligned_features.triangles.size();
      result.diagnostics = aligned_diagnostics;
      result.candidates = std::move(aligned_candidates);
      result.used_orientation_retry = true;
    }
  }
  if (result.candidates.empty()) {
    if (result.triangle_count == 0) {
      result.failure_reason = "no_descriptors";
    } else if (result.triangle_count < std::max(
        result.diagnostics.required_votes, result.diagnostics.required_inliers)) {
      result.failure_reason = "insufficient_triangles";
    } else if (result.diagnostics.best_votes < result.diagnostics.required_votes) {
      result.failure_reason = "insufficient_votes";
    } else {
      result.failure_reason = "geometry_rejected";
    }
  }
  return result;
}

const MapDatabase& RelocalizationProcessor::database() const { return impl_->database; }

void RelocalizationProcessor::reset() {
  impl_->resetWindow();
  impl_->last_stamp_ns = 0;
}

}  // namespace relocalization_processing
