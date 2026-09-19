#include "map/submap.hpp"

#include <algorithm>
#include <charconv>
#include <cmath>
#include <chrono>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <locale>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <utility>

#include <pcl/io/pcd_io.h>

namespace map_processing {

namespace {

std::ofstream openTextFile(const std::filesystem::path& path) {
  std::ofstream stream;
  stream.exceptions(std::ios::failbit | std::ios::badbit);
  stream.imbue(std::locale::classic());
  stream.open(path);
  stream << std::setprecision(17);
  return stream;
}

void writeCloud(const std::filesystem::path& path, const Cloud& cloud) {
  if (pcl::io::savePCDFileBinary(path.string(), cloud) != 0) {
    throw std::runtime_error("failed to write PCD: " + path.string());
  }
}

bool validFrame(const std::string& frame) {
  if (frame.empty()) {
    return false;
  }
  for (const unsigned char c : frame) {
    if (c < 0x20 || c == 0x7f) {
      return false;
    }
  }
  return true;
}

ConfigSetting btcSettings(const BtcConfig& config) {
  ConfigSetting settings{};
  settings.useful_corner_num_ = config.useful_corner_num;
  settings.plane_detection_thre_ = config.plane_detection_threshold;
  settings.plane_merge_normal_thre_ = config.plane_merge_normal_threshold;
  settings.plane_merge_dis_thre_ = config.plane_merge_distance_threshold;
  settings.voxel_size_ = config.voxel_size;
  settings.voxel_init_num_ = config.voxel_init_num;
  settings.proj_plane_num_ = config.projection_plane_num;
  settings.proj_image_resolution_ = config.projection_resolution;
  settings.proj_image_high_inc_ = config.projection_height_increment;
  settings.proj_dis_min_ = config.projection_distance_min;
  settings.proj_dis_max_ = config.projection_distance_max;
  settings.summary_min_thre_ = config.summary_min_threshold;
  settings.line_filter_enable_ = config.line_filter_enabled;
  settings.descriptor_near_num_ = config.descriptor_near_num;
  settings.descriptor_min_len_ = config.descriptor_min_length;
  settings.descriptor_max_len_ = config.descriptor_max_length;
  settings.non_max_suppression_radius_ = config.non_max_suppression_radius;
  settings.std_side_resolution_ = config.triangle_resolution;
  return settings;
}

void writeVector(std::ostream& stream, const Eigen::Vector3d& vector) {
  stream << vector.x() << ' ' << vector.y() << ' ' << vector.z();
}

void writeBinary(std::ostream& stream, const BinaryDescriptor& binary) {
  writeVector(stream, binary.location_);
  stream << ' ' << static_cast<unsigned int>(binary.summary_) << ' '
         << binary.occupy_array_.size() << ' ';
  if (binary.occupy_array_.empty()) {
    stream << '-';
  } else {
    for (const bool occupied : binary.occupy_array_) {
      stream << (occupied ? '1' : '0');
    }
  }
  stream << '\n';
}

// 不直接 dump C++ 结构体/vector<bool>：显式版本与字段保证跨编译器可读。
void writeBtcFeatures(const std::filesystem::path& path, const BtcFeatures& features) {
  auto stream = openTextFile(path);
  stream << "MAP_BTC 1\nsubmap_id " << features.submap_id
         << "\nbinary_count " << features.binary.size() << '\n';
  for (const auto& binary : features.binary) {
    stream << "binary ";
    writeBinary(stream, binary);
  }
  stream << "triangle_count " << features.triangles.size() << '\n';
  for (const auto& triangle : features.triangles) {
    stream << "triangle ";
    writeVector(stream, triangle.triangle_);
    stream << ' ';
    writeVector(stream, triangle.angle_);
    stream << ' ';
    writeVector(stream, triangle.center_);
    stream << '\n';
    stream << "vertex_a ";
    writeBinary(stream, triangle.binary_A_);
    stream << "vertex_b ";
    writeBinary(stream, triangle.binary_B_);
    stream << "vertex_c ";
    writeBinary(stream, triangle.binary_C_);
  }
  stream << "plane_count " << features.planes.size() << '\n';
  for (const auto& plane : features.planes) {
    stream << "plane " << plane.x << ' ' << plane.y << ' ' << plane.z << ' '
           << plane.intensity << ' ' << plane.normal_x << ' ' << plane.normal_y << ' '
           << plane.normal_z << ' ' << plane.curvature << '\n';
  }
  stream << "end\n";
  stream.close();
}

void expectToken(std::istream& stream, const char* expected) {
  std::string token;
  if (!(stream >> token) || token != expected) {
    throw std::runtime_error(std::string("invalid BTC file: expected ") + expected);
  }
}

std::uint64_t readUnsigned(std::istream& stream, std::uint64_t maximum) {
  std::string token;
  std::uint64_t value{};
  if (!(stream >> token)) {
    throw std::runtime_error("truncated BTC file");
  }
  const auto parsed = std::from_chars(token.data(), token.data() + token.size(), value);
  if (parsed.ec != std::errc{} || parsed.ptr != token.data() + token.size() || value > maximum) {
    throw std::runtime_error("invalid BTC integer/count");
  }
  return value;
}

Eigen::Vector3d readVector(std::istream& stream) {
  Eigen::Vector3d vector;
  if (!(stream >> vector.x() >> vector.y() >> vector.z()) || !vector.allFinite()) {
    throw std::runtime_error("invalid BTC vector");
  }
  return vector;
}

BinaryDescriptor readBinary(std::istream& stream) {
  BinaryDescriptor binary{};
  binary.location_ = readVector(stream);
  binary.summary_ = static_cast<unsigned char>(readUnsigned(stream, 255));
  const auto bit_count = readUnsigned(stream, 255);
  std::string bits;
  if (!(stream >> bits) || (bit_count == 0 ? bits != "-" : bits.size() != bit_count)) {
    throw std::runtime_error("invalid BTC occupancy bit count");
  }
  if (bit_count != 0) {
    for (const char bit : bits) {
      if (bit != '0' && bit != '1') {
        throw std::runtime_error("invalid BTC occupancy bit");
      }
      binary.occupy_array_.push_back(bit == '1');
    }
  }
  if (std::count(binary.occupy_array_.begin(), binary.occupy_array_.end(), true) != binary.summary_) {
    throw std::runtime_error("BTC summary does not match occupancy bits");
  }
  return binary;
}

void writeBtcMetadata(std::ostream& stream, const BtcConfig& c, std::size_t ready,
                      std::size_t binary_count, std::size_t triangle_count) {
  stream << "btc:\n"
         << "  enabled: " << (c.enabled ? "true" : "false") << '\n'
         << "  upstream_commit: 742af157036144edad9a8350330c0158ea1a40d5\n"
         << "  descriptor_format: MAP_BTC\n  descriptor_format_version: 1\n"
         << "  extraction_revision: " << kBtcExtractionRevision << '\n'
         << "  coordinates: submap_local\n"
         << "  manifest_file: " << (c.enabled ? "btc/manifest.csv" : "null") << '\n'
         << "  index_file: " << (c.enabled ? "btc/index.csv" : "null") << '\n'
         << "  ready_submap_count: " << ready << '\n'
         << "  binary_count: " << binary_count << '\n'
         << "  triangle_count: " << triangle_count << '\n'
         << "  useful_corner_num: " << c.useful_corner_num << '\n'
         << "  plane_detection_threshold: " << c.plane_detection_threshold << '\n'
         << "  plane_merge_normal_threshold: " << c.plane_merge_normal_threshold << '\n'
         << "  plane_merge_distance_threshold: " << c.plane_merge_distance_threshold << '\n'
         << "  voxel_size: " << c.voxel_size << '\n'
         << "  voxel_init_num: " << c.voxel_init_num << '\n'
         << "  projection_plane_num: " << c.projection_plane_num << '\n'
         << "  projection_resolution: " << c.projection_resolution << '\n'
         << "  projection_height_increment: " << c.projection_height_increment << '\n'
         << "  projection_distance_min: " << c.projection_distance_min << '\n'
         << "  projection_distance_max: " << c.projection_distance_max << '\n'
         << "  summary_min_threshold: " << c.summary_min_threshold << '\n'
         << "  line_filter_enabled: " << (c.line_filter_enabled ? "true" : "false") << '\n'
         << "  descriptor_near_num: " << c.descriptor_near_num << '\n'
         << "  descriptor_min_length: " << c.descriptor_min_length << '\n'
         << "  descriptor_max_length: " << c.descriptor_max_length << '\n'
         << "  non_max_suppression_radius: " << c.non_max_suppression_radius << '\n'
         << "  triangle_resolution: " << c.triangle_resolution << '\n';
}

}  // namespace

void validateBtcConfig(const BtcConfig& c) {
  const auto within = [](double value, double minimum, double maximum) {
    return std::isfinite(value) && value >= minimum && value <= maximum;
  };
  if (c.useful_corner_num < 3 || c.useful_corner_num > 5000 ||
      c.voxel_init_num < 3 || c.voxel_init_num > 10000 ||
      c.projection_plane_num < 1 || c.projection_plane_num > 8 ||
      c.descriptor_near_num < 3 || c.descriptor_near_num > 100 ||
      !within(c.voxel_size, 0.1, 5.0) ||
      !within(c.plane_detection_threshold, 1e-8, 1.0) ||
      !within(c.plane_merge_normal_threshold, 1e-6, 2.0) ||
      !within(c.plane_merge_distance_threshold, 1e-6, 5.0) ||
      !within(c.projection_resolution, 0.05, 2.0) ||
      !within(c.projection_height_increment, 0.01, 5.0) ||
      !within(c.projection_distance_min, -30.0, 30.0) ||
      !within(c.projection_distance_max, -30.0, 30.0) ||
      c.projection_distance_min >= c.projection_distance_max ||
      !within(c.descriptor_min_length, 0.01, 100.0) ||
      !within(c.descriptor_max_length, c.descriptor_min_length, 100.0) ||
      !within(c.non_max_suppression_radius, 0.01, 10.0) ||
      !within(c.triangle_resolution, 0.01, 10.0)) {
    throw std::invalid_argument("invalid BTC extraction parameters");
  }
  // 按核心的 float 参数计算，与实际投影分箱完全一致；summary_ 是 uint8。
  const auto s = btcSettings(c);
  const double bins = (static_cast<double>(s.proj_dis_max_) - s.proj_dis_min_) /
                     s.proj_image_high_inc_;
  if (bins < 1 || bins > 255 || c.summary_min_threshold < 1 ||
      c.summary_min_threshold > static_cast<int>(bins)) {
    throw std::invalid_argument("BTC requires 1..255 height bins and a valid summary threshold");
  }
}

BtcFeatures extractBtcFeatures(const Cloud& local_cloud, std::uint64_t submap_id,
                              const BtcConfig& config) {
  validateBtcConfig(config);
  BtcFeatures features;
  features.submap_id = submap_id;
  if (local_cloud.empty()) {
    return features;
  }
  auto input = pcl::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  input->reserve(local_cloud.size());
  for (const auto& point : local_cloud) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
      throw std::invalid_argument("BTC requires finite submap points");
    }
    pcl::PointXYZI converted;
    converted.x = point.x;
    converted.y = point.y;
    converted.z = point.z;
    converted.intensity = 0.0F;  // BTC 的几何提取不使用反射强度。
    input->push_back(converted);
  }
  auto settings = btcSettings(config);
  BtcDescManager manager(settings);
  manager.GenerateBtcDescs(input, submap_id, features.triangles);
  features.binary = std::move(manager.history_binary_list_.back());
  features.planes = std::move(*manager.plane_cloud_vec_.back());
  return features;
}

BtcFeatures loadBtcFeatures(const std::filesystem::path& file) {
  std::ifstream stream(file);
  stream.imbue(std::locale::classic());
  if (!stream) {
    throw std::runtime_error("cannot open BTC file: " + file.string());
  }
  const auto size = std::filesystem::file_size(file);
  expectToken(stream, "MAP_BTC");
  if (readUnsigned(stream, 1) != 1) {
    throw std::runtime_error("unsupported BTC format version");
  }
  BtcFeatures features;
  expectToken(stream, "submap_id");
  features.submap_id = readUnsigned(stream, std::numeric_limits<std::uint64_t>::max());
  expectToken(stream, "binary_count");
  const auto binary_count = readUnsigned(stream, size / 10);
  for (std::uint64_t i = 0; i < binary_count; ++i) {
    expectToken(stream, "binary");
    features.binary.push_back(readBinary(stream));
  }
  expectToken(stream, "triangle_count");
  const auto triangle_count = readUnsigned(stream, size / 40);
  for (std::uint64_t i = 0; i < triangle_count; ++i) {
    expectToken(stream, "triangle");
    BTC triangle{};
    triangle.frame_number_ = features.submap_id;
    triangle.triangle_ = readVector(stream);
    triangle.angle_ = readVector(stream);
    triangle.center_ = readVector(stream);
    if (triangle.triangle_.minCoeff() <= 0 || triangle.triangle_.maxCoeff() > 10001 ||
        triangle.triangle_[0] > triangle.triangle_[1] ||
        triangle.triangle_[1] > triangle.triangle_[2]) {
      throw std::runtime_error("invalid BTC triangle lengths");
    }
    expectToken(stream, "vertex_a");
    triangle.binary_A_ = readBinary(stream);
    expectToken(stream, "vertex_b");
    triangle.binary_B_ = readBinary(stream);
    expectToken(stream, "vertex_c");
    triangle.binary_C_ = readBinary(stream);
    features.triangles.push_back(std::move(triangle));
  }
  expectToken(stream, "plane_count");
  const auto plane_count = readUnsigned(stream, size / 10);
  for (std::uint64_t i = 0; i < plane_count; ++i) {
    expectToken(stream, "plane");
    pcl::PointXYZINormal point{};
    if (!(stream >> point.x >> point.y >> point.z >> point.intensity >> point.normal_x >>
        point.normal_y >> point.normal_z >> point.curvature) ||
        !point.getVector3fMap().allFinite() || !point.getNormalVector3fMap().allFinite() ||
        !std::isfinite(point.intensity) || !std::isfinite(point.curvature)) {
      throw std::runtime_error("invalid BTC plane");
    }
    features.planes.push_back(point);
  }
  expectToken(stream, "end");
  std::string trailing;
  if (stream >> trailing) {
    throw std::runtime_error("unexpected data after BTC file end");
  }
  return features;
}

SubmapBuilder::SubmapBuilder(const SubmapConfig& config) : config_(config) {
  if (!std::isfinite(config_.translation_threshold) || config_.translation_threshold <= 0.0 ||
      !std::isfinite(config_.radius) || config_.radius <= 0.0) {
    throw std::invalid_argument("submap translation threshold/radius must be positive and finite");
  }
}

Cloud SubmapBuilder::crop(const Cloud& static_map,
                         const Eigen::Isometry3d& map_from_submap) const {
  Cloud cloud;
  const Eigen::Isometry3d submap_from_map = map_from_submap.inverse();
  for (const auto& point : static_map) {
    const Eigen::Vector3d map_point(point.x, point.y, point.z);
    if (!map_point.allFinite() ||
        (map_point - map_from_submap.translation()).norm() > config_.radius) {
      continue;
    }
    const Eigen::Vector3d local_point = submap_from_map * map_point;
    cloud.emplace_back(static_cast<float>(local_point.x()), static_cast<float>(local_point.y()),
                       static_cast<float>(local_point.z()));
  }
  cloud.width = static_cast<std::uint32_t>(cloud.size());
  cloud.height = 1;
  cloud.is_dense = true;
  return cloud;
}

Cloud SubmapBuilder::extract(const Cloud& static_map, std::size_t id) const {
  return crop(static_map, poses_.at(id).map_from_submap);
}

std::filesystem::path SubmapBuilder::save(const Cloud& latest_static_map,
                                         const MapSaveOptions& options) const {
  if (last_stamp_ns_ <= 0 || latest_static_map.empty()) {
    throw std::runtime_error("no valid static map to save; wait for point clouds and timestamped TF");
  }
  if (options.directory.empty() || !validFrame(options.map_frame) ||
      !validFrame(options.sensor_frame)) {
    throw std::invalid_argument("save directory and frame names must be valid and nonempty");
  }
  validateBtcConfig(options.btc);
  for (const auto& point : latest_static_map) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
      throw std::invalid_argument("cannot save a static map containing non-finite points");
    }
  }

  const auto root = std::filesystem::absolute(options.directory).lexically_normal();
  std::filesystem::create_directories(root);
  const auto now = std::chrono::system_clock::now();
  const auto wall_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
    now.time_since_epoch()).count();
  const std::time_t seconds = std::chrono::system_clock::to_time_t(now);
  std::tm utc{};
  if (gmtime_r(&seconds, &utc) == nullptr) {
    throw std::runtime_error("cannot format save timestamp");
  }
  std::ostringstream name;
  name.imbue(std::locale::classic());
  name << std::put_time(&utc, "%Y%m%dT%H%M%SZ") << '_' << wall_ns;

  // create_directory 原子地保留本次目录；即使同时保存也不会覆盖旧结果。
  auto destination = root / name.str();
  for (std::size_t suffix = 1; !std::filesystem::create_directory(destination); ++suffix) {
    destination = root / (name.str() + '_' + std::to_string(suffix));
  }
  try {
    // metadata.yaml 最后写入；.incomplete 存在说明未完成，不能作为地图加载。
    auto marker = openTextFile(destination / ".incomplete");
    marker << "Map save is in progress.\n";
    marker.close();
    std::filesystem::create_directory(destination / "submaps");
    writeCloud(destination / "static_map.pcd", latest_static_map);

    auto poses = openTextFile(destination / "poses.csv");
    poses << "id,stamp_ns,tx,ty,tz,qx,qy,qz,qw,point_count,pcd_file\n";
    std::ofstream btc_manifest;
    std::ofstream btc_index;
    if (options.btc.enabled) {
      std::filesystem::create_directory(destination / "btc");
      btc_manifest = openTextFile(destination / "btc/manifest.csv");
      btc_manifest << "id,status,binary_count,triangle_count,plane_count,descriptor_file\n";
      btc_index = openTextFile(destination / "btc/index.csv");
      btc_index << "bucket_x,bucket_y,bucket_z,submap_id,triangle_id\n";
    }
    std::size_t saved_submap_count = 0;
    std::size_t btc_ready_count = 0;
    std::size_t btc_binary_count = 0;
    std::size_t btc_triangle_count = 0;
    for (const auto& anchor : poses_) {
      const auto cloud = extract(latest_static_map, anchor.id);
      std::ostringstream stem;
      stem.imbue(std::locale::classic());
      stem << std::setfill('0') << std::setw(6) << anchor.id;
      std::string pcd_file;
      if (!cloud.empty()) {
        pcd_file = "submaps/" + stem.str() + ".pcd";
        writeCloud(destination / pcd_file, cloud);
        ++saved_submap_count;
      }
      if (options.btc.enabled) {
        // 与上面 PCD 使用同一份局部点云；保存时重新生成，避免沿用早期动态残影。
        const auto features = extractBtcFeatures(cloud, anchor.id, options.btc);
        const auto descriptor_file = "btc/" + stem.str() + ".btc";
        writeBtcFeatures(destination / descriptor_file, features);
        const auto status = cloud.empty() ? "empty" :
                            (features.triangles.empty() ? "no_triangles" : "ready");
        btc_manifest << anchor.id << ',' << status << ',' << features.binary.size() << ','
                     << features.triangles.size() << ',' << features.planes.size() << ','
                     << descriptor_file << '\n';
        btc_ready_count += !features.triangles.empty();
        btc_binary_count += features.binary.size();
        btc_triangle_count += features.triangles.size();
        for (std::size_t i = 0; i < features.triangles.size(); ++i) {
          const auto& triangle = features.triangles[i];
          // 与官方 AddBtcDescs 的桶定义一致，triangle_ 已除以 triangle_resolution。
          btc_index << static_cast<std::int64_t>(triangle.triangle_[0] + 0.5) << ','
                    << static_cast<std::int64_t>(triangle.triangle_[1] + 0.5) << ','
                    << static_cast<std::int64_t>(triangle.triangle_[2] + 0.5) << ','
                    << anchor.id << ',' << i << '\n';
        }
      }
      // 空子图可能是动态物体被清理后的结果；保留 ID，不写 PCL 无法读写的空 PCD。
      const auto& translation = anchor.map_from_submap.translation();
      const Eigen::Quaterniond rotation(anchor.map_from_submap.linear());
      poses << anchor.id << ',' << anchor.stamp_ns << ',' << translation.x() << ','
            << translation.y() << ',' << translation.z() << ',' << rotation.x() << ','
            << rotation.y() << ',' << rotation.z() << ',' << rotation.w() << ','
            << cloud.size() << ',' << pcd_file << '\n';
    }
    poses.close();
    if (options.btc.enabled) {
      btc_manifest.close();
      btc_index.close();
    }

    auto metadata = openTextFile(destination / "metadata.yaml");
    metadata << "format_version: 1\n"
             << "map_frame: " << std::quoted(options.map_frame) << '\n'
             << "sensor_frame: " << std::quoted(options.sensor_frame) << '\n'
             << "snapshot_stamp_ns: " << last_stamp_ns_ << '\n'
             << "saved_at_unix_ns: " << wall_ns << '\n'
             << "static_map_file: static_map.pcd\n"
             << "poses_file: poses.csv\n"
             << "submap_frame_prefix: map_submap_\n"
             << "submap_coordinates: local\n"
             << "pose_convention: T_map_submap\n"
             << "quaternion_order: [qx, qy, qz, qw]\n"
             << "anchor_count: " << poses_.size() << '\n'
             << "saved_submap_count: " << saved_submap_count << '\n'
             << "static_point_count: " << latest_static_map.size() << '\n'
             << "submap:\n"
             << "  translation_threshold: " << config_.translation_threshold << '\n'
             << "  radius: " << config_.radius << '\n'
             << "sensor:\n"
             << "  min_range: " << options.freedom.sensor_min_range << '\n'
             << "  max_range: " << options.freedom.sensor_max_range << '\n'
             << "  min_z: " << options.freedom.sensor_min_z << '\n'
             << "  max_z: " << options.freedom.sensor_max_z << '\n'
             << "freedom:\n"
             << "  sub_voxel_size: " << options.freedom.sub_voxel_size << '\n'
             << "  counts_to_free: " << options.freedom.counts_to_free << '\n'
             << "  counts_to_revert: " << options.freedom.counts_to_revert << '\n'
             << "  num_threads: " << options.freedom.num_threads << '\n'
             << "  raycast_enhancement: false\n";
    writeBtcMetadata(metadata, options.btc, btc_ready_count, btc_binary_count, btc_triangle_count);
    metadata.close();
    std::filesystem::remove(destination / ".incomplete");
  } catch (...) {
    // 只清理本函数新建的目录，不删除 root 或以前保存的地图。
    std::error_code ignored;
    std::filesystem::remove_all(destination, ignored);
    throw;
  }
  return destination;
}

SubmapUpdate SubmapBuilder::update(const Cloud& static_map,
                                  const Eigen::Isometry3d& map_from_sensor,
                                  std::int64_t stamp_ns) {
  if (stamp_ns <= last_stamp_ns_ || !map_from_sensor.matrix().allFinite() ||
      !map_from_sensor.matrix().row(3).isApprox(Eigen::RowVector4d(0.0, 0.0, 0.0, 1.0)) ||
      !(map_from_sensor.linear().transpose() * map_from_sensor.linear())
           .isApprox(Eigen::Matrix3d::Identity(), 1e-6) ||
      std::abs(map_from_sensor.linear().determinant() - 1.0) > 1e-6) {
    throw std::invalid_argument("submap requires an increasing positive stamp and a rigid pose");
  }

  bool create = poses_.empty();
  if (!create) {
    const auto& anchor = poses_.back().map_from_submap;
    const double distance = (map_from_sensor.translation() - anchor.translation()).norm();
    // 球形裁剪范围与朝向无关；原地旋转应补全当前子图，不生成重复锚点。
    create = distance >= config_.translation_threshold;
  }

  SubmapUpdate result;
  if (create) {
    Cloud candidate = crop(static_map, map_from_sensor);
    // 尚无附近静态点时不创建空子图，等待有效地图覆盖当前位置。
    if (!candidate.empty()) {
      if (!poses_.empty()) {
        result.completed = Submap{poses_.back(), extract(static_map, poses_.back().id)};
      }
      SubmapPose pose{poses_.size(), stamp_ns, map_from_sensor};
      poses_.push_back(pose);
      result.active = Submap{pose, std::move(candidate)};
      result.created = true;
    }
  }
  if (!result.active && !poses_.empty()) {
    // 包括空结果也替换发布，不能继续显示已经被 FreeDOM 清除的旧点。
    result.active = Submap{poses_.back(), extract(static_map, poses_.back().id)};
  }
  last_stamp_ns_ = stamp_ns;
  return result;
}

}  // namespace map_processing
