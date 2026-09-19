#ifndef RELOCALIZATION_RELOCALIZATION_HPP_
#define RELOCALIZATION_RELOCALIZATION_HPP_

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Geometry>

#include "map/freedom.hpp"
#include "map/submap.hpp"

namespace relocalization_processing {

using Cloud = map_processing::Cloud;

struct SearchConfig {
  std::size_t top_k{5};
  std::size_t min_votes{5};
  std::size_t min_inliers{4};
  std::size_t max_hypotheses{64};
  std::size_t max_matches_per_candidate{2000};
  double length_tolerance{0.02};
  double binary_similarity{0.7};
  double max_vertex_error{0.5};
};

struct QueryConfig {
  std::size_t frames_per_query{10};
  double max_duration{3.0};
  double history_timeout{30.0};
};

struct SearchDiagnostics {
  std::size_t best_votes{0};
  std::size_t best_inliers{0};
  std::size_t required_votes{0};
  std::size_t required_inliers{0};
  std::size_t best_length_votes{0};
};

struct Candidate {
  std::uint64_t submap_id{0};
  std::size_t votes{0};
  std::size_t inliers{0};
  double mean_binary_similarity{0.0};
  double vertex_rmse{0.0};
  Eigen::Isometry3d submap_from_query{Eigen::Isometry3d::Identity()};
  Eigen::Isometry3d map_from_query{Eigen::Isometry3d::Identity()};
};

struct QueryResult {
  std::int64_t stamp_ns{0};
  std::size_t frame_count{0};
  std::size_t binary_count{0};
  std::size_t triangle_count{0};
  bool used_orientation_retry{false};
  Eigen::Isometry3d odom_from_query{Eigen::Isometry3d::Identity()};
  Cloud query_cloud;
  std::vector<Candidate> candidates;
  SearchDiagnostics diagnostics;
  std::string failure_reason;
};

class MapDatabase {
 public:
  explicit MapDatabase(const std::filesystem::path& snapshot_directory,
                       const SearchConfig& search = {});
  ~MapDatabase();

  MapDatabase(const MapDatabase&);
  MapDatabase& operator=(const MapDatabase&);
  MapDatabase(MapDatabase&&) noexcept;
  MapDatabase& operator=(MapDatabase&&) noexcept;

  const map_processing::BtcConfig& btcConfig() const;
  const map_processing::FreedomConfig& freedomConfig() const;
  double radius() const;
  const std::string& mapFrame() const;
  std::size_t size() const;

  std::vector<Candidate> search(const map_processing::BtcFeatures& query) const;
  std::vector<Candidate> search(const map_processing::BtcFeatures& query,
                                SearchDiagnostics& diagnostics) const;
  // 已保存的完整地图，坐标已经是建图参考系；不再施加子图锚点变换。
  Cloud loadGlobalMap() const;
  Cloud loadSubmap(std::uint64_t submap_id) const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

class RelocalizationProcessor {
 public:
  explicit RelocalizationProcessor(const std::filesystem::path& snapshot_directory,
                                   const QueryConfig& query = {},
                                   const SearchConfig& search = {});
  ~RelocalizationProcessor();

  RelocalizationProcessor(const RelocalizationProcessor&) = delete;
  RelocalizationProcessor& operator=(const RelocalizationProcessor&) = delete;
  RelocalizationProcessor(RelocalizationProcessor&&) noexcept;
  RelocalizationProcessor& operator=(RelocalizationProcessor&&) noexcept;

  std::optional<QueryResult> process(const Cloud& sensor_cloud,
                                     const Eigen::Isometry3d& odom_from_sensor,
                                     std::int64_t stamp_ns);
  const MapDatabase& database() const;
  // Start a new odometry timeline without reloading the saved map.
  void reset();

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace relocalization_processing

#endif  // RELOCALIZATION_RELOCALIZATION_HPP_
