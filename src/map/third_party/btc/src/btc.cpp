#include "btc/btc.h"

#include <pcl/common/io.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <tuple>

namespace {

bool isFinite(const Eigen::Vector3d &v) { return v.allFinite(); }

bool isFinite(const pcl::PointXYZI &p) {
  return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) &&
         std::isfinite(p.intensity);
}

void free_voxel_map(std::unordered_map<VOXEL_LOC, OctoTree *> &voxel_map) {
  for (auto &item : voxel_map) {
    delete item.second;
  }
  voxel_map.clear();
}

class VoxelMapCleanup {
 public:
  explicit VoxelMapCleanup(std::unordered_map<VOXEL_LOC, OctoTree *> &voxel_map)
      : voxel_map_(voxel_map) {}
  ~VoxelMapCleanup() { free_voxel_map(voxel_map_); }

  VoxelMapCleanup(const VoxelMapCleanup &) = delete;
  VoxelMapCleanup &operator=(const VoxelMapCleanup &) = delete;

 private:
  std::unordered_map<VOXEL_LOC, OctoTree *> &voxel_map_;
};

std::int64_t checked_voxel_index(double loc) {
  const long double value = static_cast<long double>(loc);
  if (!std::isfinite(loc) ||
      value < static_cast<long double>(
                  std::numeric_limits<std::int64_t>::min()) ||
      value > static_cast<long double>(
                  std::numeric_limits<std::int64_t>::max())) {
    throw std::invalid_argument("BTC voxel coordinate is outside int64 range");
  }
  return static_cast<std::int64_t>(loc);
}

int checked_grid_length(double span, double resolution, int base,
                        const char *axis_name) {
  if (!std::isfinite(span) || span < 0.0 || !std::isfinite(resolution) ||
      resolution <= 0.0) {
    throw std::runtime_error(std::string("BTC projection grid has invalid ") +
                             axis_name + " span");
  }
  const double units = span / resolution;
  if (!std::isfinite(units) ||
      units >
          static_cast<double>(std::numeric_limits<int>::max() - base)) {
    throw std::runtime_error(std::string("BTC projection grid ") + axis_name +
                             " axis is too large");
  }
  return std::max(base, static_cast<int>(units) + base);
}

int checked_segment_count(double span, double segment_length,
                          const char *axis_name) {
  if (!std::isfinite(segment_length) || segment_length <= 0.0) {
    throw std::runtime_error("BTC projection segment length is invalid");
  }
  const double units = span / segment_length;
  if (!std::isfinite(units) ||
      units > static_cast<double>(std::numeric_limits<int>::max() - 1)) {
    throw std::runtime_error(std::string("BTC projection segment ") +
                             axis_name + " axis is too large");
  }
  return std::max(1, static_cast<int>(units) + 1);
}

std::uint64_t rounded_bucket(double value) {
  return static_cast<std::uint64_t>(std::floor(value + 0.5));
}

}  // namespace

OctoTree::OctoTree(const ConfigSetting &config_setting)
    : config_setting_(config_setting) {
  voxel_points_.clear();
  octo_state_ = 0;
  layer_ = 0;
  init_octo_ = false;
  quater_length_ = 0;
  voxel_center_[0] = 0;
  voxel_center_[1] = 0;
  voxel_center_[2] = 0;
  for (int i = 0; i < 8; i++) {
    leaves_[i] = nullptr;
  }
  for (int i = 0; i < 6; i++) {
    is_check_connect_[i] = false;
    connect_[i] = false;
    connect_tree_[i] = nullptr;
  }
  plane_ptr_.reset(new Plane);
}

void down_sampling_voxel(pcl::PointCloud<pcl::PointXYZI> &pl_feat,
                         double voxel_size) {
  if (voxel_size < 0.01) {
    return;
  }
  std::unordered_map<VOXEL_LOC, M_POINT> voxel_map;
  const std::size_t plsize = pl_feat.size();

  for (std::size_t i = 0; i < plsize; i++) {
    pcl::PointXYZI &p_c = pl_feat[i];
    if (!isFinite(p_c)) {
      continue;
    }
    float loc_xyz[3];
    for (int j = 0; j < 3; j++) {
      loc_xyz[j] = p_c.data[j] / voxel_size;
      if (loc_xyz[j] < 0) {
        loc_xyz[j] -= 1.0F;
      }
    }

    VOXEL_LOC position(static_cast<std::int64_t>(loc_xyz[0]),
                       static_cast<std::int64_t>(loc_xyz[1]),
                       static_cast<std::int64_t>(loc_xyz[2]));
    auto iter = voxel_map.find(position);
    if (iter != voxel_map.end()) {
      iter->second.xyz[0] += p_c.x;
      iter->second.xyz[1] += p_c.y;
      iter->second.xyz[2] += p_c.z;
      iter->second.intensity += p_c.intensity;
      iter->second.count++;
    } else {
      M_POINT anp;
      anp.xyz[0] = p_c.x;
      anp.xyz[1] = p_c.y;
      anp.xyz[2] = p_c.z;
      anp.intensity = p_c.intensity;
      anp.count = 1;
      voxel_map[position] = anp;
    }
  }

  pl_feat.clear();
  pl_feat.resize(voxel_map.size());

  std::size_t i = 0;
  for (auto iter = voxel_map.begin(); iter != voxel_map.end(); ++iter) {
    pl_feat[i].x = iter->second.xyz[0] / iter->second.count;
    pl_feat[i].y = iter->second.xyz[1] / iter->second.count;
    pl_feat[i].z = iter->second.xyz[2] / iter->second.count;
    pl_feat[i].intensity = iter->second.intensity / iter->second.count;
    i++;
  }
}

double binary_similarity(const BinaryDescriptor &b1,
                         const BinaryDescriptor &b2) {
  const auto denom = static_cast<double>(b1.summary_) + b2.summary_;
  if (denom <= 0.0) {
    return 0.0;
  }
  const std::size_t common_size =
      std::min(b1.occupy_array_.size(), b2.occupy_array_.size());
  double same_occupied = 0;
  for (std::size_t i = 0; i < common_size; i++) {
    if (b1.occupy_array_[i] && b2.occupy_array_[i]) {
      same_occupied += 1.0;
    }
  }
  return 2.0 * same_occupied / denom;
}

bool binary_greater_sort(BinaryDescriptor a, BinaryDescriptor b) {
  if (a.summary_ != b.summary_) return a.summary_ > b.summary_;
  return std::make_tuple(a.location_.x(), a.location_.y(), a.location_.z()) <
         std::make_tuple(b.location_.x(), b.location_.y(), b.location_.z());
}

bool plane_greater_sort(std::shared_ptr<Plane> plane1,
                        std::shared_ptr<Plane> plane2) {
  if (!plane1 || !plane2) return static_cast<bool>(plane1);
  if (plane1->points_size_ != plane2->points_size_)
    return plane1->points_size_ > plane2->points_size_;
  return std::make_tuple(plane1->center_.x(), plane1->center_.y(), plane1->center_.z()) <
         std::make_tuple(plane2->center_.x(), plane2->center_.y(), plane2->center_.z());
}

void OctoTree::init_octo_tree() {
  if (voxel_points_.size() >
      static_cast<std::size_t>(std::max(2, config_setting_.voxel_init_num_))) {
    init_plane();
  }
}

void OctoTree::init_plane() {
  plane_ptr_->covariance_ = Eigen::Matrix3d::Zero();
  plane_ptr_->center_ = Eigen::Vector3d::Zero();
  plane_ptr_->normal_ = Eigen::Vector3d::Zero();
  plane_ptr_->points_size_ = static_cast<int>(voxel_points_.size());
  plane_ptr_->radius_ = 0;
  plane_ptr_->is_plane_ = false;
  if (voxel_points_.size() < 3) {
    return;
  }
  for (const auto &pi : voxel_points_) {
    if (!isFinite(pi)) {
      return;
    }
    plane_ptr_->covariance_ += pi * pi.transpose();
    plane_ptr_->center_ += pi;
  }
  plane_ptr_->center_ = plane_ptr_->center_ / plane_ptr_->points_size_;
  plane_ptr_->covariance_ =
      plane_ptr_->covariance_ / plane_ptr_->points_size_ -
      plane_ptr_->center_ * plane_ptr_->center_.transpose();
  if (!plane_ptr_->covariance_.allFinite()) {
    return;
  }

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(plane_ptr_->covariance_);
  if (es.info() != Eigen::Success || !es.eigenvalues().allFinite()) {
    return;
  }
  const Eigen::Vector3d evals = es.eigenvalues().cwiseMax(0.0);
  Eigen::Index evalsMin = 0;
  Eigen::Index evalsMax = 0;
  evals.minCoeff(&evalsMin);
  evals.maxCoeff(&evalsMax);
  if (evals(evalsMin) < config_setting_.plane_detection_thre_) {
    plane_ptr_->normal_ = es.eigenvectors().col(evalsMin).normalized();
    if (!plane_ptr_->normal_.allFinite()) {
      plane_ptr_->normal_ = Eigen::Vector3d::Zero();
      return;
    }
    plane_ptr_->min_eigen_value_ = static_cast<float>(evals(evalsMin));
    plane_ptr_->radius_ = static_cast<float>(std::sqrt(evals(evalsMax)));
    plane_ptr_->is_plane_ = true;

    plane_ptr_->d_ = static_cast<float>(-plane_ptr_->normal_.dot(
        plane_ptr_->center_));
    plane_ptr_->p_center_.x = plane_ptr_->center_(0);
    plane_ptr_->p_center_.y = plane_ptr_->center_(1);
    plane_ptr_->p_center_.z = plane_ptr_->center_(2);
    plane_ptr_->p_center_.normal_x = plane_ptr_->normal_(0);
    plane_ptr_->p_center_.normal_y = plane_ptr_->normal_(1);
    plane_ptr_->p_center_.normal_z = plane_ptr_->normal_(2);
  }
}

void BtcDescManager::GenerateBtcDescs(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &source_cloud,
    std::uint64_t frame_id, std::vector<BTC> &btcs_vec) {
  btcs_vec.clear();
  pcl::PointCloud<pcl::PointXYZINormal>::Ptr plane_cloud(
      new pcl::PointCloud<pcl::PointXYZINormal>);
  std::vector<BinaryDescriptor> binary_list;

  // Canonical point order makes voxel insertion, covariance accumulation and
  // projection averages independent of the incoming PointCloud2 point order.
  auto input_cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  if (source_cloud) {
    input_cloud->reserve(source_cloud->size());
    for (const auto &point : source_cloud->points) {
      if (isFinite(point)) input_cloud->push_back(point);
    }
  }
  std::sort(input_cloud->begin(), input_cloud->end(), [](const auto &a, const auto &b) {
    return std::tie(a.x, a.y, a.z) < std::tie(b.x, b.y, b.z);
  });
  if (input_cloud->empty()) {
    plane_cloud_vec_.push_back(plane_cloud);
    history_binary_list_.push_back(binary_list);
    return;
  }

  std::unordered_map<VOXEL_LOC, OctoTree *> voxel_map;
  VoxelMapCleanup voxel_map_cleanup(voxel_map);
  init_voxel_map(input_cloud, voxel_map);
  get_plane(voxel_map, plane_cloud);
  plane_cloud_vec_.push_back(plane_cloud);

  std::vector<std::shared_ptr<Plane>> proj_plane_list;
  std::vector<std::shared_ptr<Plane>> merge_plane_list;
  get_project_plane(voxel_map, proj_plane_list);
  if (proj_plane_list.empty()) {
    std::shared_ptr<Plane> single_plane(new Plane);
    single_plane->normal_ << 0, 0, 1;
    single_plane->center_ = Eigen::Vector3d::Zero();
    for (const auto &point : input_cloud->points) {
      if (isFinite(point)) {
        single_plane->center_ << point.x, point.y, point.z;
        break;
      }
    }
    single_plane->points_size_ = static_cast<int>(input_cloud->size());
    merge_plane_list.push_back(single_plane);
  } else {
    std::sort(proj_plane_list.begin(), proj_plane_list.end(),
              plane_greater_sort);
    merge_plane(proj_plane_list, merge_plane_list);
    std::sort(merge_plane_list.begin(), merge_plane_list.end(),
              plane_greater_sort);
  }

  binary_extractor(merge_plane_list, input_cloud, binary_list);
  history_binary_list_.push_back(binary_list);
  generate_btc(binary_list, frame_id, btcs_vec);
}

void BtcDescManager::AddBtcDescs(const std::vector<BTC> &btcs_vec) {
  for (const auto &single_std : btcs_vec) {
    BTC_LOC position;
    position.x = static_cast<std::int64_t>(rounded_bucket(single_std.triangle_[0]));
    position.y = static_cast<std::int64_t>(rounded_bucket(single_std.triangle_[1]));
    position.z = static_cast<std::int64_t>(rounded_bucket(single_std.triangle_[2]));
    data_base_[position].push_back(single_std);
  }
}

void BtcDescManager::init_voxel_map(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &input_cloud,
    std::unordered_map<VOXEL_LOC, OctoTree *> &voxel_map) {
  if (!input_cloud || config_setting_.voxel_size_ <= 0.0F) {
    return;
  }
  for (std::size_t i = 0; i < input_cloud->size(); i++) {
    const auto &point = input_cloud->points[i];
    if (!isFinite(point)) {
      continue;
    }
    Eigen::Vector3d p_c(point.x, point.y, point.z);
    double loc_xyz[3];
    for (int j = 0; j < 3; j++) {
      loc_xyz[j] = p_c[j] / config_setting_.voxel_size_;
      if (loc_xyz[j] < 0) {
        loc_xyz[j] -= 1.0;
      }
    }
    VOXEL_LOC position(checked_voxel_index(loc_xyz[0]),
                       checked_voxel_index(loc_xyz[1]),
                       checked_voxel_index(loc_xyz[2]));
    auto iter = voxel_map.find(position);
    if (iter != voxel_map.end()) {
      iter->second->voxel_points_.push_back(p_c);
    } else {
      OctoTree *octo_tree = new OctoTree(config_setting_);
      voxel_map[position] = octo_tree;
      voxel_map[position]->voxel_points_.push_back(p_c);
    }
  }
  for (auto iter = voxel_map.begin(); iter != voxel_map.end(); ++iter) {
    iter->second->init_octo_tree();
  }
}

void BtcDescManager::get_plane(
    const std::unordered_map<VOXEL_LOC, OctoTree *> &voxel_map,
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr &plane_cloud) {
  for (auto iter = voxel_map.begin(); iter != voxel_map.end(); iter++) {
    if (iter->second && iter->second->plane_ptr_->is_plane_) {
      pcl::PointXYZINormal pi;
      pi.x = iter->second->plane_ptr_->center_[0];
      pi.y = iter->second->plane_ptr_->center_[1];
      pi.z = iter->second->plane_ptr_->center_[2];
      pi.intensity = 0;
      pi.normal_x = iter->second->plane_ptr_->normal_[0];
      pi.normal_y = iter->second->plane_ptr_->normal_[1];
      pi.normal_z = iter->second->plane_ptr_->normal_[2];
      pi.curvature = 0;
      plane_cloud->push_back(pi);
    }
  }
}

void BtcDescManager::get_project_plane(
    std::unordered_map<VOXEL_LOC, OctoTree *> &voxel_map,
    std::vector<std::shared_ptr<Plane>> &project_plane_list) {
  std::vector<std::shared_ptr<Plane>> origin_list;
  for (auto iter = voxel_map.begin(); iter != voxel_map.end(); iter++) {
    if (iter->second && iter->second->plane_ptr_->is_plane_) {
      origin_list.push_back(iter->second->plane_ptr_);
    }
  }
  if (origin_list.size() < 2) {
    project_plane_list.clear();
    return;
  }
  std::sort(origin_list.begin(), origin_list.end(), plane_greater_sort);
  for (std::size_t i = 0; i < origin_list.size(); i++) origin_list[i]->id_ = 0;
  int current_id = 1;
  for (auto iter = origin_list.end() - 1; iter != origin_list.begin(); iter--) {
    for (auto iter2 = origin_list.begin(); iter2 != iter; iter2++) {
      Eigen::Vector3d normal_diff = (*iter)->normal_ - (*iter2)->normal_;
      Eigen::Vector3d normal_add = (*iter)->normal_ + (*iter2)->normal_;
      double dis1 =
          std::fabs((*iter)->normal_(0) * (*iter2)->center_(0) +
                    (*iter)->normal_(1) * (*iter2)->center_(1) +
                    (*iter)->normal_(2) * (*iter2)->center_(2) + (*iter)->d_);
      double dis2 =
          std::fabs((*iter2)->normal_(0) * (*iter)->center_(0) +
                    (*iter2)->normal_(1) * (*iter)->center_(1) +
                    (*iter2)->normal_(2) * (*iter)->center_(2) + (*iter2)->d_);
      if ((normal_diff.norm() < config_setting_.plane_merge_normal_thre_ ||
           normal_add.norm() < config_setting_.plane_merge_normal_thre_) &&
          dis1 < config_setting_.plane_merge_dis_thre_ &&
          dis2 < config_setting_.plane_merge_dis_thre_) {
        if ((*iter)->id_ == 0 && (*iter2)->id_ == 0) {
          (*iter)->id_ = current_id;
          (*iter2)->id_ = current_id;
          current_id++;
        } else if ((*iter)->id_ == 0 && (*iter2)->id_ != 0) {
          (*iter)->id_ = (*iter2)->id_;
        } else if ((*iter)->id_ != 0 && (*iter2)->id_ == 0) {
          (*iter2)->id_ = (*iter)->id_;
        } else if ((*iter)->id_ != (*iter2)->id_) {
          // A compatible pair can bridge two groups that were already labeled.
          const int removed_id = (*iter2)->id_;
          const int merged_id = (*iter)->id_;
          for (auto &plane : origin_list) {
            if (plane->id_ == removed_id) plane->id_ = merged_id;
          }
        }
      }
    }
  }

  std::vector<std::shared_ptr<Plane>> merge_list;
  std::vector<int> merge_flag;
  for (std::size_t i = 0; i < origin_list.size(); i++) {
    auto it =
        std::find(merge_flag.begin(), merge_flag.end(), origin_list[i]->id_);
    if (it != merge_flag.end() || origin_list[i]->id_ == 0) {
      continue;
    }
    std::shared_ptr<Plane> merge_plane(new Plane);
    (*merge_plane) = (*origin_list[i]);
    bool is_merge = false;
    for (std::size_t j = 0; j < origin_list.size(); j++) {
      if (i == j || origin_list[j]->id_ != origin_list[i]->id_) continue;
      is_merge = true;
      Eigen::Matrix3d p_pt1 =
          (merge_plane->covariance_ +
           merge_plane->center_ * merge_plane->center_.transpose()) *
          merge_plane->points_size_;
      Eigen::Matrix3d p_pt2 =
          (origin_list[j]->covariance_ +
           origin_list[j]->center_ * origin_list[j]->center_.transpose()) *
          origin_list[j]->points_size_;
      const int merged_points =
          merge_plane->points_size_ + origin_list[j]->points_size_;
      if (merged_points <= 0) {
        continue;
      }
      Eigen::Vector3d merge_center =
          (merge_plane->center_ * merge_plane->points_size_ +
           origin_list[j]->center_ * origin_list[j]->points_size_) /
          merged_points;
      Eigen::Matrix3d merge_covariance =
          (p_pt1 + p_pt2) / merged_points -
          merge_center * merge_center.transpose();
      merge_plane->covariance_ = merge_covariance;
      merge_plane->center_ = merge_center;
      merge_plane->points_size_ = merged_points;
      merge_plane->sub_plane_num_++;
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(
          merge_plane->covariance_);
      if (es.info() != Eigen::Success) {
        continue;
      }
      const Eigen::Vector3d evals = es.eigenvalues().cwiseMax(0.0);
      Eigen::Index evalsMin = 0;
      Eigen::Index evalsMax = 0;
      evals.minCoeff(&evalsMin);
      evals.maxCoeff(&evalsMax);
      merge_plane->normal_ = es.eigenvectors().col(evalsMin).normalized();
      merge_plane->radius_ = static_cast<float>(std::sqrt(evals(evalsMax)));
      merge_plane->d_ =
          static_cast<float>(-merge_plane->normal_.dot(merge_plane->center_));
      merge_plane->p_center_.x = merge_plane->center_(0);
      merge_plane->p_center_.y = merge_plane->center_(1);
      merge_plane->p_center_.z = merge_plane->center_(2);
      merge_plane->p_center_.normal_x = merge_plane->normal_(0);
      merge_plane->p_center_.normal_y = merge_plane->normal_(1);
      merge_plane->p_center_.normal_z = merge_plane->normal_(2);
    }
    if (is_merge) {
      merge_flag.push_back(merge_plane->id_);
      merge_list.push_back(merge_plane);
    }
  }
  project_plane_list = merge_list;
}

void BtcDescManager::merge_plane(
    std::vector<std::shared_ptr<Plane>> &origin_list,
    std::vector<std::shared_ptr<Plane>> &merge_plane_list) {
  merge_plane_list.clear();
  if (origin_list.empty()) {
    return;
  }
  if (origin_list.size() == 1) {
    merge_plane_list = origin_list;
    return;
  }
  std::sort(origin_list.begin(), origin_list.end(), plane_greater_sort);
  for (std::size_t i = 0; i < origin_list.size(); i++) origin_list[i]->id_ = 0;
  int current_id = 1;
  for (auto iter = origin_list.end() - 1; iter != origin_list.begin(); iter--) {
    for (auto iter2 = origin_list.begin(); iter2 != iter; iter2++) {
      Eigen::Vector3d normal_diff = (*iter)->normal_ - (*iter2)->normal_;
      Eigen::Vector3d normal_add = (*iter)->normal_ + (*iter2)->normal_;
      double dis1 =
          std::fabs((*iter)->normal_(0) * (*iter2)->center_(0) +
                    (*iter)->normal_(1) * (*iter2)->center_(1) +
                    (*iter)->normal_(2) * (*iter2)->center_(2) + (*iter)->d_);
      double dis2 =
          std::fabs((*iter2)->normal_(0) * (*iter)->center_(0) +
                    (*iter2)->normal_(1) * (*iter)->center_(1) +
                    (*iter2)->normal_(2) * (*iter)->center_(2) + (*iter2)->d_);
      if ((normal_diff.norm() < config_setting_.plane_merge_normal_thre_ ||
           normal_add.norm() < config_setting_.plane_merge_normal_thre_) &&
          dis1 < config_setting_.plane_merge_dis_thre_ &&
          dis2 < config_setting_.plane_merge_dis_thre_) {
        if ((*iter)->id_ == 0 && (*iter2)->id_ == 0) {
          (*iter)->id_ = current_id;
          (*iter2)->id_ = current_id;
          current_id++;
        } else if ((*iter)->id_ == 0 && (*iter2)->id_ != 0) {
          (*iter)->id_ = (*iter2)->id_;
        } else if ((*iter)->id_ != 0 && (*iter2)->id_ == 0) {
          (*iter2)->id_ = (*iter)->id_;
        } else if ((*iter)->id_ != (*iter2)->id_) {
          // A compatible pair can bridge two groups that were already labeled.
          const int removed_id = (*iter2)->id_;
          const int merged_id = (*iter)->id_;
          for (auto &plane : origin_list) {
            if (plane->id_ == removed_id) plane->id_ = merged_id;
          }
        }
      }
    }
  }

  std::vector<int> merge_flag;
  for (std::size_t i = 0; i < origin_list.size(); i++) {
    auto it =
        std::find(merge_flag.begin(), merge_flag.end(), origin_list[i]->id_);
    if (it != merge_flag.end()) continue;
    if (origin_list[i]->id_ == 0) {
      merge_plane_list.push_back(origin_list[i]);
      continue;
    }
    std::shared_ptr<Plane> merge_plane(new Plane);
    (*merge_plane) = (*origin_list[i]);
    bool is_merge = false;
    for (std::size_t j = 0; j < origin_list.size(); j++) {
      if (i == j || origin_list[j]->id_ != origin_list[i]->id_) continue;
      is_merge = true;
      Eigen::Matrix3d p_pt1 =
          (merge_plane->covariance_ +
           merge_plane->center_ * merge_plane->center_.transpose()) *
          merge_plane->points_size_;
      Eigen::Matrix3d p_pt2 =
          (origin_list[j]->covariance_ +
           origin_list[j]->center_ * origin_list[j]->center_.transpose()) *
          origin_list[j]->points_size_;
      const int merged_points =
          merge_plane->points_size_ + origin_list[j]->points_size_;
      if (merged_points <= 0) {
        continue;
      }
      Eigen::Vector3d merge_center =
          (merge_plane->center_ * merge_plane->points_size_ +
           origin_list[j]->center_ * origin_list[j]->points_size_) /
          merged_points;
      Eigen::Matrix3d merge_covariance =
          (p_pt1 + p_pt2) / merged_points -
          merge_center * merge_center.transpose();
      merge_plane->covariance_ = merge_covariance;
      merge_plane->center_ = merge_center;
      merge_plane->points_size_ = merged_points;
      merge_plane->sub_plane_num_ += origin_list[j]->sub_plane_num_;
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(
          merge_plane->covariance_);
      if (es.info() != Eigen::Success) {
        continue;
      }
      const Eigen::Vector3d evals = es.eigenvalues().cwiseMax(0.0);
      Eigen::Index evalsMin = 0;
      Eigen::Index evalsMax = 0;
      evals.minCoeff(&evalsMin);
      evals.maxCoeff(&evalsMax);
      merge_plane->normal_ = es.eigenvectors().col(evalsMin).normalized();
      merge_plane->radius_ = static_cast<float>(std::sqrt(evals(evalsMax)));
      merge_plane->d_ =
          static_cast<float>(-merge_plane->normal_.dot(merge_plane->center_));
      merge_plane->p_center_.x = merge_plane->center_(0);
      merge_plane->p_center_.y = merge_plane->center_(1);
      merge_plane->p_center_.z = merge_plane->center_(2);
      merge_plane->p_center_.normal_x = merge_plane->normal_(0);
      merge_plane->p_center_.normal_y = merge_plane->normal_(1);
      merge_plane->p_center_.normal_z = merge_plane->normal_(2);
    }
    if (is_merge) {
      merge_flag.push_back(merge_plane->id_);
      merge_plane_list.push_back(merge_plane);
    }
  }
}

void BtcDescManager::binary_extractor(
    const std::vector<std::shared_ptr<Plane>> &proj_plane_list,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &input_cloud,
    std::vector<BinaryDescriptor> &binary_descriptor_list) {
  binary_descriptor_list.clear();
  if (!input_cloud || input_cloud->empty() || proj_plane_list.empty()) {
    return;
  }
  std::vector<BinaryDescriptor> temp_binary_list;
  Eigen::Vector3d last_normal(0, 0, 0);
  int useful_proj_num = 0;
  for (std::size_t i = 0; i < proj_plane_list.size(); i++) {
    std::vector<BinaryDescriptor> prepare_binary_list;
    Eigen::Vector3d proj_center = proj_plane_list[i]->center_;
    Eigen::Vector3d proj_normal = proj_plane_list[i]->normal_;
    if (!proj_center.allFinite() || !proj_normal.allFinite() ||
        proj_normal.norm() < 1e-9) {
      continue;
    }
    // Face the sensor origin: a near-vertical wall must not flip its signed
    // height bins just because numerical noise changes normal.z()'s sign.
    if (proj_normal.dot(proj_center) > 0) {
      proj_normal = -proj_normal;
    }
    if ((proj_normal - last_normal).norm() < 0.3 ||
        (proj_normal + last_normal).norm() > 0.3) {
      last_normal = proj_normal;
      useful_proj_num++;
      extract_binary(proj_center, proj_normal, input_cloud,
                     prepare_binary_list);
      for (const auto &bi : prepare_binary_list) {
        temp_binary_list.push_back(bi);
      }
      if (useful_proj_num == config_setting_.proj_plane_num_) {
        break;
      }
    }
  }
  non_maxi_suppression(temp_binary_list);
  if (config_setting_.useful_corner_num_ >
      static_cast<int>(temp_binary_list.size())) {
    binary_descriptor_list = temp_binary_list;
  } else {
    std::sort(temp_binary_list.begin(), temp_binary_list.end(),
              binary_greater_sort);
    for (int i = 0; i < config_setting_.useful_corner_num_; i++) {
      binary_descriptor_list.push_back(temp_binary_list[i]);
    }
  }
}

void BtcDescManager::extract_binary(
    const Eigen::Vector3d &project_center,
    const Eigen::Vector3d &project_normal,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &input_cloud,
    std::vector<BinaryDescriptor> &binary_list) {
  binary_list.clear();
  const double binary_min_dis = config_setting_.summary_min_thre_;
  const double resolution = config_setting_.proj_image_resolution_;
  const double dis_threshold_min = config_setting_.proj_dis_min_;
  const double dis_threshold_max = config_setting_.proj_dis_max_;
  const double high_inc = config_setting_.proj_image_high_inc_;
  const bool line_filter_enable = config_setting_.line_filter_enable_ != 0;
  if (!input_cloud || input_cloud->empty() || resolution <= 0.0 ||
      high_inc <= 0.0 || dis_threshold_max <= dis_threshold_min ||
      project_normal.norm() < 1e-9) {
    return;
  }

  const Eigen::Vector3d normal = project_normal.normalized();
  const double A = normal[0];
  const double B = normal[1];
  const double C = normal[2];
  const double D =
      -(A * project_center[0] + B * project_center[1] + C * project_center[2]);
  // Both axes must be orthonormal and lie in the projection plane. The old
  // (1, 0, -(A+B)/C) vector was not perpendicular to general tilted normals.
  // Project a fixed reference axis, keeping near-horizontal planes stable
  // when tiny normal perturbations would rotate unitOrthogonal() arbitrarily.
  const Eigen::Vector3d reference = std::abs(normal.x()) < 0.9 ?
      Eigen::Vector3d::UnitX() : Eigen::Vector3d::UnitY();
  const Eigen::Vector3d x_axis = (reference - normal * normal.dot(reference)).normalized();
  const Eigen::Vector3d y_axis = normal.cross(x_axis).normalized();
  const double ax = x_axis[0];
  const double bx = x_axis[1];
  const double cx = x_axis[2];
  const double dx = -(ax * project_center[0] + bx * project_center[1] +
                      cx * project_center[2]);
  const double ay = y_axis[0];
  const double by = y_axis[1];
  const double cy = y_axis[2];
  const double dy = -(ay * project_center[0] + by * project_center[1] +
                      cy * project_center[2]);
  std::vector<Eigen::Vector2d> point_list_2d;
  std::vector<double> dis_list_2d;
  for (std::size_t i = 0; i < input_cloud->size(); i++) {
    const auto &point = input_cloud->points[i];
    if (!isFinite(point)) {
      continue;
    }
    const double x = point.x;
    const double y = point.y;
    const double z = point.z;
    const double dis = x * A + y * B + z * C + D;
    if (dis < dis_threshold_min || dis > dis_threshold_max) {
      continue;
    }

    Eigen::Vector3d cur_project;
    cur_project[0] = (-A * (B * y + C * z + D) + x * (B * B + C * C));
    cur_project[1] = (-B * (A * x + C * z + D) + y * (A * A + C * C));
    cur_project[2] = (-C * (A * x + B * y + D) + z * (A * A + B * B));
    const double denom = A * A + B * B + C * C;
    if (denom <= 0.0) {
      return;
    }
    cur_project /= denom;
    const double project_x =
        cur_project[0] * ay + cur_project[1] * by + cur_project[2] * cy + dy;
    const double project_y =
        cur_project[0] * ax + cur_project[1] * bx + cur_project[2] * cx + dx;
    point_list_2d.emplace_back(project_x, project_y);
    dis_list_2d.push_back(dis);
  }
  if (point_list_2d.size() <= 5) {
    return;
  }

  double min_x = std::numeric_limits<double>::infinity();
  double max_x = -std::numeric_limits<double>::infinity();
  double min_y = std::numeric_limits<double>::infinity();
  double max_y = -std::numeric_limits<double>::infinity();
  for (const auto &pi : point_list_2d) {
    min_x = std::min(min_x, pi[0]);
    max_x = std::max(max_x, pi[0]);
    min_y = std::min(min_y, pi[1]);
    max_y = std::max(max_y, pi[1]);
  }
  if (!std::isfinite(min_x) || !std::isfinite(max_x) ||
      !std::isfinite(min_y) || !std::isfinite(max_y)) {
    return;
  }

  const int segmen_base_num = 5;
  const double segmen_len = segmen_base_num * resolution;
  const double x_span = max_x - min_x;
  const double y_span = max_y - min_y;
  const int x_segment_num = checked_segment_count(x_span, segmen_len, "x");
  const int y_segment_num = checked_segment_count(y_span, segmen_len, "y");
  const int x_axis_len =
      checked_grid_length(x_span, resolution, segmen_base_num, "x");
  const int y_axis_len =
      checked_grid_length(y_span, resolution, segmen_base_num, "y");
  constexpr int kMaxProjectionCells = 4000000;
  const auto axis_size_64 =
      static_cast<std::int64_t>(x_axis_len) * y_axis_len;
  if (axis_size_64 > kMaxProjectionCells) {
    throw std::runtime_error(
        "BTC projection grid exceeds 4000000 cells; reduce submap span or "
        "increase projection resolution");
  }
  const int axis_size = static_cast<int>(axis_size_64);

  std::vector<std::vector<double>> dis_container(axis_size);
  std::vector<BinaryDescriptor> binary_container(axis_size);
  std::vector<double> img_count(axis_size, 0.0);
  std::vector<double> dis_array(axis_size, 0.0);
  std::vector<double> mean_x_list(axis_size, 0.0);
  std::vector<double> mean_y_list(axis_size, 0.0);
  auto index_of = [y_axis_len](int x, int y) { return x * y_axis_len + y; };

  for (std::size_t i = 0; i < point_list_2d.size(); i++) {
    int x_index = static_cast<int>((point_list_2d[i][0] - min_x) / resolution);
    int y_index = static_cast<int>((point_list_2d[i][1] - min_y) / resolution);
    if (x_index < 0 || x_index >= x_axis_len || y_index < 0 ||
        y_index >= y_axis_len) {
      continue;
    }
    const int idx = index_of(x_index, y_index);
    mean_x_list[idx] += point_list_2d[i][0];
    mean_y_list[idx] += point_list_2d[i][1];
    img_count[idx]++;
    dis_container[idx].push_back(dis_list_2d[i]);
  }

  const int cut_num =
      std::max(1, static_cast<int>((dis_threshold_max - dis_threshold_min) /
                                   high_inc));
  for (int x = 0; x < x_axis_len; x++) {
    for (int y = 0; y < y_axis_len; y++) {
      const int idx = index_of(x, y);
      if (img_count[idx] <= 0) {
        continue;
      }
      std::vector<bool> occup_list(cut_num, false);
      std::vector<double> cnt_list(cut_num, 0.0);
      for (double cell_dis : dis_container[idx]) {
        int cnt_index = static_cast<int>((cell_dis - dis_threshold_min) /
                                         high_inc);
        if (cnt_index < 0) {
          continue;
        }
        cnt_index = std::min(cnt_index, cut_num - 1);
        cnt_list[cnt_index]++;
      }
      double segmnt_dis = 0;
      for (int i = 0; i < cut_num; i++) {
        if (cnt_list[i] >= 1) {
          segmnt_dis++;
          occup_list[i] = true;
        }
      }
      dis_array[idx] = segmnt_dis;
      BinaryDescriptor single_binary;
      single_binary.occupy_array_ = occup_list;
      single_binary.summary_ =
          static_cast<unsigned char>(std::min(segmnt_dis, 255.0));
      binary_container[idx] = single_binary;
    }
  }

  std::vector<double> max_dis_list;
  std::vector<int> max_dis_x_index_list;
  std::vector<int> max_dis_y_index_list;
  for (int x_segment_index = 0; x_segment_index < x_segment_num;
       x_segment_index++) {
    for (int y_segment_index = 0; y_segment_index < y_segment_num;
         y_segment_index++) {
      double max_dis = 0;
      int max_dis_x_index = -1;
      int max_dis_y_index = -1;
      for (int x_index = x_segment_index * segmen_base_num;
           x_index < (x_segment_index + 1) * segmen_base_num &&
           x_index < x_axis_len;
           x_index++) {
        for (int y_index = y_segment_index * segmen_base_num;
             y_index < (y_segment_index + 1) * segmen_base_num &&
             y_index < y_axis_len;
             y_index++) {
          const double cell_dis = dis_array[index_of(x_index, y_index)];
          if (cell_dis > max_dis) {
            max_dis = cell_dis;
            max_dis_x_index = x_index;
            max_dis_y_index = y_index;
          }
        }
      }
      if (max_dis >= binary_min_dis && max_dis_x_index >= 0 &&
          max_dis_y_index >= 0) {
        max_dis_list.push_back(max_dis);
        max_dis_x_index_list.push_back(max_dis_x_index);
        max_dis_y_index_list.push_back(max_dis_y_index);
      }
    }
  }

  std::vector<Eigen::Vector2i> direction_list;
  direction_list.emplace_back(0, 1);
  direction_list.emplace_back(1, 0);
  direction_list.emplace_back(1, 1);
  direction_list.emplace_back(1, -1);
  for (std::size_t i = 0; i < max_dis_list.size(); i++) {
    Eigen::Vector2i p(max_dis_x_index_list[i], max_dis_y_index_list[i]);
    if (p[0] <= 0 || p[0] >= x_axis_len - 1 || p[1] <= 0 ||
        p[1] >= y_axis_len - 1) {
      continue;
    }
    bool is_add = true;
    if (line_filter_enable) {
      for (const auto &direction : direction_list) {
        Eigen::Vector2i p1 = p + direction;
        Eigen::Vector2i p2 = p - direction;
        if (p1[0] < 0 || p1[0] >= x_axis_len || p1[1] < 0 ||
            p1[1] >= y_axis_len || p2[0] < 0 || p2[0] >= x_axis_len ||
            p2[1] < 0 || p2[1] >= y_axis_len) {
          continue;
        }
        const double center_dis = dis_array[index_of(p[0], p[1])];
        const double dis1 = dis_array[index_of(p1[0], p1[1])];
        const double dis2 = dis_array[index_of(p2[0], p2[1])];
        const double threshold = center_dis - 3;
        if ((dis1 >= threshold && dis2 >= 0.5 * center_dis) ||
            (dis2 >= threshold && dis1 >= 0.5 * center_dis) ||
            (dis1 >= threshold && dis2 >= threshold)) {
          is_add = false;
        }
      }
    }
    const int cell_idx = index_of(max_dis_x_index_list[i], max_dis_y_index_list[i]);
    if (is_add && img_count[cell_idx] > 0.0) {
      const double px = mean_x_list[cell_idx] / img_count[cell_idx];
      const double py = mean_y_list[cell_idx] / img_count[cell_idx];
      Eigen::Vector3d coord = py * x_axis + px * y_axis + project_center;
      if (!coord.allFinite()) {
        continue;
      }
      BinaryDescriptor single_binary = binary_container[cell_idx];
      single_binary.location_ = coord;
      binary_list.push_back(single_binary);
    }
  }
}

void BtcDescManager::non_maxi_suppression(
    std::vector<BinaryDescriptor> &binary_list) {
  if (binary_list.empty() || config_setting_.non_max_suppression_radius_ <= 0) {
    return;
  }
  // Stable priority breaks equal-score ties instead of deleting both corners.
  std::sort(binary_list.begin(), binary_list.end(), binary_greater_sort);
  pcl::PointCloud<pcl::PointXYZ>::Ptr prepare_key_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::KdTreeFLANN<pcl::PointXYZ> kd_tree;
  std::vector<BinaryDescriptor> valid_binary_list;
  std::vector<int> pre_count_list;
  std::vector<bool> is_add_list;
  for (const auto &var : binary_list) {
    if (!var.location_.allFinite()) {
      continue;
    }
    pcl::PointXYZ pi;
    pi.x = var.location_[0];
    pi.y = var.location_[1];
    pi.z = var.location_[2];
    prepare_key_cloud->push_back(pi);
    valid_binary_list.push_back(var);
    pre_count_list.push_back(var.summary_);
    is_add_list.push_back(true);
  }
  if (prepare_key_cloud->empty()) {
    binary_list.clear();
    return;
  }
  kd_tree.setInputCloud(prepare_key_cloud);
  std::vector<int> pointIdxRadiusSearch;
  std::vector<float> pointRadiusSquaredDistance;
  const double radius = config_setting_.non_max_suppression_radius_;
  for (std::size_t i = 0; i < prepare_key_cloud->size(); i++) {
    pcl::PointXYZ searchPoint = prepare_key_cloud->points[i];
    if (kd_tree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch,
                             pointRadiusSquaredDistance) > 0) {
      for (std::size_t j = 0; j < pointIdxRadiusSearch.size(); ++j) {
        const int nearby_index = pointIdxRadiusSearch[j];
        if (nearby_index == static_cast<int>(i)) {
          continue;
        }
        if (pre_count_list[i] < pre_count_list[nearby_index] ||
            (pre_count_list[i] == pre_count_list[nearby_index] &&
             nearby_index < static_cast<int>(i))) {
          is_add_list[i] = false;
        }
      }
    }
  }
  std::vector<BinaryDescriptor> pass_binary_list;
  for (std::size_t i = 0; i < is_add_list.size(); i++) {
    if (is_add_list[i]) {
      pass_binary_list.push_back(valid_binary_list[i]);
    }
  }
  binary_list = pass_binary_list;
}

void BtcDescManager::generate_btc(
    const std::vector<BinaryDescriptor> &binary_list, std::uint64_t frame_id,
    std::vector<BTC> &btc_list) {
  btc_list.clear();
  if (binary_list.size() < 3 || config_setting_.std_side_resolution_ <= 0.0F) {
    return;
  }
  const double scale = 1.0 / config_setting_.std_side_resolution_;
  std::unordered_map<VOXEL_LOC, bool> feat_map;
  pcl::PointCloud<pcl::PointXYZ> key_cloud;
  for (const auto &var : binary_list) {
    if (!var.location_.allFinite()) {
      continue;
    }
    pcl::PointXYZ pi;
    pi.x = var.location_[0];
    pi.y = var.location_[1];
    pi.z = var.location_[2];
    key_cloud.push_back(pi);
  }
  if (key_cloud.size() < 3) {
    return;
  }
  pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kd_tree(
      new pcl::KdTreeFLANN<pcl::PointXYZ>);
  kd_tree->setInputCloud(key_cloud.makeShared());
  const int K = std::max(
      3, std::min(static_cast<int>(config_setting_.descriptor_near_num_),
                  static_cast<int>(key_cloud.size())));
  std::vector<int> pointIdxNKNSearch(K);
  std::vector<float> pointNKNSquaredDistance(K);
  for (std::size_t i = 0; i < key_cloud.size(); i++) {
    pcl::PointXYZ searchPoint = key_cloud.points[i];
    const int found = kd_tree->nearestKSearch(searchPoint, K, pointIdxNKNSearch,
                                             pointNKNSquaredDistance);
    if (found < 3) {
      continue;
    }
    for (int m = 1; m < found - 1; m++) {
      for (int n = m + 1; n < found; n++) {
        pcl::PointXYZ p1 = searchPoint;
        pcl::PointXYZ p2 = key_cloud.points[pointIdxNKNSearch[m]];
        pcl::PointXYZ p3 = key_cloud.points[pointIdxNKNSearch[n]];
        double a = std::sqrt(std::pow(p1.x - p2.x, 2) +
                             std::pow(p1.y - p2.y, 2) +
                             std::pow(p1.z - p2.z, 2));
        double b = std::sqrt(std::pow(p1.x - p3.x, 2) +
                             std::pow(p1.y - p3.y, 2) +
                             std::pow(p1.z - p3.z, 2));
        double c = std::sqrt(std::pow(p3.x - p2.x, 2) +
                             std::pow(p3.y - p2.y, 2) +
                             std::pow(p3.z - p2.z, 2));
        if (a > config_setting_.descriptor_max_len_ ||
            b > config_setting_.descriptor_max_len_ ||
            c > config_setting_.descriptor_max_len_ ||
            a < config_setting_.descriptor_min_len_ ||
            b < config_setting_.descriptor_min_len_ ||
            c < config_setting_.descriptor_min_len_) {
          continue;
        }
        double temp;
        Eigen::Vector3d A, B, C;
        Eigen::Vector3i l1, l2, l3;
        Eigen::Vector3i l_temp;
        l1 << 1, 2, 0;
        l2 << 1, 0, 3;
        l3 << 0, 2, 3;
        if (a > b) {
          temp = a;
          a = b;
          b = temp;
          l_temp = l1;
          l1 = l2;
          l2 = l_temp;
        }
        if (b > c) {
          temp = b;
          b = c;
          c = temp;
          l_temp = l2;
          l2 = l3;
          l3 = l_temp;
        }
        if (a > b) {
          temp = a;
          a = b;
          b = temp;
          l_temp = l1;
          l1 = l2;
          l2 = l_temp;
        }
        if (std::fabs(c - (a + b)) < 0.2) {
          continue;
        }

        pcl::PointXYZ d_p;
        d_p.x = a * 1000;
        d_p.y = b * 1000;
        d_p.z = c * 1000;
        VOXEL_LOC position(static_cast<std::int64_t>(d_p.x),
                           static_cast<std::int64_t>(d_p.y),
                           static_cast<std::int64_t>(d_p.z));
        auto iter = feat_map.find(position);
        BinaryDescriptor binary_A;
        BinaryDescriptor binary_B;
        BinaryDescriptor binary_C;
        if (iter == feat_map.end()) {
          if (l1[0] == l2[0]) {
            A << p1.x, p1.y, p1.z;
            binary_A = binary_list[i];
          } else if (l1[1] == l2[1]) {
            A << p2.x, p2.y, p2.z;
            binary_A = binary_list[pointIdxNKNSearch[m]];
          } else {
            A << p3.x, p3.y, p3.z;
            binary_A = binary_list[pointIdxNKNSearch[n]];
          }
          if (l1[0] == l3[0]) {
            B << p1.x, p1.y, p1.z;
            binary_B = binary_list[i];
          } else if (l1[1] == l3[1]) {
            B << p2.x, p2.y, p2.z;
            binary_B = binary_list[pointIdxNKNSearch[m]];
          } else {
            B << p3.x, p3.y, p3.z;
            binary_B = binary_list[pointIdxNKNSearch[n]];
          }
          if (l2[0] == l3[0]) {
            C << p1.x, p1.y, p1.z;
            binary_C = binary_list[i];
          } else if (l2[1] == l3[1]) {
            C << p2.x, p2.y, p2.z;
            binary_C = binary_list[pointIdxNKNSearch[m]];
          } else {
            C << p3.x, p3.y, p3.z;
            binary_C = binary_list[pointIdxNKNSearch[n]];
          }
          BTC single_descriptor;
          single_descriptor.binary_A_ = binary_A;
          single_descriptor.binary_B_ = binary_B;
          single_descriptor.binary_C_ = binary_C;
          single_descriptor.center_ = (A + B + C) / 3;
          single_descriptor.triangle_ << scale * a, scale * b, scale * c;
          single_descriptor.angle_ = Eigen::Vector3d::Zero();
          single_descriptor.frame_number_ = frame_id;
          feat_map[position] = true;
          btc_list.push_back(single_descriptor);
        }
      }
    }
  }
}
