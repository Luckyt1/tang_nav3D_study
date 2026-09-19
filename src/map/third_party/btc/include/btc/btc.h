#ifndef MAP_THIRD_PARTY_BTC_BTC_H_
#define MAP_THIRD_PARTY_BTC_BTC_H_

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Core>
#include <Eigen/Dense>

#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#define HASH_P 116101
#define MAX_N 10000000000

typedef struct ConfigSetting {
  /* for submap process*/
  double cloud_ds_size_ = 0.25;

  /* for binary descriptor*/
  int useful_corner_num_ = 30;
  float plane_merge_normal_thre_ = 0.1F;
  float plane_merge_dis_thre_ = 0.3F;
  float plane_detection_thre_ = 0.01F;
  float voxel_size_ = 1.0F;
  int voxel_init_num_ = 10;
  int proj_plane_num_ = 1;
  float proj_image_resolution_ = 0.5F;
  float proj_image_high_inc_ = 0.5F;
  float proj_dis_min_ = 0;
  float proj_dis_max_ = 5;
  float summary_min_thre_ = 10;
  int line_filter_enable_ = 0;

  /* for triangle descriptor */
  float descriptor_near_num_ = 10;
  float descriptor_min_len_ = 1;
  float descriptor_max_len_ = 10;
  float non_max_suppression_radius_ = 3.0F;
  float std_side_resolution_ = 0.2F;

  /* for place recognition*/
  int skip_near_num_ = 20;
  int candidate_num_ = 50;
  int sub_frame_num_ = 10;
  float rough_dis_threshold_ = 0.03F;
  float similarity_threshold_ = 0.7F;
  float icp_threshold_ = 0.5F;
  float normal_threshold_ = 0.1F;
  float dis_threshold_ = 0.3F;

  /* extrinsic for lidar to vehicle*/
  Eigen::Matrix3d rot_lidar_to_vehicle_ = Eigen::Matrix3d::Identity();
  Eigen::Vector3d t_lidar_to_vehicle_ = Eigen::Vector3d::Zero();

  /* for gt file style*/
  int gt_file_style_ = 0;
} ConfigSetting;

typedef struct BinaryDescriptor {
  std::vector<bool> occupy_array_;
  unsigned char summary_ = 0;
  Eigen::Vector3d location_ = Eigen::Vector3d::Zero();
} BinaryDescriptor;

// Binary Triangle Descriptor
typedef struct BTC {
  Eigen::Vector3d triangle_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d angle_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d center_ = Eigen::Vector3d::Zero();
  std::uint64_t frame_number_ = 0;
  BinaryDescriptor binary_A_;
  BinaryDescriptor binary_B_;
  BinaryDescriptor binary_C_;
} BTC;

typedef struct Plane {
  pcl::PointXYZINormal p_center_{};
  Eigen::Vector3d center_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d normal_ = Eigen::Vector3d::Zero();
  Eigen::Matrix3d covariance_ = Eigen::Matrix3d::Zero();
  float radius_ = 0;
  float min_eigen_value_ = 1;
  float d_ = 0;
  int id_ = 0;
  int sub_plane_num_ = 0;
  int points_size_ = 0;
  bool is_plane_ = false;
} Plane;

typedef struct BTCMatchList {
  std::vector<std::pair<BTC, BTC>> match_list_;
  std::pair<int, int> match_id_;
  int match_frame_;
  double mean_dis_;
} BTCMatchList;

struct M_POINT {
  float xyz[3];
  float intensity;
  int count = 0;
};

class VOXEL_LOC {
 public:
  std::int64_t x, y, z;

  VOXEL_LOC(std::int64_t vx = 0, std::int64_t vy = 0, std::int64_t vz = 0)
      : x(vx), y(vy), z(vz) {}

  bool operator==(const VOXEL_LOC &other) const {
    return (x == other.x && y == other.y && z == other.z);
  }
};

namespace std {
template <>
struct hash<VOXEL_LOC> {
  std::size_t operator()(const VOXEL_LOC &s) const {
    return static_cast<std::size_t>(
        ((((s.z) * HASH_P) % MAX_N + (s.y)) * HASH_P) % MAX_N + (s.x));
  }
};
}  // namespace std

class BTC_LOC {
 public:
  std::int64_t x, y, z, a, b, c;

  BTC_LOC(std::int64_t vx = 0, std::int64_t vy = 0, std::int64_t vz = 0,
          std::int64_t va = 0, std::int64_t vb = 0, std::int64_t vc = 0)
      : x(vx), y(vy), z(vz), a(va), b(vb), c(vc) {}

  bool operator==(const BTC_LOC &other) const {
    return (x == other.x && y == other.y && z == other.z);
  }
};

namespace std {
template <>
struct hash<BTC_LOC> {
  std::size_t operator()(const BTC_LOC &s) const {
    return static_cast<std::size_t>(
        ((((s.z) * HASH_P) % MAX_N + (s.y)) * HASH_P) % MAX_N + (s.x));
  }
};
}  // namespace std

class OctoTree {
 public:
  explicit OctoTree(const ConfigSetting &config_setting);

  ConfigSetting config_setting_;
  std::vector<Eigen::Vector3d> voxel_points_;
  std::shared_ptr<Plane> plane_ptr_;
  int layer_;
  int octo_state_;
  int merge_num_ = 0;
  bool is_project_ = false;
  std::vector<Eigen::Vector3d> project_normal;
  bool is_publish_ = false;
  OctoTree *leaves_[8];
  double voxel_center_[3];
  float quater_length_;
  bool init_octo_;

  bool is_check_connect_[6];
  bool connect_[6];
  OctoTree *connect_tree_[6];

  void init_plane();
  void init_octo_tree();
};

void down_sampling_voxel(pcl::PointCloud<pcl::PointXYZI> &pl_feat,
                         double voxel_size);

double binary_similarity(const BinaryDescriptor &b1,
                         const BinaryDescriptor &b2);

bool binary_greater_sort(BinaryDescriptor a, BinaryDescriptor b);
bool plane_greater_sort(std::shared_ptr<Plane> plane1,
                        std::shared_ptr<Plane> plane2);

class BtcDescManager {
 public:
  BtcDescManager() = default;
  explicit BtcDescManager(const ConfigSetting &config_setting)
      : config_setting_(config_setting) {}

  ConfigSetting config_setting_;
  bool print_debug_info_ = false;

  std::unordered_map<BTC_LOC, std::vector<BTC>> data_base_;
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> key_cloud_vec_;
  std::vector<std::vector<BinaryDescriptor>> history_binary_list_;
  std::vector<pcl::PointCloud<pcl::PointXYZINormal>::Ptr> plane_cloud_vec_;

  void GenerateBtcDescs(const pcl::PointCloud<pcl::PointXYZI>::Ptr &input_cloud,
                        std::uint64_t frame_id, std::vector<BTC> &btcs_vec);

  void AddBtcDescs(const std::vector<BTC> &btcs_vec);

 private:
  void init_voxel_map(const pcl::PointCloud<pcl::PointXYZI>::Ptr &input_cloud,
                      std::unordered_map<VOXEL_LOC, OctoTree *> &voxel_map);

  void get_plane(const std::unordered_map<VOXEL_LOC, OctoTree *> &voxel_map,
                 pcl::PointCloud<pcl::PointXYZINormal>::Ptr &plane_cloud);

  void get_project_plane(
      std::unordered_map<VOXEL_LOC, OctoTree *> &feat_map,
      std::vector<std::shared_ptr<Plane>> &project_plane_list);

  void merge_plane(std::vector<std::shared_ptr<Plane>> &origin_list,
                   std::vector<std::shared_ptr<Plane>> &merge_plane_list);

  void binary_extractor(
      const std::vector<std::shared_ptr<Plane>> &proj_plane_list,
      const pcl::PointCloud<pcl::PointXYZI>::Ptr &input_cloud,
      std::vector<BinaryDescriptor> &binary_descriptor_list);

  void extract_binary(const Eigen::Vector3d &project_center,
                      const Eigen::Vector3d &project_normal,
                      const pcl::PointCloud<pcl::PointXYZI>::Ptr &input_cloud,
                      std::vector<BinaryDescriptor> &binary_list);

  void non_maxi_suppression(std::vector<BinaryDescriptor> &binary_list);

  void generate_btc(const std::vector<BinaryDescriptor> &binary_list,
                    std::uint64_t frame_id, std::vector<BTC> &btc_list);
};

#endif  // MAP_THIRD_PARTY_BTC_BTC_H_
