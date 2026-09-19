#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Core>
#include <vector>

namespace scan_planner
{

struct GroundSeparatorConfig
{
  float distance_threshold{0.08F};  // 点到地面平面的最大距离，单位：m。
  int max_iterations{100};          // RANSAC 最大迭代次数。
  float max_ground_slope_deg{20.0F};  // 地面法向量与 Z 轴允许的最大夹角。
};

struct GroundPlaneResult
{
  using Cloud = pcl::PointCloud<pcl::PointXYZ>;
  Cloud::Ptr ground{new Cloud};
  Cloud::Ptr obstacles{new Cloud};
  Eigen::Vector4f plane{0.0F, 0.0F, 1.0F, 0.0F};  // ax+by+cz+d=0。
  float slope_deg{0.0F};
  // 以下数组与输入点云索引一一对应：高度为点到估计平面的有符号距离。
  std::vector<float> point_heights;
  std::vector<Eigen::Vector3f> point_normals;
  std::vector<float> point_slopes_deg;
  bool valid{false};
};

class CloudCut
{
public:
  using Cloud = pcl::PointCloud<pcl::PointXYZ>;

  explicit CloudCut(GroundSeparatorConfig config = {});

  // 输入：待分割点云；输出：地面、障碍物、平面参数和地面坡度。
  GroundPlaneResult separate(const Cloud & input) const;

private:
  GroundSeparatorConfig config_;
};

}  // namespace scan_planner
