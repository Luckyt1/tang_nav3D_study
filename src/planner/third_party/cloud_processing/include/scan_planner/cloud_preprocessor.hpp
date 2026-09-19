#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Geometry>

namespace scan_planner
{

// 点云预处理推荐流程：
// 1. filterFiniteAndRange：去除 NaN/Inf 和范围外点；
// 2. removeOutliers：使用统计邻域滤波删除孤立噪声点；
// 3. voxelDownsample：使用体素网格降采样，降低点数和计算量；
// 4. alignToGravity：根据 IMU/里程计姿态消除 roll、pitch，使 Z 轴对齐重力；
// 5. process：按 1 -> 2 -> 3 -> 4 顺序自动执行完整流程。

struct CloudPreprocessorConfig
{
  float voxel_size{0.03F};  // 体素边长，单位：m。
  float min_range{0.5F};   // 保留点的最小距离，单位：m。
  float max_range{8.0F};   // 保留点的最大距离，单位：m。
  int sor_mean_k{16};      // 离群点滤波计算邻域平均距离时使用的邻居数。
  double sor_stddev{1.0};  // 离群点阈值的标准差倍数，越小过滤越严格。
};

class CloudPreprocessor
{
public:
  using Point = pcl::PointXYZ;
  using Cloud = pcl::PointCloud<Point>;
  using CloudPtr = Cloud::Ptr;

  // 输入：预处理配置。输出：构造完成的处理器；非法配置抛出异常。
  explicit CloudPreprocessor(CloudPreprocessorConfig config = {});

  // 输入：新的预处理配置。输出：无；非法配置抛出 std::invalid_argument。
  void setConfig(const CloudPreprocessorConfig & config);
  // 输出：当前配置的只读引用，引用随处理器对象持续有效。
  const CloudPreprocessorConfig & config() const noexcept { return config_; }

  // 输入：原始点云。输出：有限坐标且距离在 [min_range, max_range] 内的新点云。
  CloudPtr filterFiniteAndRange(const Cloud & input) const;
  // 输入：待滤波点云。输出：删除统计离群点后的新点云。
  CloudPtr removeOutliers(const Cloud & input) const;
  // 输入：待降采样点云。输出：按 voxel_size 降采样后的新点云。
  CloudPtr voxelDownsample(const Cloud & input) const;
  // 输入：点云和其相对世界坐标系的姿态(w,x,y,z)。
  // 输出：消除 roll/pitch、保留 yaw 且 Z 轴与重力对齐的新点云。
  CloudPtr alignToGravity(const Cloud & input,
                          const Eigen::Quaternionf & orientation) const;

  // 输入：原始点云，以及点云坐标系相对世界坐标系的姿态(w,x,y,z)。
  // 输出：依次完成过滤、离群点剔除、降采样和重力对齐的新点云；输入不会被修改。
  CloudPtr process(const Cloud & input,
                   const Eigen::Quaternionf & orientation = Eigen::Quaternionf::Identity()) const;

private:
  CloudPreprocessorConfig config_;
};

}  // namespace scan_planner
