#include "planner/planner.h"

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "scan_planner/cloud_preprocessor.hpp"
#include "scan_planner/cloud_cut.hpp"

#include <cmath>
#include <functional>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

class Odom : public rclcpp::Node
{
public:
  using Cloud = pcl::PointCloud<pcl::PointXYZ>;
  using CloudMsg = sensor_msgs::msg::PointCloud2;

  Odom()
  : Node("odom")
  {
    RCLCPP_INFO(this->get_logger(), "Odom node started.");
    // 参数化预处理配置，便于不重新编译就调整传感器和场景对应的阈值。
    scan_planner::CloudPreprocessorConfig config;
    config.voxel_size = declare_parameter<double>("cloud.voxel_size", 0.03);
    config.min_range = declare_parameter<double>("cloud.min_range", 0.5);
    config.max_range = declare_parameter<double>("cloud.max_range", 8.0);
    config.sor_mean_k = declare_parameter<int>("cloud.sor_mean_k", 16);
    config.sor_stddev = declare_parameter<double>("cloud.sor_stddev", 1.0);
    preprocessor_ = std::make_unique<scan_planner::CloudPreprocessor>(config);
    scan_planner::GroundSeparatorConfig cut_config;
    cut_config.distance_threshold = declare_parameter<double>("cloud_cut.distance_threshold", 0.08);
    cut_config.max_iterations = declare_parameter<int>("cloud_cut.max_iterations", 100);
    cut_config.max_ground_slope_deg = declare_parameter<double>("cloud_cut.max_ground_slope_deg", 20.0);
    cloud_cut_ = std::make_unique<scan_planner::CloudCut>(cut_config);
    downsampled_pub_ =
      create_publisher<CloudMsg>("/cloud/downsampled", 10);//发布降采样后的点云，便于调试和可视化。
    ground_pub_ =
      create_publisher<CloudMsg>("/cloud/ground", 10);  // 预留：地面点云。
    obstacles_pub_ =
      create_publisher<CloudMsg>("/cloud/obstacles", 10);  // 预留：障碍物点云。



    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odin1/odometry",
      rclcpp::SensorDataQoS(),
      std::bind(
        &Odom::odomCallback,
        this,
        std::placeholders::_1
      )
    );
    cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/odin1/cloud_slam",
      rclcpp::SensorDataQoS(),
      std::bind(
        &Odom::cloudCallback,
        this,
        std::placeholders::_1
      )
    );  
 
  }

private:
  void cloudCallback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    // cloud_slam 已在 odom 系中。只做滤波和降采样，不能再用设备姿态旋转，
    // 否则点坐标与下面沿用的输入 frame_id 不一致。
    Cloud input;
    pcl::fromROSMsg(*msg, input);
    const auto processed = preprocessor_->process(input);
    CloudMsg output_msg;
    pcl::toROSMsg(*processed, output_msg);
    // 发布降采样后的点云，便于调试和可视化。
    output_msg.header = msg->header;
    downsampled_pub_->publish(output_msg);

    const auto separated = cloud_cut_->separate(*processed);
    CloudMsg ground_msg;
    pcl::toROSMsg(*separated.ground, ground_msg);
    ground_msg.header = msg->header;
    ground_pub_->publish(ground_msg);
    CloudMsg obstacles_msg;
    pcl::toROSMsg(*separated.obstacles, obstacles_msg);
    obstacles_msg.header = msg->header;
    obstacles_pub_->publish(obstacles_msg);

  }

  void odomCallback(
    const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    // 转发里程计为 TF；设备姿态不参与 odom 系点云的预处理。
    geometry_msgs::msg::TransformStamped transform;

    transform.header.stamp = msg->header.stamp;
    transform.header.frame_id = msg->header.frame_id;
    transform.child_frame_id = msg->child_frame_id;
    transform.transform.translation.x =
      msg->pose.pose.position.x;
    transform.transform.translation.y =
      msg->pose.pose.position.y;
    transform.transform.translation.z =
      msg->pose.pose.position.z;
    transform.transform.rotation =
      msg->pose.pose.orientation;
    tf_broadcaster_->sendTransform(transform);
    RCLCPP_INFO(
      this->get_logger(),
      "Rotation: %f %f %f %f",
      transform.transform.rotation.x,
      transform.transform.rotation.y,
      transform.transform.rotation.z,
      transform.transform.rotation.w
    );
  }

private:
  rclcpp::Publisher<CloudMsg>::SharedPtr downsampled_pub_;
  rclcpp::Publisher<CloudMsg>::SharedPtr ground_pub_;
  rclcpp::Publisher<CloudMsg>::SharedPtr obstacles_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster>
    tf_broadcaster_;
  std::unique_ptr<scan_planner::CloudPreprocessor> preprocessor_;
  std::unique_ptr<scan_planner::CloudCut> cloud_cut_;
};

std::shared_ptr<rclcpp::Node> planner::createCloudPreprocessorNode()
{
  return std::make_shared<Odom>();
}
