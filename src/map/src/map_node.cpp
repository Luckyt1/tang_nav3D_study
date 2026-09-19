#include <algorithm>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include "map/freedom.hpp"
#include "map/submap.hpp"

// map_node.cpp 负责 ROS 输入输出和坐标转换；动态去除和子图划分分别交给两个处理器。
class MapNode : public rclcpp::Node
{
public:
  using CloudMsg = sensor_msgs::msg::PointCloud2;

  MapNode()
  : Node("freedom_map")
  {
    const auto input_topic = declare_parameter<std::string>("input_topic", "/odin1/cloud_slam");
    const auto output_topic = declare_parameter<std::string>("output_topic", "/map/static_cloud");
    map_frame_ = declare_parameter<std::string>("map_frame", "odom");
    sensor_frame_ = declare_parameter<std::string>("sensor_frame", "lidar");
    // 10 Hz 里程计的下一帧 TF 可能晚于 100 ms 到达；留出传输/调度余量。
    tf_timeout_ = declare_parameter<double>("tf_timeout", 0.2);
    if (input_topic.empty() || output_topic.empty() || input_topic == output_topic ||
      map_frame_.empty() || sensor_frame_.empty() || map_frame_ == sensor_frame_ ||
      !std::isfinite(tf_timeout_) || tf_timeout_ < 0.0 || tf_timeout_ > 1.0)
    {
      throw std::invalid_argument("invalid topics, frames, or tf_timeout (expected 0..1 seconds)");
    }

    map_processing::FreedomConfig config;
    config.sensor_min_range = declare_parameter<double>("sensor.min_range", config.sensor_min_range);
    config.sensor_max_range = declare_parameter<double>("sensor.max_range", config.sensor_max_range);
    config.sensor_min_z = declare_parameter<double>("sensor.min_z", config.sensor_min_z);
    config.sensor_max_z = declare_parameter<double>("sensor.max_z", config.sensor_max_z);
    config.sub_voxel_size = declare_parameter<double>("freedom.sub_voxel_size", config.sub_voxel_size);
    config.counts_to_free = declare_parameter<int>("freedom.counts_to_free", config.counts_to_free);
    config.counts_to_revert = declare_parameter<int>("freedom.counts_to_revert", config.counts_to_revert);
    config.num_threads = declare_parameter<int>("freedom.num_threads", config.num_threads);
    freedom_ = std::make_unique<map_processing::FreedomProcessor>(config);

    const auto save_directory = declare_parameter<std::string>("save.directory", "maps");
    if (save_directory.empty()) {
      throw std::invalid_argument("save.directory must not be empty");
    }
    save_options_.directory = std::filesystem::absolute(save_directory).lexically_normal();
    save_options_.map_frame = map_frame_;
    save_options_.sensor_frame = sensor_frame_;
    save_options_.freedom = config;

    auto& btc = save_options_.btc;
    btc.enabled = declare_parameter<bool>("btc.enabled", btc.enabled);
    btc.useful_corner_num = declare_parameter<int>("btc.useful_corner_num", btc.useful_corner_num);
    btc.voxel_size = declare_parameter<double>("btc.voxel_size", btc.voxel_size);
    btc.voxel_init_num = declare_parameter<int>("btc.voxel_init_num", btc.voxel_init_num);
    btc.plane_detection_threshold = declare_parameter<double>(
      "btc.plane_detection_threshold", btc.plane_detection_threshold);
    btc.plane_merge_normal_threshold = declare_parameter<double>(
      "btc.plane_merge_normal_threshold", btc.plane_merge_normal_threshold);
    btc.plane_merge_distance_threshold = declare_parameter<double>(
      "btc.plane_merge_distance_threshold", btc.plane_merge_distance_threshold);
    btc.projection_plane_num = declare_parameter<int>(
      "btc.projection_plane_num", btc.projection_plane_num);
    btc.projection_resolution = declare_parameter<double>(
      "btc.projection_resolution", btc.projection_resolution);
    btc.projection_height_increment = declare_parameter<double>(
      "btc.projection_height_increment", btc.projection_height_increment);
    btc.projection_distance_min = declare_parameter<double>(
      "btc.projection_distance_min", btc.projection_distance_min);
    btc.projection_distance_max = declare_parameter<double>(
      "btc.projection_distance_max", btc.projection_distance_max);
    btc.summary_min_threshold = declare_parameter<int>(
      "btc.summary_min_threshold", btc.summary_min_threshold);
    btc.line_filter_enabled = declare_parameter<bool>(
      "btc.line_filter_enabled", btc.line_filter_enabled);
    btc.descriptor_near_num = declare_parameter<int>(
      "btc.descriptor_near_num", btc.descriptor_near_num);
    btc.descriptor_min_length = declare_parameter<double>(
      "btc.descriptor_min_length", btc.descriptor_min_length);
    btc.descriptor_max_length = declare_parameter<double>(
      "btc.descriptor_max_length", btc.descriptor_max_length);
    btc.non_max_suppression_radius = declare_parameter<double>(
      "btc.non_max_suppression_radius", btc.non_max_suppression_radius);
    btc.triangle_resolution = declare_parameter<double>(
      "btc.triangle_resolution", btc.triangle_resolution);
    map_processing::validateBtcConfig(btc);

    map_processing::SubmapConfig submap_config;
    submap_config.translation_threshold = declare_parameter<double>(
      "submap.translation_threshold", submap_config.translation_threshold);
    submap_config.radius = declare_parameter<double>("submap.radius", submap_config.radius);
    submaps_ = std::make_unique<map_processing::SubmapBuilder>(submap_config);

    // 驱动已发布 odom -> imu -> lidar。直接查询完整 TF，包含雷达相对 IMU 的外参。
    // TF 使用独立线程接收，点云回调等待 TF 时不会阻塞 TF 本身。
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_, this, true);
    submap_tf_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(this);
    cloud_pub_ = create_publisher<CloudMsg>(output_topic, rclcpp::QoS(1));
    const auto submap_qos = rclcpp::QoS(1).transient_local();
    submap_pub_ = create_publisher<CloudMsg>("/map/submap_cloud", submap_qos);
    completed_submap_pub_ = create_publisher<CloudMsg>("/map/submap_completed", submap_qos);
    submap_poses_pub_ = create_publisher<geometry_msgs::msg::PoseArray>(
      "/map/submap_poses", submap_qos);
    save_service_ = create_service<std_srvs::srv::Trigger>(
      "/map/save_map",
      [this](const std_srvs::srv::Trigger::Request::SharedPtr,
      std_srvs::srv::Trigger::Response::SharedPtr response) {
        try {
          // 默认单线程回调：保存期间地图和锚点保持同一时刻，不会边写边修改。
          const auto saved_directory = submaps_->save(latest_static_map_, save_options_);
          response->success = true;
          response->message = saved_directory.string();
          RCLCPP_INFO(get_logger(), "Saved map: %s", response->message.c_str());
        } catch (const std::exception & error) {
          response->success = false;
          response->message = error.what();
          RCLCPP_ERROR(get_logger(), "Map save failed: %s", error.what());
        }
      });
    cloud_sub_ = create_subscription<CloudMsg>(
      input_topic, rclcpp::SensorDataQoS().keep_last(5),
      std::bind(&MapNode::cloudCallback, this, std::placeholders::_1));
    RCLCPP_INFO(
      get_logger(), "FreeDOM: %s -> %s, map=%s, sensor=%s",
      input_topic.c_str(), output_topic.c_str(), map_frame_.c_str(), sensor_frame_.c_str());
    RCLCPP_INFO(get_logger(), "Timestamped TF wait timeout: %.3f s", tf_timeout_);
    RCLCPP_INFO(
      get_logger(), "Submaps: translation=%.2f m, radius=%.2f m (rotation does not create submaps)",
      submap_config.translation_threshold, submap_config.radius);
    RCLCPP_INFO(
      get_logger(), "Save service: /map/save_map, directory=%s", save_options_.directory.c_str());
    RCLCPP_INFO(get_logger(), "BTC extraction on save: %s", btc.enabled ? "enabled" : "disabled");
  }

private:
  static std::string submapFrame(std::size_t id)
  {
    return "map_submap_" + std::to_string(id);
  }

  static geometry_msgs::msg::Pose poseMessage(const Eigen::Isometry3d & transform)
  {
    geometry_msgs::msg::Pose pose;
    pose.position.x = transform.translation().x();
    pose.position.y = transform.translation().y();
    pose.position.z = transform.translation().z();
    const Eigen::Quaterniond rotation(transform.linear());
    pose.orientation.w = rotation.w();
    pose.orientation.x = rotation.x();
    pose.orientation.y = rotation.y();
    pose.orientation.z = rotation.z();
    return pose;
  }

  void publishSubmap(
    const map_processing::Submap & submap, const rclcpp::Time & stamp,
    const rclcpp::Publisher<CloudMsg>::SharedPtr & publisher)
  {
    CloudMsg cloud;
    pcl::toROSMsg(submap.cloud, cloud);
    cloud.header.stamp = stamp;
    cloud.header.frame_id = submapFrame(submap.pose.id);
    publisher->publish(cloud);
  }

  void publishSubmaps(const map_processing::SubmapUpdate & update, const rclcpp::Time & stamp)
  {
    if (!update.active) {
      return;
    }
    if (update.created) {
      const auto & anchor = update.active->pose;
      const auto pose = poseMessage(anchor.map_from_submap);
      geometry_msgs::msg::TransformStamped tf;
      tf.header.stamp = rclcpp::Time(anchor.stamp_ns, get_clock()->get_clock_type());
      tf.header.frame_id = map_frame_;
      tf.child_frame_id = submapFrame(anchor.id);
      tf.transform.translation.x = pose.position.x;
      tf.transform.translation.y = pose.position.y;
      tf.transform.translation.z = pose.position.z;
      tf.transform.rotation = pose.orientation;
      submap_tf_broadcaster_->sendTransform(tf);

      geometry_msgs::msg::PoseArray poses;
      poses.header.stamp = stamp;
      poses.header.frame_id = map_frame_;
      poses.poses.reserve(submaps_->poses().size());
      for (const auto & submap_pose : submaps_->poses()) {
        poses.poses.push_back(poseMessage(submap_pose.map_from_submap));
      }
      submap_poses_pub_->publish(poses);
      RCLCPP_INFO(
        get_logger(), "Created submap %zu: points=%zu, anchor=(%.2f, %.2f, %.2f)",
        anchor.id, update.active->cloud.size(), pose.position.x, pose.position.y, pose.position.z);
    }
    if (update.completed) {
      publishSubmap(*update.completed, stamp, completed_submap_pub_);
    }
    publishSubmap(*update.active, stamp, submap_pub_);
  }

  Eigen::Isometry3d lookupTransform(
    const std::string & target, const std::string & source, const rclcpp::Time & stamp)
  {
    if (target == source) {
      return Eigen::Isometry3d::Identity();
    }
    const auto tf = tf_buffer_->lookupTransform(
      target, source, stamp, rclcpp::Duration::from_seconds(tf_timeout_));
    const auto & rotation = tf.transform.rotation;
    const auto & translation = tf.transform.translation;
    Eigen::Quaterniond q(rotation.w, rotation.x, rotation.y, rotation.z);
    Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
    transform.translation() = Eigen::Vector3d(translation.x, translation.y, translation.z);
    if (!q.coeffs().allFinite() || q.norm() < 1e-6 || !transform.translation().allFinite()) {
      throw std::invalid_argument("TF contains an invalid rotation or translation");
    }
    transform.linear() = q.normalized().toRotationMatrix();
    return transform;
  }

  void cloudCallback(const CloudMsg::ConstSharedPtr msg)
  {
    // stamp=0 在 TF 中表示“最新”，不能用它替代点云采集时刻。
    if (msg->header.frame_id.empty() || msg->header.stamp.sec < 0 ||
      msg->header.stamp.nanosec >= 1000000000U ||
      (msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0) ||
      msg->width == 0 || msg->height == 0)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "Skip empty cloud/frame or invalid stamp");
      return;
    }
    const rclcpp::Time stamp(msg->header.stamp, get_clock()->get_clock_type());
    if (stamp.nanoseconds() <= last_stamp_ns_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "Skip duplicate/out-of-order cloud; restart this node after restarting a bag");
      return;
    }

    // PCL 对缺少字段可能只打印警告；在转换前明确检查，避免把错误数据写入历史地图。
    for (const auto * name : {"x", "y", "z"}) {
      const auto field = std::find_if(
        msg->fields.begin(), msg->fields.end(),
        [name](const auto & value) {return value.name == name;});
      if (field == msg->fields.end() || field->datatype != sensor_msgs::msg::PointField::FLOAT32 ||
        field->count != 1 || static_cast<std::uint64_t>(field->offset) + sizeof(float) > msg->point_step)
      {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "Cloud needs FLOAT32 x/y/z fields");
        return;
      }
    }
    if (msg->is_bigendian ||
      static_cast<std::uint64_t>(msg->width) * msg->point_step > msg->row_step ||
      static_cast<std::uint64_t>(msg->height) * msg->row_step > msg->data.size())
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "Skip unsupported/malformed cloud layout");
      return;
    }

    try {
      const auto map_from_sensor = lookupTransform(map_frame_, sensor_frame_, stamp);
      // cloud_slam 已在 odom 系中：先还原到雷达系，由 FreeDOM 统一变换并做射线分析。
      const Eigen::Isometry3d sensor_from_cloud = msg->header.frame_id == map_frame_ ?
        map_from_sensor.inverse() : lookupTransform(sensor_frame_, msg->header.frame_id, stamp);
      map_processing::Cloud input;
      pcl::fromROSMsg(*msg, input);
      map_processing::Cloud sensor_cloud;
      pcl::transformPointCloud(input, sensor_cloud, sensor_from_cloud.matrix());

      // 核心调用：处理器在回调之间保留历史自由空间，返回当前整张静态地图。
      auto static_map = freedom_->process(sensor_cloud, map_from_sensor);
      CloudMsg output;
      pcl::toROSMsg(static_map, output);
      output.header.stamp = msg->header.stamp;
      output.header.frame_id = map_frame_;
      cloud_pub_->publish(output);
      last_stamp_ns_ = stamp.nanoseconds();

      // 不叠加整图：按运动选择锚点，用本次最新地图替换局部子图内容。
      const auto submap_update = submaps_->update(static_map, map_from_sensor, stamp.nanoseconds());
      latest_static_map_ = std::move(static_map);
      publishSubmaps(submap_update, stamp);
    } catch (const tf2::TransformException & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000, "Skip cloud: timestamped TF unavailable: %s", error.what());
    } catch (const std::exception & error) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 3000, "Map processing failed: %s", error.what());
    }
  }

  std::string map_frame_;
  std::string sensor_frame_;
  double tf_timeout_{};
  std::int64_t last_stamp_ns_{-1};
  std::unique_ptr<map_processing::FreedomProcessor> freedom_;
  std::unique_ptr<map_processing::SubmapBuilder> submaps_;
  map_processing::Cloud latest_static_map_;
  map_processing::MapSaveOptions save_options_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> submap_tf_broadcaster_;
  rclcpp::Publisher<CloudMsg>::SharedPtr cloud_pub_;
  rclcpp::Publisher<CloudMsg>::SharedPtr submap_pub_;
  rclcpp::Publisher<CloudMsg>::SharedPtr completed_submap_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr submap_poses_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_service_;
  rclcpp::Subscription<CloudMsg>::SharedPtr cloud_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int result = 0;
  try {
    // 单线程顺序更新地图，避免多个点云回调同时修改 FreeDOM 状态。
    rclcpp::spin(std::make_shared<MapNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("freedom_map"), "%s", error.what());
    result = 1;
  }
  rclcpp::shutdown();
  return result;
}
