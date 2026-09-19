#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>

#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/qos.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

#include "relocalization/fine_registration.hpp"
#include "relocalization/msg/candidate_array.hpp"
#include "relocalization/msg/fine_result.hpp"
#include "relocalization/msg/tracking_status.hpp"
#include "relocalization/pose_tracking.hpp"
#include "relocalization/relocalization.hpp"

// 和 map 一样：入口负责 ROS/TF，数据库、查询窗口和 BTC 匹配交给处理器。
class RelocalizationNode : public rclcpp::Node {
 public:
  using CloudMsg = sensor_msgs::msg::PointCloud2;

  RelocalizationNode() : Node("btc_relocalization") {
    const auto directory = declare_parameter<std::string>("map.directory", "");
    const auto input_topic = declare_parameter<std::string>("input_topic", "/odin1/cloud_slam");
    const auto global_map_topic = declare_parameter<std::string>(
      "global_map_topic", "/relocalization/global_map");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    sensor_frame_ = declare_parameter<std::string>("sensor_frame", "lidar");
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    tf_timeout_ = declare_parameter<double>("tf_timeout", 0.2);
    if (directory.empty()) {
      throw std::invalid_argument("set map.directory to a completed map snapshot containing BTC files");
    }
    if (input_topic.empty() || global_map_topic.empty() || input_topic == global_map_topic ||
        input_topic == "/relocalization/query_cloud" ||
        global_map_topic == "/relocalization/query_cloud" ||
        input_topic == "/relocalization/aligned_cloud" ||
        global_map_topic == "/relocalization/aligned_cloud" ||
        odom_frame_.empty() || sensor_frame_.empty() || map_frame_.empty() ||
        odom_frame_ == sensor_frame_ || map_frame_ == odom_frame_ || map_frame_ == sensor_frame_ ||
        !std::isfinite(tf_timeout_) || tf_timeout_ < 0 || tf_timeout_ > 1) {
      throw std::invalid_argument("invalid topics, distinct map/odom/sensor frames, or tf_timeout");
    }

    relocalization_processing::QueryConfig query;
    query.frames_per_query = countParameter("query.frames", query.frames_per_query, 100);
    query.max_duration = declare_parameter<double>("query.max_duration", query.max_duration);
    query.history_timeout = declare_parameter<double>("query.history_timeout", query.history_timeout);
    relocalization_processing::SearchConfig search;
    search.top_k = countParameter("search.top_k", search.top_k, 50);
    search.min_votes = countParameter("search.min_votes", search.min_votes, 10000);
    search.min_inliers = countParameter("search.min_inliers", search.min_inliers, 10000);
    search.max_hypotheses = countParameter("search.max_hypotheses", search.max_hypotheses, 1000);
    search.max_matches_per_candidate = countParameter(
      "search.max_matches_per_candidate", search.max_matches_per_candidate, 100000);
    search.length_tolerance = declare_parameter<double>(
      "search.length_tolerance", search.length_tolerance);
    search.binary_similarity = declare_parameter<double>(
      "search.binary_similarity", search.binary_similarity);
    search.max_vertex_error = declare_parameter<double>(
      "search.max_vertex_error", search.max_vertex_error);
    processor_ = std::make_unique<relocalization_processing::RelocalizationProcessor>(
      directory, query, search);
    fine_enabled_ = declare_parameter<bool>("fine.enabled", true);
    const bool publish_tf = declare_parameter<bool>("fine.publish_tf", true);
    relocalization_processing::FineConfig fine;
    fine.voxel_size = declare_parameter<double>("fine.voxel_size", fine.voxel_size);
    fine.max_correspondence_distance = declare_parameter<double>(
      "fine.max_correspondence_distance", fine.max_correspondence_distance);
    fine.max_iterations = static_cast<int>(countParameter(
      "fine.max_iterations", fine.max_iterations, 500));
    fine.max_candidates = countParameter("fine.max_candidates", fine.max_candidates, 50);
    fine.min_points = countParameter("fine.min_points", fine.min_points, 1000000);
    fine.inlier_distance = declare_parameter<double>("fine.inlier_distance", fine.inlier_distance);
    fine.min_overlap = declare_parameter<double>("fine.min_overlap", fine.min_overlap);
    fine.max_rmse = declare_parameter<double>("fine.max_rmse", fine.max_rmse);
    fine.max_translation_correction = declare_parameter<double>(
      "fine.max_translation_correction", fine.max_translation_correction);
    fine.max_rotation_correction = declare_parameter<double>(
      "fine.max_rotation_correction_deg", fine.max_rotation_correction * 180.0 / M_PI) *
      M_PI / 180.0;
    fine_registration_ = std::make_unique<relocalization_processing::FineRegistration>(
      processor_->database(), fine);
    relocalization_processing::TrackingConfig tracking;
    tracking.fix_timeout = declare_parameter<double>("tracking.fix_timeout", tracking.fix_timeout);
    tracking.odom_timeout = declare_parameter<double>("tracking.odom_timeout", tracking.odom_timeout);
    const double tracking_rate = declare_parameter<double>("tracking.publish_rate", 50.0);
    if (!std::isfinite(tracking_rate) || tracking_rate <= 0.0 || tracking_rate > 200.0) {
      throw std::invalid_argument("tracking.publish_rate must be in (0, 200] Hz");
    }
    tracking_ = std::make_unique<relocalization_processing::PoseTracking>(tracking);
    if (publish_tf && fine_enabled_) {
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }
    // 完整地图已位于保存地图的参考系，与本次运行的 odom 无关。
    const auto global_map = processor_->database().loadGlobalMap();
    CloudMsg global_map_message;
    pcl::toROSMsg(global_map, global_map_message);
    global_map_message.header.frame_id = map_frame_;
    global_map_message.header.stamp = now();
    global_map_pub_ = create_publisher<CloudMsg>(
      global_map_topic, rclcpp::QoS(1).reliable().transient_local());
    // 保留启动时这条静态地图，供之后打开的 RViz/订阅者读取。
    global_map_pub_->publish(global_map_message);
    RCLCPP_INFO(get_logger(), "Published global map: topic=%s, frame=%s, points=%zu (transient local)",
      global_map_pub_->get_topic_name(), map_frame_.c_str(), global_map.size());

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    // Dynamic poses must keep advancing even if an older DDS sample is lost.
    // Keep the TF history depth for timestamped cloud queries; static TF retains
    // its reliable, transient-local QoS so late listeners receive extrinsics.
    auto dynamic_tf_qos = tf2_ros::DynamicListenerQoS();
    dynamic_tf_qos.best_effort();
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(
      *tf_buffer_, this, true, dynamic_tf_qos);
    candidates_pub_ = create_publisher<relocalization::msg::CandidateArray>(
      "/relocalization/candidates", rclcpp::QoS(1));
    poses_pub_ = create_publisher<geometry_msgs::msg::PoseArray>(
      "/relocalization/candidate_poses", rclcpp::QoS(1));
    query_pub_ = create_publisher<CloudMsg>("/relocalization/query_cloud", rclcpp::QoS(1));
    fine_pub_ = create_publisher<relocalization::msg::FineResult>(
      "/relocalization/fine_result", rclcpp::QoS(1));
    refined_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "/relocalization/pose", rclcpp::QoS(1));
    aligned_pub_ = create_publisher<CloudMsg>("/relocalization/aligned_cloud", rclcpp::QoS(1));
    tracked_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "/relocalization/tracked_pose", rclcpp::QoS(1));
    tracking_status_pub_ = create_publisher<relocalization::msg::TrackingStatus>(
      "/relocalization/tracking_status", rclcpp::QoS(1));
    processing_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    tracking_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions cloud_options;
    cloud_options.callback_group = processing_group_;
    cloud_sub_ = create_subscription<CloudMsg>(
      input_topic, rclcpp::SensorDataQoS().keep_last(5),
      std::bind(&RelocalizationNode::cloudCallback, this, std::placeholders::_1), cloud_options);
    tracking_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / tracking_rate)),
      std::bind(&RelocalizationNode::publishTracking, this), tracking_group_);
    RCLCPP_INFO(get_logger(), "Loaded %zu submaps from %s (saved frame=%s, output frame=%s)",
      processor_->database().size(), directory.c_str(), processor_->database().mapFrame().c_str(),
      map_frame_.c_str());
    RCLCPP_INFO(get_logger(), "BTC extraction revision: %d (legacy descriptors rebuilt from PCD in memory)",
      map_processing::kBtcExtractionRevision);
    RCLCPP_INFO(get_logger(), "BTC query: every %zu frames, radius=%.2f m, retained local history, TF wait=%.3f s",
      query.frames_per_query, processor_->database().radius(), tf_timeout_);
    RCLCPP_INFO(get_logger(), "TF listener defaults: dynamic=best_effort, static=reliable/transient_local");
    RCLCPP_INFO(get_logger(), "ICP refinement: enabled=%s, voxel=%.2f m, max_iterations=%d, overlap>=%.2f, "
      "RMSE<=%.2f m, publish map->odom=%s",
      fine_enabled_ ? "true" : "false", fine.voxel_size, fine.max_iterations, fine.min_overlap, fine.max_rmse,
      tf_broadcaster_ ? "true" : "false");
    RCLCPP_INFO(get_logger(), "Pose tracking: up to %.1f Hz from advancing odometry TF, "
      "fix timeout=%.1f s, odometry timeout=%.1f s",
      tracking_rate, tracking.fix_timeout, tracking.odom_timeout);
  }

 private:
  std::size_t countParameter(const std::string& name, std::size_t value, std::int64_t maximum) {
    const auto result = declare_parameter<std::int64_t>(name, static_cast<std::int64_t>(value));
    if (result <= 0 || result > maximum) {
      throw std::invalid_argument(name + " is outside the supported positive range");
    }
    return static_cast<std::size_t>(result);
  }

  static geometry_msgs::msg::Pose poseMessage(const Eigen::Isometry3d& transform) {
    geometry_msgs::msg::Pose pose;
    pose.position.x = transform.translation().x();
    pose.position.y = transform.translation().y();
    pose.position.z = transform.translation().z();
    const Eigen::Quaterniond q(transform.linear());
    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();
    return pose;
  }

  static std::int64_t steadyNowNs() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
  }

  static Eigen::Isometry3d transformMatrix(const geometry_msgs::msg::Transform& value) {
    const auto& rotation = value.rotation;
    Eigen::Quaterniond q(rotation.w, rotation.x, rotation.y, rotation.z);
    Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
    transform.translation() = Eigen::Vector3d(value.translation.x, value.translation.y,
                                             value.translation.z);
    if (!q.coeffs().allFinite() || q.norm() < 1e-6 || !transform.translation().allFinite()) {
      throw std::invalid_argument("TF contains invalid rotation/translation");
    }
    transform.linear() = q.normalized().toRotationMatrix();
    return transform;
  }

  Eigen::Isometry3d lookupTransform(
    const std::string& target, const std::string& source, const rclcpp::Time& stamp) {
    if (target == source) {
      return Eigen::Isometry3d::Identity();
    }
    const auto tf = tf_buffer_->lookupTransform(
      target, source, stamp, rclcpp::Duration::from_seconds(tf_timeout_));
    return transformMatrix(tf.transform);
  }

  void publishTracking() {
    relocalization_processing::TrackingResult result;
    try {
      // Latest common TF time, never wall-clock restamping or future extrapolation.
      const auto odometry = tf_buffer_->lookupTransform(
        odom_frame_, sensor_frame_, tf2::TimePointZero);
      const auto pose = transformMatrix(odometry.transform);
      const auto stamp = rclcpp::Time(odometry.header.stamp, get_clock()->get_clock_type());
      std::lock_guard<std::mutex> lock(tracking_mutex_);
      result = tracking_->updateOdometry(pose, stamp.nanoseconds(), steadyNowNs());
    } catch (const std::exception&) {
      std::lock_guard<std::mutex> lock(tracking_mutex_);
      result = tracking_->unavailable(steadyNowNs());
    }
    relocalization::msg::TrackingStatus status;
    status.header.frame_id = map_frame_;
    status.header.stamp = rclcpp::Time(result.stamp_ns, get_clock()->get_clock_type());
    status.odom_frame_id = odom_frame_;
    status.sensor_frame_id = sensor_frame_;
    status.valid = result.valid;
    status.status = result.status;
    status.fix_stamp = rclcpp::Time(result.fix_stamp_ns, get_clock()->get_clock_type());
    status.fix_age = result.fix_age;
    if (result.publish) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = status.header;
      pose.pose = poseMessage(result.map_from_sensor);
      tracked_pose_pub_->publish(pose);
      if (tf_broadcaster_) {
        geometry_msgs::msg::TransformStamped transform;
        transform.header = status.header;
        transform.child_frame_id = odom_frame_;
        const auto correction = poseMessage(result.map_from_odom);
        transform.transform.translation.x = correction.position.x;
        transform.transform.translation.y = correction.position.y;
        transform.transform.translation.z = correction.position.z;
        transform.transform.rotation = correction.orientation;
        tf_broadcaster_->sendTransform(transform);
      }
    }
    tracking_status_pub_->publish(status);
  }

  void publishResult(const relocalization_processing::QueryResult& result) {
    relocalization::msg::CandidateArray message;
    message.header.stamp = rclcpp::Time(result.stamp_ns, get_clock()->get_clock_type());
    message.header.frame_id = map_frame_;
    message.odom_frame_id = odom_frame_;
    message.query_frame_id = sensor_frame_;
    message.odom_from_query = poseMessage(result.odom_from_query);
    message.accumulated_frames = result.frame_count;
    message.query_point_count = result.query_cloud.size();
    message.binary_count = result.binary_count;
    message.triangle_count = result.triangle_count;
    message.failure_reason = result.failure_reason;
    message.best_votes = result.diagnostics.best_votes;
    message.best_inliers = result.diagnostics.best_inliers;
    message.required_votes = result.diagnostics.required_votes;
    message.required_inliers = result.diagnostics.required_inliers;
    message.best_length_votes = result.diagnostics.best_length_votes;
    message.status = result.triangle_count == 0 ? "no_descriptors" :
      (result.candidates.empty() ? "no_match" : "candidates");
    geometry_msgs::msg::PoseArray poses;
    poses.header = message.header;
    for (const auto& candidate : result.candidates) {
      relocalization::msg::Candidate output;
      output.submap_id = candidate.submap_id;
      output.votes = candidate.votes;
      output.inliers = candidate.inliers;
      output.mean_binary_similarity = candidate.mean_binary_similarity;
      output.vertex_rmse = candidate.vertex_rmse;
      output.submap_from_query = poseMessage(candidate.submap_from_query);
      output.map_from_query = poseMessage(candidate.map_from_query);
      poses.poses.push_back(output.map_from_query);
      message.candidates.push_back(output);
    }
    candidates_pub_->publish(message);
    poses_pub_->publish(poses);  // 无匹配时也发空数组，避免残留上一帧的候选显示。

    map_processing::Cloud odom_cloud;
    pcl::transformPointCloud(result.query_cloud, odom_cloud, result.odom_from_query.matrix());
    CloudMsg query_message;
    pcl::toROSMsg(odom_cloud, query_message);
    query_message.header.stamp = message.header.stamp;
    query_message.header.frame_id = odom_frame_;
    query_pub_->publish(query_message);
    RCLCPP_INFO(get_logger(), "BTC query: points=%zu, binary=%zu, triangles=%zu, candidates=%zu "
      "(%s), length_votes=%zu, votes=%zu/%zu, inliers=%zu/%zu, reason=%s, axes=%s",
      result.query_cloud.size(), result.binary_count, result.triangle_count,
      result.candidates.size(), message.status.c_str(), result.diagnostics.best_length_votes,
      result.diagnostics.best_votes,
      result.diagnostics.required_votes, result.diagnostics.best_inliers,
      result.diagnostics.required_inliers, result.failure_reason.c_str(),
      result.used_orientation_retry ? "odom_retry" : "sensor");
  }

  void refineAndPublish(const relocalization_processing::QueryResult& query,
                        std::int64_t query_started_steady_ns, std::uint64_t query_generation) {
    relocalization_processing::FineResult result;
    if (!fine_enabled_) {
      result.failure_reason = "disabled";
    } else {
      try {
        result = fine_registration_->refine(query);
      } catch (const std::exception& error) {
        result.failure_reason = "refinement_error";
        RCLCPP_ERROR(get_logger(), "ICP refinement failed: %s", error.what());
      }
    }
    relocalization::msg::FineResult message;
    message.header.stamp = rclcpp::Time(query.stamp_ns, get_clock()->get_clock_type());
    message.header.frame_id = map_frame_;
    message.odom_frame_id = odom_frame_;
    message.query_frame_id = sensor_frame_;
    message.success = result.success;
    message.converged = result.converged;
    message.failure_reason = result.failure_reason;
    message.submap_id = result.submap_id;
    message.query_points = result.query_points;
    message.target_points = result.target_points;
    message.inliers = result.inliers;
    message.overlap = result.overlap;
    message.rmse = result.rmse;
    message.translation_correction = result.translation_correction;
    message.rotation_correction = result.rotation_correction;
    message.map_from_query = poseMessage(result.map_from_query);
    const auto map_from_odom_pose = poseMessage(result.map_from_odom);
    message.map_from_odom.translation.x = map_from_odom_pose.position.x;
    message.map_from_odom.translation.y = map_from_odom_pose.position.y;
    message.map_from_odom.translation.z = map_from_odom_pose.position.z;
    message.map_from_odom.rotation = map_from_odom_pose.orientation;
    map_processing::Cloud aligned;
    if (result.success) {
      {
        std::lock_guard<std::mutex> lock(tracking_mutex_);
        tracking_->updateFix(result.map_from_odom, query.stamp_ns,
                             query_started_steady_ns, steadyNowNs(), query_generation);
      }
      geometry_msgs::msg::PoseStamped pose;
      pose.header = message.header;
      pose.pose = message.map_from_query;
      refined_pose_pub_->publish(pose);
      pcl::transformPointCloud(query.query_cloud, aligned, result.map_from_query.matrix());
    }
    CloudMsg aligned_message;
    pcl::toROSMsg(aligned, aligned_message);
    aligned_message.header = message.header;
    aligned_pub_->publish(aligned_message);  // Empty on failure: clear the previous aligned cloud.
    fine_pub_->publish(message);
    if (fine_enabled_) {
      RCLCPP_INFO(get_logger(), "ICP refine: success=%s, submap=%lu, inliers=%zu/%zu, "
        "overlap=%.3f, rmse=%.3f m, correction=%.3f m/%.2f deg, reason=%s",
        result.success ? "true" : "false", static_cast<unsigned long>(result.submap_id),
        result.inliers, result.query_points, result.overlap, result.rmse,
        result.translation_correction, result.rotation_correction * 180.0 / M_PI,
        result.failure_reason.empty() ? "accepted" : result.failure_reason.c_str());
    }
  }

  void cloudCallback(const CloudMsg::ConstSharedPtr msg) {
    if (msg->header.frame_id.empty() || msg->header.stamp.sec < 0 ||
        msg->header.stamp.nanosec >= 1000000000U ||
        (msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0) ||
        msg->width == 0 || msg->height == 0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "Skip empty cloud/frame or invalid stamp");
      return;
    }
    const rclcpp::Time stamp(msg->header.stamp, get_clock()->get_clock_type());
    for (const auto* name : {"x", "y", "z"}) {
      const auto field = std::find_if(msg->fields.begin(), msg->fields.end(),
        [name](const auto& value) { return value.name == name; });
      if (field == msg->fields.end() || field->datatype != sensor_msgs::msg::PointField::FLOAT32 ||
          field->count != 1 || static_cast<std::uint64_t>(field->offset) + sizeof(float) > msg->point_step) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "Cloud needs FLOAT32 x/y/z fields");
        return;
      }
    }
    if (msg->is_bigendian || static_cast<std::uint64_t>(msg->width) * msg->point_step > msg->row_step ||
        static_cast<std::uint64_t>(msg->height) * msg->row_step > msg->data.size()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "Skip unsupported/malformed cloud layout");
      return;
    }
    const auto query_started_steady_ns = steadyNowNs();
    std::uint64_t query_generation;
    {
      std::lock_guard<std::mutex> lock(tracking_mutex_);
      query_generation = tracking_->generation();
    }
    if (query_generation != processing_generation_) {
      processor_->reset();
      last_stamp_ns_ = 0;
      processing_generation_ = query_generation;
    }
    if (stamp.nanoseconds() <= last_stamp_ns_) {
      return;
    }
    try {
      const auto odom_from_sensor = lookupTransform(odom_frame_, sensor_frame_, stamp);
      const auto sensor_from_cloud = msg->header.frame_id == odom_frame_ ?
        odom_from_sensor.inverse() : lookupTransform(sensor_frame_, msg->header.frame_id, stamp);
      map_processing::Cloud input, sensor_cloud;
      pcl::fromROSMsg(*msg, input);
      pcl::transformPointCloud(input, sensor_cloud, sensor_from_cloud.matrix());
      const auto result = processor_->process(sensor_cloud, odom_from_sensor, stamp.nanoseconds());
      last_stamp_ns_ = stamp.nanoseconds();
      if (result) {
        publishResult(*result);
        refineAndPublish(*result, query_started_steady_ns, query_generation);
      }
    } catch (const tf2::TransformException& error) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "Skip cloud: timestamped TF unavailable: %s",
        error.what());
    } catch (const std::exception& error) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 3000, "BTC query failed: %s", error.what());
    }
  }

  std::string odom_frame_, sensor_frame_, map_frame_;
  double tf_timeout_{};
  bool fine_enabled_{true};
  std::int64_t last_stamp_ns_{0};
  std::uint64_t processing_generation_{0};
  std::mutex tracking_mutex_;
  std::unique_ptr<relocalization_processing::PoseTracking> tracking_;
  std::unique_ptr<relocalization_processing::RelocalizationProcessor> processor_;
  std::unique_ptr<relocalization_processing::FineRegistration> fine_registration_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Subscription<CloudMsg>::SharedPtr cloud_sub_;
  rclcpp::Publisher<CloudMsg>::SharedPtr query_pub_;
  rclcpp::Publisher<CloudMsg>::SharedPtr global_map_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr poses_pub_;
  rclcpp::Publisher<relocalization::msg::CandidateArray>::SharedPtr candidates_pub_;
  rclcpp::Publisher<relocalization::msg::FineResult>::SharedPtr fine_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr refined_pose_pub_;
  rclcpp::Publisher<CloudMsg>::SharedPtr aligned_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr tracked_pose_pub_;
  rclcpp::Publisher<relocalization::msg::TrackingStatus>::SharedPtr tracking_status_pub_;
  rclcpp::CallbackGroup::SharedPtr processing_group_, tracking_group_;
  rclcpp::TimerBase::SharedPtr tracking_timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  int status = 0;
  try {
    auto node = std::make_shared<RelocalizationNode>();
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
    executor.add_node(node);
    executor.spin();
  } catch (const std::exception& error) {
    RCLCPP_FATAL(rclcpp::get_logger("btc_relocalization"), "%s", error.what());
    status = 1;
  }
  rclcpp::shutdown();
  return status;
}
