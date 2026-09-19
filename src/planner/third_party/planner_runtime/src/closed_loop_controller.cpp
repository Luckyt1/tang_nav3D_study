#include "planner/planner.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Eigen>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <planner/msg/bspline.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.hpp>

#include "bspline_opt/uniform_bspline.h"

namespace scan_planner
{
class ClosedLoopController : public rclcpp::Node
{
public:
  explicit ClosedLoopController(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("closed_loop_controller", options)
  {
    time_forward_ = declare_parameter<double>("time_forward", 0.8);
    heading_error_threshold_ = declare_parameter<double>("heading_error_threshold", 0.8);
    kp_pos_ = declare_parameter<double>("kp_pos", 0.8);
    kp_yaw_ = declare_parameter<double>("kp_yaw", 1.5);
    max_vx_ = declare_parameter<double>("max_vx", 0.75);
    max_vy_ = std::clamp(declare_parameter<double>("max_vy", 0.4), 0.0, 0.4);
    max_vyaw_ = std::min(declare_parameter<double>("max_vyaw", 1.0), kMaxVYawLimit);
    finish_dist_ = declare_parameter<double>("finish_dist", 0.15);
    odom_timeout_ = positiveTimeoutParameter("odom_timeout", declare_parameter<double>("odom_timeout", 0.5));
    require_navigation_enable_ = declare_parameter<bool>("require_navigation_enable", false);
    navigation_timeout_ = positiveTimeoutParameter(
        "navigation_timeout", declare_parameter<double>("navigation_timeout", 0.5));

    bspline_sub_ = create_subscription<planner::msg::Bspline>(
        "planning/bspline", 10,
        std::bind(&ClosedLoopController::bsplineCallback, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "body_pose", rclcpp::SensorDataQoS(),
        std::bind(&ClosedLoopController::odomCallback, this, std::placeholders::_1));
    if (require_navigation_enable_)
    {
      navigation_enable_sub_ = create_subscription<std_msgs::msg::Bool>(
          "navigation/enabled", rclcpp::SensorDataQoS(),
          std::bind(&ClosedLoopController::navigationEnableCallback, this, std::placeholders::_1));
    }
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 20);
    execution_frozen_pub_ = create_publisher<std_msgs::msg::Bool>("planning/go2_execution_frozen", 10);
    cmd_timer_ = create_wall_timer(std::chrono::milliseconds(10),
                                   std::bind(&ClosedLoopController::cmdCallback, this));
    last_update_time_ = now();
    RCLCPP_INFO(get_logger(), "Closed-loop controller ready");
  }

private:
  static constexpr double kMaxVYawLimit = 1.0;
  using SteadyClock = std::chrono::steady_clock;

  static double normalizeAngle(double angle)
  {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
  }

  static Eigen::Vector2d clampNorm(const Eigen::Vector2d &value, double max_norm)
  {
    const double norm = value.norm();
    return (norm <= max_norm || norm < 1e-6) ? value : value / norm * max_norm;
  }

  static bool validQuaternion(const geometry_msgs::msg::Quaternion &q)
  {
    const double norm2 = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
    return std::isfinite(q.x) && std::isfinite(q.y) && std::isfinite(q.z) && std::isfinite(q.w) &&
           std::isfinite(norm2) && norm2 > 1e-6;
  }

  static double positiveTimeoutParameter(const char *name, double value)
  {
    if (!std::isfinite(value) || value <= 0.0)
      throw std::invalid_argument(std::string(name) + " must be finite and > 0");
    return value;
  }

  bool isOdomFresh(const SteadyClock::time_point &steady_now) const
  {
    return have_odom_ &&
           std::chrono::duration<double>(steady_now - last_odom_receive_time_).count() <= odom_timeout_;
  }

  bool isNavigationEnabledFresh(const SteadyClock::time_point &steady_now) const
  {
    if (!require_navigation_enable_)
      return true;
    return navigation_enabled_ &&
           std::chrono::duration<double>(steady_now - last_navigation_enable_time_).count() <=
               navigation_timeout_;
  }

  bool navigationHeartbeatStale(const SteadyClock::time_point &steady_now) const
  {
    return navigation_enabled_ &&
           std::chrono::duration<double>(steady_now - last_navigation_enable_time_).count() >
               navigation_timeout_;
  }

  void clearTrajectory()
  {
    receive_traj_ = false;
    traj_.clear();
    traj_duration_ = 0.0;
    exec_time_ = 0.0;
  }

  void stopAndClearTrajectory(bool clear_odom)
  {
    clearTrajectory();
    if (clear_odom)
      have_odom_ = false;
    publishExecutionFrozen(false);
    publishStop();
  }

  void resetOdomEpoch()
  {
    have_odom_ = false;
    last_odom_stamp_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    last_odom_receive_time_ = SteadyClock::time_point::min();
  }

  double estimateDesiredYaw(double t_cur, const Eigen::Vector3d &pos_des) const
  {
    const double t_look = std::min(traj_duration_, t_cur + time_forward_);
    Eigen::Vector3d direction = traj_[0].evaluateDeBoorT(t_look) - pos_des;
    if (direction.head<2>().squaredNorm() < 1e-4)
      direction = traj_[1].evaluateDeBoorT(t_cur);
    return direction.head<2>().squaredNorm() < 1e-4
        ? odom_yaw_ : std::atan2(direction.y(), direction.x());
  }

  void publishStop(double yaw_rate = 0.0)
  {
    geometry_msgs::msg::Twist cmd;
    cmd.angular.z = std::clamp(yaw_rate, -max_vyaw_, max_vyaw_);
    cmd_vel_pub_->publish(cmd);
  }

  void publishExecutionFrozen(bool frozen)
  {
    std_msgs::msg::Bool msg;
    msg.data = frozen;
    execution_frozen_pub_->publish(msg);
  }

  void bsplineCallback(const planner::msg::Bspline::ConstSharedPtr msg)
  {
    if (!isNavigationEnabledFresh(SteadyClock::now()))
    {
      stopAndClearTrajectory(false);
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                           "Ignoring trajectory while navigation is disabled or stale");
      return;
    }
    if (msg->pos_pts.empty() || msg->knots.empty() || msg->order <= 0)
    {
      RCLCPP_WARN(get_logger(), "Ignoring invalid B-spline");
      return;
    }
    Eigen::MatrixXd points(3, msg->pos_pts.size());
    for (size_t i = 0; i < msg->pos_pts.size(); ++i)
      points.col(i) << msg->pos_pts[i].x, msg->pos_pts[i].y, msg->pos_pts[i].z;
    Eigen::VectorXd knots(msg->knots.size());
    for (size_t i = 0; i < msg->knots.size(); ++i) knots(i) = msg->knots[i];
    UniformBspline position(points, msg->order, 0.1);
    position.setKnot(knots);
    traj_ = {position, position.getDerivative()};
    traj_.push_back(traj_[1].getDerivative());
    traj_duration_ = traj_[0].getTimeSum();
    traj_id_ = msg->traj_id;
    exec_time_ = 0.0;
    last_update_time_ = now();
    receive_traj_ = true;
    RCLCPP_INFO(get_logger(), "Received trajectory %lld, duration %.3fs",
                static_cast<long long>(traj_id_), traj_duration_);
  }

  void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
  {
    const rclcpp::Time stamp(msg->header.stamp, get_clock()->get_clock_type());
    if (stamp.nanoseconds() <= 0 ||
        (last_odom_stamp_.nanoseconds() > 0 && stamp <= last_odom_stamp_))
      return;
    const auto &position = msg->pose.pose.position;
    if (!std::isfinite(position.x) || !std::isfinite(position.y) || !std::isfinite(position.z) ||
        !validQuaternion(msg->pose.pose.orientation))
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Ignoring invalid odometry");
      return;
    }
    odom_pos_ << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;
    odom_yaw_ = tf2::getYaw(msg->pose.pose.orientation);
    have_odom_ = true;
    last_odom_stamp_ = stamp;
    last_odom_receive_time_ = SteadyClock::now();
  }

  void navigationEnableCallback(const std_msgs::msg::Bool::ConstSharedPtr msg)
  {
    const auto steady_now = SteadyClock::now();
    const bool was_recovering =
        !navigation_enabled_ || navigationHeartbeatStale(steady_now);
    navigation_enabled_ = msg->data;
    if (navigation_enabled_)
    {
      if (was_recovering)
      {
        clearTrajectory();
        resetOdomEpoch();
      }
      last_navigation_enable_time_ = steady_now;
    }
    else
    {
      stopAndClearTrajectory(true);
    }
  }

  void cmdCallback()
  {
    const auto steady_now = SteadyClock::now();
    if (!isNavigationEnabledFresh(steady_now))
    {
      stopAndClearTrajectory(false);
      return;
    }
    if (!isOdomFresh(steady_now))
    {
      stopAndClearTrajectory(true);
      return;
    }
    if (!receive_traj_)
    {
      publishExecutionFrozen(false);
      publishStop();
      return;
    }
    const auto current_time = now();
    double dt = (current_time - last_update_time_).seconds();
    if (dt < 0.0 || dt > 0.2) dt = 0.0;
    const double t_eval = std::min(exec_time_, traj_duration_);
    Eigen::Vector3d pos_des = traj_[0].evaluateDeBoorT(t_eval);
    const double yaw_error = normalizeAngle(estimateDesiredYaw(t_eval, pos_des) - odom_yaw_);
    const double yaw_command = std::clamp(kp_yaw_ * yaw_error, -max_vyaw_, max_vyaw_);
    if (std::abs(yaw_error) > heading_error_threshold_)
    {
      publishExecutionFrozen(true);
      publishStop(yaw_command);
      last_update_time_ = current_time;
      return;
    }

    publishExecutionFrozen(false);
    exec_time_ = std::min(traj_duration_, exec_time_ + dt);
    last_update_time_ = current_time;
    pos_des = traj_[0].evaluateDeBoorT(exec_time_);
    const Eigen::Vector3d vel_des = traj_[1].evaluateDeBoorT(exec_time_);
    const Eigen::Vector2d pos_error(pos_des.x() - odom_pos_.x(), pos_des.y() - odom_pos_.y());
    const Eigen::Vector2d vel_world = clampNorm(
        Eigen::Vector2d(vel_des.x(), vel_des.y()) + kp_pos_ * pos_error,
        std::max(max_vx_, max_vy_));
    const double c = std::cos(odom_yaw_);
    const double s = std::sin(odom_yaw_);
    geometry_msgs::msg::Twist command;
    // Keep forward motion nonnegative; allow bounded lateral correction in
    // the body frame while the heading controller follows the path tangent.
    command.linear.x = std::clamp(c * vel_world.x() + s * vel_world.y(), 0.0, max_vx_);
    command.linear.y = std::clamp(-s * vel_world.x() + c * vel_world.y(), -max_vy_, max_vy_);
    command.angular.z = yaw_command;
    if (exec_time_ >= traj_duration_ && pos_error.norm() < finish_dist_)
      command = geometry_msgs::msg::Twist();
    cmd_vel_pub_->publish(command);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr execution_frozen_pub_;
  rclcpp::Subscription<planner::msg::Bspline>::SharedPtr bspline_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr navigation_enable_sub_;
  rclcpp::TimerBase::SharedPtr cmd_timer_;
  bool receive_traj_{false};
  bool have_odom_{false};
  bool require_navigation_enable_{false};
  bool navigation_enabled_{false};
  std::vector<UniformBspline> traj_;
  double traj_duration_{0.0};
  std::int64_t traj_id_{0};
  Eigen::Vector3d odom_pos_{Eigen::Vector3d::Zero()};
  double odom_yaw_{0.0};
  double exec_time_{0.0};
  rclcpp::Time last_update_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_odom_stamp_{0, 0, RCL_ROS_TIME};
  SteadyClock::time_point last_odom_receive_time_{SteadyClock::time_point::min()};
  SteadyClock::time_point last_navigation_enable_time_{SteadyClock::time_point::min()};
  double time_forward_, heading_error_threshold_, kp_pos_, kp_yaw_;
  double max_vx_, max_vy_, max_vyaw_, finish_dist_;
  double odom_timeout_, navigation_timeout_;
};
}  // namespace scan_planner

std::shared_ptr<rclcpp::Node> planner::createControllerNode()
{
  return std::make_shared<scan_planner::ClosedLoopController>();
}
