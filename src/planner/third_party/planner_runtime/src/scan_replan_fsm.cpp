
#include <plan_manage/scan_replan_fsm.h>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace
{
  template <typename T>
  T load_parameter(rclcpp::Node *node, const std::string &name, const T &default_value)
  {
    if (!node->has_parameter(name)) node->declare_parameter<T>(name, default_value);
    return node->get_parameter(name).get_value<T>();
  }

  bool validQuaternion(const geometry_msgs::msg::Quaternion &q)
  {
    const double norm2 = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
    return std::isfinite(q.x) && std::isfinite(q.y) && std::isfinite(q.z) && std::isfinite(q.w) &&
           std::isfinite(norm2) && norm2 > 1e-6;
  }
} // namespace

namespace scan_planner
{

  void SCANReplanFSM::init(rclcpp::Node *node)
  {
    node_ = node;
    current_wp_ = 0;
    exec_state_ = FSM_EXEC_STATE::INIT;
    trigger_ = false;
    have_target_ = false;
    have_odom_ = false;
    have_new_target_ = false;
    rviz_height_ready_ = false;
    go2_execution_frozen_ = false;
    flag_escape_emergency_ = true;
    need_hover_stop_ = false;
    replan_fail_count_ = 0;
    last_freeze_update_time_ = node_->now();
    last_global_replan_request_time_ = rclcpp::Time(0, 0, node_->get_clock()->get_clock_type());
    last_replan_finish_time_ = node_->now();

    /*  fsm param  */
    navi_mode_ = load_parameter<int>(node_, "fsm.navi_mode", -1);
    replan_thresh_ = load_parameter<double>(node_, "fsm.thresh_replan", -1.0);
    no_replan_thresh_ = load_parameter<double>(node_, "fsm.thresh_no_replan", -1.0);
    planning_horizon_ = load_parameter<double>(node_, "fsm.planning_horizon", -1.0);
    waypoint_pass_through_speed_scale_ = load_parameter<double>(
        node_, "fsm.waypoint_pass_through_speed_scale", 0.7);
    emergency_time_ = load_parameter<double>(node_, "fsm.emergency_time", 1.0);
    enable_fail_safe_ = load_parameter<bool>(node_, "fsm.fail_safe", true);
    max_replan_fail_count_ = load_parameter<int>(node_, "fsm.max_replan_fail_count", 8);
    exec_interval_ms_ = load_parameter<int>(node_, "fsm.exec_interval_ms", 10);
    safety_interval_ms_ = load_parameter<int>(node_, "fsm.safety_interval_ms", 100);
    min_waypoint_window_size_ = load_parameter<int>(node_, "fsm.min_waypoint_window_size", 4);
    collision_check_dt_ = load_parameter<double>(node_, "fsm.collision_check_dt", 0.05);
    collision_check_horizon_ = load_parameter<double>(node_, "fsm.collision_check_horizon", 2.0);
    periodic_replan_interval_ = load_parameter<double>(node_, "fsm.periodic_replan_interval", 0.35);
    periodic_replan_min_remaining_ = load_parameter<double>(node_, "fsm.periodic_replan_min_remaining", 0.20);
    enable_completion_driven_replan_ = load_parameter<bool>(
        node_, "fsm.enable_completion_driven_replan", true);
    completion_replan_min_interval_ = load_parameter<double>(
        node_, "fsm.completion_replan_min_interval", 0.05);
    replan_stitch_lookahead_ = load_parameter<double>(
        node_, "fsm.replan_stitch_lookahead", 0.0);
    replan_stitch_max_tracking_error_ = load_parameter<double>(
        node_, "fsm.replan_stitch_max_tracking_error", 0.20);
    keep_previous_traj_min_remaining_ = load_parameter<double>(
        node_, "fsm.keep_previous_traj_min_remaining", 0.50);
    enable_target_snap_ = load_parameter<bool>(node_, "fsm.enable_target_snap", true);
    target_snap_radius_ = load_parameter<double>(node_, "fsm.target_snap_radius", 0.45);
    target_snap_z_radius_ = load_parameter<double>(node_, "fsm.target_snap_z_radius", 0.35);
    enable_global_replan_on_local_failure_ = load_parameter<bool>(
        node_, "fsm.enable_global_replan_on_local_failure", true);
    global_replan_request_cooldown_ = load_parameter<double>(
        node_, "fsm.global_replan_request_cooldown", 1.0);
    global_replan_goal_z_offset_ = load_parameter<double>(
        node_, "fsm.global_replan_goal_z_offset", 0.90);
    global_replan_goal_topic_ = load_parameter<std::string>(
        node_, "fsm.global_replan_goal_topic", "/move_base_simple/goal");
    self_inflation_z_up_ = load_parameter<double>(node_, "grid_map.obstacles_inflation_z_up", 0.0);
    self_inflation_z_down_ = load_parameter<double>(node_, "grid_map.obstacles_inflation_z_down", 0.0);
    self_double_cylinder_radius_ = load_parameter<double>(node_, "grid_map.double_cylinder_radius", 0.0);
    self_double_cylinder_offset_ = load_parameter<double>(node_, "grid_map.double_cylinder_offset", 0.0);
    body_height_ = load_parameter<double>(node_, "grid_map.body_height", 0.4);
    navigation_z_ = load_parameter<double>(node_, "fsm.navigation_z", 0.35);
    local_path_z_offset_ = load_parameter<double>(node_, "fsm.local_path_z_offset", 0.0);
    use_path_z_ = load_parameter<bool>(node_, "fsm.use_path_z", false);
    use_odom_z_ = load_parameter<bool>(node_, "fsm.use_odom_z", false);
    self_inflation_frame_id_ = load_parameter<std::string>(node_, "grid_map.frame_id", "world");

    if (navi_mode_ == NAVI_MODE::PRESET_TARGET)
    {
      const auto flat_waypoints = load_parameter<std::vector<double>>(node_, "fsm.waypoints", {});
      if (flat_waypoints.empty() || flat_waypoints.size() % 3 != 0)
        throw std::runtime_error("navi_mode=2 requires non-empty fsm.waypoints with x,y,z triples");
      waypoint_num_ = static_cast<int>(flat_waypoints.size() / 3);
      preset_waypoints_.resize(waypoint_num_);
      for (int i = 0; i < waypoint_num_; i++)
      {
        preset_waypoints_[i] = Eigen::Vector3d(flat_waypoints[3 * i], flat_waypoints[3 * i + 1],
                                               flat_waypoints[3 * i + 2]);
      }
    }

    /* initialize main modules */
    visualization_.reset(new PlanningVisualization(node_));
    planner_manager_.reset(new SCANPlannerManager);
    planner_manager_->initPlanModules(node_, visualization_);

    /* callback */
    exec_timer_ = node_->create_wall_timer(std::chrono::milliseconds(std::max(10, exec_interval_ms_)),
                                           std::bind(&SCANReplanFSM::execFSMCallback, this));
    safety_timer_ = node_->create_wall_timer(std::chrono::milliseconds(std::max(50, safety_interval_ms_)),
                                             std::bind(&SCANReplanFSM::checkCollisionCallback, this));
    odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        "body_pose", rclcpp::SensorDataQoS(),
        std::bind(&SCANReplanFSM::odometryCallback, this, std::placeholders::_1));
    go2_execution_frozen_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
        "planning/go2_execution_frozen", 10,
        std::bind(&SCANReplanFSM::go2ExecutionFrozenCallback, this, std::placeholders::_1));

    bspline_pub_ = node_->create_publisher<planner::msg::Bspline>("planning/bspline", 10);
    data_disp_pub_ = node_->create_publisher<planner::msg::DataDisp>("planning/data_display", 100);
    self_inflation_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>(
        "self_inflation", rclcpp::QoS(1).reliable().transient_local());
    global_replan_goal_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
        global_replan_goal_topic_, 10);

    if (navi_mode_ == NAVI_MODE::MANUAL_TARGET)
      goal_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
          "move_base_simple/goal", 1,
          std::bind(&SCANReplanFSM::rvizGoalCallback, this, std::placeholders::_1));
    else if (navi_mode_ == NAVI_MODE::REFERENCE_PATH)
      path_sub_ = node_->create_subscription<nav_msgs::msg::Path>(
          "initial_path", rclcpp::QoS(1).reliable().transient_local(),
          std::bind(&SCANReplanFSM::pathCallback, this, std::placeholders::_1));
    else if (navi_mode_ == NAVI_MODE::PRESET_TARGET)
      RCLCPP_INFO(node_->get_logger(), "Preset waypoint mode will start after the first odometry message");
    else
      throw std::runtime_error("fsm.navi_mode must be 1, 2, or 3");
  }

  void SCANReplanFSM::planGlobalTrajbyGivenWps()
  {
    std::vector<Eigen::Vector3d> wps = preset_waypoints_;

    for (size_t i = 0; i < wps.size(); i++)
    {
      visualization_->displayGoalPoint(wps[i], Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, i);
    }

    active_waypoints_ = wps;
    current_wp_ = 0;
    trigger_ = true;
    init_pt_ = odom_pos_;

    if (planNextWaypoint())
    {
      changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
    }
    else
    {
      RCLCPP_ERROR(node_->get_logger(), "Unable to generate global trajectory to first preset waypoint");
    }
  }

  void SCANReplanFSM::rvizGoalCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr &msg)
  {
    if (!msg)
      return;

    if (!rviz_height_ready_)
    {
      RCLCPP_WARN(node_->get_logger(), "Ignore RViz goal before receiving initial body pose");
      return;
    }

    auto path = std::make_shared<nav_msgs::msg::Path>();
    path->header = msg->header;
    path->poses.push_back(*msg);
    waypointCallback(path);
  }

  void SCANReplanFSM::waypointCallback(const nav_msgs::msg::Path::ConstSharedPtr &msg)
  {
    if (!msg || msg->poses.empty())
    {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                           "Empty waypoint message; ignoring");
      return;
    }

    if (msg->poses[0].pose.position.z < -0.1)
      return;

    cout << "Triggered!" << endl;
    trigger_ = true;
    init_pt_ = odom_pos_;

    bool success = false;
    end_pt_ << msg->poses[0].pose.position.x, msg->poses[0].pose.position.y, rviz_goal_height_;
    success = planner_manager_->planGlobalTraj(odom_pos_, odom_vel_, Eigen::Vector3d::Zero(), end_pt_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    if (success)
      success = adjustGlobalTargetIfOccupied();

    visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, 0);

    if (success)
    {

      /*** display ***/
      constexpr double step_size_t = 0.1;
      int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t);
      vector<Eigen::Vector3d> gloabl_traj(i_end);
      for (int i = 0; i < i_end; i++)
      {
        gloabl_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);
      }

      end_vel_.setZero();
      have_target_ = true;
      have_new_target_ = true;

      /*** FSM ***/
      if (exec_state_ == WAIT_TARGET)
        changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
      else if (exec_state_ == EXEC_TRAJ)
        changeFSMExecState(REPLAN_TRAJ, "TRIG");

      // visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(1, 0, 0, 1), 0.3, 0);
      visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
    }
    else
    {
      RCLCPP_ERROR(node_->get_logger(), "Unable to generate global trajectory");
    }
  }

  bool SCANReplanFSM::planGlobalTrajByWaypoints(const std::vector<Eigen::Vector3d> &waypoints)
  {
    if (waypoints.empty())
    {
      RCLCPP_WARN(node_->get_logger(), "No waypoint supplied for global trajectory");
      return false;
    }

    end_pt_ = waypoints.back();

    for (size_t i = 0; i < waypoints.size(); i++)
    {
      visualization_->displayGoalPoint(waypoints[i], Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, i);
    }

    bool success = planner_manager_->planGlobalTrajWaypoints(
        odom_pos_,
        odom_vel_,
        Eigen::Vector3d::Zero(),
        waypoints,
        Eigen::Vector3d::Zero(),
        Eigen::Vector3d::Zero());

    if (!success)
    {
      RCLCPP_ERROR(node_->get_logger(), "Unable to generate global trajectory from waypoints");
      return false;
    }

    if (!adjustGlobalTargetIfOccupied())
      return false;

    constexpr double step_size_t = 0.1;
    int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t);
    std::vector<Eigen::Vector3d> gloabl_traj(i_end);
    for (int i = 0; i < i_end; i++)
    {
      gloabl_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);
    }

    end_vel_.setZero();
    have_target_ = true;
    have_new_target_ = true;
    visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
    visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, static_cast<int>(waypoints.size()) - 1);

    return true;
  }

  bool SCANReplanFSM::planNextWaypoint()
  {
    advanceWaypointIndexByProgress();

    if (current_wp_ < 0 || current_wp_ >= (int)active_waypoints_.size())
    {
      RCLCPP_WARN(node_->get_logger(), "[navi_mode=%d] No active waypoint to plan", navi_mode_);
      return false;
    }

    end_pt_ = active_waypoints_[current_wp_];
    end_vel_.setZero();
    setStartStateFromOdomOrCurrentTraj();

    if (navi_mode_ == NAVI_MODE::REFERENCE_PATH)
      return planWaypointWindowFromStartState();

    if (current_wp_ + 1 < (int)active_waypoints_.size())
    {
      Eigen::Vector3d pass_dir = active_waypoints_[current_wp_ + 1] - active_waypoints_[current_wp_];
      if (pass_dir.norm() < 1e-3 && current_wp_ > 0)
        pass_dir = active_waypoints_[current_wp_] - active_waypoints_[current_wp_ - 1];
      if (pass_dir.norm() > 1e-3)
      {
        const double pass_speed = std::clamp(
            waypoint_pass_through_speed_scale_, 0.0, 1.0) * planner_manager_->pp_.max_vel_;
        end_vel_ = pass_dir.normalized() * pass_speed;
      }
    }

    bool success = planner_manager_->planGlobalTraj(
        start_pt_,
        start_vel_,
        start_acc_,
        end_pt_,
        end_vel_,
        Eigen::Vector3d::Zero());

    if (!success)
    {
      RCLCPP_ERROR(node_->get_logger(), "[navi_mode=%d] Unable to generate trajectory to waypoint %d",
                   navi_mode_, current_wp_ + 1);
      return false;
    }

    if (!adjustGlobalTargetIfOccupied())
      return false;

    constexpr double step_size_t = 0.1;
    int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t);
    std::vector<Eigen::Vector3d> gloabl_traj(i_end);
    for (int i = 0; i < i_end; i++)
    {
      gloabl_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);
    }

    have_target_ = true;
    have_new_target_ = true;
    visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
    visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, current_wp_);
    RCLCPP_INFO(node_->get_logger(), "[navi_mode=%d] Planning to waypoint %d/%zu: [%.2f, %.2f, %.2f]",
                navi_mode_, current_wp_ + 1, active_waypoints_.size(), end_pt_(0), end_pt_(1), end_pt_(2));

    return true;
  }

  bool SCANReplanFSM::isWaypointSequenceMode() const
  {
    return navi_mode_ == NAVI_MODE::PRESET_TARGET || navi_mode_ == NAVI_MODE::REFERENCE_PATH;
  }

  void SCANReplanFSM::advanceWaypointIndexByProgress()
  {
    if (current_wp_ < 0 || current_wp_ >= (int)active_waypoints_.size())
      return;

    while (current_wp_ + 1 < (int)active_waypoints_.size() &&
           (isWaypointReached(active_waypoints_[current_wp_], 0.55) ||
            waypointDistance(active_waypoints_[current_wp_ + 1]) <
                waypointDistance(active_waypoints_[current_wp_])))
    {
      current_wp_++;
    }
  }

  double SCANReplanFSM::waypointDistance(const Eigen::Vector3d &target) const
  {
    // OctoPlanner lifts path Z for clearance, while the ELF3 base odometry is
    // constrained to the floor plane. Progress along a reference path is
    // therefore planar; including the synthetic Z offset prevents the path's
    // start cell from being skipped even when it coincides with the robot.
    if (navi_mode_ == NAVI_MODE::REFERENCE_PATH)
      return (target.head<2>() - odom_pos_.head<2>()).norm();

    if (use_path_z_ || use_odom_z_)
      return (target - odom_pos_).norm();
    return (target.head<2>() - odom_pos_.head<2>()).norm();
  }

  bool SCANReplanFSM::isWaypointReached(const Eigen::Vector3d &target, double threshold) const
  {
    return waypointDistance(target) < threshold;
  }

  bool SCANReplanFSM::adjustGlobalTargetIfOccupied()
  {
    auto map = planner_manager_->grid_map_;
    auto &global_data = planner_manager_->global_data_;
    const double duration = global_data.global_duration_;
    if (!map || duration < 1e-3)
      return true;

    constexpr double sample_dt = 0.05;
    const int sample_num = std::max(1, static_cast<int>(std::ceil(duration / sample_dt)));
    const Eigen::Vector3d final_pt = global_data.global_traj_.evaluate(duration);
    const Eigen::Vector3d final_prev = global_data.global_traj_.evaluate(duration * (sample_num - 1) / sample_num);
    const int final_occ = map->getInflateOccupancy(final_pt, estimateYawFromSegment(final_prev, final_pt));
    if (final_occ <= 0)
      return true;

    for (int i = sample_num; i >= 0; --i)
    {
      const double t = duration * i / sample_num;
      const double prev_t = duration * std::max(0, i - 1) / sample_num;
      const Eigen::Vector3d pt = global_data.global_traj_.evaluate(t);
      const Eigen::Vector3d prev_pt = global_data.global_traj_.evaluate(prev_t);

      if (map->getInflateOccupancy(pt, estimateYawFromSegment(prev_pt, pt)) == 0)
      {
        const Eigen::Vector3d raw_end = end_pt_;
        end_pt_ = pt;
        global_data.global_duration_ = t;
        global_data.last_progress_time_ = std::min(global_data.last_progress_time_, t);
        RCLCPP_WARN(node_->get_logger(),
                    "Target [%.2f, %.2f, %.2f] is occupied; using [%.2f, %.2f, %.2f]",
                    raw_end(0), raw_end(1), raw_end(2), end_pt_(0), end_pt_(1), end_pt_(2));
        return true;
      }
    }

    RCLCPP_ERROR(node_->get_logger(),
                 "Target is occupied and no collision-free point was found on the global trajectory");
    return false;
  }

  void SCANReplanFSM::cancelCurrentRoute(const char *reason)
  {
    if (have_odom_)
      callEmergencyStop(odom_pos_);
    active_waypoints_.clear();
    current_wp_ = 0;
    have_target_ = have_new_target_ = trigger_ = false;
    need_hover_stop_ = false;
    replan_fail_count_ = 0;
    changeFSMExecState(WAIT_TARGET, reason);
    RCLCPP_WARN(node_->get_logger(), "Global route unavailable; cancelled previous local route (%s)", reason);
  }

  void SCANReplanFSM::pathCallback(const nav_msgs::msg::Path::ConstSharedPtr &msg)
  {
    if (!msg)
      return;
    if (msg->poses.empty())
    {
      cancelCurrentRoute("EMPTY_GLOBAL_PATH");
      return;
    }

    std::vector<Eigen::Vector3d> waypoints;
    waypoints.reserve(msg->poses.size());

    const double reference_z =
        use_odom_z_ && have_odom_ ? odom_pos_(2) : navigation_z_ + local_path_z_offset_;
    if (!have_odom_)
    {
      RCLCPP_WARN(node_->get_logger(),
                  "Reference path received before body_pose; using fallback height %.3f",
                  reference_z);
    }

    for (const auto& pose_stamped : msg->poses)
    {
      Eigen::Vector3d wp;
      wp(0) = pose_stamped.pose.position.x;
      wp(1) = pose_stamped.pose.position.y;
      wp(2) = use_path_z_ ? pose_stamped.pose.position.z + local_path_z_offset_ : reference_z;
      if (!wp.allFinite())
      {
        cancelCurrentRoute("INVALID_GLOBAL_PATH");
        return;
      }
      waypoints.push_back(wp);
    }

    if (waypoints.size() == 1 && have_odom_ &&
        (waypoints.front().head<2>() - odom_pos_.head<2>()).norm() < 0.35)
    {
      Eigen::Vector3d stop_pos = odom_pos_;
      if (!use_odom_z_)
        stop_pos(2) = navigation_z_ + local_path_z_offset_;
      callEmergencyStop(stop_pos);
      active_waypoints_.clear();
      current_wp_ = 0;
      have_target_ = false;
      have_new_target_ = false;
      trigger_ = false;
      need_hover_stop_ = false;
      changeFSMExecState(WAIT_TARGET, "BLOCKED_PATH");
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                           "Reference path is blocked by the global costmap; holding position");
      return;
    }

    active_waypoints_ = waypoints;
    current_wp_ = 0;
    trigger_ = true;
    bool success = planNextWaypoint();

    if (success)
    {
      /*** FSM ***/
      if (exec_state_ == WAIT_TARGET)
      {
        changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
      }
      else if (exec_state_ == EXEC_TRAJ)
      {
        changeFSMExecState(REPLAN_TRAJ, "TRIG");
      }

      RCLCPP_INFO(node_->get_logger(), "Reference path accepted as %zu streaming waypoints",
                  active_waypoints_.size());
    }
    else
    {
      cancelCurrentRoute("UNUSABLE_GLOBAL_PATH");
    }
  }

  void SCANReplanFSM::odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr &msg)
  {
    const auto &position = msg->pose.pose.position;
    const auto &linear = msg->twist.twist.linear;
    if (!std::isfinite(position.x) || !std::isfinite(position.y) || !std::isfinite(position.z) ||
        !std::isfinite(linear.x) || !std::isfinite(linear.y) || !std::isfinite(linear.z) ||
        !validQuaternion(msg->pose.pose.orientation))
    {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                           "Ignoring invalid odometry");
      return;
    }

    odom_pos_(0) = msg->pose.pose.position.x;
    odom_pos_(1) = msg->pose.pose.position.y;
    odom_pos_(2) = (use_odom_z_ ? msg->pose.pose.position.z : navigation_z_) + local_path_z_offset_;

    odom_orient_.w() = msg->pose.pose.orientation.w;
    odom_orient_.x() = msg->pose.pose.orientation.x;
    odom_orient_.y() = msg->pose.pose.orientation.y;
    odom_orient_.z() = msg->pose.pose.orientation.z;
    odom_orient_.normalize();

    if (navi_mode_ == NAVI_MODE::MANUAL_TARGET && !rviz_height_ready_)
    {
      rviz_goal_height_ = odom_pos_(2);
      rviz_height_ready_ = true;
      RCLCPP_INFO(node_->get_logger(), "Set RViz goal height from initial body_pose z: %.3f", rviz_goal_height_);
    }

    const Eigen::Vector3d body_vel(linear.x, linear.y, linear.z);
    odom_vel_ = odom_orient_ * body_vel;

    //odom_acc_ = estimateAcc( msg );

    have_odom_ = true;
    publishSelfInflationMarker();
    if (navi_mode_ == NAVI_MODE::PRESET_TARGET && !preset_started_)
    {
      preset_started_ = true;
      planGlobalTrajbyGivenWps();
    }
  }

  void SCANReplanFSM::go2ExecutionFrozenCallback(const std_msgs::msg::Bool::ConstSharedPtr &msg)
  {
    go2_execution_frozen_ = msg->data;
  }

  void SCANReplanFSM::updateLocalTrajTimeFreeze()
  {
    const rclcpp::Time now = node_->now();
    double dt = (now - last_freeze_update_time_).seconds();
    last_freeze_update_time_ = now;

    if (dt <= 0.0 || dt > 0.2)
      return;

    LocalTrajData *info = &planner_manager_->local_data_;
    if (go2_execution_frozen_ && info->start_time_.seconds() > 1e-5)
      info->start_time_ += rclcpp::Duration::from_seconds(dt);
  }

  double SCANReplanFSM::getOdomYaw() const
  {
    Eigen::Vector3d heading = odom_orient_.toRotationMatrix().col(0);
    if (heading.head<2>().squaredNorm() < 1e-8)
      return 0.0;
    return std::atan2(heading(1), heading(0));
  }

  double SCANReplanFSM::estimateYawFromSegment(const Eigen::Vector3d &from, const Eigen::Vector3d &to) const
  {
    Eigen::Vector2d diff(to(0) - from(0), to(1) - from(1));
    if (diff.squaredNorm() < 1e-8)
      return getOdomYaw();
    return std::atan2(diff(1), diff(0));
  }

  void SCANReplanFSM::publishSelfInflationMarker()
  {
    const double radius = std::max(0.0, self_double_cylinder_radius_);
    const double z_up = std::max(0.0, self_inflation_z_up_);
    const double z_down = std::max(0.0, self_inflation_z_down_);
    const double height = std::max(1e-3, z_up + z_down);

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = self_inflation_frame_id_.empty() ? "world" : self_inflation_frame_id_;
    marker.header.stamp = node_->now();
    marker.ns = "self_inflation";
    marker.type = visualization_msgs::msg::Marker::CYLINDER;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 2.0 * radius;
    marker.scale.y = 2.0 * radius;
    marker.scale.z = height;
    marker.color.r = 0.1;
    marker.color.g = 0.6;
    marker.color.b = 1.0;
    marker.color.a = 0.4;
    marker.lifetime = rclcpp::Duration::from_seconds(0.2);

    Eigen::Vector3d center = odom_pos_;
    center(2) += 0.5 * (z_up - z_down);

    Eigen::Vector3d heading(std::cos(getOdomYaw()), std::sin(getOdomYaw()), 0.0);
    Eigen::Vector3d front = center + self_double_cylinder_offset_ * heading;
    Eigen::Vector3d rear = center - self_double_cylinder_offset_ * heading;

    marker.id = 0;
    marker.pose.position.x = front(0);
    marker.pose.position.y = front(1);
    marker.pose.position.z = front(2);
    self_inflation_pub_->publish(marker);

    marker.id = 1;
    marker.pose.position.x = rear(0);
    marker.pose.position.y = rear(1);
    marker.pose.position.z = rear(2);
    self_inflation_pub_->publish(marker);
  }

  void SCANReplanFSM::changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call)
  {

    if (new_state == exec_state_)
      continuously_called_times_++;
    else
      continuously_called_times_ = 1;

    static string state_str[7] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP"};
    int pre_s = int(exec_state_);
    exec_state_ = new_state;
    cout << "[" + pos_call + "]: from " + state_str[pre_s] + " to " + state_str[int(new_state)] << endl;
  }

  std::pair<int, SCANReplanFSM::FSM_EXEC_STATE> SCANReplanFSM::timesOfConsecutiveStateCalls()
  {
    return std::pair<int, FSM_EXEC_STATE>(continuously_called_times_, exec_state_);
  }

  void SCANReplanFSM::printFSMExecState()
  {
    static string state_str[7] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP"};

    cout << "[FSM]: state: " + state_str[int(exec_state_)] << endl;
  }

  bool SCANReplanFSM::ensureFreshMap()
  {
    if (planner_manager_->grid_map_->hasRecentObservation())
    {
      if (waiting_for_map_ && have_target_ && have_odom_)
        changeFSMExecState(GEN_NEW_TRAJ, "MAP_RECOVERED");
      waiting_for_map_ = false;
      return true;
    }
    if (!waiting_for_map_ && have_odom_)
    {
      callEmergencyStop(odom_pos_);
      RCLCPP_WARN(node_->get_logger(), "Map observations unavailable or stale; holding position");
    }
    waiting_for_map_ = true;
    return false;
  }

  void SCANReplanFSM::execFSMCallback()
  {
    updateLocalTrajTimeFreeze();
    if (!ensureFreshMap())
      return;

    static int fsm_num = 0;
    fsm_num++;
    if (fsm_num == 100)
    {
      printFSMExecState();
      if (!have_odom_)
        cout << "no odom." << endl;
      if (!trigger_)
        cout << "wait for goal." << endl;
      fsm_num = 0;
    }

    switch (exec_state_)
    {
    case INIT:
    {
      if (!have_odom_)
      {
        return;
      }
      if (!trigger_)
      {
        return;
      }
      changeFSMExecState(WAIT_TARGET, "FSM");
      break;
    }

    case WAIT_TARGET:
    {
      if (!have_target_)
        return;
      else
      {
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }

    case GEN_NEW_TRAJ:
    {
      setStartStateFromOdomOrCurrentTraj();

      // Eigen::Vector3d rot_x = odom_orient_.toRotationMatrix().block(0, 0, 3, 1);
      // start_yaw_(0)         = atan2(rot_x(1), rot_x(0));
      // start_yaw_(1) = start_yaw_(2) = 0.0;

      bool flag_random_poly_init;
      if (timesOfConsecutiveStateCalls().first == 1)
        flag_random_poly_init = false;
      else
        flag_random_poly_init = true;

      bool success = callReboundReplan(true, flag_random_poly_init);
      if (success)
      {

        replan_fail_count_ = 0;
        last_replan_finish_time_ = node_->now();
        changeFSMExecState(EXEC_TRAJ, "FSM");
        flag_escape_emergency_ = true;
      }
      else
      {
        replan_fail_count_++;
        if (!recoverFromReplanFailure("GEN_NEW_TRAJ"))
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }

    case REPLAN_TRAJ:
    {

      if (planFromCurrentTraj())
      {
        replan_fail_count_ = 0;
        last_replan_finish_time_ = node_->now();
        changeFSMExecState(EXEC_TRAJ, "FSM");
      }
      else
      {
        replan_fail_count_++;
        if (!recoverFromReplanFailure("REPLAN_TRAJ"))
          changeFSMExecState(REPLAN_TRAJ, "FSM");
      }

      break;
    }

    case EXEC_TRAJ:
    {
      /* determine if need to replan */
      LocalTrajData *info = &planner_manager_->local_data_;
      rclcpp::Time time_now = node_->now();
      double t_cur = (time_now - info->start_time_).seconds();
      t_cur = min(info->duration_, t_cur);

      Eigen::Vector3d pos = info->position_traj_.evaluateDeBoorT(t_cur);

      if (enable_completion_driven_replan_ && runCompletionDrivenReplan())
        return;

      if (isWaypointSequenceMode() &&
          current_wp_ + 1 < (int)active_waypoints_.size() &&
          isWaypointReached(end_pt_, 0.5))
      {
        current_wp_++;
        if (planNextWaypoint())
        {
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
          return;
        }
        replan_fail_count_++;
        if (!recoverFromReplanFailure("WAYPOINT_ADVANCE"))
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
        return;
      }

      if (!enable_completion_driven_replan_ && periodic_replan_interval_ > 1e-3 &&
          ((navi_mode_ == NAVI_MODE::REFERENCE_PATH &&
            t_cur >= std::min(periodic_replan_interval_, std::max(0.05, info->duration_ * 0.35))) ||
           (t_cur >= periodic_replan_interval_ &&
            info->duration_ - t_cur > std::max(0.05, periodic_replan_min_remaining_))))
      {
        changeFSMExecState(REPLAN_TRAJ, "PERIODIC");
        return;
      }

      /* && (end_pt_ - pos).norm() < 0.5 */
      if (t_cur > info->duration_ - 1e-2)
      {
        if (isWaypointSequenceMode() && current_wp_ + 1 < (int)active_waypoints_.size())
        {
          current_wp_++;
          if (planNextWaypoint())
          {
            changeFSMExecState(GEN_NEW_TRAJ, "FSM");
            return;
          }
          replan_fail_count_++;
          if (!recoverFromReplanFailure("WAYPOINT_TIMEOUT"))
            changeFSMExecState(GEN_NEW_TRAJ, "FSM");
          return;
        }

        if (isWaypointSequenceMode())
        {
          active_waypoints_.clear();
          current_wp_ = 0;
        }

        have_target_ = false;

        changeFSMExecState(WAIT_TARGET, "FSM");
        return;
      }
      else if ((end_pt_ - pos).norm() < no_replan_thresh_)
      {
        // cout << "near end" << endl;
        return;
      }
      else if ((info->start_pos_ - pos).norm() < replan_thresh_)
      {
        // cout << "near start" << endl;
        return;
      }
      else
      {
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }
      break;
    }

    case EMERGENCY_STOP:
    {

      if (flag_escape_emergency_) // Avoiding repeated calls
      {
        callEmergencyStop(odom_pos_);
      }
      else
      {
        if (enable_fail_safe_ && !need_hover_stop_ && odom_vel_.norm() < 0.1)
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
        else if (enable_fail_safe_ && need_hover_stop_ && odom_vel_.norm() < 0.1)
        {
          RCLCPP_INFO(node_->get_logger(),
                      "Exiting EMERGENCY_STOP; switching to WAIT_TARGET for a new target");
          need_hover_stop_ = false;
          have_target_ = false;
          trigger_ = false;
          changeFSMExecState(WAIT_TARGET, "EMERGENCY_EXIT");
        }
      }

      flag_escape_emergency_ = false;
      break;
    }
    }

    finishProcess();

    data_disp_.header.stamp = node_->now();
    data_disp_pub_->publish(data_disp_);
  }

  void SCANReplanFSM::finishProcess()
  {
    if (replan_fail_count_ >= max_replan_fail_count_)
    {
      recoverFromReplanFailure("finishProcess");
    }
  }

  bool SCANReplanFSM::recoverFromReplanFailure(const std::string& reason)
  {
    if (replan_fail_count_ < max_replan_fail_count_)
      return false;

    RCLCPP_WARN(node_->get_logger(),
                "Local replan failed %d times at waypoint %d/%zu (%s); retrying the current target",
                replan_fail_count_, current_wp_ + 1, active_waypoints_.size(), reason.c_str());

    replan_fail_count_ = 0;
    have_new_target_ = true;
    trigger_ = true;

    if (requestGlobalReplanFromCurrentGoal(reason))
    {
      callEmergencyStop(odom_pos_);
      have_target_ = false;
      have_new_target_ = false;
      changeFSMExecState(WAIT_TARGET, "GLOBAL_REPLAN_REQUEST");
      return true;
    }

    bool refreshed = false;
    if (isWaypointSequenceMode() && current_wp_ >= 0 && current_wp_ < (int)active_waypoints_.size())
    {
      refreshed = planNextWaypoint();
    }
    else if (have_target_)
    {
      setStartStateFromOdomOrCurrentTraj();
      refreshed = planner_manager_->planGlobalTraj(
          start_pt_,
          start_vel_,
          start_acc_,
          end_pt_,
          Eigen::Vector3d::Zero(),
          Eigen::Vector3d::Zero());
      if (refreshed)
        refreshed = adjustGlobalTargetIfOccupied();
    }

    if (refreshed)
    {
      planner_manager_->global_data_.last_progress_time_ = 0.0;
      changeFSMExecState(GEN_NEW_TRAJ, "REPLAN_RECOVERY");
      return true;
    }

    RCLCPP_WARN(node_->get_logger(),
                "Failed to refresh the current local target; emergency stop, then retry the same target");
    need_hover_stop_ = false;
    flag_escape_emergency_ = true;
    changeFSMExecState(EMERGENCY_STOP, "REPLAN_RECOVERY");
    return true;
  }

  bool SCANReplanFSM::planWaypointWindowFromStartState()
  {
    advanceWaypointIndexByProgress();

    if (current_wp_ < 0 || current_wp_ >= (int)active_waypoints_.size())
      return false;

    std::vector<Eigen::Vector3d> window;
    window.reserve(active_waypoints_.size() - current_wp_);

    int end_wp = current_wp_;
    double accumulated = 0.0;
    window.push_back(active_waypoints_[end_wp]);

    const double horizon = std::max(planning_horizon_, planner_manager_->pp_.ctrl_pt_dist);
    const int min_window_size = std::max(1, min_waypoint_window_size_);
    while (end_wp + 1 < (int)active_waypoints_.size() && accumulated < horizon)
    {
      accumulated += (active_waypoints_[end_wp + 1] - active_waypoints_[end_wp]).norm();
      end_wp++;
      window.push_back(active_waypoints_[end_wp]);
    }

    while ((int)window.size() < min_window_size && end_wp + 1 < (int)active_waypoints_.size())
    {
      accumulated += (active_waypoints_[end_wp + 1] - active_waypoints_[end_wp]).norm();
      end_wp++;
      window.push_back(active_waypoints_[end_wp]);
    }

    std::vector<Eigen::Vector3d> safe_window;
    safe_window.reserve(window.size());
    for (size_t i = 0; i < window.size(); ++i)
    {
      Eigen::Vector3d safe_wp = window[i];
      const Eigen::Vector3d from = safe_window.empty() ? start_pt_ : safe_window.back();
      if (!snapTargetToNearbyFreeCell(safe_wp, from, "waypoint-window"))
      {
        if (safe_window.empty())
        {
          RCLCPP_WARN(node_->get_logger(),
                      "First waypoint-window target is occupied and cannot be snapped; local window planning aborted");
          return false;
        }
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                             "Waypoint-window target %zu is occupied; truncating local window at previous free point",
                             i + 1);
        break;
      }
      safe_window.push_back(safe_wp);
    }

    if (!safe_window.empty())
      window.swap(safe_window);

    end_pt_ = window.back();
    end_vel_.setZero();
    if (end_wp + 1 < (int)active_waypoints_.size())
    {
      Eigen::Vector3d pass_dir = active_waypoints_[end_wp + 1] - active_waypoints_[end_wp];
      if (pass_dir.norm() < 1e-3 && end_wp > 0)
        pass_dir = active_waypoints_[end_wp] - active_waypoints_[end_wp - 1];
      if (pass_dir.norm() > 1e-3)
      {
        const double pass_speed = std::clamp(
            waypoint_pass_through_speed_scale_, 0.0, 1.0) * planner_manager_->pp_.max_vel_;
        end_vel_ = pass_dir.normalized() * pass_speed;
      }
    }

    bool success = planner_manager_->planGlobalTrajWaypoints(
        start_pt_,
        start_vel_,
        start_acc_,
        window,
        end_vel_,
        Eigen::Vector3d::Zero());
    if (!success)
    {
      RCLCPP_ERROR(node_->get_logger(),
                   "[navi_mode=%d] Unable to generate waypoint-window trajectory %d-%d/%zu",
                   navi_mode_, current_wp_ + 1, end_wp + 1, active_waypoints_.size());
      return false;
    }

    if (!adjustGlobalTargetIfOccupied())
      return false;

    constexpr double step_size_t = 0.1;
    int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t);
    std::vector<Eigen::Vector3d> gloabl_traj(i_end);
    for (int i = 0; i < i_end; i++)
      gloabl_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);

    have_target_ = true;
    have_new_target_ = true;
    visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
    visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, end_wp);
    RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                         "[navi_mode=%d] Planning waypoint window %d-%d/%zu, length=%.2f, target=[%.2f, %.2f, %.2f]",
                         navi_mode_, current_wp_ + 1, end_wp + 1, active_waypoints_.size(),
                         accumulated, end_pt_(0), end_pt_(1), end_pt_(2));
    return true;
  }

  bool SCANReplanFSM::requestGlobalReplanFromCurrentGoal(const std::string& reason)
  {
    if (!enable_global_replan_on_local_failure_ || !global_replan_goal_pub_ || !have_odom_)
      return false;

    const double cooldown = std::max(0.0, global_replan_request_cooldown_);
    const rclcpp::Time now = node_->now();
    const bool has_previous_request = last_global_replan_request_time_.nanoseconds() > 0;
    const double since_last = has_previous_request ? (now - last_global_replan_request_time_).seconds() : cooldown;
    if (cooldown > 0.0 && has_previous_request && since_last >= 0.0 && since_last < cooldown)
      return false;

    Eigen::Vector3d target = end_pt_;
    if (!active_waypoints_.empty())
      target = active_waypoints_.back();

    geometry_msgs::msg::PoseStamped goal;
    goal.header.stamp = now;
    goal.header.frame_id = self_inflation_frame_id_.empty() ? "world" : self_inflation_frame_id_;
    goal.pose.position.x = target(0);
    goal.pose.position.y = target(1);
    goal.pose.position.z = target(2) - local_path_z_offset_ - global_replan_goal_z_offset_;
    goal.pose.orientation.w = 1.0;
    global_replan_goal_pub_->publish(goal);
    last_global_replan_request_time_ = now;

    RCLCPP_WARN(node_->get_logger(),
                "Requested Octo global replan after local failure (%s): goal=[%.2f, %.2f, %.2f] topic=%s",
                reason.c_str(),
                goal.pose.position.x,
                goal.pose.position.y,
                goal.pose.position.z,
                global_replan_goal_topic_.c_str());
    return true;
  }

  bool SCANReplanFSM::runCompletionDrivenReplan()
  {
    if (!have_target_ || go2_execution_frozen_)
      return false;

    LocalTrajData *info = &planner_manager_->local_data_;
    if (info->start_time_.seconds() < 1e-5 || info->duration_ <= 1e-5)
      return false;

    const rclcpp::Time now = node_->now();
    const double min_interval = std::max(0.01, completion_replan_min_interval_);
    if ((now - last_replan_finish_time_).seconds() < min_interval)
      return false;

    const double t_cur = std::clamp(
        (now - info->start_time_).seconds(), 0.0, info->duration_);
    const double previous_remaining = std::max(0.0, info->duration_ - t_cur);
    const auto wall_start = std::chrono::steady_clock::now();
    const bool success = planFromCurrentTraj();
    const double elapsed_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - wall_start).count();
    last_replan_finish_time_ = node_->now();

    if (success)
    {
      replan_fail_count_ = 0;
      RCLCPP_INFO_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 1000,
          "Completion-driven local replan succeeded in %.2f ms", elapsed_ms);
      return true;
    }

    ++replan_fail_count_;
    RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 1000,
        "Completion-driven local replan failed in %.2f ms (%d/%d); keeping previous trajectory",
        elapsed_ms, replan_fail_count_, max_replan_fail_count_);

    if (replan_fail_count_ < std::max(1, max_replan_fail_count_))
      return true;

    if (previous_remaining > std::max(0.0, keep_previous_traj_min_remaining_))
    {
      requestGlobalReplanFromCurrentGoal("CONTINUOUS_REPLAN");
      replan_fail_count_ = 0;
      RCLCPP_WARN(
          node_->get_logger(),
          "Local replans exhausted, but the previous trajectory has %.2fs remaining; continuing it while global replanning runs",
          previous_remaining);
      return true;
    }

    recoverFromReplanFailure("CONTINUOUS_REPLAN_END");
    return true;
  }

  bool SCANReplanFSM::planFromCurrentTraj()
  {
    LocalTrajData *info = &planner_manager_->local_data_;
    rclcpp::Time time_now = node_->now();
    double t_cur = (time_now - info->start_time_).seconds();
    t_cur = std::min(std::max(t_cur, 0.0), info->duration_);

    //cout << "info->velocity_traj_=" << info->velocity_traj_.get_control_points() << endl;

    Eigen::Vector3d adjusted_odom = odom_pos_;
    if (!use_odom_z_)
      adjusted_odom(2) = navigation_z_;

    const double stitch_lookahead = std::max(0.0, replan_stitch_lookahead_);
    const double stitch_t = std::min(info->duration_, t_cur + stitch_lookahead);
    const Eigen::Vector3d stitched_position = info->position_traj_.evaluateDeBoorT(stitch_t);
    const double tracking_error =
        (stitched_position.head<2>() - adjusted_odom.head<2>()).norm();
    const bool use_stitched_state =
        stitch_lookahead > 1e-4 &&
        tracking_error <= std::max(0.0, replan_stitch_max_tracking_error_);

    // Anchor each high-rate replan to the measured robot position by default.
    // Derivatives still come from the active trajectory to preserve motion continuity.
    start_pt_ = use_stitched_state ? stitched_position : adjusted_odom;
    const double derivative_t = use_stitched_state ? stitch_t : t_cur;
    start_vel_ = info->velocity_traj_.evaluateDeBoorT(derivative_t);
    start_acc_ = info->acceleration_traj_.evaluateDeBoorT(derivative_t);
    start_vel_(2) = 0.0;
    start_acc_(2) = 0.0;
    if (stitch_lookahead > 1e-4 && !use_stitched_state)
    {
      RCLCPP_WARN_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 1000,
          "Trajectory stitch tracking error %.3f m exceeds %.3f m; replanning from odometry",
          tracking_error, replan_stitch_max_tracking_error_);
    }

    const Eigen::Vector2d to_goal = end_pt_.head<2>() - odom_pos_.head<2>();
    if (to_goal.norm() > 1e-3 && start_vel_.head<2>().dot(to_goal) < 0.0)
    {
      start_vel_.setZero();
      start_acc_.setZero();
    }

    if (navi_mode_ == NAVI_MODE::REFERENCE_PATH && !active_waypoints_.empty())
    {
      if (!planWaypointWindowFromStartState())
        return false;

      bool success = callReboundReplan(true, false);
      if (!success)
      {
        success = callReboundReplan(true, true);
        if (!success)
          return false;
      }
      return true;
    }

    if (!planner_manager_->planGlobalTraj(
            start_pt_,
            start_vel_,
            start_acc_,
            end_pt_,
            end_vel_,
            Eigen::Vector3d::Zero()))
    {
      RCLCPP_ERROR(node_->get_logger(),
                   "[navi_mode=%d] Unable to refresh global trajectory from odom to current target", navi_mode_);
      return false;
    }

    if (!adjustGlobalTargetIfOccupied())
      return false;

    bool success = callReboundReplan(true, false);
    if (!success)
    {
      success = callReboundReplan(true, true);
      if (!success)
        return false;
    }

    return true;
  }

  void SCANReplanFSM::setStartStateFromOdomOrCurrentTraj()
  {
    start_pt_ = odom_pos_;
    if (!use_odom_z_)
      start_pt_(2) = navigation_z_;
    start_vel_ = odom_vel_;
    start_acc_.setZero();
    start_vel_(2) = 0.0;

    LocalTrajData *info = &planner_manager_->local_data_;
    if (info->start_time_.seconds() < 1e-5 || info->duration_ <= 1e-5)
    {
      return;
    }

    const double raw_t_cur = (node_->now() - info->start_time_).seconds();
    if (raw_t_cur < -1e-3 || raw_t_cur > info->duration_ + 0.2)
    {
      return;
    }

    const double t_cur = std::min(std::max(raw_t_cur, 0.0), info->duration_);
    start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_cur);
    start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_cur);
    start_vel_(2) = 0.0;
    start_acc_(2) = 0.0;

    const Eigen::Vector2d to_goal = end_pt_.head<2>() - odom_pos_.head<2>();
    if (to_goal.norm() > 1e-3 && start_vel_.head<2>().dot(to_goal) < 0.0)
    {
      start_vel_.setZero();
      start_acc_.setZero();
    }
  }

  bool SCANReplanFSM::snapTargetToNearbyFreeCell(Eigen::Vector3d &target,
                                                 const Eigen::Vector3d &from,
                                                 const std::string &label)
  {
    if (!enable_target_snap_ || !planner_manager_ || !planner_manager_->grid_map_)
      return true;

    auto map = planner_manager_->grid_map_;
    const double yaw = estimateYawFromSegment(from, target);
    if (map->getInflateOccupancy(target, yaw) == 0)
      return true;

    const Eigen::Vector3d original = target;
    const double resolution = std::max(1e-3, map->getResolution());
    const double snap_radius = std::max(0.0, target_snap_radius_);
    const double snap_z_radius = std::max(0.0, target_snap_z_radius_);
    const int xy_cells = std::max(1, static_cast<int>(std::ceil(snap_radius / resolution)));
    const int z_cells = std::max(0, static_cast<int>(std::ceil(snap_z_radius / resolution)));

    Eigen::Vector3d best = original;
    double best_score = std::numeric_limits<double>::infinity();
    bool found = false;

    for (int dx = -xy_cells; dx <= xy_cells; ++dx)
    {
      for (int dy = -xy_cells; dy <= xy_cells; ++dy)
      {
        const double xy_dist = std::hypot(dx * resolution, dy * resolution);
        if (xy_dist > snap_radius + 1e-6)
          continue;

        for (int dz = -z_cells; dz <= z_cells; ++dz)
        {
          Eigen::Vector3d candidate = original + Eigen::Vector3d(
              dx * resolution,
              dy * resolution,
              dz * resolution);
          if (map->getInflateOccupancy(candidate, yaw) != 0)
            continue;

          const double z_dist = std::abs(dz * resolution);
          const double from_dist = 0.05 * (candidate - from).norm();
          const double score = xy_dist + 1.4 * z_dist + from_dist;
          if (score < best_score)
          {
            best_score = score;
            best = candidate;
            found = true;
          }
        }
      }
    }

    if (!found)
    {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                           "%s target is occupied and no nearby free snap cell was found around [%.2f, %.2f, %.2f]",
                           label.c_str(), original(0), original(1), original(2));
      return false;
    }

    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                         "%s target snapped from [%.2f, %.2f, %.2f] to nearby free cell [%.2f, %.2f, %.2f]",
                         label.c_str(), original(0), original(1), original(2), best(0), best(1), best(2));
    target = best;
    return true;
  }

  void SCANReplanFSM::checkCollisionCallback()
  {
    updateLocalTrajTimeFreeze();
    if (!ensureFreshMap())
      return;

    LocalTrajData *info = &planner_manager_->local_data_;
    auto map = planner_manager_->grid_map_;

    if (exec_state_ == WAIT_TARGET || info->start_time_.seconds() < 1e-5)
      return;

    /* ---------- check trajectory ---------- */
    const double time_step = std::clamp(collision_check_dt_, 0.01, 0.20);
    double t_cur = (node_->now() - info->start_time_).seconds();
    const double t_end = std::min(info->duration_, t_cur + std::max(0.5, collision_check_horizon_));
    for (double t = t_cur; t < t_end; t += time_step)
    {
      Eigen::Vector3d pos = info->position_traj_.evaluateDeBoorT(t);
      Eigen::Vector3d pos_next = info->position_traj_.evaluateDeBoorT(std::min(t + time_step, info->duration_));
      if (map->getInflateOccupancy(pos, estimateYawFromSegment(pos, pos_next)))
      {
        if (planFromCurrentTraj()) // Make a chance
        {
          changeFSMExecState(EXEC_TRAJ, "SAFETY");
          return;
        }
        else
        {
          if (t - t_cur < emergency_time_) // 0.8s of emergency time
          {
            RCLCPP_WARN(node_->get_logger(), "Obstacle discovered; emergency stop in %.3fs", t - t_cur);
            changeFSMExecState(EMERGENCY_STOP, "SAFETY");
          }
          else
          {
            //ROS_WARN("current traj in collision, replan.");
            changeFSMExecState(REPLAN_TRAJ, "SAFETY");
          }
          return;
        }
        break;
      }
    }
  }

  bool SCANReplanFSM::callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj)
  {

    getLocalTarget();

    bool plan_success =
        planner_manager_->reboundReplan(start_pt_, start_vel_, start_acc_, local_target_pt_, local_target_vel_, (have_new_target_ || flag_use_poly_init), flag_randomPolyTraj);
    have_new_target_ = false;

    cout << "final_plan_success=" << plan_success << endl;

    if (plan_success)
    {

      auto info = &planner_manager_->local_data_;

      /* publish traj */
      planner::msg::Bspline bspline;
      bspline.order = 3;
      bspline.start_time = info->start_time_;
      bspline.traj_id = info->traj_id_;

      Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
      bspline.pos_pts.reserve(pos_pts.cols());
      for (int i = 0; i < pos_pts.cols(); ++i)
      {
        geometry_msgs::msg::Point pt;
        pt.x = pos_pts(0, i);
        pt.y = pos_pts(1, i);
        pt.z = pos_pts(2, i);
        bspline.pos_pts.push_back(pt);
      }

      Eigen::VectorXd knots = info->position_traj_.getKnot();
      bspline.knots.reserve(knots.rows());
      for (int i = 0; i < knots.rows(); ++i)
      {
        bspline.knots.push_back(knots(i));
      }

      bspline_pub_->publish(bspline);

      visualization_->displayOptimalTraj(info->position_traj_, 0);
    }

    return plan_success;
  }

  bool SCANReplanFSM::callEmergencyStop(Eigen::Vector3d stop_pos)
  {

    planner_manager_->EmergencyStop(stop_pos);

    auto info = &planner_manager_->local_data_;

    /* publish traj */
    planner::msg::Bspline bspline;
    bspline.order = 3;
    bspline.start_time = info->start_time_;
    bspline.traj_id = info->traj_id_;

    Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
    bspline.pos_pts.reserve(pos_pts.cols());
    for (int i = 0; i < pos_pts.cols(); ++i)
    {
      geometry_msgs::msg::Point pt;
      pt.x = pos_pts(0, i);
      pt.y = pos_pts(1, i);
      pt.z = pos_pts(2, i);
      bspline.pos_pts.push_back(pt);
    }

    Eigen::VectorXd knots = info->position_traj_.getKnot();
    bspline.knots.reserve(knots.rows());
    for (int i = 0; i < knots.rows(); ++i)
    {
      bspline.knots.push_back(knots(i));
    }

    bspline_pub_->publish(bspline);

    return true;
  }

  void SCANReplanFSM::getLocalTarget()
  {
    double t;

    local_target_pt_ = end_pt_;
    local_target_vel_.setZero();
    double t_step = planning_horizon_ / 20 / std::max(1e-3, planner_manager_->pp_.max_vel_);
    double dist_min = 9999, dist_min_t = 0.0;
    double target_t = planner_manager_->global_data_.global_duration_;
    for (t = planner_manager_->global_data_.last_progress_time_; t < planner_manager_->global_data_.global_duration_; t += t_step)
    {
      Eigen::Vector3d pos_t = planner_manager_->global_data_.getPosition(t);
      if (!pos_t.allFinite())
      {
        RCLCPP_ERROR(node_->get_logger(), "Global reference trajectory produced a non-finite local target");
        local_target_pt_ = end_pt_;
        target_t = planner_manager_->global_data_.global_duration_;
        break;
      }
      double dist = (pos_t - start_pt_).norm();

      if (t < planner_manager_->global_data_.last_progress_time_ + 1e-5 && dist > planning_horizon_)
      {
        RCLCPP_ERROR(node_->get_logger(),
                     "Local target progress mismatch: distance=%.3f horizon=%.3f progress_time=%.3f",
                     dist, planning_horizon_, planner_manager_->global_data_.last_progress_time_);
        local_target_pt_ = pos_t;
        target_t = t;
        planner_manager_->global_data_.last_progress_time_ = t;
        break;
      }
      if (dist < dist_min)
      {
        dist_min = dist;
        dist_min_t = t;
      }
      if (dist >= planning_horizon_)
      {
        local_target_pt_ = pos_t;
        target_t = t;
        planner_manager_->global_data_.last_progress_time_ = dist_min_t;
        break;
      }
    }
    if (t >= planner_manager_->global_data_.global_duration_) // Last global point
    {
      local_target_pt_ = end_pt_;
      target_t = planner_manager_->global_data_.global_duration_;
    }

    auto targetOccupancy = [&](const Eigen::Vector3d &pt) {
      return planner_manager_->grid_map_->getInflateOccupancy(pt, estimateYawFromSegment(odom_pos_, pt));
    };

    if (targetOccupancy(local_target_pt_) != 0)
    {
      Eigen::Vector3d snapped_target = local_target_pt_;
      if (snapTargetToNearbyFreeCell(snapped_target, start_pt_, "local"))
      {
        local_target_pt_ = snapped_target;
      }
      else
      {
      bool found_free_target = false;
      double adjusted_t = target_t;

      for (double dt = 0.0; dt <= planner_manager_->global_data_.global_duration_; dt += t_step)
      {
        double t_forward = target_t + dt;
        if (t_forward <= planner_manager_->global_data_.global_duration_)
        {
          Eigen::Vector3d pt = planner_manager_->global_data_.getPosition(t_forward);
          if (targetOccupancy(pt) == 0)
          {
            local_target_pt_ = pt;
            adjusted_t = t_forward;
            found_free_target = true;
            break;
          }
        }

        double t_backward = target_t - dt;
        if (t_backward >= std::max(0.0, dist_min_t))
        {
          Eigen::Vector3d pt = planner_manager_->global_data_.getPosition(t_backward);
          if (targetOccupancy(pt) == 0)
          {
            local_target_pt_ = pt;
            adjusted_t = t_backward;
            found_free_target = true;
            break;
          }
        }
      }

      if (found_free_target)
      {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                             "Local target was adjusted to a nearby collision-free point");
        target_t = adjusted_t;
      }
      else
      {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                             "Local target is in collision and no nearby free target was found");
      }
      }
    }

    if ((end_pt_ - local_target_pt_).norm() < (planner_manager_->pp_.max_vel_ * planner_manager_->pp_.max_vel_) / (2 * planner_manager_->pp_.max_acc_))
    {
      // local_target_vel_ = (end_pt_ - init_pt_).normalized() * planner_manager_->pp_.max_vel_ * (( end_pt_ - local_target_pt_ ).norm() / ((planner_manager_->pp_.max_vel_*planner_manager_->pp_.max_vel_)/(2*planner_manager_->pp_.max_acc_)));
      // cout << "A" << endl;
      local_target_vel_ = Eigen::Vector3d::Zero();
    }
    else
    {
      local_target_vel_ = planner_manager_->global_data_.getVelocity(target_t);
      // cout << "AA" << endl;
    }
  }

} // namespace scan_planner
