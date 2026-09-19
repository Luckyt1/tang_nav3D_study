"""Connect the imported planners to this workspace's BTC/ICP localization."""

from pathlib import Path
import math

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def snapshot_directory(value):
    if not value:
        raise ValueError("map_directory must name a completed BTC map snapshot")
    directory = Path(value).expanduser().resolve()
    required = ("metadata.yaml", "poses.csv", "static_map.pcd", "btc/manifest.csv")
    if (directory / ".incomplete").exists() or not all(
            (directory / name).is_file() for name in required):
        raise ValueError(f"Incomplete BTC/PCD map snapshot: {directory}")
    return directory


def start_navigation(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)

    def enabled(name):
        return value(name).lower() in ("true", "1", "yes", "on")

    directory = snapshot_directory(value("map_directory"))
    share = get_package_share_path("planner")
    planner_config = str(share / "config/planner.yaml")
    controller_config = str(share / "config/controllers.yaml")
    sim_time = {"use_sim_time": enabled("use_sim_time")}
    radius, height = float(value("robot_radius")), float(value("body_height"))
    if not all(math.isfinite(v) and v > 0 for v in (radius, height)):
        raise ValueError("robot_radius and body_height must be finite and positive")
    if not all(math.isfinite(float(value(name))) for name in
               ("ground_height", "mount_roll", "mount_pitch", "mount_yaw")):
        raise ValueError("ground_height and mounting angles must be finite")
    nodes = []
    if enabled("start_relocalization"):
        nodes.append(Node(
            package="relocalization", executable="relocalization_node",
            output="screen", parameters=[sim_time, {"map.directory": str(directory)}]))
    nodes.extend([
        Node(
            package="planner", executable="base_link_odometry.py",
            name="base_link_odometry", output="screen",
            parameters=[sim_time, {
                "sensor_frame": "imu", "base_frame": "base_link",
                "mount_roll": float(value("mount_roll")),
                "mount_pitch": float(value("mount_pitch")),
                "mount_yaw": float(value("mount_yaw")),
            }],
            remappings=[("input_odom", "/odin1/odometry"),
                        ("base_odom", "/base_link/odometry")]),
        Node(
            package="planner", executable="relocalization_bridge.py",
            name="relocalization_bridge", output="screen",
            parameters=[sim_time, {"map_frame": "map", "odom_frame": "odom"}],
            remappings=[("input_odom", "/base_link/odometry"),
                        ("map_odom", "/map/base_link/odometry"),
                        ("input_path", "/map/initial_path"),
                        ("output_path", "/initial_path"),
                        ("input_goal", "/goal_pose"),
                        ("input_replan_goal", "/odom/global_replan_goal"),
                        ("output_goal", "/navigation/map_goal")]),
        Node(
            package="planner", executable="octo_global_planner_node",
            name="bxi_octo_global_planner", output="screen",
            parameters=[planner_config, sim_time, {
                "frame_id": "map", "input_pcd": str(directory / "static_map.pcd"),
                "odom_topic": "/map/base_link/odometry",
                "goal_topic": "/navigation/map_goal", "path_topic": "/map/initial_path",
                "robot_radius": radius,
                # This snapshot has submap poses, not a surveyed ground trajectory.
                "trusted_ground_enabled": False,
            }]),
        Node(
            package="planner", executable="scan_planner_node",
            name="scan_planner_node", output="screen",
            parameters=[planner_config, sim_time, {
                "fsm.navi_mode": 3, "grid_map.frame_id": "odom",
                "grid_map.cloud_is_world": True,
                "grid_map.double_cylinder_radius": radius,
                "grid_map.body_height": height,
                "grid_map.ground_height": float(value("ground_height")),
                "fsm.global_replan_goal_topic": "/odom/global_replan_goal",
            }],
            remappings=[("body_pose", "/base_link/odometry"),
                        ("sensor_pose", "/odin1/odometry"),
                        ("cloud", "/odin1/cloud_slam"),
                        ("initial_path", "/initial_path")]),
    ])
    if enabled("start_controller"):
        nodes.append(Node(
            package="planner", executable="closed_loop_controller",
            name="closed_loop_controller", output="screen",
            parameters=[controller_config, sim_time, {"require_navigation_enable": True}],
            remappings=[("body_pose", "/base_link/odometry"),
                        ("cmd_vel", value("cmd_vel_topic"))]))
    if enabled("start_rviz"):
        nodes.append(Node(
            package="rviz2", executable="rviz2", name="navigation_rviz",
            arguments=["-d", str(share / "config/navigation.rviz")], parameters=[sim_time]))
    return nodes


def generate_launch_description():
    arguments = {
        "map_directory": ("", "Completed map snapshot used by both localization and planning"),
        "start_relocalization": ("true", "Set false when the existing localization node is running"),
        "start_rviz": ("true", "Show the map, global path and local trajectory"),
        "start_controller": ("false", "Enable velocity output after configuring the robot"),
        "cmd_vel_topic": ("/cmd_vel", "Controller velocity output"),
        "use_sim_time": ("false", "Use the ROS simulation clock"),
        "robot_radius": ("0.3", "Robot collision radius in meters, using the tuned 3D_nav value"),
        "body_height": ("0.35", "Robot collision height in meters"),
        "ground_height": ("-1.1", "Ground Z in odom, using the tuned 3D_nav value"),
        "mount_roll": ("0.0", "IMU mounting roll in base_link, radians"),
        "mount_pitch": ("0.7853981633974483", "IMU mounting pitch in base_link, radians (45 degrees)"),
        "mount_yaw": ("0.0", "IMU mounting yaw in base_link, radians"),
    }
    return LaunchDescription([
        DeclareLaunchArgument(name, default_value=default, description=description)
        for name, (default, description) in arguments.items()
    ] + [OpaqueFunction(function=start_navigation)])
