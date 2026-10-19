"""Connect the imported planners to this workspace's BTC/ICP localization."""

from pathlib import Path
import importlib.util
import math

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _load_sensor_config_module():
    path = Path(__file__).with_name("sensor_config.py")
    spec = importlib.util.spec_from_file_location("planner_sensor_config", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


sensor_config = _load_sensor_config_module()


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
    backend = sensor_config.normalize_backend(value("sensor_backend"))
    point_lio_config = value("point_lio_config")
    if backend == sensor_config.MID360_BACKEND:
        if not point_lio_config:
            point_lio_config = str(get_package_share_path("point_lio") / "config/mid360.yaml")
        point_lio = sensor_config.load_point_lio_config(point_lio_config)
        sensor = dict(sensor_config.MID360_SENSOR)
        sensor["lidar_extrinsic_translation"] = point_lio["extrinsic_T"]
        sensor["lidar_extrinsic_rotation"] = point_lio["extrinsic_R"]
    else:
        sensor = dict(sensor_config.ODIN_SENSOR)
    radius, height = float(value("robot_radius")), float(value("body_height"))
    if not all(math.isfinite(v) and v > 0 for v in (radius, height)):
        raise ValueError("robot_radius and body_height must be finite and positive")
    if not math.isfinite(float(value("ground_height"))):
        raise ValueError("ground_height must be finite")
    mount_roll, mount_pitch, mount_yaw = sensor_config.resolve_mount_rpy(
        backend, value("mount_roll"), value("mount_pitch"), value("mount_yaw"))
    grid_map_params = {
        "fsm.navi_mode": 3, "grid_map.frame_id": sensor.get("world_frame", "odom"),
        "grid_map.cloud_is_world": sensor["cloud_is_world"],
        "grid_map.need_extrinsic": sensor["need_extrinsic"],
        "grid_map.double_cylinder_radius": radius,
        "grid_map.body_height": height,
        "grid_map.ground_height": float(value("ground_height")),
        "fsm.global_replan_goal_topic": "/odom/global_replan_goal",
    }
    if backend == sensor_config.MID360_BACKEND:
        grid_map_params.update({
            "grid_map.lidar_extrinsic_translation": sensor["lidar_extrinsic_translation"],
            "grid_map.lidar_extrinsic_rotation": sensor["lidar_extrinsic_rotation"],
        })
    nodes = []
    if enabled("start_relocalization"):
        nodes.append(Node(
            package="relocalization", executable="relocalization_node",
            output="screen", parameters=[sim_time, {
                "map.directory": str(directory),
                "input_topic": sensor["cloud_topic"],
                "odom_frame": "odom",
                "sensor_frame": sensor["sensor_frame"],
                "map_frame": "map",
                "fine.max_gravity_angle_deg": (
                    30.0 if backend == sensor_config.MID360_BACKEND else 180.0),
            }]))
    nodes.extend([
        Node(
            package="planner", executable="base_link_odometry.py",
            name="base_link_odometry", output="screen",
            parameters=[sim_time, {
                "sensor_frame": sensor["raw_odom_frame"], "base_frame": "base_link",
                "mount_roll": mount_roll,
                "mount_pitch": mount_pitch,
                "mount_yaw": mount_yaw,
            }],
            remappings=[("input_odom", sensor["raw_odom_topic"]),
                        ("base_odom", sensor["base_odom_topic"])]),
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
            parameters=[planner_config, sim_time, grid_map_params],
            remappings=[("body_pose", sensor["base_odom_topic"]),
                        ("sensor_pose", sensor["raw_odom_topic"]),
                        ("cloud", sensor["cloud_topic"]),
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
        "sensor_backend": ("odin", "Sensor/odometry backend: odin or mid360"),
        "point_lio_config": ("", "Point-LIO YAML used only when sensor_backend:=mid360"),
        "use_sim_time": ("false", "Use the ROS simulation clock"),
        "robot_radius": ("0.3", "Robot collision radius in meters, using the tuned 3D_nav value"),
        "body_height": ("0.35", "Robot collision height in meters"),
        "ground_height": ("-1.1", "Ground Z in odom, using the tuned 3D_nav value"),
        "mount_roll": ("auto", "Sensor mounting roll in base_link, radians or auto"),
        "mount_pitch": ("auto", "Sensor mounting pitch in base_link, radians or auto"),
        "mount_yaw": ("auto", "Sensor mounting yaw in base_link, radians or auto"),
    }
    return LaunchDescription([
        DeclareLaunchArgument(name, default_value=default, description=description)
        for name, (default, description) in arguments.items()
    ] + [OpaqueFunction(function=start_navigation)])
