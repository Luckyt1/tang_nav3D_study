"""Start the MID360S Livox driver and Point-LIO odometry for navigation."""

from pathlib import Path
import importlib.util

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, Shutdown
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _load_sensor_config_module():
    path = Path(__file__).with_name("sensor_config.py")
    spec = importlib.util.spec_from_file_location("planner_sensor_config", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


sensor_config = _load_sensor_config_module()


def start_mid360_odometry(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)

    point_lio_config = value("point_lio_config")
    if not point_lio_config:
        point_lio_config = str(get_package_share_path("point_lio") / "config/mid360.yaml")
    livox_config = value("livox_config")
    if not livox_config:
        livox_config = str(get_package_share_path("livox_ros_driver2") / "config/MID360s_config.json")

    point_lio = sensor_config.load_point_lio_config(point_lio_config)
    livox_path = Path(livox_config).expanduser().resolve()
    if not livox_path.is_file():
        raise ValueError(f"Livox config does not exist: {livox_path}")
    roll, pitch, yaw = sensor_config.matrix_to_rpy(point_lio["extrinsic_R"])
    x, y, z = point_lio["extrinsic_T"]

    nodes = [
        Node(
            package="livox_ros_driver2",
            executable="livox_ros_driver2_node",
            name="livox_lidar_publisher",
            output="screen",
            condition=IfCondition(LaunchConfiguration("start_driver")),
            parameters=[{
                "xfer_format": 1,
                "multi_topic": 0,
                "data_src": 0,
                "publish_freq": 10.0,
                "output_data_type": 0,
                "frame_id": "livox_frame",
                "lvx_file_path": "/home/livox/livox_test.lvx",
                "user_config_path": str(livox_path),
                "cmdline_input_bd_code": "livox0000000001",
            }],
        ),
        Node(
            package="point_lio",
            executable="pointlio_mapping",
            name="pointlio_mapping",
            output="screen",
            parameters=[point_lio["path"], {
                **sensor_config.point_lio_overrides(),
                # Use the same validated doubles as TF and grid_map, even when
                # a YAML file spells an identity matrix with integer literals.
                "mapping.extrinsic_T": point_lio["extrinsic_T"],
                "mapping.extrinsic_R": point_lio["extrinsic_R"],
            }],
            remappings=[("/tf", "tf"), ("/tf_static", "tf_static"), ("path", "/debug/path")],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="body_raw_to_livox_frame",
            output="screen",
            arguments=[
                "--x", str(x),
                "--y", str(y),
                "--z", str(z),
                "--roll", str(roll),
                "--pitch", str(pitch),
                "--yaw", str(yaw),
                "--frame-id", "body_raw",
                "--child-frame-id", "livox_frame",
            ],
        ),
    ]
    if value("start_rviz").lower() in ("true", "1", "yes"):
        rviz_config = value("rviz_config") or str(
            get_package_share_path("planner") / "config/odometry.rviz")
        if not Path(rviz_config).is_file():
            raise ValueError(f"Odometry RViz config does not exist: {rviz_config}")
        nodes.append(Node(
            package="rviz2",
            executable="rviz2",
            name="odometry_rviz",
            output="screen",
            arguments=["-d", str(Path(rviz_config).resolve())],
            on_exit=[Shutdown(reason="Odometry RViz closed")],
        ))
    return nodes


def generate_launch_description():
    arguments = {
        "point_lio_config": ("", "Point-LIO YAML; defaults to point_lio/config/mid360.yaml"),
        "livox_config": ("", "Livox JSON; defaults to livox_ros_driver2/config/MID360s_config.json"),
        "start_driver": ("true", "Set false to use existing /livox/lidar and /livox/imu publishers"),
        "start_rviz": ("false", "Show raw Point-LIO odometry and the latest cloud"),
        "rviz_config": ("", "RViz config; defaults to planner/config/odometry.rviz"),
    }
    return LaunchDescription([
        DeclareLaunchArgument(name, default_value=default, description=description)
        for name, (default, description) in arguments.items()
    ] + [OpaqueFunction(function=start_mid360_odometry)])
