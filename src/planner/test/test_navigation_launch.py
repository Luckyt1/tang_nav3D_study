import importlib.util
import math
import os
from pathlib import Path

import pytest
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch.utilities import perform_substitutions


def module():
    path = Path(__file__).parents[1] / "launch/navigation.launch.py"
    spec = importlib.util.spec_from_file_location("navigation_launch", path)
    loaded = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(loaded)
    return loaded


def mid360_module():
    path = Path(__file__).parents[1] / "launch/mid360_odometry.launch.py"
    spec = importlib.util.spec_from_file_location("mid360_odometry_launch", path)
    loaded = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(loaded)
    return loaded


def mid360_config(tmp_path):
    path = tmp_path / "mid360.yaml"
    path.write_text(
        """
/**:
  ros__parameters:
    common:
      lid_topic: livox/lidar
      imu_topic: livox/imu
    mapping:
      extrinsic_est_en: false
      extrinsic_T: [ -0.011, -0.02329, 0.04412 ]
      extrinsic_R: [ 1.0, 0.0, 0.0,
                     0.0, 1.0, 0.0,
                     0.0, 0.0, 1.0 ]
    odometry:
      publish_odometry_without_downsample: false
    publish:
      scan_publish_en: true
      tf_send_en: true
      world_frame: odom
      body_frame: body_raw
    pcd_save:
      pcd_save_en: false
""",
        encoding="utf-8",
    )
    return path


def launch_context(**overrides):
    context = LaunchContext()
    launch = module()
    arguments = {
        action.name: perform_substitutions(LaunchContext(), action.default_value)
        for action in launch.generate_launch_description().entities
        if isinstance(action, DeclareLaunchArgument)
    }
    arguments.update({name: str(value) for name, value in overrides.items()})
    for name, value in arguments.items():
        context.launch_configurations[name] = value
    return context


def mid360_context(**overrides):
    context = LaunchContext()
    launch = mid360_module()
    arguments = {
        action.name: perform_substitutions(LaunchContext(), action.default_value)
        for action in launch.generate_launch_description().entities
        if isinstance(action, DeclareLaunchArgument)
    }
    arguments.update({name: str(value) for name, value in overrides.items()})
    for name, value in arguments.items():
        context.launch_configurations[name] = value
    return context


def node_name(node):
    return getattr(node, "_Node__node_name")


def remappings(node):
    context = LaunchContext()
    return {
        perform_substitutions(context, source): perform_substitutions(context, target)
        for source, target in getattr(node, "_Node__remappings")
    }


def parameters(node):
    context = LaunchContext()
    def convert(value):
        if isinstance(value, tuple) and value and hasattr(value[0], "perform"):
            text = perform_substitutions(context, value)
            return text[:-5] if text.endswith("\n...\n") else text
        return value
    converted = []
    for item in getattr(node, "_Node__parameters"):
        if isinstance(item, dict):
            converted.append({
                perform_substitutions(context, key): convert(value)
                for key, value in item.items()
            })
        else:
            converted.append(item)
    return converted


def arguments(node):
    return getattr(node, "_Node__arguments")


def test_rejects_missing_or_partial_localization_snapshot(tmp_path):
    launch = module()
    with pytest.raises(ValueError):
        launch.snapshot_directory("")
    with pytest.raises(ValueError):
        launch.snapshot_directory(str(tmp_path))
    for name in ("metadata.yaml", "poses.csv", "static_map.pcd", "btc/manifest.csv"):
        path = tmp_path / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.touch()
    assert launch.snapshot_directory(str(tmp_path)) == tmp_path.resolve()
    (tmp_path / ".incomplete").touch()
    with pytest.raises(ValueError):
        launch.snapshot_directory(str(tmp_path))


def test_default_launch_previews_paths_without_starting_velocity_controller(tmp_path, monkeypatch):
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path / "ros_log"))
    arguments = {
        action.name: perform_substitutions(LaunchContext(), action.default_value)
        for action in module().generate_launch_description().entities
        if isinstance(action, DeclareLaunchArgument)
    }
    assert arguments["start_controller"] == "false"
    assert arguments["sensor_backend"] == "odin"
    assert arguments["mount_pitch"] == "auto"


def test_odin_backend_keeps_existing_topics_and_mount(tmp_path, monkeypatch):
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path / "ros_log"))
    launch = module()
    monkeypatch.setattr(launch, "get_package_share_path", lambda package: tmp_path)
    for name in ("metadata.yaml", "poses.csv", "static_map.pcd", "btc/manifest.csv"):
        path = tmp_path / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.touch()

    nodes = launch.start_navigation(launch_context(map_directory=tmp_path))
    relocalization = next(node for node in nodes if node_name(node) is None)
    base_odom = next(node for node in nodes if node_name(node) == "base_link_odometry")
    scan = next(node for node in nodes if node_name(node) == "scan_planner_node")

    assert parameters(relocalization)[1]["input_topic"] == "/odin1/cloud_slam"
    assert parameters(relocalization)[1]["sensor_frame"] == "lidar"
    assert parameters(relocalization)[1]["fine.max_gravity_angle_deg"] == 180.0
    assert remappings(base_odom) == {
        "input_odom": "/odin1/odometry",
        "base_odom": "/base_link/odometry",
    }
    assert parameters(base_odom)[1]["sensor_frame"] == "imu"
    assert parameters(base_odom)[1]["mount_pitch"] == pytest.approx(math.pi / 4.0)
    assert remappings(scan)["cloud"] == "/odin1/cloud_slam"
    assert parameters(scan)[2]["grid_map.need_extrinsic"] is False


def test_mid360_backend_uses_point_lio_topics_and_extrinsic(tmp_path, monkeypatch):
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path / "ros_log"))
    launch = module()
    monkeypatch.setattr(launch, "get_package_share_path", lambda package: tmp_path)
    for name in ("metadata.yaml", "poses.csv", "static_map.pcd", "btc/manifest.csv"):
        path = tmp_path / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.touch()

    nodes = launch.start_navigation(launch_context(
        map_directory=tmp_path,
        sensor_backend="mid360",
        point_lio_config=mid360_config(tmp_path),
    ))
    relocalization = next(node for node in nodes if node_name(node) is None)
    base_odom = next(node for node in nodes if node_name(node) == "base_link_odometry")
    scan = next(node for node in nodes if node_name(node) == "scan_planner_node")

    assert parameters(relocalization)[1]["input_topic"] == "/cloud_registered"
    assert parameters(relocalization)[1]["sensor_frame"] == "livox_frame"
    assert parameters(relocalization)[1]["odom_frame"] == "odom"
    assert parameters(relocalization)[1]["fine.max_gravity_angle_deg"] == 30.0
    assert remappings(base_odom) == {
        "input_odom": "/aft_mapped_to_init",
        "base_odom": "/base_link/odometry",
    }
    assert parameters(base_odom)[1]["sensor_frame"] == "body_raw"
    assert parameters(base_odom)[1]["mount_roll"] == pytest.approx(math.pi)
    assert remappings(scan)["sensor_pose"] == "/aft_mapped_to_init"
    assert remappings(scan)["cloud"] == "/cloud_registered"
    assert parameters(scan)[2]["grid_map.lidar_extrinsic_translation"] == pytest.approx(
        [-0.011, -0.02329, 0.04412])
    assert parameters(scan)[2]["grid_map.lidar_extrinsic_rotation"] == pytest.approx(
        [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0])


def test_mid360_odometry_defaults_to_mid360s_driver_config(tmp_path, monkeypatch):
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path / "ros_log"))
    launch = mid360_module()
    livox_share = tmp_path / "livox_ros_driver2"
    livox_config = livox_share / "config/MID360s_config.json"
    livox_config.parent.mkdir(parents=True)
    livox_config.write_text("{}", encoding="utf-8")
    monkeypatch.setattr(launch, "get_package_share_path", lambda package: livox_share)

    nodes = launch.start_mid360_odometry(mid360_context(
        point_lio_config=mid360_config(tmp_path),
    ))
    driver = next(node for node in nodes if node_name(node) == "livox_lidar_publisher")
    assert parameters(driver)[0]["user_config_path"] == str(livox_config)


def test_mid360_odometry_launch_forces_driver_point_lio_and_static_tf(tmp_path, monkeypatch):
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path / "ros_log"))
    config = mid360_config(tmp_path)
    livox_config = tmp_path / "MID360_config.json"
    livox_config.write_text("{}", encoding="utf-8")

    nodes = mid360_module().start_mid360_odometry(mid360_context(
        point_lio_config=config,
        livox_config=livox_config,
    ))
    driver = next(node for node in nodes if node_name(node) == "livox_lidar_publisher")
    point_lio = next(node for node in nodes if node_name(node) == "pointlio_mapping")
    static_tf = next(node for node in nodes if node_name(node) == "body_raw_to_livox_frame")

    assert parameters(driver)[0]["xfer_format"] == 1
    assert parameters(driver)[0]["multi_topic"] == 0
    assert parameters(driver)[0]["frame_id"] == "livox_frame"
    assert parameters(driver)[0]["publish_freq"] == pytest.approx(10.0)
    assert parameters(driver)[0]["user_config_path"] == str(livox_config)

    overrides = parameters(point_lio)[1]
    assert overrides["common.lid_topic"] == "/livox/lidar"
    assert overrides["common.imu_topic"] == "/livox/imu"
    assert overrides["preprocess.lidar_type"] == 1
    assert overrides["mapping.extrinsic_est_en"] is False
    assert overrides["publish.world_frame"] == "odom"
    assert overrides["publish.body_frame"] == "body_raw"
    assert overrides["publish.scan_publish_en"] is True
    assert overrides["publish.tf_send_en"] is True
    assert overrides["odometry.publish_odometry_without_downsample"] is False
    assert overrides["pcd_save.pcd_save_en"] is False

    tf_args = arguments(static_tf)
    assert "--frame-id" in tf_args
    assert tf_args[tf_args.index("--frame-id") + 1] == "body_raw"
    assert tf_args[tf_args.index("--child-frame-id") + 1] == "livox_frame"
    assert float(tf_args[tf_args.index("--x") + 1]) == pytest.approx(-0.011)


@pytest.mark.parametrize("rotation", [
    [1, 0, 0, 0, 2, 0, 0, 0, 1],  # scaling is not a rotation
    [1, 0, 0, 0, 1, 0, 0, 0, -1],  # reflections cannot be represented by TF
])
def test_mid360_rejects_invalid_rotation_before_starting_nodes(tmp_path, rotation):
    import yaml

    path = mid360_config(tmp_path)
    data = yaml.safe_load(path.read_text())
    data['/**']['ros__parameters']['mapping']['extrinsic_R'] = rotation
    path.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError, match='orthonormal'):
        module().sensor_config.load_point_lio_config(str(path))


def test_mid360_passes_normalized_file_and_double_extrinsics_to_point_lio(tmp_path, monkeypatch):
    import yaml

    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path / 'ros_log'))
    path = mid360_config(tmp_path)
    data = yaml.safe_load(path.read_text())
    mapping = data['/**']['ros__parameters']['mapping']
    mapping['extrinsic_T'] = [0, 0, 0]
    # Non-identity rotation also exercises the TF conversion direction.
    mapping['extrinsic_R'] = [0, -1, 0, 1, 0, 0, 0, 0, 1]
    path.write_text(yaml.safe_dump(data))
    livox_path = tmp_path / 'MID360_config.json'
    livox_path.write_text('{}')
    home_relative_path = '~/' + os.path.relpath(path, Path.home())
    nodes = mid360_module().start_mid360_odometry(mid360_context(
        point_lio_config=home_relative_path, livox_config=livox_path))
    lio = next(node for node in nodes if node_name(node) == 'pointlio_mapping')
    overrides = parameters(lio)[1]
    assert all(isinstance(v, float) for v in overrides['mapping.extrinsic_R'])
    assert all(isinstance(v, float) for v in overrides['mapping.extrinsic_T'])
    from launch_ros.utilities import evaluate_parameters
    evaluated = evaluate_parameters(LaunchContext(), getattr(lio, '_Node__parameters'))
    assert evaluated[0] == path.resolve()
    tf = next(node for node in nodes if node_name(node) == 'body_raw_to_livox_frame')
    tf_args = arguments(tf)
    assert float(tf_args[tf_args.index('--yaw') + 1]) == pytest.approx(math.pi / 2)


def test_mid360_rejects_missing_driver_config(tmp_path):
    with pytest.raises(ValueError, match='Livox config'):
        mid360_module().start_mid360_odometry(mid360_context(
            point_lio_config=mid360_config(tmp_path),
            livox_config=tmp_path / 'missing.json'))
