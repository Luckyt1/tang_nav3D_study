import importlib.util
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
    assert float(arguments["mount_pitch"]) == pytest.approx(0.7853981633974483)
