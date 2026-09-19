import importlib.util
import math
from pathlib import Path

import pytest
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry, Path as NavPath


def load_module():
    path = Path(__file__).parents[1] / "scripts" / "relocalization_bridge.py"
    spec = importlib.util.spec_from_file_location("relocalization_bridge", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def odom_to_map_transform():
    transform = TransformStamped()
    transform.header.frame_id = "map"
    transform.child_frame_id = "odom"
    transform.transform.translation.x = -2.0
    transform.transform.translation.y = 1.0
    transform.transform.rotation.z = math.sin(-math.pi / 4.0)
    transform.transform.rotation.w = math.cos(-math.pi / 4.0)
    return transform


def test_odometry_is_transformed_into_map_frame():
    module = load_module()
    odom = Odometry()
    odom.header.frame_id = "odom"
    odom.child_frame_id = "base_link"
    odom.pose.pose.position.x = 1.0
    odom.pose.pose.orientation.w = 1.0

    transformed = module.transform_odometry(odom, odom_to_map_transform())

    assert transformed.header.frame_id == "map"
    assert transformed.child_frame_id == "base_link"
    assert transformed.pose.pose.position.x == pytest.approx(-2.0)
    assert transformed.pose.pose.position.y == pytest.approx(0.0)


def test_map_path_is_transformed_back_into_odom_frame():
    module = load_module()
    transform = TransformStamped()
    transform.header.frame_id = "odom"
    transform.child_frame_id = "map"
    transform.transform.translation.x = 1.0
    transform.transform.translation.y = 2.0
    transform.transform.rotation.z = math.sin(math.pi / 4.0)
    transform.transform.rotation.w = math.cos(math.pi / 4.0)

    path = NavPath()
    path.header.frame_id = "map"
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.pose.position.x = 1.0
    pose.pose.orientation.w = 1.0
    path.poses.append(pose)

    transformed = module.transform_path(path, transform)

    assert transformed.header.frame_id == "odom"
    assert transformed.poses[0].header.frame_id == "odom"
    assert transformed.poses[0].pose.position.x == pytest.approx(1.0)
    assert transformed.poses[0].pose.position.y == pytest.approx(3.0)


