import importlib.util
import math
from pathlib import Path

import pytest
from nav_msgs.msg import Odometry


def load_module():
    path = Path(__file__).parents[1] / "scripts" / "base_link_odometry.py"
    spec = importlib.util.spec_from_file_location("base_link_odometry", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_downward_device_pitch_restores_level_base_and_forward_velocity():
    module = load_module()
    q_base_sensor = module.quaternion_from_rpy(0.0, math.pi / 4.0, 0.0)
    q_sensor_base = module.quaternion_conjugate(q_base_sensor)

    q_odom_base = module.normalize_quaternion(
        module.quaternion_multiply(q_base_sensor, q_sensor_base)
    )
    assert q_odom_base == pytest.approx((0.0, 0.0, 0.0, 1.0))

    forward_in_sensor = module.rotate_vector(q_sensor_base, (1.0, 0.0, 0.0))
    forward_in_base = module.rotate_vector(q_base_sensor, forward_in_sensor)
    assert forward_in_base == pytest.approx((1.0, 0.0, 0.0))


def test_odometry_callback_publishes_base_link_pose():
    module = load_module()
    mount = module.quaternion_from_rpy(0.0, math.pi / 4.0, 0.0)

    class Publisher:
        message = None

        def publish(self, message):
            self.message = message

    adapter = type("Adapter", (), {})()
    adapter.base_frame = "base_link"
    adapter.q_base_sensor = mount
    adapter.q_sensor_base = module.quaternion_conjugate(mount)
    adapter.publisher = Publisher()

    raw = Odometry()
    raw.child_frame_id = "imu"
    raw.pose.pose.orientation.x = mount[0]
    raw.pose.pose.orientation.y = mount[1]
    raw.pose.pose.orientation.z = mount[2]
    raw.pose.pose.orientation.w = mount[3]
    raw.twist.twist.linear.x, raw.twist.twist.linear.y, raw.twist.twist.linear.z = (
        module.rotate_vector(adapter.q_sensor_base, (1.0, 0.0, 0.0))
    )
    raw.twist.covariance[0] = 1.0

    module.BaseLinkOdometry.odometry_callback(adapter, raw)

    corrected = adapter.publisher.message
    assert corrected.child_frame_id == "base_link"
    assert (
        corrected.pose.pose.orientation.x,
        corrected.pose.pose.orientation.y,
        corrected.pose.pose.orientation.z,
        corrected.pose.pose.orientation.w,
    ) == pytest.approx((0.0, 0.0, 0.0, 1.0))
    assert (
        corrected.twist.twist.linear.x,
        corrected.twist.twist.linear.y,
        corrected.twist.twist.linear.z,
    ) == pytest.approx((1.0, 0.0, 0.0))
    assert corrected.twist.covariance[0] == pytest.approx(0.5)
    assert corrected.twist.covariance[2] == pytest.approx(-0.5)
    assert corrected.twist.covariance[12] == pytest.approx(-0.5)
    assert corrected.twist.covariance[14] == pytest.approx(0.5)
