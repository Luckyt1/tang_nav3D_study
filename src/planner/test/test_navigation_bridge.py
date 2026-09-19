"""Exercise navigation gating against cached TF without publishing actuator commands."""

from copy import deepcopy
import importlib.util
import itertools
import math
import os
from pathlib import Path
import time

import pytest
import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry, Path as NavPath
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from relocalization.msg import TrackingStatus
from std_msgs.msg import Bool
from tf2_msgs.msg import TFMessage


def load_bridge():
    source = Path(__file__).parents[1] / 'scripts' / 'relocalization_bridge.py'
    spec = importlib.util.spec_from_file_location('navigation_bridge_under_test', source)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def make_pose(frame, point):
    result = PoseStamped()
    result.header.frame_id = frame
    result.header.stamp.sec = 20
    result.pose.position.x, result.pose.position.y, result.pose.position.z = point
    result.pose.orientation.w = 1.0
    return result


def make_path():
    result = NavPath()
    result.header.frame_id = 'map'
    result.header.stamp.sec = 20
    result.header.stamp.nanosec = 123
    result.poses = [make_pose('', (2.0, 0.0, 0.5)), make_pose('map', (1.0, 1.0, 0.7))]
    return result


def make_odometry():
    result = Odometry()
    result.header.frame_id = 'odom'
    result.header.stamp.sec = 20
    result.child_frame_id = 'lidar'
    result.pose.pose = make_pose('odom', (1.0, 2.0, 0.3)).pose
    result.twist.twist.linear.x = 0.4
    return result


def map_from_odom():
    transform = TransformStamped()
    transform.header.frame_id = 'map'
    transform.child_frame_id = 'odom'
    transform.header.stamp.sec = 10
    transform.transform.translation.x = 2.0
    transform.transform.translation.y = -1.0
    transform.transform.translation.z = 0.5
    transform.transform.rotation.z = math.sin(math.pi / 4.0)
    transform.transform.rotation.w = math.cos(math.pi / 4.0)
    return transform


def assert_position(point, expected):
    assert (point.x, point.y, point.z) == pytest.approx(expected, abs=1e-6)


class BridgeHarness:
    def __init__(self, module):
        self.bridge = module.RelocalizationBridge()
        self.peer = Node('navigation_bridge_test_driver')
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.bridge)
        self.executor.add_node(self.peer)
        self.odometry = []
        self.paths = []
        self.goals = []
        self.enabled = []
        self.path_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.peer.create_subscription(Odometry, 'map_odom', self.odometry.append,
                                      qos_profile_sensor_data)
        self.peer.create_subscription(NavPath, 'output_path', self.paths.append, self.path_qos)
        self.peer.create_subscription(PoseStamped, 'output_goal', self.goals.append, 10)
        self.peer.create_subscription(Bool, 'navigation/enabled', self.enabled.append,
                                      qos_profile_sensor_data)
        self.status_pub = self.peer.create_publisher(
            TrackingStatus, '/relocalization/tracking_status', qos_profile_sensor_data)
        self.odom_pub = self.peer.create_publisher(Odometry, 'input_odom', qos_profile_sensor_data)
        self.path_pub = self.peer.create_publisher(NavPath, 'input_path', self.path_qos)
        self.goal_pub = self.peer.create_publisher(PoseStamped, 'input_goal', 10)
        self.replan_pub = self.peer.create_publisher(PoseStamped, 'input_replan_goal', 10)
        self.tf_pub = self.peer.create_publisher(TFMessage, '/tf', qos_profile_sensor_data)

    def start(self):
        publishers = (self.status_pub, self.odom_pub, self.path_pub,
                      self.goal_pub, self.replan_pub, self.tf_pub)
        self.wait(lambda: all(pub.get_subscription_count() > 0 for pub in publishers))
        message = TFMessage(transforms=[map_from_odom()])
        self.wait(lambda: self.cached_tf(), publish=lambda: self.tf_pub.publish(message))
        self.wait(lambda: self.enabled and not self.enabled[-1].data and self.paths)

    def close(self):
        self.executor.remove_node(self.bridge)
        self.executor.remove_node(self.peer)
        self.bridge.destroy_node()
        self.peer.destroy_node()
        self.executor.shutdown()

    def cached_tf(self):
        return self.bridge.tf_buffer.can_transform('map', 'odom', Time())

    def wait(self, condition, timeout=3.0, publish=None):
        deadline = time.monotonic() + timeout
        while not condition() and time.monotonic() < deadline:
            if publish is not None:
                publish()
            self.executor.spin_once(timeout_sec=0.02)
        assert condition(), 'timed out waiting for bridge messages'

    def spin(self, duration=0.15):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=0.02)

    def status(self, valid=True, state='tracking', map_frame='map', odom_frame='odom'):
        message = TrackingStatus()
        message.header.frame_id = map_frame
        message.odom_frame_id = odom_frame
        message.sensor_frame_id = 'lidar'
        message.valid = valid
        message.status = state
        self.status_pub.publish(message)

    def enable(self):
        self.status()
        self.wait(lambda: self.enabled and self.enabled[-1].data)

    def publish_inputs(self):
        self.odom_pub.publish(make_odometry())
        self.path_pub.publish(make_path())
        self.goal_pub.publish(make_pose('odom', (1.0, -1.0, 0.2)))
        self.replan_pub.publish(make_pose('map', (-2.0, 3.0, 1.0)))

    def assert_blocked(self):
        self.spin(0.05)
        counts = (len(self.odometry), len(self.paths), len(self.goals))
        self.publish_inputs()
        self.spin()
        assert (len(self.odometry), len(self.paths), len(self.goals)) == counts
        assert not self.enabled[-1].data
        assert self.cached_tf(), 'blocking must hold even while the old TF remains cached'


_domains = itertools.count()


@pytest.fixture
def bridge(monkeypatch, tmp_path):
    monkeypatch.setenv('ROS_DOMAIN_ID', str(200 + (os.getpid() + next(_domains)) % 20))
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path))
    rclpy.init(args=[])
    harness = None
    try:
        harness = BridgeHarness(load_bridge())
        harness.start()
        yield harness
    finally:
        if harness is not None:
            harness.close()
        rclpy.shutdown()


def test_valid_tracking_converts_odometry_path_and_both_goal_inputs(bridge):
    bridge.enable()
    bridge.publish_inputs()
    bridge.wait(lambda: bridge.odometry and any(path.poses for path in bridge.paths)
                and len(bridge.goals) == 2)
    odometry = bridge.odometry[-1]
    assert odometry.header.frame_id == 'map'
    assert odometry.header.stamp == make_odometry().header.stamp
    assert odometry.child_frame_id == 'lidar'
    assert odometry.twist == make_odometry().twist
    assert_position(odometry.pose.pose.position, (0.0, 0.0, 0.8))
    assert odometry.pose.pose.orientation.z == pytest.approx(math.sin(math.pi / 4.0))
    path = next(path for path in reversed(bridge.paths) if path.poses)
    assert path.header.frame_id == 'odom'
    assert path.header.stamp == make_path().header.stamp
    for pose, expected in zip(path.poses, [(1.0, 0.0, 0.0), (2.0, 1.0, 0.2)]):
        assert pose.header.frame_id == 'odom'
        assert pose.header.stamp == path.header.stamp
        assert_position(pose.pose.position, expected)
    assert all(goal.header.frame_id == 'map' for goal in bridge.goals)
    positions = sorted((goal.pose.position.x, goal.pose.position.y, goal.pose.position.z)
                       for goal in bridge.goals)
    assert positions[0] == pytest.approx((-2.0, 3.0, 1.0))
    assert positions[1] == pytest.approx((3.0, 0.0, 0.7))


def test_invalid_tracking_blocks_cached_tf_and_clears_retained_path_once(bridge):
    bridge.enable()
    bridge.path_pub.publish(make_path())
    bridge.wait(lambda: any(path.poses for path in bridge.paths))
    count_before = len(bridge.paths)
    bridge.status(valid=False, state='odometry_stale')
    bridge.wait(lambda: not bridge.enabled[-1].data and len(bridge.paths) > count_before)
    assert len(bridge.paths) == count_before + 1
    assert not bridge.paths[-1].poses
    assert bridge.paths[-1].header.frame_id == 'odom'
    bridge.status(valid=False, state='odometry_stale')
    bridge.assert_blocked()

    # A planner which subscribes after invalidation must receive the clearing sample.
    late_paths = []
    subscription = bridge.peer.create_subscription(
        NavPath, 'output_path', late_paths.append, bridge.path_qos)
    try:
        bridge.wait(lambda: bool(late_paths))
        assert not late_paths[-1].poses
        assert late_paths[-1].header.frame_id == 'odom'
    finally:
        bridge.peer.destroy_subscription(subscription)


def test_stopped_tracking_heartbeat_disables_forwarding_then_valid_status_recovers(bridge):
    bridge.enable()
    bridge.path_pub.publish(make_path())
    bridge.wait(lambda: any(path.poses for path in bridge.paths))
    path_count = len(bridge.paths)
    bridge.wait(lambda: not bridge.enabled[-1].data and len(bridge.paths) > path_count,
                timeout=2.0)
    assert not bridge.paths[-1].poses
    bridge.assert_blocked()

    counts = (len(bridge.odometry), len(bridge.paths), len(bridge.goals))
    bridge.enable()
    bridge.publish_inputs()
    bridge.wait(lambda: len(bridge.odometry) > counts[0] and len(bridge.paths) > counts[1]
                and len(bridge.goals) >= counts[2] + 2)
    assert_position(bridge.odometry[-1].pose.pose.position, (0.0, 0.0, 0.8))
    assert bridge.paths[-1].poses
    assert all(goal.header.frame_id == 'map' for goal in bridge.goals[-2:])


@pytest.mark.parametrize('status', [
    {'state': 'fix_expired'},
    {'map_frame': 'other_map'},
    {'odom_frame': 'other_odom'},
])
def test_inconsistent_tracking_status_never_enables_navigation(bridge, status):
    bridge.status(**status)
    bridge.assert_blocked()


def test_valid_tracking_rejects_wrong_input_frames_and_mixed_path_frames(bridge):
    bridge.enable()
    counts = (len(bridge.odometry), len(bridge.paths), len(bridge.goals))
    wrong_odom = make_odometry()
    wrong_odom.header.frame_id = 'other_odom'
    bridge.odom_pub.publish(wrong_odom)
    wrong_path = make_path()
    wrong_path.header.frame_id = 'odom'
    bridge.path_pub.publish(wrong_path)
    mixed_path = make_path()
    mixed_path.poses[0].header.frame_id = 'odom'
    bridge.path_pub.publish(mixed_path)
    bridge.goal_pub.publish(make_pose('camera', (1.0, 2.0, 3.0)))
    bridge.replan_pub.publish(make_pose('base_link', (1.0, 2.0, 3.0)))
    bridge.spin()
    assert (len(bridge.odometry), len(bridge.paths), len(bridge.goals)) == counts
    assert bridge.enabled[-1].data


def test_transform_helpers_leave_source_messages_and_headers_unchanged():
    module = load_bridge()
    path = make_path()
    original_path = deepcopy(path)
    transform = TransformStamped()
    transform.header.frame_id = 'odom'
    transform.child_frame_id = 'map'
    transform.transform.rotation.w = 1.0
    transformed_path = module.transform_path(path, transform)
    assert path == original_path
    assert transformed_path.header is not path.header
    assert transformed_path.poses[0].header.frame_id == 'odom'
    assert path.poses[0].header.frame_id == ''

    odometry = make_odometry()
    original_odometry = deepcopy(odometry)
    transformed_odometry = module.transform_odometry(odometry, map_from_odom())
    assert odometry == original_odometry
    assert transformed_odometry.header is not odometry.header
