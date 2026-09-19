"""Integration tests for the ROS 2 BTC relocalization node."""

import math
import os
import shutil
import subprocess
import sys
import tempfile
import time
import unittest
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped
from rclpy.qos import HistoryPolicy
from rclpy.qos import DurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_msgs.msg import TFMessage

from relocalization.msg import CandidateArray
from relocalization.msg import FineResult
from relocalization.msg import TrackingStatus


NODE_EXECUTABLE = sys.argv.pop(1)
FIXTURE_EXECUTABLE = sys.argv.pop(1)

INPUT_TOPIC = '/relocalization_test/input'
CANDIDATES_TOPIC = '/relocalization/candidates'
POSES_TOPIC = '/relocalization/candidate_poses'
QUERY_TOPIC = '/relocalization/query_cloud'
GLOBAL_MAP_TOPIC = '/relocalization/global_map'
FINE_TOPIC = '/relocalization/fine_result'
REFINED_POSE_TOPIC = '/relocalization/pose'
ALIGNED_TOPIC = '/relocalization/aligned_cloud'
TRACKED_POSE_TOPIC = '/relocalization/tracked_pose'
TRACKING_STATUS_TOPIC = '/relocalization/tracking_status'
ODOM_FRAME = 'test_odom'
SENSOR_FRAME = 'test_lidar'
MAP_FRAME = 'test_map'


def yaw_quaternion(yaw):
    half = yaw / 2.0
    return (0.0, 0.0, math.sin(half), math.cos(half))


def make_transform(parent, child, stamp_sec, translation, yaw=0.0, nanosec=123456789):
    transform = TransformStamped()
    transform.header.frame_id = parent
    transform.header.stamp.sec = stamp_sec
    transform.header.stamp.nanosec = nanosec
    transform.child_frame_id = child
    transform.transform.translation.x = float(translation[0])
    transform.transform.translation.y = float(translation[1])
    transform.transform.translation.z = float(translation[2])
    x, y, z, w = yaw_quaternion(yaw)
    transform.transform.rotation.x = x
    transform.transform.rotation.y = y
    transform.transform.rotation.z = z
    transform.transform.rotation.w = w
    return transform


def make_cloud(frame_id, stamp_sec, points, nanosec=123456789):
    header = Header()
    header.frame_id = frame_id
    header.stamp.sec = stamp_sec
    header.stamp.nanosec = nanosec
    return point_cloud2.create_cloud_xyz32(header, points)


def read_xyz(cloud):
    return [
        tuple(float(value) for value in row)
        for row in point_cloud2.read_points(
            cloud, field_names=('x', 'y', 'z'), skip_nans=False)
    ]


def transform_point(transform, point):
    q = transform.rotation
    x, y, z = point
    tx, ty, tz = (2.0 * (q.y * z - q.z * y),
                  2.0 * (q.z * x - q.x * z),
                  2.0 * (q.x * y - q.y * x))
    return (
        x + q.w * tx + q.y * tz - q.z * ty + transform.translation.x,
        y + q.w * ty + q.z * tx - q.x * tz + transform.translation.y,
        z + q.w * tz + q.x * ty - q.y * tx + transform.translation.z,
    )


def assert_points_include(testcase, actual_points, expected_points, places=4):
    unmatched = list(actual_points)
    for expected in expected_points:
        match_index = None
        for index, actual in enumerate(unmatched):
            if all(round(actual[i] - expected[i], places) == 0 for i in range(3)):
                match_index = index
                break
        testcase.assertIsNotNone(
            match_index,
            f'expected point {expected} was not present in {actual_points}')
        unmatched.pop(match_index)


def run_fixture(mode=None):
    root = tempfile.mkdtemp(prefix='relocalization_fixture_root_')
    completed = subprocess.run(
        [FIXTURE_EXECUTABLE, root] + ([mode] if mode else []),
        check=True,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    lines = [line.strip() for line in completed.stdout.splitlines() if line.strip()]
    if not lines:
        raise AssertionError('fixture did not print a snapshot directory')
    snapshot = Path(lines[-1])
    if not snapshot.is_dir():
        raise AssertionError(f'fixture snapshot is not a directory: {snapshot}')
    return root, snapshot


class RelocalizationNodeIntegrationTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        os.environ['ROS_DOMAIN_ID'] = str(150 + os.getpid() % 40)
        os.environ['ROS_LOCALHOST_ONLY'] = '1'
        cls.logs = tempfile.TemporaryDirectory(prefix='relocalization_node_ros_')
        os.environ['ROS_LOG_DIR'] = cls.logs.name
        cls.fixture_root, cls.snapshot = run_fixture()
        cls.room_root, cls.room_snapshot = run_fixture('room')
        cls.room_points = [
            tuple(map(float, line.split()))
            for line in (Path(cls.room_root) / 'query.xyz').read_text().splitlines()
        ]
        rclpy.init()
        cls.node = rclpy.create_node('relocalization_node_integration_test')
        cls.candidates = {}
        cls.poses = {}
        cls.query_clouds = {}
        cls.fine_results = {}
        cls.fine_messages = []
        cls.aligned_clouds = {}
        cls.refined_poses = []
        cls.map_transforms = []
        cls.tracked_poses = []
        cls.tracking_statuses = []

        def record_fine_result(message):
            cls.fine_results[message.header.stamp.sec] = message
            cls.fine_messages.append(message)

        cls.node.create_subscription(
            CandidateArray,
            CANDIDATES_TOPIC,
            lambda msg: cls.candidates.__setitem__(msg.header.stamp.sec, msg),
            QoSProfile(depth=20),
        )
        cls.node.create_subscription(
            PoseArray,
            POSES_TOPIC,
            lambda msg: cls.poses.__setitem__(msg.header.stamp.sec, msg),
            QoSProfile(depth=20),
        )
        cls.node.create_subscription(
            PointCloud2,
            QUERY_TOPIC,
            lambda msg: cls.query_clouds.__setitem__(msg.header.stamp.sec, msg),
            QoSProfile(depth=20),
        )
        cls.node.create_subscription(
            FineResult, FINE_TOPIC, record_fine_result, QoSProfile(depth=20),
        )
        cls.node.create_subscription(
            PointCloud2, ALIGNED_TOPIC,
            lambda msg: cls.aligned_clouds.__setitem__(msg.header.stamp.sec, msg),
            QoSProfile(depth=20),
        )
        cls.node.create_subscription(
            PoseStamped, REFINED_POSE_TOPIC, cls.refined_poses.append, QoSProfile(depth=20),
        )
        cls.node.create_subscription(
            PoseStamped, TRACKED_POSE_TOPIC, cls.tracked_poses.append, QoSProfile(depth=100),
        )
        cls.node.create_subscription(
            TrackingStatus, TRACKING_STATUS_TOPIC, cls.tracking_statuses.append,
            QoSProfile(depth=100),
        )
        cls.node.create_subscription(
            TFMessage, '/tf',
            lambda msg: cls.map_transforms.extend(
                transform for transform in msg.transforms
                if transform.header.frame_id == MAP_FRAME
                and transform.child_frame_id == ODOM_FRAME),
            QoSProfile(depth=50),
        )
        cls.cloud_pub = cls.node.create_publisher(
            PointCloud2,
            INPUT_TOPIC,
            QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=20,
                reliability=ReliabilityPolicy.BEST_EFFORT,
            ),
        )
        cls.tf_pub = cls.node.create_publisher(TFMessage, '/tf', QoSProfile(depth=50))

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()
        cls.logs.cleanup()
        shutil.rmtree(cls.fixture_root, ignore_errors=True)
        shutil.rmtree(cls.room_root, ignore_errors=True)

    def setUp(self):
        type(self).candidates.clear()
        type(self).poses.clear()
        type(self).query_clouds.clear()
        type(self).fine_results.clear()
        type(self).fine_messages.clear()
        type(self).aligned_clouds.clear()
        type(self).refined_poses.clear()
        type(self).map_transforms.clear()
        type(self).tracked_poses.clear()
        type(self).tracking_statuses.clear()
        room_test = self._testMethodName.startswith(('test_fine_', 'test_tracking_'))
        snapshot = self.room_snapshot if room_test else self.snapshot
        extra_parameters = []
        if self._testMethodName == 'test_fine_pose_remains_available_when_tf_disabled':
            extra_parameters = ['-p', 'fine.publish_tf:=false']
        if self._testMethodName == 'test_tracking_expires_fix_while_odometry_keeps_advancing':
            extra_parameters = ['-p', 'tracking.fix_timeout:=1.0']
        if self._testMethodName == 'test_tracking_best_effort_tf_with_late_static_extrinsic':
            self.tf_pub = self.node.create_publisher(
                TFMessage, '/tf', QoSProfile(
                    depth=100, reliability=ReliabilityPolicy.BEST_EFFORT))
            self.addCleanup(self.node.destroy_publisher, self.tf_pub)
            self.imu_to_sensor = ((0.15, -0.08, 0.12), 0.1)
            static_publisher = self.node.create_publisher(
                TFMessage, '/tf_static', QoSProfile(
                    depth=1, reliability=ReliabilityPolicy.RELIABLE,
                    durability=DurabilityPolicy.TRANSIENT_LOCAL))
            self.addCleanup(self.node.destroy_publisher, static_publisher)
            # Publish once before the listener exists: only durable replay can supply it.
            translation, yaw = self.imu_to_sensor
            static_publisher.publish(TFMessage(transforms=[make_transform(
                'test_imu_late', SENSOR_FRAME, 1, translation, yaw)]))
        self.process = subprocess.Popen(
            [
                NODE_EXECUTABLE,
                '--ros-args',
                '-p', f'map.directory:={snapshot}',
                '-p', f'input_topic:={INPUT_TOPIC}',
                '-p', f'odom_frame:={ODOM_FRAME}',
                '-p', f'sensor_frame:={SENSOR_FRAME}',
                '-p', f'map_frame:={MAP_FRAME}',
                '-p', 'tf_timeout:=0.2',
                '-p', f'query.frames:={1 if room_test else 2}',
                '-p', 'query.max_duration:=30.0',
            ] + extra_parameters,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        self.wait_for(self.ros_graph_ready)

    def tearDown(self):
        self.process.terminate()
        try:
            self.process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            self.process.kill()
            self.process.wait(timeout=5)

    @classmethod
    def ros_graph_ready(cls):
        return (
            cls.cloud_pub.get_subscription_count() > 0
            and cls.tf_pub.get_subscription_count() > 0
            and cls.node.count_publishers(CANDIDATES_TOPIC) > 0
            and cls.node.count_publishers(POSES_TOPIC) > 0
            and cls.node.count_publishers(QUERY_TOPIC) > 0
            and cls.node.count_publishers(FINE_TOPIC) > 0
            and cls.node.count_publishers(REFINED_POSE_TOPIC) > 0
            and cls.node.count_publishers(ALIGNED_TOPIC) > 0
            and cls.node.count_publishers(TRACKED_POSE_TOPIC) > 0
            and cls.node.count_publishers(TRACKING_STATUS_TOPIC) > 0
        )

    def wait_for(self, predicate, timeout=5.0, publish=None):
        deadline = time.monotonic() + timeout
        while not predicate() and time.monotonic() < deadline:
            self.assertIsNone(self.process.poll(), 'relocalization_node exited')
            if publish is not None:
                publish()
            rclpy.spin_once(self.node, timeout_sec=0.05)
        self.assertTrue(predicate(), 'timed out waiting for ROS graph or messages')

    def assert_no_candidate_message(self, publish, timeout=0.35):
        count_before = len(self.candidates)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.assertIsNone(self.process.poll(), 'relocalization_node exited')
            publish()
            rclpy.spin_once(self.node, timeout_sec=0.05)
        self.assertEqual(len(self.candidates), count_before)

    def odometry_message(self, stamp_sec, translation, yaw, nanosec=123456789):
        child = SENSOR_FRAME
        if hasattr(self, 'imu_to_sensor'):
            # Preserve the requested odom->lidar pose through a nontrivial static extrinsic.
            offset, extrinsic_yaw = self.imu_to_sensor
            child = 'test_imu_late'
            yaw -= extrinsic_yaw
            c, s = math.cos(yaw), math.sin(yaw)
            translation = (translation[0] - c * offset[0] + s * offset[1],
                           translation[1] - s * offset[0] - c * offset[1],
                           translation[2] - offset[2])
        return TFMessage(transforms=[make_transform(
            ODOM_FRAME, child, stamp_sec, translation, yaw, nanosec=nanosec)])

    def publish_tf_and_cloud_until_candidates(self, stamp_sec, points, frame_id=SENSOR_FRAME,
                                              translation=(0.0, 0.0, 0.0), yaw=0.0,
                                              timeout=5.0):
        cloud = make_cloud(frame_id, stamp_sec, points)
        tf_msg = self.odometry_message(stamp_sec, translation, yaw)
        self.wait_for(
            lambda: stamp_sec in self.candidates,
            timeout=timeout,
            publish=lambda: (self.tf_pub.publish(tf_msg), self.cloud_pub.publish(cloud)),
        )
        self.wait_for(lambda: stamp_sec in self.poses and stamp_sec in self.query_clouds)
        return self.candidates[stamp_sec], self.poses[stamp_sec], self.query_clouds[stamp_sec]

    def publish_tf_and_cloud_for_window(self, stamp_sec, points, frame_id=SENSOR_FRAME,
                                        translation=(0.0, 0.0, 0.0), yaw=0.0, timeout=0.25):
        cloud = make_cloud(frame_id, stamp_sec, points)
        tf_msg = self.odometry_message(stamp_sec, translation, yaw)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.assertIsNone(self.process.poll(), 'relocalization_node exited')
            self.tf_pub.publish(tf_msg)
            self.cloud_pub.publish(cloud)
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def spin_for(self, duration=0.35):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            self.assertIsNone(self.process.poll(), 'relocalization_node exited')
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def assert_rigid_pose_near(self, translation, rotation, expected_translation, expected_yaw):
        for actual, expected in zip(
                (translation.x, translation.y, translation.z), expected_translation):
            self.assertAlmostEqual(actual, expected, delta=0.08)
        quaternion = (rotation.x, rotation.y, rotation.z, rotation.w)
        norm = math.sqrt(sum(value * value for value in quaternion))
        self.assertAlmostEqual(norm, 1.0, places=5)
        dot = abs(sum(a * b for a, b in zip(quaternion, yaw_quaternion(expected_yaw))))
        angle = 2.0 * math.acos(min(1.0, dot / norm))
        self.assertLess(angle, 0.05)

    def assert_room_fine_success(self, stamp_sec, expect_tf=True):
        # Saved map and live odometry deliberately have distinct translations and yaw.
        odom_translation = (1.2, -0.4, 0.3)
        odom_yaw = -0.4
        candidates, _, query_cloud = self.publish_tf_and_cloud_until_candidates(
            stamp_sec, self.room_points, translation=odom_translation, yaw=odom_yaw,
            timeout=15.0)
        self.wait_for(
            lambda: stamp_sec in self.fine_results and stamp_sec in self.aligned_clouds,
            timeout=15.0)
        fine = self.fine_results[stamp_sec]
        self.assertTrue(fine.success, fine.failure_reason)
        self.assertTrue(fine.converged)
        self.assertEqual(fine.failure_reason, '')
        self.assertEqual(fine.submap_id, 0)
        self.assertGreaterEqual(fine.inliers, 50)
        self.assertGreaterEqual(fine.overlap, 0.6)
        self.assertLessEqual(fine.rmse, 0.15)
        self.assertEqual(fine.header.frame_id, MAP_FRAME)
        self.assertEqual(fine.odom_frame_id, ODOM_FRAME)
        self.assertEqual(fine.query_frame_id, SENSOR_FRAME)
        self.assertEqual(fine.header.stamp, candidates.header.stamp)
        self.assertEqual(fine.header.stamp.sec, stamp_sec)
        self.assertEqual(fine.header.stamp.nanosec, 123456789)
        self.assert_rigid_pose_near(
            fine.map_from_query.position, fine.map_from_query.orientation,
            (4.0, -2.0, 0.5), 0.6)

        map_from_odom_yaw = 0.6 - odom_yaw
        c, s = math.cos(map_from_odom_yaw), math.sin(map_from_odom_yaw)
        ox, oy, oz = odom_translation
        expected_translation = (4.0 - c * ox + s * oy,
                                -2.0 - s * ox - c * oy, 0.5 - oz)
        self.assert_rigid_pose_near(
            fine.map_from_odom.translation, fine.map_from_odom.rotation,
            expected_translation, map_from_odom_yaw)
        self.wait_for(lambda: any(p.header.stamp.sec == stamp_sec for p in self.refined_poses))
        pose = next(p for p in self.refined_poses if p.header.stamp.sec == stamp_sec)
        self.assertEqual(pose.header, fine.header)
        self.assertEqual(pose.pose, fine.map_from_query)
        if expect_tf:
            self.wait_for(lambda: any(t.header.stamp.sec == stamp_sec for t in self.map_transforms))
            transform = next(t for t in self.map_transforms if t.header.stamp.sec == stamp_sec)
            self.assertEqual(transform.header, fine.header)
            self.assertEqual(transform.child_frame_id, ODOM_FRAME)
            self.assertEqual(transform.transform, fine.map_from_odom)

        aligned = self.aligned_clouds[stamp_sec]
        self.assertEqual(aligned.header, fine.header)
        self.assertGreater(aligned.width * aligned.height, 50)
        self.assertEqual(aligned.width * aligned.height, candidates.query_point_count)
        odom_points, aligned_points = read_xyz(query_cloud), read_xyz(aligned)
        self.assertEqual(len(odom_points), len(aligned_points))
        for index in range(0, len(odom_points), max(1, len(odom_points) // 20)):
            expected = transform_point(fine.map_from_odom, odom_points[index])
            for actual, wanted in zip(aligned_points[index], expected):
                self.assertAlmostEqual(actual, wanted, delta=1e-4)
        return fine

    def wait_for_tracking_status(self, status, stamp=None, publish=None, timeout=3.0):
        def matches(message):
            return message.status == status and (stamp is None or message.header.stamp == stamp)

        self.wait_for(lambda: any(matches(message) for message in self.tracking_statuses),
                      timeout=timeout, publish=publish)
        return next(message for message in reversed(self.tracking_statuses) if matches(message))

    def assert_tracking_update(self, fix, stamp_sec, nanosec, translation, yaw, expect_tf=True):
        tf_message = self.odometry_message(stamp_sec, translation, yaw, nanosec=nanosec)
        stamp = tf_message.transforms[0].header.stamp
        self.wait_for(
            lambda: any(p.header.stamp == stamp for p in self.tracked_poses),
            publish=lambda: self.tf_pub.publish(tf_message),
        )
        tracked = next(p for p in self.tracked_poses if p.header.stamp == stamp)
        self.assertEqual(tracked.header.frame_id, MAP_FRAME)
        expected_translation = transform_point(fix.map_from_odom, translation)
        self.assert_rigid_pose_near(
            tracked.pose.position, tracked.pose.orientation, expected_translation, 1.0 + yaw)
        status = self.wait_for_tracking_status('tracking', stamp)
        self.assertTrue(status.valid)
        self.assertEqual(status.header.frame_id, MAP_FRAME)
        self.assertEqual(status.odom_frame_id, ODOM_FRAME)
        self.assertEqual(status.sensor_frame_id, SENSOR_FRAME)
        self.assertEqual(status.fix_stamp, fix.header.stamp)
        message_age = (stamp.sec - fix.header.stamp.sec +
                       (stamp.nanosec - fix.header.stamp.nanosec) * 1e-9)
        self.assertGreaterEqual(status.fix_age + 1e-6, message_age)
        if expect_tf:
            self.wait_for(lambda: any(t.header.stamp == stamp for t in self.map_transforms))
            transform = next(t for t in self.map_transforms if t.header.stamp == stamp)
            self.assertEqual(transform.header, tracked.header)
            self.assertEqual(transform.child_frame_id, ODOM_FRAME)
            self.assertEqual(transform.transform, fix.map_from_odom)
        return tf_message

    def test_fine_success_then_failed_query_clears_alignment_without_republishing_fix(self):
        self.assert_room_fine_success(100)
        self.spin_for(0.1)
        pose_count, tf_count = len(self.refined_poses), len(self.map_transforms)
        candidates, _, _ = self.publish_tf_and_cloud_until_candidates(
            131, [(1.0, 0.0, 0.0), (1.4, 0.2, 0.1), (1.8, -0.2, 0.0)])
        self.wait_for(lambda: 131 in self.fine_results and 131 in self.aligned_clouds)
        self.assertEqual(candidates.status, 'no_descriptors')
        self.assertFalse(candidates.candidates)
        self.assertFalse(self.fine_results[131].success)
        self.assertEqual(self.fine_results[131].failure_reason, 'no_candidates')
        self.assertEqual(self.fine_results[131].header.stamp, candidates.header.stamp)
        aligned = self.aligned_clouds[131]
        self.assertEqual(aligned.header, self.fine_results[131].header)
        self.assertEqual(aligned.width * aligned.height, 0)
        self.assertFalse(aligned.data)
        self.spin_for()
        # Lists, not dictionaries: repeated publication of an old stamp must also fail.
        self.assertEqual(len(self.refined_poses), pose_count)
        self.assertEqual(len(self.map_transforms), tf_count)

    def test_fine_pose_remains_available_when_tf_disabled(self):
        fine = self.assert_room_fine_success(200, expect_tf=False)
        self.assert_tracking_update(
            fine, 201, 123456789, (1.4, -0.3, 0.35), -0.2, expect_tf=False)
        self.spin_for()
        self.assertFalse(self.map_transforms)

    def test_tracking_propagates_only_new_odometry_and_detects_frozen_input(self):
        fine = self.assert_room_fine_success(300)
        fine_count, pose_count = len(self.fine_messages), len(self.refined_poses)
        for index in range(1, 5):
            duplicate = self.assert_tracking_update(
                fine, 300, 123456789 + index * 100000000,
                (1.2 + index * 0.1, -0.4 + index * 0.05, 0.3 + index * 0.02),
                -0.4 + index * 0.1)
        self.spin_for(0.1)
        tracked_count, tf_count = len(self.tracked_poses), len(self.map_transforms)
        self.assertGreaterEqual(tracked_count, 5)
        for messages in (self.tracked_poses, self.map_transforms):
            stamps = [(message.header.stamp.sec, message.header.stamp.nanosec)
                      for message in messages]
            self.assertEqual(len(stamps), len(set(stamps)), 'duplicate odometry output stamp')
        # Keep delivering the same TF: lookup succeeds, but odometry has stopped progressing.
        stale = self.wait_for_tracking_status(
            'odometry_stale', duplicate.transforms[0].header.stamp,
            publish=lambda: self.tf_pub.publish(duplicate), timeout=2.5)
        self.assertFalse(stale.valid)
        self.assertEqual(stale.fix_stamp, fine.header.stamp)
        self.spin_for(0.1)
        self.assertEqual(len(self.tracked_poses), tracked_count)
        self.assertEqual(len(self.map_transforms), tf_count)
        self.assertEqual(len(self.fine_messages), fine_count)
        self.assertEqual(len(self.refined_poses), pose_count)

    def test_tracking_expires_fix_while_odometry_keeps_advancing(self):
        fine = self.assert_room_fine_success(400)
        self.assert_tracking_update(fine, 400, 373456789, (1.3, -0.4, 0.3), -0.3)
        self.assert_tracking_update(fine, 400, 623456789, (1.4, -0.3, 0.3), -0.2)
        self.spin_for(0.05)
        tracked_count, tf_count = len(self.tracked_poses), len(self.map_transforms)
        fine_count, pose_count = len(self.fine_messages), len(self.refined_poses)
        for nanosec in (373456789, 623456789):
            odometry = make_transform(
                ODOM_FRAME, SENSOR_FRAME, 401, (1.5, -0.2, 0.3), -0.1, nanosec=nanosec)
            message = TFMessage(transforms=[odometry])
            expired = self.wait_for_tracking_status(
                'fix_expired', odometry.header.stamp,
                publish=lambda: self.tf_pub.publish(message))
            self.assertFalse(expired.valid)
            self.assertEqual(expired.fix_stamp, fine.header.stamp)
            self.assertGreater(expired.fix_age, 1.0)
        self.spin_for(0.1)
        self.assertEqual(len(self.tracked_poses), tracked_count)
        self.assertEqual(len(self.map_transforms), tf_count)
        self.assertEqual(len(self.fine_messages), fine_count)
        self.assertEqual(len(self.refined_poses), pose_count)

    def test_tracking_without_fix_never_publishes_pose_or_map_tf(self):
        for stamp_sec in (500, 501):
            odometry = make_transform(
                ODOM_FRAME, SENSOR_FRAME, stamp_sec, (1.2, -0.4, 0.3), -0.4)
            message = TFMessage(transforms=[odometry])
            waiting = self.wait_for_tracking_status(
                'waiting_for_fix', odometry.header.stamp,
                publish=lambda: self.tf_pub.publish(message))
            self.assertFalse(waiting.valid)
        self.spin_for(0.1)
        self.assertFalse(self.tracked_poses)
        self.assertFalse(self.map_transforms)
        self.assertFalse(self.refined_poses)
        self.assertFalse(self.fine_messages)

    def test_tf_listener_uses_best_effort_dynamic_and_durable_static_qos(self):
        def listeners(topic):
            return [info for info in self.node.get_subscriptions_info_by_topic(topic)
                    if info.node_name == 'btc_relocalization']

        self.wait_for(lambda: listeners('/tf') and listeners('/tf_static'))
        dynamic, static = listeners('/tf'), listeners('/tf_static')
        self.assertEqual(len(dynamic), 1)
        self.assertEqual(len(static), 1)
        self.assertEqual(dynamic[0].qos_profile.reliability, ReliabilityPolicy.BEST_EFFORT)
        self.assertEqual(static[0].qos_profile.reliability, ReliabilityPolicy.RELIABLE)
        self.assertEqual(static[0].qos_profile.durability, DurabilityPolicy.TRANSIENT_LOCAL)

    def test_tracking_best_effort_tf_with_late_static_extrinsic(self):
        self.wait_for(lambda: self.tf_pub.get_subscription_count() > 0)
        fine = self.assert_room_fine_success(600)
        fine_count, pose_count = len(self.fine_messages), len(self.refined_poses)
        self.assert_tracking_update(fine, 600, 373456789, (1.4, -0.3, 0.4), -0.2)
        self.assertEqual(len(self.fine_messages), fine_count)
        self.assertEqual(len(self.refined_poses), pose_count)

    def test_missing_timestamped_tf_does_not_publish(self):
        first = make_cloud(SENSOR_FRAME, 10, [(1.0, 0.0, 0.0)])
        second = make_cloud(SENSOR_FRAME, 11, [(1.4, 0.2, 0.1)])

        self.assert_no_candidate_message(lambda: self.cloud_pub.publish(first))
        self.assert_no_candidate_message(lambda: self.cloud_pub.publish(second))

    def test_global_map_published_without_input_and_retained_for_late_subscribers(self):
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        original = None
        # 第二个订阅者在第一条地图已经收到后才创建，验证晚加入也能取得缓存。
        for _ in range(2):
            maps = []
            subscription = self.node.create_subscription(PointCloud2, GLOBAL_MAP_TOPIC, maps.append, qos)
            try:
                self.wait_for(lambda: bool(maps))
                cloud = maps[0]
                self.assertEqual(cloud.header.frame_id, MAP_FRAME)
                self.assertEqual(cloud.width * cloud.height, 3)
                assert_points_include(self, read_xyz(cloud),
                                      [(1.0, 0.0, 0.0), (1.4, 0.2, 0.1), (1.8, -0.2, 0.0)])
                if original is None:
                    original = cloud
                else:
                    self.assertEqual(cloud.header.stamp, original.header.stamp)
                    self.assertEqual(cloud.data, original.data)
            finally:
                self.node.destroy_subscription(subscription)
        self.assertFalse(self.candidates)  # 无实时点云/TF，也无需 BTC 匹配成功。

    def test_two_valid_frames_publish_no_descriptor_result_and_empty_pose_array(self):
        self.publish_tf_and_cloud_for_window(20, [(1.0, 0.0, 0.0)])

        candidates, poses, query_cloud = self.publish_tf_and_cloud_until_candidates(
            21, [(1.4, 0.2, 0.1)])

        self.assertEqual(candidates.header.frame_id, MAP_FRAME)
        self.assertEqual(candidates.odom_frame_id, ODOM_FRAME)
        self.assertEqual(candidates.query_frame_id, SENSOR_FRAME)
        self.assertEqual(candidates.accumulated_frames, 2)
        self.assertEqual(candidates.status, 'no_descriptors')
        self.assertEqual(candidates.failure_reason, 'no_descriptors')
        self.assertEqual(candidates.best_votes, 0)
        self.assertEqual(candidates.best_length_votes, 0)
        self.assertEqual(candidates.best_inliers, 0)
        self.assertEqual(candidates.required_votes, 5)
        self.assertEqual(candidates.required_inliers, 4)
        self.assertEqual(len(candidates.candidates), 0)
        self.assertGreater(candidates.query_point_count, 0)
        self.assertEqual(candidates.binary_count, 0)
        self.assertEqual(candidates.triangle_count, 0)
        self.assertEqual(poses.header.frame_id, MAP_FRAME)
        self.assertEqual(len(poses.poses), 0)
        self.assertEqual(query_cloud.header.frame_id, ODOM_FRAME)
        assert_points_include(self, read_xyz(query_cloud), [(1.0, 0.0, 0.0), (1.4, 0.2, 0.1)])

    def test_duplicate_stamp_does_not_count_toward_query_window(self):
        self.publish_tf_and_cloud_for_window(30, [(1.0, 0.0, 0.0)])

        self.assert_no_candidate_message(
            lambda: (
                self.tf_pub.publish(TFMessage(transforms=[
                    make_transform(ODOM_FRAME, SENSOR_FRAME, 30, (0.0, 0.0, 0.0))])),
                self.cloud_pub.publish(make_cloud(SENSOR_FRAME, 30, [(1.4, 0.2, 0.1)]))),
        )
        candidates, _, _ = self.publish_tf_and_cloud_until_candidates(31, [(1.8, -0.2, 0.0)])

        self.assertEqual(candidates.accumulated_frames, 2)
        self.assertEqual(candidates.header.stamp.sec, 31)

    def test_query_window_resets_after_publish(self):
        self.publish_tf_and_cloud_for_window(40, [(1.0, 0.0, 0.0)])
        self.publish_tf_and_cloud_until_candidates(41, [(1.4, 0.2, 0.1)])

        self.assert_no_candidate_message(
            lambda: (
                self.tf_pub.publish(TFMessage(transforms=[
                    make_transform(ODOM_FRAME, SENSOR_FRAME, 42, (0.0, 0.0, 0.0))])),
                self.cloud_pub.publish(make_cloud(SENSOR_FRAME, 42, [(1.8, -0.2, 0.0)]))),
        )

    def test_odom_frame_input_is_transformed_to_sensor_before_query_output(self):
        self.publish_tf_and_cloud_for_window(
            50,
            [(11.0, -2.0, 0.5)],
            frame_id=ODOM_FRAME,
            translation=(10.0, -2.0, 0.5),
        )

        candidates, _, query_cloud = self.publish_tf_and_cloud_until_candidates(
            51,
            [(11.4, -1.8, 0.6)],
            frame_id=ODOM_FRAME,
            translation=(10.0, -2.0, 0.5),
        )

        self.assertEqual(candidates.accumulated_frames, 2)
        self.assertEqual(query_cloud.header.frame_id, ODOM_FRAME)
        assert_points_include(self, read_xyz(query_cloud), [(11.0, -2.0, 0.5), (11.4, -1.8, 0.6)])

    def test_missing_map_parameter_fails_fast(self):
        process = subprocess.Popen(
            [
                NODE_EXECUTABLE,
                '--ros-args',
                '-p', 'map.directory:=/tmp/relocalization_missing_snapshot_for_test',
            ],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        deadline = time.monotonic() + 5.0
        while process.poll() is None and time.monotonic() < deadline:
            time.sleep(0.05)
        if process.poll() is None:
            process.kill()
            process.wait(timeout=5)
            self.fail('node did not exit for missing map.directory')
        self.assertNotEqual(process.returncode, 0)


if __name__ == '__main__':
    unittest.main()
