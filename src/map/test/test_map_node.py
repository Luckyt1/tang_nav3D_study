"""Integration tests for the ROS 2 FreeDOM map node.

The test starts the real executable and drives it with timestamped TF and
PointCloud2 messages. It intentionally keeps all state in a private ROS domain
and temporary log directory so it can run on a development machine without
talking to a robot.
"""

import csv
import math
import os
import subprocess
import sys
import tempfile
import time
import unittest
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import TransformStamped
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage


EXECUTABLE = sys.argv.pop(1)

INPUT_TOPIC = '/odin1/cloud_slam'
OUTPUT_TOPIC = '/map/static_cloud'
SUBMAP_TOPIC = '/map/submap_cloud'
COMPLETED_SUBMAP_TOPIC = '/map/submap_completed'
SUBMAP_POSES_TOPIC = '/map/submap_poses'
SAVE_SERVICE = '/map/save_map'
MAP_FRAME = 'odom'
IMU_FRAME = 'imu'
SENSOR_FRAME = 'lidar'
POSE_COLUMNS = [
    'id',
    'stamp_ns',
    'tx',
    'ty',
    'tz',
    'qx',
    'qy',
    'qz',
    'qw',
    'point_count',
    'pcd_file',
]


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


def make_malformed_cloud(stamp_sec):
    header = Header()
    header.frame_id = SENSOR_FRAME
    header.stamp.sec = stamp_sec
    return point_cloud2.create_cloud(
        header,
        [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        ],
        [(1.0, 2.0)],
    )


def read_xyz(cloud):
    return [
        tuple(float(value) for value in row)
        for row in point_cloud2.read_points(
            cloud, field_names=('x', 'y', 'z'), skip_nans=False)
    ]


def all_points_finite(points):
    return all(math.isfinite(value) for point in points for value in point)


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


def assert_pose_near(testcase, pose, translation, yaw, places=4):
    testcase.assertAlmostEqual(pose.position.x, translation[0], places=places)
    testcase.assertAlmostEqual(pose.position.y, translation[1], places=places)
    testcase.assertAlmostEqual(pose.position.z, translation[2], places=places)
    x, y, z, w = yaw_quaternion(yaw)
    testcase.assertAlmostEqual(pose.orientation.x, x, places=places)
    testcase.assertAlmostEqual(pose.orientation.y, y, places=places)
    testcase.assertAlmostEqual(pose.orientation.z, z, places=places)
    testcase.assertAlmostEqual(pose.orientation.w, w, places=places)


def read_pose_rows(save_dir):
    with (save_dir / 'poses.csv').open(newline='') as stream:
        reader = csv.DictReader(stream)
        if reader.fieldnames != POSE_COLUMNS:
            raise AssertionError(f'unexpected CSV columns: {reader.fieldnames}')
        return list(reader)


def pcd_point_count(path):
    with path.open('r', encoding='utf-8', errors='replace') as stream:
        for line in stream:
            if line.startswith('POINTS '):
                return int(line.split()[1])
            if line.startswith('DATA '):
                break
    raise AssertionError(f'PCD file does not declare POINTS: {path}')


def file_bytes_by_relative_path(save_dir):
    return {
        path.relative_to(save_dir): path.read_bytes()
        for path in sorted(save_dir.rglob('*'))
        if path.is_file()
    }


def assert_pose_row_near(testcase, row, submap_id, stamp_ns, translation, yaw, places=4):
    testcase.assertEqual(row['id'], str(submap_id))
    testcase.assertEqual(row['stamp_ns'], str(stamp_ns))
    testcase.assertAlmostEqual(float(row['tx']), translation[0], places=places)
    testcase.assertAlmostEqual(float(row['ty']), translation[1], places=places)
    testcase.assertAlmostEqual(float(row['tz']), translation[2], places=places)
    x, y, z, w = yaw_quaternion(yaw)
    testcase.assertAlmostEqual(float(row['qx']), x, places=places)
    testcase.assertAlmostEqual(float(row['qy']), y, places=places)
    testcase.assertAlmostEqual(float(row['qz']), z, places=places)
    testcase.assertAlmostEqual(float(row['qw']), w, places=places)


class MapNodeIntegrationTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        os.environ['ROS_DOMAIN_ID'] = str(120 + os.getpid() % 80)
        os.environ['ROS_LOCALHOST_ONLY'] = '1'
        cls.logs = tempfile.TemporaryDirectory(prefix='map_node_ros_')
        os.environ['ROS_LOG_DIR'] = cls.logs.name
        rclpy.init()
        cls.node = rclpy.create_node('map_node_integration_test')
        cls.received = {}
        cls.output_count = 0
        cls.submaps = {}
        cls.completed_submaps = {}
        cls.submap_poses = []
        cls.static_tf_messages = []

        def remember_output(msg):
            cls.output_count += 1
            cls.received[msg.header.stamp.sec] = msg

        cls.node.create_subscription(
            PointCloud2,
            OUTPUT_TOPIC,
            remember_output,
            QoSProfile(depth=20),
        )
        submap_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        cls.node.create_subscription(
            PointCloud2,
            SUBMAP_TOPIC,
            lambda msg: cls.submaps.__setitem__(msg.header.stamp.sec, msg),
            submap_qos,
        )
        cls.node.create_subscription(
            PointCloud2,
            COMPLETED_SUBMAP_TOPIC,
            lambda msg: cls.completed_submaps.__setitem__(msg.header.stamp.sec, msg),
            submap_qos,
        )
        cls.node.create_subscription(
            PoseArray,
            SUBMAP_POSES_TOPIC,
            lambda msg: cls.submap_poses.append(msg),
            submap_qos,
        )
        cls.node.create_subscription(
            TFMessage,
            '/tf_static',
            lambda msg: cls.static_tf_messages.append(msg),
            submap_qos,
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
        cls.tf_pub = cls.node.create_publisher(
            TFMessage,
            '/tf',
            QoSProfile(depth=50),
        )
        cls.static_tf_pub = cls.node.create_publisher(
            TFMessage,
            '/tf_static',
            QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                reliability=ReliabilityPolicy.RELIABLE,
            ),
        )
        cls.save_client = cls.node.create_client(Trigger, SAVE_SERVICE)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()
        cls.logs.cleanup()

    def setUp(self):
        type(self).received.clear()
        type(self).output_count = 0
        type(self).submaps.clear()
        type(self).completed_submaps.clear()
        type(self).submap_poses.clear()
        type(self).static_tf_messages.clear()
        self.save_root = tempfile.TemporaryDirectory(prefix='map_node_save_')
        self.process = subprocess.Popen(
            [
                EXECUTABLE,
                '--ros-args',
                '-p', 'sensor.min_range:=0.0',
                '-p', 'sensor.max_range:=10.0',
                '-p', 'sensor.min_z:=-4.0',
                '-p', 'sensor.max_z:=4.0',
                '-p', 'freedom.sub_voxel_size:=0.02',
                '-p', 'freedom.counts_to_free:=999',
                '-p', 'freedom.counts_to_revert:=999',
                '-p', 'freedom.num_threads:=1',
                '-p', 'submap.translation_threshold:=2.0',
                '-p', 'submap.radius:=5.0',
                '-p', f'save.directory:={self.save_root.name}',
            ],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        self.wait_for(self.ros_graph_ready)
        self.publish_static_extrinsic()

    def tearDown(self):
        self.process.terminate()
        try:
            self.process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            self.process.kill()
            self.process.wait(timeout=5)
        self.save_root.cleanup()

    @classmethod
    def ros_graph_ready(cls):
        return (
            cls.cloud_pub.get_subscription_count() > 0
            and cls.tf_pub.get_subscription_count() > 0
            and cls.static_tf_pub.get_subscription_count() > 0
            and cls.node.count_publishers(OUTPUT_TOPIC) > 0
            and cls.node.count_publishers(SUBMAP_TOPIC) > 0
            and cls.node.count_publishers(COMPLETED_SUBMAP_TOPIC) > 0
            and cls.node.count_publishers(SUBMAP_POSES_TOPIC) > 0
            and cls.save_client.service_is_ready()
        )

    @classmethod
    def wait_for_class(cls, predicate, timeout=10.0, publish=None):
        deadline = time.monotonic() + timeout
        while not predicate() and time.monotonic() < deadline:
            if publish is not None:
                publish()
            rclpy.spin_once(cls.node, timeout_sec=0.05)
        if not predicate():
            raise AssertionError('timed out waiting for ROS graph or messages')

    @classmethod
    def publish_static_extrinsic(cls):
        static_tf = TFMessage()
        static_tf.transforms.append(
            make_transform(IMU_FRAME, SENSOR_FRAME, 0, (0.3, -0.2, 0.5), 0.0)
        )
        cls.static_tf_pub.publish(static_tf)
        rclpy.spin_once(cls.node, timeout_sec=0.05)

    def wait_for(self, predicate, timeout=5.0, publish=None):
        deadline = time.monotonic() + timeout
        while not predicate() and time.monotonic() < deadline:
            self.assertIsNone(self.process.poll(), 'map_node exited')
            if publish is not None:
                publish()
            rclpy.spin_once(self.node, timeout_sec=0.05)
        self.assertTrue(predicate(), 'timed out waiting for ROS graph or messages')

    def dynamic_tf(self, stamp_sec, translation, yaw):
        msg = TFMessage()
        msg.transforms.append(
            make_transform(MAP_FRAME, IMU_FRAME, stamp_sec, translation, yaw)
        )
        return msg

    def publish_cloud_until_output(self, cloud, expected_stamp, tf_msg):
        self.wait_for(
            lambda: expected_stamp in self.received,
            publish=lambda: (self.tf_pub.publish(tf_msg), self.cloud_pub.publish(cloud)),
        )
        return self.received[expected_stamp]

    def publish_cloud_until_static_and_active_submap(self, cloud, expected_stamp, tf_msg):
        self.wait_for(
            lambda: expected_stamp in self.received and expected_stamp in self.submaps,
            publish=lambda: (self.tf_pub.publish(tf_msg), self.cloud_pub.publish(cloud)),
        )
        return self.received[expected_stamp], self.submaps[expected_stamp]

    def wait_for_submap_stamp(self, cache, expected_stamp, publish):
        self.wait_for(lambda: expected_stamp in cache, publish=publish)
        return cache[expected_stamp]

    def call_save_map(self):
        self.wait_for(lambda: self.save_client.service_is_ready())
        future = self.save_client.call_async(Trigger.Request())
        self.wait_for(lambda: future.done())
        self.assertIsNone(future.exception())
        return future.result()

    def assert_no_new_output(self, publish, timeout=0.35):
        count_before = self.output_count
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.assertIsNone(self.process.poll(), 'map_node exited')
            publish()
            rclpy.spin_once(self.node, timeout_sec=0.05)
        self.assertEqual(self.output_count, count_before)

    def publish_for_window(self, publish, timeout=0.35):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.assertIsNone(self.process.poll(), 'map_node exited')
            publish()
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def test_does_not_publish_when_timestamped_tf_is_missing(self):
        cloud = make_cloud(SENSOR_FRAME, 10, [(1.0, 0.0, 0.0)])

        self.assert_no_new_output(lambda: self.cloud_pub.publish(cloud))
        # 有旧 TF 也不能拿“最新可用位姿”代替点云时刻的位姿。
        stale_tf = self.dynamic_tf(9, (0.0, 0.0, 0.0), 0.0)
        self.assert_no_new_output(
            lambda: (self.tf_pub.publish(stale_tf), self.cloud_pub.publish(cloud)))

    def test_waits_for_delayed_timestamped_tf_with_default_timeout(self):
        # 10 Hz 的下一帧 TF 加上调度抖动可能晚于旧的 100 ms 等待上限。
        # 只发一次点云，避免重发掩盖因等待超时而丢帧的问题。
        stale_tf = self.dynamic_tf(119, (0.0, 0.0, 0.0), 0.0)
        self.publish_for_window(lambda: self.tf_pub.publish(stale_tf), timeout=0.1)
        self.cloud_pub.publish(make_cloud(SENSOR_FRAME, 120, [(1.0, 0.0, 0.0)]))
        deadline = time.monotonic() + 0.15
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.005)
        self.assertNotIn(120, self.received, 'must wait for the timestamped pose')

        self.tf_pub.publish(self.dynamic_tf(120, (3.0, 0.0, 0.0), 0.0))
        self.wait_for(lambda: 120 in self.received, timeout=2.0)
        self.assertEqual(self.received[120].header.stamp.sec, 120)
        # 静态雷达外参 (0.3,-0.2,0.5) 加上新位姿，不能用旧 TF 代替。
        assert_points_include(self, read_xyz(self.received[120]), [(4.3, -0.2, 0.5)])

    def test_save_map_rejects_empty_map(self):
        response = self.call_save_map()

        self.assertFalse(response.success)
        self.assertTrue(response.message)
        self.assertEqual(list(Path(self.save_root.name).iterdir()), [])

    def test_sensor_frame_cloud_is_transformed_to_world_and_prior_map_persists(self):
        first_tf = self.dynamic_tf(20, (50.0, 2.0, 0.0), math.pi / 2.0)
        first_cloud = make_cloud(SENSOR_FRAME, 20, [(1.0, 0.0, 0.0)])

        first_output = self.publish_cloud_until_output(first_cloud, 20, first_tf)

        self.assertEqual(first_output.header.frame_id, MAP_FRAME)
        first_expected = [(50.2, 3.3, 0.5)]
        assert_points_include(self, read_xyz(first_output), first_expected)

        second_tf = self.dynamic_tf(21, (-1.0, 0.5, 0.2), -math.pi / 2.0)
        second_cloud = make_cloud(SENSOR_FRAME, 21, [(0.0, 2.0, -0.5)])

        second_output = self.publish_cloud_until_output(second_cloud, 21, second_tf)

        second_expected = [(50.2, 3.3, 0.5), (0.8, 0.2, 0.2)]
        assert_points_include(self, read_xyz(second_output), second_expected)

    def test_world_frame_cloud_keeps_coordinates_with_nonzero_sensor_pose(self):
        tf_msg = self.dynamic_tf(30, (50.0, -1.0, 0.3), math.radians(30.0))
        cloud = make_cloud(MAP_FRAME, 30, [(50.8, -0.7, 0.8)])

        output = self.publish_cloud_until_output(cloud, 30, tf_msg)

        assert_points_include(self, read_xyz(output), [(50.8, -0.7, 0.8)])

    def test_motion_submaps_publish_local_clouds_completed_snapshots_poses_and_static_tf(self):
        initial_tf = self.dynamic_tf(80, (0.0, 0.0, 0.0), 0.0)
        initial_cloud = make_cloud(SENSOR_FRAME, 80, [(1.0, 0.0, 0.0), (4.5, 0.0, 0.0)])

        _, initial_submap = self.publish_cloud_until_static_and_active_submap(
            initial_cloud, 80, initial_tf)
        self.wait_for(lambda: len(self.submap_poses) >= 1)

        self.assertEqual(initial_submap.header.frame_id, 'map_submap_0')
        assert_points_include(self, read_xyz(initial_submap), [(1.0, 0.0, 0.0), (4.5, 0.0, 0.0)])
        self.assertEqual(len(self.submap_poses[-1].poses), 1)
        self.assertEqual(self.submap_poses[-1].header.frame_id, MAP_FRAME)
        assert_pose_near(self, self.submap_poses[-1].poses[0], (0.3, -0.2, 0.5), 0.0)

        pose_publish_count = len(self.submap_poses)
        below_threshold_tf = self.dynamic_tf(81, (1.5, 0.0, 0.0), 0.0)
        below_threshold_cloud = make_cloud(SENSOR_FRAME, 81, [(0.5, 0.0, 0.0)])

        _, unchanged_submap = self.publish_cloud_until_static_and_active_submap(
            below_threshold_cloud, 81, below_threshold_tf)
        self.publish_for_window(
            publish=lambda: (
                self.tf_pub.publish(below_threshold_tf),
                self.cloud_pub.publish(below_threshold_cloud)),
            timeout=0.2,
        )

        self.assertEqual(unchanged_submap.header.frame_id, 'map_submap_0')
        assert_points_include(self, read_xyz(unchanged_submap), [(2.0, 0.0, 0.0)])
        self.assertEqual(len(self.completed_submaps), 0)
        self.assertEqual(len(self.submap_poses), pose_publish_count)

        threshold_tf = self.dynamic_tf(82, (2.1, 0.0, 0.0), 0.0)
        threshold_cloud = make_cloud(SENSOR_FRAME, 82, [(0.5, 0.0, 0.0)])
        publish_threshold = lambda: (
            self.tf_pub.publish(threshold_tf),
            self.cloud_pub.publish(threshold_cloud))

        _, new_active_submap = self.publish_cloud_until_static_and_active_submap(
            threshold_cloud, 82, threshold_tf)
        completed_submap = self.wait_for_submap_stamp(
            self.completed_submaps, 82, publish_threshold)
        self.wait_for(lambda: len(self.submap_poses) >= pose_publish_count + 1)
        self.wait_for(
            lambda: any(
                {'map_submap_0', 'map_submap_1'}.issubset(
                    {tf.child_frame_id for tf in msg.transforms})
                for msg in self.static_tf_messages
            )
        )

        self.assertEqual(new_active_submap.header.frame_id, 'map_submap_1')
        assert_points_include(self, read_xyz(new_active_submap), [(0.5, 0.0, 0.0)])
        self.assertEqual(completed_submap.header.frame_id, 'map_submap_0')
        assert_points_include(self, read_xyz(completed_submap), [(2.6, 0.0, 0.0)])
        self.assertEqual(len(self.submap_poses[-1].poses), 2)
        assert_pose_near(self, self.submap_poses[-1].poses[0], (0.3, -0.2, 0.5), 0.0)
        assert_pose_near(self, self.submap_poses[-1].poses[1], (2.4, -0.2, 0.5), 0.0)

    def test_save_map_writes_snapshot_files_poses_metadata_and_unique_directories(self):
        first_tf = self.dynamic_tf(100, (0.0, 0.0, 0.0), 0.0)
        first_cloud = make_cloud(SENSOR_FRAME, 100, [(1.0, 0.0, 0.0), (4.5, 0.0, 0.0)])
        second_tf = self.dynamic_tf(101, (2.1, 0.0, 0.0), 0.0)
        second_cloud = make_cloud(SENSOR_FRAME, 101, [(0.5, 0.0, 0.0)])

        self.publish_cloud_until_static_and_active_submap(first_cloud, 100, first_tf)
        self.publish_cloud_until_static_and_active_submap(second_cloud, 101, second_tf)
        self.wait_for(lambda: len(self.submap_poses) >= 2)

        first_response = self.call_save_map()

        self.assertTrue(first_response.success)
        first_save_dir = Path(first_response.message)
        self.assertTrue(first_save_dir.is_absolute())
        self.assertTrue(first_save_dir.is_dir())
        self.assertEqual(str(first_save_dir), first_response.message)
        self.assertEqual(
            os.path.commonpath([self.save_root.name, str(first_save_dir)]),
            self.save_root.name,
        )

        static_map = first_save_dir / 'static_map.pcd'
        metadata = first_save_dir / 'metadata.yaml'
        poses_csv = first_save_dir / 'poses.csv'
        submap_0 = first_save_dir / 'submaps' / '000000.pcd'
        submap_1 = first_save_dir / 'submaps' / '000001.pcd'
        for path in [static_map, metadata, poses_csv, submap_0, submap_1]:
            self.assertTrue(path.is_file(), f'missing saved file: {path}')
        self.assertGreater(pcd_point_count(static_map), 0)

        rows = read_pose_rows(first_save_dir)
        self.assertEqual(len(rows), 2)
        assert_pose_row_near(
            self, rows[0], 0, 100123456789, (0.3, -0.2, 0.5), 0.0)
        assert_pose_row_near(
            self, rows[1], 1, 101123456789, (2.4, -0.2, 0.5), 0.0)
        self.assertEqual(rows[0]['pcd_file'], 'submaps/000000.pcd')
        self.assertEqual(rows[1]['pcd_file'], 'submaps/000001.pcd')
        self.assertEqual(int(rows[0]['point_count']), pcd_point_count(submap_0))
        self.assertEqual(int(rows[1]['point_count']), pcd_point_count(submap_1))
        self.assertGreater(int(rows[0]['point_count']), 0)
        self.assertGreater(int(rows[1]['point_count']), 0)

        metadata_text = metadata.read_text(encoding='utf-8')
        self.assertIn('snapshot', metadata_text.lower())
        self.assertIn('101123456789', metadata_text)
        self.assertIn('btc:\n  enabled: true', metadata_text)
        with (first_save_dir / 'btc/manifest.csv').open(newline='') as stream:
            descriptors = list(csv.DictReader(stream))
        self.assertEqual([row['id'] for row in descriptors], ['0', '1'])
        # 此场景只有几个点，保存仍成功，但不能伪报成可检索的三角描述子。
        for row in descriptors:
            self.assertEqual(row['status'], 'no_triangles')
            self.assertEqual(row['triangle_count'], '0')
            descriptor = first_save_dir / row['descriptor_file']
            self.assertTrue(descriptor.is_file())
            self.assertTrue(descriptor.read_text().startswith(
                f"MAP_BTC 1\nsubmap_id {row['id']}\n"))
        with (first_save_dir / 'btc/index.csv').open(newline='') as stream:
            self.assertEqual(list(csv.DictReader(stream)), [])
        first_files = file_bytes_by_relative_path(first_save_dir)

        second_response = self.call_save_map()

        self.assertTrue(second_response.success)
        second_save_dir = Path(second_response.message)
        self.assertTrue(second_save_dir.is_absolute())
        self.assertTrue(second_save_dir.is_dir())
        self.assertNotEqual(second_save_dir, first_save_dir)
        self.assertEqual(first_files, file_bytes_by_relative_path(first_save_dir))

    def test_in_place_rotation_keeps_updating_the_current_submap_without_creating_new_ids(self):
        seed_tf = self.dynamic_tf(90, (0.0, 0.0, 0.0), 0.0)
        seed_cloud = make_cloud(SENSOR_FRAME, 90, [(1.0, 0.0, 0.0)])

        _, seed_submap = self.publish_cloud_until_static_and_active_submap(
            seed_cloud, 90, seed_tf)
        self.wait_for(lambda: len(self.submap_poses) >= 1)

        self.assertEqual(seed_submap.header.frame_id, 'map_submap_0')
        self.assertEqual(len(self.submap_poses[-1].poses), 1)
        assert_pose_near(self, self.submap_poses[-1].poses[0], (0.3, -0.2, 0.5), 0.0)
        pose_publish_count = len(self.submap_poses)

        rotations = [
            (91, math.radians(35.0)),
            (92, math.radians(120.0)),
            (93, math.radians(179.0)),
            (94, math.radians(-179.0)),
        ]
        for stamp, yaw in rotations:
            tf_msg = self.dynamic_tf(stamp, (0.0, 0.0, 0.0), yaw)
            cloud = make_cloud(SENSOR_FRAME, stamp, [(1.0, 0.0, 0.0)])

            _, active_submap = self.publish_cloud_until_static_and_active_submap(
                cloud, stamp, tf_msg)

            self.assertEqual(active_submap.header.frame_id, 'map_submap_0')
            self.assertEqual(active_submap.header.stamp.sec, stamp)
            self.assertGreater(len(read_xyz(active_submap)), 0)

        self.publish_for_window(
            publish=lambda: (
                self.tf_pub.publish(self.dynamic_tf(94, (0.0, 0.0, 0.0), math.radians(-179.0))),
                self.cloud_pub.publish(make_cloud(SENSOR_FRAME, 94, [(1.0, 0.0, 0.0)]))),
            timeout=0.2,
        )

        self.assertEqual(len(self.completed_submaps), 0)
        self.assertEqual(len(self.submap_poses), pose_publish_count)
        self.assertEqual(len(self.submap_poses[-1].poses), 1)
        assert_pose_near(self, self.submap_poses[-1].poses[0], (0.3, -0.2, 0.5), 0.0)

    def test_duplicate_and_out_of_order_clouds_are_dropped(self):
        tf_msg = self.dynamic_tf(40, (0.0, 0.0, 0.0), 0.0)
        first_cloud = make_cloud(SENSOR_FRAME, 40, [(2.0, 0.0, 0.0)])
        first_output = self.publish_cloud_until_output(first_cloud, 40, tf_msg)
        first_count = len(read_xyz(first_output))

        duplicate = make_cloud(SENSOR_FRAME, 40, [(3.0, 0.0, 0.0)])
        out_of_order = make_cloud(SENSOR_FRAME, 39, [(4.0, 0.0, 0.0)])

        self.assert_no_new_output(lambda: (self.tf_pub.publish(tf_msg), self.cloud_pub.publish(duplicate)))
        self.assert_no_new_output(lambda: (self.tf_pub.publish(tf_msg), self.cloud_pub.publish(out_of_order)))
        self.assertEqual(len(read_xyz(self.received[40])), first_count)

    def test_malformed_and_nan_clouds_do_not_poison_following_valid_cloud(self):
        tf_49 = self.dynamic_tf(49, (0.0, 0.0, 0.0), 0.0)
        tf_50 = self.dynamic_tf(50, (0.0, 0.0, 0.0), 0.0)
        tf_51 = self.dynamic_tf(51, (0.0, 0.0, 0.0), 0.0)
        tf_52 = self.dynamic_tf(52, (0.0, 0.0, 0.0), 0.0)
        seed = make_cloud(SENSOR_FRAME, 49, [(0.5, 0.0, 0.0)])
        malformed = make_malformed_cloud(50)
        nan_cloud = make_cloud(SENSOR_FRAME, 51, [(math.nan, 0.0, 0.0)])
        valid = make_cloud(SENSOR_FRAME, 52, [(1.5, 0.0, 0.0)])

        self.publish_cloud_until_output(seed, 49, tf_49)
        self.assert_no_new_output(lambda: (self.tf_pub.publish(tf_50), self.cloud_pub.publish(malformed)))
        self.publish_for_window(
            publish=lambda: (self.tf_pub.publish(tf_51), self.cloud_pub.publish(nan_cloud)),
        )
        if 51 in self.received:
            self.assertTrue(all_points_finite(read_xyz(self.received[51])))
        output = self.publish_cloud_until_output(valid, 52, tf_52)

        actual = read_xyz(output)
        self.assertTrue(all_points_finite(actual))
        assert_points_include(self, actual, [(0.8, -0.2, 0.5)])
        assert_points_include(self, actual, [(1.8, -0.2, 0.5)])


if __name__ == '__main__':
    unittest.main()
