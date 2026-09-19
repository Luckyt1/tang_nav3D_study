"""Exercise the real node with odom-frame clouds while device attitude changes."""

import math
import os
import subprocess
import sys
import tempfile
import time
import unittest

import rclpy
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_msgs.msg import TFMessage


EXECUTABLE = sys.argv.pop(1)


class CloudFrameTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # Keep this test separate from any connected robot or other ROS session.
        os.environ['ROS_DOMAIN_ID'] = str(80 + os.getpid() % 140)
        os.environ['ROS_LOCALHOST_ONLY'] = '1'
        cls.logs = tempfile.TemporaryDirectory(prefix='cloud_frame_ros_')
        os.environ['ROS_LOG_DIR'] = cls.logs.name
        rclpy.init()
        cls.node = rclpy.create_node('cloud_frame_test')
        cls.process = subprocess.Popen(
            [EXECUTABLE], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    @classmethod
    def tearDownClass(cls):
        cls.process.terminate()
        try:
            cls.process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            cls.process.kill()
            cls.process.wait(timeout=5)
        cls.node.destroy_node()
        rclpy.shutdown()
        cls.logs.cleanup()

    def wait_for(self, predicate, timeout=10, publish=None):
        deadline = time.monotonic() + timeout
        while not predicate() and time.monotonic() < deadline:
            self.assertIsNone(self.process.poll(), 'processing node exited')
            if publish is not None:
                publish()
            rclpy.spin_once(self.node, timeout_sec=0.05)
        self.assertTrue(predicate(), 'timed out waiting for ROS messages/discovery')

    def test_odom_cloud_keeps_coordinates_across_attitudes(self):
        received = {}
        acknowledged = set()
        self.node.create_subscription(
            PointCloud2, '/cloud/downsampled',
            lambda msg: received.__setitem__(msg.header.stamp.sec, msg), 10)
        self.node.create_subscription(
            TFMessage, '/tf',
            lambda msg: acknowledged.update(t.header.stamp.sec for t in msg.transforms),
            100)
        odom_pub = self.node.create_publisher(
            Odometry, '/odin1/odometry', qos_profile_sensor_data)
        cloud_pub = self.node.create_publisher(
            PointCloud2, '/odin1/cloud_slam', qos_profile_sensor_data)
        self.wait_for(lambda: odom_pub.get_subscription_count() > 0
                      and cloud_pub.get_subscription_count() > 0
                      and self.node.count_publishers('/cloud/downsampled') > 0
                      and self.node.count_publishers('/tf') > 0)

        points = [(1.0, 2.0, 0.0), (1.0, 2.0, 1.0), (2.0, 1.0, -0.2)]
        # Pure heading crosses the old Euler branch; tilt must also leave an
        # already-world-frame cloud unchanged. Include a heading near 180 deg.
        for stamp, (roll, pitch, yaw) in enumerate(
                [(0, 0, 0.1), (0, 0, -0.1), (20, 10, -30), (-15, 5, 179.9)], 1):
            with self.subTest(roll=roll, pitch=pitch, yaw=yaw):
                r, p, y = [math.radians(v) / 2 for v in (roll, pitch, yaw)]
                cr, sr = math.cos(r), math.sin(r)
                cp, sp = math.cos(p), math.sin(p)
                cy, sy = math.cos(y), math.sin(y)
                odom = Odometry()
                odom.header.frame_id = 'odom'
                odom.header.stamp.sec = stamp
                odom.child_frame_id = 'imu'
                odom.pose.pose.orientation.w = cr * cp * cy + sr * sp * sy
                odom.pose.pose.orientation.x = sr * cp * cy - cr * sp * sy
                odom.pose.pose.orientation.y = cr * sp * cy + sr * cp * sy
                odom.pose.pose.orientation.z = cr * cp * sy - sr * sp * cy
                # A TF acknowledgement proves the odometry callback ran before
                # the cloud callback. Retry because discovery and best-effort
                # delivery can lose the first message even with a matched peer.
                self.wait_for(lambda: stamp in acknowledged,
                              publish=lambda: odom_pub.publish(odom))
                header = Header()
                header.frame_id = 'odom'
                header.stamp.sec = stamp
                header.stamp.nanosec = 123456
                cloud = point_cloud2.create_cloud_xyz32(header, points)
                self.wait_for(lambda: stamp in received,
                              publish=lambda: cloud_pub.publish(cloud))
                actual = received[stamp]
                self.assertEqual(actual.header, header)
                xyz = sorted(tuple(float(v) for v in row) for row in
                             point_cloud2.read_points(
                                 actual, field_names=('x', 'y', 'z'), skip_nans=False))
                self.assertEqual(len(xyz), len(points))
                for output, expected in zip(xyz, sorted(points)):
                    for value, reference in zip(output, expected):
                        self.assertAlmostEqual(value, reference, places=5)


if __name__ == '__main__':
    unittest.main()
