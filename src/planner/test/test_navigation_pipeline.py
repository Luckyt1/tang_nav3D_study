"""Exercise the installed preview launch with a synthetic floor and real planner nodes."""

import math
import os
import signal
import subprocess
import time

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry, Path
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from relocalization.msg import TrackingStatus
from planner.msg import Bspline
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from std_msgs.msg import Bool, Header
from tf2_msgs.msg import TFMessage


def test_preview_pipeline_produces_paths_and_cancels_on_localization_loss(tmp_path):
    os.environ['ROS_DOMAIN_ID'] = str(210 + os.getpid() % 15)
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    os.environ['ROS_LOG_DIR'] = str(tmp_path / 'ros_log')
    snapshot = tmp_path / 'map with spaces'
    (snapshot / 'btc').mkdir(parents=True)
    for name in ('metadata.yaml', 'poses.csv', 'btc/manifest.csv'):
        (snapshot / name).touch()
    floor = [(i * .1, j * .1, -1.1) for i in range(-25, 36) for j in range(-25, 26)]
    mapped = [(x + 2., y - 1., z + .3) for x, y, z in floor]
    pcd = ('VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n'
           f'WIDTH {len(mapped)}\nHEIGHT 1\nPOINTS {len(mapped)}\nDATA ascii\n')
    (snapshot / 'static_map.pcd').write_text(
        pcd + ''.join(f'{x} {y} {z}\n' for x, y, z in mapped))
    rclpy.init()
    node = rclpy.create_node('navigation_pipeline_test')
    dynamic_qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT)
    latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
    odom_pub = node.create_publisher(Odometry, '/odin1/odometry', qos_profile_sensor_data)
    cloud_pub = node.create_publisher(PointCloud2, '/odin1/cloud_slam', qos_profile_sensor_data)
    tf_pub = node.create_publisher(TFMessage, '/tf', dynamic_qos)
    status_pub = node.create_publisher(TrackingStatus, '/relocalization/tracking_status', 1)
    goal_pub = node.create_publisher(PoseStamped, '/goal_pose', 10)
    global_paths, local_paths, trajectories, enabled = [], [], [], []
    node.create_subscription(Path, '/map/initial_path', global_paths.append, latched)
    node.create_subscription(Path, '/initial_path', local_paths.append, latched)
    node.create_subscription(Bspline, '/planning/bspline', trajectories.append, 10)
    node.create_subscription(Bool, '/navigation/enabled', enabled.append, qos_profile_sensor_data)
    log_path = tmp_path / 'navigation.log'
    with log_path.open('w') as log:
        process = subprocess.Popen([
            'ros2', 'launch', 'planner', 'navigation.launch.py',
            f'map_directory:={snapshot}', 'start_relocalization:=false', 'start_rviz:=false',
        ], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            def pump(duration, valid=True, send_goal=False, done=lambda: False):
                deadline = time.monotonic() + duration
                next_sample, next_goal = 0., 0.
                while time.monotonic() < deadline:
                    assert process.poll() is None, log_path.read_text()[-5000:]
                    now = time.monotonic()
                    stamp = node.get_clock().now().to_msg()
                    if now >= next_sample:
                        odom = Odometry()
                        odom.header = Header(stamp=stamp, frame_id='odom')
                        odom.child_frame_id = 'imu'
                        # The tuned mounting pitch is +45 degrees; the corrected
                        # base remains level. Cloud points are already in odom.
                        odom.pose.pose.orientation.y = math.sin(math.pi / 8.)
                        odom.pose.pose.orientation.w = math.cos(math.pi / 8.)
                        odom_pub.publish(odom)
                        tf = TransformStamped()
                        tf.header = Header(stamp=stamp, frame_id='map')
                        tf.child_frame_id = 'odom'
                        tf.transform.translation.x = 2.
                        tf.transform.translation.y = -1.
                        tf.transform.translation.z = .3
                        tf.transform.rotation.w = 1.
                        tf_pub.publish(TFMessage(transforms=[tf]))
                        status = TrackingStatus()
                        status.header = Header(stamp=stamp, frame_id='map')
                        status.odom_frame_id = 'odom'
                        status.sensor_frame_id = 'lidar'
                        status.valid = valid
                        status.status = 'tracking' if valid else 'fix_expired'
                        status.fix_stamp = stamp
                        status_pub.publish(status)
                        cloud_pub.publish(create_cloud_xyz32(odom.header, floor))
                        next_sample = now + .1
                    if send_goal and now >= next_goal:
                        goal = PoseStamped()
                        goal.header = Header(stamp=stamp, frame_id='map')
                        goal.pose.position.x = 3.5
                        goal.pose.position.y = -1.
                        goal.pose.position.z = .3
                        goal.pose.orientation.w = 1.
                        goal_pub.publish(goal)
                        next_goal = now + 1.
                    rclpy.spin_once(node, timeout_sec=.01)
                    if done():
                        return True
                return done()

            assert pump(15., done=lambda: enabled and enabled[-1].data), log_path.read_text()[-5000:]
            assert pump(20., send_goal=True, done=lambda: (
                any(len(p.poses) >= 2 for p in global_paths)
                and any(len(p.poses) >= 2 for p in local_paths)
                # An emergency hold is also a B-spline: require actual motion.
                and any(len(t.pos_pts) >= 4
                        and max(p.x for p in t.pos_pts) - min(p.x for p in t.pos_pts) > .1
                        for t in trajectories))), log_path.read_text()[-8000:]
            assert next(p for p in global_paths if p.poses).header.frame_id == 'map'
            assert next(p for p in local_paths if p.poses).header.frame_id == 'odom'
            assert node.count_publishers('/cmd_vel') == 0
            local_paths.clear()
            assert pump(3., valid=False, done=lambda: (
                enabled and not enabled[-1].data
                and local_paths and not local_paths[-1].poses)), log_path.read_text()[-5000:]
        finally:
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=5)
            node.destroy_node()
            rclpy.shutdown()
