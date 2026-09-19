"""Regression coverage for controller odom and navigation freshness gates."""

import time
import unittest

import launch
import launch_ros.actions
import launch_testing.actions
import pytest
import rclpy
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
from planner.msg import Bspline
from std_msgs.msg import Bool


@pytest.mark.launch_test
def generate_test_description():
    controller = launch_ros.actions.Node(
        package="planner",
        executable="closed_loop_controller",
        namespace="freshness_test",
        parameters=[{
            "max_vx": 0.6,
            "kp_pos": 1.0,
            "odom_timeout": 0.2,
            "require_navigation_enable": True,
            "navigation_timeout": 0.2,
        }],
        arguments=["--ros-args", "--log-level", "warn"],
        output="screen",
    )
    return launch.LaunchDescription([controller, launch_testing.actions.ReadyToTest()])


class TestControllerFreshness(unittest.TestCase):
    def test_stale_inputs_clear_trajectory_and_require_new_trajectory(self):
        rclpy.init()
        node = rclpy.create_node("controller_freshness_test")
        try:
            prefix = "/freshness_test"
            odom_pub = node.create_publisher(Odometry, f"{prefix}/body_pose", 10)
            nav_pub = node.create_publisher(Bool, f"{prefix}/navigation/enabled", 10)
            traj_pub = node.create_publisher(Bspline, f"{prefix}/planning/bspline", 10)
            commands = []
            node.create_subscription(Twist, f"{prefix}/cmd_vel", commands.append, 10)

            def make_odom(stamp=None):
                odom = Odometry()
                odom.header.stamp = stamp if stamp is not None else node.get_clock().now().to_msg()
                odom.pose.pose.orientation.w = 1.0
                return odom

            def make_trajectory():
                trajectory = Bspline()
                trajectory.order = 3
                trajectory.traj_id = 77
                trajectory.pos_pts = [Point(x=1.0, y=0.0, z=0.0) for _ in range(4)]
                trajectory.knots = [float(i) for i in range(-3, 5)]
                return trajectory

            enabled = Bool()
            enabled.data = True
            disabled = Bool()
            disabled.data = False
            trajectory = make_trajectory()

            def drain_commands(duration=0.1):
                deadline = time.monotonic() + duration
                while time.monotonic() < deadline:
                    rclpy.spin_once(node, timeout_sec=0.02)
                commands.clear()

            def spin_for(duration, publish_odom=True, publish_nav=True, publish_traj=False,
                         fixed_stamp=None, nav_msg=None):
                samples = []
                deadline = time.monotonic() + duration
                next_publish = 0.0
                while time.monotonic() < deadline:
                    if time.monotonic() >= next_publish:
                        if publish_odom:
                            odom_pub.publish(make_odom(fixed_stamp))
                        if publish_nav:
                            nav_pub.publish(nav_msg if nav_msg is not None else enabled)
                        if publish_traj:
                            traj_pub.publish(trajectory)
                        next_publish = time.monotonic() + 0.05
                    rclpy.spin_once(node, timeout_sec=0.02)
                    if commands:
                        samples.append(commands[-1])
                return samples

            def wait_until_moving(timeout, **kwargs):
                drain_commands()
                deadline = time.monotonic() + timeout
                next_publish = 0.0
                while time.monotonic() < deadline:
                    if time.monotonic() >= next_publish:
                        if kwargs.get("publish_odom", True):
                            odom_pub.publish(make_odom(kwargs.get("fixed_stamp")))
                        if kwargs.get("publish_nav", True):
                            nav_pub.publish(kwargs.get("nav_msg", enabled))
                        if kwargs.get("publish_traj", False):
                            traj_pub.publish(trajectory)
                        next_publish = time.monotonic() + 0.05
                    rclpy.spin_once(node, timeout_sec=0.02)
                    if commands and commands[-1].linear.x > 0.02:
                        return True
                return False

            def assert_held_for(duration, **kwargs):
                drain_commands()
                start = time.monotonic()
                samples = spin_for(duration, **kwargs)
                elapsed = time.monotonic() - start
                self.assertGreaterEqual(elapsed, duration * 0.9)
                self.assertGreaterEqual(elapsed, 0.25)
                self.assertGreaterEqual(len(samples), 5)
                self.assertTrue(all(
                    abs(cmd.linear.x) + abs(cmd.linear.y) + abs(cmd.angular.z) < 1e-6
                    for cmd in samples[-5:]))

            self.assertTrue(wait_until_moving(2.0, publish_traj=True))

            assert_held_for(0.5, publish_odom=False, publish_nav=True)

            old_stamp = node.get_clock().now().to_msg()
            self.assertTrue(wait_until_moving(2.0, publish_odom=True, publish_nav=True,
                                              publish_traj=True, fixed_stamp=old_stamp))
            assert_held_for(0.5, publish_odom=True, publish_nav=True, fixed_stamp=old_stamp)

            assert_held_for(0.5, publish_odom=True, publish_nav=True)
            self.assertTrue(wait_until_moving(2.0, publish_odom=True, publish_nav=True,
                                              publish_traj=True))

            assert_held_for(0.5, publish_odom=True, publish_nav=True,
                            publish_traj=True, nav_msg=disabled)
            assert_held_for(0.5, publish_odom=True, publish_nav=True)
            self.assertTrue(wait_until_moving(2.0, publish_odom=True, publish_nav=True,
                                              publish_traj=True))

            assert_held_for(0.5, publish_odom=True, publish_nav=False,
                            publish_traj=True)
            assert_held_for(0.5, publish_odom=True, publish_nav=True)
            self.assertTrue(wait_until_moving(2.0, publish_odom=True, publish_nav=True,
                                              publish_traj=True))
        finally:
            node.destroy_node()
            rclpy.shutdown()
