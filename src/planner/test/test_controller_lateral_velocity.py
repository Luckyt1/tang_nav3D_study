"""Exercise lateral correction and its hard limit through the ROS topics."""

import math
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


@pytest.mark.launch_test
def generate_test_description():
    controllers = []
    for name, limit in (("default", None), ("over_limit", 1.0), ("lower", 0.1), ("disabled", 0.0)):
        parameters = {"max_vx": 1.0, "kp_pos": 1.0}
        if limit is not None:
            parameters["max_vy"] = limit
        controllers.append(launch_ros.actions.Node(
            package="planner",
            executable="closed_loop_controller",
            namespace=f"lateral_test/{name}",
            parameters=[parameters],
            arguments=["--ros-args", "--log-level", "warn"],
            output="screen",
        ))
    return launch.LaunchDescription([
        *controllers, launch_testing.actions.ReadyToTest()
    ])


class TestLateralVelocity(unittest.TestCase):
    def test_correction_in_body_frame_and_velocity_limits(self):
        rclpy.init()
        node = rclpy.create_node("lateral_velocity_test")
        try:
            for name, limit in (("default", 0.4), ("over_limit", 0.4), ("lower", 0.1), ("disabled", 0.0)):
                prefix = f"/lateral_test/{name}"
                odom_pub = node.create_publisher(Odometry, f"{prefix}/body_pose", 10)
                traj_pub = node.create_publisher(Bspline, f"{prefix}/planning/bspline", 10)
                commands = []
                sub = node.create_subscription(Twist, f"{prefix}/cmd_vel", commands.append, 10)
                for yaw, target, sign in ((0.0, (0.2, 2.0), 1), (0.0, (0.2, -2.0), -1),
                                          (math.pi / 2.0, (2.0, 0.2), -1)):
                    odom = Odometry()
                    odom.pose.pose.orientation.z = math.sin(yaw / 2.0)
                    odom.pose.pose.orientation.w = math.cos(yaw / 2.0)
                    trajectory = Bspline()
                    trajectory.order = 3
                    trajectory.pos_pts = [Point(x=target[0], y=target[1], z=0.0) for _ in range(4)]
                    trajectory.knots = [float(i) for i in range(-3, 5)]
                    commands.clear()
                    deadline = time.monotonic() + 5.0
                    next_publish = 0.0
                    matched = False
                    while time.monotonic() < deadline:
                        if time.monotonic() >= next_publish:
                            odom.header.stamp = node.get_clock().now().to_msg()
                            odom_pub.publish(odom)
                            traj_pub.publish(trajectory)
                            next_publish = time.monotonic() + 0.1
                        rclpy.spin_once(node, timeout_sec=0.05)
                        # Forward motion proves even the disabled case consumed inputs.
                        if commands and odom_pub.get_subscription_count() and traj_pub.get_subscription_count():
                            command = commands[-1]
                            if abs(command.linear.y - sign * limit) < 1e-6 and command.linear.x > 0.01:
                                matched = True
                                self.assertLessEqual(command.linear.x, 1.0)
                                self.assertAlmostEqual(command.angular.z, 0.0, places=6)
                                break
                    self.assertTrue(matched, f"{name}: no lateral correction {sign * limit} at yaw={yaw}")
                    self.assertTrue(all(abs(cmd.linear.y) <= limit + 1e-6 for cmd in commands))
                node.destroy_subscription(sub)
                node.destroy_publisher(odom_pub)
                node.destroy_publisher(traj_pub)
        finally:
            node.destroy_node()
            rclpy.shutdown()
