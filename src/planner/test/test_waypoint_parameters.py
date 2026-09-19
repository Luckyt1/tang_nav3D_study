"""Verify that the flat ROS 2 waypoint array is accepted at startup."""

import os
import time
import unittest

import launch
import launch_ros.actions
import launch_testing.actions
import pytest
import rclpy
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters


@pytest.mark.launch_test
def generate_test_description():
    root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
    planner = launch_ros.actions.Node(
        package="planner",
        executable="scan_planner_node",
        name="scan_planner_node",
        parameters=[
            os.path.join(root, "config", "planner.yaml"),
            os.path.join(root, "config", "keypoints.example.yaml"),
        ],
        output="screen",
    )
    return (
        launch.LaunchDescription([planner, launch_testing.actions.ReadyToTest()]),
        {"planner": planner},
    )


class TestWaypointParameters(unittest.TestCase):
    def test_process_starts_and_exposes_waypoint_parameters(self, proc_info, planner):
        proc_info.assertWaitForStartup(process=planner, timeout=15)
        rclpy.init()
        node = rclpy.create_node("waypoint_parameter_probe")
        try:
            client = node.create_client(GetParameters, "/scan_planner_node/get_parameters")
            deadline = time.monotonic() + 15.0
            while time.monotonic() < deadline and not client.wait_for_service(timeout_sec=0.1):
                rclpy.spin_once(node, timeout_sec=0.01)
            self.assertTrue(client.service_is_ready(), "planner parameter service did not become ready")

            request = GetParameters.Request()
            request.names = ["fsm.navi_mode", "fsm.waypoints"]
            future = client.call_async(request)
            while time.monotonic() < deadline and not future.done():
                rclpy.spin_once(node, timeout_sec=0.05)
            self.assertTrue(future.done(), "timed out reading planner parameters")
            values = future.result().values
            self.assertEqual(values[0].type, ParameterType.PARAMETER_INTEGER)
            self.assertEqual(values[0].integer_value, 2)
            self.assertEqual(values[1].type, ParameterType.PARAMETER_DOUBLE_ARRAY)
            self.assertEqual(len(values[1].double_array_value), 6)
        finally:
            node.destroy_node()
            rclpy.shutdown()
