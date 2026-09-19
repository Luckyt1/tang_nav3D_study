#!/usr/bin/env python3

from copy import deepcopy
import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from relocalization.msg import TrackingStatus
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from tf2_geometry_msgs import do_transform_pose_stamped
from tf2_ros import Buffer, TransformException, TransformListener
from std_msgs.msg import Bool


def transform_odometry(message, transform):
    pose = PoseStamped()
    pose.header = message.header
    pose.pose = message.pose.pose
    transformed_pose = do_transform_pose_stamped(pose, transform)

    transformed = deepcopy(message)
    transformed.header.frame_id = transform.header.frame_id
    transformed.pose.pose = transformed_pose.pose
    return transformed


def transform_path(message, transform):
    transformed = Path()
    transformed.header = deepcopy(message.header)
    transformed.header.frame_id = transform.header.frame_id
    for pose in message.poses:
        source_pose = deepcopy(pose)
        if not source_pose.header.frame_id:
            source_pose.header.frame_id = message.header.frame_id
        output_pose = do_transform_pose_stamped(source_pose, transform)
        output_pose.header.stamp = message.header.stamp
        transformed.poses.append(output_pose)
    return transformed


class RelocalizationBridge(Node):
    def __init__(self):
        super().__init__("relocalization_bridge")
        self.map_frame = self.declare_parameter("map_frame", "map").value
        self.odom_frame = self.declare_parameter("odom_frame", "odom").value
        self.tracking_timeout = self.declare_parameter("tracking_timeout", 0.5).value
        if not math.isfinite(self.tracking_timeout) or self.tracking_timeout <= 0:
            raise ValueError("tracking_timeout must be finite and positive")
        self.tracking_status = None
        self.tracking_received = 0.0
        self.enabled = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer, self,
            qos=QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.waiting_for_tf = True

        path_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.map_odom_publisher = self.create_publisher(
            Odometry, "map_odom", qos_profile_sensor_data
        )
        self.odom_path_publisher = self.create_publisher(Path, "output_path", path_qos)
        self.map_goal_publisher = self.create_publisher(PoseStamped, "output_goal", 10)
        self.enabled_publisher = self.create_publisher(
            Bool, "navigation/enabled", qos_profile_sensor_data)
        self.create_subscription(
            TrackingStatus, "/relocalization/tracking_status", self.tracking_callback,
            qos_profile_sensor_data)
        self.create_subscription(
            Odometry, "input_odom", self.odometry_callback, qos_profile_sensor_data
        )
        self.create_subscription(Path, "input_path", self.path_callback, path_qos)
        self.create_subscription(PoseStamped, "input_goal", self.goal_callback, 10)
        self.create_subscription(PoseStamped, "input_replan_goal", self.goal_callback, 10)
        self.create_timer(0.1, self.publish_state)

    def tracking_callback(self, message):
        self.tracking_status = message
        self.tracking_received = time.monotonic()
        if not self.tracking_valid() and self.enabled is not False:
            self.publish_state()

    def tracking_valid(self):
        status = self.tracking_status
        return bool(
            status is not None and status.valid and status.status == "tracking"
            and status.header.frame_id == self.map_frame
            and status.odom_frame_id == self.odom_frame
            and time.monotonic() - self.tracking_received <= self.tracking_timeout)

    def publish_state(self):
        ready = self.tracking_valid() and self.lookup(self.map_frame, self.odom_frame) is not None
        self.enabled_publisher.publish(Bool(data=ready))
        if not ready and self.enabled is not False:
            # Also clear the transient-local path for late or recovering planners.
            empty = Path()
            empty.header.frame_id = self.odom_frame
            empty.header.stamp = self.get_clock().now().to_msg()
            self.odom_path_publisher.publish(empty)
        self.enabled = ready

    def lookup(self, target_frame, source_frame):
        try:
            transform = self.tf_buffer.lookup_transform(target_frame, source_frame, Time())
        except TransformException as error:
            if not self.waiting_for_tf:
                self.waiting_for_tf = True
            if self.waiting_for_tf:
                self.get_logger().warn(
                    f"Waiting for relocalization TF {source_frame} -> {target_frame}: {error}",
                    once=True,
                )
            return None
        if self.waiting_for_tf:
            self.get_logger().info(
                f"Relocalization TF ready: {source_frame} -> {target_frame}; navigation unlocked"
            )
            self.waiting_for_tf = False
        return transform

    def odometry_callback(self, message):
        if not self.tracking_valid() or message.header.frame_id != self.odom_frame:
            return
        source_frame = message.header.frame_id
        transform = self.lookup(self.map_frame, source_frame)
        if transform is not None:
            self.map_odom_publisher.publish(transform_odometry(message, transform))

    def path_callback(self, message):
        if not self.tracking_valid() or message.header.frame_id != self.map_frame:
            return
        if any(pose.header.frame_id not in ("", self.map_frame) for pose in message.poses):
            self.get_logger().warn("Discarding a path with mixed coordinate frames")
            return
        source_frame = message.header.frame_id
        transform = self.lookup(self.odom_frame, source_frame)
        if transform is not None:
            self.odom_path_publisher.publish(transform_path(message, transform))

    def goal_callback(self, message):
        if not self.tracking_valid():
            self.get_logger().warn("Waiting for valid localization before accepting a goal", once=True)
            return
        source_frame = message.header.frame_id
        if source_frame not in (self.map_frame, self.odom_frame):
            self.get_logger().warn("Goal must be in map or odom coordinates")
            return
        transform = self.lookup(self.map_frame, source_frame)
        if transform is not None:
            self.map_goal_publisher.publish(do_transform_pose_stamped(message, transform))


def main(args=None):
    rclpy.init(args=args)
    node = RelocalizationBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
