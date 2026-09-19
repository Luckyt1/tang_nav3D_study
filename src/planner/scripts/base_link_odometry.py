#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


def quaternion_from_rpy(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return normalize_quaternion((
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    ))


def normalize_quaternion(q):
    norm = math.sqrt(sum(value * value for value in q))
    if norm < 1e-12:
        raise ValueError("quaternion norm must be non-zero")
    return tuple(value / norm for value in q)


def quaternion_multiply(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def quaternion_conjugate(q):
    return (-q[0], -q[1], -q[2], q[3])


def rotate_vector(q, vector):
    rotated = quaternion_multiply(
        quaternion_multiply(q, (vector[0], vector[1], vector[2], 0.0)),
        quaternion_conjugate(q),
    )
    return rotated[:3]


def rotate_twist_covariance(covariance, q):
    columns = [
        rotate_vector(q, (1.0, 0.0, 0.0)),
        rotate_vector(q, (0.0, 1.0, 0.0)),
        rotate_vector(q, (0.0, 0.0, 1.0)),
    ]
    rotation = [[columns[column][row] for column in range(3)] for row in range(3)]
    transform = [[0.0] * 6 for _ in range(6)]
    for block in (0, 3):
        for row in range(3):
            for column in range(3):
                transform[block + row][block + column] = rotation[row][column]

    source = [list(covariance[row * 6:(row + 1) * 6]) for row in range(6)]
    left = [
        [sum(transform[row][k] * source[k][column] for k in range(6)) for column in range(6)]
        for row in range(6)
    ]
    rotated = [
        [sum(left[row][k] * transform[column][k] for k in range(6)) for column in range(6)]
        for row in range(6)
    ]
    return [value for row in rotated for value in row]


class BaseLinkOdometry(Node):
    def __init__(self):
        super().__init__("base_link_odometry")
        sensor_frame = self.declare_parameter("sensor_frame", "imu").value
        self.base_frame = self.declare_parameter("base_frame", "base_link").value
        mount_roll = self.declare_parameter("mount_roll", 0.0).value
        mount_pitch = self.declare_parameter("mount_pitch", math.pi / 4.0).value
        mount_yaw = self.declare_parameter("mount_yaw", 0.0).value

        self.q_base_sensor = quaternion_from_rpy(mount_roll, mount_pitch, mount_yaw)
        self.q_sensor_base = quaternion_conjugate(self.q_base_sensor)
        self.publisher = self.create_publisher(Odometry, "base_odom", qos_profile_sensor_data)
        self.subscription = self.create_subscription(
            Odometry, "input_odom", self.odometry_callback, qos_profile_sensor_data
        )

        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = sensor_frame
        transform.child_frame_id = self.base_frame
        transform.transform.rotation.x = self.q_sensor_base[0]
        transform.transform.rotation.y = self.q_sensor_base[1]
        transform.transform.rotation.z = self.q_sensor_base[2]
        transform.transform.rotation.w = self.q_sensor_base[3]
        self.static_broadcaster = StaticTransformBroadcaster(self)
        self.static_broadcaster.sendTransform(transform)

        self.get_logger().info(
            f"Correcting {sensor_frame} odometry to {self.base_frame}: "
            f"mount rpy=({mount_roll:.3f}, {mount_pitch:.3f}, {mount_yaw:.3f}) rad"
        )

    def odometry_callback(self, msg):
        corrected = Odometry()
        corrected.header = msg.header
        corrected.child_frame_id = self.base_frame
        corrected.pose.pose.position = msg.pose.pose.position

        q_odom_sensor = normalize_quaternion((
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        ))
        q_odom_base = normalize_quaternion(
            quaternion_multiply(q_odom_sensor, self.q_sensor_base)
        )
        corrected.pose.pose.orientation.x = q_odom_base[0]
        corrected.pose.pose.orientation.y = q_odom_base[1]
        corrected.pose.pose.orientation.z = q_odom_base[2]
        corrected.pose.pose.orientation.w = q_odom_base[3]
        corrected.pose.covariance = msg.pose.covariance

        linear = rotate_vector(
            self.q_base_sensor,
            (msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z),
        )
        angular = rotate_vector(
            self.q_base_sensor,
            (msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z),
        )
        corrected.twist.twist.linear.x, corrected.twist.twist.linear.y, corrected.twist.twist.linear.z = linear
        corrected.twist.twist.angular.x, corrected.twist.twist.angular.y, corrected.twist.twist.angular.z = angular
        corrected.twist.covariance = rotate_twist_covariance(
            msg.twist.covariance, self.q_base_sensor
        )
        self.publisher.publish(corrected)


def main(args=None):
    rclpy.init(args=args)
    node = BaseLinkOdometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
