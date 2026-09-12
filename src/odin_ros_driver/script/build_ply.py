#!/usr/bin/env python3
"""Accumulate a PointCloud2 topic into a voxel-filtered PLY file."""

import argparse
import math
import struct
import tempfile
import time
from pathlib import Path
from types import SimpleNamespace


FLOAT32 = 7
UINT32 = 6


def read_points(message):
    fields = {field.name: field for field in message.fields}
    missing = {"x", "y", "z"} - fields.keys()
    if missing:
        raise ValueError(f"PointCloud2 is missing fields: {', '.join(sorted(missing))}")
    if any(fields[name].datatype != FLOAT32 for name in ("x", "y", "z")):
        raise ValueError("x, y and z fields must use FLOAT32")

    color = fields.get("rgb") or fields.get("rgba")
    if color and color.datatype not in (FLOAT32, UINT32):
        color = None

    endian = ">" if message.is_bigendian else "<"
    unpack_float = struct.Struct(endian + "f").unpack_from
    unpack_color = struct.Struct(endian + "I").unpack_from
    data = memoryview(message.data)

    for row in range(message.height):
        row_start = row * message.row_step
        for column in range(message.width):
            start = row_start + column * message.point_step
            x = unpack_float(data, start + fields["x"].offset)[0]
            y = unpack_float(data, start + fields["y"].offset)[0]
            z = unpack_float(data, start + fields["z"].offset)[0]
            if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
                continue

            red = green = blue = 0
            if color:
                packed = unpack_color(data, start + color.offset)[0]
                red = (packed >> 16) & 0xFF
                green = (packed >> 8) & 0xFF
                blue = packed & 0xFF
            yield x, y, z, red, green, blue


def write_ply(path, points, with_color=False, ascii_output=False):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    properties = "property float x\nproperty float y\nproperty float z\n"
    if with_color:
        properties += "property uchar red\nproperty uchar green\nproperty uchar blue\n"
    encoding = "ascii" if ascii_output else "binary_little_endian"
    header = (
        f"ply\nformat {encoding} 1.0\nelement vertex {len(points)}\n"
        f"{properties}end_header\n"
    )

    if ascii_output:
        with path.open("w", encoding="ascii") as output:
            output.write(header)
            for x, y, z, red, green, blue in points:
                suffix = f" {red} {green} {blue}" if with_color else ""
                output.write(f"{x:.7g} {y:.7g} {z:.7g}{suffix}\n")
        return

    record = struct.Struct("<fffBBB" if with_color else "<fff")
    with path.open("wb") as output:
        output.write(header.encode("ascii"))
        for point in points:
            output.write(record.pack(*(point if with_color else point[:3])))


def self_test():
    field = lambda name, offset, datatype=FLOAT32: SimpleNamespace(
        name=name, offset=offset, datatype=datatype
    )
    packed_rgb = (12 << 16) | (34 << 8) | 56
    message = SimpleNamespace(
        fields=[field("x", 0), field("y", 4), field("z", 8), field("rgb", 12)],
        is_bigendian=False,
        height=1,
        width=2,
        point_step=16,
        row_step=32,
        data=struct.pack("<fffIfffI", 1.0, 2.0, 3.0, packed_rgb, math.nan, 0.0, 0.0, 0),
    )
    points = list(read_points(message))
    assert points == [(1.0, 2.0, 3.0, 12, 34, 56)]
    with tempfile.TemporaryDirectory() as directory:
        output = Path(directory) / "test.ply"
        write_ply(output, points, with_color=True)
        contents = output.read_bytes()
        assert b"element vertex 1\n" in contents
        assert contents.endswith(struct.pack("<fffBBB", *points[0]))
    print("build_ply self-test passed")


def parse_args():
    parser = argparse.ArgumentParser(
        description="Accumulate /odin1/cloud_slam into a static voxel-filtered PLY map."
    )
    parser.add_argument("output", nargs="?", default="map.ply", help="output .ply path")
    parser.add_argument("--topic", default="/odin1/cloud_slam")
    parser.add_argument("--voxel-size", type=float, default=0.05, help="voxel size in metres")
    parser.add_argument("--duration", type=float, default=0.0, help="seconds to record; 0 waits for Ctrl+C")
    parser.add_argument("--with-color", action="store_true", help="write RGB properties")
    parser.add_argument("--ascii", action="store_true", help="write larger ASCII PLY instead of binary")
    parser.add_argument("--self-test", action="store_true", help=argparse.SUPPRESS)
    args = parser.parse_args()
    if args.voxel_size <= 0:
        parser.error("--voxel-size must be greater than zero")
    if args.duration < 0:
        parser.error("--duration cannot be negative")
    if Path(args.output).suffix.lower() != ".ply":
        parser.error("output path must end in .ply")
    return args


def run(args):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import PointCloud2

    class Collector(Node):
        def __init__(self):
            super().__init__("build_ply")
            self.voxels = {}
            self.frames = 0
            self.input_points = 0
            self.frame_id = None
            self.create_subscription(PointCloud2, args.topic, self.collect, qos_profile_sensor_data)

        def collect(self, message):
            if self.frame_id is None:
                self.frame_id = message.header.frame_id
                self.get_logger().info(f"Collecting points in frame '{self.frame_id}'")
            elif message.header.frame_id != self.frame_id:
                self.get_logger().error(
                    f"Ignoring frame '{message.header.frame_id}'; expected '{self.frame_id}'"
                )
                return

            try:
                points = read_points(message)
                for point in points:
                    self.input_points += 1
                    key = tuple(math.floor(value / args.voxel_size) for value in point[:3])
                    self.voxels.setdefault(key, point)
            except (ValueError, struct.error) as error:
                self.get_logger().error(str(error))
                return

            self.frames += 1
            if self.frames % 10 == 0:
                self.get_logger().info(
                    f"Frames: {self.frames}, input points: {self.input_points}, "
                    f"retained voxels: {len(self.voxels)}"
                )

    rclpy.init()
    node = Collector()
    node.get_logger().info(
        f"Listening on {args.topic}; "
        + (f"recording for {args.duration:g} s" if args.duration else "press Ctrl+C to save")
    )
    started = time.monotonic()
    try:
        while rclpy.ok() and (not args.duration or time.monotonic() - started < args.duration):
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        points = list(node.voxels.values())
        if points:
            write_ply(args.output, points, args.with_color, args.ascii)
            node.get_logger().info(
                f"Saved {len(points)} points from {node.frames} frames to {Path(args.output).resolve()}"
            )
            exit_code = 0
        else:
            node.get_logger().error("No points received; no PLY file was written")
            exit_code = 1
        node.destroy_node()
        rclpy.shutdown()
    return exit_code


def main():
    args = parse_args()
    if args.self_test:
        self_test()
    else:
        raise SystemExit(run(args))


if __name__ == "__main__":
    main()
