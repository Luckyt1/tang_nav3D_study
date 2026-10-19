"""Shared launch-time sensor configuration helpers."""

from __future__ import annotations

import math
from pathlib import Path

import yaml


ODIN_BACKEND = "odin"
MID360_BACKEND = "mid360"


ODIN_SENSOR = {
    "raw_odom_topic": "/odin1/odometry",
    "base_odom_topic": "/base_link/odometry",
    "cloud_topic": "/odin1/cloud_slam",
    "sensor_frame": "lidar",
    "raw_odom_frame": "imu",
    "mount_rpy": (0.0, math.pi / 4.0, 0.0),
    "cloud_is_world": True,
    "need_extrinsic": False,
}


MID360_SENSOR = {
    "raw_odom_topic": "/aft_mapped_to_init",
    "base_odom_topic": "/base_link/odometry",
    "cloud_topic": "/cloud_registered",
    "sensor_frame": "livox_frame",
    "raw_odom_frame": "body_raw",
    "world_frame": "odom",
    "mount_rpy": (math.pi, 0.0, 0.0),
    "cloud_is_world": True,
    "need_extrinsic": True,
}


def normalize_backend(value: str) -> str:
    backend = value.strip().lower()
    if backend not in (ODIN_BACKEND, MID360_BACKEND):
        raise ValueError("sensor_backend must be 'odin' or 'mid360'")
    return backend


def parse_optional_float(value: str, fallback: float) -> float:
    text = str(value).strip().lower()
    if text in ("", "auto"):
        return fallback
    parsed = float(text)
    if not math.isfinite(parsed):
        raise ValueError("mounting angles must be finite")
    return parsed


def resolve_mount_rpy(backend: str, roll: str, pitch: str, yaw: str) -> tuple[float, float, float]:
    defaults = (MID360_SENSOR if backend == MID360_BACKEND else ODIN_SENSOR)["mount_rpy"]
    return (
        parse_optional_float(roll, defaults[0]),
        parse_optional_float(pitch, defaults[1]),
        parse_optional_float(yaw, defaults[2]),
    )


def _ros_parameters(data: object) -> dict:
    if not isinstance(data, dict):
        return {}
    if "/**" in data and isinstance(data["/**"], dict):
        return data["/**"].get("ros__parameters", {}) or {}
    return data.get("ros__parameters", data) or {}


def load_point_lio_config(path: str) -> dict:
    config_path = Path(path).expanduser().resolve()
    if not config_path.is_file():
        raise ValueError(f"Point-LIO config does not exist: {config_path}")
    with config_path.open("r", encoding="utf-8") as stream:
        params = _ros_parameters(yaml.safe_load(stream))
    mapping = params.get("mapping", {}) or {}
    extrinsic_t = _float_list(mapping.get("extrinsic_T"), 3, "mapping.extrinsic_T")
    extrinsic_r = _float_list(mapping.get("extrinsic_R"), 9, "mapping.extrinsic_R")
    rows = [extrinsic_r[index:index + 3] for index in (0, 3, 6)]
    determinant = (
        rows[0][0] * (rows[1][1] * rows[2][2] - rows[1][2] * rows[2][1])
        - rows[0][1] * (rows[1][0] * rows[2][2] - rows[1][2] * rows[2][0])
        + rows[0][2] * (rows[1][0] * rows[2][1] - rows[1][1] * rows[2][0])
    )
    if abs(determinant - 1.0) > 1e-3 or any(
        abs(sum(rows[i][k] * rows[j][k] for k in range(3)) - (1.0 if i == j else 0.0)) > 1e-3
        for i in range(3) for j in range(3)
    ):
        raise ValueError("mapping.extrinsic_R must be an orthonormal rotation matrix")
    return {
        "path": str(config_path),
        "extrinsic_T": extrinsic_t,
        "extrinsic_R": extrinsic_r,
    }


def _float_list(value: object, size: int, name: str) -> list[float]:
    if not isinstance(value, list) or len(value) != size:
        raise ValueError(f"{name} must contain {size} numeric values")
    parsed = [float(item) for item in value]
    if not all(math.isfinite(item) for item in parsed):
        raise ValueError(f"{name} must contain only finite values")
    return parsed


def matrix_to_rpy(matrix: list[float]) -> tuple[float, float, float]:
    r00, r01, r02, r10, r11, r12, r20, r21, r22 = matrix
    sy = math.hypot(r00, r10)
    if sy > 1e-9:
        roll = math.atan2(r21, r22)
        pitch = math.atan2(-r20, sy)
        yaw = math.atan2(r10, r00)
    else:
        roll = math.atan2(-r12, r11)
        pitch = math.atan2(-r20, sy)
        yaw = 0.0
    return roll, pitch, yaw


def point_lio_overrides() -> dict:
    return {
        "common.lid_topic": "/livox/lidar",
        "common.imu_topic": "/livox/imu",
        "preprocess.lidar_type": 1,
        "mapping.extrinsic_est_en": False,
        "publish.world_frame": "odom",
        "publish.body_frame": "body_raw",
        "publish.scan_publish_en": True,
        "publish.tf_send_en": True,
        "odometry.publish_odometry_without_downsample": False,
        "pcd_save.pcd_save_en": False,
    }
