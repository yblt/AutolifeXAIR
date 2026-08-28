#!/usr/bin/env python3
"""分别采集前/后激光雷达叠加图，不发布任何 ROS 指令。"""

from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import rclpy
import yaml
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener


def transform_xyz_plane(
    x: np.ndarray, y: np.ndarray, transform: Any
) -> tuple[np.ndarray, np.ndarray]:
    q = transform.rotation
    r00 = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    r01 = 2.0 * (q.x * q.y - q.z * q.w)
    r10 = 2.0 * (q.x * q.y + q.z * q.w)
    r11 = 1.0 - 2.0 * (q.x * q.x + q.z * q.z)
    return (
        transform.translation.x + r00 * x + r01 * y,
        transform.translation.y + r10 * x + r11 * y,
    )


def map_pixels(
    x: np.ndarray,
    y: np.ndarray,
    origin: tuple[float, float, float],
    resolution: float,
    height: int,
) -> tuple[np.ndarray, np.ndarray]:
    dx = x - origin[0]
    dy = y - origin[1]
    cosine = math.cos(origin[2])
    sine = math.sin(origin[2])
    grid_x = (cosine * dx + sine * dy) / resolution
    grid_y = (-sine * dx + cosine * dy) / resolution
    return np.rint(grid_x).astype(np.int32), height - 1 - np.rint(grid_y).astype(np.int32)


def metrics(values: np.ndarray) -> dict[str, float | int | None]:
    if values.size == 0:
        return {"count": 0, "median_m": None, "p90_m": None, "within_0_20m": None}
    return {
        "count": int(values.size),
        "median_m": round(float(np.median(values)), 4),
        "p90_m": round(float(np.percentile(values, 90)), 4),
        "within_0_20m": round(float(np.mean(values <= 0.20)), 4),
    }


class Capture(Node):
    def __init__(self, topics: dict[str, str]) -> None:
        super().__init__("capture_split_lidar_overlay")
        self.scans: dict[str, LaserScan] = {}
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._scan_subscriptions = [
            self.create_subscription(LaserScan, topic, self.callback(name), qos)
            for name, topic in topics.items()
        ]
        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def callback(self, name: str):
        def receive(message: LaserScan) -> None:
            self.scans[name] = message

        return receive

    def wait(self, names: set[str], timeout: float) -> None:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and not names.issubset(self.scans):
            rclpy.spin_once(self, timeout_sec=0.1)
        missing = names.difference(self.scans)
        if missing:
            raise RuntimeError(f"timed out waiting for: {sorted(missing)}")

    def lookup(self, target: str, source: str, timeout: float = 5.0) -> Any:
        deadline = time.monotonic() + timeout
        last_error: Exception | None = None
        while time.monotonic() < deadline:
            try:
                return self.tf_buffer.lookup_transform(target, source, Time()).transform
            except TransformException as exc:
                last_error = exc
                rclpy.spin_once(self, timeout_sec=0.1)
        raise RuntimeError(f"cannot transform {source} -> {target}: {last_error}")


def scan_points(scan: LaserScan) -> tuple[np.ndarray, np.ndarray]:
    ranges = np.asarray(scan.ranges, dtype=np.float64)
    angles = scan.angle_min + np.arange(ranges.size, dtype=np.float64) * scan.angle_increment
    valid = np.isfinite(ranges) & (ranges >= scan.range_min) & (ranges <= scan.range_max)
    return ranges[valid] * np.cos(angles[valid]), ranges[valid] * np.sin(angles[valid])


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--map-yaml", required=True)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--front-topic", required=True)
    parser.add_argument("--rear-topic", required=True)
    parser.add_argument("--timeout", type=float, default=10.0)
    args = parser.parse_args()

    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    map_yaml = Path(args.map_yaml)
    metadata = yaml.safe_load(map_yaml.read_text(encoding="utf-8"))
    image_path = Path(metadata["image"])
    if not image_path.is_absolute():
        image_path = map_yaml.parent / image_path
    map_gray = cv2.imread(str(image_path), cv2.IMREAD_GRAYSCALE)
    if map_gray is None:
        raise RuntimeError(f"cannot read map image: {image_path}")
    map_color = cv2.cvtColor(map_gray, cv2.COLOR_GRAY2BGR)
    height, width = map_gray.shape
    resolution = float(metadata["resolution"])
    origin = tuple(float(value) for value in metadata["origin"])
    occupancy = (255.0 - map_gray.astype(np.float32)) / 255.0
    occupied = occupancy >= float(metadata["occupied_thresh"])
    wall_distance = cv2.distanceTransform((~occupied).astype(np.uint8), cv2.DIST_L2, 5) * resolution

    topics = {"front": args.front_topic, "rear": args.rear_topic}
    rclpy.init()
    node = Capture(topics)
    try:
        node.wait(set(topics), args.timeout)
        overlays: dict[str, np.ndarray] = {}
        result: dict[str, Any] = {"map": str(map_yaml), "sensors": {}}
        combined = map_color.copy()
        colors = {"front": (0, 255, 255), "rear": (255, 255, 0)}
        for name in ("front", "rear"):
            scan = node.scans[name]
            scan_x, scan_y = scan_points(scan)
            base_from_scan = node.lookup("base_link", scan.header.frame_id)
            map_from_scan = node.lookup("map", scan.header.frame_id)
            base_x, base_y = transform_xyz_plane(scan_x, scan_y, base_from_scan)
            map_x, map_y = transform_xyz_plane(scan_x, scan_y, map_from_scan)
            px, py = map_pixels(map_x, map_y, origin, resolution, height)
            in_map = (px >= 0) & (px < width) & (py >= 0) & (py < height)
            distances = wall_distance[py[in_map], px[in_map]]
            left = in_map & (base_y > 0.0)
            left_distances = wall_distance[py[left], px[left]]
            overlay = map_color.copy()
            for x_pixel, y_pixel in zip(px[in_map], py[in_map], strict=False):
                cv2.circle(overlay, (int(x_pixel), int(y_pixel)), 2, colors[name], -1)
                cv2.circle(combined, (int(x_pixel), int(y_pixel)), 2, colors[name], -1)
            cv2.putText(
                overlay,
                f"{name}: all={metrics(distances)['within_0_20m']} left={metrics(left_distances)['within_0_20m']}",
                (10, 24),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (20, 20, 20),
                1,
                cv2.LINE_AA,
            )
            cv2.imwrite(str(output_dir / f"{name}_laser_map_overlay.png"), overlay)
            overlays[name] = overlay
            result["sensors"][name] = {
                "topic": topics[name],
                "frame": scan.header.frame_id,
                "valid_endpoints": int(scan_x.size),
                "all_endpoints_to_wall": metrics(distances),
                "robot_left_endpoints_to_wall": metrics(left_distances),
                "base_transform": {
                    "x": base_from_scan.translation.x,
                    "y": base_from_scan.translation.y,
                    "quaternion": [
                        base_from_scan.rotation.x,
                        base_from_scan.rotation.y,
                        base_from_scan.rotation.z,
                        base_from_scan.rotation.w,
                    ],
                },
            }
        cv2.putText(
            combined,
            "Yellow: front lidar   Cyan: rear lidar   Black: static walls",
            (10, 24),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (20, 20, 20),
            1,
            cv2.LINE_AA,
        )
        cv2.imwrite(str(output_dir / "front_rear_combined_overlay.png"), combined)
        (output_dir / "split_lidar_diagnostics.json").write_text(
            json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
        )
        print(json.dumps(result, ensure_ascii=False, indent=2))
        return 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
