#!/usr/bin/env python3
"""从 ROS 2 只读采集激光/地图与局部代价地图诊断数据。"""

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
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener


def yaw_from_quaternion(quaternion: Any) -> float:
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
    )


def transform_xy(x: np.ndarray, y: np.ndarray, transform: Any) -> tuple[np.ndarray, np.ndarray]:
    yaw = yaw_from_quaternion(transform.rotation)
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    return (
        transform.translation.x + cosine * x - sine * y,
        transform.translation.y + sine * x + cosine * y,
    )


def world_to_grid(
    x: np.ndarray,
    y: np.ndarray,
    origin_x: float,
    origin_y: float,
    origin_yaw: float,
    resolution: float,
) -> tuple[np.ndarray, np.ndarray]:
    delta_x = x - origin_x
    delta_y = y - origin_y
    cosine = math.cos(origin_yaw)
    sine = math.sin(origin_yaw)
    grid_x = (cosine * delta_x + sine * delta_y) / resolution
    grid_y = (-sine * delta_x + cosine * delta_y) / resolution
    return grid_x, grid_y


def grid_to_world(
    grid_x: np.ndarray,
    grid_y: np.ndarray,
    origin_x: float,
    origin_y: float,
    origin_yaw: float,
    resolution: float,
) -> tuple[np.ndarray, np.ndarray]:
    local_x = grid_x * resolution
    local_y = grid_y * resolution
    cosine = math.cos(origin_yaw)
    sine = math.sin(origin_yaw)
    return (
        origin_x + cosine * local_x - sine * local_y,
        origin_y + sine * local_x + cosine * local_y,
    )


def map_pixels(
    x: np.ndarray,
    y: np.ndarray,
    map_origin: tuple[float, float, float],
    resolution: float,
    height: int,
) -> tuple[np.ndarray, np.ndarray]:
    grid_x, grid_y = world_to_grid(x, y, *map_origin, resolution)
    return np.rint(grid_x).astype(np.int32), height - 1 - np.rint(grid_y).astype(np.int32)


def add_legend(image: np.ndarray, text: str) -> np.ndarray:
    output = image.copy()
    cv2.rectangle(output, (0, 0), (output.shape[1] - 1, 34), (245, 245, 245), -1)
    cv2.putText(output, text, (10, 23), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (20, 20, 20), 1, cv2.LINE_AA)
    return output


def angular_difference_degrees(first: float, second: float) -> float:
    return abs((first - second + 180.0) % 360.0 - 180.0)


def evaluate_alignment(
    scan_x: np.ndarray,
    scan_y: np.ndarray,
    pose_x: float,
    pose_y: float,
    pose_yaw: float,
    static_distance_m: np.ndarray,
    map_origin: tuple[float, float, float],
    map_resolution: float,
) -> dict[str, float | int]:
    cosine = math.cos(pose_yaw)
    sine = math.sin(pose_yaw)
    map_x = pose_x + cosine * scan_x - sine * scan_y
    map_y = pose_y + sine * scan_x + cosine * scan_y
    pixel_x, pixel_y = map_pixels(
        map_x, map_y, map_origin, map_resolution, static_distance_m.shape[0]
    )
    in_bounds = (
        (pixel_x >= 0)
        & (pixel_x < static_distance_m.shape[1])
        & (pixel_y >= 0)
        & (pixel_y < static_distance_m.shape[0])
    )
    distances = static_distance_m[pixel_y[in_bounds], pixel_x[in_bounds]]
    in_bounds_ratio = float(np.mean(in_bounds)) if in_bounds.size else 0.0
    if distances.size == 0:
        return {
            "score": 0.0,
            "count": 0,
            "in_bounds_ratio": in_bounds_ratio,
            "median_m": 999.0,
            "p90_m": 999.0,
            "within_0_10m": 0.0,
            "within_0_20m": 0.0,
        }
    sigma = 0.15
    likelihood = np.exp(-0.5 * np.square(distances / sigma))
    return {
        "score": float(np.mean(likelihood) * in_bounds_ratio),
        "count": int(distances.size),
        "in_bounds_ratio": in_bounds_ratio,
        "median_m": float(np.median(distances)),
        "p90_m": float(np.percentile(distances, 90)),
        "within_0_10m": float(np.mean(distances <= 0.10)),
        "within_0_20m": float(np.mean(distances <= 0.20)),
    }


def select_distinct_candidates(
    candidates: list[dict[str, float | int]],
    limit: int,
    minimum_xy_distance: float,
    minimum_yaw_distance: float,
) -> list[dict[str, float | int]]:
    selected: list[dict[str, float | int]] = []
    for candidate in sorted(candidates, key=lambda item: float(item["score"]), reverse=True):
        if all(
            math.hypot(
                float(candidate["x"]) - float(existing["x"]),
                float(candidate["y"]) - float(existing["y"]),
            )
            >= minimum_xy_distance
            or angular_difference_degrees(
                float(candidate["yaw_deg"]), float(existing["yaw_deg"])
            )
            >= minimum_yaw_distance
            for existing in selected
        ):
            selected.append(candidate)
            if len(selected) >= limit:
                break
    return selected


def search_alignment(
    scan_x: np.ndarray,
    scan_y: np.ndarray,
    current_x: float,
    current_y: float,
    current_yaw: float,
    static_distance_m: np.ndarray,
    map_origin: tuple[float, float, float],
    map_resolution: float,
    expected_yaw_deg: float | None = None,
    search_xy_m: float = 2.0,
) -> dict[str, Any]:
    current_yaw_deg = math.degrees(current_yaw)

    def candidate(x: float, y: float, yaw_deg: float) -> dict[str, float | int]:
        metrics = evaluate_alignment(
            scan_x,
            scan_y,
            x,
            y,
            math.radians(yaw_deg),
            static_distance_m,
            map_origin,
            map_resolution,
        )
        return {
            "x": round(x, 4),
            "y": round(y, 4),
            "yaw_deg": round((yaw_deg + 180.0) % 360.0 - 180.0, 4),
            **{key: round(float(value), 6) if isinstance(value, float) else value for key, value in metrics.items()},
        }

    if expected_yaw_deg is None:
        coarse_yaws = current_yaw_deg + np.arange(-180.0, 180.0, 10.0)
        coarse_yaw_range = 180.0
        coarse_yaw_step = 10.0
    else:
        coarse_yaws = expected_yaw_deg + np.arange(-30.0, 30.001, 5.0)
        coarse_yaw_range = 30.0
        coarse_yaw_step = 5.0

    coarse: list[dict[str, float | int]] = []
    for yaw_deg in coarse_yaws:
        for dx in np.arange(-search_xy_m, search_xy_m + 0.001, 0.2):
            for dy in np.arange(-search_xy_m, search_xy_m + 0.001, 0.2):
                coarse.append(candidate(current_x + float(dx), current_y + float(dy), yaw_deg))

    coarse_seeds = select_distinct_candidates(coarse, 5, 0.5, 15.0)
    refined: list[dict[str, float | int]] = []
    for seed in coarse_seeds:
        for yaw_offset in np.arange(-8.0, 8.001, 1.0):
            for dx in np.arange(-0.30, 0.301, 0.03):
                for dy in np.arange(-0.30, 0.301, 0.03):
                    refined.append(
                        candidate(
                            float(seed["x"]) + float(dx),
                            float(seed["y"]) + float(dy),
                            float(seed["yaw_deg"]) + float(yaw_offset),
                        )
                    )
    best_candidates = select_distinct_candidates(refined, 10, 0.15, 3.0)
    current = candidate(current_x, current_y, current_yaw_deg)
    return {
        "search_bounds": {
            "coarse_xy_m": search_xy_m,
            "coarse_xy_step_m": 0.2,
            "coarse_yaw_center_deg": expected_yaw_deg,
            "coarse_yaw_range_deg": coarse_yaw_range,
            "coarse_yaw_step_deg": coarse_yaw_step,
            "refine_xy_m": 0.30,
            "refine_xy_step_m": 0.03,
            "refine_yaw_deg": 8.0,
            "refine_yaw_step_deg": 1.0,
        },
        "current": current,
        "coarse_seeds": coarse_seeds,
        "candidates": best_candidates,
    }


def draw_alignment_candidate(
    map_color: np.ndarray,
    scan_x: np.ndarray,
    scan_y: np.ndarray,
    candidate: dict[str, float | int],
    current_pose: tuple[float, float, float],
    map_origin: tuple[float, float, float],
    map_resolution: float,
    label: str,
) -> np.ndarray:
    pose_x = float(candidate["x"])
    pose_y = float(candidate["y"])
    pose_yaw = math.radians(float(candidate["yaw_deg"]))
    cosine = math.cos(pose_yaw)
    sine = math.sin(pose_yaw)
    point_x = pose_x + cosine * scan_x - sine * scan_y
    point_y = pose_y + sine * scan_x + cosine * scan_y
    pixel_x, pixel_y = map_pixels(
        point_x, point_y, map_origin, map_resolution, map_color.shape[0]
    )
    in_bounds = (
        (pixel_x >= 0)
        & (pixel_x < map_color.shape[1])
        & (pixel_y >= 0)
        & (pixel_y < map_color.shape[0])
    )
    output = map_color.copy()
    for px, py in zip(pixel_x[in_bounds], pixel_y[in_bounds], strict=False):
        cv2.circle(output, (int(px), int(py)), 2, (0, 255, 255), -1)

    current_px, current_py = map_pixels(
        np.asarray([current_pose[0]]),
        np.asarray([current_pose[1]]),
        map_origin,
        map_resolution,
        map_color.shape[0],
    )
    candidate_px, candidate_py = map_pixels(
        np.asarray([pose_x]),
        np.asarray([pose_y]),
        map_origin,
        map_resolution,
        map_color.shape[0],
    )
    cv2.circle(output, (int(current_px[0]), int(current_py[0])), 7, (0, 0, 255), -1)
    candidate_point = (int(candidate_px[0]), int(candidate_py[0]))
    cv2.circle(output, candidate_point, 7, (0, 180, 0), -1)
    arrow_length = max(18, int(round(0.8 / map_resolution)))
    arrow_end = (
        int(round(candidate_point[0] + arrow_length * math.cos(pose_yaw))),
        int(round(candidate_point[1] - arrow_length * math.sin(pose_yaw))),
    )
    cv2.arrowedLine(output, candidate_point, arrow_end, (255, 0, 255), 3, tipLength=0.3)
    return add_legend(output, label)


class DiagnosticCapture(Node):
    def __init__(self) -> None:
        super().__init__("capture_navigation_diagnostics")
        self.scan: LaserScan | None = None
        self.costmap: OccupancyGrid | None = None
        scan_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        costmap_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(LaserScan, "/merged", self._scan_callback, scan_qos)
        self.create_subscription(
            OccupancyGrid,
            "/local_costmap/costmap",
            self._costmap_callback,
            costmap_qos,
        )
        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def _scan_callback(self, message: LaserScan) -> None:
        self.scan = message

    def _costmap_callback(self, message: OccupancyGrid) -> None:
        self.costmap = message

    def wait_for_inputs(self, timeout: float) -> None:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and (self.scan is None or self.costmap is None):
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.scan is None:
            raise RuntimeError("timed out waiting for /merged")
        if self.costmap is None:
            raise RuntimeError("timed out waiting for /local_costmap/costmap")

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


def capture(args: argparse.Namespace) -> dict[str, Any]:
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    map_yaml_path = Path(args.map_yaml)
    map_metadata = yaml.safe_load(map_yaml_path.read_text(encoding="utf-8"))
    image_path = Path(map_metadata["image"])
    if not image_path.is_absolute():
        image_path = map_yaml_path.parent / image_path
    map_gray = cv2.imread(str(image_path), cv2.IMREAD_GRAYSCALE)
    if map_gray is None:
        raise RuntimeError(f"cannot read map image: {image_path}")
    map_color = cv2.cvtColor(map_gray, cv2.COLOR_GRAY2BGR)
    map_height, map_width = map_gray.shape
    map_resolution = float(map_metadata["resolution"])
    map_origin = tuple(float(value) for value in map_metadata["origin"])

    if int(map_metadata.get("negate", 0)):
        occupancy_probability = map_gray.astype(np.float32) / 255.0
    else:
        occupancy_probability = (255.0 - map_gray.astype(np.float32)) / 255.0
    static_occupied = occupancy_probability >= float(map_metadata["occupied_thresh"])
    static_distance_m = cv2.distanceTransform(
        (~static_occupied).astype(np.uint8), cv2.DIST_L2, 5
    ) * map_resolution

    node = DiagnosticCapture()
    try:
        node.wait_for_inputs(args.timeout)
        assert node.scan is not None
        assert node.costmap is not None
        scan = node.scan
        costmap = node.costmap

        map_from_scan = node.lookup("map", scan.header.frame_id)
        map_from_base = node.lookup("map", "base_link")
        map_from_costmap = node.lookup("map", costmap.header.frame_id)
        costmap_from_scan = node.lookup(costmap.header.frame_id, scan.header.frame_id)
        costmap_from_base = node.lookup(costmap.header.frame_id, "base_link")

        ranges = np.asarray(scan.ranges, dtype=np.float64)
        angles = scan.angle_min + np.arange(ranges.size, dtype=np.float64) * scan.angle_increment
        valid = np.isfinite(ranges) & (ranges >= scan.range_min) & (ranges <= scan.range_max)
        scan_x = ranges[valid] * np.cos(angles[valid])
        scan_y = ranges[valid] * np.sin(angles[valid])
        laser_map_x, laser_map_y = transform_xy(scan_x, scan_y, map_from_scan)
        laser_px, laser_py = map_pixels(
            laser_map_x, laser_map_y, map_origin, map_resolution, map_height
        )
        laser_in_map = (
            (laser_px >= 0)
            & (laser_px < map_width)
            & (laser_py >= 0)
            & (laser_py < map_height)
        )
        laser_wall_distances = static_distance_m[laser_py[laser_in_map], laser_px[laser_in_map]]

        laser_overlay = map_color.copy()
        robot_x = np.asarray([map_from_base.translation.x])
        robot_y = np.asarray([map_from_base.translation.y])
        robot_px, robot_py = map_pixels(robot_x, robot_y, map_origin, map_resolution, map_height)
        for px, py in zip(laser_px[laser_in_map], laser_py[laser_in_map], strict=False):
            cv2.circle(laser_overlay, (int(px), int(py)), 2, (0, 255, 255), -1)
        robot_point = (int(robot_px[0]), int(robot_py[0]))
        robot_yaw = yaw_from_quaternion(map_from_base.rotation)
        arrow_length = max(18, int(round(0.8 / map_resolution)))
        arrow_end = (
            int(round(robot_point[0] + arrow_length * math.cos(robot_yaw))),
            int(round(robot_point[1] - arrow_length * math.sin(robot_yaw))),
        )
        cv2.circle(laser_overlay, robot_point, 7, (0, 0, 255), -1)
        cv2.arrowedLine(laser_overlay, robot_point, arrow_end, (255, 0, 255), 3, tipLength=0.3)
        laser_overlay = add_legend(
            laser_overlay,
            "Yellow: laser endpoints   Red/magenta: robot pose/heading   Black: static walls",
        )
        cv2.imwrite(str(output_dir / "laser_map_overlay.png"), laser_overlay)

        alignment_search: dict[str, Any] | None = None
        if args.search:
            alignment_search = search_alignment(
                scan_x,
                scan_y,
                map_from_base.translation.x,
                map_from_base.translation.y,
                robot_yaw,
                static_distance_m,
                map_origin,
                map_resolution,
                args.expected_yaw_deg,
                args.search_xy_m,
            )
            best_alignment = alignment_search["candidates"][0]
            opposite_yaw = (float(best_alignment["yaw_deg"]) + 360.0) % 360.0 - 180.0
            opposite_metrics = evaluate_alignment(
                scan_x,
                scan_y,
                float(best_alignment["x"]),
                float(best_alignment["y"]),
                math.radians(opposite_yaw),
                static_distance_m,
                map_origin,
                map_resolution,
            )
            opposite_candidate: dict[str, float | int] = {
                "x": float(best_alignment["x"]),
                "y": float(best_alignment["y"]),
                "yaw_deg": round(opposite_yaw, 4),
                **{
                    key: round(float(value), 6) if isinstance(value, float) else value
                    for key, value in opposite_metrics.items()
                },
            }
            alignment_search["opposite_of_best"] = opposite_candidate
            for index, alignment_candidate in enumerate(
                alignment_search["candidates"][:3], start=1
            ):
                label = (
                    f"Candidate {index}: x={alignment_candidate['x']:.3f} "
                    f"y={alignment_candidate['y']:.3f} "
                    f"yaw={alignment_candidate['yaw_deg']:.1f} deg   "
                    f"score={alignment_candidate['score']:.3f} "
                    f"within20cm={alignment_candidate['within_0_20m']:.1%}   "
                    "Yellow: laser   Green: candidate   Red: current"
                )
                candidate_image = draw_alignment_candidate(
                    map_color,
                    scan_x,
                    scan_y,
                    alignment_candidate,
                    (
                        map_from_base.translation.x,
                        map_from_base.translation.y,
                        robot_yaw,
                    ),
                    map_origin,
                    map_resolution,
                    label,
                )
                cv2.imwrite(
                    str(output_dir / f"alignment_candidate_{index}.png"), candidate_image
                )
            opposite_image = draw_alignment_candidate(
                map_color,
                scan_x,
                scan_y,
                opposite_candidate,
                (
                    map_from_base.translation.x,
                    map_from_base.translation.y,
                    robot_yaw,
                ),
                map_origin,
                map_resolution,
                (
                    f"Opposite heading: x={opposite_candidate['x']:.3f} "
                    f"y={opposite_candidate['y']:.3f} "
                    f"yaw={opposite_candidate['yaw_deg']:.1f} deg   "
                    f"score={opposite_candidate['score']:.3f} "
                    f"within20cm={opposite_candidate['within_0_20m']:.1%}"
                ),
            )
            cv2.imwrite(str(output_dir / "alignment_opposite_heading.png"), opposite_image)
            (output_dir / "alignment_search.json").write_text(
                json.dumps(alignment_search, ensure_ascii=False, indent=2) + "\n",
                encoding="utf-8",
            )

        cost_width = int(costmap.info.width)
        cost_height = int(costmap.info.height)
        costs = np.asarray(costmap.data, dtype=np.int16).reshape((cost_height, cost_width))
        cost_image = np.full((cost_height, cost_width, 3), (128, 128, 128), dtype=np.uint8)
        free = costs == 0
        inflated = (costs > 0) & (costs < 99)
        lethal = costs >= 99
        cost_image[free] = (255, 255, 255)
        if np.any(inflated):
            intensity = np.clip(costs[inflated] * 2, 0, 220).astype(np.uint8)
            cost_image[inflated] = np.column_stack(
                (
                    np.zeros_like(intensity),
                    255 - intensity // 2,
                    np.full_like(intensity, 255),
                )
            )
        cost_image[lethal] = (0, 0, 255)
        cost_image = np.flipud(cost_image).copy()

        cost_origin = costmap.info.origin
        cost_origin_yaw = yaw_from_quaternion(cost_origin.orientation)
        base_cost_x = np.asarray([costmap_from_base.translation.x])
        base_cost_y = np.asarray([costmap_from_base.translation.y])
        base_grid_x, base_grid_y = world_to_grid(
            base_cost_x,
            base_cost_y,
            cost_origin.position.x,
            cost_origin.position.y,
            cost_origin_yaw,
            costmap.info.resolution,
        )
        base_pixel = (
            int(round(base_grid_x[0])),
            cost_height - 1 - int(round(base_grid_y[0])),
        )
        if 0 <= base_pixel[0] < cost_width and 0 <= base_pixel[1] < cost_height:
            cv2.circle(cost_image, base_pixel, 5, (255, 0, 255), -1)
        scale = max(2, min(6, 900 // max(cost_width, cost_height)))
        cost_image = cv2.resize(
            cost_image, (cost_width * scale, cost_height * scale), interpolation=cv2.INTER_NEAREST
        )
        cost_image = add_legend(
            cost_image,
            "Red: lethal obstacles   Orange: inflation   White: free   Gray: unknown   Magenta: robot",
        )
        cv2.imwrite(str(output_dir / "local_costmap.png"), cost_image)

        cell_y, cell_x = np.nonzero(costs > 0)
        cell_center_x = cell_x.astype(np.float64) + 0.5
        cell_center_y = cell_y.astype(np.float64) + 0.5
        cell_frame_x, cell_frame_y = grid_to_world(
            cell_center_x,
            cell_center_y,
            cost_origin.position.x,
            cost_origin.position.y,
            cost_origin_yaw,
            costmap.info.resolution,
        )
        occupied_map_x, occupied_map_y = transform_xy(
            cell_frame_x, cell_frame_y, map_from_costmap
        )
        occupied_px, occupied_py = map_pixels(
            occupied_map_x, occupied_map_y, map_origin, map_resolution, map_height
        )
        occupied_in_map = (
            (occupied_px >= 0)
            & (occupied_px < map_width)
            & (occupied_py >= 0)
            & (occupied_py < map_height)
        )
        overlay = map_color.copy()
        cell_cost_values = costs[cell_y, cell_x]
        for px, py, cost in zip(
            occupied_px[occupied_in_map],
            occupied_py[occupied_in_map],
            cell_cost_values[occupied_in_map],
            strict=False,
        ):
            color = (0, 0, 255) if cost >= 99 else (0, 165, 255)
            cv2.circle(overlay, (int(px), int(py)), 1, color, -1)
        cv2.circle(overlay, robot_point, 7, (255, 0, 255), -1)
        overlay = add_legend(
            overlay,
            "Local costmap on static map   Red: lethal   Orange: inflation   Magenta: robot",
        )
        cv2.imwrite(str(output_dir / "costmap_map_overlay.png"), overlay)

        laser_cost_x, laser_cost_y = transform_xy(scan_x, scan_y, costmap_from_scan)
        laser_cost_grid_x, laser_cost_grid_y = world_to_grid(
            laser_cost_x,
            laser_cost_y,
            cost_origin.position.x,
            cost_origin.position.y,
            cost_origin_yaw,
            costmap.info.resolution,
        )
        laser_cost_col = np.floor(laser_cost_grid_x).astype(np.int32)
        laser_cost_row = np.floor(laser_cost_grid_y).astype(np.int32)
        laser_in_costmap = (
            (laser_cost_col >= 0)
            & (laser_cost_col < cost_width)
            & (laser_cost_row >= 0)
            & (laser_cost_row < cost_height)
        )
        endpoint_costs = costs[
            laser_cost_row[laser_in_costmap], laser_cost_col[laser_in_costmap]
        ]

        lethal_y, lethal_x = np.nonzero(lethal)
        lethal_frame_x, lethal_frame_y = grid_to_world(
            lethal_x.astype(np.float64) + 0.5,
            lethal_y.astype(np.float64) + 0.5,
            cost_origin.position.x,
            cost_origin.position.y,
            cost_origin_yaw,
            costmap.info.resolution,
        )
        lethal_map_x, lethal_map_y = transform_xy(
            lethal_frame_x, lethal_frame_y, map_from_costmap
        )
        lethal_px, lethal_py = map_pixels(
            lethal_map_x, lethal_map_y, map_origin, map_resolution, map_height
        )
        lethal_in_map = (
            (lethal_px >= 0)
            & (lethal_px < map_width)
            & (lethal_py >= 0)
            & (lethal_py < map_height)
        )
        lethal_wall_distances = static_distance_m[
            lethal_py[lethal_in_map], lethal_px[lethal_in_map]
        ]

        def distance_metrics(values: np.ndarray) -> dict[str, float | int | None]:
            if values.size == 0:
                return {"count": 0, "median_m": None, "p90_m": None, "within_0_10m": None, "within_0_20m": None}
            return {
                "count": int(values.size),
                "median_m": round(float(np.median(values)), 4),
                "p90_m": round(float(np.percentile(values, 90)), 4),
                "within_0_10m": round(float(np.mean(values <= 0.10)), 4),
                "within_0_20m": round(float(np.mean(values <= 0.20)), 4),
            }

        artifacts = [
            "laser_map_overlay.png",
            "local_costmap.png",
            "costmap_map_overlay.png",
        ]
        if alignment_search is not None:
            artifacts.extend(
                [
                    "alignment_candidate_1.png",
                    "alignment_candidate_2.png",
                    "alignment_candidate_3.png",
                    "alignment_opposite_heading.png",
                    "alignment_search.json",
                ]
            )

        result = {
            "captured_unix": time.time(),
            "map": {
                "yaml": str(map_yaml_path),
                "image": str(image_path),
                "width": map_width,
                "height": map_height,
                "resolution": map_resolution,
                "origin": map_origin,
            },
            "robot_pose_map": {
                "x": map_from_base.translation.x,
                "y": map_from_base.translation.y,
                "yaw_deg": math.degrees(robot_yaw),
            },
            "laser": {
                "topic": "/merged",
                "frame": scan.header.frame_id,
                "valid_endpoints": int(scan_x.size),
                "endpoints_in_static_map": int(np.count_nonzero(laser_in_map)),
                "endpoint_to_static_wall": distance_metrics(laser_wall_distances),
                "endpoints_in_local_costmap": int(endpoint_costs.size),
                "endpoint_cost_class": {
                    "unknown": int(np.count_nonzero(endpoint_costs < 0)),
                    "free": int(np.count_nonzero(endpoint_costs == 0)),
                    "inflated": int(np.count_nonzero((endpoint_costs > 0) & (endpoint_costs < 99))),
                    "lethal": int(np.count_nonzero(endpoint_costs >= 99)),
                },
            },
            "local_costmap": {
                "topic": "/local_costmap/costmap",
                "frame": costmap.header.frame_id,
                "width": cost_width,
                "height": cost_height,
                "resolution": costmap.info.resolution,
                "cell_counts": {
                    "unknown": int(np.count_nonzero(costs < 0)),
                    "free": int(np.count_nonzero(free)),
                    "inflated": int(np.count_nonzero(inflated)),
                    "lethal": int(np.count_nonzero(lethal)),
                },
                "lethal_cells_to_static_wall": distance_metrics(lethal_wall_distances),
            },
            "alignment_search": alignment_search,
            "artifacts": artifacts,
        }
        (output_dir / "diagnostics.json").write_text(
            json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
        )
        return result
    finally:
        node.destroy_node()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--map-yaml", required=True)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--timeout", type=float, default=10.0)
    parser.add_argument("--search", action="store_true")
    parser.add_argument("--expected-yaw-deg", type=float)
    parser.add_argument("--search-xy-m", type=float, default=2.0)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    rclpy.init()
    try:
        result = capture(args)
        print(json.dumps(result, ensure_ascii=False, indent=2))
        return 0
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
