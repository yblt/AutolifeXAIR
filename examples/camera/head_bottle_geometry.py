#!/usr/bin/env python3
"""像素 + 深度 -> 相机光学坐标 3D -> 机器人基座坐标的几何计算。

头部相机瓶子定位第 1 步的纯数值原语，零依赖（见
``openspec/changes/add-head-camera-assisted-positioning``）。本模块不
依赖 ROS、``autolife`` SDK、OpenCV 或 NumPy：可以完全离线 import 和
单测。

``backproject_pixel``、``validate_transform`` 和
``transform_point_to_base`` 是数值原语，输入非法时"可以"抛出
``ValueError``。唯一的顶层入口 :func:`locate_bottle_in_base` 是
fail-closed 的：它从不抛异常，一切失败——校验失败或意外错误——都通过
返回的 :class:`BottlePositionResult` 报告。

这里消费的 4x4 基座<-相机变换应来自机上
``CameraTransformer.compute_object_pose_in_base()`` 调用。本模块不
自行计算正向运动学或 camera_link->光学坐标的约定；它只校验并应用
已经算好的矩阵。
"""

from __future__ import annotations

import math
import numbers
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from typing import Any


def _json_value(value: Any) -> Any:
    """把结果元数据转成普通的 JSON 兼容值。"""

    if value is None or isinstance(value, (bool, int, float, str)):
        return value
    if isinstance(value, Mapping):
        return {str(key): _json_value(item) for key, item in value.items()}
    if isinstance(value, Sequence) and not isinstance(value, (str, bytes, bytearray)):
        return [_json_value(item) for item in value]
    item = getattr(value, "item", None)
    if callable(item):
        try:
            return _json_value(item())
        except Exception:
            pass
    return str(value)


def _finite_real(value: Any, label: str) -> float:
    if value is None or isinstance(value, (bool, str, bytes, bytearray)):
        raise ValueError(f"{label} must be finite")
    try:
        result = float(value)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(f"{label} must be finite") from exc
    if not math.isfinite(result):
        raise ValueError(f"{label} must be finite")
    return result


def _finite_positive(value: Any, label: str) -> float:
    result = _finite_real(value, label)
    if result <= 0:
        raise ValueError(f"{label} must be positive")
    return result


def _integer_like(value: Any, label: str, *, positive: bool = False) -> int:
    if value is None or isinstance(value, (bool, str, bytes, bytearray)):
        raise ValueError(f"{label} must be a finite integer-like value")
    if isinstance(value, numbers.Integral):
        result = int(value)
    elif isinstance(value, numbers.Real):
        numeric = float(value)
        if not math.isfinite(numeric) or not numeric.is_integer():
            raise ValueError(f"{label} must be a finite integer-like value")
        result = int(numeric)
    else:
        raise ValueError(f"{label} must be a finite integer-like value")
    if positive and result <= 0:
        raise ValueError(f"{label} must be positive")
    return result


def _safe_float(value: Any) -> float | None:
    """对仅用于诊断的字段做尽力而为的有限浮点转换。"""

    if isinstance(value, bool) or not isinstance(value, numbers.Real):
        return None
    result = float(value)
    return result if math.isfinite(result) else None


@dataclass(frozen=True)
class CameraIntrinsics:
    """对齐后头部彩色/深度流的针孔内参。"""

    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    height: int

    def __post_init__(self) -> None:
        fx = _finite_positive(self.fx, "intrinsics.fx")
        fy = _finite_positive(self.fy, "intrinsics.fy")
        width = _integer_like(self.width, "intrinsics.width", positive=True)
        height = _integer_like(self.height, "intrinsics.height", positive=True)
        cx = _finite_real(self.cx, "intrinsics.cx")
        cy = _finite_real(self.cy, "intrinsics.cy")
        if not (0 <= cx < width):
            raise ValueError("intrinsics.cx must satisfy 0 <= cx < width")
        if not (0 <= cy < height):
            raise ValueError("intrinsics.cy must satisfy 0 <= cy < height")
        object.__setattr__(self, "fx", fx)
        object.__setattr__(self, "fy", fy)
        object.__setattr__(self, "cx", cx)
        object.__setattr__(self, "cy", cy)
        object.__setattr__(self, "width", width)
        object.__setattr__(self, "height", height)


@dataclass(frozen=True)
class DepthLimits:
    """原始 uint16 深度读数的有效工作距离窗口。

    ``depth_scale_m_per_unit`` 默认 0.001，即 D435i 的 uint16 深度帧
    以毫米表达。
    """

    min_depth_m: float = 0.20
    max_depth_m: float = 3.00
    depth_scale_m_per_unit: float = 0.001

    def __post_init__(self) -> None:
        min_depth = _finite_positive(self.min_depth_m, "limits.min_depth_m")
        max_depth = _finite_positive(self.max_depth_m, "limits.max_depth_m")
        scale = _finite_positive(self.depth_scale_m_per_unit, "limits.depth_scale_m_per_unit")
        if min_depth >= max_depth:
            raise ValueError("limits.min_depth_m must be less than limits.max_depth_m")
        object.__setattr__(self, "min_depth_m", min_depth)
        object.__setattr__(self, "max_depth_m", max_depth)
        object.__setattr__(self, "depth_scale_m_per_unit", scale)


@dataclass(frozen=True)
class Point3:
    """在某个调用方已知坐标系下的有限 3D 点。"""

    x: float
    y: float
    z: float

    def __post_init__(self) -> None:
        object.__setattr__(self, "x", _finite_real(self.x, "point.x"))
        object.__setattr__(self, "y", _finite_real(self.y, "point.y"))
        object.__setattr__(self, "z", _finite_real(self.z, "point.z"))

    def as_tuple(self) -> tuple[float, float, float]:
        return (self.x, self.y, self.z)

    def to_record(self) -> dict[str, float]:
        return {"x": self.x, "y": self.y, "z": self.z}


@dataclass(frozen=True)
class BottlePositionResult:
    """一次像素+深度 -> 基座坐标定位的 fail-closed 结果。"""

    status: str
    failure_code: str
    reason: str
    pixel: tuple[float, float] | None
    depth_raw: int | None
    depth_m: float | None
    point_camera: Point3 | None
    point_base: Point3 | None
    joint_state_age_s: float | None
    evidence: Mapping[str, Any]

    @property
    def located(self) -> bool:
        return self.status == "located" and self.failure_code == "ok" and self.point_base is not None

    def to_record(self) -> dict[str, Any]:
        """返回可 JSON 序列化的记录，不直接暴露 dataclass 值。"""

        return {
            "status": self.status,
            "failure_code": self.failure_code,
            "reason": self.reason,
            "pixel": _json_value(self.pixel),
            "depth_raw": _json_value(self.depth_raw),
            "depth_m": _json_value(self.depth_m),
            "point_camera": _json_value(self.point_camera.to_record() if self.point_camera is not None else None),
            "point_base": _json_value(self.point_base.to_record() if self.point_base is not None else None),
            "joint_state_age_s": _json_value(self.joint_state_age_s),
            "evidence": _json_value(self.evidence),
        }


def _pixel_components(pixel: Any, intrinsics: CameraIntrinsics) -> tuple[float, float]:
    try:
        u_raw, v_raw = pixel
    except (TypeError, ValueError) as exc:
        raise ValueError("pixel must be a two-element (u, v) sequence") from exc
    u = _finite_real(u_raw, "pixel.u")
    v = _finite_real(v_raw, "pixel.v")
    if not (0 <= u < intrinsics.width):
        raise ValueError("pixel.u must satisfy 0 <= u < width")
    if not (0 <= v < intrinsics.height):
        raise ValueError("pixel.v must satisfy 0 <= v < height")
    return u, v


def backproject_pixel(
    pixel: tuple[float, float],
    depth_raw: int,
    intrinsics: CameraIntrinsics,
    limits: DepthLimits = DepthLimits(),
) -> Point3:
    """把一个像素 + 原始深度读数反投影到相机光学坐标系。

    这是内部数值原语：任何非法输入都"可以"抛 ``ValueError``。
    fail-closed 的入口是包装了本调用的
    :func:`locate_bottle_in_base`。
    """

    if not isinstance(intrinsics, CameraIntrinsics):
        raise ValueError("intrinsics must be a CameraIntrinsics")
    if not isinstance(limits, DepthLimits):
        raise ValueError("limits must be a DepthLimits")
    u, v = _pixel_components(pixel, intrinsics)
    depth_raw_int = _integer_like(depth_raw, "depth_raw", positive=True)
    depth_m = depth_raw_int * limits.depth_scale_m_per_unit
    if not (limits.min_depth_m <= depth_m <= limits.max_depth_m):
        raise ValueError(
            f"depth {depth_m:.6f} m is outside [{limits.min_depth_m}, {limits.max_depth_m}]"
        )
    z = depth_m
    x = (u - intrinsics.cx) / intrinsics.fx * z
    y = (v - intrinsics.cy) / intrinsics.fy * z
    return Point3(x, y, z)


def validate_transform(matrix: Sequence[Sequence[float]]) -> tuple[tuple[float, ...], ...]:
    """校验 4x4 齐次变换并返回归一化副本。

    除非矩阵是有限的 4x4 嵌套序列、末行为 ``[0, 0, 0, 1]``、且左上
    3x3 旋转块近似为正交正旋转（``R @ R.T`` 接近单位阵且 ``det(R)``
    接近 1——即无反射、无缩放），否则抛 ``ValueError``。
    """

    try:
        rows = list(matrix)
    except TypeError as exc:
        raise ValueError("transform must be a 4x4 nested sequence") from exc
    if len(rows) != 4:
        raise ValueError("transform must have exactly 4 rows")

    normalized: list[tuple[float, ...]] = []
    for row_index, row in enumerate(rows):
        try:
            columns = list(row)
        except TypeError as exc:
            raise ValueError(f"transform row {row_index} must be a 4-element sequence") from exc
        if len(columns) != 4:
            raise ValueError(f"transform row {row_index} must have exactly 4 columns")
        normalized.append(
            tuple(
                _finite_real(value, f"transform[{row_index}][{col}]")
                for col, value in enumerate(columns)
            )
        )

    expected_last_row = (0.0, 0.0, 0.0, 1.0)
    if any(abs(a - b) > 1e-9 for a, b in zip(normalized[3], expected_last_row)):
        raise ValueError("transform last row must be [0, 0, 0, 1]")

    rotation = [row[:3] for row in normalized[:3]]
    # R @ R.T 必须近似单位阵（正交旋转）。
    for i in range(3):
        for j in range(3):
            dot = sum(rotation[i][k] * rotation[j][k] for k in range(3))
            expected = 1.0 if i == j else 0.0
            if abs(dot - expected) > 1e-6:
                raise ValueError("transform rotation block is not orthogonal")

    # 手工展开的 3x3 行列式（无 NumPy）：必须是正旋转，即 det(R)
    # 约等于 1，排除反射和整体缩放。
    determinant = (
        rotation[0][0] * (rotation[1][1] * rotation[2][2] - rotation[1][2] * rotation[2][1])
        - rotation[0][1] * (rotation[1][0] * rotation[2][2] - rotation[1][2] * rotation[2][0])
        + rotation[0][2] * (rotation[1][0] * rotation[2][1] - rotation[1][1] * rotation[2][0])
    )
    if abs(determinant - 1.0) > 1e-6:
        raise ValueError("transform rotation block must have determinant 1 (proper rotation)")

    return tuple(normalized)


def transform_point_to_base(point_camera: Point3, transform: Sequence[Sequence[float]]) -> Point3:
    """把已校验的基座<-相机齐次变换应用到一个点上。"""

    if not isinstance(point_camera, Point3):
        raise ValueError("point_camera must be a Point3")
    normalized = validate_transform(transform)
    x, y, z = point_camera.x, point_camera.y, point_camera.z
    base_x = normalized[0][0] * x + normalized[0][1] * y + normalized[0][2] * z + normalized[0][3]
    base_y = normalized[1][0] * x + normalized[1][1] * y + normalized[1][2] * z + normalized[1][3]
    base_z = normalized[2][0] * x + normalized[2][1] * y + normalized[2][2] * z + normalized[2][3]
    return Point3(base_x, base_y, base_z)


def locate_bottle_in_base(
    *,
    pixel: tuple[float, float],
    depth_raw: int,
    intrinsics: CameraIntrinsics,
    transform: Sequence[Sequence[float]],
    limits: DepthLimits = DepthLimits(),
    joint_state_age_s: float,
    max_joint_state_age_s: float,
) -> BottlePositionResult:
    """fail-closed 入口：像素+深度+内参+变换 -> 基座坐标点。

    本函数从不抛异常。一切失败（非法输入、深度越界、关节状态过期
    或任何意外错误）都通过返回的 :class:`BottlePositionResult` 报告，
    此时 ``point_base is None``。
    """

    def evidence(camera_position_in_base: tuple[float, float, float] | None = None) -> dict[str, Any]:
        return {
            "intrinsics": (
                {
                    "fx": intrinsics.fx,
                    "fy": intrinsics.fy,
                    "cx": intrinsics.cx,
                    "cy": intrinsics.cy,
                    "width": intrinsics.width,
                    "height": intrinsics.height,
                }
                if isinstance(intrinsics, CameraIntrinsics)
                else None
            ),
            "depth_scale_m_per_unit": limits.depth_scale_m_per_unit if isinstance(limits, DepthLimits) else None,
            "min_depth_m": limits.min_depth_m if isinstance(limits, DepthLimits) else None,
            "max_depth_m": limits.max_depth_m if isinstance(limits, DepthLimits) else None,
            "max_joint_state_age_s": max_joint_state_age_s,
            "joint_state_age_s": joint_state_age_s,
            "camera_position_in_base": camera_position_in_base,
        }

    def fail(
        code: str,
        reason: str,
        *,
        pixel_value: tuple[float, float] | None = None,
        depth_value: int | None = None,
        depth_m: float | None = None,
        point_camera: Point3 | None = None,
        camera_position_in_base: tuple[float, float, float] | None = None,
    ) -> BottlePositionResult:
        return BottlePositionResult(
            status=code,
            failure_code=code,
            reason=reason,
            pixel=pixel_value,
            depth_raw=depth_value,
            depth_m=depth_m,
            point_camera=point_camera,
            point_base=None,
            joint_state_age_s=_safe_float(joint_state_age_s),
            evidence=evidence(camera_position_in_base=camera_position_in_base),
        )

    try:
        if not isinstance(intrinsics, CameraIntrinsics):
            return fail("invalid_intrinsics", "intrinsics must be a CameraIntrinsics instance")
        if not isinstance(limits, DepthLimits):
            return fail("invalid_limits", "limits must be a DepthLimits instance")

        try:
            u, v = _pixel_components(pixel, intrinsics)
        except Exception as exc:
            return fail("invalid_pixel", f"invalid pixel: {exc}")

        try:
            depth_raw_int = _integer_like(depth_raw, "depth_raw", positive=True)
        except Exception as exc:
            return fail("invalid_depth", f"invalid depth_raw: {exc}", pixel_value=(u, v))

        depth_m = depth_raw_int * limits.depth_scale_m_per_unit
        if not (limits.min_depth_m <= depth_m <= limits.max_depth_m):
            return fail(
                "depth_out_of_range",
                f"depth {depth_m:.6f} m is outside [{limits.min_depth_m}, {limits.max_depth_m}]",
                pixel_value=(u, v),
                depth_value=depth_raw_int,
                depth_m=depth_m,
            )

        point_camera = backproject_pixel((u, v), depth_raw_int, intrinsics, limits)

        try:
            normalized_transform = validate_transform(transform)
        except Exception as exc:
            return fail(
                "invalid_transform",
                f"invalid transform: {exc}",
                pixel_value=(u, v),
                depth_value=depth_raw_int,
                depth_m=depth_m,
                point_camera=point_camera,
            )
        camera_position_in_base = (
            normalized_transform[0][3],
            normalized_transform[1][3],
            normalized_transform[2][3],
        )

        age_valid = (
            isinstance(joint_state_age_s, numbers.Real)
            and not isinstance(joint_state_age_s, bool)
            and math.isfinite(float(joint_state_age_s))
            and float(joint_state_age_s) >= 0
        )
        max_age_valid = (
            isinstance(max_joint_state_age_s, numbers.Real)
            and not isinstance(max_joint_state_age_s, bool)
            and math.isfinite(float(max_joint_state_age_s))
            and float(max_joint_state_age_s) > 0
        )
        if not age_valid or not max_age_valid:
            return fail(
                "invalid_joint_state",
                "joint_state_age_s must be a finite non-negative number and "
                "max_joint_state_age_s must be a finite positive number",
                pixel_value=(u, v),
                depth_value=depth_raw_int,
                depth_m=depth_m,
                point_camera=point_camera,
                camera_position_in_base=camera_position_in_base,
            )
        if float(joint_state_age_s) > float(max_joint_state_age_s):
            return fail(
                "stale_joint_state",
                f"joint_state_age_s {joint_state_age_s} exceeds max_joint_state_age_s {max_joint_state_age_s}",
                pixel_value=(u, v),
                depth_value=depth_raw_int,
                depth_m=depth_m,
                point_camera=point_camera,
                camera_position_in_base=camera_position_in_base,
            )

        point_base = transform_point_to_base(point_camera, normalized_transform)

        return BottlePositionResult(
            status="located",
            failure_code="ok",
            reason="pixel backprojected and transformed into the base frame",
            pixel=(u, v),
            depth_raw=depth_raw_int,
            depth_m=depth_m,
            point_camera=point_camera,
            point_base=point_base,
            joint_state_age_s=float(joint_state_age_s),
            evidence=evidence(camera_position_in_base=camera_position_in_base),
        )
    except Exception as exc:  # pragma: no cover - defensive fail-closed catch-all
        return fail("processing_error", f"unexpected processing failure: {exc}")


__all__ = [
    "CameraIntrinsics",
    "DepthLimits",
    "Point3",
    "BottlePositionResult",
    "backproject_pixel",
    "validate_transform",
    "transform_point_to_base",
    "locate_bottle_in_base",
]
