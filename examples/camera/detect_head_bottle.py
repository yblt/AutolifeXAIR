#!/usr/bin/env python3
"""单帧头部相机瓶身检测。

本模块不依赖 ROS 或机器人 SDK。调用方传入一个鸭子类型的 BGR 帧
（``image``、``frame_id``、``received_at``），检测器只返回二维图像
测量值：不做深度反投影，也不把坐标转换到机器人基座坐标（那部分
组合在独立的几何模块里）。OpenCV 经 :class:`_OpenCVBackend` 惰性
加载；在没装相机或 GPU 驱动的开发机上离线 import 本模块也能工作。

检测器是 fail-closed 的：所有失败路径（非法帧、非法配置、backend
不可用/不完整、处理异常、无合格候选、多候选歧义场景）都返回
:class:`BottleDetectionResult` 而不抛异常，且当多个连通域都合格时
绝不猜测"最可能"的候选。

检测到的瓶子对外发布的参考像素是其检测框的"底边"中心
（``bottom_center``），不是几何中心：瓶底更贴近桌面平面，能为
配套的反投影步骤提供更稳定的深度采样。
"""

from __future__ import annotations

import math
import numbers
import time
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from typing import Any, Callable


MAX_MORPHOLOGY_KERNEL_DIMENSION = 31
MAX_MORPHOLOGY_ITERATIONS = 8
_MAX_HUE = 179
_MAX_SATURATION_OR_VALUE = 255


def _json_value(value: Any) -> Any:
    """把检测器元数据转成普通的 JSON 兼容值。"""

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


@dataclass(frozen=True)
class ROI:
    x: int
    y: int
    width: int
    height: int

    @property
    def right(self) -> int:
        return self.x + self.width

    @property
    def bottom(self) -> int:
        return self.y + self.height

    def as_tuple(self) -> tuple[int, int, int, int]:
        return self.x, self.y, self.width, self.height


@dataclass(frozen=True)
class HSVRange:
    """一个 OpenCV 约定的 HSV 区间：H 在 [0,179]，S/V 在 [0,255]。

    ``h_min`` 可以大于 ``h_max``，表达色相绕环（例如横跨色相圆环
    两端的红色区间）；此时 :meth:`bounds` 返回两段而不是一段。
    """

    h_min: int
    h_max: int
    s_min: int
    s_max: int
    v_min: int
    v_max: int

    def __post_init__(self) -> None:
        h_min = _integer_like(self.h_min, "hsv_range.h_min")
        h_max = _integer_like(self.h_max, "hsv_range.h_max")
        s_min = _integer_like(self.s_min, "hsv_range.s_min")
        s_max = _integer_like(self.s_max, "hsv_range.s_max")
        v_min = _integer_like(self.v_min, "hsv_range.v_min")
        v_max = _integer_like(self.v_max, "hsv_range.v_max")
        for label, value in (("h_min", h_min), ("h_max", h_max)):
            if not 0 <= value <= _MAX_HUE:
                raise ValueError(f"hsv_range.{label} must be between 0 and {_MAX_HUE}")
        for label, value in (
            ("s_min", s_min),
            ("s_max", s_max),
            ("v_min", v_min),
            ("v_max", v_max),
        ):
            if not 0 <= value <= _MAX_SATURATION_OR_VALUE:
                raise ValueError(
                    f"hsv_range.{label} must be between 0 and {_MAX_SATURATION_OR_VALUE}"
                )
        if s_min > s_max:
            raise ValueError("hsv_range.s_min must not exceed hsv_range.s_max")
        if v_min > v_max:
            raise ValueError("hsv_range.v_min must not exceed hsv_range.v_max")
        object.__setattr__(self, "h_min", h_min)
        object.__setattr__(self, "h_max", h_max)
        object.__setattr__(self, "s_min", s_min)
        object.__setattr__(self, "s_max", s_max)
        object.__setattr__(self, "v_min", v_min)
        object.__setattr__(self, "v_max", v_max)

    def bounds(
        self,
    ) -> tuple[tuple[tuple[int, int, int], tuple[int, int, int]], ...]:
        """返回一或两组与 BGR 顺序无关的 ``(lower, upper)`` HSV 边界。"""

        if self.h_min <= self.h_max:
            return (
                (
                    (self.h_min, self.s_min, self.v_min),
                    (self.h_max, self.s_max, self.v_max),
                ),
            )
        return (
            (
                (self.h_min, self.s_min, self.v_min),
                (_MAX_HUE, self.s_max, self.v_max),
            ),
            (
                (0, self.s_min, self.v_min),
                (self.h_max, self.s_max, self.v_max),
            ),
        )


@dataclass(frozen=True)
class BottleDetectorConfig:
    roi: ROI = ROI(0, 0, 640, 480)
    hsv_ranges: tuple[HSVRange, ...] = ()
    morphology_kernel: tuple[int, int] = (5, 5)
    morphology_iterations: int = 1
    morphology_operation: str = "close"
    min_area: int = 300
    max_area: int = 307200
    min_width: int = 10
    max_width: int = 640
    min_height: int = 20
    max_height: int = 480
    min_aspect_ratio: float = 0.15
    max_aspect_ratio: float = 1.5
    min_fill_ratio: float = 0.3
    max_fill_ratio: float = 1.0
    reject_roi_boundary: bool = True
    config_id: str = ""


@dataclass(frozen=True)
class BottleCandidate:
    label: int
    center: tuple[float, float]
    bbox: tuple[int, int, int, int]
    area: int
    width: int
    height: int
    aspect_ratio: float
    fill_ratio: float
    bottom_center: tuple[float, float]


@dataclass(frozen=True)
class BottleDetectionResult:
    frame_id: int | None
    frame_shape: tuple[int, int, int] | None
    frame_dtype: str | None
    color_order: str
    received_at: float | None
    roi: ROI | None
    effective_parameters: Mapping[str, Any]
    candidates: tuple[BottleCandidate, ...]
    selected_candidate: BottleCandidate | None
    status: str
    failure_code: str
    reason: str

    @property
    def detected(self) -> bool:
        return self.status == "detected" and self.selected_candidate is not None

    @property
    def center(self) -> tuple[float, float] | None:
        return self.selected_candidate.center if self.selected_candidate else None

    @property
    def bbox(self) -> tuple[int, int, int, int] | None:
        return self.selected_candidate.bbox if self.selected_candidate else None

    @property
    def area(self) -> int | None:
        return self.selected_candidate.area if self.selected_candidate else None

    @property
    def dark_area(self) -> int | None:
        """与 ``SleeveDetectionResult`` 对齐的鸭子类型别名，供门控复用。"""

        return self.area

    @property
    def fill_ratio(self) -> float | None:
        return self.selected_candidate.fill_ratio if self.selected_candidate else None

    @property
    def bbox_height_ratio(self) -> float | None:
        if self.selected_candidate is None or self.frame_shape is None:
            return None
        frame_height = self.frame_shape[0]
        if frame_height <= 0:
            return None
        return self.selected_candidate.height / float(frame_height)

    @property
    def reference_pixel(self) -> tuple[float, float] | None:
        return self.selected_candidate.bottom_center if self.selected_candidate else None

    def to_record(self) -> dict[str, Any]:
        """返回可 JSON 序列化的记录，不直接暴露 dataclass 值。"""

        return {
            "frame_id": _json_value(self.frame_id),
            "frame_shape": _json_value(self.frame_shape),
            "timestamp": _json_value(self.received_at),
            "center": _json_value(self.center),
            "bbox": _json_value(self.bbox),
            "bbox_height_ratio": _json_value(self.bbox_height_ratio),
            "area": _json_value(self.area),
            "fill_ratio": _json_value(self.fill_ratio),
            "reference_pixel": _json_value(self.reference_pixel),
            "effective_parameters": _json_value(self.effective_parameters),
            "candidates": [
                {
                    "label": candidate.label,
                    "center": _json_value(candidate.center),
                    "bbox": _json_value(candidate.bbox),
                    "area": candidate.area,
                    "aspect_ratio": candidate.aspect_ratio,
                    "fill_ratio": candidate.fill_ratio,
                    "bottom_center": _json_value(candidate.bottom_center),
                }
                for candidate in self.candidates
            ],
            "status": self.status,
            "failure_code": self.failure_code,
            "reason": self.reason,
            "frame_dtype": self.frame_dtype,
            "color_order": self.color_order,
            "roi": _json_value(self.roi.as_tuple() if isinstance(self.roi, ROI) else None),
        }


@dataclass(frozen=True)
class _Component:
    x: int
    y: int
    width: int
    height: int
    area: int
    center_x: float
    center_y: float


class _OpenCVBackend:
    def __init__(self, cv2_module: Any) -> None:
        self._cv2 = cv2_module

    @classmethod
    def load(cls) -> "_OpenCVBackend":
        try:
            import cv2  # type: ignore
        except Exception as exc:  # pragma: no cover - target-environment dependent
            raise RuntimeError(f"OpenCV backend unavailable: {exc}") from exc
        return cls(cv2)

    def crop(self, image: Any, roi: ROI) -> Any:
        return image[roi.y : roi.bottom, roi.x : roi.right]

    def bgr_to_hsv(self, image: Any) -> Any:
        return self._cv2.cvtColor(image, self._cv2.COLOR_BGR2HSV)

    def in_range(self, hsv: Any, lower: Any, upper: Any) -> Any:
        try:
            import numpy as np  # type: ignore
        except Exception as exc:  # pragma: no cover - target-environment dependent
            raise RuntimeError(f"NumPy backend unavailable: {exc}") from exc
        return self._cv2.inRange(
            hsv, np.array(lower, dtype=np.uint8), np.array(upper, dtype=np.uint8)
        )

    def mask_union(self, first: Any, second: Any) -> Any:
        return self._cv2.bitwise_or(first, second)

    def morph(self, mask: Any, operation: str, kernel: tuple[int, int], iterations: int) -> Any:
        if operation == "none" or iterations == 0:
            return mask
        structuring_element = self._cv2.getStructuringElement(
            self._cv2.MORPH_ELLIPSE, kernel
        )
        morph_operation = self._cv2.MORPH_CLOSE if operation == "close" else self._cv2.MORPH_OPEN
        return self._cv2.morphologyEx(
            mask,
            morph_operation,
            structuring_element,
            iterations=iterations,
        )

    def connected_components(self, mask: Any) -> tuple[_Component, ...]:
        count, _labels, stats, centroids = self._cv2.connectedComponentsWithStats(
            mask, connectivity=8
        )
        components: list[_Component] = []
        for label in range(1, int(count)):
            x, y, width, height, area = (int(value) for value in stats[label][:5])
            center_x, center_y = (float(value) for value in centroids[label][:2])
            components.append(
                _Component(x, y, width, height, area, center_x, center_y)
            )
        return tuple(components)


def _field(value: Any, *names: str) -> Any:
    if isinstance(value, Mapping):
        for name in names:
            if name in value:
                return value[name]
    for name in names:
        try:
            item = getattr(value, name)
        except AttributeError:
            continue
        if item is not None:
            return item
    return None


def _integer_like(value: Any, label: str, *, nonnegative: bool = False, positive: bool = False) -> int:
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
    if nonnegative and result < 0:
        raise ValueError(f"{label} must be non-negative")
    if positive and result <= 0:
        raise ValueError(f"{label} must be positive")
    return result


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


def _shape(image: Any) -> tuple[int, int, int]:
    value = getattr(image, "shape", None)
    if value is None or isinstance(value, (str, bytes, bytearray)):
        raise ValueError("frame image must expose a rank-3 shape")
    try:
        dimensions = tuple(value)
    except TypeError as exc:
        raise ValueError("frame image must expose a rank-3 shape") from exc
    if len(dimensions) != 3:
        raise ValueError(f"frame image shape must be HxWx3, received {value!r}")
    result = tuple(
        _integer_like(item, f"frame shape[{index}]", positive=True)
        for index, item in enumerate(dimensions)
    )
    if result[2] != 3:
        raise ValueError(f"frame image shape must be HxWx3, received {value!r}")
    return result  # type: ignore[return-value]


def _dtype_name(value: Any) -> str:
    name = getattr(value, "name", None)
    return str(name if name is not None else value).strip()


def _is_uint8_dtype(value: Any) -> bool:
    return _dtype_name(value).casefold() in {
        "uint8",
        "numpy.uint8",
        "<class 'numpy.uint8'>",
    }


def _validate_frame(frame: Any) -> tuple[Any, int, tuple[int, int, int], str, float]:
    image = _field(frame, "image", "frame")
    if image is None:
        raise ValueError("frame is missing image")
    frame_id = _integer_like(_field(frame, "frame_id", "id"), "frame_id", nonnegative=True)
    received_at = _finite_real(
        _field(frame, "received_at", "receive_timestamp", "timestamp"), "received_at"
    )
    dimensions = _shape(image)
    dtype = _dtype_name(getattr(image, "dtype", None))
    if not _is_uint8_dtype(getattr(image, "dtype", None)):
        raise ValueError(f"frame image dtype must be uint8, received {dtype!r}")
    color_order = _field(frame, "color_order", "pixel_format", "format")
    if color_order is not None and str(getattr(color_order, "value", color_order)).upper() != "BGR":
        raise ValueError(f"frame image must be BGR, received {color_order!r}")
    bgr_marker = _field(frame, "bgr", "is_bgr")
    if bgr_marker is not None and bgr_marker is not True:
        raise ValueError("frame image must be marked as BGR")
    return image, frame_id, dimensions, "uint8", received_at


def _validate_config(config: Any, dimensions: tuple[int, int, int]) -> tuple[BottleDetectorConfig, ROI]:
    if not isinstance(config, BottleDetectorConfig):
        raise ValueError("config must be a BottleDetectorConfig")
    if not isinstance(config.roi, ROI):
        raise ValueError("config.roi must be an ROI")
    roi = ROI(
        _integer_like(config.roi.x, "roi.x", nonnegative=True),
        _integer_like(config.roi.y, "roi.y", nonnegative=True),
        _integer_like(config.roi.width, "roi.width", positive=True),
        _integer_like(config.roi.height, "roi.height", positive=True),
    )
    frame_height, frame_width, _ = dimensions
    if roi.right > frame_width or roi.bottom > frame_height:
        raise ValueError(f"ROI {roi.as_tuple()!r} lies outside frame {(frame_width, frame_height)!r}")

    if not isinstance(config.hsv_ranges, (tuple, list)) or len(config.hsv_ranges) == 0:
        raise ValueError("hsv_ranges must contain at least one HSVRange")
    hsv_ranges: list[HSVRange] = []
    for index, item in enumerate(config.hsv_ranges):
        if not isinstance(item, HSVRange):
            raise ValueError(f"hsv_ranges[{index}] must be an HSVRange")
        hsv_ranges.append(item)

    if not isinstance(config.reject_roi_boundary, bool):
        raise ValueError("reject_roi_boundary must be a boolean")

    kernel = config.morphology_kernel
    if isinstance(kernel, numbers.Integral) and not isinstance(kernel, bool):
        kernel = (int(kernel), int(kernel))
    try:
        kernel_values = tuple(kernel)
    except TypeError as exc:
        raise ValueError("morphology_kernel must contain two dimensions") from exc
    if len(kernel_values) != 2:
        raise ValueError("morphology_kernel must contain two dimensions")
    kernel_values = tuple(
        _integer_like(value, f"morphology_kernel[{index}]", positive=True)
        for index, value in enumerate(kernel_values)
    )
    if any(value % 2 == 0 for value in kernel_values):
        raise ValueError("morphology_kernel dimensions must be odd")
    if any(value > MAX_MORPHOLOGY_KERNEL_DIMENSION for value in kernel_values):
        raise ValueError(f"morphology_kernel dimensions must be <= {MAX_MORPHOLOGY_KERNEL_DIMENSION}")
    iterations = _integer_like(config.morphology_iterations, "morphology_iterations", nonnegative=True)
    if iterations > MAX_MORPHOLOGY_ITERATIONS:
        raise ValueError(f"morphology_iterations must be <= {MAX_MORPHOLOGY_ITERATIONS}")
    if not isinstance(config.morphology_operation, str):
        raise ValueError("morphology_operation must be a string")
    operation = config.morphology_operation.strip().casefold()
    if operation not in {"close", "open", "none"}:
        raise ValueError("morphology_operation must be 'close', 'open', or 'none'")

    min_area = _integer_like(config.min_area, "min_area", positive=True)
    max_area = _integer_like(config.max_area, "max_area", positive=True)
    min_width = _integer_like(config.min_width, "min_width", positive=True)
    max_width = _integer_like(config.max_width, "max_width", positive=True)
    min_height = _integer_like(config.min_height, "min_height", positive=True)
    max_height = _integer_like(config.max_height, "max_height", positive=True)
    if min_area > max_area or min_width > max_width or min_height > max_height:
        raise ValueError("minimum area/width/height must not exceed its maximum")
    min_aspect = _finite_real(config.min_aspect_ratio, "min_aspect_ratio")
    max_aspect = _finite_real(config.max_aspect_ratio, "max_aspect_ratio")
    min_fill = _finite_real(config.min_fill_ratio, "min_fill_ratio")
    max_fill = _finite_real(config.max_fill_ratio, "max_fill_ratio")
    if min_aspect <= 0 or max_aspect <= 0 or min_aspect > max_aspect:
        raise ValueError("aspect ratio range must be positive and ordered")
    if min_fill <= 0 or max_fill <= 0 or min_fill > max_fill or max_fill > 1:
        raise ValueError("fill ratio range must be positive, ordered, and <= 1")

    if not isinstance(config.config_id, str):
        raise ValueError("config_id must be a string")
    config_id = config.config_id
    if config_id != "" and not config_id.strip():
        raise ValueError("config_id must be a non-empty, non-whitespace string when set")

    return BottleDetectorConfig(
        roi=roi,
        hsv_ranges=tuple(hsv_ranges),
        morphology_kernel=(kernel_values[0], kernel_values[1]),
        morphology_iterations=iterations,
        morphology_operation=operation,
        min_area=min_area,
        max_area=max_area,
        min_width=min_width,
        max_width=max_width,
        min_height=min_height,
        max_height=max_height,
        min_aspect_ratio=min_aspect,
        max_aspect_ratio=max_aspect,
        min_fill_ratio=min_fill,
        max_fill_ratio=max_fill,
        reject_roi_boundary=config.reject_roi_boundary,
        config_id=config_id,
    ), roi


def _effective_parameters(config: Any, roi: ROI | None = None) -> dict[str, Any]:
    if not isinstance(config, BottleDetectorConfig):
        return {"config": repr(config)}
    selected_roi = roi if roi is not None else config.roi
    return {
        "roi": selected_roi.as_tuple() if isinstance(selected_roi, ROI) else None,
        "hsv_ranges": [
            {
                "h_min": item.h_min,
                "h_max": item.h_max,
                "s_min": item.s_min,
                "s_max": item.s_max,
                "v_min": item.v_min,
                "v_max": item.v_max,
            }
            for item in config.hsv_ranges
            if isinstance(item, HSVRange)
        ],
        "reject_roi_boundary": config.reject_roi_boundary,
        "morphology_kernel": config.morphology_kernel,
        "morphology_iterations": config.morphology_iterations,
        "morphology_operation": config.morphology_operation,
        "min_area": config.min_area,
        "max_area": config.max_area,
        "min_width": config.min_width,
        "max_width": config.max_width,
        "min_height": config.min_height,
        "max_height": config.max_height,
        "min_aspect_ratio": config.min_aspect_ratio,
        "max_aspect_ratio": config.max_aspect_ratio,
        "min_fill_ratio": config.min_fill_ratio,
        "max_fill_ratio": config.max_fill_ratio,
        "config_id": config.config_id,
    }


def _result(
    *,
    frame_id: int | None,
    frame_shape: tuple[int, int, int] | None,
    frame_dtype: str | None,
    received_at: float | None,
    roi: ROI | None,
    parameters: Mapping[str, Any],
    candidates: Sequence[BottleCandidate] = (),
    selected: BottleCandidate | None = None,
    status: str = "not_detected",
    failure_code: str = "processing_error",
    reason: str,
) -> BottleDetectionResult:
    return BottleDetectionResult(frame_id, frame_shape, frame_dtype, "BGR", received_at,
        roi, dict(parameters), tuple(candidates), selected, status, failure_code, reason)


def _candidate_list(
    components: Sequence[_Component], roi: ROI, config: BottleDetectorConfig
) -> tuple[BottleCandidate, ...]:
    candidates: list[BottleCandidate] = []
    for label, component in enumerate(components, start=1):
        x = _integer_like(component.x, "component x")
        y = _integer_like(component.y, "component y")
        width = _integer_like(component.width, "component width", positive=True)
        height = _integer_like(component.height, "component height", positive=True)
        area = _integer_like(component.area, "component area", positive=True)
        center_x = _finite_real(component.center_x, "component center x")
        center_y = _finite_real(component.center_y, "component center y")
        if x < 0 or y < 0 or x + width > roi.width or y + height > roi.height:
            raise ValueError("component bounding box lies outside the ROI")
        if area > width * height:
            raise ValueError("component area exceeds its bounding box")
        if not (x <= center_x < x + width and y <= center_y < y + height):
            raise ValueError("component centroid lies outside its bounding box")
        touches_boundary = (
            x == 0
            or y == 0
            or x + width == roi.width
            or y + height == roi.height
        )
        if config.reject_roi_boundary and touches_boundary:
            continue
        aspect = width / height
        fill = area / float(width * height)
        if not (
            config.min_area <= area <= config.max_area
            and config.min_width <= width <= config.max_width
            and config.min_height <= height <= config.max_height
            and config.min_aspect_ratio <= aspect <= config.max_aspect_ratio
            and config.min_fill_ratio <= fill <= config.max_fill_ratio
        ):
            continue
        candidates.append(
            BottleCandidate(
                label,
                (roi.x + x + width / 2.0, roi.y + y + height / 2.0),
                (roi.x + x, roi.y + y, width, height),
                area,
                width,
                height,
                aspect,
                fill,
                (roi.x + x + width / 2.0, roi.y + y + height - 1.0),
            )
        )
    candidates.sort(key=lambda item: (item.bbox, item.area, item.center))
    return tuple(
        BottleCandidate(
            index,
            item.center,
            item.bbox,
            item.area,
            item.width,
            item.height,
            item.aspect_ratio,
            item.fill_ratio,
            item.bottom_center,
        )
        for index, item in enumerate(candidates, start=1)
    )


def _protocol_missing(backend: Any, *, require_morph: bool) -> tuple[str, ...]:
    operations = (
        "crop",
        "bgr_to_hsv",
        "in_range",
        "mask_union",
        "connected_components",
    )
    if require_morph:
        operations = operations[:4] + ("morph",) + operations[4:]
    return tuple(
        name for name in operations if not callable(getattr(backend, name, None))
    )


def detect_head_bottle(
    frame: Any,
    config: BottleDetectorConfig,
    *,
    backend: Any = None,
) -> BottleDetectionResult:
    """运行一次头部相机瓶子检测器，返回 fail-closed 结果。"""

    frame_id = frame_shape = frame_dtype = received_at = None
    try:
        image, frame_id, frame_shape, frame_dtype, received_at = _validate_frame(frame)
    except Exception as exc:
        return _result(
            frame_id=None,
            frame_shape=None,
            frame_dtype=None,
            received_at=None,
            roi=None,
            parameters=_effective_parameters(config),
            status="invalid_frame",
            failure_code="invalid_frame",
            reason=f"invalid frame: {exc}",
        )

    try:
        normalized, roi = _validate_config(config, frame_shape)
    except Exception as exc:
        return _result(
            frame_id=frame_id,
            frame_shape=frame_shape,
            frame_dtype=frame_dtype,
            received_at=received_at,
            roi=None,
            parameters=_effective_parameters(config),
            status="invalid_parameters",
            failure_code="invalid_config",
            reason=f"invalid detector configuration: {exc}",
        )

    parameters = _effective_parameters(normalized, roi)

    def finish(
        code: str,
        reason: str,
        *,
        status: str = "not_detected",
        candidates: Sequence[BottleCandidate] = (),
        selected: BottleCandidate | None = None,
    ) -> BottleDetectionResult:
        return _result(
            frame_id=frame_id,
            frame_shape=frame_shape,
            frame_dtype=frame_dtype,
            received_at=received_at,
            roi=roi,
            parameters=parameters,
            candidates=candidates,
            selected=selected,
            status=status,
            failure_code=code,
            reason=reason,
        )

    if backend is None:
        try:
            backend = _OpenCVBackend.load()
        except Exception as exc:
            return finish("backend_unavailable", str(exc), status="backend_unavailable")
    try:
        missing = _protocol_missing(
            backend,
            require_morph=(normalized.morphology_operation != "none" and normalized.morphology_iterations > 0),
        )
    except Exception as exc:
        return finish("invalid_backend", f"backend protocol inspection failed: {exc}", status="invalid_backend")
    if missing:
        return finish("invalid_backend", "backend is missing required operations: " + ", ".join(missing), status="invalid_backend")

    try:
        cropped = backend.crop(image, roi)
        hsv = backend.bgr_to_hsv(cropped)
        mask = None
        for hsv_range in normalized.hsv_ranges:
            for lower, upper in hsv_range.bounds():
                band_mask = backend.in_range(hsv, lower, upper)
                mask = band_mask if mask is None else backend.mask_union(mask, band_mask)
        if normalized.morphology_operation != "none" and normalized.morphology_iterations > 0:
            mask = backend.morph(
                mask,
                normalized.morphology_operation,
                normalized.morphology_kernel,
                normalized.morphology_iterations,
            )
        components = backend.connected_components(mask)
        candidates = _candidate_list(components, roi, normalized)
    except Exception as exc:
        return finish("processing_error", f"detector processing failed: {exc}", status="processing_error")

    if not candidates:
        return finish(
            "no_qualifying_candidate",
            "no connected component satisfies the configured area and geometry filters",
        )
    if len(candidates) > 1:
        return finish(
            "multiple_qualifying_candidates",
            f"{len(candidates)} connected components satisfy the configured filters; selection is ambiguous",
            status="ambiguous",
            candidates=candidates,
        )
    return finish(
        "ok",
        "exactly one connected component satisfies the configured filters",
        status="detected",
        candidates=candidates,
        selected=candidates[0],
    )


__all__ = [
    "ROI",
    "HSVRange",
    "BottleDetectorConfig",
    "BottleCandidate",
    "BottleDetectionResult",
    "detect_head_bottle",
]
