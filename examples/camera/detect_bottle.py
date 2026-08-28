#!/usr/bin/env python3
"""单帧 HSV 瓶子检测与连续帧一致性门。

针对 point2 黑桌布上绿茶瓶的精简现场检测器：HSV 区间掩膜、有界
形态学、连通域、尺寸与长宽比过滤，加唯一候选规则。调用方传入一个
鸭子类型的 BGR 帧（``image``、``frame_id``）；检测器只返回二维图像
测量值，且对场景条件绝不抛异常——所有失败路径都是带类型的结果。
OpenCV/NumPy 经可注入的 backend 惰性加载，因此离线也能 import 和
测试本模块。

对外发布的接近度量是 ``width_ratio``（检测框宽 / 图像宽）：瓶子
轮廓宽度就是瓶径，旋转下稳定，且整个接近过程都留在画面内；而高度
会在画面下缘被裁掉，还依赖标签上绿白渐变的分界位置。
"""

from __future__ import annotations

import math
import numbers
import time
from collections.abc import Sequence
from dataclasses import dataclass
from typing import Any, Callable


MAX_MORPHOLOGY_KERNEL_DIMENSION = 31
MAX_MORPHOLOGY_ITERATIONS = 8
MAX_ACQUISITION_TIMEOUT_SECONDS = 30.0


def _int_value(value: Any, label: str, *, minimum: int | None = None, maximum: int | None = None) -> int:
    if isinstance(value, bool) or not isinstance(value, numbers.Integral):
        raise ValueError(f"{label} must be an integer, received {value!r}")
    result = int(value)
    if minimum is not None and result < minimum:
        raise ValueError(f"{label} must be >= {minimum}, received {result}")
    if maximum is not None and result > maximum:
        raise ValueError(f"{label} must be <= {maximum}, received {result}")
    return result


def _float_value(value: Any, label: str, *, minimum: float | None = None, maximum: float | None = None) -> float:
    if isinstance(value, bool) or not isinstance(value, numbers.Real):
        raise ValueError(f"{label} must be a number, received {value!r}")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{label} must be finite, received {value!r}")
    if minimum is not None and result < minimum:
        raise ValueError(f"{label} must be >= {minimum}, received {result}")
    if maximum is not None and result > maximum:
        raise ValueError(f"{label} must be <= {maximum}, received {result}")
    return result


def _hsv_triple(value: Any, label: str) -> tuple[int, int, int]:
    if not isinstance(value, Sequence) or isinstance(value, (str, bytes, bytearray)) or len(value) != 3:
        raise ValueError(f"{label} must be a sequence of three integers")
    hue = _int_value(value[0], f"{label}[0] (H)", minimum=0, maximum=179)
    saturation = _int_value(value[1], f"{label}[1] (S)", minimum=0, maximum=255)
    brightness = _int_value(value[2], f"{label}[2] (V)", minimum=0, maximum=255)
    return hue, saturation, brightness


@dataclass(frozen=True)
class BottleDetectorConfig:
    """已验证的单帧检测器参数；非法值直接抛异常。"""

    hsv_lower: tuple[int, int, int] = (45, 70, 80)
    hsv_upper: tuple[int, int, int] = (80, 255, 255)
    min_area: int = 2000
    max_area: int = 307200
    min_width: int = 10
    max_width: int = 640
    min_height: int = 10
    max_height: int = 480
    min_aspect_ratio: float = 0.5
    max_aspect_ratio: float = 8.0
    morphology_kernel: tuple[int, int] = (5, 5)
    morphology_iterations: int = 1
    # 有多个合格连通域时，若最大者面积至少是第二名的这么多倍则接受它
    # （近距离掩膜会碎裂；场景里只有一个绿瓶）。面积相当的候选保持
    # 歧义并 fail-closed。0 表示禁用此规则。
    dominance_ratio: float = 3.0
    # 候选检测框的"中心"必须落在这个 (x, y, 宽, 高) 区域内。瓶子站在
    # 固定桌面上，因此标定的画面下部 ROI 可以剔除弱光下越过 HSV 阈值
    # 的墙面/背景噪声，而合理裁到画面边缘的近距离瓶子仍被接受。
    # 默认值接受整幅画面。
    roi: tuple[int, int, int, int] = (0, 0, 10_000, 10_000)

    def __post_init__(self) -> None:
        lower = _hsv_triple(self.hsv_lower, "hsv_lower")
        upper = _hsv_triple(self.hsv_upper, "hsv_upper")
        # 只校验 S/V 两通道：色相是圆环，lower[0] > upper[0]（如红色
        # 170..10）是合法的绕环区间。这个豁免、bottle_grasp_config
        # .load_config 里的孪生检查、以及 _OpenCVBackend.color_mask 的
        # 绕环分支是"一套不可拆分的组合"——若放松这里的检查而
        # color_mask 只有单次 inRange，绕环区间会静默返回全黑掩膜
        # （0 个候选，无报错）。
        if any(low > high for low, high in zip(lower[1:], upper[1:])):
            raise ValueError(f"hsv_lower {lower} must not exceed hsv_upper {upper} per channel")
        object.__setattr__(self, "hsv_lower", lower)
        object.__setattr__(self, "hsv_upper", upper)
        min_area = _int_value(self.min_area, "min_area", minimum=1)
        max_area = _int_value(self.max_area, "max_area", minimum=min_area)
        min_width = _int_value(self.min_width, "min_width", minimum=1)
        max_width = _int_value(self.max_width, "max_width", minimum=min_width)
        min_height = _int_value(self.min_height, "min_height", minimum=1)
        max_height = _int_value(self.max_height, "max_height", minimum=min_height)
        min_aspect = _float_value(self.min_aspect_ratio, "min_aspect_ratio", minimum=0.01)
        max_aspect = _float_value(self.max_aspect_ratio, "max_aspect_ratio", minimum=min_aspect)
        kernel = self.morphology_kernel
        if not isinstance(kernel, Sequence) or isinstance(kernel, (str, bytes, bytearray)) or len(kernel) != 2:
            raise ValueError("morphology_kernel must be a (width, height) pair")
        kernel = (
            _int_value(kernel[0], "morphology_kernel[0]", minimum=1, maximum=MAX_MORPHOLOGY_KERNEL_DIMENSION),
            _int_value(kernel[1], "morphology_kernel[1]", minimum=1, maximum=MAX_MORPHOLOGY_KERNEL_DIMENSION),
        )
        iterations = _int_value(
            self.morphology_iterations, "morphology_iterations", minimum=0, maximum=MAX_MORPHOLOGY_ITERATIONS
        )
        roi = self.roi
        if not isinstance(roi, Sequence) or isinstance(roi, (str, bytes, bytearray)) or len(roi) != 4:
            raise ValueError("roi must be an (x, y, width, height) quadruple")
        roi = (
            _int_value(roi[0], "roi[0] (x)", minimum=0, maximum=10_000),
            _int_value(roi[1], "roi[1] (y)", minimum=0, maximum=10_000),
            _int_value(roi[2], "roi[2] (width)", minimum=1, maximum=10_000),
            _int_value(roi[3], "roi[3] (height)", minimum=1, maximum=10_000),
        )
        object.__setattr__(self, "min_area", min_area)
        object.__setattr__(self, "max_area", max_area)
        object.__setattr__(self, "min_width", min_width)
        object.__setattr__(self, "max_width", max_width)
        object.__setattr__(self, "min_height", min_height)
        object.__setattr__(self, "max_height", max_height)
        object.__setattr__(self, "min_aspect_ratio", min_aspect)
        object.__setattr__(self, "max_aspect_ratio", max_aspect)
        object.__setattr__(self, "morphology_kernel", kernel)
        object.__setattr__(self, "morphology_iterations", iterations)
        object.__setattr__(self, "roi", roi)
        object.__setattr__(
            self, "dominance_ratio", _float_value(self.dominance_ratio, "dominance_ratio", minimum=0.0)
        )


@dataclass(frozen=True)
class BottleCandidate:
    """图像坐标下的一个合格连通域。"""

    bbox: tuple[int, int, int, int]
    area: int
    center: tuple[float, float]
    width_ratio: float
    height_ratio: float


@dataclass(frozen=True)
class BottleDetectionResult:
    status: str
    failure_code: str
    reason: str
    candidate: BottleCandidate | None = None
    rejected_candidates: int = 0
    frame_id: Any = None
    frame_shape: tuple[int, int, int] | None = None

    @property
    def detected(self) -> bool:
        return self.status == "detected" and self.failure_code == "ok" and self.candidate is not None


def _failure(
    code: str,
    reason: str,
    *,
    status: str = "error",
    frame_id: Any = None,
    frame_shape: tuple[int, int, int] | None = None,
    rejected: int = 0,
) -> BottleDetectionResult:
    return BottleDetectionResult(status, code, reason, None, rejected, frame_id, frame_shape)


class _OpenCVBackend:
    """真实的 OpenCV/NumPy 掩膜与连通域操作，惰性加载。"""

    def __init__(self) -> None:
        import cv2
        import numpy

        self._cv2 = cv2
        self._np = numpy

    def color_mask(
        self,
        image: Any,
        lower: tuple[int, int, int],
        upper: tuple[int, int, int],
        kernel: tuple[int, int],
        iterations: int,
    ) -> Any:
        cv2 = self._cv2
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        # 支持绕环的掩膜；必须与"只校验 S/V"的区间检查保持一致
        # （BottleDetectorConfig.__post_init__ 和 config 模块）。
        # 若把这里退回单次 inRange，红色 170..10 之类的绕环区间会
        # 静默产生空掩膜——红瓶链路依赖下面的 else 分支。
        np = self._np
        if lower[0] <= upper[0]:
            mask = cv2.inRange(hsv, np.array(lower, dtype="uint8"), np.array(upper, dtype="uint8"))
        else:
            # 色相区间跨 179/0 绕环（红色）：取两个子区间的并集。
            mask = cv2.bitwise_or(
                cv2.inRange(hsv, np.array(lower, dtype="uint8"),
                            np.array((179, upper[1], upper[2]), dtype="uint8")),
                cv2.inRange(hsv, np.array((0, lower[1], lower[2]), dtype="uint8"),
                            np.array(upper, dtype="uint8")),
            )
        if iterations > 0:
            element = cv2.getStructuringElement(cv2.MORPH_RECT, tuple(kernel))
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, element, iterations=iterations)
        return mask

    def component_stats(self, mask: Any) -> list[tuple[int, int, int, int, int]]:
        count, _labels, stats, _centroids = self._cv2.connectedComponentsWithStats(mask)
        return [tuple(int(value) for value in stats[index]) for index in range(1, count)]


def _frame_shape(image: Any) -> tuple[int, int, int]:
    shape = getattr(image, "shape", None)
    if not isinstance(shape, Sequence) or isinstance(shape, (str, bytes, bytearray)) or len(shape) != 3:
        raise ValueError(f"frame image must have a rank-3 shape, received {shape!r}")
    height, width, channels = (int(value) for value in shape)
    if height <= 0 or width <= 0 or channels != 3:
        raise ValueError(f"frame image must be HxWx3 BGR, received shape {shape!r}")
    dtype = getattr(image, "dtype", None)
    dtype_name = str(getattr(dtype, "name", dtype))
    if dtype_name.casefold() != "uint8":
        raise ValueError(f"frame image dtype must be uint8, received {dtype_name!r}")
    return height, width, channels


def detect_bottle(
    frame: Any,
    config: BottleDetectorConfig,
    *,
    backend: Any = None,
) -> BottleDetectionResult:
    """在单帧中检测唯一的瓶子候选，fail-closed。"""

    if not isinstance(config, BottleDetectorConfig):
        try:
            config = BottleDetectorConfig(**dict(config))
        except Exception as exc:
            return _failure("invalid_config", f"detector config is invalid: {exc}")
    image = getattr(frame, "image", None)
    frame_id = getattr(frame, "frame_id", None)
    if image is None:
        return _failure("invalid_frame", "frame is missing its image")
    try:
        shape = _frame_shape(image)
    except ValueError as exc:
        return _failure("invalid_frame", str(exc), frame_id=frame_id)
    if backend is None:
        try:
            backend = _OpenCVBackend()
        except Exception as exc:
            return _failure("backend_unavailable", f"OpenCV backend is unavailable: {exc}", frame_id=frame_id, frame_shape=shape)
    try:
        mask = backend.color_mask(
            image,
            config.hsv_lower,
            config.hsv_upper,
            config.morphology_kernel,
            config.morphology_iterations,
        )
        stats = list(backend.component_stats(mask))
    except Exception as exc:
        return _failure("processing_error", f"detection processing failed: {exc}", frame_id=frame_id, frame_shape=shape)

    height, width, _ = shape
    qualifying: list[BottleCandidate] = []
    rejected = 0
    for entry in stats:
        if not isinstance(entry, Sequence) or len(entry) != 5:
            return _failure("processing_error", f"backend returned invalid component stats: {entry!r}", frame_id=frame_id, frame_shape=shape)
        x, y, w, h, area = (int(value) for value in entry)
        if w <= 0 or h <= 0 or area <= 0:
            rejected += 1
            continue
        roi_x, roi_y, roi_w, roi_h = config.roi
        center_x = x + w / 2.0
        center_y = y + h / 2.0
        if not (roi_x <= center_x <= roi_x + roi_w and roi_y <= center_y <= roi_y + roi_h):
            rejected += 1
            continue
        aspect = h / w
        if (
            area < config.min_area
            or area > config.max_area
            or w < config.min_width
            or w > config.max_width
            or h < config.min_height
            or h > config.max_height
            or aspect < config.min_aspect_ratio
            or aspect > config.max_aspect_ratio
        ):
            rejected += 1
            continue
        qualifying.append(
            BottleCandidate(
                bbox=(x, y, w, h),
                area=area,
                center=(x + w / 2.0, y + h / 2.0),
                width_ratio=w / width,
                height_ratio=h / height,
            )
        )
    if not qualifying:
        return _failure(
            "no_qualifying_candidate",
            f"no qualifying candidate ({rejected} rejected)",
            status="no_candidate",
            frame_id=frame_id,
            frame_shape=shape,
            rejected=rejected,
        )
    if len(qualifying) > 1:
        qualifying.sort(key=lambda candidate: -candidate.area)
        if (
            config.dominance_ratio > 0
            and qualifying[0].area >= config.dominance_ratio * qualifying[1].area
        ):
            return BottleDetectionResult(
                "detected",
                "ok",
                f"dominant candidate ({qualifying[0].area} px >= {config.dominance_ratio}x runner-up)",
                qualifying[0],
                rejected + len(qualifying) - 1,
                frame_id,
                shape,
            )
        return _failure(
            "multiple_qualifying_candidates",
            f"{len(qualifying)} comparable qualifying candidates are ambiguous",
            status="ambiguous",
            frame_id=frame_id,
            frame_shape=shape,
            rejected=rejected,
        )
    return BottleDetectionResult("detected", "ok", "unique qualifying candidate", qualifying[0], rejected, frame_id, shape)


@dataclass(frozen=True)
class StableBottleConfig:
    """单次有界采集的连续帧一致性门。"""

    required_frames: int = 3
    acquisition_timeout_seconds: float = 10.0
    max_recaptures: int = 1
    max_center_delta_px: float = 20.0
    max_width_relative_delta: float = 0.25

    def __post_init__(self) -> None:
        object.__setattr__(self, "required_frames", _int_value(self.required_frames, "required_frames", minimum=1))
        object.__setattr__(
            self,
            "acquisition_timeout_seconds",
            _float_value(
                self.acquisition_timeout_seconds,
                "acquisition_timeout_seconds",
                minimum=0.1,
                maximum=MAX_ACQUISITION_TIMEOUT_SECONDS,
            ),
        )
        object.__setattr__(self, "max_recaptures", _int_value(self.max_recaptures, "max_recaptures", minimum=0))
        object.__setattr__(
            self, "max_center_delta_px", _float_value(self.max_center_delta_px, "max_center_delta_px", minimum=0.0)
        )
        object.__setattr__(
            self,
            "max_width_relative_delta",
            _float_value(self.max_width_relative_delta, "max_width_relative_delta", minimum=0.0, maximum=1.0),
        )


@dataclass(frozen=True)
class StableBottleResult:
    status: str
    failure_code: str
    reason: str
    attempts: int
    frames_used: int
    candidate: BottleCandidate | None = None
    frame_id: Any = None
    frame: Any = None

    @property
    def stable(self) -> bool:
        return self.status == "stable" and self.failure_code == "ok" and self.candidate is not None


_FATAL_CODES = frozenset({"invalid_frame", "invalid_config", "backend_unavailable", "processing_error"})


def _consistent(previous: BottleCandidate, current: BottleCandidate, config: StableBottleConfig) -> bool:
    center_delta = math.dist(previous.center, current.center)
    if center_delta > config.max_center_delta_px:
        return False
    reference = max(previous.width_ratio, 1e-9)
    return abs(current.width_ratio - previous.width_ratio) / reference <= config.max_width_relative_delta


def acquire_stable_bottle(
    reader: Any,
    detector_config: BottleDetectorConfig,
    stable_config: StableBottleConfig,
    *,
    detector: Callable[..., BottleDetectionResult] = detect_bottle,
    clock: Callable[[], float] = time.monotonic,
) -> StableBottleResult:
    """返回一个稳定的唯一候选，或一个带类型的 fail-closed 结果。

    reader 必须提供 ``read(timeout=...)``，返回鸭子类型的帧。单次
    有界采集在遇到无候选或歧义帧后，允许 ``max_recaptures`` 次就地
    重启连续帧计数；致命的检测器错误码和 reader 失败会立即结束
    本次采集。
    """

    attempts = 0
    frames_used = 0
    streak: list[tuple[BottleCandidate, Any, Any]] = []
    deadline = clock() + stable_config.acquisition_timeout_seconds
    last_reason = "no frames processed"
    # 失败结果带上最后读到的帧：多次现场事故（2026-08-21 T3 等）因
    # 检测失败时刻的帧未留存而只能靠旁证定位根因。仅附加证据，
    # 不改变任何判定。
    last_frame: Any = None
    last_frame_id: Any = None
    while True:
        remaining = deadline - clock()
        if remaining <= 0:
            return StableBottleResult(
                "timeout", "acquisition_timeout", f"stable acquisition timed out: {last_reason}", attempts, frames_used,
                frame_id=last_frame_id, frame=last_frame,
            )
        try:
            frame = reader.read(timeout=remaining)
        except Exception as exc:
            return StableBottleResult(
                "error", "frame_unavailable", f"frame read failed: {exc}", attempts, frames_used
            )
        frames_used += 1
        result = detector(frame, detector_config)
        last_frame = frame
        last_frame_id = result.frame_id
        if result.failure_code in _FATAL_CODES:
            return StableBottleResult("error", result.failure_code, result.reason, attempts, frames_used)
        if not result.detected:
            last_reason = result.reason
            streak.clear()
            if attempts >= stable_config.max_recaptures:
                return StableBottleResult(
                    "failed", result.failure_code, f"acquisition failed after recapture: {result.reason}", attempts, frames_used,
                    frame_id=last_frame_id, frame=last_frame,
                )
            attempts += 1
            continue
        candidate = result.candidate
        assert candidate is not None
        if streak and not _consistent(streak[-1][0], candidate, stable_config):
            last_reason = "candidate moved beyond the stability thresholds"
            streak.clear()
        streak.append((candidate, result.frame_id, frame))
        if len(streak) >= stable_config.required_frames:
            final_candidate, final_frame_id, final_frame = streak[-1]
            return StableBottleResult(
                "stable", "ok", "consecutive frames agree", attempts, frames_used, final_candidate, final_frame_id, final_frame
            )
