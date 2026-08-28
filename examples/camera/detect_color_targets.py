#!/usr/bin/env python3
"""头部相机帧上的衣物与黄色篮筐检测。

检测引擎就是 ``detect_bottle.py`` 里现场验证过的机制（HSV 区间
掩膜、有界形态学、连通域、尺寸/长宽比/ROI 过滤、唯一或占优候选
规则、连续帧稳定性门）。这套机制与颜色无关、完全配置驱动，因此
本模块刻意委托给它，而不是按目标 fork 副本：一套经过测试的
fail-closed 实现服务所有颜色目标。各目标的差异只有一个默认值
dict 加各自一个小的目标专属辅助函数：

- 衣物（蓝色叠好衣物）：``masked_depth_median``——黑色绒布桌布
  吸收红外，衣物周围的深度图布满空洞。抓取点深度取"腐蚀后"衣物
  颜色掩膜内原始 ``uint16`` 深度值的中位数（腐蚀去掉噪声多的掩膜
  边缘），带最少有效像素门；反投影像素取同一批有效像素的质心。
- 黄色篮筐：``basket_reference_pixel``——筐沿参考像素是候选检测框
  顶边中心向下内缩后的位置。现场实测（2026-08-13 point3 帧，
  task 2.2）表明：正好在顶边上的深度会渗入筐沿后方 1.5 m 的
  背景地面，而顶边下方 10-15 px 处能锁定真实筐沿表面；因此内缩量
  是标定配置的一部分。

共享引擎产生的失败原因文案会提到 "green candidate"；记录证据的
调用方应依赖带类型的 ``failure_code`` 字段，它们与目标无关。
"""

from __future__ import annotations

import importlib.util
import numbers
import statistics
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any


_ENGINE_CACHE: dict[str, Any] = {}


def _engine() -> Any:
    """按文件路径加载同目录的检测引擎，带缓存。"""

    if "engine" in _ENGINE_CACHE:
        return _ENGINE_CACHE["engine"]
    module_path = Path(__file__).resolve().parent / "detect_bottle.py"
    spec = importlib.util.spec_from_file_location("xr_color_targets_engine", module_path)
    if spec is None or spec.loader is None:  # pragma: no cover - repo layout invariant
        raise ImportError(f"could not load detection engine from {module_path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    _ENGINE_CACHE["engine"] = module
    return module


def stable_config(**overrides: Any) -> Any:
    """构建连续帧稳定性门的配置。"""

    return _engine().StableBottleConfig(**overrides)


# ---------------------------------------------------------------------------
# 衣物目标（黑桌布上的蓝色叠好衣物）
# ---------------------------------------------------------------------------

# 蓝色衣物默认值；每个值都可被标定抓取配置覆盖
# （比赛现场可能临时更换目标颜色）。
CLOTHES_DEFAULTS: dict[str, Any] = {
    "hsv_lower": (100, 80, 40),
    "hsv_upper": (130, 255, 255),
    "min_area": 3000,
    "min_aspect_ratio": 0.05,
    "max_aspect_ratio": 3.0,
}

MAX_ERODE_KERNEL_PX = 31


def clothes_detector_config(**overrides: Any) -> Any:
    """用衣物默认值构建已验证的检测器配置。"""

    merged = dict(CLOTHES_DEFAULTS)
    merged.update(overrides)
    return _engine().BottleDetectorConfig(**merged)


def detect_clothes(frame: Any, config: Any, *, backend: Any = None) -> Any:
    """在单帧中检测唯一的衣物候选，fail-closed。"""

    return _engine().detect_bottle(frame, config, backend=backend)


def acquire_stable_clothes(
    reader: Any,
    detector_configuration: Any,
    stable_configuration: Any,
    **kwargs: Any,
) -> Any:
    """返回一个稳定的唯一候选，或一个带类型的 fail-closed 结果。"""

    return _engine().acquire_stable_bottle(
        reader, detector_configuration, stable_configuration, **kwargs
    )


@dataclass(frozen=True)
class MaskedDepthSample:
    """腐蚀后衣物掩膜内的原始深度中位数及其像素质心。"""

    depth_raw: float
    pixel: tuple[float, float]
    valid_px: int


class _OpenCVMaskBackend:
    """真实的 HSV 掩膜 + 腐蚀，惰性加载以便离线 import。"""

    def __init__(self) -> None:
        import cv2
        import numpy

        self._cv2 = cv2
        self._np = numpy

    def eroded_mask(
        self,
        image: Any,
        lower: tuple[int, int, int],
        upper: tuple[int, int, int],
        kernel_px: int,
    ) -> Any:
        cv2 = self._cv2
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(
            hsv,
            self._np.array(lower, dtype="uint8"),
            self._np.array(upper, dtype="uint8"),
        )
        element = cv2.getStructuringElement(cv2.MORPH_RECT, (kernel_px, kernel_px))
        return cv2.erode(mask, element, iterations=1)


def _positive_odd(value: Any, label: str, maximum: int) -> int:
    if isinstance(value, bool) or not isinstance(value, numbers.Integral):
        raise ValueError(f"{label} must be an integer, received {value!r}")
    result = int(value)
    if result < 1 or result > maximum or result % 2 == 0:
        raise ValueError(f"{label} must be an odd integer within [1, {maximum}], received {result}")
    return result


def masked_depth_median(
    frame: Any,
    candidate: Any,
    *,
    hsv_lower: tuple[int, int, int],
    hsv_upper: tuple[int, int, int],
    erode_kernel_px: int,
    min_valid_px: int,
    backend: Any = None,
) -> MaskedDepthSample:
    """腐蚀后衣物掩膜内、限制在候选检测框范围里的原始深度中位数。
    输入非法，或同时带掩膜且深度非零的像素少于 ``min_valid_px`` 时，
    fail-closed 抛出 ``ValueError``（吸收红外的桌布上的空洞被排除
    在外）。
    """

    kernel_px = _positive_odd(erode_kernel_px, "erode_kernel_px", MAX_ERODE_KERNEL_PX)
    if isinstance(min_valid_px, bool) or not isinstance(min_valid_px, numbers.Integral) or min_valid_px < 1:
        raise ValueError(f"min_valid_px must be a positive integer, received {min_valid_px!r}")

    image = getattr(frame, "image", None)
    depth = getattr(frame, "depth", None)
    if image is None or depth is None:
        raise ValueError("frame must carry both a color image and a depth image")
    depth_dtype = getattr(depth, "dtype", None)
    if depth_dtype is not None:
        dtype_name = str(getattr(depth_dtype, "name", depth_dtype))
        if dtype_name.casefold() != "uint16":
            raise ValueError(f"depth image dtype must be raw uint16, received {dtype_name!r}")

    bbox = getattr(candidate, "bbox", None)
    try:
        x, y, w, h = (int(value) for value in bbox)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"candidate bbox must be an (x, y, w, h) quadruple, received {bbox!r}") from exc
    if w <= 0 or h <= 0:
        raise ValueError(f"candidate bbox must have positive size, received {bbox!r}")

    if backend is None:
        backend = _OpenCVMaskBackend()
    mask = backend.eroded_mask(image, tuple(hsv_lower), tuple(hsv_upper), kernel_px)

    rows = len(depth)
    columns = len(depth[0]) if rows else 0
    if len(mask) != rows or (rows and len(mask[0]) != columns):
        raise ValueError("mask and depth image sizes disagree")

    values: list[float] = []
    sum_u = 0.0
    sum_v = 0.0
    for v in range(max(0, y), min(rows, y + h)):
        mask_row = mask[v]
        depth_row = depth[v]
        for u in range(max(0, x), min(columns, x + w)):
            if mask_row[u] and depth_row[u] > 0:
                values.append(float(depth_row[u]))
                sum_u += u
                sum_v += v
    if len(values) < min_valid_px:
        raise ValueError(
            f"only {len(values)} valid masked depth pixels, below the required {min_valid_px}"
        )
    count = len(values)
    return MaskedDepthSample(
        # median_low 保证 depth_raw 是实际观测到的原始 uint16 值：
        # 偶数个样本时普通中位数会把中间两值平均出 .5，
        # 被下游的整数型深度验证拒绝。
        depth_raw=float(statistics.median_low(values)),
        pixel=(sum_u / count, sum_v / count),
        valid_px=count,
    )


# ---------------------------------------------------------------------------
# 黄色篮筐目标（投放点自上而下看到的筐沿）
# ---------------------------------------------------------------------------

# 黄色篮筐默认值；每个值都可被标定投放配置覆盖。长宽比窗口接受
# 瓶子默认值会拒绝的扁宽筐沿连通域（h/w 远小于 1）。
YELLOW_BASKET_DEFAULTS: dict[str, Any] = {
    "hsv_lower": (20, 80, 60),
    "hsv_upper": (32, 255, 255),
    "min_area": 2000,
    "min_aspect_ratio": 0.05,
    "max_aspect_ratio": 3.0,
}


def basket_detector_config(**overrides: Any) -> Any:
    """用黄色篮筐默认值构建已验证的检测器配置。"""

    merged = dict(YELLOW_BASKET_DEFAULTS)
    merged.update(overrides)
    return _engine().BottleDetectorConfig(**merged)


# 现场实测默认值（2026-08-13，point3）：连通域顶边下方约 6 px 内的
# 深度被背景污染，约 10 px 以下开始稳定。
RIM_INSET_DEFAULT_PX = 12


def basket_reference_pixel(candidate: Any, *, rim_inset_px: int = RIM_INSET_DEFAULT_PX) -> tuple[float, float]:
    """返回筐沿参考像素：顶边中心向下内缩后的位置。

    内缩量会被截断在检测框内。候选没有可用的 4 元 bbox 或内缩量为
    负时抛出 ``ValueError``。
    """

    bbox = getattr(candidate, "bbox", None)
    try:
        x, y, w, h = (float(value) for value in bbox)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"candidate bbox must be an (x, y, w, h) quadruple, received {bbox!r}") from exc
    if w <= 0 or h <= 0:
        raise ValueError(f"candidate bbox must have positive size, received {bbox!r}")
    if not isinstance(rim_inset_px, int) or isinstance(rim_inset_px, bool) or rim_inset_px < 0:
        raise ValueError(f"rim_inset_px must be a non-negative integer, received {rim_inset_px!r}")
    return (x + w / 2.0, y + min(float(rim_inset_px), h - 1.0))


def detect_yellow_basket(frame: Any, config: Any, *, backend: Any = None) -> Any:
    """在单帧中检测唯一的黄色篮筐候选，fail-closed。"""

    return _engine().detect_bottle(frame, config, backend=backend)


def acquire_stable_yellow_basket(
    reader: Any,
    detector_configuration: Any,
    stable_configuration: Any,
    **kwargs: Any,
) -> Any:
    """返回一个稳定的唯一候选，或一个带类型的 fail-closed 结果。"""

    return _engine().acquire_stable_bottle(
        reader, detector_configuration, stable_configuration, **kwargs
    )


__all__ = [
    "CLOTHES_DEFAULTS",
    "YELLOW_BASKET_DEFAULTS",
    "MaskedDepthSample",
    "clothes_detector_config",
    "basket_detector_config",
    "stable_config",
    "detect_clothes",
    "detect_yellow_basket",
    "acquire_stable_clothes",
    "acquire_stable_yellow_basket",
    "masked_depth_median",
    "basket_reference_pixel",
]
