#!/usr/bin/env python3
"""头部相机篮筐投放的 fail-closed 标定配置。

投放 runner 只有在配置完整、数值有限（finite）、身份字段与请求的
camera/workcell 匹配、且每个运动相关数值都落在下方
硬上限之内时，才允许实机执行。任何缺失、越界或不一致都会在任何
运动发生之前抛出 :class:`ConfigError`。本模块不依赖机器人、ROS
或相机。

硬上限（来自变更 spec 与设计）：
单步平移 ``<= 0.10 m``；相对筐沿点的悬停偏移，垂直方向在
``[0.01, 0.50] m`` 内、水平方向在 ``+/-0.40 m`` 内；释放确认容差在
``[0.005, 0.30] m`` 内；工作包络绝对边界在 ``+/-2.0 m`` 内且回撤
位置必须位于包络内；夹爪释放位置必须与标定的持物带可区分；
单一总超时 ``<= 600 s``。
"""

from __future__ import annotations

import json
import math
import numbers
from collections.abc import Mapping, Sequence
from dataclasses import dataclass, fields
from pathlib import Path
from typing import Any


SCHEMA_VERSION = 1

STEP_MAX_M = 0.10
# 2026-08-17 场地：衣物投放的真值偏移为 x=-0.353，超出原 0.30 的
# 合理性上限；每个目标仍由绝对包络约束。
HOVER_OFFSET_XY_MAX_M = 0.40
HOVER_OFFSET_Z_MIN_M = 0.01
HOVER_OFFSET_Z_MAX_M = 0.50
CONFIRM_TOLERANCE_MIN_M = 0.005
CONFIRM_TOLERANCE_MAX_M = 0.30
ENVELOPE_ABS_MAX_M = 2.0
GRIPPER_POSITION_MIN = 0.0
GRIPPER_POSITION_MAX = 360.0
GRIPPER_TOLERANCE_MAX = 5.0
HELD_TOLERANCE_MAX = 20.0
DEPTH_MIN_FLOOR_M = 0.05
DEPTH_MAX_CEILING_M = 6.0
MAX_JOINT_STATE_AGE_CEILING_S = 5.0
OVERALL_TIMEOUT_MAX_S = 600.0
# 颈部俯仰的保守指令区间，位于 URDF Joint_Neck_Pitch 限位之内，
# 与现场验证过的 head_pitch.py 工具一致。
HEAD_PITCH_MIN_DEG = -40.0
HEAD_PITCH_MAX_DEG = 25.0


class ConfigError(ValueError):
    """该投放配置不可信赖，不能用于实机执行。"""


def _require(mapping: Mapping[str, Any], key: str) -> Any:
    if key not in mapping or mapping[key] is None:
        raise ConfigError(f"configuration is missing required field {key!r}")
    return mapping[key]


def _text(value: Any, label: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise ConfigError(f"{label} must be a non-empty string, received {value!r}")
    return value.strip()


def _number(value: Any, label: str, *, minimum: float | None = None, maximum: float | None = None) -> float:
    if isinstance(value, bool) or not isinstance(value, numbers.Real):
        raise ConfigError(f"{label} must be a number, received {value!r}")
    result = float(value)
    if not math.isfinite(result):
        raise ConfigError(f"{label} must be finite, received {value!r}")
    if minimum is not None and result < minimum:
        raise ConfigError(f"{label} must be >= {minimum}, received {result}")
    if maximum is not None and result > maximum:
        raise ConfigError(f"{label} must be <= {maximum}, received {result}")
    return result


def _integer(value: Any, label: str, *, minimum: int, maximum: int) -> int:
    if isinstance(value, bool) or not isinstance(value, numbers.Integral):
        raise ConfigError(f"{label} must be an integer, received {value!r}")
    result = int(value)
    if result < minimum or result > maximum:
        raise ConfigError(f"{label} must be within [{minimum}, {maximum}], received {result}")
    return result


def _hsv_triple(value: Any, label: str) -> tuple[int, int, int]:
    if not isinstance(value, Sequence) or isinstance(value, (str, bytes, bytearray)) or len(value) != 3:
        raise ConfigError(f"{label} must be a sequence of three integers")
    return (
        _integer(value[0], f"{label}[0] (H)", minimum=0, maximum=179),
        _integer(value[1], f"{label}[1] (S)", minimum=0, maximum=255),
        _integer(value[2], f"{label}[2] (V)", minimum=0, maximum=255),
    )


def _pair(value: Any, label: str, *, minimum: float, maximum: float) -> tuple[float, float]:
    if not isinstance(value, Sequence) or isinstance(value, (str, bytes, bytearray)) or len(value) != 2:
        raise ConfigError(f"{label} must be a [min, max] pair")
    low = _number(value[0], f"{label}[0]", minimum=minimum, maximum=maximum)
    high = _number(value[1], f"{label}[1]", minimum=minimum, maximum=maximum)
    if low >= high:
        raise ConfigError(f"{label} must satisfy min < max, received [{low}, {high}]")
    return low, high


def _triple_m(value: Any, label: str, *, minimum: float, maximum: float) -> tuple[float, float, float]:
    if not isinstance(value, Sequence) or isinstance(value, (str, bytes, bytearray)) or len(value) != 3:
        raise ConfigError(f"{label} must be a sequence of three numbers")
    return (
        _number(value[0], f"{label}[0]", minimum=minimum, maximum=maximum),
        _number(value[1], f"{label}[1]", minimum=minimum, maximum=maximum),
        _number(value[2], f"{label}[2]", minimum=minimum, maximum=maximum),
    )


@dataclass(frozen=True)
class BasketDropConfig:
    """一份已验证的投放配置，绑定到特定目标与工位。"""

    schema_version: int  # 配置结构版本，必须等于 SCHEMA_VERSION
    robot_id: str  # 标定出处记录；整机同型号、标定通用，不参与校验
    camera_id: str  # 相机标识
    target_id: str  # 投放目标标识（篮筐）
    workcell_id: str  # 工位标识（场地/摆位）
    # 检测（头部彩色帧）
    hsv_lower: tuple[int, int, int]  # HSV 颜色下界
    hsv_upper: tuple[int, int, int]  # HSV 颜色上界
    min_area: int  # 有效检测的最小连通域面积，像素
    detection_roi_px: tuple[int, int, int, int]  # 检测感兴趣区域 [x, y, 宽, 高]，像素
    # 检测前自动下发的颈部俯仰角（度，负值 = 低头）；
    # runner 分有界小步转到该角度。
    head_pitch_deg: float
    # 深度采样与定位有效性
    rim_inset_px: int  # 深度采样点自筐沿向内收缩的像素数
    depth_window: int  # 深度采样窗口边长，像素，须为奇数
    min_depth_m: float  # 采样深度有效下界，米
    max_depth_m: float  # 采样深度有效上界，米
    max_joint_state_age_s: float  # 关节状态允许的最大时延，秒
    # 运动
    hover_offset_m: tuple[float, float, float]  # 相对筐沿点的悬停偏移 [dx, dy, dz]，米
    confirm_tolerance_m: float  # 到位确认容差，米
    retract_position_m: tuple[float, float, float]  # 释放后的回撤位置（绝对坐标），米
    envelope_x_m: tuple[float, float]  # 允许工作包络 X 范围 [min, max]，米
    envelope_y_m: tuple[float, float]  # 允许工作包络 Y 范围 [min, max]，米
    envelope_z_m: tuple[float, float]  # 允许工作包络 Z 范围 [min, max]，米
    # 横向（Y）运动只允许在 X 位于此线或其后方时进行：篮筐后壁高、
    # 前壁低，所以手必须在越过前沿之前就与筐口横向对齐，返程也必须
    # 先退到此线后方才能横移（2026-08-13 现场教训）。
    y_align_x_m: float
    step_max_m: float  # 单步平移上限，米
    overall_timeout_s: float  # 整个投放流程的总超时，秒
    # 夹爪
    held_feedback_center: float  # 持物状态的期望反馈中心值
    held_feedback_tolerance: float  # 持物验证带半宽
    open_position: float  # 释放（张开）位置
    gripper_tolerance: float  # 夹爪常规到位容差

    def to_record(self) -> dict[str, Any]:
        record: dict[str, Any] = {}
        for item in fields(self):
            value = getattr(self, item.name)
            record[item.name] = list(value) if isinstance(value, tuple) else value
        return record

    def contains(self, position: Sequence[float]) -> bool:
        """绝对位置 (x, y, z) 落在工作包络内时返回 True。"""

        x, y, z = (float(value) for value in position)
        return (
            self.envelope_x_m[0] <= x <= self.envelope_x_m[1]
            and self.envelope_y_m[0] <= y <= self.envelope_y_m[1]
            and self.envelope_z_m[0] <= z <= self.envelope_z_m[1]
        )


def parse_config(value: Any) -> BasketDropConfig:
    """把一个 mapping 验证为可信配置，失败则抛出 ConfigError。"""

    if not isinstance(value, Mapping):
        raise ConfigError(f"configuration must be a JSON object, received {type(value).__name__}")
    version = _integer(_require(value, "schema_version"), "schema_version", minimum=1, maximum=1_000_000)
    if version != SCHEMA_VERSION:
        raise ConfigError(f"schema_version {version} is not supported (expected {SCHEMA_VERSION})")

    hsv_lower = _hsv_triple(_require(value, "hsv_lower"), "hsv_lower")
    hsv_upper = _hsv_triple(_require(value, "hsv_upper"), "hsv_upper")
    if any(low > high for low, high in zip(hsv_lower, hsv_upper)):
        raise ConfigError(f"hsv_lower {hsv_lower} must not exceed hsv_upper {hsv_upper} per channel")

    roi = _require(value, "detection_roi_px")
    if not isinstance(roi, Sequence) or isinstance(roi, (str, bytes, bytearray)) or len(roi) != 4:
        raise ConfigError("detection_roi_px must be an [x, y, width, height] quadruple")
    detection_roi = (
        _integer(roi[0], "detection_roi_px[0]", minimum=0, maximum=10_000),
        _integer(roi[1], "detection_roi_px[1]", minimum=0, maximum=10_000),
        _integer(roi[2], "detection_roi_px[2]", minimum=1, maximum=10_000),
        _integer(roi[3], "detection_roi_px[3]", minimum=1, maximum=10_000),
    )

    depth_window = _integer(_require(value, "depth_window"), "depth_window", minimum=1, maximum=9)
    if depth_window % 2 == 0:
        raise ConfigError(f"depth_window must be odd, received {depth_window}")
    min_depth = _number(
        _require(value, "min_depth_m"), "min_depth_m", minimum=DEPTH_MIN_FLOOR_M, maximum=DEPTH_MAX_CEILING_M
    )
    max_depth = _number(
        _require(value, "max_depth_m"), "max_depth_m", minimum=DEPTH_MIN_FLOOR_M, maximum=DEPTH_MAX_CEILING_M
    )
    if min_depth >= max_depth:
        raise ConfigError("min_depth_m must be below max_depth_m")

    hover = _require(value, "hover_offset_m")
    if not isinstance(hover, Sequence) or isinstance(hover, (str, bytes, bytearray)) or len(hover) != 3:
        raise ConfigError("hover_offset_m must be a [dx, dy, dz] triple")
    hover_offset = (
        _number(hover[0], "hover_offset_m[0]", minimum=-HOVER_OFFSET_XY_MAX_M, maximum=HOVER_OFFSET_XY_MAX_M),
        _number(hover[1], "hover_offset_m[1]", minimum=-HOVER_OFFSET_XY_MAX_M, maximum=HOVER_OFFSET_XY_MAX_M),
        _number(hover[2], "hover_offset_m[2]", minimum=HOVER_OFFSET_Z_MIN_M, maximum=HOVER_OFFSET_Z_MAX_M),
    )

    envelope_x = _pair(_require(value, "envelope_x_m"), "envelope_x_m", minimum=-ENVELOPE_ABS_MAX_M, maximum=ENVELOPE_ABS_MAX_M)
    envelope_y = _pair(_require(value, "envelope_y_m"), "envelope_y_m", minimum=-ENVELOPE_ABS_MAX_M, maximum=ENVELOPE_ABS_MAX_M)
    envelope_z = _pair(_require(value, "envelope_z_m"), "envelope_z_m", minimum=-ENVELOPE_ABS_MAX_M, maximum=ENVELOPE_ABS_MAX_M)
    y_align_x = _number(
        _require(value, "y_align_x_m"), "y_align_x_m", minimum=envelope_x[0], maximum=envelope_x[1]
    )

    retract = _triple_m(
        _require(value, "retract_position_m"),
        "retract_position_m",
        minimum=-ENVELOPE_ABS_MAX_M,
        maximum=ENVELOPE_ABS_MAX_M,
    )
    for axis, (low, high), coordinate in (
        ("x", envelope_x, retract[0]),
        ("y", envelope_y, retract[1]),
        ("z", envelope_z, retract[2]),
    ):
        if not (low <= coordinate <= high):
            raise ConfigError(
                f"retract_position_m {axis}={coordinate} lies outside the envelope [{low}, {high}]"
            )

    held_center = _number(
        _require(value, "held_feedback_center"),
        "held_feedback_center",
        minimum=GRIPPER_POSITION_MIN,
        maximum=GRIPPER_POSITION_MAX,
    )
    held_tolerance = _number(
        _require(value, "held_feedback_tolerance"),
        "held_feedback_tolerance",
        minimum=0.1,
        maximum=HELD_TOLERANCE_MAX,
    )
    open_position = _number(
        _require(value, "open_position"), "open_position", minimum=GRIPPER_POSITION_MIN, maximum=GRIPPER_POSITION_MAX
    )
    gripper_tolerance = _number(
        _require(value, "gripper_tolerance"), "gripper_tolerance", minimum=0.1, maximum=GRIPPER_TOLERANCE_MAX
    )
    # 已释放的夹爪读数绝不能落进持物验证带；仍持物的夹爪
    # 也绝不能被验证为已释放。
    if abs(held_center - open_position) <= held_tolerance + gripper_tolerance:
        raise ConfigError(
            "open_position must sit outside the holding band: "
            f"|{held_center} - {open_position}| <= {held_tolerance} + {gripper_tolerance}"
        )

    return BasketDropConfig(
        schema_version=version,
        robot_id=_text(_require(value, "robot_id"), "robot_id"),
        camera_id=_text(_require(value, "camera_id"), "camera_id"),
        target_id=_text(_require(value, "target_id"), "target_id"),
        workcell_id=_text(_require(value, "workcell_id"), "workcell_id"),
        hsv_lower=hsv_lower,
        hsv_upper=hsv_upper,
        min_area=_integer(_require(value, "min_area"), "min_area", minimum=1, maximum=307_200),
        detection_roi_px=detection_roi,
        head_pitch_deg=_number(
            _require(value, "head_pitch_deg"),
            "head_pitch_deg",
            minimum=HEAD_PITCH_MIN_DEG,
            maximum=HEAD_PITCH_MAX_DEG,
        ),
        rim_inset_px=_integer(_require(value, "rim_inset_px"), "rim_inset_px", minimum=0, maximum=100),
        depth_window=depth_window,
        min_depth_m=min_depth,
        max_depth_m=max_depth,
        max_joint_state_age_s=_number(
            _require(value, "max_joint_state_age_s"),
            "max_joint_state_age_s",
            minimum=0.05,
            maximum=MAX_JOINT_STATE_AGE_CEILING_S,
        ),
        hover_offset_m=hover_offset,
        confirm_tolerance_m=_number(
            _require(value, "confirm_tolerance_m"),
            "confirm_tolerance_m",
            minimum=CONFIRM_TOLERANCE_MIN_M,
            maximum=CONFIRM_TOLERANCE_MAX_M,
        ),
        retract_position_m=retract,
        envelope_x_m=envelope_x,
        envelope_y_m=envelope_y,
        envelope_z_m=envelope_z,
        y_align_x_m=y_align_x,
        step_max_m=_number(_require(value, "step_max_m"), "step_max_m", minimum=0.005, maximum=STEP_MAX_M),
        overall_timeout_s=_number(
            _require(value, "overall_timeout_s"), "overall_timeout_s", minimum=1.0, maximum=OVERALL_TIMEOUT_MAX_S
        ),
        held_feedback_center=held_center,
        held_feedback_tolerance=held_tolerance,
        open_position=open_position,
        gripper_tolerance=gripper_tolerance,
    )


def load_config(path: Any) -> BasketDropConfig:
    """读取、解析并验证 ``path`` 处的 JSON 配置文件。"""

    location = Path(path)
    try:
        raw = location.read_text(encoding="utf-8")
    except OSError as exc:
        raise ConfigError(f"configuration file {location} is unreadable: {exc}") from exc
    try:
        decoded = json.loads(raw)
    except json.JSONDecodeError as exc:
        raise ConfigError(f"configuration file {location} is not valid JSON: {exc}") from exc
    return parse_config(decoded)


def require_identity(
    config: BasketDropConfig,
    *,
    camera_id: str,
    workcell_id: str,
) -> BasketDropConfig:
    """配置若是为其他相机或工位生成的，则拒绝执行。

    ``target_id`` 只作记录（写入证据），不参与比对：同一工位换目标
    颜色只改 JSON 即可。
    """

    expected = {
        "camera_id": camera_id,
        "workcell_id": workcell_id,
    }
    for name, wanted in expected.items():
        actual = getattr(config, name)
        if actual != wanted:
            raise ConfigError(f"configuration {name} {actual!r} does not match required {wanted!r}")
    return config
