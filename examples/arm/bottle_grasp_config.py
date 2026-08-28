#!/usr/bin/env python3
"""瓶子抓取的 fail-closed 标定配置。

runner 只有在配置完整、数值有限（finite）、身份字段与请求的
camera/workcell 匹配、且每个运动相关数值都落在下方
现场批准的硬上限之内时，才允许实机执行。任何缺失、越界或不一致
都会在任何运动发生之前抛出 :class:`ConfigError`。本模块不依赖
机器人、ROS 或相机。

硬上限（2026-08-13 现场批准，见变更设计；前伸上限于 2026-08-13
经用户批准放宽以适配更远的瓶子摆位——原 ``0.58 m`` 绝对 /
``0.53 m`` 前伸上限对应相机碰撞点约在 ``0.60`` 的瓶位；当前摆位
远了约 ``0.20 m``，碰撞点相应外移）：
左臂几何数值镜像自右臂实测标定值，未独立复测。例外：column_to_y_sign 于
2026-08-22 经左腕相机物理真值比对实测为 -1（与右臂相同；瓶子朝机器人右侧
平移 5 cm 时画面列 288→411，即图像右 = 机器人右，相机正立安装不镜像）；grasp_z_m 0.90 与 grip_feedback_center 188/close_tolerance 5
为 2026-08-22 左臂实机标定值（夹住瓶子时左爪堵转反馈四次实测 190.4/187.2/187.5/185.4，取中 188）；detection_roi_px y 起点 100 与 vertical_window_px 下限 100 为左腕相机
现场值（抓取高度首帧瓶子中心行实测 154–177，右臂值 160/140 会在首帧拒检）。
final_forward_m 0.12 沿用右臂值（2026-08-22 曾因 FINAL_FORWARD 收敛超时降到 0.09/0.07，根因实为
跑器旋转门 1° 过严而非前伸过长，旋转门放宽到 3° 后改回）。
左臂 X 绝对范围 ``0.04-0.70 m``，
自观察锚点起的总前伸 ``<= 0.65 m``，X 粗/细步长
``<= 0.05 / 0.005 m``，Y 单步 ``<= 0.005 m`` 且累计 ``<= 0.35 m``
（上调历史见 ``Y_CUMULATIVE_MAX_M`` 旁注释），
可选的最终盲推 ``<= 0.15 m``，抓取高度不低于复位高度以下
``0.25 m``，闭合角度 ``10-360``，验证带宽不超过 ``+/-5``。
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

X_ABSOLUTE_MIN_M = 0.04
X_ABSOLUTE_MAX_M = 0.70
X_TOTAL_FORWARD_MAX_M = 0.65
X_COARSE_STEP_MAX_M = 0.05
X_FINE_STEP_MAX_M = 0.005
Y_STEP_MAX_M = 0.005
# 原始值 0.10；2026-08-19 经用户批准上调到 0.20（红瓶偏置 run 需要
# 17 cm 的 Y 跟踪）；2026-08-21 经用户批准先后上调到 0.30 和 0.35：
# 左极限瓶位 run 20260821T050902Z 和 20260821T051433Z 分别耗尽了
# 0.20 和 0.30，后者在宽度比 0.303（闭合带 0.39）时距对中仅剩约
# 8 mm——接近过程中透视漂移会持续消耗 Y 额度。若 0.35 也被耗尽，
# 不要再上调此上限，转而修正基座停靠位姿。单步 0.005 m 上限、
# 每步反馈、0.8 倍前伸的中线方向预算、整体超时仍是失控防护。
Y_CUMULATIVE_MAX_M = 0.35
FINAL_FORWARD_MAX_M = 0.15
# 原始值 0.10（为了让手部相机视野更宽）；实测对检测没有帮助，还白白
# 消耗 0.10 m 中线方向的工作空间，故 2026-08-21 起改为 0。此阶段的
# 观察锚点仍从实时反馈建立。
BACK_CLEARANCE_M = 0.0
# 机械臂 daemon 会静默拒绝 IK 解自碰撞的目标。2026-08-21 实测：
# 从复位位姿出发，向中线方向（+Y）的可达范围与前伸大致 1:1 增长
# （前伸 10 cm 则左移 10 cm 可通过，20 cm 对应 20 cm）。+Y 跟踪
# 预算取前伸的 0.8 倍以留余量。
MIDLINE_Y_PER_FORWARD_RATIO = 0.8
LIFT_M = 0.10
GRASP_Z_RELATIVE_FLOOR_M = 0.25
GRIPPER_OPEN_POSITION = 10.0
GRIPPER_CLOSE_MIN = 10.0
GRIPPER_CLOSE_MAX = 360.0
GRIP_TOLERANCE_MAX = 5.0
OVERALL_TIMEOUT_MAX_S = 600.0


class ConfigError(ValueError):
    """该抓取配置不可信赖，不能用于实机执行。"""


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


def _triple(value: Any, label: str) -> tuple[int, int, int]:
    if not isinstance(value, Sequence) or isinstance(value, (str, bytes, bytearray)) or len(value) != 3:
        raise ConfigError(f"{label} must be a sequence of three integers")
    return (
        _integer(value[0], f"{label}[0]", minimum=0, maximum=179 if label.endswith("lower") or label.endswith("upper") else 255),
        _integer(value[1], f"{label}[1]", minimum=0, maximum=255),
        _integer(value[2], f"{label}[2]", minimum=0, maximum=255),
    )


@dataclass(frozen=True)
class BottleGraspConfig:
    """一份已验证的抓取配置，绑定到特定目标与工位。"""

    schema_version: int  # 配置结构版本，必须等于 SCHEMA_VERSION
    robot_id: str  # 标定出处记录；整机同型号、标定通用，不参与校验
    camera_id: str  # 相机标识
    target_id: str  # 抓取目标标识（如具体颜色的瓶子）
    workcell_id: str  # 工位标识（场地/摆位）
    # 检测
    hsv_lower: tuple[int, int, int]  # HSV 颜色下界；H 允许绕环（低 > 高，如红色）
    hsv_upper: tuple[int, int, int]  # HSV 颜色上界
    min_area: int  # 有效检测的最小连通域面积，像素
    detection_roi_px: tuple[int, int, int, int]  # 检测感兴趣区域 [x, y, 宽, 高]，像素
    # 对准（图像列 <-> 机器人 Y）
    target_column_px: float  # 对中目标像素列：检测框中心应对准的列坐标
    column_tolerance_px: float  # 列对准容差，像素；误差在带内视为已对中
    column_to_y_sign: int  # 像素列误差映射到机器人 Y 方向的符号，-1 或 +1
    vertical_window_px: tuple[float, float]  # 检测框中心的有效垂直窗口 [min, max]，像素
    # 运动
    grasp_z_m: float  # 抓取高度（基座坐标 Z），米
    y_step_m: float  # Y 方向单步伺服步长，米
    x_coarse_step_m: float  # X 前进粗步长，米（远离目标时用）
    x_fine_step_m: float  # X 前进细步长，米（进入细步进区后用）
    fine_zone_width_ratio: float  # 宽度比（检测框宽/图像宽）达到该值后改用细步进
    close_width_ratio_min: float  # 允许闭合的宽度比下界（到达闭合带）
    close_width_ratio_max: float  # 宽度比超过此上界视为冲过头，中止且不自动回退
    final_forward_m: float  # 闭合前的最终盲推距离，米；0 表示禁用
    x_absolute_min_m: float  # 左臂 X 绝对下界，米
    x_absolute_max_m: float  # 左臂 X 绝对上界，米
    # 抓取
    close_position: float  # 夹爪闭合目标角度（空夹时会精确收敛到该值）
    grip_feedback_center: float  # 夹到瓶身被堵转时的期望反馈中心值
    close_tolerance: float  # 反馈验证带半宽：feedback ∈ center±tolerance 才算抓住
    overall_timeout_s: float  # 整个抓取流程的总超时，秒

    def to_record(self) -> dict[str, Any]:
        record: dict[str, Any] = {}
        for item in fields(self):
            value = getattr(self, item.name)
            record[item.name] = list(value) if isinstance(value, tuple) else value
        return record


def parse_config(value: Any) -> BottleGraspConfig:
    """把一个 mapping 验证为可信配置，失败则抛出 ConfigError。"""

    if not isinstance(value, Mapping):
        raise ConfigError(f"configuration must be a JSON object, received {type(value).__name__}")
    version = _integer(_require(value, "schema_version"), "schema_version", minimum=1, maximum=1_000_000)
    if version != SCHEMA_VERSION:
        raise ConfigError(f"schema_version {version} is not supported (expected {SCHEMA_VERSION})")

    hsv_lower = _triple(_require(value, "hsv_lower"), "hsv_lower")
    hsv_upper = _triple(_require(value, "hsv_upper"), "hsv_upper")
    # 只校验 S/V 两通道：色相 H 允许绕环（下界 H > 上界 H，如红色
    # 170..10）。必须与 BottleDetectorConfig.__post_init__ 以及
    # detect_bottle._OpenCVBackend.color_mask 的绕环分支保持一致。
    if any(low > high for low, high in zip(hsv_lower[1:], hsv_upper[1:])):
        raise ConfigError(f"hsv_lower {hsv_lower} must not exceed hsv_upper {hsv_upper} per channel")

    window = _require(value, "vertical_window_px")
    if not isinstance(window, Sequence) or isinstance(window, (str, bytes, bytearray)) or len(window) != 2:
        raise ConfigError("vertical_window_px must be a [min, max] pair")
    window_min = _number(window[0], "vertical_window_px[0]", minimum=0.0, maximum=10_000.0)
    window_max = _number(window[1], "vertical_window_px[1]", minimum=0.0, maximum=10_000.0)
    if window_min >= window_max:
        raise ConfigError("vertical_window_px must satisfy min < max")

    sign = _integer(_require(value, "column_to_y_sign"), "column_to_y_sign", minimum=-1, maximum=1)
    if sign == 0:
        raise ConfigError("column_to_y_sign must be -1 or +1")

    roi = _require(value, "detection_roi_px")
    if not isinstance(roi, Sequence) or isinstance(roi, (str, bytes, bytearray)) or len(roi) != 4:
        raise ConfigError("detection_roi_px must be an [x, y, width, height] quadruple")
    detection_roi = (
        _integer(roi[0], "detection_roi_px[0]", minimum=0, maximum=10_000),
        _integer(roi[1], "detection_roi_px[1]", minimum=0, maximum=10_000),
        _integer(roi[2], "detection_roi_px[2]", minimum=1, maximum=10_000),
        _integer(roi[3], "detection_roi_px[3]", minimum=1, maximum=10_000),
    )

    x_abs_min = _number(_require(value, "x_absolute_min_m"), "x_absolute_min_m", minimum=X_ABSOLUTE_MIN_M, maximum=X_ABSOLUTE_MAX_M)
    x_abs_max = _number(_require(value, "x_absolute_max_m"), "x_absolute_max_m", minimum=X_ABSOLUTE_MIN_M, maximum=X_ABSOLUTE_MAX_M)
    if x_abs_min >= x_abs_max:
        raise ConfigError("x_absolute_min_m must be below x_absolute_max_m")

    close_min = _number(_require(value, "close_width_ratio_min"), "close_width_ratio_min", minimum=0.01, maximum=1.0)
    close_max = _number(_require(value, "close_width_ratio_max"), "close_width_ratio_max", minimum=0.01, maximum=1.0)
    if close_min >= close_max:
        raise ConfigError("close_width_ratio_min must be below close_width_ratio_max")
    fine_zone = _number(_require(value, "fine_zone_width_ratio"), "fine_zone_width_ratio", minimum=0.01, maximum=1.0)
    if fine_zone > close_min:
        raise ConfigError("fine_zone_width_ratio must not exceed close_width_ratio_min")

    close_position = _number(
        _require(value, "close_position"), "close_position", minimum=GRIPPER_CLOSE_MIN, maximum=GRIPPER_CLOSE_MAX
    )
    close_tolerance = _number(
        _require(value, "close_tolerance"), "close_tolerance", minimum=0.1, maximum=GRIP_TOLERANCE_MAX
    )
    grip_center = _number(
        _require(value, "grip_feedback_center"),
        "grip_feedback_center",
        minimum=GRIPPER_CLOSE_MIN,
        maximum=GRIPPER_CLOSE_MAX,
    )
    # 空夹（没夹到东西）的夹爪会精确收敛到 close_position；验证带必须
    # 排除该值，否则抓空也会通过验证。
    if grip_center + close_tolerance >= close_position:
        raise ConfigError(
            "grip_feedback_center + close_tolerance must stay below close_position "
            f"({grip_center} + {close_tolerance} >= {close_position}); a closed-on-air "
            "gripper would otherwise pass verification"
        )

    return BottleGraspConfig(
        schema_version=version,
        robot_id=_text(_require(value, "robot_id"), "robot_id"),
        camera_id=_text(_require(value, "camera_id"), "camera_id"),
        target_id=_text(_require(value, "target_id"), "target_id"),
        workcell_id=_text(_require(value, "workcell_id"), "workcell_id"),
        hsv_lower=hsv_lower,
        hsv_upper=hsv_upper,
        min_area=_integer(_require(value, "min_area"), "min_area", minimum=1, maximum=307_200),
        detection_roi_px=detection_roi,
        target_column_px=_number(_require(value, "target_column_px"), "target_column_px", minimum=0.0, maximum=10_000.0),
        column_tolerance_px=_number(
            _require(value, "column_tolerance_px"), "column_tolerance_px", minimum=1.0, maximum=1_000.0
        ),
        column_to_y_sign=sign,
        vertical_window_px=(window_min, window_max),
        grasp_z_m=_number(_require(value, "grasp_z_m"), "grasp_z_m", minimum=0.3, maximum=1.5),
        y_step_m=_number(_require(value, "y_step_m"), "y_step_m", minimum=0.0005, maximum=Y_STEP_MAX_M),
        x_coarse_step_m=_number(
            _require(value, "x_coarse_step_m"), "x_coarse_step_m", minimum=0.001, maximum=X_COARSE_STEP_MAX_M
        ),
        x_fine_step_m=_number(
            _require(value, "x_fine_step_m"), "x_fine_step_m", minimum=0.0005, maximum=X_FINE_STEP_MAX_M
        ),
        fine_zone_width_ratio=fine_zone,
        close_width_ratio_min=close_min,
        close_width_ratio_max=close_max,
        final_forward_m=_number(
            _require(value, "final_forward_m"), "final_forward_m", minimum=0.0, maximum=FINAL_FORWARD_MAX_M
        ),
        x_absolute_min_m=x_abs_min,
        x_absolute_max_m=x_abs_max,
        close_position=close_position,
        grip_feedback_center=grip_center,
        close_tolerance=close_tolerance,
        overall_timeout_s=_number(
            _require(value, "overall_timeout_s"), "overall_timeout_s", minimum=1.0, maximum=OVERALL_TIMEOUT_MAX_S
        ),
    )


def load_config(path: Any) -> BottleGraspConfig:
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
    config: BottleGraspConfig,
    *,
    camera_id: str,
    workcell_id: str,
) -> BottleGraspConfig:
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
