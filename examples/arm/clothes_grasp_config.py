#!/usr/bin/env python3
"""point4 衣物抓取的 fail-closed 标定配置。

抓取 runner 只有在配置完整、数值有限（finite）、身份字段与请求的
camera/workcell 匹配、且每个运动相关数值都落在下方
硬上限之内时，才允许实机执行。任何缺失、越界或不一致都会在任何
运动发生之前抛出 :class:`ConfigError`。本模块不依赖机器人、ROS
或相机。

桌面安全高度的推导（spec 的"单一字段"规则）：配置只暴露唯一一个
桌面高度字段，外加标定好的指尖到 EEF 的垂直偏移（标定捏取姿态下
的工具几何；2026-08-14 在 54 度俯仰下实测 0.185 m）。所有安全
高度都在代码中由这两者推导，以 EEF 坐标表达（EEF 值 = 指尖坐标
高度 + 指尖偏移）：捏取高度（指尖位于 ``table + PINCH_OFFSET_M``
——是"深压"，现场验证过：轻触什么都捏不到）、硬 Z 下限（指尖位于
``table - FLOOR_OFFSET_M``；更低的目标一律拒绝，绝不截断放行）、
低区上限（指尖位于 ``table + SAFE_CLEARANCE_M``；低于它只允许
纯垂直运动，保证夹爪不会横扫到桌沿，也不会钩住轻质桌上垂下的
桌布）。感知偏移的 Z 分量只用于修正桌面一致性门所用的感知桌面
高度，绝不移动上述推导高度。

2026-08-22 robot-260 左臂现场值：grasp_offset_m Z -0.02（274 的 +0.096 是其相机
偏差；260 卷尺桌高 0.75、衣厚 3-4 cm、感知衣面 0.8035 → 相机读高约 2 cm）；
fingertip_offset_z_m 0.235（左臂同指令 Z 比右臂实际低约 5 cm，抓瓶段 0.85→0.90
实测推得，方向 fail-safe）；envelope_x_m 上限 0.78（用户决策 2026-08-22，现场监督：
衣物捏取目标 x=0.733 超出右臂沿用的 0.70）；cloth_feedback_center 330±20（run 20260822T032459Z 左爪捏住衣物
堵转反馈 320.7、torque 1.37，右爪值 352 会判捏空；布量不可预计故带取宽，上沿 350
仍低于捏空收敛值 360）；pinch_rotation_quat 取镜像值 [0.626, 0.324, -0.322, 0.632]（2026-08-22 实测：两臂复位
四元数相同，镜像值=右臂值绕夹爪指向轴转 180° 的等价姿态，POSTURE 需转 179.5° 而非 54°；
但改用右臂原值后 run 20260822T034200Z 在 ALIGN 阶段手臂不动、目标不收敛（该腕部构型下
左臂前伸不可达），而镜像值 run 20260822T032459Z 抓取成功——以实机成功者为准）。其余左臂几何数值（抓取偏移、捏取姿态四元数、Y 向包络）镜像自右臂
实测值，未独立复测。
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

# 推导高度用常量（spec：在桌面 + 0-0.02 m 的深压带内捏取；
# 2026-08-14 现场标定——成功夹住的那次用的约 +0.01）。
PINCH_OFFSET_M = 0.01
FLOOR_OFFSET_M = 0.02
SAFE_CLEARANCE_M = 0.15
FINGERTIP_OFFSET_MIN_M = 0.0
FINGERTIP_OFFSET_MAX_M = 0.35
QUAT_NORM_TOLERANCE = 0.01

TABLE_HEIGHT_MIN_M = 0.5
TABLE_HEIGHT_MAX_M = 1.2
CONSISTENCY_WINDOW_MIN_M = 0.01
CONSISTENCY_WINDOW_MAX_M = 0.15
OFFSET_XY_MAX_M = 0.30
OFFSET_Z_MAX_M = 0.10
RETRY_OFFSET_MAX_M = 0.05
STEP_MAX_M = 0.10
DESCEND_STEP_MAX_M = 0.05
ENVELOPE_ABS_MAX_M = 2.0
GRIPPER_POSITION_MIN = 0.0
GRIPPER_POSITION_MAX = 360.0
CLOSE_POSITION_MIN = 10.0
GRIPPER_TOLERANCE_MAX = 5.0
# 右爪捏衣堵转值与 close_position 只差 8，带只能 ±5；左爪实测差约 40（2026-08-22），
# 且布量不可预计、堵转位置随之浮动，上限放到 20——验证逻辑仍要求 close_position
# 落在带外（见下文校验），捏空必拒。
CLOTH_TOLERANCE_MAX = 20.0
DEPTH_MIN_FLOOR_M = 0.05
DEPTH_MAX_CEILING_M = 6.0
MAX_JOINT_STATE_AGE_CEILING_S = 5.0
OVERALL_TIMEOUT_MAX_S = 600.0
MAX_ERODE_KERNEL_PX = 31
# 颈部俯仰的保守指令区间，位于 URDF Joint_Neck_Pitch 限位之内，
# 与现场验证过的 head_pitch.py 工具一致。
HEAD_PITCH_MIN_DEG = -40.0
HEAD_PITCH_MAX_DEG = 25.0


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


@dataclass(frozen=True)
class ClothesGraspConfig:
    """一份已验证的抓取配置，绑定到特定目标与工位。"""

    schema_version: int  # 配置结构版本，必须等于 SCHEMA_VERSION
    robot_id: str  # 标定出处记录；整机同型号、标定通用，不参与校验
    camera_id: str  # 相机标识
    target_id: str  # 抓取目标标识
    workcell_id: str  # 工位标识（场地/摆位）
    # 检测（头部彩色帧；目标颜色由配置决定）
    hsv_lower: tuple[int, int, int]  # HSV 颜色下界
    hsv_upper: tuple[int, int, int]  # HSV 颜色上界
    min_area: int  # 有效检测的最小连通域面积，像素
    detection_roi_px: tuple[int, int, int, int]  # 检测感兴趣区域 [x, y, 宽, 高]，像素
    # 掩膜内深度采样
    erode_kernel_px: int  # 掩膜腐蚀核尺寸，像素，须为奇数
    min_valid_depth_px: int  # 掩膜内有效深度像素的最少数量
    min_depth_m: float  # 采样深度有效下界，米
    max_depth_m: float  # 采样深度有效上界，米
    max_joint_state_age_s: float  # 关节状态允许的最大时延，秒
    # 检测前自动下发的颈部俯仰角（度，负值 = 低头）；
    # runner 分有界小步转到该角度。
    head_pitch_deg: float
    # 桌面安全：唯一暴露的桌面高度字段（现场卷尺实测），
    # 加上"深度 vs 桌面"一致性窗口。
    table_height_m: float
    surface_consistency_window_m: float
    # 标定捏取姿态下指尖到 EEF 的垂直偏移
    # （工具几何；EEF 原点位于指尖上方这么远）。
    fingertip_offset_z_m: float
    # 标定的捏取姿态四元数（全身复位后腕部呈水平；
    # runner 分有界小步转到该姿态）。
    pinch_rotation_quat: tuple[float, float, float, float]
    # 标定的感知偏移（真值减感知点）。XY 修正抓取目标；
    # Z 只修正桌面一致性门所用的感知桌面高度。
    grasp_offset_m: tuple[float, float, float]
    # 可选的水平微调：空捏后唯一一次重试下探之前，在安全高度施加。
    retry_offset_m: tuple[float, float]
    # 运动
    envelope_x_m: tuple[float, float]  # 允许工作包络 X 范围 [min, max]，米
    envelope_y_m: tuple[float, float]  # 允许工作包络 Y 范围 [min, max]，米
    envelope_z_m: tuple[float, float]  # 允许工作包络 Z 范围 [min, max]，米
    step_max_m: float  # 单步位移上限，米
    descend_step_m: float  # 下探阶段单步步长，米
    overall_timeout_s: float  # 整个抓取流程的总超时，秒
    # 夹爪
    open_position: float  # 张开位置
    close_position: float  # 闭合目标位置
    cloth_feedback_center: float  # 捏住布料时的期望反馈中心值
    cloth_feedback_tolerance: float  # 布料反馈验证带半宽
    gripper_tolerance: float  # 夹爪常规到位容差

    # EEF 坐标下的推导安全高度（绝不是配置字段；指尖偏移把三者
    # 一起平移，保持相互几何关系不变）。
    @property
    def pinch_z_m(self) -> float:
        return self.table_height_m + PINCH_OFFSET_M + self.fingertip_offset_z_m

    @property
    def floor_z_m(self) -> float:
        return self.table_height_m - FLOOR_OFFSET_M + self.fingertip_offset_z_m

    @property
    def safe_z_m(self) -> float:
        return self.table_height_m + SAFE_CLEARANCE_M + self.fingertip_offset_z_m

    def to_record(self) -> dict[str, Any]:
        record: dict[str, Any] = {}
        for item in fields(self):
            value = getattr(self, item.name)
            record[item.name] = list(value) if isinstance(value, tuple) else value
        record["derived_pinch_z_m"] = self.pinch_z_m
        record["derived_floor_z_m"] = self.floor_z_m
        record["derived_safe_z_m"] = self.safe_z_m
        return record

    def contains(self, position: Sequence[float]) -> bool:
        """绝对位置 (x, y, z) 落在工作包络内时返回 True。"""

        x, y, z = (float(value) for value in position)
        return (
            self.envelope_x_m[0] <= x <= self.envelope_x_m[1]
            and self.envelope_y_m[0] <= y <= self.envelope_y_m[1]
            and self.envelope_z_m[0] <= z <= self.envelope_z_m[1]
        )


def parse_config(value: Any) -> ClothesGraspConfig:
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

    erode_kernel = _integer(
        _require(value, "erode_kernel_px"), "erode_kernel_px", minimum=1, maximum=MAX_ERODE_KERNEL_PX
    )
    if erode_kernel % 2 == 0:
        raise ConfigError(f"erode_kernel_px must be odd, received {erode_kernel}")
    min_depth = _number(
        _require(value, "min_depth_m"), "min_depth_m", minimum=DEPTH_MIN_FLOOR_M, maximum=DEPTH_MAX_CEILING_M
    )
    max_depth = _number(
        _require(value, "max_depth_m"), "max_depth_m", minimum=DEPTH_MIN_FLOOR_M, maximum=DEPTH_MAX_CEILING_M
    )
    if min_depth >= max_depth:
        raise ConfigError("min_depth_m must be below max_depth_m")

    table_height = _number(
        _require(value, "table_height_m"),
        "table_height_m",
        minimum=TABLE_HEIGHT_MIN_M,
        maximum=TABLE_HEIGHT_MAX_M,
    )

    fingertip_offset = _number(
        _require(value, "fingertip_offset_z_m"),
        "fingertip_offset_z_m",
        minimum=FINGERTIP_OFFSET_MIN_M,
        maximum=FINGERTIP_OFFSET_MAX_M,
    )

    quat = _require(value, "pinch_rotation_quat")
    if not isinstance(quat, Sequence) or isinstance(quat, (str, bytes, bytearray)) or len(quat) != 4:
        raise ConfigError("pinch_rotation_quat must be a sequence of four numbers")
    pinch_quat = tuple(
        _number(quat[index], f"pinch_rotation_quat[{index}]", minimum=-1.0, maximum=1.0)
        for index in range(4)
    )
    norm = math.sqrt(sum(component * component for component in pinch_quat))
    if abs(norm - 1.0) > QUAT_NORM_TOLERANCE:
        raise ConfigError(f"pinch_rotation_quat must be normalized (norm {norm:.4f} deviates beyond {QUAT_NORM_TOLERANCE})")

    offset = _require(value, "grasp_offset_m")
    if not isinstance(offset, Sequence) or isinstance(offset, (str, bytes, bytearray)) or len(offset) != 3:
        raise ConfigError("grasp_offset_m must be a [dx, dy, dz] triple")
    grasp_offset = (
        _number(offset[0], "grasp_offset_m[0]", minimum=-OFFSET_XY_MAX_M, maximum=OFFSET_XY_MAX_M),
        _number(offset[1], "grasp_offset_m[1]", minimum=-OFFSET_XY_MAX_M, maximum=OFFSET_XY_MAX_M),
        _number(offset[2], "grasp_offset_m[2]", minimum=-OFFSET_Z_MAX_M, maximum=OFFSET_Z_MAX_M),
    )

    retry = _require(value, "retry_offset_m")
    if not isinstance(retry, Sequence) or isinstance(retry, (str, bytes, bytearray)) or len(retry) != 2:
        raise ConfigError("retry_offset_m must be a [dx, dy] pair")
    retry_offset = (
        _number(retry[0], "retry_offset_m[0]", minimum=-RETRY_OFFSET_MAX_M, maximum=RETRY_OFFSET_MAX_M),
        _number(retry[1], "retry_offset_m[1]", minimum=-RETRY_OFFSET_MAX_M, maximum=RETRY_OFFSET_MAX_M),
    )

    envelope_x = _pair(_require(value, "envelope_x_m"), "envelope_x_m", minimum=-ENVELOPE_ABS_MAX_M, maximum=ENVELOPE_ABS_MAX_M)
    envelope_y = _pair(_require(value, "envelope_y_m"), "envelope_y_m", minimum=-ENVELOPE_ABS_MAX_M, maximum=ENVELOPE_ABS_MAX_M)
    envelope_z = _pair(_require(value, "envelope_z_m"), "envelope_z_m", minimum=-ENVELOPE_ABS_MAX_M, maximum=ENVELOPE_ABS_MAX_M)

    open_position = _number(
        _require(value, "open_position"), "open_position", minimum=GRIPPER_POSITION_MIN, maximum=GRIPPER_POSITION_MAX
    )
    close_position = _number(
        _require(value, "close_position"), "close_position", minimum=CLOSE_POSITION_MIN, maximum=GRIPPER_POSITION_MAX
    )
    cloth_center = _number(
        _require(value, "cloth_feedback_center"),
        "cloth_feedback_center",
        minimum=GRIPPER_POSITION_MIN,
        maximum=GRIPPER_POSITION_MAX,
    )
    cloth_tolerance = _number(
        _require(value, "cloth_feedback_tolerance"),
        "cloth_feedback_tolerance",
        minimum=0.1,
        maximum=CLOTH_TOLERANCE_MAX,
    )
    gripper_tolerance = _number(
        _require(value, "gripper_tolerance"), "gripper_tolerance", minimum=0.1, maximum=GRIPPER_TOLERANCE_MAX
    )
    # 张开状态的夹爪读数绝不能落进"捏住布料"验证带；空捏到全闭
    # 也绝不能被验证为捏取成功。
    if abs(cloth_center - open_position) <= cloth_tolerance + gripper_tolerance:
        raise ConfigError(
            "open_position must sit outside the cloth-holding band: "
            f"|{cloth_center} - {open_position}| <= {cloth_tolerance} + {gripper_tolerance}"
        )
    if abs(cloth_center - close_position) <= cloth_tolerance:
        raise ConfigError(
            "close_position must sit outside the cloth-holding band, otherwise an empty "
            f"pinch verifies as held: |{cloth_center} - {close_position}| <= {cloth_tolerance}"
        )

    config = ClothesGraspConfig(
        schema_version=version,
        robot_id=_text(_require(value, "robot_id"), "robot_id"),
        camera_id=_text(_require(value, "camera_id"), "camera_id"),
        target_id=_text(_require(value, "target_id"), "target_id"),
        workcell_id=_text(_require(value, "workcell_id"), "workcell_id"),
        hsv_lower=hsv_lower,
        hsv_upper=hsv_upper,
        min_area=_integer(_require(value, "min_area"), "min_area", minimum=1, maximum=307_200),
        detection_roi_px=detection_roi,
        erode_kernel_px=erode_kernel,
        min_valid_depth_px=_integer(
            _require(value, "min_valid_depth_px"), "min_valid_depth_px", minimum=1, maximum=307_200
        ),
        min_depth_m=min_depth,
        max_depth_m=max_depth,
        max_joint_state_age_s=_number(
            _require(value, "max_joint_state_age_s"),
            "max_joint_state_age_s",
            minimum=0.05,
            maximum=MAX_JOINT_STATE_AGE_CEILING_S,
        ),
        head_pitch_deg=_number(
            _require(value, "head_pitch_deg"),
            "head_pitch_deg",
            minimum=HEAD_PITCH_MIN_DEG,
            maximum=HEAD_PITCH_MAX_DEG,
        ),
        table_height_m=table_height,
        fingertip_offset_z_m=fingertip_offset,
        pinch_rotation_quat=pinch_quat,
        surface_consistency_window_m=_number(
            _require(value, "surface_consistency_window_m"),
            "surface_consistency_window_m",
            minimum=CONSISTENCY_WINDOW_MIN_M,
            maximum=CONSISTENCY_WINDOW_MAX_M,
        ),
        grasp_offset_m=grasp_offset,
        retry_offset_m=retry_offset,
        envelope_x_m=envelope_x,
        envelope_y_m=envelope_y,
        envelope_z_m=envelope_z,
        step_max_m=_number(_require(value, "step_max_m"), "step_max_m", minimum=0.005, maximum=STEP_MAX_M),
        descend_step_m=_number(
            _require(value, "descend_step_m"), "descend_step_m", minimum=0.005, maximum=DESCEND_STEP_MAX_M
        ),
        overall_timeout_s=_number(
            _require(value, "overall_timeout_s"), "overall_timeout_s", minimum=1.0, maximum=OVERALL_TIMEOUT_MAX_S
        ),
        open_position=open_position,
        close_position=close_position,
        cloth_feedback_center=cloth_center,
        cloth_feedback_tolerance=cloth_tolerance,
        gripper_tolerance=gripper_tolerance,
    )

    # 各推导工作高度必须全部落在包络内可达；在这里拒绝，
    # 远好过手臂已经伸出去才发现踩了下限。
    for label, height in (
        ("derived pinch height", config.pinch_z_m),
        ("derived safe height", config.safe_z_m),
    ):
        if not (envelope_z[0] <= height <= envelope_z[1]):
            raise ConfigError(
                f"{label} {height:.3f} m lies outside envelope_z_m [{envelope_z[0]}, {envelope_z[1]}]"
            )
    return config


def load_config(path: Any) -> ClothesGraspConfig:
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
    config: ClothesGraspConfig,
    *,
    camera_id: str,
    workcell_id: str,
) -> ClothesGraspConfig:
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
