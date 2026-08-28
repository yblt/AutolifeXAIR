#!/usr/bin/env python3
"""头部相机引导的叠好衣物抓取 runner（point4 衣物桌）。

流程（单次人工启动的任务，默认 preview）::

    PREFLIGHT -> HEAD_PITCH (有界、到位门控的颈部分步；失败绝不
    移动手臂) -> LOCATE (头部 RGBD：稳定的衣物检测、腐蚀掩膜内的
    深度中位数、CameraTransformer 基座坐标点加标定偏移)
    -> TABLE_GATE (感知的衣物表面高度必须与卷尺实测的桌面高度在
    配置窗口内一致) -> OPEN_GRIPPER -> POSTURE (有界 slerp 分步
    转到标定捏取姿态——整机复位后腕部呈水平；旋转在身体附近完成，
    先于任何越过桌面的运动) -> ALIGN (水平运动只允许在推导安全
    高度或其上方进行) -> DESCEND (纯垂直、有界小步、硬 Z 下限；
    张开的手指要压"进"衣物——2026-08-14 现场验证：轻触表面什么都
    捏不到) -> PINCH (闭合兜起布料，"立即"验证捏布反馈带——读数在
    负载下会向全闭漂移；允许一次重试：张爪、垂直上升、在安全高度
    水平微调、再下探、再闭合) -> LIFT (纯垂直回到安全高度，保持
    抓握) -> COMPLETE (持物待命，绝不自动释放)

桌面安全规则（spec `clothes-grasp`）：每个发布的目标都必须位于
推导硬下限（``table - 0.02 m``）或其上方；任何低于推导安全高度
（``table + 0.15 m``）的运动段必须是纯垂直的——轻质带垂布的桌子
绝不允许出现低位横扫。违规目标一律拒绝（绝不截断放行）并停止
本次运行。

手臂没有力/力矩反馈，因此接触从不靠力检测：捏取高度由标定桌面
高度推导，感知只提供水平抓取点。夹爪闭合之后不再咨询任何相机
（提起的衣物会垂盖住手部相机）。

状态机（`run_grasp`）是注入式 runtime 边界之上的纯逻辑，离线测试
因此永不 import ROS、SDK 或 OpenCV。具体的 `_RosRuntime` 继承
篮筐投放 runtime，沿用现场验证过的手臂原语和头部管线，并新增
衣物定位器（`detect_clothes` + 掩膜深度中位数 + 基座坐标投影）。
"""

from __future__ import annotations

import argparse
import importlib.util
import json
import math
import sys
import time
from collections.abc import Mapping, Sequence
from dataclasses import dataclass, field
from pathlib import Path
from types import SimpleNamespace
from typing import Any, Callable

try:  # 从项目根目录按包导入。
    from examples.arm.right.clothes_grasp_config import (
        ClothesGraspConfig,
        ConfigError,
        load_config,
        require_identity,
    )
except ImportError:  # 在 examples/arm 目录下直接执行。
    from clothes_grasp_config import (  # type: ignore[no-redef]
        ClothesGraspConfig,
        ConfigError,
        load_config,
        require_identity,
    )


_MODULE_CACHE: dict[str, Any] = {}


def _load_sibling(subdir: str, name: str) -> Any:
    """按文件路径加载项目模块，并保证其自身的 import 仍然可用。"""

    key = f"{subdir}/{name}"
    if key in _MODULE_CACHE:
        return _MODULE_CACHE[key]
    # 本文件位于 examples/arm/right/：parents[2] 为 examples/，parents[3] 为项目根。
    module_dir = Path(__file__).resolve().parents[2] / subdir
    for entry in (str(module_dir), str(Path(__file__).resolve().parents[3])):
        if entry not in sys.path:
            sys.path.insert(0, entry)
    module_path = module_dir / f"{name}.py"
    spec = importlib.util.spec_from_file_location(f"xr_clothes_grasp_{name}", module_path)
    if spec is None or spec.loader is None:  # pragma: no cover - repo layout invariant
        raise ImportError(f"could not load sibling module {name!r} from {module_path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    _MODULE_CACHE[key] = module
    return module


# 与已验收的篮筐投放 runner 共享的现场验证机制：手臂原语、头部管线
# runtime、有界颈部俯仰和路径规划。
_DROP = _load_sibling("arm/right", "run_basket_drop")

RunnerError = _DROP.RunnerError
FeedbackError = _DROP.FeedbackError
Pose = _DROP.Pose
EEFState = _DROP.EEFState
GripperState = _DROP.GripperState
parse_eef_state = _DROP.parse_eef_state
parse_gripper_state = _DROP.parse_gripper_state
plan_axis_steps = _DROP.plan_axis_steps
drop_interface_names = _DROP.drop_interface_names

BOUNDARY_TOLERANCE = 1e-6
# 该规则防的是"厘米级"横扫撞上桌沿或桌布；而真实双臂反馈会停在
# 距指令目标几毫米的位置，导致纯垂直步进相对反馈出现幻影横向分量
# （现场中止 20260814T065254Z：0.001 m）。
LOW_ZONE_LATERAL_TOLERANCE_M = 0.01
# 横向运动发生在安全高度"加"此余量处：若正好指令到边界，稳定后的
# 反馈会低于边界几毫米，导致之后每个横向步进都被拒绝（现场中止
# 20260814T065654Z）。边界本身仍是拒绝线。
ALIGN_CLEARANCE_M = 0.03
ROTATION_STEP_MAX_DEG = 20.0
ROTATION_DONE_TOLERANCE_DEG = 0.5

EXPECTED_CAMERA_ID = "mod_camera_rgbd_head"

STAGES = (
    "PREFLIGHT",
    "HEAD_PITCH",
    "LOCATE",
    "TABLE_GATE",
    "OPEN_GRIPPER",
    "POSTURE",
    "ALIGN",
    "DESCEND",
    "PINCH",
    "LIFT",
    "COMPLETE",
)


def quat_angle_deg(first: Sequence[float], second: Sequence[float]) -> float:
    """两个四元数之间的旋转角（度；与符号约定无关；输入先归一化，
    因此略偏离单位长度的反馈值也能干净比较）。"""

    q0 = tuple(float(value) for value in first)
    q1 = tuple(float(value) for value in second)
    norm0 = math.sqrt(sum(value * value for value in q0))
    norm1 = math.sqrt(sum(value * value for value in q1))
    if norm0 <= 0.0 or norm1 <= 0.0:
        raise RunnerError("quaternions must be non-zero to compare orientations")
    dot = abs(sum(a * b for a, b in zip(q0, q1))) / (norm0 * norm1)
    return math.degrees(2.0 * math.acos(min(1.0, dot)))


def quat_slerp(first: Sequence[float], second: Sequence[float], fraction: float) -> tuple[float, ...]:
    """单位四元数之间的球面插值，结果重新归一化。"""

    q0 = tuple(float(value) for value in first)
    q1 = tuple(float(value) for value in second)
    dot = sum(a * b for a, b in zip(q0, q1))
    if dot < 0.0:
        q1 = tuple(-value for value in q1)
        dot = -dot
    dot = min(1.0, dot)
    if dot > 0.9995:
        blended = tuple(a + fraction * (b - a) for a, b in zip(q0, q1))
    else:
        theta = math.acos(dot)
        s0 = math.sin((1.0 - fraction) * theta) / math.sin(theta)
        s1 = math.sin(fraction * theta) / math.sin(theta)
        blended = tuple(s0 * a + s1 * b for a, b in zip(q0, q1))
    norm = math.sqrt(sum(value * value for value in blended))
    if norm <= 0.0:  # pragma: no cover - degenerate inputs rejected upstream
        raise RunnerError("quaternion interpolation degenerated to zero norm")
    return tuple(value / norm for value in blended)


def check_table_rules(
    previous: Sequence[float], waypoint: Sequence[float], config: ClothesGraspConfig
) -> str | None:
    """运动段违反推导桌面安全规则时返回拒绝原因，否则返回
    ``None``。目标一律拒绝，绝不截断放行。"""

    px, py, pz = (float(value) for value in previous)
    x, y, z = (float(value) for value in waypoint)
    if z < config.floor_z_m - BOUNDARY_TOLERANCE:
        return (
            f"target z {z:.3f} m lies below the derived hard floor {config.floor_z_m:.3f} m "
            f"(table {config.table_height_m:.3f} m)"
        )
    horizontal = math.hypot(x - px, y - py)
    if horizontal > LOW_ZONE_LATERAL_TOLERANCE_M and min(pz, z) < config.safe_z_m - BOUNDARY_TOLERANCE:
        return (
            f"lateral motion of {horizontal:.3f} m below the derived safe height "
            f"{config.safe_z_m:.3f} m is forbidden (pure vertical only)"
        )
    return None


def _with_right_position(state: EEFState, position: Sequence[float]) -> EEFState:
    right = state.right
    return EEFState(state.left, Pose(tuple(float(value) for value in position), right.rotation))


@dataclass
class GraspRunResult:
    status: str = "aborted"
    last_stage: str = "PREFLIGHT"
    reason: str = ""
    stage_history: list[dict[str, Any]] = field(default_factory=list)
    measurements: list[dict[str, Any]] = field(default_factory=list)
    issued_targets: int = 0

    @property
    def complete(self) -> bool:
        return self.status == "complete"

    def to_record(self) -> dict[str, Any]:
        return {
            "status": self.status,
            "last_stage": self.last_stage,
            "reason": self.reason,
            "stage_history": list(self.stage_history),
            "measurements": list(self.measurements),
            "issued_targets": self.issued_targets,
        }


def run_grasp(
    config: ClothesGraspConfig,
    runtime: Any,
    *,
    clock: Callable[[], float] = time.monotonic,
) -> GraspRunResult:
    """经 runtime 边界执行一次有界抓取，fail-closed。

    runtime 必须提供：``read_eef_state()``、``read_neck_state()``、
    ``publish_neck_target((roll, pitch, yaw))``、
    ``wait_for_neck_settle(pitch)``、``publish_eef_target``、
    ``wait_for_eef_feedback``、``publish_gripper_target``、
    ``wait_for_gripper_feedback(target, tolerance)`` 和
    ``locate_clothes(config)``；可选的 ``save_frame(tag, frame)`` 用于
    保存证据图像。夹爪一旦捏住布料就绝不自动张开；
    中止只停止发布后续目标。
    """

    result = GraspRunResult()

    def record_stage(name: str, detail: Mapping[str, Any] | None = None) -> None:
        result.last_stage = name
        result.stage_history.append({"stage": name, "at": clock(), **(dict(detail) if detail else {})})

    def abort(reason: str) -> GraspRunResult:
        result.status = "aborted"
        result.reason = reason
        return result

    def save_frame(tag: str, frame: Any) -> None:
        saver = getattr(runtime, "save_frame", None)
        if callable(saver) and frame is not None:
            try:
                saver(tag, frame)
            except Exception:
                pass  # 证据丢失绝不允许改变运动行为

    try:
        record_stage("PREFLIGHT")
        state = parse_eef_state(runtime.read_eef_state())
        if not config.contains(state.right.position):
            return abort(
                f"preflight right EEF position {list(state.right.position)} is outside the "
                "configured workspace envelope"
            )

        deadline = clock() + config.overall_timeout_s

        def check_deadline(context: str) -> None:
            if clock() >= deadline:
                raise RunnerError(
                    f"overall timeout of {config.overall_timeout_s} s reached during {context}; "
                    "stopping further target publication"
                )

        def move_and_wait(target: EEFState) -> EEFState:
            runtime.publish_eef_target(target)
            result.issued_targets += 1
            return parse_eef_state(runtime.wait_for_eef_feedback(target, loaded=True))

        def execute_path(goal: Sequence[float], step: float, context: str) -> None:
            nonlocal state
            for waypoint in plan_axis_steps(state.right.position, goal, step):
                check_deadline(context)
                if not config.contains(waypoint):
                    raise RunnerError(f"planned waypoint {list(waypoint)} left the workspace envelope")
                violation = check_table_rules(state.right.position, waypoint, config)
                if violation is not None:
                    raise RunnerError(f"table-safety refusal during {context}: {violation}")
                state = move_and_wait(_with_right_position(state, waypoint))

        record_stage("HEAD_PITCH", {"target_deg": config.head_pitch_deg})
        neck = runtime.read_neck_state()
        try:
            roll, pitch, yaw = (float(value) for value in neck)
        except (TypeError, ValueError) as exc:
            return abort(f"neck state is unreadable: {exc}; no motion targets published")
        if not all(math.isfinite(value) for value in (roll, pitch, yaw)):
            return abort("neck state is not finite; no motion targets published")
        pitch_steps = 0
        while abs(config.head_pitch_deg - pitch) > _DROP.HEAD_PITCH_SETTLE_TOLERANCE_DEG:
            check_deadline("the head pitch")
            pitch_steps += 1
            if pitch_steps > _DROP.HEAD_PITCH_MAX_STEPS:
                return abort(
                    f"neck pitch did not reach {config.head_pitch_deg} deg within "
                    f"{_DROP.HEAD_PITCH_MAX_STEPS} bounded steps (at {pitch:.2f} deg); "
                    "stopping before any arm target"
                )
            delta = config.head_pitch_deg - pitch
            step = max(-_DROP.HEAD_PITCH_STEP_MAX_DEG, min(_DROP.HEAD_PITCH_STEP_MAX_DEG, delta))
            step_target = pitch + step
            runtime.publish_neck_target((roll, step_target, yaw))
            result.issued_targets += 1
            pitch = float(runtime.wait_for_neck_settle(step_target))
        result.stage_history[-1]["settled_pitch_deg"] = pitch

        # 感知必须在手臂升入头部相机视野之前完成；此后的流程不再
        # 咨询任何相机。
        record_stage("LOCATE")
        located = runtime.locate_clothes(config)
        result.measurements.append(
            {
                "phase": "locate",
                "at": clock(),
                "located": bool(getattr(located, "located", False)),
                "point_base": (
                    list(located.point_base) if getattr(located, "point_base", None) is not None else None
                ),
                "reason": getattr(located, "reason", ""),
                "record": getattr(located, "record", None),
            }
        )
        if not getattr(located, "located", False):
            return abort(
                f"clothes localization failed: {getattr(located, 'reason', 'unknown')}; "
                "no motion targets published"
            )
        save_frame("locate", getattr(located, "frame", None))
        perceived_x, perceived_y, perceived_z = (float(value) for value in located.point_base)
        target_x = perceived_x + config.grasp_offset_m[0]
        target_y = perceived_y + config.grasp_offset_m[1]
        surface_z = perceived_z + config.grasp_offset_m[2]

        record_stage(
            "TABLE_GATE",
            {"surface_z_m": surface_z, "table_height_m": config.table_height_m},
        )
        surface_error = abs(surface_z - config.table_height_m)
        result.stage_history[-1]["surface_error_m"] = surface_error
        if surface_error > config.surface_consistency_window_m + BOUNDARY_TOLERANCE:
            return abort(
                f"perceived garment surface height {surface_z:.3f} m disagrees with the measured "
                f"table height {config.table_height_m:.3f} m by {surface_error:.3f} m, beyond the "
                f"{config.surface_consistency_window_m} m window; no motion targets published"
            )

        pinch = (target_x, target_y, config.pinch_z_m)
        align_z = config.safe_z_m + ALIGN_CLEARANCE_M
        hover = (target_x, target_y, align_z)
        for label, point in (("pinch target", pinch), ("safe-height target", hover)):
            if not config.contains(point):
                return abort(
                    f"{label} {list(point)} is outside the configured workspace envelope; "
                    "no motion targets published"
                )

        record_stage("OPEN_GRIPPER", {"open_position": config.open_position})
        check_deadline("the gripper opening")
        runtime.publish_gripper_target(config.open_position)
        result.issued_targets += 1
        parse_gripper_state(
            runtime.wait_for_gripper_feedback(config.open_position, config.gripper_tolerance)
        )

        # 在"身体附近"转到标定捏取姿态，先于任何把手带到桌面上方的
        # 运动（复位后腕部呈水平；绕基座 Y 正向旋转会让手低头——
        # 2026-08-14 现场验证的符号约定）。
        record_stage("POSTURE", {"target_quat": list(config.pinch_rotation_quat)})
        current_quat = tuple(float(value) for value in state.right.rotation)
        posture_angle = quat_angle_deg(current_quat, config.pinch_rotation_quat)
        result.stage_history[-1]["angle_deg"] = posture_angle
        if posture_angle > ROTATION_DONE_TOLERANCE_DEG:
            rotation_steps = max(1, math.ceil(posture_angle / ROTATION_STEP_MAX_DEG))
            for index in range(1, rotation_steps + 1):
                check_deadline("the pinch posture rotation")
                stepped = quat_slerp(current_quat, config.pinch_rotation_quat, index / rotation_steps)
                state = move_and_wait(
                    EEFState(state.left, Pose(state.right.position, stepped))
                )
            result.stage_history[-1]["rotation_steps"] = rotation_steps

        record_stage("ALIGN", {"hover": list(hover), "pinch": list(pinch)})
        execute_path(hover, config.step_max_m, "the alignment at safe height")

        grasped = False
        for attempt in (0, 1):
            record_stage("DESCEND", {"attempt": attempt, "pinch_z_m": config.pinch_z_m})
            execute_path((state.right.position[0], state.right.position[1], config.pinch_z_m),
                         config.descend_step_m, "the vertical descent")

            record_stage("PINCH", {"attempt": attempt, "close_position": config.close_position})
            check_deadline("the pinch")
            runtime.publish_gripper_target(config.close_position)
            result.issued_targets += 1
            try:
                grip = parse_gripper_state(
                    runtime.wait_for_gripper_feedback(
                        config.cloth_feedback_center, config.cloth_feedback_tolerance
                    )
                )
            except FeedbackError as exc:
                result.stage_history[-1]["empty_pinch"] = str(exc)
                if attempt == 1:
                    return abort(
                        "pinch verification failed after the single retry: gripper feedback never "
                        f"entered the cloth-holding band ({exc}); stopping, no automatic remedies"
                    )
                # 唯一一次重试：重新张爪、纯垂直上升、在安全高度
                # 水平微调，然后再次下探。
                check_deadline("the retry reopening")
                runtime.publish_gripper_target(config.open_position)
                result.issued_targets += 1
                parse_gripper_state(
                    runtime.wait_for_gripper_feedback(config.open_position, config.gripper_tolerance)
                )
                execute_path(
                    (state.right.position[0], state.right.position[1], align_z),
                    config.descend_step_m,
                    "the retry ascent",
                )
                retry_point = (
                    state.right.position[0] + config.retry_offset_m[0],
                    state.right.position[1] + config.retry_offset_m[1],
                    align_z,
                )
                if not config.contains(retry_point):
                    return abort(
                        f"retry target {list(retry_point)} is outside the configured workspace envelope"
                    )
                execute_path(retry_point, config.step_max_m, "the retry alignment")
                continue
            result.stage_history[-1]["gripper_position"] = grip.position
            grasped = True
            break
        if not grasped:  # pragma: no cover - loop invariant
            return abort("pinch loop ended without a verified grasp")

        record_stage("LIFT", {"lift_z_m": align_z})
        execute_path(
            (state.right.position[0], state.right.position[1], align_z),
            config.descend_step_m,
            "the lift",
        )

        record_stage("COMPLETE")
        result.status = "complete"
        result.reason = (
            "grasp complete; garment pinched, verified in the cloth-holding band, and lifted to "
            "the safe height (grip kept closed)"
        )
        return result
    except RunnerError as exc:
        return abort(str(exc))
    except Exception as exc:  # 防御边界：遇到意外错误绝不继续运动。
        return abort(f"unexpected runner failure: {exc}")


# ---------------------------------------------------------------------------
# 具体的 ROS runtime（只在真正 `run` 时构造）。
# ---------------------------------------------------------------------------


class _RosRuntime(_DROP._RosRuntime):
    """篮筐投放 runtime（手臂原语、颈部、头部管线）外加衣物定位器：
    稳定 HSV 检测、腐蚀掩膜内深度中位数，以及经官方相机变换链
    投影到基座坐标。"""

    def locate_clothes(self, config: ClothesGraspConfig) -> Any:
        def fail(reason: str, *, frame: Any = None, record: Any = None) -> SimpleNamespace:
            return SimpleNamespace(located=False, point_base=None, reason=reason, frame=frame, record=record)

        try:
            self._ensure_head_pipeline()
            probe, geometry = self._probe, self._geometry
            clothes = _load_sibling("camera", "detect_color_targets")

            deadline = time.monotonic() + _DROP.LOCATE_TRANSFORM_TIMEOUT_S
            transform, joint_age = probe._wait_for_fresh_transform(
                self._rclpy,
                self._node,
                self._transformer,
                self._joint_arrival,
                self._identity,
                config.max_joint_state_age_s,
                deadline,
                self.names["gripper_state_topic"],
            )

            detector_config = clothes.clothes_detector_config(
                hsv_lower=config.hsv_lower,
                hsv_upper=config.hsv_upper,
                min_area=config.min_area,
                roi=config.detection_roi_px,
            )
            acquisition = clothes.acquire_stable_clothes(
                self._pair_reader, detector_config, clothes.stable_config()
            )
            if not getattr(acquisition, "stable", False):
                return fail(f"stable clothes detection failed: {getattr(acquisition, 'reason', 'unknown')}")

            candidate = acquisition.candidate
            frame = acquisition.frame
            try:
                sample = clothes.masked_depth_median(
                    frame,
                    candidate,
                    hsv_lower=config.hsv_lower,
                    hsv_upper=config.hsv_upper,
                    erode_kernel_px=config.erode_kernel_px,
                    min_valid_px=config.min_valid_depth_px,
                )
            except ValueError as exc:
                return fail(f"masked depth sampling failed: {exc}", frame=frame)

            fx = probe._field(self._intrinsics_raw, "fx")
            fy = probe._field(self._intrinsics_raw, "fy")
            ppx = probe._field(self._intrinsics_raw, "ppx")
            ppy = probe._field(self._intrinsics_raw, "ppy")
            if fx is None or fy is None or ppx is None or ppy is None:
                return fail("head-camera intrinsics are missing one of fx/fy/ppx/ppy", frame=frame)
            shape = frame.image.shape
            intrinsics = geometry.CameraIntrinsics(
                fx=fx, fy=fy, cx=ppx, cy=ppy, width=int(shape[1]), height=int(shape[0])
            )
            limits = geometry.DepthLimits(min_depth_m=config.min_depth_m, max_depth_m=config.max_depth_m)
            position = geometry.locate_bottle_in_base(
                pixel=sample.pixel,
                depth_raw=sample.depth_raw,
                intrinsics=intrinsics,
                transform=transform,
                limits=limits,
                joint_state_age_s=joint_age,
                max_joint_state_age_s=config.max_joint_state_age_s,
            )
            record = {
                "detection": {
                    "bbox": list(candidate.bbox),
                    "center": list(candidate.center),
                    "area": candidate.area,
                    "frame_id": acquisition.frame_id,
                },
                "reference_pixel": list(sample.pixel),
                "depth_raw_median": sample.depth_raw,
                "valid_depth_px": sample.valid_px,
                "position": position.to_record(),
                "joint_state_age_s": joint_age,
            }
            if not position.located:
                return fail(f"base-frame localization failed: {position.reason}", frame=frame, record=record)
            point = position.point_base
            return SimpleNamespace(
                located=True,
                point_base=(point.x, point.y, point.z),
                reason="ok",
                frame=frame,
                record=record,
            )
        except Exception as exc:
            return fail(f"clothes localization errored: {exc}")


# ---------------------------------------------------------------------------
# CLI：默认 preview；单次抓取用 `run --execute --reset-confirmed`。
# ---------------------------------------------------------------------------


def _json_text(value: Any) -> str:
    return json.dumps(value, separators=(",", ":"), ensure_ascii=True, allow_nan=False)


def _preview_text(config_path: str) -> str:
    lines = [
        "Head-camera clothes-grasp preview",
        "  mode: preview",
        "  planned_stages: " + " -> ".join(STAGES),
        (
            "  limits: horizontal motion only at-or-above the derived safe height; descent is "
            "pure vertical in bounded small steps; any target below the derived hard floor is "
            "refused (never clamped); no force-based contact detection (position-controlled arm)"
        ),
        "  end_state: gripper closed on the garment, arm at safe height, never auto-releasing",
        "  abort_semantics: an abort stops publishing further targets only; e-stop is the field guard",
        f"  config: {config_path}",
    ]
    try:
        config = load_config(config_path)
    except ConfigError as exc:
        lines.append(f"  config_status: UNAVAILABLE ({exc})")
        lines.append("  derived_heights: unavailable until the configuration validates")
    else:
        lines.append("  config_status: valid")
        lines.append(
            "  derived_heights: "
            f"table={config.table_height_m:.3f} m, pinch_z={config.pinch_z_m:.3f} m, "
            f"floor_z={config.floor_z_m:.3f} m, safe_z={config.safe_z_m:.3f} m"
        )
        lines.append(
            "  detection: "
            f"hsv={list(config.hsv_lower)}..{list(config.hsv_upper)}, "
            f"roi={list(config.detection_roi_px)}, head_pitch={config.head_pitch_deg} deg"
        )
        lines.append(
            "  gripper: "
            f"open={config.open_position}, close={config.close_position}, "
            f"cloth_band={config.cloth_feedback_center}+/-{config.cloth_feedback_tolerance}"
        )
    lines += [
        "  topics: " + _json_text(drop_interface_names()),
        "  gates: --execute and --reset-confirmed absent",
        "  no action: no EEF publication and no gripper publication",
    ]
    return "\n".join(lines)


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Head-camera-guided folded-clothes grasp.")
    parser.add_argument("mode", nargs="?", default="preview", choices=("preview", "run"))
    parser.add_argument("--execute", action="store_true", help="confirm physical execution")
    parser.add_argument(
        "--reset-confirmed",
        action="store_true",
        help="confirm the whole-robot reset was completed on site",
    )
    parser.add_argument("--config", default="right/clothes_grasp.json", help="calibration configuration path")
    parser.add_argument("--workcell-id", default="s2-test-point4-clothes-table")
    parser.add_argument("--evidence-dir", default=None)
    args = parser.parse_args(argv)

    if args.mode == "preview":
        print(_preview_text(args.config))
        return 0
    if not args.execute or not args.reset_confirmed:
        print(
            "Error: physical execution requires both --execute and --reset-confirmed after the "
            "whole-robot reset, on-site monitoring, and physical emergency-stop readiness are "
            "confirmed.",
            file=sys.stderr,
        )
        return 2

    try:
        config = load_config(args.config)
        require_identity(
            config,
            camera_id=EXPECTED_CAMERA_ID,
            workcell_id=args.workcell_id,
        )
    except ConfigError as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 3

    evidence_dir = args.evidence_dir
    if evidence_dir is None:
        import datetime

        stamp = datetime.datetime.now(datetime.timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        evidence_dir = Path.home() / "Documents" / "AutolifeXAIR" / "evidence" / "clothes_grasp" / stamp

    runtime = None
    try:
        runtime = _RosRuntime(drop_interface_names(), evidence_dir)
        runtime.preflight_interfaces()
        result = run_grasp(config, runtime)
    except RunnerError as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 4
    finally:
        if runtime is not None:
            runtime.close()

    record = {"config": config.to_record(), "result": result.to_record()}
    record_path = Path(evidence_dir) / "run_record.json"
    record_path.parent.mkdir(parents=True, exist_ok=True)
    record_path.write_text(json.dumps(record, ensure_ascii=False, indent=1), encoding="utf-8")
    print(json.dumps(result.to_record(), ensure_ascii=False, indent=1))
    print(f"evidence: {record_path}")
    return 0 if result.complete else 5


if __name__ == "__main__":
    raise SystemExit(main())
