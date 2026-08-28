#!/usr/bin/env python3
"""精简的右手瓶子视觉伺服抓取 runner。

流程（单次人工启动的任务，默认 preview）::

    PREFLIGHT -> OPEN_GRIPPER(10) -> BACK_CLEARANCE(-0.10 m)
    -> SET_GRASP_HEIGHT(标定固定 Z，此后不再发 Z 目标)
    -> SERVO (Y 对中 + 按宽度比的 X 逼近，分级步长)
    -> [可选 FINAL_FORWARD] -> CLOSE_GRIP -> VERIFY_GRIP
    -> LIFT_HOLD(+0.10 m) -> COMPLETE (保持闭爪，不释放)

安全底线：实机执行需要 ``run --execute --reset-confirmed``；每个手臂
阶段发布一个完整双臂绝对目标，并等到新鲜的双臂反馈后才进行下一步；
所有步长与包络都由已验证的配置（`bottle_grasp_config`）封顶；任何
视觉或反馈失败都 fail-closed 中止，不重试、不重新张爪、不释放。
软件中止只能停止发布后续目标——它无法取消控制器已接受的在途目标；
物理急停才是现场保障。

状态机（`run_grasp`）是注入式 runtime 边界之上的纯逻辑，离线测试
因此永不 import ROS、SDK 或 OpenCV。具体的 `_RosRuntime` 沿用已部署
示例现场验证过的原生接口约定（相同的话题、载荷与反馈门）。

速览（本文件分三层，自上而下）：

1. 纯逻辑层 ``run_grasp``：抓取状态机本体，不 import ROS/SDK/OpenCV，
   全部硬件操作经注入的 runtime 完成，因此离线单测可覆盖全部分支。
2. 适配层 ``_RosRuntime``：把抽象动作落到厂商原生话题（完整双臂绝对
   位姿 JSON + 夹爪位置目标），并实现"发布后等反馈收敛"的门。
3. CLI 层 ``main``：默认 preview 零动作；实体执行必须
   ``run --execute --reset-confirmed`` 双开关，并通过配置身份校验。

安全设计要点：每步动作先验预算再发布；所有失败路径统一走 abort
（fail-closed），不自动回退、不自动松爪、不自动重试。
"""

from __future__ import annotations

import argparse
import json
import math
import numbers
import os
import sys
import time
from collections.abc import Mapping, Sequence
from dataclasses import dataclass, field
from pathlib import Path as _Path
from typing import Any, Callable

# 裸跑（python examples/arm/run_bottle_grasp.py ...）时 sys.path 只有
# examples/arm；SERVO 阶段 import 检测器就会死在
# "No module named 'examples'"。这里插入项目根目录，让裸命令无需
# 导出 PYTHONPATH 也能工作。
_HERE = _Path(__file__).resolve().parent
_ROOT = _HERE.parent.parent.parent if _HERE.name == "right" else _HERE.parent
for _path in (str(_ROOT), str(_HERE)):
    if _path not in sys.path:
        sys.path.insert(0, _path)

try:  # 从项目根目录按包导入。
    from examples.arm.right.bottle_grasp_config import (
        BACK_CLEARANCE_M,
        GRASP_Z_RELATIVE_FLOOR_M,
        GRIPPER_OPEN_POSITION,
        LIFT_M,
        MIDLINE_Y_PER_FORWARD_RATIO,
        X_ABSOLUTE_MAX_M,
        X_TOTAL_FORWARD_MAX_M,
        Y_CUMULATIVE_MAX_M,
        ConfigError,
        BottleGraspConfig,
        load_config,
        require_identity,
    )
except ImportError:  # 在 examples/arm 目录下直接执行。
    from bottle_grasp_config import (  # type: ignore[no-redef]
        BACK_CLEARANCE_M,
        GRASP_Z_RELATIVE_FLOOR_M,
        GRIPPER_OPEN_POSITION,
        LIFT_M,
        MIDLINE_Y_PER_FORWARD_RATIO,
        X_ABSOLUTE_MAX_M,
        X_TOTAL_FORWARD_MAX_M,
        Y_CUMULATIVE_MAX_M,
        ConfigError,
        BottleGraspConfig,
        load_config,
        require_identity,
    )


# 空载反馈收敛门：每个手臂目标发布后，左右臂实际位姿都必须进入该误差带
# 才算"到位"，下一步才允许发布；超时则中止整个任务。
DEFAULT_FEEDBACK_TIMEOUT_SECONDS = 5.0
POSITION_FEEDBACK_MAX_M = 0.003  # 位置误差 <= 3 mm
ROTATION_FEEDBACK_MAX_DEG = 1.0  # 旋转误差 <= 1°
# 2026-08-13 现场验证：持瓶后手臂下垂、腕部微扭几毫米/几度是实测现象，
# 因此负载阶段（抬升）使用放宽的收敛门；空载阶段保持严格门。
# （抬升阶段若沿用严格门会永远无法收敛，故负载阶段单独放宽到
# 0.02 m / 3° / 10 s。）
LOADED_POSITION_FEEDBACK_MAX_M = 0.02
LOADED_ROTATION_FEEDBACK_MAX_DEG = 3.0
LOADED_FEEDBACK_TIMEOUT_SECONDS = 10.0
BOUNDARY_TOLERANCE = 1e-9

DEFAULT_DOMAIN_ID = "0"
# 整机可能更换：机器编号从本机主机名推导。
DEFAULT_ROBOT_SUFFIX = os.uname().nodename.rsplit("-", 1)[-1]
EXPECTED_CAMERA_ID = "mod_camera_hand_right"

STAGES = (
    "PREFLIGHT",        # 预检：读实时状态，右臂 X 必须在批准区间内
    "OPEN_GRIPPER",     # 张爪到 10 并等反馈确认
    "BACK_CLEARANCE",   # 后退 0.10 m，实际到达位置即"观察锚点"
    "SET_GRASP_HEIGHT", # 一次性设定抓取高度，此后到抬升前不再动 Z
    "SERVO",            # 视觉伺服：Y 对中 + X 分级步进逼近
    "FINAL_FORWARD",    # 盲进最后一段（瓶子已近到视觉不可靠）
    "CLOSE_GRIP",       # 发布闭爪目标
    "VERIFY_GRIP",      # 用堵转反馈带验证真的夹住了（抓空必失败）
    "LIFT_HOLD",        # 持物抬升 0.10 m（放宽反馈门）
    "COMPLETE",         # 完成：保持闭爪持物，绝不自动松爪
)


class RunnerError(RuntimeError):
    """导致任务停止的运行时、配置或接口异常。"""


class FeedbackError(RunnerError):
    """新鲜反馈未能满足目标收敛门。"""


def _finite(value: Any, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, numbers.Real):
        raise RunnerError(f"{label} must be a number, received {value!r}")
    result = float(value)
    if not math.isfinite(result):
        raise RunnerError(f"{label} must be finite, received {value!r}")
    return result


@dataclass(frozen=True)
class Pose:
    position: tuple[float, float, float]
    rotation: tuple[float, float, float, float]

    def __post_init__(self) -> None:
        if len(self.position) != 3 or len(self.rotation) != 4:
            raise RunnerError("pose requires three position and four rotation values")
        position = tuple(_finite(value, "pose.position") for value in self.position)
        rotation = tuple(_finite(value, "pose.rotation") for value in self.rotation)
        norm = math.sqrt(sum(value * value for value in rotation))
        if math.isclose(norm, 0.0):
            raise RunnerError("pose rotation must not be the zero quaternion")
        object.__setattr__(self, "position", position)
        object.__setattr__(self, "rotation", tuple(value / norm for value in rotation))


@dataclass(frozen=True)
class EEFState:
    left: Pose
    right: Pose


@dataclass(frozen=True)
class GripperState:
    position: float
    communication_lost: bool


def _value(mapping: Any, *names: str) -> Any:
    for name in names:
        if isinstance(mapping, Mapping) and name in mapping and mapping[name] is not None:
            return mapping[name]
        item = getattr(mapping, name, None)
        if item is not None:
            return item
    return None


def _pose(value: Any, label: str) -> Pose:
    position = _value(value, "position")
    rotation = _value(value, "rotation")
    if position is None or rotation is None:
        raise RunnerError(f"{label} pose must contain position and rotation")
    return Pose(tuple(position), tuple(rotation))


def parse_eef_state(value: Any) -> EEFState:
    """把已确认格式的双臂状态 JSON 解析为已验证的状态对象。"""

    if isinstance(value, EEFState):
        return value
    left = _value(value, "left", "left_eef_pose")
    right = _value(value, "right", "right_eef_pose")
    if left is None and _value(value, "pos_left_in_robot") is not None:
        left = {"position": _value(value, "pos_left_in_robot"), "rotation": _value(value, "quat_left_in_robot")}
    if right is None and _value(value, "pos_right_in_robot") is not None:
        right = {"position": _value(value, "pos_right_in_robot"), "rotation": _value(value, "quat_right_in_robot")}
    if left is None or right is None:
        raise RunnerError("EEF feedback must contain complete left and right poses")
    return EEFState(_pose(left, "left"), _pose(right, "right"))


def parse_gripper_state(value: Any) -> GripperState:
    """解析右爪反馈，包括其通信状态标志。"""

    if isinstance(value, GripperState):
        return value
    root = _value(value, "right_gripper_state")
    if root is None:
        root = value
    position = _value(root, "position", "positions")
    if isinstance(position, Sequence) and not isinstance(position, (str, bytes, bytearray)):
        if len(position) != 1:
            raise RunnerError("right_gripper_state.position must contain one value")
        position = position[0]
    if position is None:
        raise RunnerError("gripper feedback is missing right_gripper_state.position")
    lost = _value(root, "communication_lost")
    if isinstance(lost, Sequence) and not isinstance(lost, (str, bytes, bytearray)):
        if not lost or any(not isinstance(item, bool) for item in lost):
            raise RunnerError("communication_lost must be a non-empty boolean array")
        lost = any(lost)
    if not isinstance(lost, bool):
        raise RunnerError("communication_lost must be boolean or a boolean array")
    return GripperState(_finite(position, "gripper position"), lost)


def state_payload(state: EEFState) -> dict[str, list[float]]:
    """已确认的运动话题所要求的完整双臂载荷。"""

    return {
        "pos_left_in_robot": list(state.left.position),
        "quat_left_in_robot": list(state.left.rotation),
        "pos_right_in_robot": list(state.right.position),
        "quat_right_in_robot": list(state.right.rotation),
    }


def move_right(state: EEFState, *, dx: float = 0.0, dy: float = 0.0, dz: float = 0.0) -> EEFState:
    right = state.right
    return EEFState(
        state.left,
        Pose(
            (right.position[0] + dx, right.position[1] + dy, right.position[2] + dz),
            right.rotation,
        ),
    )


def set_right_z(state: EEFState, z_value: float) -> EEFState:
    right = state.right
    return EEFState(state.left, Pose((right.position[0], right.position[1], z_value), right.rotation))


def _position_error(first: Pose, second: Pose) -> float:
    return math.dist(first.position, second.position)


def _rotation_error_degrees(first: Pose, second: Pose) -> float:
    dot = min(1.0, max(0.0, abs(sum(a * b for a, b in zip(first.rotation, second.rotation)))))
    return math.degrees(2.0 * math.acos(dot))


# 双臂反馈门：左右两侧的位置/旋转误差必须同时进入容差带。未选侧（左臂）
# 也受门约束，可以及早发现"只想动右臂却把左臂带动了"的异常。
def eef_target_reached(
    actual: EEFState,
    target: EEFState,
    *,
    position_tolerance_m: float = POSITION_FEEDBACK_MAX_M,
    rotation_tolerance_deg: float = ROTATION_FEEDBACK_MAX_DEG,
) -> bool:
    return (
        _position_error(actual.right, target.right) <= position_tolerance_m + BOUNDARY_TOLERANCE
        and _rotation_error_degrees(actual.right, target.right) <= rotation_tolerance_deg + BOUNDARY_TOLERANCE
        and _position_error(actual.left, target.left) <= position_tolerance_m + BOUNDARY_TOLERANCE
        and _rotation_error_degrees(actual.left, target.left) <= rotation_tolerance_deg + BOUNDARY_TOLERANCE
    )


def gripper_target_reached(actual: GripperState, target: float, tolerance: float) -> bool:
    return not actual.communication_lost and abs(actual.position - target) <= tolerance + BOUNDARY_TOLERANCE


# 厂商原生接口话题名（带 <domain>_<robot> 后缀，如 0_<机器编号>）。注意：
# eef_target_topic 无服务端限幅，payload 必须是完整双臂绝对位姿；漏字段
# 或未补齐另一侧会导致双臂同时运动。所有限幅都在本脚本侧完成。
def interface_names(domain_id: str = DEFAULT_DOMAIN_ID, robot_suffix: str = DEFAULT_ROBOT_SUFFIX) -> dict[str, str]:
    suffix = f"{str(domain_id).strip()}_{str(robot_suffix).strip()}"
    return {
        "eef_state_topic": f"/topic_arm_current_robot_eef_pose_{suffix}",
        "eef_target_topic": f"/topic_arm_move_eef_pose_in_robot_frame_{suffix}",
        "gripper_target_topic": f"/topic_arm_gripper_target_joints_position_{suffix}",
        "gripper_state_topic": f"/topic_arm_whole_body_and_gripper_current_joints_status_{suffix}",
        "right_mode_service": f"/node_mod_motor_right_arm_control_{suffix}/set_parameters",
    }


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
    config: BottleGraspConfig,
    runtime: Any,
    *,
    clock: Callable[[], float] = time.monotonic,
) -> GraspRunResult:
    """经 runtime 边界执行一次有界抓取，fail-closed。

    runtime 必须提供：``read_eef_state()``、``publish_eef_target``、
    ``wait_for_eef_feedback``、``publish_gripper_target``、
    ``wait_for_gripper_feedback(target, tolerance)`` 和
    ``acquire_stable(config)``；可选的 ``save_frame(tag, frame)`` 用于
    保存证据图像。任何代码路径都不会重新张爪或释放。
    """

    result = GraspRunResult()

    # 记录阶段轨迹（连同测量值一起写入 run_record.json 证据文件）。
    def record_stage(name: str, detail: Mapping[str, Any] | None = None) -> None:
        result.last_stage = name
        result.stage_history.append({"stage": name, "at": clock(), **(dict(detail) if detail else {})})

    # 唯一失败出口：置 aborted 并带原因返回（fail-closed，就地冻结）。
    def abort(reason: str) -> GraspRunResult:
        result.status = "aborted"
        result.reason = reason
        return result

    # 存证据帧；存图失败绝不允许影响运动行为。
    def save_frame(tag: str, frame: Any) -> None:
        saver = getattr(runtime, "save_frame", None)
        if callable(saver) and frame is not None:
            try:
                saver(tag, frame)
            except Exception:
                pass  # 证据丢失绝不允许改变运动行为

    # 手臂动作的唯一入口：发布一个完整双臂绝对目标，然后阻塞等待反馈
    # 收敛后才返回——保证"上一步没到位，下一步永远不会发"。
    def move_and_wait(target: EEFState, *, loaded: bool = False) -> EEFState:
        runtime.publish_eef_target(target)
        result.issued_targets += 1
        return parse_eef_state(runtime.wait_for_eef_feedback(target, loaded=loaded))

    try:
        # 阶段 1 预检：读一帧实时双臂状态；右臂 X 不在批准区间说明复位
        # 异常或姿态不对，一步都不动直接拒跑。
        record_stage("PREFLIGHT")
        state = parse_eef_state(runtime.read_eef_state())
        if not config.x_absolute_min_m <= state.right.position[0] <= config.x_absolute_max_m:
            return abort(
                f"preflight right X {state.right.position[0]:.4f} is outside the approved "
                f"[{config.x_absolute_min_m}, {config.x_absolute_max_m}] range"
            )

        # 阶段 2 张爪到 GRIPPER_OPEN_POSITION(=10) 并等反馈确认到位。
        record_stage("OPEN_GRIPPER")
        runtime.publish_gripper_target(GRIPPER_OPEN_POSITION)
        result.issued_targets += 1
        parse_gripper_state(runtime.wait_for_gripper_feedback(GRIPPER_OPEN_POSITION, config.close_tolerance))

        # 阶段 3 后退 0.10 m 拉开观察距离。注意 anchor（观察锚点）取的是
        # 反馈回来的实际位置而非计算目标；之后所有前进预算都以它结算。
        record_stage("BACK_CLEARANCE")
        back_target = move_right(state, dx=-BACK_CLEARANCE_M)
        if back_target.right.position[0] < config.x_absolute_min_m:
            return abort("back clearance would leave the approved X range")
        anchor = move_and_wait(back_target)
        anchor_x = anchor.right.position[0]
        anchor_y = anchor.right.position[1]

        # 阶段 4 一次性把 Z 设到标定抓取高度，且不得低于锚点下方 0.25 m
        # （防扎桌）。此后直到抬升前不再出现任何 Z 目标——伺服只在 X/Y
        # 平面进行，把三维问题降成二维。
        record_stage("SET_GRASP_HEIGHT", {"grasp_z_m": config.grasp_z_m})
        if config.grasp_z_m < anchor.right.position[2] - GRASP_Z_RELATIVE_FLOOR_M:
            return abort(
                f"grasp height {config.grasp_z_m:.3f} is below the -{GRASP_Z_RELATIVE_FLOOR_M} m floor "
                f"relative to the anchor height {anchor.right.position[2]:.3f}"
            )
        current = move_and_wait(set_right_z(anchor, config.grasp_z_m))

        # 阶段 5 视觉伺服主循环：每轮"看清 -> 判断 -> 只动一小步"。每轮
        # 必须先拿到连续多帧一致的稳定检测（acquire_stable）才允许动作。
        record_stage("SERVO")
        deadline = clock() + config.overall_timeout_s  # 伺服全程总预算（秒）
        y_net = 0.0  # Y 修正累计里程表（上限 Y_CUMULATIVE_MAX_M）
        first_frame_saved = False
        last_acquisition = None
        while True:
            if clock() >= deadline:
                return abort(f"overall visual-servo timeout of {config.overall_timeout_s} s reached")
            acquisition = runtime.acquire_stable(config)
            last_acquisition = acquisition
            if not getattr(acquisition, "stable", False):
                return abort(f"stable detection failed: {getattr(acquisition, 'reason', 'unknown')}")
            candidate = acquisition.candidate
            result.measurements.append(
                {
                    "at": clock(),
                    "center": list(candidate.center),
                    "width_ratio": candidate.width_ratio,
                    "bbox": list(candidate.bbox),
                    "frame_id": acquisition.frame_id,
                }
            )
            if not first_frame_saved:
                save_frame("initial_detection", getattr(acquisition, "frame", None))
                first_frame_saved = True
            # 门 1：候选纵向中心跑出有效窗口 => 高度或场景异常，中止。
            row = candidate.center[1]
            window_min, window_max = config.vertical_window_px
            if not window_min <= row <= window_max:
                return abort(
                    f"candidate vertical center {row:.1f} px left the validity window "
                    f"[{window_min}, {window_max}]"
                )
            # 门 2：宽度比例（瓶宽/图宽，作距离代理）超过闭合带上限 =>
            # 冲过头，中止而不自动回退。
            if candidate.width_ratio > config.close_width_ratio_max:
                return abort(
                    f"width ratio {candidate.width_ratio:.3f} overshot the close band maximum "
                    f"{config.close_width_ratio_max}; no automatic backtracking"
                )
            # 出口：横向对准（列偏差在容差内）且够近（进入闭合带）时跳出
            # 循环，进入后续闭爪流程。
            column_error = candidate.center[0] - config.target_column_px
            centered = abs(column_error) <= config.column_tolerance_px
            in_band = candidate.width_ratio >= config.close_width_ratio_min
            if centered and in_band:
                save_frame("pre_close", getattr(acquisition, "frame", None))
                break
            # 未对准 => 先修 Y 一步（方向由标定符号 column_to_y_sign 决定），
            # 修完立即 continue 重新观察，绝不边对中边前进。
            # 例外：向中线（+Y）的可达空间受 daemon 自碰撞门约束，实测约与
            # 前伸量 1:1 增长；预算不足时不中止，改为落入下方 X 前进分支，
            # 先扩大工作空间，下一轮再修 Y。
            if not centered:
                direction = config.column_to_y_sign if column_error > 0 else -config.column_to_y_sign
                step = direction * config.y_step_m
                if abs(y_net + step) > Y_CUMULATIVE_MAX_M + BOUNDARY_TOLERANCE:
                    return abort(f"Y correction would exceed the cumulative {Y_CUMULATIVE_MAX_M} m limit")
                midline_budget_m = MIDLINE_Y_PER_FORWARD_RATIO * (current.right.position[0] - anchor_x)
                midline_deferred = (
                    step > 0
                    and current.right.position[1] - anchor_y + step > midline_budget_m + BOUNDARY_TOLERANCE
                )
                if not midline_deferred:
                    current = move_and_wait(move_right(current, dy=step))
                    y_net += step
                    continue
            # 已对准但还不够近 => 前进一步 X：远处粗步、进入细区后细步。
            # 发布前先预扣最终前伸（final_forward）的额度，保证走到闭合带
            # 时总前进预算仍然够用；同时校验绝对 X 上限。
            step = (
                config.x_fine_step_m
                if candidate.width_ratio >= config.fine_zone_width_ratio
                else config.x_coarse_step_m
            )
            forward_from_anchor = current.right.position[0] - anchor_x
            if forward_from_anchor + step + config.final_forward_m > X_TOTAL_FORWARD_MAX_M + BOUNDARY_TOLERANCE:
                return abort(
                    f"X advance would exceed the {X_TOTAL_FORWARD_MAX_M} m total forward budget "
                    f"(reserving {config.final_forward_m} m final forward)"
                )
            if current.right.position[0] + step > config.x_absolute_max_m + BOUNDARY_TOLERANCE:
                return abort(f"X advance would leave the approved absolute range (max {config.x_absolute_max_m} m)")
            current = move_and_wait(move_right(current, dx=step))

        # 阶段 6 盲进：瓶子已近到即将出画/被爪遮挡，视觉不再可靠，按标定
        # 距离走完最后一段（仍受绝对 X 上限约束）。
        if config.final_forward_m > 0.0:
            record_stage("FINAL_FORWARD", {"distance_m": config.final_forward_m})
            if current.right.position[0] + config.final_forward_m > config.x_absolute_max_m + BOUNDARY_TOLERANCE:
                return abort("final forward would leave the approved absolute X range")
            current = move_and_wait(move_right(current, dx=config.final_forward_m))

        # 阶段 7 闭爪并验证：命令发 close_position，但等待的反馈中心是
        # grip_feedback_center±close_tolerance。真夹到瓶子会被瓶身堵转停在
        # 标定中心附近；夹空则一路闭到 close_position 本身——配置层强制
        # 该值在验证带外，因此"抓空"必然验证失败。失败后夹爪保持原样交
        # 由现场处理，绝不自动张爪重试。
        record_stage("CLOSE_GRIP", {"close_position": config.close_position})
        runtime.publish_gripper_target(config.close_position)
        result.issued_targets += 1
        try:
            # 真夹住会堵转停在标定反馈中心附近；夹空则收敛到
            # close_position，而配置层保证该值落在验证带之外。
            grip = parse_gripper_state(
                runtime.wait_for_gripper_feedback(config.grip_feedback_center, config.close_tolerance)
            )
        except FeedbackError as exc:
            record_stage("VERIFY_GRIP")
            return abort(f"grip verification failed: {exc}; gripper is left as-is for on-site handling")
        record_stage("VERIFY_GRIP", {"gripper_position": grip.position})

        # 阶段 8 持物抬升：全流程唯一 loaded=True 的动作，使用放宽反馈门。
        record_stage("LIFT_HOLD", {"lift_m": LIFT_M})
        current = move_and_wait(move_right(current, dz=LIFT_M), loaded=True)

        # 阶段 9 完成：保持闭爪持物待命，任何代码路径都不会自动松爪。
        record_stage("COMPLETE")
        result.status = "complete"
        result.reason = "grasp complete; holding the bottle with the gripper closed"
        return result
    except RunnerError as exc:
        return abort(str(exc))
    except Exception as exc:  # 防御边界：遇到意外错误绝不继续运动。
        return abort(f"unexpected runner failure: {exc}")


# ---------------------------------------------------------------------------
# 具体的 ROS runtime（只在真正 `run` 时构造）。
# ---------------------------------------------------------------------------


def _json_text(value: Any) -> str:
    return json.dumps(value, separators=(",", ":"), ensure_ascii=True, allow_nan=False)


class _RosRuntime:
    """沿用现场验证过的约定的原生接口适配层。"""

    def __init__(self, names: Mapping[str, str], evidence_dir: Any) -> None:
        import pathlib

        # 2026-08-13 现场验证：AutoLife SDK 必须在 rclpy.init() 之前
        # import——之后再 import 会死在其依赖 ssl 的链条里
        # （"'NoneType' object has no attribute 'VerifyMode'"）。在这里
        # import 会缓存模块，相机的惰性加载器随后直接复用。
        try:
            import cv2  # noqa: F401
            from autolife_robot_sdk import GLOBAL_VARS  # noqa: F401
            from autolife_robot_sdk.utils import (  # noqa: F401
                list_camera_shm_outputs,
                open_camera_shm_consumer,
            )
        except ImportError as exc:
            raise RunnerError(f"AutoLife SDK / OpenCV are unavailable before ROS init: {exc}") from exc

        try:
            import rclpy
            from rcl_interfaces.msg import Parameter, ParameterValue
            from rcl_interfaces.srv import SetParameters
            from rclpy.node import Node
            from std_msgs.msg import String
        except ImportError as exc:  # pragma: no cover - target-only dependency
            raise RunnerError(f"ROS 2 runtime is unavailable: {exc}") from exc
        self._rclpy = rclpy
        self._string_type = String
        self._parameter_type = Parameter
        self._parameter_value_type = ParameterValue
        self._set_parameters_type = SetParameters
        self.names = dict(names)
        self.evidence_dir = pathlib.Path(evidence_dir)
        self.evidence_dir.mkdir(parents=True, exist_ok=True)
        rclpy.init(args=None)
        self._node = Node("bottle_grasp")
        self._eef_pub = self._node.create_publisher(String, self.names["eef_target_topic"], 10)
        self._gripper_pub = self._node.create_publisher(String, self.names["gripper_target_topic"], 10)
        self._eef_raw: Any = None
        self._gripper_raw: Any = None
        self._eef_seq = 0
        self._gripper_seq = 0
        self._node.create_subscription(String, self.names["eef_state_topic"], self._on_eef, 10)
        self._node.create_subscription(String, self.names["gripper_state_topic"], self._on_gripper, 10)
        self._mode_client = self._node.create_client(SetParameters, self.names["right_mode_service"])
        self._camera: Any = None

    def _on_eef(self, message: Any) -> None:
        self._eef_seq += 1
        self._eef_raw = str(message.data)

    def _on_gripper(self, message: Any) -> None:
        self._gripper_seq += 1
        self._gripper_raw = str(message.data)

    def _spin_until(self, predicate: Callable[[], bool], timeout: float, label: str) -> None:
        deadline = time.monotonic() + timeout
        while self._rclpy.ok() and not predicate():
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise FeedbackError(f"timed out waiting for {label}")
            self._rclpy.spin_once(self._node, timeout_sec=min(0.1, remaining))
        if not self._rclpy.ok():
            raise RunnerError("ROS context stopped while waiting for feedback")

    def _require_subscriber(self, publisher: Any, topic: str) -> None:
        self._spin_until(
            lambda: int(publisher.get_subscription_count()) > 0,
            DEFAULT_FEEDBACK_TIMEOUT_SECONDS,
            f"a subscriber on {topic}",
        )

    # 实跑前接口预检：确认两个目标话题都有订阅者，并通过参数服务把右爪
    # 切到 position 模式；任何一步不满足都在发生动作前失败。
    def preflight_interfaces(self) -> None:
        self._require_subscriber(self._eef_pub, self.names["eef_target_topic"])
        self._require_subscriber(self._gripper_pub, self.names["gripper_target_topic"])
        if not self._mode_client.wait_for_service(timeout_sec=DEFAULT_FEEDBACK_TIMEOUT_SECONDS):
            raise RunnerError(f"position-mode service unavailable: {self.names['right_mode_service']}")
        request = self._set_parameters_type.Request()
        parameter = self._parameter_type()
        parameter.name = "param_arm_gripper_joint_control_mode"
        value = self._parameter_value_type()
        value.type = 4  # PARAMETER_STRING
        value.string_value = "position"
        parameter.value = value
        request.parameters = [parameter]
        future = self._mode_client.call_async(request)
        self._spin_until(lambda: future.done(), DEFAULT_FEEDBACK_TIMEOUT_SECONDS, "right position mode")
        response = future.result()
        results = getattr(response, "results", None)
        if not results or not bool(getattr(results[0], "successful", False)):
            raise RunnerError("right position-mode service rejected the request")

    def read_eef_state(self) -> EEFState:
        baseline = self._eef_seq
        self._spin_until(lambda: self._eef_seq > baseline and self._eef_raw is not None, DEFAULT_FEEDBACK_TIMEOUT_SECONDS, "EEF state")
        return parse_eef_state(json.loads(self._eef_raw))

    def publish_eef_target(self, target: EEFState, repeats: int = 1) -> None:
        self._require_subscriber(self._eef_pub, self.names["eef_target_topic"])
        self._eef_baseline = self._eef_seq
        message = self._string_type()
        message.data = _json_text(state_payload(target))
        # 控制器可能丢弃单次发布的目标（夹爪侧 2026-08-13 现场验证；
        # EEF 目标在 2026-08-20 的投放流程中也出现过：某一轴步进始终没动，
        # 负载反馈门超时）。绝对全位姿目标是幂等的，所以不能容忍丢消息的
        # 调用方像夹爪路径一样传 repeats=3；默认值让伺服循环保持单次发布。
        for index in range(max(1, int(repeats))):
            if index:
                time.sleep(0.2)
            self._eef_pub.publish(message)

    def wait_for_eef_feedback(self, target: EEFState, *, loaded: bool = False) -> EEFState:
        state_box: list[EEFState] = []
        position_tolerance = LOADED_POSITION_FEEDBACK_MAX_M if loaded else POSITION_FEEDBACK_MAX_M
        rotation_tolerance = LOADED_ROTATION_FEEDBACK_MAX_DEG if loaded else ROTATION_FEEDBACK_MAX_DEG
        timeout = LOADED_FEEDBACK_TIMEOUT_SECONDS if loaded else DEFAULT_FEEDBACK_TIMEOUT_SECONDS

        def reached() -> bool:
            if self._eef_seq <= getattr(self, "_eef_baseline", 0) or self._eef_raw is None:
                return False
            try:
                state = parse_eef_state(json.loads(self._eef_raw))
            except Exception:
                return False
            if eef_target_reached(
                state,
                target,
                position_tolerance_m=position_tolerance,
                rotation_tolerance_deg=rotation_tolerance,
            ):
                state_box.append(state)
                return True
            return False

        self._spin_until(reached, timeout, "EEF target convergence")
        return state_box[-1]

    def publish_gripper_target(self, target: float) -> None:
        self._require_subscriber(self._gripper_pub, self.names["gripper_target_topic"])
        self._gripper_baseline = self._gripper_seq
        message = self._string_type()
        message.data = _json_text({"right_gripper_target_joints_position": [float(target)]})
        # 2026-08-13 现场验证：单次发布可能被控制器丢弃
        # （control_gripper.py 的一次性发布从未让夹爪动过）；
        # 可用的 arm_move 路径是把同一个有界目标连发 3 次。
        for _ in range(3):
            self._gripper_pub.publish(message)
            time.sleep(0.2)

    def wait_for_gripper_feedback(self, target: float, tolerance: float) -> GripperState:
        state_box: list[GripperState] = []

        def reached() -> bool:
            if self._gripper_seq <= getattr(self, "_gripper_baseline", 0) or self._gripper_raw is None:
                return False
            try:
                state = parse_gripper_state(json.loads(self._gripper_raw))
            except Exception:
                return False
            if gripper_target_reached(state, target, tolerance):
                state_box.append(state)
                return True
            return False

        self._spin_until(reached, 20, "gripper target convergence")
        return state_box[-1]

    def acquire_stable(self, config: BottleGraspConfig) -> Any:
        try:
            from examples.camera.detect_bottle import (
                BottleDetectorConfig,
                StableBottleConfig,
                acquire_stable_bottle,
            )
            from examples.camera.read_hand_camera import RightHandDecodedCamera
        except ImportError as exc:  # pragma: no cover - target-only path layout
            raise RunnerError(f"detection modules are unavailable: {exc}") from exc
        if self._camera is None:
            self._camera = RightHandDecodedCamera(timeout=2.0)
        detector_config = BottleDetectorConfig(
            hsv_lower=config.hsv_lower,
            hsv_upper=config.hsv_upper,
            min_area=config.min_area,
            roi=config.detection_roi_px,
        )
        return acquire_stable_bottle(self._camera, detector_config, StableBottleConfig())

    def save_frame(self, tag: str, frame: Any) -> None:
        image = getattr(frame, "image", None)
        frame_id = getattr(frame, "frame_id", "unknown")
        if image is None:
            return
        try:
            import cv2

            cv2.imwrite(str(self.evidence_dir / f"{tag}_id{frame_id}.jpg"), image)
        except Exception:
            pass

    def close(self) -> None:
        camera = self._camera
        self._camera = None
        if camera is not None:
            try:
                camera.close()
            except Exception:
                pass
        try:
            self._node.destroy_node()
        finally:
            self._rclpy.shutdown()


# ---------------------------------------------------------------------------
# CLI：默认 preview；单次任务用 `run --execute --reset-confirmed`。
# ---------------------------------------------------------------------------


def _preview_text(config_path: str) -> str:
    lines = [
        "Bottle grasp preview",
        "  mode: preview",
        "  planned_stages: " + " -> ".join(STAGES),
        (
            "  limits: "
            f"back_clearance={BACK_CLEARANCE_M}m; y_step<=0.005m; y_net<={Y_CUMULATIVE_MAX_M}m; "
            f"x_coarse<=0.05m; x_fine<=0.005m; x_forward_total<={X_TOTAL_FORWARD_MAX_M}m; "
            f"x_absolute<={X_ABSOLUTE_MAX_M}m; final_forward<=0.15m (default 0); "
            f"grasp_z_floor=reset-{GRASP_Z_RELATIVE_FLOOR_M}m; lift={LIFT_M}m; no Z targets during servo"
        ),
        "  end_state: gripper stays closed holding the bottle (no automatic release)",
        f"  config: {config_path}",
        "  topics: " + _json_text(interface_names()),
        "  gates: --execute absent; --reset-confirmed absent",
        "  no action: no EEF publication and no gripper publication",
    ]
    return "\n".join(lines)


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Lean right-hand bottle visual-servo grasp.")
    parser.add_argument("mode", nargs="?", default="preview", choices=("preview", "run"))
    parser.add_argument("--execute", action="store_true", help="confirm physical execution")
    parser.add_argument(
        "--reset-confirmed",
        action="store_true",
        help="confirm that an independent whole-robot reset visibly completed",
    )
    parser.add_argument("--config", default="right/bottle_grasp.json", help="calibration configuration path")
    parser.add_argument("--workcell-id", default="s2-test-point2-black-cloth")
    parser.add_argument("--evidence-dir", default=None)
    args = parser.parse_args(argv)

    # 默认 preview：只打印计划与限值，不 import ROS、零动作。实体执行必须
    # 同时提供 --execute 与 --reset-confirmed（独立整机复位已肉眼确认完成）。
    if args.mode == "preview":
        print(_preview_text(args.config))
        return 0
    if not args.execute or not args.reset_confirmed:
        print(
            "Error: physical execution requires both --execute and --reset-confirmed "
            "after an independent whole-robot reset has visibly completed.",
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
        import pathlib

        stamp = datetime.datetime.now(datetime.timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        evidence_dir = pathlib.Path.home() / "Documents" / "AutolifeXAIR" / "evidence" / "bottle_grasp" / stamp

    runtime = None
    try:
        runtime = _RosRuntime(interface_names(), evidence_dir)
        runtime.preflight_interfaces()
        result = run_grasp(config, runtime)
    except RunnerError as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 4
    finally:
        if runtime is not None:
            runtime.close()

    record = {"config": config.to_record(), "result": result.to_record()}
    import pathlib

    record_path = pathlib.Path(evidence_dir) / "run_record.json"
    record_path.parent.mkdir(parents=True, exist_ok=True)
    record_path.write_text(json.dumps(record, ensure_ascii=False, indent=1), encoding="utf-8")
    print(json.dumps(result.to_record(), ensure_ascii=False, indent=1))
    print(f"evidence: {record_path}")
    # 退出码：0 成功、2 缺执行门禁、3 配置错误、4 运行时错误、5 任务中止。
    return 0 if result.complete else 5


if __name__ == "__main__":
    raise SystemExit(main())
