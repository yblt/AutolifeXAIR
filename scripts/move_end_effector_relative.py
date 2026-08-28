#!/usr/bin/env python3
"""预览或发布一条有界的相对末端执行器指令。

实测的 AutoLife 接口在 ``std_msgs/msg/String`` 上接受双臂完整绝对
位姿。本示例对一侧施加一个小的项目侧相对增量，另一侧通常沿用同一
状态快照，然后发出完整的四字段载荷。经明确批准的实验也可以在多次
调用间使用固定的未选侧锚点。这套相对约定是获批的临时实验策略，
不是 AutoLife 的增量 API：

* 位置为机器人基座坐标下的米；
* 四元数用 ``[x, y, z, w]``，且 ``q_target = q_delta * q_current``；
* 单次平移范数不超过 0.30 m，或单次旋转幅度不超过 20.0 度；
* 单个进程受起始位姿局部包络约束，不超过 0.30 m/20.0 度；
* 只改一侧，只发布一条完整指令，无连续指令流。

默认 preview。``--state-json`` 是仅限离线的状态来源；``--execute``
要求新鲜的 ROS 状态和显式安全门。ROS 的 import 被推迟，因此
``--help`` 和离线 preview 不需要 ROS。
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
from dataclasses import dataclass
from typing import Any, Mapping, Sequence


EXIT_OK = 0
EXIT_ARGUMENT = 2
EXIT_ROS_UNAVAILABLE = 3
EXIT_STATE = 4
EXIT_PUBLISH = 5
EXIT_FEEDBACK = 6
EXIT_INTERRUPTED = 130

TRANSLATION_MAX_METRES = 0.30
ROTATION_MAX_DEGREES = 20.0
START_POSE_ENVELOPE_MAX_METRES = 0.30
START_POSE_ENVELOPE_MAX_DEGREES = 20.0
# 只吸收批准包络边界上的浮点舍入误差。
START_POSE_ENVELOPE_TOLERANCE_METRES = 1e-12
START_POSE_ENVELOPE_TOLERANCE_DEGREES = 1e-12
UNSELECTED_ANCHOR_MAX_METRES = 0.003
UNSELECTED_ANCHOR_MAX_DEGREES = 1.0
UNSELECTED_ANCHOR_TOLERANCE_METRES = 1e-12
UNSELECTED_ANCHOR_TOLERANCE_DEGREES = 1e-12
STATE_TIMEOUT_SECONDS = 5.0
PUBLISH_SUBSCRIBER_TIMEOUT_SECONDS = 5.0
MODE_SERVICE_TIMEOUT_SECONDS = 5.0
# 控制器可能丢弃单次发布的目标（夹爪侧 2026-08-13、EEF 目标
# 2026-08-20/21 均现场验证过）；绝对全位姿目标是幂等的，因此像
# 抓取链一样重复发布。
PUBLISH_REPEATS = 3
PUBLISH_REPEAT_INTERVAL_SECONDS = 0.2
FEEDBACK_TIMEOUT_SECONDS = 10.0
FEEDBACK_POSITION_MAX_M = 0.02
FEEDBACK_ROTATION_MAX_DEG = 3.0


class StateValidationError(ValueError):
    """EEF 状态不符合已确认的 JSON 形态时抛出。"""


@dataclass(frozen=True)
class Pose:
    position: tuple[float, float, float]
    rotation: tuple[float, float, float, float]


@dataclass(frozen=True)
class EefState:
    left: Pose
    right: Pose


@dataclass(frozen=True)
class RelativeDelta:
    operation: str
    translation: tuple[float, float, float] | None = None
    axis: str | None = None
    degrees: float | None = None


@dataclass(frozen=True)
class RelativePlan:
    side: str
    delta: RelativeDelta
    current: EefState
    target: EefState
    payload: dict[str, list[float]]
    unselected_anchor: Pose | None = None
    unselected_anchor_position_distance: float | None = None
    unselected_anchor_rotation_degrees: float | None = None


def _default_identifier(name: str, fallback: str) -> str:
    value = os.environ.get(name, fallback).strip()
    return value or fallback


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Preview one bounded relative left/right EEF translation or "
            "rotation. Execution requires --execute, --safety-acknowledged, "
            "and --experimental-policy-approved."
        )
    )
    parser.add_argument(
        "--side",
        choices=("left", "right"),
        required=True,
        help="EEF side to change (required; no implicit side)",
    )
    operation_group = parser.add_mutually_exclusive_group()
    operation_group.add_argument(
        "--translate",
        nargs=3,
        metavar=("DX", "DY", "DZ"),
        help=(
            "one robot-base-frame translation in metres; vector norm must be "
            f"> 0 and <= {TRANSLATION_MAX_METRES:g}"
        ),
    )
    operation_group.add_argument(
        "--rotate",
        nargs=2,
        metavar=("AXIS", "DEGREES"),
        help=(
            "one robot-base-frame rotation about x, y, or z; absolute degrees "
            f"must be > 0 and <= {ROTATION_MAX_DEGREES:g}"
        ),
    )
    parser.add_argument(
        "--state-json",
        help=(
            "inline JSON state for offline preview only; it is rejected with "
            "--execute (default reads the current-state topic)"
        ),
    )
    parser.add_argument(
        "--unselected-anchor-json",
        help=(
            "full-shape EEF state JSON whose unselected-side pose is held "
            "across repeated calls; preview only unless the active hold "
            "gate is also supplied"
        ),
    )
    parser.add_argument(
        "--state-timeout",
        type=float,
        default=STATE_TIMEOUT_SECONDS,
        metavar="SECONDS",
        help=f"bounded wait for a current state (default: {STATE_TIMEOUT_SECONDS:g})",
    )
    parser.add_argument(
        "--domain-id",
        default=_default_identifier("ROS_DOMAIN_ID", "0"),
        help="ROS domain identifier (default: ROS_DOMAIN_ID or 0)",
    )
    parser.add_argument(
        "--robot-id",
        default=os.uname().nodename.rsplit("-", 1)[-1],  # 一律按主机名推导
        help="robot identifier (default: hostname suffix)",
    )
    parser.add_argument(
        "--execute",
        action="store_true",
        help="enable one physical publication (first execution gate)",
    )
    parser.add_argument(
        "--safety-acknowledged",
        action="store_true",
        help=(
            "confirm the workspace is clear, an operator has the emergency "
            "stop, and this target is approved (second execution gate)"
        ),
    )
    parser.add_argument(
        "--experimental-policy-approved",
        action="store_true",
        help=(
            "confirm the project-side temporary relative-control policy was "
            "approved (third execution gate)"
        ),
    )
    parser.add_argument(
        "--active-unselected-hold-approved",
        action="store_true",
        help=(
            "confirm that an anchor may actively correct the unselected arm "
            "to its fixed pose during execution"
        ),
    )
    return parser


def _parse_finite(raw: Any, label: str) -> float:
    if isinstance(raw, bool) or not isinstance(raw, (int, float)):
        raise StateValidationError(f"{label} must be a finite number")
    value = float(raw)
    if not math.isfinite(value):
        raise StateValidationError(f"{label} must be a finite number")
    return value


def _parse_cli_finite(raw: Any, label: str) -> float:
    """解析命令行数字，同时保留严格的 JSON 校验语义。"""

    if isinstance(raw, bool):
        raise ValueError(f"{label} must be a finite number")
    try:
        value = float(raw)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{label} must be a finite number") from exc
    if not math.isfinite(value):
        raise ValueError(f"{label} must be a finite number")
    return value


def _parse_delta(args: argparse.Namespace, parser: argparse.ArgumentParser) -> RelativeDelta:
    if args.translate is None and args.rotate is None:
        parser.error("provide exactly one of --translate DX DY DZ or --rotate AXIS DEGREES")

    if args.translate is not None:
        try:
            values = tuple(
                _parse_cli_finite(raw, f"translate component {index + 1}")
                for index, raw in enumerate(args.translate)
            )
        except ValueError as exc:
            parser.error(str(exc))
        norm = math.sqrt(sum(value * value for value in values))
        if norm == 0.0:
            parser.error("--translate vector must be non-zero")
        if norm > TRANSLATION_MAX_METRES:
            parser.error(
                f"--translate vector norm must be <= {TRANSLATION_MAX_METRES:g} m "
                f"(got {norm:.9g})"
            )
        return RelativeDelta("translate", translation=values)

    raw_axis, raw_degrees = args.rotate
    axis = str(raw_axis).strip().lower()
    if axis not in {"x", "y", "z"}:
        parser.error("--rotate AXIS must use one of x, y, or z")
    try:
        degrees = _parse_cli_finite(raw_degrees, "rotation degrees")
    except ValueError as exc:
        parser.error(str(exc))
    if degrees == 0.0:
        parser.error("--rotate degrees must be non-zero")
    if abs(degrees) > ROTATION_MAX_DEGREES:
        parser.error(
            f"absolute --rotate degrees must be <= {ROTATION_MAX_DEGREES:g} "
            f"(got {degrees:.9g})"
        )
    return RelativeDelta("rotate", axis=axis, degrees=degrees)


def _validate_args(args: argparse.Namespace, parser: argparse.ArgumentParser) -> None:
    if args.state_timeout <= 0 or not math.isfinite(args.state_timeout):
        parser.error("--state-timeout must be a finite value greater than zero")
    domain_id = str(args.domain_id).strip()
    if not domain_id:
        parser.error("--domain-id must not be empty")
    try:
        if int(domain_id) < 0:
            raise ValueError
    except (TypeError, ValueError):
        parser.error("--domain-id must be a non-negative integer")
    if not str(args.robot_id).strip():
        parser.error("--robot-id must not be empty")
    if args.execute and not args.safety_acknowledged:
        parser.error("--execute requires --safety-acknowledged")
    if args.execute and not args.experimental_policy_approved:
        parser.error("--execute requires --experimental-policy-approved")
    if args.execute and args.state_json is not None:
        parser.error("--execute requires a realtime state; --state-json is offline-only")
    if (
        args.execute
        and getattr(args, "unselected_anchor_json", None) is not None
        and not getattr(args, "active_unselected_hold_approved", False)
    ):
        parser.error(
            "--execute with --unselected-anchor-json requires "
            "--active-unselected-hold-approved"
        )


def interface_names(domain_id: str, robot_id: str) -> dict[str, str]:
    """构建在目标机器人上实测到的接口名。"""

    suffix = f"{domain_id}_{robot_id}"
    return {
        "state_topic": f"/topic_arm_current_robot_eef_pose_{suffix}",
        "move_topic": f"/topic_arm_move_eef_pose_in_robot_frame_{suffix}",
        "left_mode_service": f"/node_mod_motor_left_arm_control_{suffix}/set_parameters",
        "right_mode_service": f"/node_mod_motor_right_arm_control_{suffix}/set_parameters",
    }


def _as_sequence(value: Any, label: str, length: int) -> tuple[Any, ...]:
    if isinstance(value, (str, bytes, bytearray)) or not isinstance(value, Sequence):
        raise StateValidationError(f"{label} must be an array of length {length}")
    if len(value) != length:
        raise StateValidationError(f"{label} must be an array of length {length}")
    return tuple(value)


def _normalize_quaternion(values: Sequence[Any], label: str) -> tuple[float, float, float, float]:
    numbers = tuple(_parse_finite(value, f"{label}[{index}]") for index, value in enumerate(values))
    norm = math.sqrt(sum(value * value for value in numbers))
    if norm == 0.0:
        raise StateValidationError(f"{label} must not be the zero quaternion")
    return tuple(value / norm for value in numbers)


def _parse_pose(raw_pose: Any, side: str) -> Pose:
    if not isinstance(raw_pose, Mapping):
        raise StateValidationError(f"{side}_eef_pose must be an object")
    if "position" not in raw_pose or "rotation" not in raw_pose:
        raise StateValidationError(f"{side}_eef_pose requires position and rotation")
    position_values = _as_sequence(raw_pose["position"], f"{side}.position", 3)
    rotation_values = _as_sequence(raw_pose["rotation"], f"{side}.rotation", 4)
    position = tuple(
        _parse_finite(value, f"{side}.position[{index}]")
        for index, value in enumerate(position_values)
    )
    rotation = _normalize_quaternion(rotation_values, f"{side}.rotation")
    return Pose(position=position, rotation=rotation)


def parse_state(value: Any) -> EefState:
    """严格校验并归一化双侧当前 EEF 状态。"""

    if not isinstance(value, Mapping):
        raise StateValidationError("state JSON must be an object")
    if "left_eef_pose" not in value or "right_eef_pose" not in value:
        raise StateValidationError("state JSON requires left_eef_pose and right_eef_pose")
    return EefState(
        left=_parse_pose(value["left_eef_pose"], "left"),
        right=_parse_pose(value["right_eef_pose"], "right"),
    )


def _parse_state_json(raw: str) -> EefState:
    try:
        decoded = json.loads(raw)
    except (TypeError, json.JSONDecodeError) as exc:
        raise StateValidationError(f"--state-json is not valid JSON: {exc}") from exc
    return parse_state(decoded)


def _unselected_side(side: str) -> str:
    return "right" if side == "left" else "left"


def _pose_position_distance(first: Pose, second: Pose) -> float:
    return math.sqrt(
        sum((first.position[index] - second.position[index]) ** 2 for index in range(3))
    )


def _pose_rotation_distance_degrees(first: Pose, second: Pose) -> float:
    dot = sum(first.rotation[index] * second.rotation[index] for index in range(4))
    clamped_abs_dot = min(1.0, max(0.0, abs(dot)))
    return math.degrees(2.0 * math.acos(clamped_abs_dot))


def _validate_unselected_anchor(side: str, current: Pose, anchor: Pose) -> tuple[float, float]:
    """拒绝与当前位姿已相距过远的固定未选侧锚点。"""

    unselected = _unselected_side(side)
    position_distance = _pose_position_distance(current, anchor)
    if position_distance > (
        UNSELECTED_ANCHOR_MAX_METRES + UNSELECTED_ANCHOR_TOLERANCE_METRES
    ):
        raise StateValidationError(
            f"{unselected} current pose exceeds the unselected anchor: position "
            f"distance {position_distance:.9g} m > {UNSELECTED_ANCHOR_MAX_METRES:g} m"
        )

    rotation_degrees = _pose_rotation_distance_degrees(current, anchor)
    if rotation_degrees > (
        UNSELECTED_ANCHOR_MAX_DEGREES + UNSELECTED_ANCHOR_TOLERANCE_DEGREES
    ):
        raise StateValidationError(
            f"{unselected} current pose exceeds the unselected anchor: rotation "
            f"distance {rotation_degrees:.9g} deg > {UNSELECTED_ANCHOR_MAX_DEGREES:g} deg"
        )
    return position_distance, rotation_degrees


def _quaternion_multiply(
    left: Sequence[float], right: Sequence[float]
) -> tuple[float, float, float, float]:
    x1, y1, z1, w1 = left
    x2, y2, z2, w2 = right
    return (
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    )


def _axis_quaternion(axis: str, degrees: float) -> tuple[float, float, float, float]:
    half_angle = math.radians(degrees) / 2.0
    sine = math.sin(half_angle)
    components = {"x": (sine, 0.0, 0.0), "y": (0.0, sine, 0.0), "z": (0.0, 0.0, sine)}
    x, y, z = components[axis]
    return (x, y, z, math.cos(half_angle))


def _apply_delta(
    state: EefState,
    side: str,
    delta: RelativeDelta,
    unselected_anchor: Pose | None = None,
) -> EefState:
    selected = state.left if side == "left" else state.right
    if delta.operation == "translate":
        assert delta.translation is not None
        target_position = tuple(
            selected.position[index] + delta.translation[index] for index in range(3)
        )
        target_rotation = selected.rotation
    else:
        assert delta.axis is not None and delta.degrees is not None
        target_position = selected.position
        target_rotation = _normalize_quaternion(
            _quaternion_multiply(_axis_quaternion(delta.axis, delta.degrees), selected.rotation),
            "target.rotation",
        )
    changed = Pose(position=target_position, rotation=target_rotation)
    unselected = _unselected_side(side)
    left = changed if side == "left" else state.left
    right = changed if side == "right" else state.right
    if unselected_anchor is not None:
        if unselected == "left":
            left = unselected_anchor
        else:
            right = unselected_anchor
    return EefState(left=left, right=right)


def _pose_values(pose: Pose) -> dict[str, list[float]]:
    return {
        "position": [float(value) for value in pose.position],
        "rotation": [float(value) for value in pose.rotation],
    }


def _payload_for_state(state: EefState) -> dict[str, list[float]]:
    return {
        "pos_left_in_robot": list(state.left.position),
        "quat_left_in_robot": list(state.left.rotation),
        "pos_right_in_robot": list(state.right.position),
        "quat_right_in_robot": list(state.right.rotation),
    }


def _validate_start_pose_envelope(side: str, current: Pose, target: Pose) -> None:
    """拒绝超出单进程局部位姿包络的目标。"""

    position_distance = math.sqrt(
        sum(
            (target.position[index] - current.position[index]) ** 2
            for index in range(3)
        )
    )
    if position_distance > (
        START_POSE_ENVELOPE_MAX_METRES + START_POSE_ENVELOPE_TOLERANCE_METRES
    ):
        raise StateValidationError(
            f"{side} target exceeds the start-pose envelope: position distance "
            f"{position_distance:.9g} m > {START_POSE_ENVELOPE_MAX_METRES:g} m"
        )

    dot = sum(
        current.rotation[index] * target.rotation[index] for index in range(4)
    )
    clamped_abs_dot = min(1.0, max(0.0, abs(dot)))
    rotation_degrees = math.degrees(2.0 * math.acos(clamped_abs_dot))
    if rotation_degrees > (
        START_POSE_ENVELOPE_MAX_DEGREES + START_POSE_ENVELOPE_TOLERANCE_DEGREES
    ):
        raise StateValidationError(
            f"{side} target exceeds the start-pose envelope: rotation distance "
            f"{rotation_degrees:.9g} deg > {START_POSE_ENVELOPE_MAX_DEGREES:g} deg"
        )


def make_plan(
    side: str,
    delta: RelativeDelta,
    state: EefState,
    unselected_anchor_state: EefState | None = None,
) -> RelativePlan:
    unselected_anchor = None
    anchor_position_distance = None
    anchor_rotation_degrees = None
    if unselected_anchor_state is not None:
        unselected = _unselected_side(side)
        unselected_anchor = (
            unselected_anchor_state.left
            if unselected == "left"
            else unselected_anchor_state.right
        )
        current_unselected = state.left if unselected == "left" else state.right
        anchor_position_distance, anchor_rotation_degrees = _validate_unselected_anchor(
            side, current_unselected, unselected_anchor
        )
    target = _apply_delta(state, side, delta, unselected_anchor)
    current_pose = state.left if side == "left" else state.right
    target_pose = target.left if side == "left" else target.right
    _validate_start_pose_envelope(side, current_pose, target_pose)
    return RelativePlan(
        side=side,
        delta=delta,
        current=state,
        target=target,
        payload=_payload_for_state(target),
        unselected_anchor=unselected_anchor,
        unselected_anchor_position_distance=anchor_position_distance,
        unselected_anchor_rotation_degrees=anchor_rotation_degrees,
    )


def _json_text(value: Any) -> str:
    return json.dumps(value, separators=(",", ":"), ensure_ascii=False, allow_nan=False)


def _format_pose(pose: Pose) -> str:
    return _json_text(_pose_values(pose))


def _delta_text(delta: RelativeDelta) -> str:
    if delta.operation == "translate":
        assert delta.translation is not None
        return f"translate metres={_json_text(list(delta.translation))}"
    assert delta.axis is not None and delta.degrees is not None
    return f"rotate axis={delta.axis} degrees={delta.degrees:.9g}"


def print_preview(
    plan: RelativePlan,
    names: Mapping[str, str],
    state_source: str,
    execute: bool,
    safety_acknowledged: bool,
    experimental_policy_approved: bool,
    active_unselected_hold_approved: bool = False,
) -> None:
    print("Relative EEF command preview")
    print(f"  side: {plan.side}")
    print(f"  state_source: {state_source}")
    print(f"  current_left_eef_pose: {_format_pose(plan.current.left)}")
    print(f"  current_right_eef_pose: {_format_pose(plan.current.right)}")
    print(f"  delta: {_delta_text(plan.delta)}")
    print(f"  target_left_eef_pose: {_format_pose(plan.target.left)}")
    print(f"  target_right_eef_pose: {_format_pose(plan.target.right)}")
    unselected = _unselected_side(plan.side)
    if plan.unselected_anchor is None:
        print(
            "  unselected_anchor_source: disabled (current snapshot; "
            f"{unselected} target follows current snapshot)"
        )
        print("  active_unselected_hold_gate: absent (not applicable)")
    else:
        print("  unselected_anchor_source: --unselected-anchor-json")
        print(
            f"  unselected_anchor_{unselected}_pose: "
            f"{_format_pose(plan.unselected_anchor)}"
        )
        print(
            "  current_to_unselected_anchor: "
            f"position_distance={plan.unselected_anchor_position_distance:.9g} m; "
            f"rotation_distance={plan.unselected_anchor_rotation_degrees:.9g} deg"
        )
        print(
            "  active_unselected_hold_gate: "
            f"{'present' if active_unselected_hold_approved else 'absent'}"
        )
        print(
            "  warning: an anchored execute may actively correct the "
            f"unselected {unselected} arm toward the fixed pose"
        )
    print(f"  state_topic: {names['state_topic']}")
    print(f"  move_topic: {names['move_topic']}")
    print(f"  std_msgs/msg/String payload: {_json_text(plan.payload)}")
    print(
        "  policy: project-side temporary relative convention; "
        f"translation norm <= {TRANSLATION_MAX_METRES:g} m; "
        f"rotation abs <= {ROTATION_MAX_DEGREES:g} deg; "
        f"start envelope <= {START_POSE_ENVELOPE_MAX_METRES:g} m/"
        f"{START_POSE_ENVELOPE_MAX_DEGREES:g} deg; one bounded publish; no accumulation"
    )
    print(f"  execution_gate --execute: {'present' if execute else 'absent'}")
    print(
        "  safety_gate --safety-acknowledged: "
        f"{'present' if safety_acknowledged else 'absent'}"
    )
    print(
        "  policy_gate --experimental-policy-approved: "
        f"{'present' if experimental_policy_approved else 'absent'}"
    )


def _ros_imports() -> tuple[Any, Any, Any]:
    """只在读取当前状态或门禁通过的执行时才 import ROS。"""

    try:
        import rclpy
        from rclpy.node import Node
        from std_msgs.msg import String
    except ImportError as exc:  # pragma: no cover - target-only dependency
        raise RuntimeError(
            "ROS 2 runtime is unavailable; current EEF state cannot be read "
            "(offline --state-json previews do not require ROS)."
        ) from exc
    return rclpy, Node, String


def _read_state(
    rclpy: Any,
    node: Any,
    string_type: Any,
    topic: str,
    timeout: float,
) -> EefState:
    result: dict[str, EefState | StateValidationError | None] = {"state": None, "error": None}

    def callback(message: Any) -> None:
        if result["state"] is not None or result["error"] is not None:
            return
        try:
            result["state"] = _parse_state_json(str(message.data))
        except StateValidationError as exc:
            result["error"] = exc

    node.create_subscription(string_type, topic, callback, 10)
    deadline = time.monotonic() + timeout
    while rclpy.ok() and result["state"] is None and result["error"] is None:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            break
        rclpy.spin_once(node, timeout_sec=min(0.1, remaining))
    if result["error"] is not None:
        raise result["error"]
    state = result["state"]
    if state is None:
        raise TimeoutError(f"no valid current EEF state received on {topic} within {timeout:g} seconds")
    return state


def _set_position_mode(
    rclpy: Any,
    node: Any,
    service_name: str,
    timeout: float = MODE_SERVICE_TIMEOUT_SECONDS,
) -> None:
    """把所选手臂的电机节点切到 position 模式（抓取链验证过的预检）。"""

    from rcl_interfaces.msg import Parameter, ParameterValue
    from rcl_interfaces.srv import SetParameters

    client = node.create_client(SetParameters, service_name)
    if not client.wait_for_service(timeout_sec=timeout):
        raise TimeoutError(f"position-mode service unavailable: {service_name}")
    request = SetParameters.Request()
    parameter = Parameter()
    parameter.name = "param_arm_gripper_joint_control_mode"
    value = ParameterValue()
    value.type = 4  # PARAMETER_STRING
    value.string_value = "position"
    parameter.value = value
    request.parameters = [parameter]
    future = client.call_async(request)
    deadline = time.monotonic() + timeout
    while rclpy.ok() and not future.done():
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise TimeoutError(f"position-mode request timed out: {service_name}")
        rclpy.spin_once(node, timeout_sec=min(0.1, remaining))
    response = future.result()
    results = getattr(response, "results", None)
    if not results or not bool(getattr(results[0], "successful", False)):
        raise RuntimeError(f"position-mode service rejected the request: {service_name}")


def _publish_bounded(
    rclpy: Any,
    node: Any,
    string_type: Any,
    topic: str,
    payload: str,
    repeats: int = PUBLISH_REPEATS,
    timeout: float = PUBLISH_SUBSCRIBER_TIMEOUT_SECONDS,
) -> None:
    publisher = node.create_publisher(string_type, topic, 10)
    deadline = time.monotonic() + timeout
    while rclpy.ok():
        subscriber_count = int(publisher.get_subscription_count())
        if subscriber_count > 0:
            break
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise TimeoutError(
                f"no subscriber appeared on {topic} within {timeout:g} seconds; nothing was published"
            )
        rclpy.spin_once(node, timeout_sec=min(0.1, remaining))
    else:
        raise RuntimeError("ROS context stopped before a move-topic subscriber appeared")
    message = string_type()
    message.data = payload
    for index in range(max(1, int(repeats))):
        if index:
            time.sleep(PUBLISH_REPEAT_INTERVAL_SECONDS)
        publisher.publish(message)


def _wait_for_feedback(
    rclpy: Any,
    node: Any,
    string_type: Any,
    topic: str,
    plan: RelativePlan,
    timeout: float = FEEDBACK_TIMEOUT_SECONDS,
) -> tuple[bool, float, float]:
    """轮询状态话题，直到所选侧到达目标或超时。"""

    target = plan.target.left if plan.side == "left" else plan.target.right
    latest: dict[str, Pose | None] = {"pose": None}

    def callback(message: Any) -> None:
        try:
            state = _parse_state_json(str(message.data))
        except StateValidationError:
            return
        latest["pose"] = state.left if plan.side == "left" else state.right

    node.create_subscription(string_type, topic, callback, 10)
    deadline = time.monotonic() + timeout
    position_error = math.inf
    rotation_error = math.inf
    while rclpy.ok():
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            break
        rclpy.spin_once(node, timeout_sec=min(0.1, remaining))
        pose = latest["pose"]
        if pose is None:
            continue
        position_error = _pose_position_distance(pose, target)
        rotation_error = _pose_rotation_distance_degrees(pose, target)
        if position_error <= FEEDBACK_POSITION_MAX_M and rotation_error <= FEEDBACK_ROTATION_MAX_DEG:
            return True, position_error, rotation_error
    return False, position_error, rotation_error


def run(
    args: argparse.Namespace,
    delta: RelativeDelta,
    names: Mapping[str, str],
) -> int:
    anchor_json = getattr(args, "unselected_anchor_json", None)
    active_unselected_hold_approved = bool(
        getattr(args, "active_unselected_hold_approved", False)
    )
    anchor_state = None
    if anchor_json is not None:
        try:
            anchor_state = _parse_state_json(anchor_json)
        except StateValidationError as exc:
            print(f"ERROR: --unselected-anchor-json: {exc}", file=sys.stderr)
            return EXIT_STATE

    if args.state_json is not None:
        try:
            state = _parse_state_json(args.state_json)
        except StateValidationError as exc:
            print(f"ERROR: {exc}", file=sys.stderr)
            return EXIT_STATE
        try:
            plan = make_plan(args.side, delta, state, anchor_state)
        except StateValidationError as exc:
            print(f"ERROR: {exc}", file=sys.stderr)
            return EXIT_STATE
        print_preview(
            plan,
            names,
            "--state-json (offline preview)",
            args.execute,
            args.safety_acknowledged,
            args.experimental_policy_approved,
            active_unselected_hold_approved,
        )
        print("Dry run only: --state-json is offline-only; no ROS import or command was published.")
        return EXIT_OK

    os.environ["ROS_DOMAIN_ID"] = str(args.domain_id).strip()
    try:
        rclpy, node_type, string_type = _ros_imports()
    except RuntimeError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return EXIT_ROS_UNAVAILABLE

    node = None
    try:
        rclpy.init(args=None)
        node = node_type("move_end_effector_relative_example")
        state = _read_state(rclpy, node, string_type, names["state_topic"], args.state_timeout)
        plan = make_plan(args.side, delta, state, anchor_state)
        print_preview(
            plan,
            names,
            f"realtime {names['state_topic']}",
            args.execute,
            args.safety_acknowledged,
            args.experimental_policy_approved,
            active_unselected_hold_approved,
        )
        if not args.execute:
            print("Dry run only: no EEF command was published.")
            return EXIT_OK
        try:
            mode_service = names[f"{plan.side}_mode_service"]
            _set_position_mode(rclpy, node, mode_service)
            print(f"Position mode confirmed via {mode_service}")
        except Exception as exc:
            print(f"ERROR: position-mode preflight failed: {exc}", file=sys.stderr)
            print("No EEF command was published (fail-closed).", file=sys.stderr)
            return EXIT_PUBLISH
        try:
            _publish_bounded(
                rclpy,
                node,
                string_type,
                names["move_topic"],
                _json_text(plan.payload),
            )
        except Exception as exc:
            print(f"ERROR: EEF command publish failed: {exc}", file=sys.stderr)
            print("No EEF command was published (fail-closed).", file=sys.stderr)
            return EXIT_PUBLISH
        print(
            f"Published one bounded target ({PUBLISH_REPEATS}x, idempotent) to "
            f"{names['move_topic']}: {_json_text(plan.payload)}"
        )
        reached, position_error, rotation_error = _wait_for_feedback(
            rclpy, node, string_type, names["state_topic"], plan
        )
        if reached:
            print(
                f"Feedback: target reached (position error {position_error * 1000:.1f} mm, "
                f"rotation error {rotation_error:.2f} deg)."
            )
            return EXIT_OK
        print(
            f"ERROR: no convergence within {FEEDBACK_TIMEOUT_SECONDS:g} s "
            f"(position error {position_error * 1000:.1f} mm, rotation error "
            f"{rotation_error:.2f} deg); the arm may not have executed the command.",
            file=sys.stderr,
        )
        print(
            "Hint: the arm daemon silently drops targets it rejects. Check its log "
            "on the robot for the exact reason, e.g.: journalctl --user --since '-5 min' "
            "| grep -E 'self-collide|change too much' "
            "(self-collision prediction and per-message joint-change limits are the "
            "two known reject gates in arm_task_control_base.move_arm_eef_in_robot_frame).",
            file=sys.stderr,
        )
        return EXIT_FEEDBACK
    except KeyboardInterrupt:
        print("Interrupted; no further EEF commands were published.", file=sys.stderr)
        return EXIT_INTERRUPTED
    except (StateValidationError, TimeoutError) as exc:
        print(f"ERROR: current EEF state unavailable: {exc}", file=sys.stderr)
        return EXIT_STATE
    except Exception as exc:  # 让 CLI 失败信息可操作，不打印 traceback
        print(f"ERROR: EEF relative example failed: {exc}", file=sys.stderr)
        return EXIT_STATE
    finally:
        if node is not None:
            try:
                node.destroy_node()
            except Exception:
                pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    _validate_args(args, parser)
    delta = _parse_delta(args, parser)
    names = interface_names(str(args.domain_id).strip(), str(args.robot_id).strip())
    return run(args, delta, names)


if __name__ == "__main__":
    raise SystemExit(main())
