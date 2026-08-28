#!/usr/bin/env python3
"""预览或发布两个夹爪的基本位置目标。

本命令刻意保持小而直接：使用已确认的 ``std_msgs/msg/String`` 位置
话题和两个电机控制节点上的标准 ROS 参数服务。不实现力矩控制、抓取
选择、反馈或自动物理序列。默认 preview；实机发布必须同时给
``--execute`` 和 ``--safety-acknowledged``。

ROS 的 import 推迟到请求执行时才发生。因此没装 ROS 2 或机器人
Python 包的开发机也能用 ``--help`` 和普通 preview。
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
from dataclasses import dataclass
from typing import Any, Mapping


EXIT_OK = 0
EXIT_ARGUMENT = 2
EXIT_ROS_UNAVAILABLE = 3
EXIT_MODE_SERVICE = 4
EXIT_PUBLISH = 5
EXIT_INTERRUPTED = 130

OPEN_POSITION = 10.0
CLOSE_POSITION = 360.0
MIN_POSITION = OPEN_POSITION
MAX_POSITION = CLOSE_POSITION

# 把绝对目标重复发几次可以扛住瞬态的 DDS 连接问题，
# 而短小的固定窗口避免了持续驱动。
PUBLISH_REPEATS = 3
PUBLISH_INTERVAL_SECONDS = 0.05
SERVICE_TIMEOUT_SECONDS = 5.0
PUBLISH_SUBSCRIBER_TIMEOUT_SECONDS = 5.0


@dataclass(frozen=True)
class GripperPlan:
    """preview 与执行共用的、已解析并验证的指令数据。"""

    side: str
    action: str
    target_position: float
    payload: dict[str, list[float]]


def _default_identifier(name: str, fallback: str) -> str:
    """返回非空的环境变量值，否则返回记录在案的目标默认值。"""

    value = os.environ.get(name, fallback).strip()
    return value or fallback


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Preview a basic left/right gripper position target. Physical "
            "publication requires --execute and --safety-acknowledged."
        )
    )
    parser.add_argument(
        "--side",
        choices=("left", "right", "both"),
        required=True,
        help="gripper side to command (required; no implicit side)",
    )
    value_group = parser.add_mutually_exclusive_group()
    value_group.add_argument(
        "--action",
        choices=("open", "close"),
        help="named target: open (10) or close (360 controller position units)",
    )
    value_group.add_argument(
        "--target",
        dest="target_option",
        metavar="POSITION",
        help="numeric controller position in the inclusive range 10..360",
    )
    parser.add_argument(
        "value",
        nargs="?",
        metavar="VALUE",
        help="optional positional alias for open, close, or a numeric target",
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
        help="enable physical publication (first execution gate)",
    )
    parser.add_argument(
        "--safety-acknowledged",
        action="store_true",
        help=(
            "confirm the workspace is clear, an operator has the emergency "
            "stop, and this target is approved (second execution gate)"
        ),
    )
    return parser


def _parse_position(raw_value: str) -> tuple[str, float]:
    """解析 open/close 或有限数值位置，并强制其范围。"""

    raw = str(raw_value).strip()
    lowered = raw.lower()
    if lowered == "open":
        return "open", OPEN_POSITION
    if lowered == "close":
        return "close", CLOSE_POSITION
    try:
        value = float(raw)
    except (TypeError, ValueError) as exc:
        raise ValueError("target must be 'open', 'close', or a number from 10 to 360") from exc
    if not math.isfinite(value):
        raise ValueError("target must be a finite controller position from 10 to 360")
    if not MIN_POSITION <= value <= MAX_POSITION:
        raise ValueError(
            f"target must be in the inclusive range {MIN_POSITION:g}..{MAX_POSITION:g}"
        )
    return "numeric", float(value)


def _resolve_plan(args: argparse.Namespace, parser: argparse.ArgumentParser) -> GripperPlan:
    supplied = []
    if args.action is not None:
        supplied.append(("action", args.action))
    if args.target_option is not None:
        supplied.append(("--target", args.target_option))
    if args.value is not None:
        supplied.append(("positional value", args.value))
    if len(supplied) != 1:
        parser.error("provide exactly one of --action, --target, or positional VALUE")

    source, raw_value = supplied[0]
    try:
        action, target_position = _parse_position(str(raw_value))
    except ValueError as exc:
        parser.error(f"invalid {source}: {exc}")

    payload: dict[str, list[float]] = {}
    target = [target_position]
    if args.side in ("left", "both"):
        payload["left_gripper_target_joints_position"] = target.copy()
    if args.side in ("right", "both"):
        payload["right_gripper_target_joints_position"] = target.copy()
    return GripperPlan(args.side, action, target_position, payload)


def _validate_args(args: argparse.Namespace, parser: argparse.ArgumentParser) -> None:
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


def interface_names(domain_id: str, robot_id: str) -> dict[str, str]:
    """构建已确认的话题名和参数服务名。"""

    suffix = f"{domain_id}_{robot_id}"
    return {
        "position_topic": f"/topic_arm_gripper_target_joints_position_{suffix}",
        "left_mode_service": f"/node_mod_motor_left_arm_control_{suffix}/set_parameters",
        "right_mode_service": f"/node_mod_motor_right_arm_control_{suffix}/set_parameters",
    }


def _payload_text(payload: Mapping[str, list[float]]) -> str:
    return json.dumps(payload, separators=(",", ":"), ensure_ascii=False)


def print_preview(
    plan: GripperPlan,
    names: Mapping[str, str],
    execute: bool,
    safety_acknowledged: bool,
) -> None:
    print("Gripper command preview")
    print(f"  side: {plan.side}")
    print(f"  action: {plan.action}")
    print(f"  target_position: {plan.target_position:.6g}")
    print(f"  position_topic: {names['position_topic']}")
    print(f"  left_mode_service: {names['left_mode_service']}")
    print(f"  right_mode_service: {names['right_mode_service']}")
    print(f"  std_msgs/msg/String payload: {_payload_text(plan.payload)}")
    omitted = "right" if plan.side == "left" else "left" if plan.side == "right" else "none"
    print(f"  unselected_side: {omitted} (omitted from payload)")
    print(f"  execution_gate --execute: {'present' if execute else 'absent'}")
    print(
        "  safety_gate --safety-acknowledged: "
        f"{'present' if safety_acknowledged else 'absent'}"
    )


def _ros_imports() -> tuple[Any, Any, Any, Any, Any, Any, Any]:
    """只在门禁通过的执行路径上才 import ROS。"""

    try:
        import rclpy
        from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
        from rcl_interfaces.srv import SetParameters
        from rclpy.node import Node
        from std_msgs.msg import String
    except ImportError as exc:  # pragma: no cover - depends on target environment
        raise RuntimeError(
            "ROS 2 runtime is unavailable; activate the target robot environment "
            "before using --execute (previews do not require ROS)."
        ) from exc
    return rclpy, Node, SetParameters, Parameter, ParameterValue, ParameterType, String


def _call_service(rclpy: Any, node: Any, client: Any, request: Any, timeout: float) -> Any:
    future = client.call_async(request)
    deadline = time.monotonic() + timeout
    while rclpy.ok() and not future.done():
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            break
        rclpy.spin_once(node, timeout_sec=min(0.1, remaining))
    if not future.done():
        raise TimeoutError("timed out waiting for SetParameters response")
    return future.result()


def _make_mode_request(
    set_parameters_type: Any,
    parameter_type: Any,
    parameter_value_type: Any,
    parameter_type_constants: Any,
) -> Any:
    request = set_parameters_type.Request()
    parameter = parameter_type()
    parameter.name = "param_arm_gripper_joint_control_mode"
    value = parameter_value_type()
    string_type = getattr(parameter_type_constants, "PARAMETER_STRING", 4)
    value.type = int(string_type)
    value.string_value = "position"
    parameter.value = value
    request.parameters = [parameter]
    return request


def _mode_result(response: Any) -> tuple[bool, str]:
    results = getattr(response, "results", None)
    if not results:
        return False, "SetParameters response contained no result"
    result = results[0]
    successful = bool(getattr(result, "successful", False))
    reason = str(getattr(result, "reason", ""))
    return successful, reason


def _set_position_mode(
    rclpy: Any,
    node: Any,
    set_parameters_type: Any,
    parameter_type: Any,
    parameter_value_type: Any,
    parameter_type_constants: Any,
    service_name: str,
) -> tuple[bool, str]:
    client = node.create_client(set_parameters_type, service_name)
    if not client.wait_for_service(timeout_sec=SERVICE_TIMEOUT_SECONDS):
        return False, f"service unavailable: {service_name}"
    request = _make_mode_request(
        set_parameters_type,
        parameter_type,
        parameter_value_type,
        parameter_type_constants,
    )
    try:
        response = _call_service(rclpy, node, client, request, SERVICE_TIMEOUT_SECONDS)
    except TimeoutError as exc:
        return False, str(exc)
    except Exception as exc:  # 让 CLI 失败信息可操作，不打印 traceback
        return False, f"SetParameters call failed: {exc}"
    successful, reason = _mode_result(response)
    if not successful:
        return False, reason or "SetParameters reported failure"
    return True, reason or "accepted"


def _publish_target(rclpy: Any, node: Any, string_type: Any, topic: str, payload: str) -> None:
    publisher = node.create_publisher(string_type, topic, 10)
    deadline = time.monotonic() + PUBLISH_SUBSCRIBER_TIMEOUT_SECONDS
    while rclpy.ok():
        try:
            subscriber_count = int(publisher.get_subscription_count())
        except Exception as exc:
            raise RuntimeError(f"could not inspect subscribers for {topic}: {exc}") from exc
        if subscriber_count > 0:
            break
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise TimeoutError(
                f"no subscriber appeared on {topic} within "
                f"{PUBLISH_SUBSCRIBER_TIMEOUT_SECONDS:g} seconds"
            )
        rclpy.spin_once(node, timeout_sec=min(0.1, remaining))
    else:
        raise RuntimeError("ROS context stopped before a gripper topic subscriber appeared")

    message = string_type()
    message.data = payload
    for index in range(PUBLISH_REPEATS):
        publisher.publish(message)
        if index + 1 < PUBLISH_REPEATS:
            time.sleep(PUBLISH_INTERVAL_SECONDS)


def run(
    plan: GripperPlan,
    names: Mapping[str, str],
    execute: bool,
    domain_id: str,
) -> int:
    """打印 preview；门禁通过时设置模式并发布位置目标。"""

    if not execute:
        print("Dry run only: no ROS import and no gripper command was published.")
        return EXIT_OK

    # rclpy 创建 DDS 上下文时会读 ROS_DOMAIN_ID。
    os.environ["ROS_DOMAIN_ID"] = domain_id
    try:
        (
            rclpy,
            node_type,
            set_parameters_type,
            parameter_type,
            parameter_value_type,
            parameter_type_constants,
            string_type,
        ) = _ros_imports()
    except RuntimeError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return EXIT_ROS_UNAVAILABLE

    node = None
    try:
        rclpy.init(args=None)
        node = node_type("control_gripper_example")
        sides = ("left", "right") if plan.side == "both" else (plan.side,)
        for side in sides:
            service_name = names[f"{side}_mode_service"]
            ok, reason = _set_position_mode(
                rclpy,
                node,
                set_parameters_type,
                parameter_type,
                parameter_value_type,
                parameter_type_constants,
                service_name,
            )
            if not ok:
                print(
                    f"ERROR: failed to set position mode on {service_name}: {reason}",
                    file=sys.stderr,
                )
                print("No gripper command was published (fail-closed).", file=sys.stderr)
                return EXIT_MODE_SERVICE
            print(f"Position mode accepted by {side} motor controller: {reason}")

        payload = _payload_text(plan.payload)
        try:
            _publish_target(rclpy, node, string_type, names["position_topic"], payload)
        except Exception as exc:
            print(f"ERROR: gripper position publish failed: {exc}", file=sys.stderr)
            return EXIT_PUBLISH
        print(
            f"Published std_msgs/msg/String to {names['position_topic']} "
            f"({PUBLISH_REPEATS} bounded publishes): {payload}"
        )
        return EXIT_OK
    except KeyboardInterrupt:
        print("Interrupted; no further gripper commands were published.", file=sys.stderr)
        return EXIT_INTERRUPTED
    except Exception as exc:  # 让 CLI 失败信息可操作，不打印 traceback
        print(f"ERROR: gripper example failed: {exc}", file=sys.stderr)
        return EXIT_PUBLISH
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
    plan = _resolve_plan(args, parser)
    names = interface_names(str(args.domain_id).strip(), str(args.robot_id).strip())
    print_preview(plan, names, args.execute, args.safety_acknowledged)
    return run(plan, names, args.execute, str(args.domain_id).strip())


if __name__ == "__main__":
    raise SystemExit(main())
