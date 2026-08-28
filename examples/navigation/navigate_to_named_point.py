#!/usr/bin/env python3
"""导航到 AutoLife 机器人上一个命名的预置点位。

除非同时提供 ``--execute`` 和 ``--safety-acknowledged``，本脚本有意
保持零执行动作。它会先查询当前激活的地图，因此写错的点位名不可能
被发给导航节点。ROS 的 import 推迟到参数解析之后，让没装 ROS 的
开发机也能用 ``--help``。
"""

from __future__ import annotations

import argparse
import json
import math
import os
import re
import sys
import time
from typing import Any, Iterable, Mapping


EXIT_OK = 0
EXIT_ARGUMENT = 2
EXIT_ROS_UNAVAILABLE = 3
EXIT_SERVICE_UNAVAILABLE = 4
EXIT_MAP_ERROR = 5
EXIT_FEEDBACK_FAILURE = 6
EXIT_FEEDBACK_TIMEOUT = 7
EXIT_INTERRUPTED = 130

GET_MAPS_PAYLOAD = {"cmd": "get_maps"}
CANCEL_FLUSH_TIMEOUT = 0.2

# 机器人的 navigating_feedback 发布不带关键词的进度 JSON，例如
# {"navigation_time": ..., "estimated_time_remaining": 0.0,
#  "distance_remaining": 0.0, "number_of_recoveries": 0}。到达只能从
# 这些字段推断；下面的阈值防的是路径尚不存在时导航栈发出的全零字段。
# ARRIVAL_DISTANCE_EPS_M 必须大于 xy_goal_tolerance（nav2_params.yaml
# 里是 0.05）加一个代价地图栅格的路径量化误差（0.05）：Nav2 按欧氏
# 距离判定成功，而 distance_remaining 是沿路径的，目标检查器通过后
# 可能冻结在约 0.07。
ARRIVAL_DISTANCE_EPS_M = 0.15
ARRIVAL_ETA_EPS_S = 0.1
ARRIVAL_MIN_NAVIGATION_TIME_S = 1.0
ARRIVAL_MIN_PROGRESS_DISTANCE_M = 0.2
ARRIVAL_STABLE_FRAMES = 3
ARRIVAL_QUIET_SECONDS = 2.0

# 首选到达信号：/robot_navigation_*/is_navigating（std_msgs/Bool）
# 以约 1 Hz 持续发布，只有整个 Nav2 NavigateToPose action 结束
# （含最终偏航对齐）后才变 false（2026-08-20 现场验证；上面的进度帧
# 推断保留为兜底，因为其 distance_remaining 可能冻结在阈值之外而
# 漏判到达）。
IS_NAVIGATING_FALSE_STABLE_FRAMES = 2


def _default_identifier(name: str, fallback: str) -> str:
    """返回非空的环境变量值，否则返回已验证的目标默认值。"""

    value = os.environ.get(name, fallback).strip()
    return value or fallback


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Query the active map and preview navigation to a prepared named "
            "position. Physical publication requires both explicit gates."
        )
    )
    parser.add_argument("point", help="exact prepared-position name in the active map")
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
        "--service-timeout",
        type=float,
        default=5.0,
        metavar="SECONDS",
        help="bounded wait for the map service and response (default: 5)",
    )
    parser.add_argument(
        "--feedback-timeout",
        type=float,
        default=30.0,
        metavar="SECONDS",
        help="bounded wait for terminal navigation feedback (default: 30)",
    )
    parser.add_argument(
        "--execute",
        action="store_true",
        help="enable publication of the navigation command (first execution gate)",
    )
    parser.add_argument(
        "--safety-acknowledged",
        action="store_true",
        help=(
            "confirm the area is clear, an operator has the emergency stop, and "
            "the target is approved (second execution gate)"
        ),
    )
    return parser


def _validate_args(args: argparse.Namespace, parser: argparse.ArgumentParser) -> None:
    if args.service_timeout <= 0:
        parser.error("--service-timeout must be greater than zero")
    if args.feedback_timeout <= 0:
        parser.error("--feedback-timeout must be greater than zero")
    domain_id = str(args.domain_id).strip()
    if not domain_id:
        parser.error("--domain-id must not be empty")
    try:
        if int(domain_id) < 0:
            raise ValueError
    except ValueError:
        parser.error("--domain-id must be a non-negative integer")
    if not str(args.robot_id).strip():
        parser.error("--robot-id must not be empty")


def topic_names(domain_id: str, robot_id: str) -> dict[str, str]:
    """构建已确认的机器人 API 所用的话题/服务名。"""

    suffix = f"{domain_id}_{robot_id}"
    return {
        "map_service": f"/robot_map_{suffix}/map_command",
        "go_topic": f"/robot_navigation_{suffix}/go",
        "cancel_topic": f"/robot_navigation_{suffix}/cancel",
        "feedback_topic": f"/robot_navigation_{suffix}/navigating_feedback",
        "is_navigating_topic": f"/robot_navigation_{suffix}/is_navigating",
    }


def _ros_imports() -> tuple[Any, Any, Any, Any, Any, Any]:
    """只在命令确实需要与 ROS 通信时才 import 它。"""

    try:
        import rclpy
        from autolife_robot_srvs.srv import SetString
        from rclpy.node import Node
        from std_msgs.msg import Bool, Empty, String
    except ImportError as exc:  # pragma: no cover - depends on target environment
        raise RuntimeError(
            "ROS 2 runtime is unavailable; activate the target robot environment "
            "before running a map query (python --help still works locally)."
        ) from exc
    return rclpy, Node, SetString, String, Empty, Bool


def _message_to_python(value: Any) -> Any:
    """把 ROS 响应/消息或普通 JSON 型值转成 Python 值。"""

    if value is None or isinstance(value, (str, int, float, bool)):
        return value
    if isinstance(value, Mapping):
        return {str(key): _message_to_python(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_message_to_python(item) for item in value]

    field_names: Iterable[str] = ()
    get_fields = getattr(value, "get_fields_and_field_types", None)
    if callable(get_fields):
        try:
            field_names = get_fields().keys()
        except Exception:
            field_names = ()
    if not field_names:
        slots = getattr(value, "__slots__", ())
        field_names = (str(slot).lstrip("_") for slot in slots)
    converted: dict[str, Any] = {}
    for field_name in field_names:
        if hasattr(value, field_name):
            converted[field_name] = _message_to_python(getattr(value, field_name))
    return converted if converted else value


def _decode_json_text(value: Any) -> Any:
    if isinstance(value, str):
        try:
            return json.loads(value)
        except json.JSONDecodeError:
            return value
    return value


def response_payload(response: Any) -> Any:
    """从 SetString 响应中提取 get_maps 的 JSON/对象载荷。"""

    converted = _message_to_python(response)
    if isinstance(converted, Mapping):
        if "active_map" in converted and "maps" in converted:
            return converted
        # 机上安装的服务通常把 JSON 响应放在 data 字段里。
        for field_name in ("data", "message", "result", "response"):
            if field_name in converted:
                decoded = _decode_json_text(converted[field_name])
                if isinstance(decoded, Mapping):
                    return decoded
    decoded = _decode_json_text(converted)
    return decoded


def _map_name(map_value: Any) -> str | None:
    if isinstance(map_value, Mapping):
        value = map_value.get("name")
        return str(value) if value is not None else None
    return None


def _active_map_payload(payload: Any) -> tuple[str | None, Any]:
    if not isinstance(payload, Mapping):
        return None, None
    active_map = payload.get("active_map")
    if isinstance(active_map, Mapping):
        active_name = _map_name(active_map)
        return active_name, active_map
    active_name = str(active_map) if active_map is not None else None
    maps = payload.get("maps")
    if isinstance(maps, Mapping):
        if active_name in maps:
            return active_name, maps[active_name]
        # 有些响应用任意 id 作键存放地图对象。
        for key, value in maps.items():
            if _map_name(value) == active_name:
                return active_name, value
    elif isinstance(maps, list):
        for value in maps:
            if _map_name(value) == active_name:
                return active_name, value
    return active_name, None


def prepared_position_names(payload: Any) -> tuple[str | None, list[str]]:
    """返回激活地图名和精确的预置点位名列表。"""

    active_name, active_map = _active_map_payload(payload)
    if not isinstance(active_map, Mapping):
        return active_name, []
    positions = active_map.get("prepared_positions", [])
    names: list[str] = []
    if isinstance(positions, Mapping):
        positions = positions.values()
    if isinstance(positions, (list, tuple)) or not isinstance(positions, (str, bytes)):
        try:
            for position in positions:
                if isinstance(position, Mapping) and position.get("name") is not None:
                    names.append(str(position["name"]))
        except TypeError:
            pass
    return active_name, names


def feedback_status(raw: str) -> dict[str, Any]:
    """解析常见的终态关键词，同时保留未经修改的原始反馈。"""

    parsed: Any = raw
    try:
        parsed = json.loads(raw)
    except json.JSONDecodeError:
        pass
    # "error_code=0" 表示"无错误"（观测到的终态帧：'Navigation
    # finished ...: SUCCEEDED, error_code=0'）；关键词匹配前先去掉它，
    # 免得其中的 "error" 子串把成功帧误判成失败。
    lower = re.sub(r"error[_ ]?code\s*[:=]\s*0\b", "", raw.lower())
    status = "progress"
    if any(token in lower for token in ("cancel", "canceled", "cancelled")):
        status = "canceled"
    elif any(token in lower for token in ("fail", "error", "abort", "reject")):
        status = "failure"
    elif any(
        token in lower
        for token in ("success", "succeed", "arrived", "complete", "completed", "finished", "done")
    ):
        status = "success"
    return {"status": status, "value": parsed}


def _finite_float(value: Any) -> float | None:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    number = float(value)
    return number if math.isfinite(number) else None


class ProgressArrivalDetector:
    """从不带关键词的进度反馈帧中推断到达。

    只有当本次运行早先观测到过正的目标距离、而现在距离与时间估计都
    归零时，该帧才算到达候选。要求先出现过正距离，可以排除路径规划
    尚未产生路线时发布的全零帧。
    """

    def __init__(self) -> None:
        self.last_frame_candidate = False
        self._saw_progress_distance = False
        self._stable_frames = 0

    def _frame_candidate(self, value: Any) -> bool:
        if not isinstance(value, Mapping):
            return False
        distance = _finite_float(value.get("distance_remaining"))
        eta = _finite_float(value.get("estimated_time_remaining"))
        navigation_time = _finite_float(value.get("navigation_time"))
        if distance is None or eta is None or navigation_time is None:
            return False
        if distance >= ARRIVAL_MIN_PROGRESS_DISTANCE_M:
            self._saw_progress_distance = True
        return (
            self._saw_progress_distance
            and navigation_time >= ARRIVAL_MIN_NAVIGATION_TIME_S
            and distance <= ARRIVAL_DISTANCE_EPS_M
            and eta <= ARRIVAL_ETA_EPS_S
        )

    def update(self, value: Any) -> bool:
        """记录一帧反馈；到达状态稳定后返回 True。"""

        self.last_frame_candidate = self._frame_candidate(value)
        self._stable_frames = self._stable_frames + 1 if self.last_frame_candidate else 0
        return self._stable_frames >= ARRIVAL_STABLE_FRAMES


class IsNavigatingArrivalDetector:
    """从周期性的 ``is_navigating`` Bool 流中推断到达。

    到达即观测到 true -> false 的转换：导航栈必须先为本目标报告过
    正在导航，随后连续 ``IS_NAVIGATING_FALSE_STABLE_FRAMES`` 帧再次
    报告空闲。要求先出现过 true，可以排除目标被接受前发布的空闲
    false 帧；false 连击数则防途中单帧抖动。
    """

    def __init__(self) -> None:
        self.last_active: bool | None = None
        self._saw_navigating = False
        self._false_streak = 0

    def update(self, active: bool) -> bool:
        """记录一帧；到达状态稳定后返回 True。"""

        self.last_active = active
        if active:
            self._saw_navigating = True
            self._false_streak = 0
            return False
        if not self._saw_navigating:
            return False
        self._false_streak += 1
        return self._false_streak >= IS_NAVIGATING_FALSE_STABLE_FRAMES


def print_preview(
    point: str,
    active_map: str | None,
    known_points: list[str],
    names: Mapping[str, str],
    execute: bool,
    safety_acknowledged: bool,
    validation: str,
) -> None:
    payload = point
    print("Navigation preview")
    print(f"  active_map: {active_map or '<unknown>'}")
    print(f"  requested_point: {point!r}")
    print(f"  validation: {validation}")
    print(f"  known_points: {', '.join(known_points) if known_points else '<none>'}")
    print(f"  go_topic: {names['go_topic']}")
    print(f"  cancel_topic: {names['cancel_topic']}")
    print(f"  feedback_topic: {names['feedback_topic']}")
    print(f"  is_navigating_topic: {names['is_navigating_topic']}")
    print(f"  std_msgs/String payload: {payload!r}")
    print(f"  execution_gate --execute: {'present' if execute else 'absent'}")
    print(
        "  safety_gate --safety-acknowledged: "
        f"{'present' if safety_acknowledged else 'absent'}"
    )


def _call_service(rclpy: Any, node: Any, client: Any, request: Any, timeout: float) -> Any:
    future = client.call_async(request)
    deadline = time.monotonic() + timeout
    while rclpy.ok() and not future.done():
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            break
        rclpy.spin_once(node, timeout_sec=min(0.1, remaining))
    if not future.done():
        raise TimeoutError("timed out waiting for get_maps response")
    return future.result()


def _wait_for_feedback(rclpy: Any, node: Any, state: dict[str, Any], timeout: float) -> str:
    deadline = time.monotonic() + timeout
    while rclpy.ok():
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            return "timeout"
        rclpy.spin_once(node, timeout_sec=min(0.1, remaining))
        terminal = state.get("terminal")
        if terminal:
            return terminal
        last_frame_monotonic = state.get("last_frame_monotonic")
        if (
            state.get("arrival_candidate")
            and last_frame_monotonic is not None
            and time.monotonic() - last_frame_monotonic >= ARRIVAL_QUIET_SECONDS
        ):
            print(
                "Arrival inferred: feedback went quiet with distance_remaining "
                "at zero; treating as success."
            )
            return "success"
    return "interrupted"


def _publish_cancel_and_flush(
    rclpy: Any,
    node: Any,
    publisher: Any,
    empty_message_type: Any,
    timeout: float = CANCEL_FLUSH_TIMEOUT,
) -> None:
    """发布取消命令，然后给 ROS 执行器一段短的有界冲刷时间。"""

    publisher.publish(empty_message_type())
    deadline = time.monotonic() + timeout
    while rclpy.ok():
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            return
        rclpy.spin_once(node, timeout_sec=min(0.05, remaining))


def run(args: argparse.Namespace) -> int:
    domain_id = str(args.domain_id).strip()
    # rclpy 初始化 DDS 上下文时会读 ROS_DOMAIN_ID。保持进程上下文与
    # 操作员选择的话题后缀一致。
    os.environ["ROS_DOMAIN_ID"] = domain_id
    names = topic_names(domain_id, str(args.robot_id).strip())
    try:
        rclpy, Node, SetString, String, Empty, Bool = _ros_imports()
    except RuntimeError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return EXIT_ROS_UNAVAILABLE

    node = None
    executed = False
    cancel_publisher = None
    try:
        rclpy.init(args=None)
        node = Node("navigate_to_named_point_example")
        map_client = node.create_client(SetString, names["map_service"])
        if not map_client.wait_for_service(timeout_sec=args.service_timeout):
            print(f"ERROR: map service unavailable: {names['map_service']}", file=sys.stderr)
            print_preview(
                args.point,
                None,
                [],
                names,
                args.execute,
                args.safety_acknowledged,
                "service unavailable",
            )
            return EXIT_SERVICE_UNAVAILABLE

        request = SetString.Request()
        if not hasattr(request, "data"):
            print("ERROR: SetString.Request has no confirmed 'data' field", file=sys.stderr)
            print_preview(
                args.point,
                None,
                [],
                names,
                args.execute,
                args.safety_acknowledged,
                "unsupported SetString request",
            )
            return EXIT_MAP_ERROR
        request.data = json.dumps(GET_MAPS_PAYLOAD, separators=(",", ":"))
        try:
            response = _call_service(rclpy, node, map_client, request, args.service_timeout)
        except TimeoutError as exc:
            print(f"ERROR: {exc}", file=sys.stderr)
            print_preview(
                args.point,
                None,
                [],
                names,
                args.execute,
                args.safety_acknowledged,
                "map response timeout",
            )
            return EXIT_SERVICE_UNAVAILABLE
        except Exception as exc:
            print(f"ERROR: get_maps service call failed: {exc}", file=sys.stderr)
            print_preview(
                args.point,
                None,
                [],
                names,
                args.execute,
                args.safety_acknowledged,
                "get_maps service error",
            )
            return EXIT_SERVICE_UNAVAILABLE

        payload = response_payload(response)
        active_map, known_points = prepared_position_names(payload)
        if not isinstance(payload, Mapping) or active_map is None:
            print("ERROR: get_maps response lacks active_map/maps/prepared_positions", file=sys.stderr)
            print_preview(
                args.point,
                active_map,
                known_points,
                names,
                args.execute,
                args.safety_acknowledged,
                "invalid get_maps response",
            )
            return EXIT_MAP_ERROR

        if args.point not in known_points:
            validation = "rejected: point is not in active-map prepared_positions"
        else:
            validation = "valid"
        print_preview(
            args.point,
            active_map,
            known_points,
            names,
            args.execute,
            args.safety_acknowledged,
            validation,
        )
        if args.point not in known_points:
            print(
                f"ERROR: unknown point {args.point!r}; no navigation command was published",
                file=sys.stderr,
            )
            return EXIT_MAP_ERROR
        if not (args.execute and args.safety_acknowledged):
            print("Dry run only: both execution gates are required; no command was published.")
            return EXIT_OK

        feedback_state: dict[str, Any] = {
            "terminal": None,
            "last_raw": None,
            "last_frame_monotonic": None,
            "arrival_candidate": False,
        }
        arrival_detector = ProgressArrivalDetector()

        def feedback_callback(message: Any) -> None:
            raw = str(message.data)
            parsed = feedback_status(raw)
            print(f"Feedback raw: {raw}")
            print(f"Feedback parsed: {json.dumps(parsed, ensure_ascii=False, default=str)}")
            feedback_state["last_raw"] = raw
            feedback_state["last_frame_monotonic"] = time.monotonic()
            if parsed["status"] in {"success", "failure", "canceled"}:
                feedback_state["terminal"] = parsed["status"]
            elif arrival_detector.update(parsed["value"]):
                print(
                    "Arrival detected: distance_remaining stable at zero; "
                    "treating as success."
                )
                feedback_state["terminal"] = "success"
            feedback_state["arrival_candidate"] = arrival_detector.last_frame_candidate

        nav_state_detector = IsNavigatingArrivalDetector()

        def is_navigating_callback(message: Any) -> None:
            active = bool(message.data)
            transitioned = nav_state_detector.last_active is not active
            arrived = nav_state_detector.update(active)
            if transitioned:
                print(f"is_navigating: {active}")
            if arrived and feedback_state["terminal"] is None:
                print(
                    "Arrival detected: is_navigating reported true, then settled "
                    "false; treating as success."
                )
                feedback_state["terminal"] = "success"

        node.create_subscription(String, names["feedback_topic"], feedback_callback, 10)
        node.create_subscription(
            Bool, names["is_navigating_topic"], is_navigating_callback, 10
        )
        go_publisher = node.create_publisher(String, names["go_topic"], 10)
        cancel_publisher = node.create_publisher(Empty, names["cancel_topic"], 10)
        command = String()
        command.data = args.point
        go_publisher.publish(command)
        executed = True
        print(f"Published std_msgs/String to {names['go_topic']}: {args.point!r}")

        try:
            result = _wait_for_feedback(rclpy, node, feedback_state, args.feedback_timeout)
        except KeyboardInterrupt:
            result = "interrupted"
        if result == "timeout":
            print("ERROR: feedback timeout; publishing cancellation", file=sys.stderr)
            _publish_cancel_and_flush(rclpy, node, cancel_publisher, Empty)
            return EXIT_FEEDBACK_TIMEOUT
        if result == "interrupted":
            print("Interrupted; publishing cancellation", file=sys.stderr)
            _publish_cancel_and_flush(rclpy, node, cancel_publisher, Empty)
            return EXIT_INTERRUPTED
        if result == "failure":
            print("ERROR: navigation reported failure", file=sys.stderr)
            return EXIT_FEEDBACK_FAILURE
        print(f"Navigation feedback reached terminal status: {result}")
        # 此处"不要"取消：成功通常在 Nav2 还在收尾最终偏航对齐
        # （force_final_orientation_goal_dist）时就已从进度帧推断出来；
        # 2026-08-20 在这个时点取消曾导致基座背对 P3 篮筐。残余的
        # 目标跟踪由调用方在基座稳定后取消（见 flow 的 do_nav），
        # 保证手臂环节从静止的基座开始。
        return EXIT_OK
    except KeyboardInterrupt:
        if executed and cancel_publisher is not None:
            _publish_cancel_and_flush(rclpy, node, cancel_publisher, Empty)
            print("Interrupted; cancellation published", file=sys.stderr)
        return EXIT_INTERRUPTED
    except Exception as exc:  # 让 CLI 失败信息可操作，不打印 traceback
        print(f"ERROR: navigation example failed: {exc}", file=sys.stderr)
        return EXIT_MAP_ERROR
    finally:
        if node is not None:
            node.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    _validate_args(args, parser)
    return run(args)


if __name__ == "__main__":
    raise SystemExit(main())
