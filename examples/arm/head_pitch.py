#!/usr/bin/env python3
"""预览或执行一次有界的颈部俯仰运动（仅头部上下）。

向手臂服务的全身目标关节位置话题发布一个绝对颈部目标，载荷只设
``neck_target_joints_position`` 一个键，因此腿、腰和双臂永远不会
被指令。roll 和 yaw 目标钉在当前上报的颈部角度上；只有 pitch 变化。

安全模型（与 ``play_zero_position.py`` 一致）：

- 默认 preview；真实运动必须同时给 ``--execute`` 和
  ``--safety-acknowledged``。
- pitch 目标必须落在 URDF ``Joint_Neck_Pitch`` 限位内的保守区间，
  且单次调用的变化量有上限。
- 发布任何东西之前，必须能从关节状态反馈话题新鲜读到当前颈部
  角度（否则 fail-closed）。
- 发布后轮询反馈话题并报告稳定后的角度；操作员仍须目视运动直到
  停止。

服务侧接口（现场只读验证，2026-08-13）：

- 话题 ``/topic_arm_whole_body_target_joints_position_{DOMAIN}_{ROBOT}``
  （``std_msgs/String`` JSON），由
  ``AutolifeRobotArm/src/autolife_robot_arm/arm_joint_control_base.py``
  （``_sub_whole_body_target_joints_position``）消费；缺席的键不会被触碰。
- 角度单位是度；颈部顺序为 ``[roll, pitch, yaw]``；pitch 正值是抬头
  （``arm_vr_cmd_handler.py`` 的 ``HEAD_PITCH_UP``）。
- URDF ``Joint_Neck_Pitch`` 限位：-45.0 到 +30.0 度；服务在此之上
  还有自己的死区和速度限制。
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time


EXIT_OK = 0
EXIT_UNEXPECTED = 1
EXIT_ARGS = 2
EXIT_ROS = 3
EXIT_NO_SUBSCRIBER = 4
EXIT_NO_FEEDBACK = 5

# URDF 限位内的保守指令区间，保证服务永远收不到贴着硬限位的目标：
# Joint_Neck_Pitch -45.0..+30.0 度，Joint_Neck_Yaw -60.0..+60.0 度。
PITCH_MIN_DEG = -40.0
PITCH_MAX_DEG = 25.0
YAW_MIN_DEG = -55.0
YAW_MAX_DEG = 55.0
# 单次调用每个轴最多改变这么多（小步进）。
MAX_STEP_DEG = 20.0

FEEDBACK_TIMEOUT_SECONDS = 5.0
SUBSCRIBER_TIMEOUT_SECONDS = 5.0
SETTLE_SECONDS = 4.0
SETTLE_TOLERANCE_DEG = 1.0


def validate_axis_target(
    current_deg: float,
    target_deg: float,
    band_min_deg: float,
    band_max_deg: float,
    label: str,
) -> str | None:
    """目标可接受时返回 None，否则返回可读的拒绝原因。"""

    if not (isinstance(target_deg, (int, float)) and math.isfinite(target_deg)):
        return f"{label} target must be a finite number"
    if not (isinstance(current_deg, (int, float)) and math.isfinite(current_deg)):
        return f"current {label} must be a finite number"
    if not (band_min_deg <= target_deg <= band_max_deg):
        return (
            f"{label} target {target_deg:.2f} deg is outside the allowed band "
            f"[{band_min_deg}, {band_max_deg}]"
        )
    step = abs(target_deg - current_deg)
    if step > MAX_STEP_DEG:
        return (
            f"single-invocation {label} change {step:.2f} deg exceeds the {MAX_STEP_DEG} deg cap; "
            "move in smaller steps"
        )
    return None


def validate_pitch_target(current_pitch_deg: float, target_pitch_deg: float) -> str | None:
    return validate_axis_target(
        current_pitch_deg, target_pitch_deg, PITCH_MIN_DEG, PITCH_MAX_DEG, "pitch"
    )


def validate_yaw_target(current_yaw_deg: float, target_yaw_deg: float) -> str | None:
    return validate_axis_target(current_yaw_deg, target_yaw_deg, YAW_MIN_DEG, YAW_MAX_DEG, "yaw")


def build_payload(roll_deg: float, pitch_deg: float, yaw_deg: float) -> str:
    return json.dumps(
        {"neck_target_joints_position": [roll_deg, pitch_deg, yaw_deg]},
        separators=(",", ":"),
    )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Preview or execute a bounded neck-pitch move. Preview is the "
            "default and publishes nothing."
        )
    )
    parser.add_argument(
        "--pitch",
        type=float,
        default=None,
        metavar="DEG",
        help=(
            "absolute pitch target in degrees (positive = head up); "
            f"allowed band [{PITCH_MIN_DEG}, {PITCH_MAX_DEG}], "
            f"max change per invocation {MAX_STEP_DEG}"
        ),
    )
    parser.add_argument(
        "--yaw",
        type=float,
        default=None,
        metavar="DEG",
        help=(
            "absolute yaw target in degrees (positive = head left, negative = "
            f"head right); allowed band [{YAW_MIN_DEG}, {YAW_MAX_DEG}], "
            f"max change per invocation {MAX_STEP_DEG}"
        ),
    )
    parser.add_argument("--execute", action="store_true", help="publish the neck target")
    parser.add_argument(
        "--safety-acknowledged",
        action="store_true",
        help=(
            "confirm the head swing envelope is clear, an operator is "
            "supervising, and the physical emergency stop is reachable"
        ),
    )
    return parser


def _read_neck_feedback(rclpy: object, node: object, topic: str) -> list[float]:
    """从状态话题新鲜读取一次颈部 [roll, pitch, yaw]（度）。"""

    from std_msgs.msg import String

    holder: dict[str, list[float]] = {}

    def _on_status(msg: object) -> None:
        try:
            data = json.loads(msg.data)
            position = data["neck_joint_state"]["position"]
            if isinstance(position, list) and len(position) == 3:
                holder["neck"] = [float(value) for value in position]
        except Exception:
            pass

    subscription = node.create_subscription(String, topic, _on_status, 10)
    try:
        deadline = time.monotonic() + FEEDBACK_TIMEOUT_SECONDS
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if "neck" in holder:
                return holder["neck"]
        raise TimeoutError(
            f"no neck feedback on {topic} within {FEEDBACK_TIMEOUT_SECONDS:.1f}s"
        )
    finally:
        node.destroy_subscription(subscription)


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    if args.execute and not args.safety_acknowledged:
        parser.error("--execute requires --safety-acknowledged")
    if args.execute and args.pitch is None and args.yaw is None:
        parser.error("--execute requires --pitch and/or --yaw")

    domain_id = os.environ.get("ROS_DOMAIN_ID", "0").strip() or "0"
    robot_id = os.uname().nodename.rsplit("-", 1)[-1]  # 机器编号一律按主机名推导，不信环境变量
    command_topic = f"/topic_arm_whole_body_target_joints_position_{domain_id}_{robot_id}"
    status_topic = f"/topic_arm_whole_body_and_gripper_current_joints_status_{domain_id}_{robot_id}"

    try:
        import rclpy
        from std_msgs.msg import String
    except ImportError as exc:
        print(f"ERROR[{EXIT_ROS}]: ROS runtime unavailable: {exc}", file=sys.stderr)
        return EXIT_ROS

    node = None
    try:
        rclpy.init(args=None)
        node = rclpy.create_node("xr_head_pitch")

        try:
            roll, pitch, yaw = _read_neck_feedback(rclpy, node, status_topic)
        except TimeoutError as exc:
            print(f"ERROR[{EXIT_NO_FEEDBACK}]: {exc}", file=sys.stderr)
            return EXIT_NO_FEEDBACK

        print("Neck pitch tool")
        print(f"  command_topic: {command_topic}")
        print(f"  status_topic: {status_topic}")
        print(f"  current_neck_deg: roll={roll:.2f} pitch={pitch:.2f} yaw={yaw:.2f}")
        print(f"  allowed_pitch_band_deg: [{PITCH_MIN_DEG}, {PITCH_MAX_DEG}]")
        print(f"  max_step_deg: {MAX_STEP_DEG}")

        if args.pitch is None and args.yaw is None:
            print("Preview only: no --pitch/--yaw given, nothing to publish.")
            return EXIT_OK

        target_pitch = args.pitch if args.pitch is not None else pitch
        target_yaw = args.yaw if args.yaw is not None else yaw
        for reason in (
            validate_pitch_target(pitch, target_pitch),
            validate_yaw_target(yaw, target_yaw),
        ):
            if reason is not None:
                print(f"ERROR[{EXIT_ARGS}]: {reason}", file=sys.stderr)
                return EXIT_ARGS

        payload = build_payload(roll, target_pitch, target_yaw)
        print(f"  target_pitch_deg: {target_pitch:.2f}")
        print(f"  target_yaw_deg: {target_yaw:.2f}")
        print(f"  payload: {payload}")
        print(f"  execution_gate --execute: {'present' if args.execute else 'absent'}")
        print(
            "  safety_gate --safety-acknowledged: "
            f"{'present' if args.safety_acknowledged else 'absent'}"
        )
        if not args.execute:
            print("Dry run only: no neck command was published.")
            return EXIT_OK

        publisher = node.create_publisher(String, command_topic, 10)
        deadline = time.monotonic() + SUBSCRIBER_TIMEOUT_SECONDS
        while rclpy.ok() and publisher.get_subscription_count() < 1:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                print(
                    f"ERROR[{EXIT_NO_SUBSCRIBER}]: no subscriber on {command_topic}; "
                    "neck command not published",
                    file=sys.stderr,
                )
                return EXIT_NO_SUBSCRIBER
            rclpy.spin_once(node, timeout_sec=min(0.1, remaining))

        publisher.publish(String(data=payload))
        print("Published one neck target; observe the head until motion stops.")

        settle_deadline = time.monotonic() + SETTLE_SECONDS
        settled = None

        def _settled_at_target(values: list[float]) -> bool:
            return (
                abs(values[1] - target_pitch) <= SETTLE_TOLERANCE_DEG
                and abs(values[2] - target_yaw) <= SETTLE_TOLERANCE_DEG
            )

        while time.monotonic() < settle_deadline:
            try:
                settled = _read_neck_feedback(rclpy, node, status_topic)
            except TimeoutError:
                break
            if _settled_at_target(settled):
                break
            time.sleep(0.2)
        if settled is None:
            print("WARNING: could not re-read neck feedback after publishing.")
        else:
            print(
                f"  settled_neck_deg: roll={settled[0]:.2f} pitch={settled[1]:.2f} "
                f"yaw={settled[2]:.2f} (target pitch {target_pitch:.2f}, "
                f"target yaw {target_yaw:.2f})"
            )
            if not _settled_at_target(settled):
                print(
                    "WARNING: the neck has not settled at the target within "
                    f"{SETTLE_SECONDS:.1f}s; keep observing the head."
                )
        return EXIT_OK
    except KeyboardInterrupt:
        print("Interrupted; no further neck command will be published.", file=sys.stderr)
        return 130
    except Exception as exc:
        print(f"ERROR[{EXIT_UNEXPECTED}]: neck pitch tool failed: {exc}", file=sys.stderr)
        return EXIT_UNEXPECTED
    finally:
        if node is not None:
            node.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    raise SystemExit(main())
