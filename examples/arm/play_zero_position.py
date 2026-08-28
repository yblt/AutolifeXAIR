#!/usr/bin/env python3
"""预览或发布 AutoLife S2 预置的零位（zero_position）动作。"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time


SUBSCRIBER_TIMEOUT_SECONDS = 5.0
PUBLISH_FLUSH_SECONDS = 0.5
ACTION_NAME = "zero_position"


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Preview the configured zero-position action. Execution moves the "
            "neck, both arms, and waist/leg joints."
        )
    )
    parser.add_argument("--execute", action="store_true", help="publish the action")
    parser.add_argument(
        "--safety-acknowledged",
        action="store_true",
        help=(
            "confirm navigation is stopped, the whole-body motion envelope is "
            "clear, an operator is supervising, and emergency stop is available"
        ),
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    if args.execute and not args.safety_acknowledged:
        parser.error("--execute requires --safety-acknowledged")

    domain_id = os.environ.get("ROS_DOMAIN_ID", "0").strip() or "0"
    robot_id = os.uname().nodename.rsplit("-", 1)[-1]  # 机器编号一律按主机名推导，不信环境变量
    topic = f"/topic_arm_robot_action_{domain_id}_{robot_id}"
    payload = {"action_type": "play", "action_name": ACTION_NAME}

    print("Zero-position action preview")
    print(f"  topic: {topic}")
    print("  message_type: std_msgs/msg/String")
    print(f"  payload: {json.dumps(payload, separators=(',', ':'))}")
    print("  configured_motion: neck, left_arm, right_arm, waist_leg -> zero")
    print("  configured_duration_seconds: 3.0")
    print(f"  execution_gate --execute: {'present' if args.execute else 'absent'}")
    print(
        "  safety_gate --safety-acknowledged: "
        f"{'present' if args.safety_acknowledged else 'absent'}"
    )

    if not args.execute:
        print("Dry run only: no action command was published.")
        return 0

    try:
        import rclpy
        from rclpy.node import Node
        from std_msgs.msg import String
    except ImportError as exc:
        print(f"ERROR: ROS runtime unavailable: {exc}", file=sys.stderr)
        return 3

    node = None
    try:
        rclpy.init(args=None)
        node = Node("xr_zero_position_action")
        publisher = node.create_publisher(String, topic, 10)
        deadline = time.monotonic() + SUBSCRIBER_TIMEOUT_SECONDS
        while rclpy.ok() and publisher.get_subscription_count() < 1:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                print(
                    f"ERROR: no subscriber on {topic}; action not published",
                    file=sys.stderr,
                )
                return 4
            rclpy.spin_once(node, timeout_sec=min(0.1, remaining))

        publisher.publish(String(data=json.dumps(payload, separators=(",", ":"))))
        rclpy.spin_once(node, timeout_sec=PUBLISH_FLUSH_SECONDS)
        print(f"Published one {ACTION_NAME!r} action to {topic}.")
        print("Observe the robot until the configured motion finishes.")
        return 0
    except KeyboardInterrupt:
        print("Interrupted; no further action command will be published.", file=sys.stderr)
        return 130
    except Exception as exc:
        print(f"ERROR: zero-position action failed: {exc}", file=sys.stderr)
        return 5
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
