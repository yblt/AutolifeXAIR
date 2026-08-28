"""等待底盘在导航目标之后完全静止。

到达检测器返回后 Nav2 仍会对基座做微调；消费感知到基座链路的手臂
工作绝不能在基座还在移动时开始。本辅助脚本订阅轮式里程计，只有
实测 twist 连续低于静止阈值达到要求时长后才以 0 退出。

退出码：0 已静止；1 超时仍在移动；2 无里程计数据。
"""

from __future__ import annotations

import argparse
import os
import sys
import time

LINEAR_STILL_MPS = 0.01
ANGULAR_STILL_RADPS = 0.02


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    robot_id = os.uname().nodename.rsplit("-", 1)[-1]  # 机器编号一律按主机名推导，不信环境变量
    parser.add_argument("--topic", default=f"/topic_gv_wheel_odom_0_{robot_id}")
    parser.add_argument("--still-seconds", type=float, default=3.0)
    parser.add_argument("--timeout", type=float, default=60.0)
    args = parser.parse_args(argv)

    import rclpy
    from nav_msgs.msg import Odometry

    rclpy.init()
    node = rclpy.create_node("wait_base_settle")
    state = {"last_msg": None, "still_since": None}

    def on_odom(msg: Odometry) -> None:
        now = time.monotonic()
        state["last_msg"] = now
        twist = msg.twist.twist
        moving = (
            abs(twist.linear.x) > LINEAR_STILL_MPS
            or abs(twist.linear.y) > LINEAR_STILL_MPS
            or abs(twist.angular.z) > ANGULAR_STILL_RADPS
        )
        state["still_since"] = None if moving else (state["still_since"] or now)

    node.create_subscription(Odometry, args.topic, on_odom, 10)
    deadline = time.monotonic() + args.timeout
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
            since = state["still_since"]
            if since is not None and time.monotonic() - since >= args.still_seconds:
                print(f"base settled ({args.still_seconds:.0f} s below thresholds)")
                return 0
        if state["last_msg"] is None:
            print("no odometry received; cannot confirm the base is stationary", file=sys.stderr)
            return 2
        print(f"base still moving after {args.timeout:.0f} s", file=sys.stderr)
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
