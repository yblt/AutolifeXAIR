#!/usr/bin/env python3
"""深度引导的右臂瓶子抓取：低头 -> 定位 -> 伸手 -> 抓握。

把 2026-08-19 现场验证的各环节组合成一条有界流程（证据
`docs/evidence/head-camera-snapshots/20260819-green-locate/`）：

1. HEAD    以 <=18 度的分步把颈部降到标定俯仰角。
2. LOCATE  采集一对头部 RGB-D 帧（复用 ``head_bottle_probe capture``），
           检测绿色瓶盖（只取瓶盖连通域；深度严格在瓶盖内部采样），
           反投影，经 ``CameraTransformer`` 变换，再施加真值修正。
3. REACH   按实测指尖工具偏移计算 hover -> engage 路径点并移动右腕；
           每段行程切成 <=0.30 m 的小段，每段稳定后才走下一段。
4. GRIP    张爪、下探、按 ``bottle_grasp.json`` 的标定位置闭合，
           并验证抓握反馈带。

安全模型（与其他 runner 一致）：

- 默认 preview，且不创建任何 ROS publisher。
- 真实运行必须同时给 ``--execute`` 和 ``--safety-acknowledged``。
- fail-closed：任何阶段失败都就地停止。流程绝不重试、绝不自动
  后撤、失败时绝不张开已闭合的夹爪。
- 左臂永远不会被指令离开其快照位姿。
"""

from __future__ import annotations

import argparse
import json
import math
import os
import subprocess
import sys
import time
from pathlib import Path

EXIT_OK = 0
EXIT_UNEXPECTED = 1
EXIT_ARGS = 2
EXIT_ROS = 3
EXIT_NO_SUBSCRIBER = 4
EXIT_NO_FEEDBACK = 5
EXIT_HEAD = 6
EXIT_CAPTURE = 7
EXIT_DETECTION = 8
EXIT_LOCALIZATION = 9
EXIT_PLAN = 10
EXIT_MOTION = 11
EXIT_GRIPPER = 12
EXIT_GRIP_EMPTY = 13

# --- 标定常量（出处：证据 20260819-green-locate）----
HEAD_PITCH_TARGET_DEG = -30.0     # 真值比对所用的定位姿态
HEAD_PITCH_TOLERANCE_DEG = 1.5
HEAD_STEP_MAX_DEG = 18.0          # 低于服务/head_pitch 的 20 度上限
HEAD_OFFSET_DEG = (0.0, -1.0, 4.0)  # CameraTransformer 偏移（head_bottle_probe）
BASE_CORRECTION_M = (-0.012, -0.048, 0.053)   # 卷尺真值比对
TOOL_OFFSET_M = (0.228, 0.025, -0.037)        # 指尖 - 手腕，腕部水平姿态

# --- 流程几何 ---------------------------------------------------------
HOVER_ABOVE_CAP_M = 0.10          # 指尖在瓶盖上方的余隙
ENGAGE_FORWARD_M = 0.05           # 把瓶盖推进指间这么远
DESCEND_M = 0.20                  # 沿瓶身的垂直下探量
STEP_MAX_M = 0.30                 # 每次发布的平移上限（arm_move 策略）

# 瓶盖检测合理性（"只取瓶盖"规则：瓶盖+瓶肩粘连的连通域曾造成
# 20 cm 深度误差；0.6-1.1 m 处干净的瓶盖约 20-50 px 宽）。
CAP_AREA_MIN_PX = 300
CAP_AREA_MAX_PX = 2600
CAP_ASPECT_MIN = 0.6
CAP_ASPECT_MAX = 1.8
DEPTH_MIN_MM = 450.0
DEPTH_MAX_MM = 1600.0
GREEN_HSV_LOWER = (35, 60, 40)
GREEN_HSV_UPPER = (90, 255, 255)

# 修正后的瓶子点必须落在这个基座坐标工作空间盒内。
BOTTLE_BOX = ((0.30, 0.90), (-0.45, 0.45), (0.70, 1.20))
# 指令的腕部目标必须保持在这个盒内。
WRIST_BOX = ((-0.05, 0.70), (-0.60, 0.25), (0.60, 1.35))

# --- 稳定门（2026-08-19 曾有在途指令毁掉一次运行）--------
SETTLE_NEAR_M = 0.03              # 距目标这么近即可接受……
SETTLE_QUIET_M = 0.008            # ……且在此带内静止满 1 秒
SETTLE_QUIET_S = 1.0
SETTLE_TIMEOUT_S = 15.0

GRIPPER_OPEN_POSITION = 10.0      # 控制器全开限位（范围 10..360）
GRIPPER_OPEN_MAX_FEEDBACK = 25.0  # 下探前必须接近全开
GRIPPER_PUBLISH_REPEATS = 3
GRIPPER_TIMEOUT_S = 6.0

FEEDBACK_TIMEOUT_S = 5.0
SUBSCRIBER_TIMEOUT_S = 5.0


class FlowError(Exception):
    def __init__(self, code: int, message: str) -> None:
        super().__init__(message)
        self.code = code
        self.message = message


def project_root() -> Path:
    here = Path(__file__).resolve().parent
    return here.parent.parent.parent if here.name == "right" else here.parent


def load_gripper_calibration(root: Path) -> tuple[float, float, float]:
    """从抓取 JSON 读出 close_position、grip_feedback_center、close_tolerance。"""
    config_path = root / "right" / "bottle_grasp.json"
    try:
        raw = json.loads(config_path.read_text(encoding="utf-8"))
        return (
            float(raw["close_position"]),
            float(raw["grip_feedback_center"]),
            float(raw["close_tolerance"]),
        )
    except Exception as exc:
        raise FlowError(EXIT_ARGS, f"cannot read gripper calibration from {config_path}: {exc}")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Head-down -> depth-camera locate -> right-arm grasp flow. "
            "Preview is the default and moves nothing."
        )
    )
    parser.add_argument("--execute", action="store_true", help="enable physical motion")
    parser.add_argument(
        "--safety-acknowledged",
        action="store_true",
        help=(
            "confirm the workspace is clear of people/obstacles, an operator "
            "is supervising, and the physical emergency stop is reachable"
        ),
    )
    return parser


def _in_box(point, box) -> bool:
    return all(lo <= value <= hi for value, (lo, hi) in zip(point, box))


# --------------------------------------------------------------------------
# ROS 管线
# --------------------------------------------------------------------------

class Bus:
    """单个节点：状态/EEF 订阅，加上（仅 execute 模式的）publisher。"""

    def __init__(self, rclpy, node, execute: bool, domain: str, robot: str):
        from std_msgs.msg import String

        self.rclpy = rclpy
        self.node = node
        self.String = String
        suffix = f"{domain}_{robot}"
        self.latest_status = None
        self.latest_eef = None
        node.create_subscription(
            String, f"/topic_arm_whole_body_and_gripper_current_joints_status_{suffix}",
            self._on_status, 10)
        node.create_subscription(
            String, f"/topic_arm_current_robot_eef_pose_{suffix}", self._on_eef, 10)
        self.body_pub = self.move_pub = self.grip_pub = None
        if execute:
            self.body_pub = node.create_publisher(
                String, f"/topic_arm_whole_body_target_joints_position_{suffix}", 10)
            self.move_pub = node.create_publisher(
                String, f"/topic_arm_move_eef_pose_in_robot_frame_{suffix}", 10)
            self.grip_pub = node.create_publisher(
                String, f"/topic_arm_gripper_target_joints_position_{suffix}", 10)

    def _on_status(self, msg) -> None:
        try:
            self.latest_status = json.loads(msg.data)
        except Exception:
            pass

    def _on_eef(self, msg) -> None:
        try:
            self.latest_eef = json.loads(msg.data)
        except Exception:
            pass

    def spin(self, seconds: float) -> None:
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            self.rclpy.spin_once(self.node, timeout_sec=0.05)

    def wait_fresh(self, attr: str, timeout: float = FEEDBACK_TIMEOUT_S):
        setattr(self, attr, None)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.rclpy.spin_once(self.node, timeout_sec=0.05)
            value = getattr(self, attr)
            if value is not None:
                return value
        raise FlowError(EXIT_NO_FEEDBACK, f"no fresh {attr} within {timeout:.1f}s")

    def neck_deg(self):
        status = self.wait_fresh("latest_status")
        return [float(v) for v in status["neck_joint_state"]["position"]]

    def right_gripper(self) -> float:
        status = self.wait_fresh("latest_status")
        return float(status["right_gripper_state"]["position"][0])

    def eef_state(self):
        state = self.wait_fresh("latest_eef")
        def pose(side):
            p = state[f"{side}_eef_pose"]
            return [float(v) for v in p["position"]], [float(v) for v in p["rotation"]]
        return pose("left"), pose("right")

    def require_subscriber(self, publisher, label: str) -> None:
        deadline = time.monotonic() + SUBSCRIBER_TIMEOUT_S
        while publisher.get_subscription_count() < 1:
            if time.monotonic() >= deadline:
                raise FlowError(EXIT_NO_SUBSCRIBER, f"no subscriber on the {label} topic")
            self.rclpy.spin_once(self.node, timeout_sec=0.1)

    def publish(self, publisher, payload: dict, label: str, repeats: int = 1) -> None:
        self.require_subscriber(publisher, label)
        text = json.dumps(payload, separators=(",", ":"))
        for index in range(repeats):
            publisher.publish(self.String(data=text))
            if index + 1 < repeats:
                self.spin(0.2)
        print(f"  published {label}: {text}")


# --------------------------------------------------------------------------
# 各阶段
# --------------------------------------------------------------------------

def stage_head(bus: Bus, execute: bool) -> None:
    roll, pitch, yaw = bus.neck_deg()
    print(f"[head] current neck deg: roll={roll:.2f} pitch={pitch:.2f} yaw={yaw:.2f}")
    if abs(pitch - HEAD_PITCH_TARGET_DEG) <= HEAD_PITCH_TOLERANCE_DEG:
        print("[head] already at the locate pose")
        return
    steps = []
    current = pitch
    while abs(HEAD_PITCH_TARGET_DEG - current) > 1e-6:
        move = max(-HEAD_STEP_MAX_DEG, min(HEAD_STEP_MAX_DEG, HEAD_PITCH_TARGET_DEG - current))
        current += move
        steps.append(round(current, 2))
    print(f"[head] pitch steps -> {steps}")
    if not execute:
        return
    for target in steps:
        bus.publish(bus.body_pub, {"neck_target_joints_position": [roll, target, yaw]},
                    "neck target")
        deadline = time.monotonic() + SETTLE_TIMEOUT_S
        while True:
            _, now_pitch, _ = bus.neck_deg()
            if abs(now_pitch - target) <= HEAD_PITCH_TOLERANCE_DEG:
                break
            if time.monotonic() >= deadline:
                raise FlowError(EXIT_HEAD, f"neck did not reach {target} deg (at {now_pitch:.2f})")
        bus.spin(0.5)
    print("[head] locate pose reached")


def stage_locate(root: Path, run_dir: Path, node) -> tuple[float, float, float]:
    import numpy as np
    import cv2
    from autolife_robot_vision.camera_transformer import CameraTransformer

    capture_dir = run_dir / "capture"
    probe = root / "examples" / "camera" / "head_bottle_probe.py"
    result = subprocess.run(
        [sys.executable, str(probe), "capture", "--output-dir", str(capture_dir), "--frames", "1"],
        capture_output=True, text=True)
    if result.returncode != 0:
        raise FlowError(EXIT_CAPTURE, f"head capture failed: {result.stderr.strip()[-300:]}")

    data = np.load(capture_dir / "frame_000.npz")
    color, depth = data["color"], data["depth"].astype(np.float32)
    K = json.loads((capture_dir / "intrinsics.json").read_text(encoding="utf-8"))["intrinsics"]

    hsv = cv2.cvtColor(color, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array(GREEN_HSV_LOWER, np.uint8), np.array(GREEN_HSV_UPPER, np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    count, _, stats, _ = cv2.connectedComponentsWithStats(mask)
    caps = []
    for i in range(1, count):
        x, y, w, h, area = (int(v) for v in stats[i])
        if CAP_AREA_MIN_PX <= area <= CAP_AREA_MAX_PX and CAP_ASPECT_MIN <= h / w <= CAP_ASPECT_MAX:
            caps.append((x, y, w, h, area))
    if not caps:
        cv2.imwrite(str(run_dir / "locate_failed.png"), color)
        raise FlowError(EXIT_DETECTION, "no cap-like green component (see locate_failed.png)")
    caps.sort(key=lambda c: c[1])
    if len(caps) > 1 and caps[1][1] - caps[0][1] < 40:
        raise FlowError(EXIT_DETECTION, f"ambiguous green candidates: {caps[:2]}")
    x, y, w, h, area = caps[0]
    u, v = x + w // 2, y + 2
    window = depth[y + h // 3 : y + h, x + w // 4 : x + 3 * w // 4]
    window = window[(window > DEPTH_MIN_MM) & (window < DEPTH_MAX_MM)]
    if window.size == 0:
        raise FlowError(EXIT_DETECTION, "no valid depth inside the cap component")
    z = float(np.median(window))

    fx, fy, cx, cy = K["fx"], K["fy"], K["ppx"], K["ppy"]
    pc = np.array([(u - cx) * z / fx, (v - cy) * z / fy, z, 1000.0]) / 1000.0

    vis = color.copy()
    cv2.rectangle(vis, (x, y), (x + w, y + h), (0, 255, 0), 2)
    cv2.imwrite(str(run_dir / "locate_annotated.png"), vis)

    import rclpy
    transformer = CameraTransformer(HEAD_OFFSET_DEG, node)
    transform = None
    deadline = time.monotonic() + 8.0
    while time.monotonic() < deadline and transform is None:
        rclpy.spin_once(node, timeout_sec=0.05)
        transform = transformer.compute_object_pose_in_base(np.eye(4))
    if transform is None:
        raise FlowError(EXIT_LOCALIZATION, "no fresh base<-camera transform within 8s")
    T = np.asarray(transform, dtype=float)
    raw = (T @ pc)[:3]
    corrected = tuple(float(r + c) for r, c in zip(raw, BASE_CORRECTION_M))
    print(f"[locate] cap bbox=({x},{y},{w},{h}) area={area} depth={z:.0f}mm")
    print(f"[locate] raw base point = ({raw[0]:.3f}, {raw[1]:.3f}, {raw[2]:.3f}) m")
    print(f"[locate] corrected     = ({corrected[0]:.3f}, {corrected[1]:.3f}, {corrected[2]:.3f}) m")
    if not _in_box(corrected, BOTTLE_BOX):
        raise FlowError(EXIT_LOCALIZATION, f"corrected point {corrected} outside workspace box {BOTTLE_BOX}")
    (run_dir / "locate.json").write_text(json.dumps({
        "cap_bbox": [x, y, w, h], "depth_mm": z,
        "raw_base": list(map(float, raw)), "corrected_base": list(corrected),
    }, indent=2), encoding="utf-8")
    return corrected


def plan_waypoints(bottle, wrist_start):
    bx, by, bz = bottle
    ox, oy, oz = TOOL_OFFSET_M
    hover = (bx - ox, by - oy, bz + HOVER_ABOVE_CAP_M - oz)
    engage = (hover[0] + ENGAGE_FORWARD_M, hover[1], hover[2])
    grasp = (engage[0], engage[1], engage[2] - DESCEND_M)
    legs = [
        ("raise", (wrist_start[0], wrist_start[1], hover[2])),
        ("hover", hover),
        ("engage", engage),
        ("open-gripper", None),
        ("descend", grasp),
        ("close-gripper", None),
    ]
    for name, target in legs:
        if target is not None and not _in_box(target, WRIST_BOX):
            raise FlowError(EXIT_PLAN, f"wrist target {name}={target} outside safe box {WRIST_BOX}")
    return legs


def wait_settled(bus: Bus, target) -> None:
    history = []
    deadline = time.monotonic() + SETTLE_TIMEOUT_S
    while True:
        (_, _), (pos, _) = bus.eef_state()
        now = time.monotonic()
        history.append((now, pos))
        history = [(t, p) for t, p in history if now - t <= SETTLE_QUIET_S]
        near = math.dist(pos, target) <= SETTLE_NEAR_M
        quiet = (
            now - history[0][0] >= SETTLE_QUIET_S * 0.8
            and max(math.dist(p, pos) for _, p in history) <= SETTLE_QUIET_M
        )
        if near and quiet:
            return
        if now >= deadline:
            raise FlowError(
                EXIT_MOTION,
                f"arm did not settle at {tuple(round(v,3) for v in target)} "
                f"(at {tuple(round(v,3) for v in pos)}); stopping in place")


def move_right_wrist(bus: Bus, target, right_quat) -> None:
    while True:
        (left_pos, left_quat), (right_pos, _) = bus.eef_state()
        remaining = math.dist(right_pos, target)
        if remaining <= SETTLE_NEAR_M:
            break
        scale = min(1.0, STEP_MAX_M / remaining)
        chunk = [rp + (tv - rp) * scale for rp, tv in zip(right_pos, target)]
        bus.publish(bus.move_pub, {
            "pos_left_in_robot": left_pos, "quat_left_in_robot": left_quat,
            "pos_right_in_robot": chunk, "quat_right_in_robot": right_quat,
        }, "right wrist target")
        wait_settled(bus, chunk)


def command_gripper(bus: Bus, position: float, label: str) -> None:
    bus.publish(bus.grip_pub, {"right_gripper_target_joints_position": [position]},
                f"gripper {label}", repeats=GRIPPER_PUBLISH_REPEATS)
    bus.spin(GRIPPER_TIMEOUT_S)


def main(argv=None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    if args.execute and not args.safety_acknowledged:
        parser.error("--execute requires --safety-acknowledged")

    root = project_root()
    close_position, grip_center, grip_tolerance = load_gripper_calibration(root)
    run_dir = root / "evidence" / "depth_guided_grasp" / time.strftime("%Y%m%dT%H%M%SZ", time.gmtime())
    run_dir.mkdir(parents=True, exist_ok=True)
    mode = "RUN" if args.execute else "PREVIEW"
    print(f"Depth-guided grasp [{mode}]  evidence: {run_dir}")
    print(f"  gripper close={close_position} expect feedback {grip_center}±{grip_tolerance}")

    domain = os.environ.get("ROS_DOMAIN_ID", "0").strip() or "0"
    robot = os.environ.get("ROBOT_ID", "").strip() or os.uname().nodename.rsplit("-", 1)[-1]

    try:
        # 在 rclpy 之前加载 conda 环境的 libssl/cv2：ROS 库会把系统
        # libssl.so.3 拖进进程，cv2 的 libcurl 若被迫复用那份副本就会
        # 报 "OPENSSL_3.2.0 not found"。
        import ssl  # noqa: F401
        import cv2  # noqa: F401
        import rclpy
    except ImportError as exc:
        print(f"ERROR[{EXIT_ROS}]: ROS runtime unavailable: {exc}", file=sys.stderr)
        return EXIT_ROS

    node = None
    try:
        rclpy.init(args=None)
        node = rclpy.create_node("xr_depth_guided_grasp")
        bus = Bus(rclpy, node, args.execute, domain, robot)

        stage_head(bus, args.execute)
        pitch_now = bus.neck_deg()[1]
        if abs(pitch_now - HEAD_PITCH_TARGET_DEG) > HEAD_PITCH_TOLERANCE_DEG:
            print(f"[locate] skipped: neck pitch {pitch_now:.1f} deg is not the locate pose "
                  f"({HEAD_PITCH_TARGET_DEG} deg); preview with the head lowered for a full plan")
            return EXIT_OK

        bottle = stage_locate(root, run_dir, node)
        (left_pos, _), (right_pos, right_quat) = bus.eef_state()
        print(f"[plan] right wrist at ({right_pos[0]:.3f}, {right_pos[1]:.3f}, {right_pos[2]:.3f})")
        legs = plan_waypoints(bottle, right_pos)
        for name, target in legs:
            shown = "" if target is None else f" -> ({target[0]:.3f}, {target[1]:.3f}, {target[2]:.3f})"
            print(f"[plan] {name}{shown}")
        (run_dir / "plan.json").write_text(json.dumps(
            [{"leg": n, "wrist_target": t} for n, t in legs], indent=2), encoding="utf-8")

        if not args.execute:
            print("Preview only: nothing was moved.")
            return EXIT_OK

        for name, target in legs:
            print(f"[motion] {name}")
            if name == "open-gripper":
                command_gripper(bus, GRIPPER_OPEN_POSITION, "open")
                feedback = bus.right_gripper()
                if feedback > GRIPPER_OPEN_MAX_FEEDBACK:
                    raise FlowError(EXIT_GRIPPER, f"gripper did not open (feedback {feedback:.1f})")
            elif name == "close-gripper":
                command_gripper(bus, close_position, "close")
                feedback = bus.right_gripper()
                print(f"[grip] feedback = {feedback:.1f} (expect {grip_center}±{grip_tolerance})")
                if abs(feedback - grip_center) <= grip_tolerance:
                    print("GRASP OK: bottle held; flow complete. Gripper stays closed.")
                else:
                    print("GRASP EMPTY/UNCERTAIN: feedback outside the grip band; "
                          "gripper left as-is, arm stopped in place.", file=sys.stderr)
                    return EXIT_GRIP_EMPTY
            else:
                move_right_wrist(bus, target, right_quat)
        return EXIT_OK
    except FlowError as exc:
        print(f"ERROR[{exc.code}]: {exc.message}", file=sys.stderr)
        return exc.code
    except KeyboardInterrupt:
        print("Interrupted; no further command will be published.", file=sys.stderr)
        return 130
    except Exception as exc:
        print(f"ERROR[{EXIT_UNEXPECTED}]: {exc}", file=sys.stderr)
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
