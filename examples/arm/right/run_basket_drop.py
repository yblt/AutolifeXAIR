#!/usr/bin/env python3
"""头部相机引导的黄色篮筐投放 runner。

流程（单次人工启动的任务，默认 preview）::

    PREFLIGHT -> VERIFY_HOLDING (夹爪反馈必须在标定持物带内，
    否则拒绝执行) -> LOCATE (头部 RGBD：稳定的黄色筐沿检测、
    筐沿像素深度采样、CameraTransformer 基座坐标点)
    -> CONFIRM (任何运动之前做第二次独立定位；两点的水平偏差
    必须在配置容差内一致)
    -> MOVE_TO_HOVER (单轴逐步有界移动到第二个点 + 标定释放偏移)
    -> RELEASE (每一步的完整双臂反馈全部通过后才张爪)
    -> RETRACT -> COMPLETE

2026-08-13 现场修订：原方案的"运动后再检测"门被现场证伪——在释放
位置，机器人自己的右前臂完全遮挡了头部相机看篮筐的视线（见
task-3.1 证据 `release_pose_view.jpg`）。因此释放门改为"运动前
双定位一致 + 每步双臂反馈门"。释放偏移的标定方式是（手工摆放的
真值 EEF 减去感知筐沿点），以吸收感知链路的系统性偏差
（2026-08-13 实测偏远 27 cm）。

安全底线：实机执行需要 ``run --execute``；右爪反馈证明确实持物
之前 runner 拒绝运动；每个手臂阶段发布一个完整双臂绝对目标，并
等到新鲜的双臂反馈后才进行下一步；每步平移和工作包络都由已验证
的配置（`basket_drop_config`）封顶；定位失败或一致性检查失败会
在任何运动之前中止；反馈失败、触限或总超时都会保持闭爪并停止
发布后续目标。软件中止只能停止发布后续目标——它无法取消控制器
已接受的在途目标；物理急停才是现场保障。

状态机（`run_drop`）是注入式 runtime 边界之上的纯逻辑，离线测试
因此永不 import ROS、SDK 或 OpenCV。具体的 `_RosRuntime` 复用
`run_bottle_grasp` 现场验证过的手臂原语（双臂载荷、反馈门、夹爪
目标三连发、负载放宽收敛）和 `head_bottle_probe` 现场验证过的
头部相机管线（SHM 帧对、内参、CameraTransformer、深度采样）。
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
    from examples.arm.right.basket_drop_config import (
        STEP_MAX_M,
        BasketDropConfig,
        ConfigError,
        load_config,
        require_identity,
    )
except ImportError:  # 在 examples/arm 目录下直接执行。
    from basket_drop_config import (  # type: ignore[no-redef]
        STEP_MAX_M,
        BasketDropConfig,
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
    spec = importlib.util.spec_from_file_location(f"xr_basket_drop_{name}", module_path)
    if spec is None or spec.loader is None:  # pragma: no cover - repo layout invariant
        raise ImportError(f"could not load sibling module {name!r} from {module_path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    _MODULE_CACHE[key] = module
    return module


# 与已验收的抓取 runner 共享的现场验证手臂原语：位姿解析、双臂载荷、
# 反馈门、话题名和错误类型。
_ARM = _load_sibling("arm/right", "run_bottle_grasp")

RunnerError = _ARM.RunnerError
FeedbackError = _ARM.FeedbackError
Pose = _ARM.Pose
EEFState = _ARM.EEFState
GripperState = _ARM.GripperState
parse_eef_state = _ARM.parse_eef_state
parse_gripper_state = _ARM.parse_gripper_state
state_payload = _ARM.state_payload
interface_names = _ARM.interface_names


def drop_interface_names(
    domain_id: str = _ARM.DEFAULT_DOMAIN_ID, robot_suffix: str = _ARM.DEFAULT_ROBOT_SUFFIX
) -> dict[str, str]:
    """抓取 runner 的话题集合，外加有界颈部俯仰阶段所用的
    全身目标话题（仅含颈部字段的载荷）。"""

    names = interface_names(domain_id, robot_suffix)
    suffix = f"{str(domain_id).strip()}_{str(robot_suffix).strip()}"
    names["whole_body_target_topic"] = f"/topic_arm_whole_body_target_joints_position_{suffix}"
    return names

BOUNDARY_TOLERANCE = 1e-9
LOCATE_TRANSFORM_TIMEOUT_S = 10.0

# 2026-08-21 T3 事故加固：nav 判达后 Nav2 残余偏航对齐可能尚未完成，
# 首次检测窗口内篮子可能还没进入 ROI。LOCATE 有界重试跨约 10 s 骑过
# 对齐完成；检测门本身不放宽，全部尝试失败仍 fail-closed 不发运动目标。
LOCATE_MAX_ATTEMPTS = 3
LOCATE_RETRY_DELAY_S = 4.0

# 检测前的有界自动颈部俯仰，与现场验证过的 head_pitch.py 工具一致：
# 仅含颈部字段的载荷、每步 <=20 度、到位门控。
HEAD_PITCH_STEP_MAX_DEG = 20.0
HEAD_PITCH_SETTLE_TOLERANCE_DEG = 1.5
HEAD_PITCH_SETTLE_TIMEOUT_S = 8.0
HEAD_PITCH_MAX_STEPS = 6

EXPECTED_CAMERA_ID = "mod_camera_rgbd_head"

STAGES = (
    "PREFLIGHT",
    "VERIFY_HOLDING",
    "HEAD_PITCH",
    "LOCATE",
    "CONFIRM",
    "MOVE_TO_HOVER",
    "RELEASE",
    "RETRACT",
    "COMPLETE",
)


def plan_axis_steps(
    start: Sequence[float],
    goal: Sequence[float],
    step_max_m: float,
    *,
    y_align_x_m: float | None = None,
) -> list[tuple[float, float, float]]:
    """从 ``start`` 到 ``goal`` 的单轴逐步绝对路径点序列。

    每个路径点只改变一个轴，且改变量不超过 ``step_max_m``。垂直上升
    先于水平运动，垂直下降放在最后。设置了 ``y_align_x_m`` 时
    （2026-08-13 现场教训：篮筐后壁高、前壁低），横向（Y）运动只在
    X 位于该对齐线或其后方时进行：规划先把 X 移到对齐线，再对齐 Y，
    最后走完剩余的 X——接近和撤退方向都是如此。只要两个端点都在
    轴对齐包络内，所有路径点也都在包络内，因为对齐线已被验证位于
    包络的 X 范围之内。
    """

    if not isinstance(step_max_m, (int, float)) or not math.isfinite(step_max_m) or step_max_m <= 0:
        raise RunnerError(f"step_max_m must be a positive finite number, received {step_max_m!r}")
    current = [float(value) for value in start]
    target = [float(value) for value in goal]
    if len(current) != 3 or len(target) != 3:
        raise RunnerError("start and goal must be (x, y, z) triples")

    # 每个规划条目是 (轴, 该轴的绝对目标值)。
    plan: list[tuple[int, float]] = []
    if target[2] > current[2]:
        plan.append((2, target[2]))
    needs_y = abs(target[1] - current[1]) > BOUNDARY_TOLERANCE
    if y_align_x_m is not None and needs_y and abs(target[0] - current[0]) > BOUNDARY_TOLERANCE:
        align_x = float(y_align_x_m)
        # 把中间停靠点截断到实际经过的线段内。
        low, high = min(current[0], target[0]), max(current[0], target[0])
        align_x = min(max(align_x, low), high)
        plan.append((0, align_x))
        plan.append((1, target[1]))
        plan.append((0, target[0]))
    else:
        plan.append((0, target[0]))
        plan.append((1, target[1]))
    if target[2] < current[2]:
        plan.append((2, target[2]))

    waypoints: list[tuple[float, float, float]] = []
    for axis, stop in plan:
        delta = stop - current[axis]
        if abs(delta) <= BOUNDARY_TOLERANCE:
            continue
        count = max(1, math.ceil(abs(delta) / step_max_m - BOUNDARY_TOLERANCE))
        for index in range(count):
            if index == count - 1:
                current[axis] = stop  # 精确落到终点，避免浮点漂移
            else:
                current[axis] += delta / count
            waypoints.append((current[0], current[1], current[2]))
    return waypoints


def _with_right_position(state: EEFState, position: Sequence[float]) -> EEFState:
    right = state.right
    return EEFState(state.left, Pose(tuple(float(value) for value in position), right.rotation))


@dataclass
class DropRunResult:
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


def run_drop(
    config: BasketDropConfig,
    runtime: Any,
    *,
    clock: Callable[[], float] = time.monotonic,
    sleep: Callable[[float], None] = time.sleep,
) -> DropRunResult:
    """经 runtime 边界执行一次有界投放，fail-closed。

    runtime 必须提供：``read_eef_state()``、``read_gripper_state()``、
    ``read_neck_state()``、``publish_neck_target((roll, pitch, yaw))``、
    ``wait_for_neck_settle(pitch)``、``publish_eef_target``、
    ``wait_for_eef_feedback``、``publish_gripper_target``、
    ``wait_for_gripper_feedback(target, tolerance)`` 和
    ``locate_basket(config)``；可选的 ``save_frame(tag, frame)`` 用于
    保存证据图像。RELEASE 之前的所有失败路径都保持闭爪；
    中止只停止发布后续目标。
    """

    result = DropRunResult()

    def record_stage(name: str, detail: Mapping[str, Any] | None = None) -> None:
        result.last_stage = name
        result.stage_history.append({"stage": name, "at": clock(), **(dict(detail) if detail else {})})

    def abort(reason: str) -> DropRunResult:
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

    def move_and_wait(target: EEFState, *, loaded: bool) -> EEFState:
        # repeats=3：防止控制器丢弃单次发布的目标导致单轴步进卡死
        # （2026-08-20，MOVE_TO_HOVER 两次停滞）。
        runtime.publish_eef_target(target, repeats=3)
        result.issued_targets += 1
        return parse_eef_state(runtime.wait_for_eef_feedback(target, loaded=loaded))

    def locate(phase: str) -> Any:
        located = runtime.locate_basket(config)
        result.measurements.append(
            {
                "phase": phase,
                "at": clock(),
                "located": bool(getattr(located, "located", False)),
                "point_base": (
                    list(located.point_base) if getattr(located, "point_base", None) is not None else None
                ),
                "reason": getattr(located, "reason", ""),
                "record": getattr(located, "record", None),
            }
        )
        return located

    try:
        record_stage("PREFLIGHT")
        state = parse_eef_state(runtime.read_eef_state())
        if not config.contains(state.right.position):
            return abort(
                f"preflight right EEF position {list(state.right.position)} is outside the "
                f"configured workspace envelope"
            )

        record_stage("VERIFY_HOLDING")
        grip = parse_gripper_state(runtime.read_gripper_state())
        if grip.communication_lost:
            return abort("right gripper feedback reports communication_lost; refusing to execute")
        band_error = abs(grip.position - config.held_feedback_center)
        if band_error > config.held_feedback_tolerance + BOUNDARY_TOLERANCE:
            return abort(
                f"right gripper feedback {grip.position:.1f} is outside the holding band "
                f"{config.held_feedback_center}+/-{config.held_feedback_tolerance}; "
                "no object is held, refusing to execute (no motion targets published)"
            )
        result.stage_history[-1]["gripper_position"] = grip.position

        deadline = clock() + config.overall_timeout_s

        def check_deadline(context: str) -> None:
            if clock() >= deadline:
                raise RunnerError(
                    f"overall timeout of {config.overall_timeout_s} s reached during {context}; "
                    "stopping further target publication"
                )

        record_stage("HEAD_PITCH", {"target_deg": config.head_pitch_deg})
        neck = runtime.read_neck_state()
        try:
            roll, pitch, yaw = (float(value) for value in neck)
        except (TypeError, ValueError) as exc:
            return abort(f"neck state is unreadable: {exc}; no motion targets published")
        if not all(math.isfinite(value) for value in (roll, pitch, yaw)):
            return abort("neck state is not finite; no motion targets published")
        pitch_steps = 0
        while abs(config.head_pitch_deg - pitch) > HEAD_PITCH_SETTLE_TOLERANCE_DEG:
            check_deadline("the head pitch")
            pitch_steps += 1
            if pitch_steps > HEAD_PITCH_MAX_STEPS:
                return abort(
                    f"neck pitch did not reach {config.head_pitch_deg} deg within "
                    f"{HEAD_PITCH_MAX_STEPS} bounded steps (at {pitch:.2f} deg); "
                    "stopping before any arm target"
                )
            delta = config.head_pitch_deg - pitch
            step = max(-HEAD_PITCH_STEP_MAX_DEG, min(HEAD_PITCH_STEP_MAX_DEG, delta))
            step_target = pitch + step
            runtime.publish_neck_target((roll, step_target, yaw))
            result.issued_targets += 1
            pitch = float(runtime.wait_for_neck_settle(step_target))
        result.stage_history[-1]["settled_pitch_deg"] = pitch

        record_stage("LOCATE")
        located = locate("locate")
        locate_attempts = 1
        while not getattr(located, "located", False) and locate_attempts < LOCATE_MAX_ATTEMPTS:
            save_frame(f"locate_failed_{locate_attempts}", getattr(located, "frame", None))
            check_deadline("the basket localization retry")
            sleep(LOCATE_RETRY_DELAY_S)
            locate_attempts += 1
            located = locate("locate")
        if not getattr(located, "located", False):
            save_frame(f"locate_failed_{locate_attempts}", getattr(located, "frame", None))
            return abort(
                f"basket localization failed after {locate_attempts} attempts: "
                f"{getattr(located, 'reason', 'unknown')}; no motion targets published"
            )
        save_frame("locate", getattr(located, "frame", None))
        first_x, first_y, _first_z = (float(value) for value in located.point_base)

        # 2026-08-13 现场修订：第二次独立定位发生在任何运动"之前"——
        # 在释放位置右前臂会遮挡篮筐，"运动后再检测"门在物理上不可行。
        # 两次独立稳定采集的一致性构成释放门的视觉半边；每步双臂
        # 反馈门构成其到位半边。
        record_stage("CONFIRM")
        confirm = locate("confirm")
        if not getattr(confirm, "located", False):
            save_frame("confirm_failed", getattr(confirm, "frame", None))
            return abort(
                "second localization failed: "
                f"{getattr(confirm, 'reason', 'unknown')}; no motion targets published"
            )
        save_frame("confirm", getattr(confirm, "frame", None))
        basket_x, basket_y, basket_z = (float(value) for value in confirm.point_base)
        horizontal_error = math.hypot(basket_x - first_x, basket_y - first_y)
        result.stage_history[-1]["horizontal_error_m"] = horizontal_error
        if horizontal_error > config.confirm_tolerance_m + BOUNDARY_TOLERANCE:
            return abort(
                f"the two localizations disagree by {horizontal_error:.3f} m horizontally, beyond "
                f"the {config.confirm_tolerance_m} m tolerance; no motion targets published"
            )

        hover = (
            basket_x + config.hover_offset_m[0],
            basket_y + config.hover_offset_m[1],
            basket_z + config.hover_offset_m[2],
        )
        if not config.contains(hover):
            return abort(
                f"release target {list(hover)} is outside the configured workspace envelope; "
                "no motion targets published"
            )

        record_stage("MOVE_TO_HOVER", {"hover": list(hover), "basket": [basket_x, basket_y, basket_z]})
        current = state
        for waypoint in plan_axis_steps(
            state.right.position, hover, config.step_max_m, y_align_x_m=config.y_align_x_m
        ):
            check_deadline("the move to the release target")
            if not config.contains(waypoint):
                return abort(f"planned waypoint {list(waypoint)} left the workspace envelope")
            current = move_and_wait(_with_right_position(current, waypoint), loaded=True)

        record_stage("RELEASE", {"open_position": config.open_position})
        check_deadline("the release")
        runtime.publish_gripper_target(config.open_position)
        result.issued_targets += 1
        release = parse_gripper_state(
            runtime.wait_for_gripper_feedback(config.open_position, config.gripper_tolerance)
        )
        result.stage_history[-1]["gripper_position"] = release.position

        record_stage("RETRACT", {"retract": list(config.retract_position_m)})
        for waypoint in plan_axis_steps(
            current.right.position, config.retract_position_m, config.step_max_m, y_align_x_m=config.y_align_x_m
        ):
            check_deadline("the retract")
            if not config.contains(waypoint):
                return abort(f"planned retract waypoint {list(waypoint)} left the workspace envelope")
            # 2026-08-13 现场验证（run 20260813T101854Z）：伸出的手臂在
            # 返程收敛不到空载严格的 3 mm 门；回撤精度无关紧要，所以
            # 即使瓶子已释放，回撤仍使用放宽门。
            current = move_and_wait(_with_right_position(current, waypoint), loaded=True)

        record_stage("COMPLETE")
        result.status = "complete"
        result.reason = "drop complete; bottle released into the basket and arm retracted"
        return result
    except RunnerError as exc:
        return abort(str(exc))
    except Exception as exc:  # 防御边界：遇到意外错误绝不继续运动。
        return abort(f"unexpected runner failure: {exc}")


# ---------------------------------------------------------------------------
# 具体的 ROS runtime（只在真正 `run` 时构造）。
# ---------------------------------------------------------------------------


class _HeadPairReader:
    """把配对的头部彩色/深度帧包装成检测器可用的鸭子类型帧。"""

    def __init__(self, probe: Any, color_consumer: Any, depth_consumer: Any) -> None:
        self._probe = probe
        self._color = color_consumer
        self._depth = depth_consumer

    def read(self, *, timeout: float) -> Any:
        color_sample, depth_sample = self._probe._poll_matched_pair(
            self._color, self._depth, timeout, 0.05
        )
        return SimpleNamespace(
            image=color_sample[0],
            frame_id=int(color_sample[1]),
            depth=depth_sample[0],
            received_at=time.time(),
        )


class _RosRuntime(_ARM._RosRuntime):
    """抓取基线的手臂/夹爪 runtime，外加头部相机定位器。

    原样继承现场验证过的约定：SDK/cv2 在 ``rclpy.init`` 之前
    import、完整双臂目标、夹爪目标三连发、严格/负载两档反馈门。
    夹爪状态话题就是全身关节状态话题，因此其消息到达时刻同时充当
    ``CameraTransformer`` 所需的关节状态新鲜度信号。
    """

    def __init__(self, names: Mapping[str, str], evidence_dir: Any) -> None:
        super().__init__(names, evidence_dir)
        self._whole_body_pub = self._node.create_publisher(
            self._string_type, self.names["whole_body_target_topic"], 10
        )
        self._joint_arrival: dict[str, float | None] = {"last_arrival": None}
        self._head_ready = False
        self._probe: Any = None
        self._basket: Any = None
        self._geometry: Any = None
        self._transformer: Any = None
        self._identity: Any = None
        self._pair_reader: Any = None
        self._head_consumers: list[tuple[str, Any]] = []
        self._intrinsics_raw: Any = None

    def _on_gripper(self, message: Any) -> None:
        super()._on_gripper(message)
        # 只记录到达时刻；载荷由夹爪反馈门负责解析。
        self._joint_arrival["last_arrival"] = time.monotonic()

    def _neck_position(self) -> tuple[float, float, float] | None:
        if self._gripper_raw is None:
            return None
        try:
            position = json.loads(self._gripper_raw)["neck_joint_state"]["position"]
            roll, pitch, yaw = (float(value) for value in position)
        except Exception:
            return None
        return (roll, pitch, yaw)

    def read_neck_state(self) -> tuple[float, float, float]:
        baseline = self._gripper_seq
        self._spin_until(
            lambda: self._gripper_seq > baseline and self._neck_position() is not None,
            _ARM.DEFAULT_FEEDBACK_TIMEOUT_SECONDS,
            "fresh neck state",
        )
        neck = self._neck_position()
        assert neck is not None
        return neck

    def publish_neck_target(self, rpy: Sequence[float]) -> None:
        self._require_subscriber(self._whole_body_pub, self.names["whole_body_target_topic"])
        message = self._string_type()
        # 仅含颈部字段的载荷：缺席的键不会被手臂服务触碰
        # （现场验证过的约定，见 examples/arm/head_pitch.py）。
        message.data = _json_text({"neck_target_joints_position": [float(value) for value in rpy]})
        self._whole_body_pub.publish(message)

    def wait_for_neck_settle(self, target_pitch: float) -> float:
        settled: list[float] = []

        def reached() -> bool:
            neck = self._neck_position()
            if neck is None:
                return False
            if abs(neck[1] - float(target_pitch)) <= HEAD_PITCH_SETTLE_TOLERANCE_DEG:
                settled.append(neck[1])
                return True
            return False

        self._spin_until(reached, HEAD_PITCH_SETTLE_TIMEOUT_S, "neck pitch settle")
        return settled[-1]

    def read_gripper_state(self) -> GripperState:
        baseline = self._gripper_seq
        self._spin_until(
            lambda: self._gripper_seq > baseline and self._gripper_raw is not None,
            _ARM.DEFAULT_FEEDBACK_TIMEOUT_SECONDS,
            "fresh gripper state",
        )
        return parse_gripper_state(json.loads(self._gripper_raw))

    def _ensure_head_pipeline(self) -> None:
        if self._head_ready:
            return
        probe = _load_sibling("camera", "head_bottle_probe")
        self._basket = _load_sibling("camera", "detect_color_targets")
        self._geometry = _load_sibling("camera", "head_bottle_geometry")
        import numpy

        global_vars, list_outputs, open_consumer_fn = probe._load_sdk()
        settings = probe._resolve_camera_settings(global_vars)
        outputs = probe._catalog_outputs(list_outputs, settings)
        color_consumer = probe._open_consumer(
            open_consumer_fn, probe._select_output(outputs, "color", "color"), "color"
        )
        self._head_consumers.append(("color", color_consumer))
        depth_consumer = probe._open_consumer(
            open_consumer_fn, probe._select_output(outputs, "depth", "depth"), "depth"
        )
        self._head_consumers.append(("depth", depth_consumer))
        self._intrinsics_raw = probe._get_intrinsics(color_consumer)
        CameraTransformer = probe._load_camera_transformer()
        self._transformer = CameraTransformer(numpy.array(probe.HEAD_OFFSET_DEG, dtype=float), self._node)
        self._identity = numpy.eye(4)
        self._pair_reader = _HeadPairReader(probe, color_consumer, depth_consumer)
        self._probe = probe
        self._head_ready = True

    def locate_basket(self, config: BasketDropConfig) -> Any:
        def fail(reason: str, *, frame: Any = None, record: Any = None) -> SimpleNamespace:
            return SimpleNamespace(located=False, point_base=None, reason=reason, frame=frame, record=record)

        try:
            self._ensure_head_pipeline()
            probe, basket, geometry = self._probe, self._basket, self._geometry

            deadline = time.monotonic() + LOCATE_TRANSFORM_TIMEOUT_S
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

            detector_config = basket.basket_detector_config(
                hsv_lower=config.hsv_lower,
                hsv_upper=config.hsv_upper,
                min_area=config.min_area,
                roi=config.detection_roi_px,
            )
            acquisition = basket.acquire_stable_yellow_basket(
                self._pair_reader, detector_config, basket.stable_config()
            )
            if not getattr(acquisition, "stable", False):
                return fail(
                    f"stable basket detection failed: {getattr(acquisition, 'reason', 'unknown')}",
                    frame=getattr(acquisition, "frame", None),
                )

            candidate = acquisition.candidate
            u, v = basket.basket_reference_pixel(candidate, rim_inset_px=config.rim_inset_px)
            frame = acquisition.frame
            try:
                depth_raw, depth_values = probe.sample_depth(frame.depth, u, v, config.depth_window)
            except ValueError as exc:
                return fail(f"depth sampling failed at rim pixel ({u:.1f}, {v:.1f}): {exc}", frame=frame)

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
                pixel=(u, v),
                depth_raw=depth_raw,
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
                "reference_pixel": [u, v],
                "depth_raw": depth_raw,
                "depth_values_used": list(depth_values),
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
            return fail(f"basket localization errored: {exc}")

    def close(self) -> None:
        consumers = list(self._head_consumers)
        self._head_consumers = []
        for _stream, consumer in reversed(consumers):
            close = getattr(consumer, "close", None)
            if callable(close):
                try:
                    close()
                except Exception:
                    pass
        super().close()


# ---------------------------------------------------------------------------
# CLI：默认 preview；单次确认过的投放用 `run --execute`。
# ---------------------------------------------------------------------------


def _json_text(value: Any) -> str:
    return json.dumps(value, separators=(",", ":"), ensure_ascii=True, allow_nan=False)


def _preview_text(config_path: str) -> str:
    lines = [
        "Head-camera basket-drop preview",
        "  mode: preview",
        "  planned_stages: " + " -> ".join(STAGES),
        (
            "  limits: "
            f"per-step<= {STEP_MAX_M}m (axis-at-a-time, rise first / descend last); "
            "targets stay inside the configured absolute envelope; one overall timeout; "
            "automatic neck pitch to the configured detection angle (neck-only payload, "
            f"<= {HEAD_PITCH_STEP_MAX_DEG} deg per settle-gated step) before detection; "
            "two pre-motion localizations must agree before any motion; release only "
            "after every step's full dual-arm feedback passed"
        ),
        "  precondition: right gripper feedback must sit inside the calibrated holding band",
        "  end_state: gripper open, bottle released, arm at the calibrated retract position",
        "  abort_semantics: an abort stops publishing further targets only; e-stop is the field guard",
        f"  config: {config_path}",
        "  topics: " + _json_text(drop_interface_names()),
        "  gates: --execute absent",
        "  no action: no EEF publication and no gripper publication",
    ]
    return "\n".join(lines)


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Head-camera-guided yellow-basket drop.")
    parser.add_argument("mode", nargs="?", default="preview", choices=("preview", "run"))
    parser.add_argument("--execute", action="store_true", help="confirm physical execution")
    parser.add_argument("--config", default="right/basket_drop.json", help="calibration configuration path")
    parser.add_argument("--workcell-id", default="s2-test-point3-baskets")
    parser.add_argument("--evidence-dir", default=None)
    args = parser.parse_args(argv)

    if args.mode == "preview":
        print(_preview_text(args.config))
        return 0
    if not args.execute:
        print(
            "Error: physical execution requires --execute after on-site monitoring and "
            "physical emergency-stop readiness are confirmed.",
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
        evidence_dir = Path.home() / "Documents" / "AutolifeXAIR" / "evidence" / "basket_drop" / stamp

    runtime = None
    try:
        runtime = _RosRuntime(drop_interface_names(), evidence_dir)
        runtime.preflight_interfaces()
        result = run_drop(config, runtime)
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
