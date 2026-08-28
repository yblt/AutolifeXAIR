#!/usr/bin/env python3
"""只读的头部相机瓶子定位探针
（``openspec/changes/add-head-camera-assisted-positioning`` 的 task 2）。

两个子命令：

- ``capture``：从头部 RGB-D SHM 目录保存 N 对配对的彩色/深度帧
  （外加内参）。不需要 ROS。
- ``locate``：对 N 个样本中的每一个，等待新鲜的
  ``CameraTransformer`` 基座<-相机变换，轮询一对新鲜的彩色/深度
  帧，运行离线瓶子检测器，把参考像素经采样深度反投影，再用
  ``head_bottle_geometry.locate_bottle_in_base`` 变换到基座坐标。

本探针严格只读：从不构造 ROS publisher，从不发送运动指令。其全部
ROS 活动只有两个订阅（``CameraTransformer`` 内部的关节状态订阅，
以及一个只记录每条关节状态消息"到达时刻"的本地订阅——本模块从不
解析该话题的载荷）。

所有 ROS、NumPy、机器人 SDK 和视觉相关的 import 都推迟到函数
内部，因此 ``--help`` 和下方的纯辅助函数（``sample_depth``、
``load_detector_config``）在没装这些依赖的开发机上也能工作。
"""

from __future__ import annotations

import argparse
import json
import os
import statistics
import sys
import time
from collections.abc import Mapping, Sequence
from pathlib import Path
from typing import Any


# 显式的非零退出码让 shell 脚本能区分前置条件失败（或部分成功的
# `locate` 运行）与完全成功的运行。
EXIT_OK = 0
EXIT_UNEXPECTED = 1
EXIT_ARGS = 2
EXIT_SDK = 3
EXIT_CATALOG = 4
EXIT_OUTPUT = 5
EXIT_SHM = 6
EXIT_FRAME = 7
EXIT_INTRINSICS = 8
EXIT_CONFIG = 9
EXIT_OUTPUT_EXISTS = 10
EXIT_ROS = 11
EXIT_JOINT_STATE = 12
EXIT_DETECTION = 13
EXIT_LOCALIZATION = 14

# 来源：AutoLife SDK kinematics/robot_v2_2.json 的 head_offset，已在
# 机上现场验证（见 openspec/changes/add-head-camera-assisted-positioning/
# design.md 决策 #1）。单位为度；由 CameraTransformer 消费，其内部
# 完成度->弧度转换。不作为 CLI 选项暴露。
HEAD_OFFSET_DEG = (0.0, -1.0, 4.0)

_ROS_SPIN_TIMEOUT_S = 0.05
_DEFAULT_POLL_INTERVAL_S = 0.05


class ProbeError(Exception):
    """预期内的、可操作的探针失败，带独立退出码。"""

    def __init__(self, code: int, message: str) -> None:
        super().__init__(message)
        self.code = code
        self.message = message


# ---------------------------------------------------------------------------
# 同目录模块加载（仅限纯 Python 安全模块：模块作用域不含
# cv2/numpy/ROS）。按绝对文件路径加载，行为不依赖本脚本自身的
# 调用方式或包解析。
# ---------------------------------------------------------------------------

_MODULE_CACHE: dict[str, Any] = {}


def _load_sibling_module(name: str) -> Any:
    if name in _MODULE_CACHE:
        return _MODULE_CACHE[name]
    import importlib.util

    module_path = Path(__file__).resolve().parent / f"{name}.py"
    spec = importlib.util.spec_from_file_location(f"xr_head_bottle_probe_{name}", module_path)
    if spec is None or spec.loader is None:
        raise ProbeError(EXIT_UNEXPECTED, f"could not load sibling module {name!r} from {module_path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    try:
        spec.loader.exec_module(module)
    except Exception as exc:  # pragma: no cover - defensive
        raise ProbeError(EXIT_UNEXPECTED, f"sibling module {name!r} failed to import: {exc}") from exc
    _MODULE_CACHE[name] = module
    return module


def _detector_module() -> Any:
    return _load_sibling_module("detect_head_bottle")


def _geometry_module() -> Any:
    return _load_sibling_module("head_bottle_geometry")


# ---------------------------------------------------------------------------
# 纯的、可离线测试的原语（不 import cv2/numpy/ROS）。
# ---------------------------------------------------------------------------


class DepthSampleError(ValueError):
    """请求的深度窗口没有产生有效读数时抛出。"""


def sample_depth(depth: Any, u: float, v: float, window: int) -> tuple[int, list[int]]:
    """返回像素 ``(u, v)`` 附近一个稳健的原始 uint16 深度读数。

    ``depth`` 只需支持 ``depth.shape``（类似 ``(高, 宽)`` 的二元组）
    和 ``depth[row][col]`` 索引；普通嵌套列表和 NumPy uint16 数组都
    满足此契约，因此本函数无需 NumPy 即可单测。

    返回 ``(selected_raw_value, raw_values_used)``，其中
    ``raw_values_used`` 是窗口截断到数组边界后其中非零原始读数的
    有序列表。非零读数为偶数个时返回中间两值中较小者（绝不插值
    取平均），因此结果永远是实际观测到的原始样本之一。

    ``window`` 不是 <= 9 的正奇数、截断后的窗口为空、或窗口内所有
    候选读数均为零（无效深度）时，抛出 :class:`DepthSampleError`
    （``ValueError`` 的子类）。
    """

    if not isinstance(window, int) or isinstance(window, bool):
        raise DepthSampleError("window must be an int")
    if window <= 0 or window % 2 == 0:
        raise DepthSampleError("window must be a positive odd integer")
    if window > 9:
        raise DepthSampleError("window must be <= 9")

    shape = getattr(depth, "shape", None)
    if shape is None:
        raise DepthSampleError("depth must expose a shape")
    try:
        height, width = int(shape[0]), int(shape[1])
    except (TypeError, ValueError, IndexError) as exc:
        raise DepthSampleError("depth shape must be a 2-tuple-like (height, width)") from exc
    if height <= 0 or width <= 0:
        raise DepthSampleError("depth shape must be positive")

    row = round(float(v))
    col = round(float(u))
    half = window // 2
    row_start = max(0, row - half)
    row_end = min(height - 1, row + half)
    col_start = max(0, col - half)
    col_end = min(width - 1, col + half)
    if row_start > row_end or col_start > col_end:
        raise DepthSampleError("depth window lies outside the array bounds")

    raw_values: list[int] = []
    for row_index in range(row_start, row_end + 1):
        depth_row = depth[row_index]
        for col_index in range(col_start, col_end + 1):
            raw_values.append(int(depth_row[col_index]))

    non_zero = sorted(value for value in raw_values if value != 0)
    if not non_zero:
        raise DepthSampleError("all depth samples in the window are zero (invalid depth)")
    selected = int(statistics.median_low(non_zero))
    return selected, non_zero


_REQUIRED_CONFIG_KEYS = frozenset({"config_id", "roi", "hsv_ranges"})
_OPTIONAL_CONFIG_KEYS = frozenset(
    {
        "morphology_kernel",
        "morphology_iterations",
        "morphology_operation",
        "min_area",
        "max_area",
        "min_width",
        "max_width",
        "min_height",
        "max_height",
        "min_aspect_ratio",
        "max_aspect_ratio",
        "min_fill_ratio",
        "max_fill_ratio",
        "reject_roi_boundary",
    }
)
_ALL_CONFIG_KEYS = _REQUIRED_CONFIG_KEYS | _OPTIONAL_CONFIG_KEYS
_HSV_RANGE_KEYS = frozenset({"h_min", "h_max", "s_min", "s_max", "v_min", "v_max"})


def load_detector_config(mapping: Mapping[str, Any]) -> Any:
    """把严格 JSON 形态的 mapping 解析为 ``BottleDetectorConfig``。

    这是纯的结构/类型整形函数：校验 schema（必需/可选/未知键）并用
    ``examples/camera/detect_head_bottle.py`` 里的
    ``ROI``/``HSVRange``/``BottleDetectorConfig`` 构造实例，但不重复
    那些 dataclass 构造器已做的范围校验。任何结构性问题都抛
    ``ValueError``。
    """

    if not isinstance(mapping, Mapping):
        raise ValueError("detector config must be a JSON object")

    unknown = set(mapping) - _ALL_CONFIG_KEYS
    if unknown:
        raise ValueError(f"unknown detector config keys: {sorted(unknown)}")
    missing = _REQUIRED_CONFIG_KEYS - set(mapping)
    if missing:
        raise ValueError(f"missing required detector config keys: {sorted(missing)}")

    detect = _detector_module()

    config_id = mapping["config_id"]
    if not isinstance(config_id, str) or not config_id.strip():
        raise ValueError("config_id must be a non-empty string")

    roi_value = mapping["roi"]
    if not isinstance(roi_value, (list, tuple)) or len(roi_value) != 4:
        raise ValueError("roi must be a 4-element [x, y, width, height] array")
    roi = detect.ROI(*roi_value)

    hsv_ranges_value = mapping["hsv_ranges"]
    if not isinstance(hsv_ranges_value, (list, tuple)) or len(hsv_ranges_value) == 0:
        raise ValueError("hsv_ranges must be a non-empty array")
    hsv_ranges: list[Any] = []
    for index, item in enumerate(hsv_ranges_value):
        if not isinstance(item, Mapping):
            raise ValueError(f"hsv_ranges[{index}] must be a JSON object")
        unknown_hsv = set(item) - _HSV_RANGE_KEYS
        if unknown_hsv:
            raise ValueError(f"hsv_ranges[{index}] has unknown keys: {sorted(unknown_hsv)}")
        missing_hsv = _HSV_RANGE_KEYS - set(item)
        if missing_hsv:
            raise ValueError(f"hsv_ranges[{index}] is missing keys: {sorted(missing_hsv)}")
        hsv_ranges.append(
            detect.HSVRange(
                h_min=item["h_min"],
                h_max=item["h_max"],
                s_min=item["s_min"],
                s_max=item["s_max"],
                v_min=item["v_min"],
                v_max=item["v_max"],
            )
        )

    kwargs: dict[str, Any] = {
        "config_id": config_id,
        "roi": roi,
        "hsv_ranges": tuple(hsv_ranges),
    }
    if "morphology_kernel" in mapping:
        kernel = mapping["morphology_kernel"]
        if not isinstance(kernel, (list, tuple)) or len(kernel) != 2:
            raise ValueError("morphology_kernel must be a 2-element array")
        kwargs["morphology_kernel"] = (kernel[0], kernel[1])
    for key in _OPTIONAL_CONFIG_KEYS - {"morphology_kernel"}:
        if key in mapping:
            kwargs[key] = mapping[key]

    try:
        return detect.BottleDetectorConfig(**kwargs)
    except TypeError as exc:
        raise ValueError(f"invalid detector config: {exc}") from exc


# ---------------------------------------------------------------------------
# JSON / 描述符辅助函数（仅标准库）。
# ---------------------------------------------------------------------------


def _field(value: Any, *names: str) -> Any:
    for name in names:
        if isinstance(value, Mapping) and name in value:
            item = value[name]
            if item is not None:
                return item
        try:
            item = getattr(value, name)
        except AttributeError:
            continue
        if item is not None:
            return item
    return None


def _text(value: Any) -> str | None:
    if value is None:
        return None
    enum_value = getattr(value, "value", value)
    result = str(enum_value).strip()
    return result or None


def _global_value(global_vars: Any, *names: str) -> Any:
    for name in names:
        if isinstance(global_vars, Mapping) and name in global_vars:
            value = global_vars[name]
            if value is not None:
                return value
        try:
            value = getattr(global_vars, name)
        except AttributeError:
            continue
        if value is not None:
            return value
    return None


def _json_value(value: Any) -> Any:
    """把 SDK/numpy 标量值转成确定性的 JSON 安全值。"""

    if value is None or isinstance(value, (bool, int, float, str)):
        return value
    if isinstance(value, Mapping):
        return {str(key): _json_value(item) for key, item in value.items()}
    if isinstance(value, Sequence) and not isinstance(value, (str, bytes, bytearray)):
        return [_json_value(item) for item in value]
    item = getattr(value, "item", None)
    if callable(item):
        try:
            return _json_value(item())
        except Exception:
            pass
    return str(value)


# ---------------------------------------------------------------------------
# SDK / 共享内存辅助函数（SDK 惰性 import；两个子命令共用）。
# ---------------------------------------------------------------------------


def _load_sdk() -> tuple[Any, Any, Any]:
    try:
        from autolife_robot_sdk import GLOBAL_VARS
        from autolife_robot_sdk.utils import (
            list_camera_shm_outputs,
            open_camera_shm_consumer,
        )
    except Exception as exc:  # pragma: no cover - depends on target environment
        raise ProbeError(
            EXIT_SDK,
            "AutoLife SDK is unavailable; activate the preinstalled robot environment "
            f"before running this probe ({exc})",
        ) from exc
    return GLOBAL_VARS, list_camera_shm_outputs, open_camera_shm_consumer


def _lazy_numpy() -> Any:
    try:
        import numpy
    except Exception as exc:  # pragma: no cover - depends on target environment
        raise ProbeError(EXIT_SDK, f"NumPy is unavailable: {exc}") from exc
    return numpy


def _resolve_camera_settings(global_vars: Any) -> dict[str, Any]:
    model = _global_value(global_vars, "ACTIVE_ROBOT_MODEL", "ROBOT_MODEL", "robot_model")
    version = _global_value(global_vars, "ACTIVE_ROBOT_VERSION", "ROBOT_VERSION", "robot_version")
    module = _global_value(global_vars, "ACTIVE_CAMERA_MODULE", "HEAD_CAMERA_MODULE", "camera_module")
    if model is None or version is None:
        raise ProbeError(EXIT_SDK, "robot model/version are not available in GLOBAL_VARS")
    return {
        "robot_model": model,
        "robot_version": version,
        "module": str(module or "mod_camera_rgbd_head"),
    }


def _catalog_outputs(list_outputs: Any, settings: dict[str, Any]) -> list[Any]:
    try:
        outputs = list(
            list_outputs(
                settings["robot_model"],
                settings["robot_version"],
                module_name=settings["module"],
            )
        )
    except Exception as exc:
        raise ProbeError(
            EXIT_CATALOG,
            f"camera catalog lookup failed for module {settings['module']!r}: {exc}",
        ) from exc
    if not outputs:
        raise ProbeError(
            EXIT_CATALOG, f"camera catalog contains no outputs for module {settings['module']!r}"
        )
    return outputs


def _select_output(outputs: list[Any], requested_name: str, stream: str) -> Any:
    matches = [
        output
        for output in outputs
        if (_text(_field(output, "output_name", "name")) or "").casefold() == requested_name.casefold()
    ]
    if len(matches) == 1:
        return matches[0]
    available = [_text(_field(output, "output_name", "name")) or "<unnamed>" for output in outputs]
    raise ProbeError(
        EXIT_OUTPUT,
        f"{stream} output {requested_name!r} is not uniquely present in the catalog; "
        "available output_name values: " + ", ".join(available),
    )


def _open_consumer(open_consumer_fn: Any, output: Any, stream: str) -> Any:
    output_name = _text(_field(output, "output_name", "name")) or stream
    try:
        consumer = open_consumer_fn(output, name=f"xr_head_bottle_probe_{stream}")
    except Exception as exc:
        raise ProbeError(
            EXIT_SHM, f"{stream} output {output_name!r} SHM open failed: {exc}"
        ) from exc
    if consumer is None or not callable(getattr(consumer, "get_latest", None)):
        raise ProbeError(EXIT_SHM, f"{stream} output {output_name!r} returned no usable consumer")
    return consumer


def _close_consumers(consumers: list[tuple[str, Any]]) -> None:
    errors: list[str] = []
    for stream, consumer in reversed(consumers):
        close = getattr(consumer, "close", None)
        if not callable(close):
            continue
        try:
            close()
        except Exception as exc:  # pragma: no cover - SDK-specific cleanup failure
            errors.append(f"{stream}: {exc}")
    if errors:
        print("WARNING: consumer cleanup failed: " + "; ".join(errors), file=sys.stderr)


def _get_latest(consumer: Any, stream: str) -> tuple[Any, Any, Any] | None:
    try:
        sample = consumer.get_latest(nonblock=True, with_meta=True)
    except Exception as exc:
        raise ProbeError(EXIT_FRAME, f"{stream} frame read failed: {exc}") from exc
    if sample is None:
        return None
    if not isinstance(sample, Sequence) or isinstance(sample, (str, bytes)) or len(sample) != 3:
        raise ProbeError(EXIT_FRAME, f"{stream} consumer returned an invalid frame result")
    image, frame_id, metadata = sample
    if image is None or frame_id is None:
        return None
    return image, frame_id, metadata


def _poll_matched_pair(
    color_consumer: Any, depth_consumer: Any, timeout: float, interval: float
) -> tuple[tuple[Any, Any, Any], tuple[Any, Any, Any]]:
    """轮询两个消费者的最新帧，直到二者 frame id 匹配。"""

    deadline = time.monotonic() + max(timeout, 0.0)
    while True:
        color_sample = _get_latest(color_consumer, "color")
        depth_sample = _get_latest(depth_consumer, "depth")
        if color_sample is not None and depth_sample is not None:
            if int(color_sample[1]) == int(depth_sample[1]):
                return color_sample, depth_sample
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise ProbeError(
                EXIT_FRAME,
                f"timed out after {timeout:.3f}s waiting for a matching color/depth frame_id pair",
            )
        if interval:
            time.sleep(min(interval, remaining))


def _get_intrinsics(color_consumer: Any) -> Any:
    get_intrinsics = getattr(color_consumer, "get_intrinsics", None)
    if not callable(get_intrinsics):
        raise ProbeError(EXIT_INTRINSICS, "color consumer does not expose get_intrinsics()")
    try:
        raw = get_intrinsics()
    except Exception as exc:
        raise ProbeError(EXIT_INTRINSICS, f"get_intrinsics() failed: {exc}") from exc
    if raw is None:
        raise ProbeError(EXIT_INTRINSICS, "color consumer get_intrinsics() returned None")
    return raw


def _save_frame_pair(
    path: Path, color_sample: tuple[Any, Any, Any], depth_sample: tuple[Any, Any, Any]
) -> dict[str, Any]:
    numpy = _lazy_numpy()
    color_image, color_frame_id, _color_meta = color_sample
    depth_image, depth_frame_id, _depth_meta = depth_sample
    captured_monotonic = time.monotonic()
    captured_unix = time.time()
    numpy.savez_compressed(
        path,
        color=color_image,
        depth=depth_image,
        color_frame_id=int(color_frame_id),
        depth_frame_id=int(depth_frame_id),
        captured_monotonic=captured_monotonic,
        captured_unix=captured_unix,
    )
    shape = getattr(color_image, "shape", None)
    width = int(shape[1]) if shape is not None and len(shape) > 1 else None
    height = int(shape[0]) if shape is not None and len(shape) > 0 else None
    return {
        "path": str(path),
        "color_frame_id": int(color_frame_id),
        "depth_frame_id": int(depth_frame_id),
        "captured_monotonic": captured_monotonic,
        "captured_unix": captured_unix,
        "width": width,
        "height": height,
    }


# ---------------------------------------------------------------------------
# capture
# ---------------------------------------------------------------------------


def _run_capture(args: argparse.Namespace) -> dict[str, Any]:
    if args.frames < 1:
        raise ProbeError(EXIT_CONFIG, "--frames must be >= 1")
    if args.timeout <= 0:
        raise ProbeError(EXIT_CONFIG, "--timeout must be > 0")
    if args.interval < 0:
        raise ProbeError(EXIT_CONFIG, "--interval must be >= 0")

    output_dir = Path(args.output_dir).expanduser()
    output_dir.mkdir(parents=True, exist_ok=True)
    frame_paths = [output_dir / f"frame_{index:03d}.npz" for index in range(args.frames)]
    intrinsics_path = output_dir / "intrinsics.json"
    for candidate in (*frame_paths, intrinsics_path):
        if candidate.exists():
            raise ProbeError(EXIT_OUTPUT_EXISTS, f"refusing to overwrite existing output file: {candidate}")

    global_vars, list_outputs, open_consumer_fn = _load_sdk()
    settings = _resolve_camera_settings(global_vars)
    outputs = _catalog_outputs(list_outputs, settings)
    color_output = _select_output(outputs, "color", "color")
    depth_output = _select_output(outputs, "depth", "depth")

    consumers: list[tuple[str, Any]] = []
    try:
        color_consumer = _open_consumer(open_consumer_fn, color_output, "color")
        consumers.append(("color", color_consumer))
        depth_consumer = _open_consumer(open_consumer_fn, depth_output, "depth")
        consumers.append(("depth", depth_consumer))

        intrinsics_raw = _get_intrinsics(color_consumer)

        frame_records: list[dict[str, Any]] = []
        for path in frame_paths:
            color_sample, depth_sample = _poll_matched_pair(
                color_consumer, depth_consumer, args.timeout, args.interval
            )
            frame_records.append(_save_frame_pair(path, color_sample, depth_sample))

        last = frame_records[-1]
        intrinsics_record = {
            "intrinsics": _json_value(intrinsics_raw),
            "width": last["width"],
            "height": last["height"],
        }
        intrinsics_path.write_text(
            json.dumps(intrinsics_record, ensure_ascii=False, sort_keys=True), encoding="utf-8"
        )

        return {
            "status": "ok",
            "output_dir": str(output_dir),
            "robot_model": _json_value(settings["robot_model"]),
            "robot_version": _json_value(settings["robot_version"]),
            "module": settings["module"],
            "frames": frame_records,
            "intrinsics_path": str(intrinsics_path),
        }
    finally:
        _close_consumers(consumers)


# ---------------------------------------------------------------------------
# locate
# ---------------------------------------------------------------------------


def _load_ros() -> Any:
    try:
        import rclpy
    except Exception as exc:  # pragma: no cover - depends on target environment
        raise ProbeError(EXIT_ROS, f"ROS 2 (rclpy) is unavailable: {exc}") from exc
    return rclpy


def _load_camera_transformer() -> Any:
    # 厂商 CameraTransformer 用环境变量 ROBOT_ID 拼关节状态话题。机器编号
    # 一律按主机名推导并写回环境，使厂商库与本脚本始终一致；环境里原有的
    # 值（哪怕写错，如 274）一律覆盖——症状曾是 "[KDL] No joints_pos yet"
    # 直到超时。
    hostname_id = os.uname().nodename.rsplit("-", 1)[-1]
    previous = os.environ.get("ROBOT_ID", "").strip()
    if previous and previous != hostname_id:
        print(
            f"WARNING: overriding ROBOT_ID={previous!r} with hostname suffix {hostname_id!r}",
            file=sys.stderr,
        )
    os.environ["ROBOT_ID"] = hostname_id
    try:
        from autolife_robot_vision.camera_transformer import CameraTransformer
    except Exception as exc:  # pragma: no cover - depends on target environment
        raise ProbeError(
            EXIT_ROS, f"autolife_robot_vision.camera_transformer is unavailable: {exc}"
        ) from exc
    return CameraTransformer


def _joint_state_topic_name() -> str:
    domain = os.environ.get("ROS_DOMAIN_ID", "0")
    robot_id = os.uname().nodename.rsplit("-", 1)[-1]  # 机器编号一律按主机名推导，不信环境变量
    return f"/topic_arm_whole_body_and_gripper_current_joints_status_{domain}_{robot_id}"


def _wait_for_fresh_transform(
    rclpy_module: Any,
    node: Any,
    transformer: Any,
    joint_state: dict[str, float | None],
    identity_matrix: Any,
    max_joint_age: float,
    deadline: float,
    topic_name: str,
) -> tuple[Any, float]:
    """spin 到新鲜的基座<-相机变换可用为止，否则 fail-closed。

    从不解析关节状态消息内容；``joint_state`` 只记录最近一条消息的
    单调时钟到达时刻。
    """

    while True:
        rclpy_module.spin_once(node, timeout_sec=_ROS_SPIN_TIMEOUT_S)
        now = time.monotonic()
        last_arrival = joint_state.get("last_arrival")
        age = (now - last_arrival) if last_arrival is not None else None
        if age is not None and age <= max_joint_age:
            transform = transformer.compute_object_pose_in_base(identity_matrix)
            if transform is not None:
                return transform, age
        if now >= deadline:
            if last_arrival is None:
                raise ProbeError(
                    EXIT_JOINT_STATE, f"no joint state message has arrived on topic {topic_name!r}"
                )
            raise ProbeError(
                EXIT_JOINT_STATE,
                "timed out waiting for a fresh base<-camera transform "
                f"(joint_state_age_s={age:.3f}, max_joint_age={max_joint_age:.3f})",
            )


def _sample_error_record(index: int, error: ProbeError) -> dict[str, Any]:
    return {
        "index": index,
        "status": "failed",
        "failure_code": str(error.code),
        "reason": error.message,
        "detection": None,
        "position": None,
        "depth_sampling": None,
        "color_frame_id": None,
        "depth_frame_id": None,
        "transform": None,
        "joint_state_age_s": None,
    }


def _summarize_positions(n_requested: int, points: list[Mapping[str, float]]) -> dict[str, Any]:
    summary: dict[str, Any] = {"n_requested": n_requested, "n_located": len(points)}
    for axis in ("x", "y", "z"):
        values = [point[axis] for point in points]
        summary[axis] = {
            "values": values,
            "min": min(values) if values else None,
            "max": max(values) if values else None,
            "spread": (max(values) - min(values)) if values else None,
        }
    return summary


def _positive_odd_window(value: str) -> int:
    try:
        parsed = int(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("must be an integer") from exc
    if parsed <= 0 or parsed % 2 == 0 or parsed > 9:
        raise argparse.ArgumentTypeError("must be a positive odd integer <= 9")
    return parsed


def _run_locate(args: argparse.Namespace) -> tuple[dict[str, Any], int]:
    if args.samples < 1:
        raise ProbeError(EXIT_CONFIG, "--samples must be >= 1")
    if args.timeout <= 0:
        raise ProbeError(EXIT_CONFIG, "--timeout must be > 0")
    if args.max_joint_age <= 0:
        raise ProbeError(EXIT_CONFIG, "--max-joint-age must be > 0")
    if args.min_depth <= 0 or args.max_depth <= args.min_depth:
        raise ProbeError(EXIT_CONFIG, "--min-depth must be > 0 and less than --max-depth")

    config_path = Path(args.config).expanduser()
    try:
        raw_config = json.loads(config_path.read_text(encoding="utf-8"))
    except Exception as exc:
        raise ProbeError(EXIT_CONFIG, f"could not read/parse --config {config_path}: {exc}") from exc
    try:
        detector_config = load_detector_config(raw_config)
    except Exception as exc:
        raise ProbeError(EXIT_CONFIG, f"invalid detector config {config_path}: {exc}") from exc

    save_frames_dir: Path | None = None
    if args.save_frames is not None:
        save_frames_dir = Path(args.save_frames).expanduser()
        save_frames_dir.mkdir(parents=True, exist_ok=True)
        for index in range(args.samples):
            candidate = save_frames_dir / f"frame_{index:03d}.npz"
            if candidate.exists():
                raise ProbeError(
                    EXIT_OUTPUT_EXISTS, f"refusing to overwrite existing output file: {candidate}"
                )

    global_vars, list_outputs, open_consumer_fn = _load_sdk()
    settings = _resolve_camera_settings(global_vars)
    outputs = _catalog_outputs(list_outputs, settings)
    color_output = _select_output(outputs, "color", "color")
    depth_output = _select_output(outputs, "depth", "depth")

    geometry = _geometry_module()
    detect = _detector_module()

    numpy = _lazy_numpy()
    rclpy = _load_ros()
    try:
        from std_msgs.msg import String
    except Exception as exc:  # pragma: no cover - depends on target environment
        raise ProbeError(EXIT_ROS, f"std_msgs.msg.String is unavailable: {exc}") from exc
    CameraTransformer = _load_camera_transformer()

    topic_name = _joint_state_topic_name()

    consumers: list[tuple[str, Any]] = []
    node: Any = None
    rclpy_initialized = False
    try:
        color_consumer = _open_consumer(open_consumer_fn, color_output, "color")
        consumers.append(("color", color_consumer))
        depth_consumer = _open_consumer(open_consumer_fn, depth_output, "depth")
        consumers.append(("depth", depth_consumer))

        intrinsics_raw = _get_intrinsics(color_consumer)
        fx = _field(intrinsics_raw, "fx")
        fy = _field(intrinsics_raw, "fy")
        ppx = _field(intrinsics_raw, "ppx")
        ppy = _field(intrinsics_raw, "ppy")
        if fx is None or fy is None or ppx is None or ppy is None:
            raise ProbeError(EXIT_INTRINSICS, "intrinsics is missing one of fx/fy/ppx/ppy")
        model_id = _field(intrinsics_raw, "model_id")
        coeffs = _field(intrinsics_raw, "coeffs")

        rclpy.init(args=None)
        rclpy_initialized = True
        node = rclpy.create_node("xr_head_bottle_probe")
        joint_state: dict[str, float | None] = {"last_arrival": None}

        def _on_joint_state(_msg: Any) -> None:
            # 绝不解析 msg.data：只记录到达时刻。
            joint_state["last_arrival"] = time.monotonic()

        node.create_subscription(String, topic_name, _on_joint_state, 10)
        head_offset = numpy.array(HEAD_OFFSET_DEG, dtype=float)
        transformer = CameraTransformer(head_offset, node)
        identity_matrix = numpy.eye(4)

        sample_records: list[dict[str, Any]] = []
        failure_exit_codes: list[int] = []

        for sample_index in range(args.samples):
            deadline = time.monotonic() + args.timeout
            try:
                transform_matrix, joint_age = _wait_for_fresh_transform(
                    rclpy,
                    node,
                    transformer,
                    joint_state,
                    identity_matrix,
                    args.max_joint_age,
                    deadline,
                    topic_name,
                )
                remaining = max(0.05, deadline - time.monotonic())
                color_sample, depth_sample = _poll_matched_pair(
                    color_consumer, depth_consumer, remaining, _DEFAULT_POLL_INTERVAL_S
                )
            except ProbeError as exc:
                sample_records.append(_sample_error_record(sample_index, exc))
                failure_exit_codes.append(exc.code)
                continue

            color_image, color_frame_id, _color_meta = color_sample
            depth_image, depth_frame_id, _depth_meta = depth_sample
            transform_rows = [list(row) for row in transform_matrix]

            if save_frames_dir is not None:
                frame_path = save_frames_dir / f"frame_{sample_index:03d}.npz"
                try:
                    if frame_path.exists():
                        raise ProbeError(
                            EXIT_OUTPUT_EXISTS,
                            f"refusing to overwrite existing output file: {frame_path}",
                        )
                    _save_frame_pair(frame_path, color_sample, depth_sample)
                except ProbeError as exc:
                    sample_records.append(_sample_error_record(sample_index, exc))
                    failure_exit_codes.append(exc.code)
                    continue

            frame = {
                "image": color_image,
                "frame_id": int(color_frame_id),
                "received_at": time.time(),
                "color_order": "BGR",
            }
            detection = detect.detect_head_bottle(frame, detector_config)
            detection_record = detection.to_record()

            if not detection.detected:
                sample_records.append(
                    {
                        "index": sample_index,
                        "status": "detection_failed",
                        "failure_code": "detection_failed",
                        "reason": detection.reason,
                        "detection": detection_record,
                        "position": None,
                        "depth_sampling": None,
                        "color_frame_id": int(color_frame_id),
                        "depth_frame_id": int(depth_frame_id),
                        "transform": transform_rows,
                        "joint_state_age_s": joint_age,
                    }
                )
                failure_exit_codes.append(EXIT_DETECTION)
                continue

            u, v = detection.reference_pixel
            shape = getattr(color_image, "shape", None)
            if shape is None or len(shape) < 2:
                sample_records.append(
                    {
                        "index": sample_index,
                        "status": "localization_failed",
                        "failure_code": "invalid_frame_shape",
                        "reason": "color frame has no usable shape",
                        "detection": detection_record,
                        "position": None,
                        "depth_sampling": None,
                        "color_frame_id": int(color_frame_id),
                        "depth_frame_id": int(depth_frame_id),
                        "transform": transform_rows,
                        "joint_state_age_s": joint_age,
                    }
                )
                failure_exit_codes.append(EXIT_LOCALIZATION)
                continue

            intrinsics = geometry.CameraIntrinsics(
                fx=fx, fy=fy, cx=ppx, cy=ppy, width=int(shape[1]), height=int(shape[0])
            )

            try:
                depth_value, depth_values_used = sample_depth(depth_image, u, v, args.depth_window)
                depth_error = None
            except ValueError as exc:
                depth_value = None
                depth_values_used = []
                depth_error = str(exc)

            depth_sampling_record = {
                "window": args.depth_window,
                "pixel": [u, v],
                "raw_values_used": depth_values_used,
                "selected_raw_value": depth_value,
                "error": depth_error,
            }

            if depth_value is None:
                sample_records.append(
                    {
                        "index": sample_index,
                        "status": "localization_failed",
                        "failure_code": "invalid_depth",
                        "reason": depth_error,
                        "detection": detection_record,
                        "position": None,
                        "depth_sampling": depth_sampling_record,
                        "color_frame_id": int(color_frame_id),
                        "depth_frame_id": int(depth_frame_id),
                        "transform": transform_rows,
                        "joint_state_age_s": joint_age,
                    }
                )
                failure_exit_codes.append(EXIT_LOCALIZATION)
                continue

            limits = geometry.DepthLimits(min_depth_m=args.min_depth, max_depth_m=args.max_depth)
            position = geometry.locate_bottle_in_base(
                pixel=(u, v),
                depth_raw=depth_value,
                intrinsics=intrinsics,
                transform=transform_matrix,
                limits=limits,
                joint_state_age_s=joint_age,
                max_joint_state_age_s=args.max_joint_age,
            )
            position_record = position.to_record()
            located = position.located
            sample_records.append(
                {
                    "index": sample_index,
                    "status": "located" if located else "localization_failed",
                    "failure_code": position.failure_code,
                    "reason": position.reason,
                    "detection": detection_record,
                    "position": position_record,
                    "depth_sampling": depth_sampling_record,
                    "color_frame_id": int(color_frame_id),
                    "depth_frame_id": int(depth_frame_id),
                    "transform": transform_rows,
                    "joint_state_age_s": joint_age,
                }
            )
            if not located:
                failure_exit_codes.append(EXIT_LOCALIZATION)

        located_points = [
            record["position"]["point_base"] for record in sample_records if record["status"] == "located"
        ]
        summary = _summarize_positions(args.samples, located_points)

        result = {
            "status": "ok" if summary["n_located"] == args.samples else "partial",
            "settings": {
                "robot_model": _json_value(settings["robot_model"]),
                "robot_version": _json_value(settings["robot_version"]),
                "module": settings["module"],
                "samples": args.samples,
                "timeout": args.timeout,
                "max_joint_age": args.max_joint_age,
                "depth_window": args.depth_window,
                "min_depth": args.min_depth,
                "max_depth": args.max_depth,
                "head_offset_deg": list(HEAD_OFFSET_DEG),
                "joint_state_topic": topic_name,
                "save_frames": str(save_frames_dir) if save_frames_dir is not None else None,
            },
            "intrinsics": {
                "model_id": _json_value(model_id),
                "fx": _json_value(fx),
                "fy": _json_value(fy),
                "ppx": _json_value(ppx),
                "ppy": _json_value(ppy),
                "coeffs": _json_value(coeffs),
            },
            "samples": sample_records,
            "summary": summary,
        }
        exit_code = EXIT_OK if not failure_exit_codes else failure_exit_codes[0]
        return result, exit_code
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy_initialized:
            rclpy.shutdown()
        _close_consumers(consumers)


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Read-only head-camera bottle-positioning probe (capture/locate)."
    )
    subparsers = parser.add_subparsers(dest="command")

    capture = subparsers.add_parser(
        "capture",
        help="save N matched head color/depth frame pairs plus intrinsics (no ROS required)",
    )
    capture.add_argument("--output-dir", required=True, metavar="DIR")
    capture.add_argument("--frames", type=int, default=5, metavar="N")
    capture.add_argument("--timeout", type=float, default=5.0, metavar="SECONDS")
    capture.add_argument("--interval", type=float, default=0.2, metavar="SECONDS")

    locate = subparsers.add_parser(
        "locate",
        help="detect the bottle, back-project it, and transform it into the base frame",
    )
    locate.add_argument("--config", required=True, metavar="FILE")
    locate.add_argument("--samples", type=int, default=5, metavar="N")
    locate.add_argument("--timeout", type=float, default=10.0, metavar="SECONDS")
    locate.add_argument("--max-joint-age", type=float, default=1.0, metavar="SECONDS")
    locate.add_argument("--depth-window", type=_positive_odd_window, default=1, metavar="K")
    locate.add_argument("--min-depth", type=float, default=0.20, metavar="M")
    locate.add_argument("--max-depth", type=float, default=3.00, metavar="M")
    locate.add_argument("--save-frames", default=None, metavar="DIR")

    return parser


def main(argv: Sequence[str] | None = None) -> int:
    parser = _parser()
    args = parser.parse_args(argv)
    if args.command is None:
        parser.print_help()
        return EXIT_ARGS

    try:
        if args.command == "capture":
            result: dict[str, Any] = _run_capture(args)
            exit_code = EXIT_OK
        else:
            result, exit_code = _run_locate(args)
    except ProbeError as exc:
        print(f"ERROR[{exc.code}]: {exc.message}", file=sys.stderr)
        return exc.code
    except KeyboardInterrupt:
        print("ERROR[130]: interrupted while running the head-bottle probe", file=sys.stderr)
        return 130
    except Exception as exc:  # pragma: no cover - defensive boundary for SDK differences
        print(f"ERROR[{EXIT_UNEXPECTED}]: unexpected probe failure: {exc}", file=sys.stderr)
        return EXIT_UNEXPECTED

    print(json.dumps(result, ensure_ascii=False, sort_keys=True, default=_json_value))
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
