#!/usr/bin/env python3
"""通过 AutoLife SDK 相机目录读取解码后的手部相机帧。

本示例用 ``list_camera_shm_outputs`` 发现每个选中的手部相机，并用
``open_camera_shm_consumer`` 打开选中的目录输出。它不自行构造共享
内存名、不解码 JPEG 数据、不写图像文件，也不做任何物理执行动作。

同时请求两侧时各侧独立处理。一侧失败会保留在 JSON 结果里，另一侧
仍会尝试；任何一侧失败进程都以非零码退出。
"""

from __future__ import annotations

import argparse
import json
import math
import numbers
import sys
import time
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from typing import Any


# 稳定、显式的非零退出码让 shell 脚本能利用失败信息，也让 both 模式
# 输出中的两侧失败可区分。
EXIT_OK = 0
EXIT_UNEXPECTED = 1
EXIT_SDK = 3
EXIT_CATALOG = 4
EXIT_OUTPUT = 5
EXIT_SHM = 6
EXIT_FRAME = 7
EXIT_METADATA = 8


class CameraReadError(Exception):
    """预期内的、可操作的手部相机读取失败。"""

    def __init__(self, code: int, message: str) -> None:
        super().__init__(message)
        self.code = code
        self.message = message


@dataclass(frozen=True)
class DecodedBGRFrame:
    """一帧新鲜的左手解码 BGR 图像及本地接收时间戳。

    ``received_at`` 取自 reader 的本地单调时钟。它只适用于本地
    新鲜度检查，绝不能当作与机器人或其他设备同步的时间戳。
    """

    image: Any
    frame_id: Any
    received_at: float

    @property
    def receive_timestamp(self) -> float:
        """给偏好显式命名的调用方的兼容别名。"""

        return self.received_at


def _field(value: Any, *names: str, default: Any = None) -> Any:
    """从 SDK 描述符对象或 mapping 中读取一个字段。"""

    for name in names:
        if isinstance(value, Mapping) and name in value:
            return value[name]
        try:
            result = getattr(value, name)
        except AttributeError:
            continue
        if result is not None:
            return result
    return default


def _text(value: Any) -> str | None:
    """对 SDK 枚举和普通值返回可用的字符串。"""

    if value is None:
        return None
    enum_value = getattr(value, "value", value)
    result = str(enum_value).strip()
    return result or None


def _json_value(value: Any) -> Any:
    """转换常见的 numpy/SDK 值，保证 JSON 输出确定性。"""

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


def _global_value(global_vars: Any, *names: str) -> Any:
    """从 GLOBAL_VARS 取设置，不假设其具体类型。"""

    for name in names:
        if isinstance(global_vars, Mapping) and name in global_vars:
            return global_vars[name]
        try:
            value = getattr(global_vars, name)
        except AttributeError:
            continue
        if value is not None:
            return value
    return None


def _load_sdk() -> tuple[Any, Any, Any]:
    """惰性加载 SDK，让开发 PC 离线也能用 ``--help``。"""

    try:
        from autolife_robot_sdk import GLOBAL_VARS
        from autolife_robot_sdk.utils import (
            list_camera_shm_outputs,
            open_camera_shm_consumer,
        )
    except Exception as exc:  # pragma: no cover - depends on target environment
        raise CameraReadError(
            EXIT_SDK,
            "AutoLife SDK is unavailable; activate the preinstalled robot environment "
            f"before running this example ({exc})",
        ) from exc
    return GLOBAL_VARS, list_camera_shm_outputs, open_camera_shm_consumer


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Read a decoded left/right hand-camera frame from the SDK SHM catalog."
    )
    parser.add_argument(
        "--side",
        choices=("left", "right", "both"),
        default="both",
        help="hand camera side to read (default: both)",
    )
    parser.add_argument(
        "--robot-model",
        default=None,
        help="SDK robot model (default: GLOBAL_VARS.ACTIVE_ROBOT_MODEL)",
    )
    parser.add_argument(
        "--robot-version",
        default=None,
        help="SDK robot version (default: GLOBAL_VARS.ACTIVE_ROBOT_VERSION)",
    )
    parser.add_argument(
        "--output-name",
        default="decoded",
        help="catalog output_name (default: decoded; may explicitly select another output)",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=None,
        help="maximum seconds to wait per selected side (default: GLOBAL_VARS or 2.0)",
    )
    parser.add_argument(
        "--poll-interval",
        type=float,
        default=None,
        help="seconds between nonblocking polls (default: GLOBAL_VARS or 0.02)",
    )
    return parser


SIDE_MODULES = {
    "left": "mod_camera_hand_left",
    "right": "mod_camera_hand_right",
}


def _resolve_settings(args: argparse.Namespace, global_vars: Any) -> dict[str, Any]:
    model = getattr(args, "robot_model", None) or _global_value(
        global_vars, "ACTIVE_ROBOT_MODEL", "ROBOT_MODEL", "robot_model"
    )
    version = getattr(args, "robot_version", None) or _global_value(
        global_vars, "ACTIVE_ROBOT_VERSION", "ROBOT_VERSION", "robot_version"
    )
    timeout = getattr(args, "timeout", None)
    if timeout is None:
        timeout = _global_value(
            global_vars,
            "CAMERA_READ_TIMEOUT_SECONDS",
            "CAMERA_SHM_TIMEOUT_SECONDS",
            "camera_read_timeout",
        )
    poll_interval = getattr(args, "poll_interval", None)
    if poll_interval is None:
        poll_interval = _global_value(
            global_vars,
            "CAMERA_POLL_INTERVAL_SECONDS",
            "CAMERA_SHM_POLL_INTERVAL_SECONDS",
            "camera_poll_interval",
        )

    if model is None or version is None:
        raise CameraReadError(
            EXIT_SDK,
            "robot model/version are not available in GLOBAL_VARS; provide "
            "--robot-model and --robot-version",
        )
    timeout = 2.0 if timeout is None else timeout
    poll_interval = 0.02 if poll_interval is None else poll_interval
    try:
        timeout = float(timeout)
        poll_interval = float(poll_interval)
    except (TypeError, ValueError) as exc:
        raise CameraReadError(EXIT_SDK, "timeout and poll interval must be numeric") from exc
    if not math.isfinite(timeout) or timeout <= 0:
        raise CameraReadError(EXIT_SDK, "--timeout must be finite and greater than zero")
    if not math.isfinite(poll_interval) or poll_interval < 0:
        raise CameraReadError(EXIT_SDK, "--poll-interval must be finite and non-negative")

    side = getattr(args, "side", "both")
    if side not in SIDE_MODULES and side != "both":
        # argparse 通常已强制此项；保留这道检查让嵌入脚本或测试直接
        # 调用本辅助函数时也安全。
        raise CameraReadError(EXIT_SDK, "--side must be left, right, or both")
    output_name = _text(getattr(args, "output_name", "decoded"))
    if output_name is None:
        raise CameraReadError(EXIT_OUTPUT, "--output-name must not be empty")
    return {
        "robot_model": model,
        "robot_version": version,
        "timeout": timeout,
        "poll_interval": poll_interval,
        "side": side,
        "output_name": output_name,
    }


def _selected_sides(side: str) -> tuple[str, ...]:
    return ("left", "right") if side == "both" else (side,)


def _catalog_outputs(
    list_camera_shm_outputs: Any, settings: dict[str, Any], module: str
) -> list[Any]:
    try:
        outputs = list(
            list_camera_shm_outputs(
                settings["robot_model"],
                settings["robot_version"],
                module_name=module,
            )
        )
    except Exception as exc:
        raise CameraReadError(
            EXIT_CATALOG,
            f"camera catalog lookup failed for module {module!r}: {exc}",
        ) from exc
    if not outputs:
        raise CameraReadError(
            EXIT_CATALOG,
            f"camera catalog contains no outputs for module {module!r}",
        )
    return outputs


def _select_output(outputs: list[Any], requested_name: str, side: str) -> Any:
    matches = [
        output
        for output in outputs
        if (_text(_field(output, "output_name", "name")) or "").casefold()
        == requested_name.casefold()
    ]
    if len(matches) == 1:
        selected = matches[0]
        output_name = _text(_field(selected, "output_name", "name"))
        catalog_format = _text(_field(selected, "pixel_format", "format"))
        if output_name is None:
            raise CameraReadError(
                EXIT_METADATA,
                f"{side} catalog output has no output_name metadata",
            )
        if catalog_format is None:
            raise CameraReadError(
                EXIT_METADATA,
                f"{side} catalog output {requested_name!r} has no pixel format metadata",
            )
        return selected
    available = [
        _text(_field(output, "output_name", "name")) or "<unnamed>" for output in outputs
    ]
    if not matches:
        reason = "is not present in the SDK catalog"
    else:
        reason = f"is ambiguous ({len(matches)} catalog entries)"
    raise CameraReadError(
        EXIT_OUTPUT,
        f"{side} output {requested_name!r} {reason}; available output_name values: "
        + ", ".join(available),
    )


def _open_consumer(open_camera_shm_consumer: Any, output: Any, side: str) -> Any:
    """打开目录描述符，不自行构造 SHM 名称/路径。"""

    output_name = _text(_field(output, "output_name", "name")) or "<unnamed>"
    try:
        # SHM 身份由目录描述符携带。这里的 ``name`` 只是本地消费者
        # 标签，且由目录数据派生；本示例没有硬编码任何 SHM 路径或
        # 缓冲区名。
        consumer = open_camera_shm_consumer(
            output,
            name=f"xr_hand_camera_{side}_{output_name}",
        )
    except Exception as exc:
        raise CameraReadError(
            EXIT_SHM,
            f"{side} output {output_name!r} is catalogued but its shared memory "
            f"could not be opened: {exc}",
        ) from exc
    return consumer


def _poll_frame(consumer: Any, side: str, timeout: float, poll_interval: float) -> tuple[Any, Any, Any]:
    if consumer is None or not callable(getattr(consumer, "get_latest", None)):
        raise CameraReadError(
            EXIT_SHM,
            f"{side} consumer did not return a usable SHM reader with get_latest()",
        )
    deadline = time.monotonic() + timeout
    while True:
        try:
            # 有意使用非阻塞调用：即使生产者已停止或共享内存区不存在，
            # 超时也保持有界。
            sample = consumer.get_latest(nonblock=True, with_meta=True)
        except Exception as exc:
            raise CameraReadError(
                EXIT_FRAME,
                f"{side} frame read failed while polling the SHM consumer: {exc}",
            ) from exc
        if sample is not None:
            if not isinstance(sample, Sequence) or isinstance(sample, (str, bytes, bytearray)):
                raise CameraReadError(
                    EXIT_FRAME,
                    f"{side} consumer returned an invalid frame result (expected image, "
                    "frame_id, metadata)",
                )
            if len(sample) != 3:
                raise CameraReadError(
                    EXIT_FRAME,
                    f"{side} consumer returned {len(sample)} values instead of image, "
                    "frame_id, metadata",
                )
            image, frame_id, metadata = sample
            if image is None:
                raise CameraReadError(EXIT_FRAME, f"{side} consumer returned an invalid empty image")
            if frame_id is None:
                raise CameraReadError(EXIT_METADATA, f"{side} frame is missing frame_id metadata")
            if metadata is None:
                raise CameraReadError(EXIT_METADATA, f"{side} frame is missing runtime metadata")
            return image, frame_id, metadata
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise CameraReadError(
                EXIT_FRAME,
                f"timed out after {timeout:.3f}s waiting for a {side} hand-camera frame",
            )
        if poll_interval:
            time.sleep(min(poll_interval, remaining))


def _catalog_size(output: Any) -> tuple[Any, Any]:
    width = _field(output, "width")
    height = _field(output, "height")
    if width is None or height is None:
        size = _field(output, "size", "shape")
        if isinstance(size, Mapping):
            width = width if width is not None else _field(size, "width", "w")
            height = height if height is not None else _field(size, "height", "h")
        elif isinstance(size, Sequence) and not isinstance(size, (str, bytes, bytearray)):
            if len(size) >= 2:
                width = width if width is not None else size[0]
                height = height if height is not None else size[1]
    return width, height


def _format_token(value: Any) -> str | None:
    """归一化 SDK 像素格式值，便于严格比较。"""

    text = _text(value)
    if text is None:
        return None
    return "".join(character for character in text.upper() if character.isalnum())


def _coerce_positive_dimension(value: Any, label: str, *, error_code: int) -> int:
    """只接受正的、有限的整数型尺寸，不做截断。"""

    if isinstance(value, (bool, str, bytes, bytearray)) or value is None:
        raise CameraReadError(
            error_code,
            f"{label} must be a positive finite integer-like value, received {value!r}",
        )
    if isinstance(value, numbers.Integral):
        integer = int(value)
    elif isinstance(value, numbers.Real):
        try:
            numeric = float(value)
        except (TypeError, ValueError, OverflowError) as exc:
            raise CameraReadError(
                error_code,
                f"{label} must be a positive finite integer-like value, received {value!r}",
            ) from exc
        if not math.isfinite(numeric) or not numeric.is_integer():
            raise CameraReadError(
                error_code,
                f"{label} must be a positive finite integer-like value, received {value!r}",
            )
        integer = int(numeric)
    else:
        raise CameraReadError(
            error_code,
            f"{label} must be a positive finite integer-like value, received {value!r}",
        )
    if integer <= 0:
        raise CameraReadError(
            error_code,
            f"{label} must be a positive finite integer-like value, received {value!r}",
        )
    return integer


def _shape_dimensions(image: Any, side: str = "left") -> tuple[int, int, int]:
    """返回验证过的三阶 ``(高, 宽, 通道数)`` 图像形状。"""

    shape = getattr(image, "shape", None)
    if not isinstance(shape, Sequence) or isinstance(shape, (str, bytes, bytearray)):
        raise CameraReadError(EXIT_FRAME, f"{side} frame is missing a rank-3 image shape")
    if len(shape) != 3:
        raise CameraReadError(
            EXIT_FRAME,
            f"{side} frame shape must have rank 3, received {shape!r}",
        )
    dimensions = tuple(
        _coerce_positive_dimension(value, f"{side} frame shape[{index}]", error_code=EXIT_FRAME)
        for index, value in enumerate(shape)
    )
    if dimensions[2] != 3:
        raise CameraReadError(
            EXIT_FRAME,
            f"{side} frame must have 3 BGR channels, received shape {shape!r}",
        )
    return dimensions


def _validate_left_decoded_frame(image: Any, metadata: Any, output: Any) -> tuple[int, int, int]:
    """对照目录/运行时元数据验证一帧左手解码图像。"""

    catalog_format = _format_token(_field(output, "pixel_format", "format"))
    if catalog_format != "BGR":
        raise CameraReadError(
            EXIT_METADATA,
            "left decoded catalog pixel format must be BGR, "
            f"received {_text(_field(output, 'pixel_format', 'format'))!r}",
        )
    runtime_format = _format_token(
        _field(
            metadata,
            "pixel_format_str",
            "runtime_pixel_format",
            "pixel_format",
            "format",
            "encoding",
            "image_format",
        )
    )
    if runtime_format is not None and runtime_format != "BGR":
        raise CameraReadError(
            EXIT_METADATA,
            "left decoded runtime pixel format must be BGR, "
            f"received {_text(_field(metadata, 'pixel_format_str', 'runtime_pixel_format', 'pixel_format', 'format', 'encoding', 'image_format'))!r}",
        )

    dtype_value = getattr(image, "dtype", None)
    dtype = _text(getattr(dtype_value, "name", dtype_value))
    if dtype is None or dtype.casefold() != "uint8":
        raise CameraReadError(
            EXIT_METADATA,
            f"left decoded frame dtype must be exactly uint8, received {dtype!r}",
        )
    height, width, channels = _shape_dimensions(image)
    catalog_width, catalog_height = _catalog_size(output)
    if catalog_width is None or catalog_height is None:
        raise CameraReadError(
            EXIT_METADATA,
            "left decoded catalog output is missing width/height size metadata",
        )
    expected_width = _coerce_positive_dimension(
        catalog_width,
        "left decoded catalog width",
        error_code=EXIT_METADATA,
    )
    expected_height = _coerce_positive_dimension(
        catalog_height,
        "left decoded catalog height",
        error_code=EXIT_METADATA,
    )
    if width != expected_width or height != expected_height:
        raise CameraReadError(
            EXIT_METADATA,
            "left decoded frame shape does not agree with catalog size: "
            f"frame={(height, width, channels)!r}, catalog={(expected_height, expected_width)!r}",
        )
    return height, width, channels


def _coerce_frame_id(value: Any) -> int:
    """只接受有限的整数型 frame ID，绝不接受任意值。"""

    if isinstance(value, (bool, str, bytes, bytearray)) or value is None:
        raise CameraReadError(
            EXIT_METADATA,
            f"left decoded frame_id must be a finite integer-like value, received {value!r}",
        )
    if isinstance(value, numbers.Integral):
        integer = int(value)
        if integer >= 0:
            return integer
        raise CameraReadError(
            EXIT_METADATA,
            f"left decoded frame_id must be a non-negative finite integer-like value, received {value!r}",
        )
    if isinstance(value, numbers.Real):
        try:
            numeric = float(value)
        except (TypeError, ValueError, OverflowError) as exc:
            raise CameraReadError(
                EXIT_METADATA,
                f"left decoded frame_id must be a finite integer-like value, received {value!r}",
            ) from exc
        if math.isfinite(numeric) and numeric.is_integer() and numeric >= 0:
            return int(numeric)
    raise CameraReadError(
        EXIT_METADATA,
        f"left decoded frame_id must be a finite integer-like value, received {value!r}",
    )


JPEG_FORMATS = frozenset({"JPEG", "JPG", "MJPG", "MJPEG"})


def _load_jpeg_decoder() -> Any:
    """返回 ``bytes/ndarray -> BGR ndarray | None`` 的本地 JPEG 解码器。"""

    try:
        import cv2
        import numpy
    except ImportError as exc:
        raise CameraReadError(
            EXIT_SDK,
            f"cv2/numpy are required to decode left JPEG frames locally: {exc}",
        ) from exc

    def decode(data: Any) -> Any:
        buffer = numpy.frombuffer(memoryview(data), dtype=numpy.uint8)
        return cv2.imdecode(buffer, cv2.IMREAD_COLOR)

    return decode


def _decode_left_jpeg_frame(decoder: Any, image: Any, metadata: Any, output: Any) -> Any:
    """把一帧左手 JPEG 字节本地解成 BGR，并对照目录尺寸校验。"""

    if _format_token(_field(output, "pixel_format", "format")) not in JPEG_FORMATS:
        raise CameraReadError(
            EXIT_METADATA,
            "left JPEG catalog pixel format must be JPEG/MJPG, "
            f"received {_text(_field(output, 'pixel_format', 'format'))!r}",
        )
    runtime_format = _format_token(
        _field(
            metadata,
            "pixel_format_str",
            "runtime_pixel_format",
            "pixel_format",
            "format",
            "encoding",
            "image_format",
        )
    )
    if runtime_format is not None and runtime_format not in JPEG_FORMATS:
        raise CameraReadError(
            EXIT_METADATA,
            f"left JPEG runtime pixel format must be JPEG/MJPG, received {runtime_format!r}",
        )
    if not callable(decoder):
        raise CameraReadError(EXIT_SDK, "left JPEG decoder is not callable")
    try:
        decoded = decoder(image)
    except Exception as exc:
        raise CameraReadError(EXIT_FRAME, f"left JPEG frame decode failed: {exc}") from exc
    if decoded is None:
        raise CameraReadError(EXIT_FRAME, "left JPEG frame decode returned no image")
    dtype_value = getattr(decoded, "dtype", None)
    dtype = _text(getattr(dtype_value, "name", dtype_value))
    if dtype is None or dtype.casefold() != "uint8":
        raise CameraReadError(
            EXIT_METADATA,
            f"left decoded-from-JPEG frame dtype must be exactly uint8, received {dtype!r}",
        )
    height, width, channels = _shape_dimensions(decoded)
    catalog_width, catalog_height = _catalog_size(output)
    if catalog_width is None or catalog_height is None:
        raise CameraReadError(
            EXIT_METADATA,
            "left JPEG catalog output is missing width/height size metadata",
        )
    expected_width = _coerce_positive_dimension(
        catalog_width, "left JPEG catalog width", error_code=EXIT_METADATA
    )
    expected_height = _coerce_positive_dimension(
        catalog_height, "left JPEG catalog height", error_code=EXIT_METADATA
    )
    if width != expected_width or height != expected_height:
        raise CameraReadError(
            EXIT_METADATA,
            "left decoded-from-JPEG frame shape does not agree with catalog size: "
            f"frame={(height, width, channels)!r}, catalog={(expected_height, expected_width)!r}",
        )
    return decoded


class LeftHandDecodedCamera:
    """可复用的、目录驱动的左手新鲜 BGR 帧 reader。

    本类有意不提供任意 output-name 或 SHM-name 参数：打开时总是发现
    ``mod_camera_hand_left``，并按 ``source`` 选择其唯一的 ``decoded``
    目录输出（默认）或唯一的 ``JPEG`` 目录输出。``source="jpeg"`` 时
    读到的 MJPEG 字节由本地解码器（默认 ``cv2.imdecode``）解成 BGR，
    用于厂商 decoded 管线不出帧的机器（见 .codex/issues.md 2026-08-22）；
    对外返回值形态不变。SDK 可调用对象与解码器可注入，离线测试和
    消费方无需 import 机器人依赖即可使用本 reader。
    """

    MODULE = "mod_camera_hand_left"
    OUTPUT_NAME = "decoded"
    JPEG_OUTPUT_NAME = "JPEG"
    SOURCES = ("decoded", "jpeg")

    def __init__(
        self,
        robot_model: Any = None,
        robot_version: Any = None,
        *,
        timeout: float = 2.0,
        poll_interval: float = 0.02,
        list_camera_shm_outputs: Any = None,
        open_camera_shm_consumer: Any = None,
        clock: Any = time.monotonic,
        sleeper: Any = time.sleep,
        source: str = "decoded",
        decoder: Any = None,
    ) -> None:
        self.robot_model = robot_model
        self.robot_version = robot_version
        if source not in self.SOURCES:
            raise CameraReadError(
                EXIT_OUTPUT,
                f"camera source must be one of {self.SOURCES}, received {source!r}",
            )
        self.source = source
        self._decoder = decoder
        self.timeout = self._finite_timeout(timeout)
        self.poll_interval = self._finite_poll_interval(poll_interval)
        self._list_camera_shm_outputs = list_camera_shm_outputs
        self._open_camera_shm_consumer = open_camera_shm_consumer
        self._clock = clock
        self._sleeper = sleeper
        if not callable(self._clock) or not callable(self._sleeper):
            raise TypeError("clock and sleeper must be callable")
        self._output: Any = None
        self._consumer: Any = None
        self._opened = False
        self._closed = False
        self._last_frame_id: Any = None
        self._has_last_frame_id = False

    @staticmethod
    def _finite_timeout(value: Any) -> float:
        try:
            result = float(value)
        except (TypeError, ValueError) as exc:
            raise CameraReadError(EXIT_SDK, "camera timeout must be numeric") from exc
        if not math.isfinite(result) or result <= 0:
            raise CameraReadError(EXIT_SDK, "camera timeout must be finite and greater than zero")
        return result

    @staticmethod
    def _finite_poll_interval(value: Any) -> float:
        try:
            result = float(value)
        except (TypeError, ValueError) as exc:
            raise CameraReadError(EXIT_SDK, "camera poll interval must be numeric") from exc
        if not math.isfinite(result) or result < 0:
            raise CameraReadError(
                EXIT_SDK,
                "camera poll interval must be finite and non-negative",
            )
        return result

    def __enter__(self) -> "LeftHandDecodedCamera":
        return self.open()

    def __exit__(self, exc_type: Any, exc_value: Any, traceback: Any) -> bool:
        self.close()
        return False

    @property
    def output(self) -> Any:
        """选中的目录描述符，:meth:`open` 之后可用。"""

        return self._output

    @property
    def consumer(self) -> Any:
        """已打开的 SDK 消费者，:meth:`open` 之后可用。"""

        return self._consumer

    def open(self) -> "LeftHandDecodedCamera":
        """发现并打开唯一的左手 decoded 目录输出。"""

        if self._opened:
            return self
        if self._closed:
            raise CameraReadError(EXIT_SHM, "left decoded camera reader is already closed")

        consumer: Any = None
        try:
            list_outputs = self._list_camera_shm_outputs
            open_consumer = self._open_camera_shm_consumer
            if list_outputs is None or open_consumer is None:
                global_vars, sdk_list_outputs, sdk_open_consumer = _load_sdk()
                list_outputs = list_outputs or sdk_list_outputs
                open_consumer = open_consumer or sdk_open_consumer
                self.robot_model = self.robot_model or _global_value(
                    global_vars, "ACTIVE_ROBOT_MODEL", "ROBOT_MODEL", "robot_model"
                )
                self.robot_version = self.robot_version or _global_value(
                    global_vars, "ACTIVE_ROBOT_VERSION", "ROBOT_VERSION", "robot_version"
                )
            if self.robot_model is None or self.robot_version is None:
                raise CameraReadError(
                    EXIT_SDK,
                    "robot model/version are required for the left decoded camera reader",
                )
            settings = {
                "robot_model": self.robot_model,
                "robot_version": self.robot_version,
            }
            outputs = _catalog_outputs(list_outputs, settings, self.MODULE)
            if self.source == "jpeg":
                output = _select_output(outputs, self.JPEG_OUTPUT_NAME, "left")
                if _format_token(_field(output, "pixel_format", "format")) not in JPEG_FORMATS:
                    raise CameraReadError(
                        EXIT_METADATA,
                        "left JPEG catalog output must advertise a JPEG/MJPG pixel format",
                    )
                if self._decoder is None:
                    self._decoder = _load_jpeg_decoder()
            else:
                output = _select_output(outputs, self.OUTPUT_NAME, "left")
                if _format_token(_field(output, "pixel_format", "format")) != "BGR":
                    raise CameraReadError(
                        EXIT_METADATA,
                        "left decoded catalog output must advertise BGR pixel format",
                    )
            consumer = _open_consumer(open_consumer, output, "left")
            if consumer is None or not callable(getattr(consumer, "get_latest", None)):
                raise CameraReadError(
                    EXIT_SHM,
                    "left decoded catalog output did not return a usable SHM consumer",
                )
            self._output = output
            self._consumer = consumer
            self._opened = True
            return self
        except BaseException:
            # 若 SDK 已返回消费者但后续打开或元数据验证失败，
            # 先关闭它再向上传播原始异常。
            self._consumer = consumer
            self.close()
            raise

    def _strictly_greater(self, candidate: Any, baseline: Any) -> bool:
        try:
            return bool(candidate > baseline)
        except Exception as exc:
            raise CameraReadError(
                EXIT_METADATA,
                f"left decoded frame_id values are not comparable: {candidate!r}, {baseline!r}",
            ) from exc

    def _read_once(self, timeout: float) -> DecodedBGRFrame:
        if not self._opened or self._consumer is None or self._output is None:
            raise CameraReadError(EXIT_SHM, "left decoded camera reader is not open")
        deadline = self._clock() + timeout
        # 首次读取必须消耗一个观测样本作为基线。一旦返回过帧，后续
        # 读取直接用已返回的 ID 当基线，因此第一帧更新的样本会被
        # 直接返回，不会再被当作新基线丢弃。
        baseline: Any = self._last_frame_id if self._has_last_frame_id else None
        has_baseline = self._has_last_frame_id
        while True:
            try:
                sample = self._consumer.get_latest(nonblock=True, with_meta=True)
            except Exception as exc:
                raise CameraReadError(
                    EXIT_FRAME,
                    f"left decoded frame read failed while polling the SHM consumer: {exc}",
                ) from exc
            if sample is not None:
                received_at = self._clock()
                if not isinstance(sample, Sequence) or isinstance(
                    sample, (str, bytes, bytearray)
                ):
                    raise CameraReadError(
                        EXIT_FRAME,
                        "left decoded consumer returned an invalid frame result",
                    )
                if len(sample) != 3:
                    raise CameraReadError(
                        EXIT_FRAME,
                        "left decoded consumer must return image, frame_id, metadata",
                    )
                image, frame_id, metadata = sample
                if image is None:
                    raise CameraReadError(EXIT_FRAME, "left decoded consumer returned an empty image")
                if frame_id is None:
                    raise CameraReadError(EXIT_METADATA, "left decoded frame is missing frame_id metadata")
                if metadata is None:
                    raise CameraReadError(
                        EXIT_METADATA,
                        "left decoded frame is missing runtime metadata",
                    )
                frame_id = _coerce_frame_id(frame_id)
                if self.source == "jpeg":
                    image = _decode_left_jpeg_frame(
                        self._decoder, image, metadata, self._output
                    )
                else:
                    _validate_left_decoded_frame(image, metadata, self._output)
                if not has_baseline:
                    baseline = frame_id
                    has_baseline = True
                elif self._strictly_greater(frame_id, baseline):
                    self._last_frame_id = frame_id
                    self._has_last_frame_id = True
                    return DecodedBGRFrame(image, frame_id, float(received_at))
            remaining = deadline - self._clock()
            if remaining <= 0:
                raise CameraReadError(
                    EXIT_FRAME,
                    f"timed out after {timeout:.3f}s waiting for a fresh left hand-camera frame",
                )
            if self.poll_interval:
                self._sleeper(min(self.poll_interval, remaining))

    def read(self, timeout: Any = None) -> DecodedBGRFrame:
        """返回严格更新的解码 BGR 帧，否则 fail-closed。

        第一个非空样本只作观测基线，绝不返回。之后每次调用都以上次
        返回的 frame ID 为基线，因此重复或递减的 ID 不可能被报告为
        新鲜帧。读取失败会关闭消费者；读取成功则保持打开，供后续
        严格递增的调用或显式/上下文管理器关闭。
        """

        if not self._opened:
            self.open()
        try:
            read_timeout = self.timeout if timeout is None else self._finite_timeout(timeout)
            return self._read_once(read_timeout)
        except BaseException:
            self.close()
            raise

    def close(self) -> None:
        """只关闭消费者一次；可安全重复调用。"""

        if self._closed:
            return
        self._closed = True
        consumer = self._consumer
        self._consumer = None
        self._opened = False
        if consumer is not None:
            _close_consumer("left", consumer)


def _frame_record(
    side: str,
    module: str,
    output: Any,
    frame: tuple[Any, Any, Any],
) -> dict[str, Any]:
    image, frame_id, metadata = frame
    shape = getattr(image, "shape", None)
    dtype = _text(getattr(image, "dtype", None))
    catalog_format = _text(_field(output, "pixel_format", "format"))
    runtime_format = _text(
        _field(
            metadata,
            "pixel_format_str",
            "runtime_pixel_format",
            "pixel_format",
            "format",
            "encoding",
            "image_format",
        )
    )
    width, height = _catalog_size(output)
    if frame_id is None:
        raise CameraReadError(EXIT_METADATA, f"{side} frame is missing frame_id metadata")
    if shape is None or dtype is None:
        raise CameraReadError(
            EXIT_METADATA,
            f"{side} frame is missing shape or dtype metadata",
        )
    if not isinstance(shape, Sequence) or isinstance(shape, (str, bytes, bytearray)) or not shape:
        raise CameraReadError(EXIT_FRAME, f"{side} frame has an invalid shape {shape!r}")
    if catalog_format is None or runtime_format is None:
        missing = []
        if catalog_format is None:
            missing.append("catalog pixel format")
        if runtime_format is None:
            missing.append("runtime pixel format")
        raise CameraReadError(
            EXIT_METADATA,
            f"{side} frame is missing " + " and ".join(missing) + " metadata",
        )
    if width is None or height is None:
        raise CameraReadError(
            EXIT_METADATA,
            f"{side} catalog output is missing width/height size metadata",
        )
    effective_format = runtime_format or catalog_format
    return {
        "side": side,
        "module": module,
        "output_name": _json_value(_field(output, "output_name", "name")),
        "frame_id": _json_value(frame_id),
        "shape": _json_value(shape),
        "dtype": dtype,
        "pixel_format": effective_format,
        "effective_pixel_format": effective_format,
        "catalog_pixel_format": catalog_format,
        "runtime_pixel_format": runtime_format,
        # 保留头部相机示例的字段拼写，方便习惯那份输出的读者，
        # 同时也暴露显式的 runtime 命名。
        "metadata_pixel_format": runtime_format,
        "catalog_size": {
            "width": _json_value(width),
            "height": _json_value(height),
        },
    }


def _close_consumer(side: str, consumer: Any) -> None:
    if consumer is None:
        return
    close = getattr(consumer, "close", None)
    if not callable(close):
        print(f"WARNING: {side} consumer has no close()", file=sys.stderr)
        return
    try:
        close()
    except Exception as exc:  # pragma: no cover - SDK-specific cleanup failure
        print(f"WARNING: {side} consumer cleanup failed: {exc}", file=sys.stderr)


def _read_side(
    side: str,
    settings: dict[str, Any],
    list_camera_shm_outputs: Any,
    open_camera_shm_consumer: Any,
) -> dict[str, Any]:
    module = SIDE_MODULES[side]
    output_name = settings["output_name"]
    output: Any = None
    consumer: Any = None
    try:
        outputs = _catalog_outputs(list_camera_shm_outputs, settings, module)
        output = _select_output(outputs, output_name, side)
        consumer = _open_consumer(open_camera_shm_consumer, output, side)
        frame = _poll_frame(consumer, side, settings["timeout"], settings["poll_interval"])
        return {
            "status": "ok",
            **_frame_record(side, module, output, frame),
        }
    finally:
        # 读取成功与 open 之后的一切失败（包括畸形的消费者/帧结果）
        # 都会走到这里。
        _close_consumer(side, consumer)


def _run(args: argparse.Namespace) -> dict[str, Any]:
    global_vars, list_outputs, open_consumer = _load_sdk()
    settings = _resolve_settings(args, global_vars)
    sides = _selected_sides(settings["side"])
    records: dict[str, dict[str, Any]] = {}
    for side in sides:
        module = SIDE_MODULES[side]
        try:
            records[side] = _read_side(side, settings, list_outputs, open_consumer)
        except CameraReadError as exc:
            records[side] = {
                "status": "error",
                "side": side,
                "module": module,
                "output_name": settings["output_name"],
                "error_code": exc.code,
                "error": exc.message,
            }
        except Exception as exc:  # pragma: no cover - defensive SDK boundary
            records[side] = {
                "status": "error",
                "side": side,
                "module": module,
                "output_name": settings["output_name"],
                "error_code": EXIT_UNEXPECTED,
                "error": f"unexpected hand-camera failure: {exc}",
            }
    succeeded = all(record.get("status") == "ok" for record in records.values())
    return {
        "status": "ok" if succeeded else "error",
        "robot_model": _json_value(settings["robot_model"]),
        "robot_version": _json_value(settings["robot_version"]),
        "side": settings["side"],
        "sides": records,
    }


def _result_exit_code(result: Mapping[str, Any]) -> int:
    if result.get("status") == "ok":
        return EXIT_OK
    sides = result.get("sides")
    if isinstance(sides, Mapping):
        for record in sides.values():
            if isinstance(record, Mapping) and record.get("status") != "ok":
                try:
                    code = int(record.get("error_code", EXIT_UNEXPECTED))
                except (TypeError, ValueError):
                    code = EXIT_UNEXPECTED
                return code if code != EXIT_OK else EXIT_UNEXPECTED
    return EXIT_UNEXPECTED


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    try:
        result = _run(args)
    except CameraReadError as exc:
        print(f"ERROR[{exc.code}]: {exc.message}", file=sys.stderr)
        return exc.code
    except KeyboardInterrupt:
        print("ERROR[130]: interrupted while reading hand-camera frames", file=sys.stderr)
        return 130
    except Exception as exc:  # pragma: no cover - defensive command boundary
        print(f"ERROR[{EXIT_UNEXPECTED}]: unexpected hand-camera reader failure: {exc}", file=sys.stderr)
        return EXIT_UNEXPECTED

    print(json.dumps(result, ensure_ascii=False, sort_keys=True, default=_json_value))
    if result.get("status") != "ok":
        sides = result.get("sides")
        if isinstance(sides, Mapping):
            for side, record in sides.items():
                if isinstance(record, Mapping) and record.get("status") != "ok":
                    print(
                        f"ERROR[{record.get('error_code', EXIT_UNEXPECTED)}] {side}: "
                        f"{record.get('error', 'hand-camera read failed')}",
                        file=sys.stderr,
                    )
    return _result_exit_code(result)


if __name__ == "__main__":
    raise SystemExit(main())


class RightHandDecodedCamera(LeftHandDecodedCamera):
    """右腕相机读取器：与左手类逻辑完全相同，仅模块名不同（供 examples/arm/right/ 使用）。"""

    MODULE = "mod_camera_hand_right"
