#!/usr/bin/env python3
"""只读的头部相机篮筐投放定位探针
（``openspec/changes/add-head-camera-basket-drop`` 的 task 2.2）。

对 N 个样本中的每一个，本探针精确运行投放 runner 的定位路径——
稳定的黄色筐沿检测（`detect_yellow_basket`）、内缩筐沿参考像素、
邻域中位数深度采样、``CameraTransformer`` 基座坐标链
（`head_bottle_geometry`）——使用 runner 将来会用的同一份已验证
投放配置，并打印基座坐标下的筐沿坐标供卷尺比对。

严格只读：从不构造 ROS publisher，从不发送运动或夹爪指令。其全部
ROS 活动只有订阅（``CameraTransformer`` 内部的关节状态订阅，加
一个只记录每条关节状态消息"到达时刻"的本地订阅）。runner 的
``locate_basket`` 实现以未绑定方式借用，因此探针结果就是 runner
的结果。
"""

from __future__ import annotations

import argparse
import importlib.util
import json
import sys
import time
from pathlib import Path
from typing import Any


EXIT_OK = 0
EXIT_UNEXPECTED = 1
EXIT_ARGS = 2
EXIT_CONFIG = 3
EXIT_ROS = 4
EXIT_PARTIAL = 5

_MODULE_CACHE: dict[str, Any] = {}


def _load_sibling(subdir: str, name: str) -> Any:
    key = f"{subdir}/{name}"
    if key in _MODULE_CACHE:
        return _MODULE_CACHE[key]
    module_dir = Path(__file__).resolve().parent.parent / subdir
    for entry in (str(module_dir), str(module_dir.parent.parent)):
        if entry not in sys.path:
            sys.path.insert(0, entry)
    module_path = module_dir / f"{name}.py"
    spec = importlib.util.spec_from_file_location(f"xr_basket_probe_{name}", module_path)
    if spec is None or spec.loader is None:  # pragma: no cover - repo layout invariant
        raise ImportError(f"could not load sibling module {name!r} from {module_path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    _MODULE_CACHE[key] = module
    return module


class _ProbeLocator:
    """在仅订阅节点上运行 runner 的定位路径。"""

    def __init__(self, rclpy_module: Any, node: Any, names: dict[str, str], runner: Any) -> None:
        self._rclpy = rclpy_module
        self._node = node
        self.names = dict(names)
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
        # 以未绑定方式借用 runner 的方法，让探针行为"就是" runner
        # 行为；两个方法都不发布任何东西。
        self._ensure_head_pipeline = runner._RosRuntime._ensure_head_pipeline.__get__(self)
        self.locate_basket = runner._RosRuntime.locate_basket.__get__(self)

    def note_joint_arrival(self) -> None:
        self._joint_arrival["last_arrival"] = time.monotonic()

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


def _save_annotated(frame: Any, record: Any, path: Path) -> bool:
    try:
        import cv2

        image = frame.image.copy()
        if record and record.get("detection"):
            x, y, w, h = (int(value) for value in record["detection"]["bbox"])
            cv2.rectangle(image, (x, y), (x + w, y + h), (0, 0, 255), 2)
        if record and record.get("reference_pixel"):
            u, v = (int(round(value)) for value in record["reference_pixel"])
            cv2.drawMarker(image, (u, v), (255, 0, 0), cv2.MARKER_CROSS, 20, 2)
        cv2.imwrite(str(path), image)
        return True
    except Exception:
        return False


def main(argv: Any = None) -> int:
    parser = argparse.ArgumentParser(
        description="Read-only basket-drop localization probe (no motion, no publishers)."
    )
    parser.add_argument("--config", required=True, metavar="FILE", help="validated basket_drop.json")
    parser.add_argument("--samples", type=int, default=5, metavar="N")
    parser.add_argument("--save-frames", default=None, metavar="DIR", help="save annotated color frames")
    parser.add_argument("--workcell-id", default="s2-test-point3-baskets")
    args = parser.parse_args(argv)
    if args.samples < 1:
        print(f"ERROR[{EXIT_ARGS}]: --samples must be >= 1", file=sys.stderr)
        return EXIT_ARGS

    runner = _load_sibling("arm", "run_basket_drop")
    config_module = _load_sibling("arm", "basket_drop_config")
    try:
        config = config_module.load_config(args.config)
        config_module.require_identity(
            config,
            camera_id=runner.EXPECTED_CAMERA_ID,
            workcell_id=args.workcell_id,
        )
    except config_module.ConfigError as exc:
        print(f"ERROR[{EXIT_CONFIG}]: {exc}", file=sys.stderr)
        return EXIT_CONFIG

    save_dir: Path | None = None
    if args.save_frames is not None:
        save_dir = Path(args.save_frames).expanduser()
        save_dir.mkdir(parents=True, exist_ok=True)

    # 2026-08-13 教训：AutoLife SDK 和 cv2 必须在 rclpy.init() 之前
    # import；探针辅助模块会为 SHM 管线缓存它们。
    probe_module = _load_sibling("camera", "head_bottle_probe")
    try:
        import cv2  # noqa: F401

        probe_module._load_sdk()
    except Exception as exc:
        print(f"ERROR[{EXIT_ROS}]: SDK/OpenCV unavailable before ROS init: {exc}", file=sys.stderr)
        return EXIT_ROS

    try:
        import rclpy
        from std_msgs.msg import String
    except ImportError as exc:
        print(f"ERROR[{EXIT_ROS}]: ROS runtime unavailable: {exc}", file=sys.stderr)
        return EXIT_ROS

    names = runner.interface_names()
    node = None
    locator = None
    try:
        rclpy.init(args=None)
        node = rclpy.create_node("xr_basket_drop_probe")
        locator = _ProbeLocator(rclpy, node, names, runner)
        # 只记录到达时刻；此处从不解析载荷。
        node.create_subscription(
            String, names["gripper_state_topic"], lambda _msg: locator.note_joint_arrival(), 10
        )

        samples: list[dict[str, Any]] = []
        located_points: list[dict[str, float]] = []
        for index in range(args.samples):
            outcome = locator.locate_basket(config)
            record = {
                "index": index,
                "located": bool(outcome.located),
                "reason": outcome.reason,
                "point_base": list(outcome.point_base) if outcome.point_base is not None else None,
                "detail": outcome.record,
            }
            if save_dir is not None and getattr(outcome, "frame", None) is not None:
                frame_path = save_dir / f"probe_{index:03d}.jpg"
                if _save_annotated(outcome.frame, outcome.record, frame_path):
                    record["frame_path"] = str(frame_path)
            samples.append(record)
            if outcome.located:
                x, y, z = outcome.point_base
                located_points.append({"x": x, "y": y, "z": z})

        summary = probe_module._summarize_positions(args.samples, located_points)
        result = {
            "status": "ok" if summary["n_located"] == args.samples else "partial",
            "config": config.to_record(),
            "samples": samples,
            "summary": summary,
        }
        print(json.dumps(result, ensure_ascii=False, sort_keys=True, default=str))
        return EXIT_OK if summary["n_located"] == args.samples else EXIT_PARTIAL
    except KeyboardInterrupt:
        print("ERROR[130]: interrupted while probing", file=sys.stderr)
        return 130
    except Exception as exc:  # pragma: no cover - defensive boundary for SDK differences
        print(f"ERROR[{EXIT_UNEXPECTED}]: unexpected probe failure: {exc}", file=sys.stderr)
        return EXIT_UNEXPECTED
    finally:
        if locator is not None:
            locator.close()
        if node is not None:
            node.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    raise SystemExit(main())
