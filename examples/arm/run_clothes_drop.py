#!/usr/bin/env python3
"""头部相机引导的衣物投放 runner（point5 衣物篮筐）。

状态机就是现场验证过的篮筐投放 runner，原封不动
（``run_basket_drop.run_drop``）：VERIFY_HOLDING（此处用标定的
"捏住布料"验证带；空爪或反馈无效则拒绝执行）-> 有界到位门控的
颈部俯仰 -> 运动前两次独立定位且必须一致 -> 单轴逐步有界接近、
每步带双臂反馈门 -> 释放 -> 回撤。途中反馈失败会保持闭爪并停止
发布后续目标。全程不咨询手部相机——提起的衣物会垂盖住它。

与瓶子投放的差异只有标定数值，全部由机上 ``clothes_drop.json``
承载（point5 身份、捏布验证带、point5 的 ROI/颜色/头部俯仰、为
垂落衣物保守调"高"的释放偏移）——见 ``clothes_drop_config.py``。
因此本模块只是把共享机制绑定到衣物默认值的薄入口；不 fork 任何
逻辑，已验收的投放行为就不可能在两个目标之间漂移。
"""

from __future__ import annotations

import argparse
import importlib.util
import json
import sys
from collections.abc import Sequence
from pathlib import Path
from typing import Any

try:  # 从项目根目录按包导入。
    from examples.arm.clothes_drop_config import (
        STEP_MAX_M,
        ConfigError,
        load_config,
        require_identity,
    )
except ImportError:  # 在 examples/arm 目录下直接执行。
    from clothes_drop_config import (  # type: ignore[no-redef]
        STEP_MAX_M,
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
    module_dir = Path(__file__).resolve().parent.parent / subdir
    for entry in (str(module_dir), str(module_dir.parent.parent)):
        if entry not in sys.path:
            sys.path.insert(0, entry)
    module_path = module_dir / f"{name}.py"
    spec = importlib.util.spec_from_file_location(f"xr_clothes_drop_{name}", module_path)
    if spec is None or spec.loader is None:  # pragma: no cover - repo layout invariant
        raise ImportError(f"could not load sibling module {name!r} from {module_path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    _MODULE_CACHE[key] = module
    return module


# 完整的现场验证投放机制，不 fork 直接复用。
_DROP = _load_sibling("arm", "run_basket_drop")

RunnerError = _DROP.RunnerError
FeedbackError = _DROP.FeedbackError
Pose = _DROP.Pose
EEFState = _DROP.EEFState
GripperState = _DROP.GripperState
run_drop = _DROP.run_drop
plan_axis_steps = _DROP.plan_axis_steps
drop_interface_names = _DROP.drop_interface_names
STAGES = _DROP.STAGES

EXPECTED_CAMERA_ID = "mod_camera_rgbd_head"


def _json_text(value: Any) -> str:
    return json.dumps(value, separators=(",", ":"), ensure_ascii=True, allow_nan=False)


def _preview_text(config_path: str) -> str:
    lines = [
        "Head-camera clothes-drop preview",
        "  mode: preview",
        "  planned_stages: " + " -> ".join(STAGES),
        (
            "  limits: "
            f"per-step <= {STEP_MAX_M} m (axis-at-a-time, rise first / descend last); targets stay "
            "inside the configured absolute envelope; one overall timeout; automatic neck pitch to "
            "the configured detection angle (settle-gated bounded steps) before detection; two "
            "pre-motion localizations must agree before any motion; a mid-path feedback failure "
            "keeps the gripper closed; no hand camera is consulted (the garment drapes over it)"
        ),
        "  precondition: left gripper feedback must sit inside the calibrated cloth-holding band",
        "  end_state: gripper open, garment released into the basket, arm at the retract position",
        "  abort_semantics: an abort stops publishing further targets only; e-stop is the field guard",
        f"  config: {config_path}",
        "  topics: " + _json_text(drop_interface_names()),
        "  gates: --execute absent",
        "  no action: no EEF publication and no gripper publication",
    ]
    return "\n".join(lines)


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Head-camera-guided clothes drop.")
    parser.add_argument("mode", nargs="?", default="preview", choices=("preview", "run"))
    parser.add_argument("--execute", action="store_true", help="confirm physical execution")
    parser.add_argument("--config", default="clothes_drop.json", help="calibration configuration path")
    parser.add_argument("--workcell-id", default="s2-test-point5-baskets")
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
        evidence_dir = Path.home() / "Documents" / "AutolifeXAIR" / "evidence" / "clothes_drop" / stamp

    runtime = None
    try:
        runtime = _DROP._RosRuntime(drop_interface_names(), evidence_dir)
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
