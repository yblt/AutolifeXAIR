#!/usr/bin/env python3
"""point5 衣物投放的 fail-closed 标定配置。

衣物投放原封不动地复用篮筐投放的配置 schema——字段、硬上限、
验证逻辑完全相同——因为两个任务只有标定"数值"不同，结构没有差异
（`add-clothes-grasp-drop` 设计决策 7）。因此本模块直接转导出
``basket_drop_config.py`` 里现场验证过的机制，而不是 fork 出第二份
副本；一套经过测试的 fail-closed 实现同时服务两个目标。

共享字段在衣物场景下的专属语义（写在机上 ``clothes_drop.json``
里，绝不写死在代码中）：

- ``held_feedback_center`` / ``held_feedback_tolerance``：标定的
  "捏住布料"验证带（来自抓取标定的捏衣夹爪反馈），不是瓶子的
  验证带——空夹爪必须拒绝投放；
- ``hover_offset_m[2]``：相对感知筐沿点的释放高度。衣物要保守地
  标"高"：提起的衣物会垂到夹爪下方很远，必须能越过篮筐前沿；
- ``workcell_id``：point5 工位标识，保证 point3 的篮筐标定绝无可能
  在这里被复用（``target_id`` 只作记录，不参与比对）；
- ``detection_roi_px`` / ``hsv_lower`` / ``hsv_upper``：point5
  停靠视角下的 point5 篮筐。

point5 的感知到基座链路是全新的（新头部姿态、新目标）：其标定
偏移必须在首次实机臂运动之前来自一次物理真值比对，与抓取侧
要求完全一致。
"""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path
from typing import Any


_BASE_CACHE: dict[str, Any] = {}


def _base() -> Any:
    """加载同目录的篮筐投放配置模块，带缓存。"""

    if "base" in _BASE_CACHE:
        return _BASE_CACHE["base"]
    module_path = Path(__file__).resolve().parent / "basket_drop_config.py"
    spec = importlib.util.spec_from_file_location("xr_clothes_drop_config_base", module_path)
    if spec is None or spec.loader is None:  # pragma: no cover - repo layout invariant
        raise ImportError(f"could not load base configuration from {module_path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    _BASE_CACHE["base"] = module
    return module


_MODULE = _base()

SCHEMA_VERSION = _MODULE.SCHEMA_VERSION
STEP_MAX_M = _MODULE.STEP_MAX_M
ConfigError = _MODULE.ConfigError
ClothesDropConfig = _MODULE.BasketDropConfig
parse_config = _MODULE.parse_config
load_config = _MODULE.load_config
require_identity = _MODULE.require_identity

__all__ = [
    "SCHEMA_VERSION",
    "STEP_MAX_M",
    "ConfigError",
    "ClothesDropConfig",
    "parse_config",
    "load_config",
    "require_identity",
]
