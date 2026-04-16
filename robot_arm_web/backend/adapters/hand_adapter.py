from __future__ import annotations

import sys
import time
from pathlib import Path
from typing import Any, Dict, Optional


class HandAdapter:
    """
    灵巧手执行适配层（阶段1先提供 mock 行为）。
    后续接入 linkerhand-ros2-sdk 时替换 execute_hand_plan 实现。
    """

    def __init__(self, config: Optional[Dict[str, Any]] = None):
        cfg = config or {}
        self._enabled = bool(cfg.get("enabled", True))
        self._hand_type = str(cfg.get("hand_type", "left"))
        self._hand_joint = str(cfg.get("hand_joint", "L6"))
        self._can = str(cfg.get("can", "can2"))
        self._modbus = str(cfg.get("modbus", "None"))
        self._integration_mode = "python_linkerhand_api" if self._enabled else "disabled"
        self._api = None
        self._init_error: Optional[str] = None

    @staticmethod
    def _validate_list(values: Any, expected_len: int, min_v: int, max_v: int) -> bool:
        if not isinstance(values, list) or len(values) != expected_len:
            return False
        for value in values:
            try:
                iv = int(value)
            except Exception:
                return False
            if iv < min_v or iv > max_v:
                return False
        return True

    def _resolve_preset_pose(self, hand_plan: Dict[str, Any]) -> Optional[list]:
        pose = hand_plan.get("pose_0_255")
        if self._validate_list(pose, expected_len=6, min_v=0, max_v=255):
            return [int(v) for v in pose]
        preset_map = {
            "open_relaxed": [200, 255, 255, 255, 255, 180],
            "pinch_soft": [120, 90, 255, 255, 255, 180],
            "close_soft": [80, 80, 80, 80, 80, 160],
        }
        shape = hand_plan.get("shape")
        if isinstance(shape, str) and shape in preset_map:
            return preset_map[shape]
        return None

    def _ensure_api(self):
        if not self._enabled:
            self._init_error = "hand disabled by config"
            return None
        if self._api is not None:
            return self._api
        if self._init_error:
            return None
        try:
            sdk_root = Path(__file__).resolve().parents[3] / "src" / "linkerhand-ros2-sdk" / "linker_hand_ros2_sdk"
            sdk_root_str = str(sdk_root)
            if sdk_root.exists() and sdk_root_str not in sys.path:
                sys.path.insert(0, sdk_root_str)
            from linker_hand_ros2_sdk.LinkerHand.linker_hand_api import LinkerHandApi

            self._api = LinkerHandApi(
                hand_type=self._hand_type,
                hand_joint=self._hand_joint,
                modbus=self._modbus,
                can=self._can,
            )
            return self._api
        except SystemExit as e:
            self._init_error = f"LinkerHandApi exited: {e}"
            return None
        except Exception as e:
            self._init_error = str(e)
            return None

    def is_ready(self) -> bool:
        return self._ensure_api() is not None

    def execute_hand_plan(self, hand_plan: Dict[str, Any], action_id: str) -> Dict[str, Any]:
        if not isinstance(hand_plan, dict) or not hand_plan:
            return {"ok": True, "status": "skipped", "detail": {"reason": "empty hand plan"}}
        api = self._ensure_api()
        if api is None:
            return {"ok": False, "status": "not_ready", "detail": {"error": self._init_error or "api unavailable"}}
        pose = self._resolve_preset_pose(hand_plan)
        if pose is None:
            return {"ok": False, "status": "invalid_plan", "detail": {"reason": "pose_0_255 or known shape required"}}
        speed = hand_plan.get("speed_0_255")
        torque = hand_plan.get("torque_0_255")
        sleep_ms = int(hand_plan.get("sleep_ms", 50) or 50)
        if speed is not None and not self._validate_list(speed, expected_len=6, min_v=10, max_v=255):
            return {"ok": False, "status": "invalid_plan", "detail": {"reason": "speed_0_255 must be len=6 in [10,255]"}}
        if torque is not None and not self._validate_list(torque, expected_len=6, min_v=0, max_v=255):
            return {"ok": False, "status": "invalid_plan", "detail": {"reason": "torque_0_255 must be len=6 in [0,255]"}}
        try:
            if speed is not None:
                api.set_joint_speed([int(v) for v in speed])
            if torque is not None:
                api.set_torque([int(v) for v in torque])
            api.finger_move([int(v) for v in pose])
            if sleep_ms > 0:
                time.sleep(sleep_ms / 1000.0)
        except Exception as e:
            return {"ok": False, "status": "sdk_failed", "detail": {"exception": str(e)}}
        return {
            "ok": True,
            "status": "executed",
            "detail": {
                "action_id": action_id,
                "mode": self._integration_mode,
                "plan": hand_plan,
                "can": self._can,
                "hand_joint": self._hand_joint,
            },
        }

    def telemetry(self) -> Dict[str, Any]:
        return {
            "ready": self.is_ready(),
            "integration_mode": self._integration_mode,
            "hand_type": self._hand_type,
            "hand_joint": self._hand_joint,
            "can": self._can,
            "error": self._init_error,
        }
