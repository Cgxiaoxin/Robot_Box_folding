from __future__ import annotations

from typing import Any, Dict

from ..motion_types import Frame, MotionType


class A7Adapter:
    """A7 执行适配层，复用 GroupController 能力。"""

    def __init__(self, controller):
        self.controller = controller

    def is_ready(self, arm_id: str) -> bool:
        arm = self.controller.arms.get(arm_id)
        arm_state = self.controller.arm_states.get(arm_id)
        return bool(arm is not None and arm_state and arm_state.connected and arm_state.initialized)

    def build_motion_payload(
        self,
        request_id: str,
        action_id: str,
        arm_id: str,
        arm_plan: Dict[str, Any],
    ) -> Dict[str, Any]:
        return {
            "command_id": request_id,
            "arm_id": arm_id,
            "motion_type": arm_plan.get("motion_type", MotionType.CARTESIAN_PTP.value),
            "frame": arm_plan.get("frame", Frame.BASE.value),
            "target": {
                "pose": arm_plan.get("pose") or {},
            },
            "constraints": arm_plan.get("constraints") or {},
            "options": {
                "blocking": False,
                "action_id": action_id,
            },
        }

    def telemetry(self, arm_id: str) -> Dict[str, Any]:
        arm_state = self.controller.arm_states.get(arm_id)
        return {
            "arm_id": arm_id,
            "connected": bool(arm_state and arm_state.connected),
            "initialized": bool(arm_state and arm_state.initialized),
        }
