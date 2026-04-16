"""动作注册表路由测试。"""

from dataclasses import dataclass

from backend.motion_service import MotionService


@dataclass
class _ArmState:
    connected: bool = True
    initialized: bool = True


class _FakeController:
    def __init__(self):
        self.arms = {"left": object(), "right": object()}
        self.arm_states = {"left": _ArmState(), "right": _ArmState()}

    def set_joint_offsets(self, arm_id, joints):
        return True

    def go_to_zero(self, arm_id=None):
        return True


class _FakeCartesian:
    def compute_ik(self, arm_id, x, y, z, current_joints, roll=None, pitch=None, yaw=None):
        return {"joints": list(current_joints), "error_mm": 0.1}

    def compute_fk(self, arm_id, joints):
        return {"x": 0.1, "y": 0.2, "z": 0.3, "roll": 0.0, "pitch": 0.0, "yaw": 0.0}


def _build_service():
    service = MotionService(
        controller=_FakeController(),
        cartesian=_FakeCartesian(),
        emit_fn=lambda event, payload, to=None: None,
        get_joints_fn=lambda arm_ctrl, arm_id: [0.0] * 7,
    )
    service._refresh_action_registry = lambda: None
    service._a7_adapter.is_ready = lambda arm_id: True
    return service


def test_registry_action_builds_motion_command_with_override():
    service = _build_service()
    service._action_registry = {
        "reach_pregrasp": {
            "timeout_ms": 8000,
            "safe_abort_action": "retreat_to_safe_pose",
            "arm_plan": {
                "motion_type": "CARTESIAN_PTP",
                "frame": "BASE",
                "pose": {"x": 0.3, "y": 0.1, "z": 0.25},
                "constraints": {"max_velocity": 0.2},
            },
            "hand_plan": {"shape": "open_relaxed"},
        }
    }

    captured = {}

    def _fake_submit(sid, cmd, allow_preempt=False):
        captured["cmd"] = cmd
        return {"accepted": True, "command_id": cmd.command_id, "detail": {}}

    service._submit_command = _fake_submit

    payload = {
        "request_id": "req-100",
        "run_id": "run-100",
        "step_id": "step-100",
        "action_id": "reach_pregrasp",
        "arm_id": "left",
        "params": {
            "arm": {
                "pose": {"x": 0.31},  # 仅覆盖 x
            }
        },
    }
    result = service.execute_action("sid-1", payload)

    assert result["success"] is True
    cmd = captured["cmd"]
    assert cmd.target["pose"]["x"] == 0.31
    assert cmd.target["pose"]["y"] == 0.1
    assert cmd.options["timeout_ms"] == 8000
    assert cmd.options["safe_abort_action"] == "retreat_to_safe_pose"


def test_action_params_fallback_without_registry():
    service = _build_service()
    service._action_registry = {}
    service._submit_command = lambda sid, cmd, allow_preempt=False: {"accepted": True, "command_id": cmd.command_id, "detail": {}}

    payload = {
        "request_id": "req-200",
        "run_id": "run-200",
        "step_id": "step-200",
        "action_id": "custom_runtime_action",
        "arm_id": "left",
        "params": {
            "arm": {
                "motion_type": "CARTESIAN_PTP",
                "frame": "BASE",
                "pose": {"x": 0.2, "y": 0.2, "z": 0.2},
            }
        },
    }
    result = service.execute_action("sid-1", payload)
    assert result["success"] is True
