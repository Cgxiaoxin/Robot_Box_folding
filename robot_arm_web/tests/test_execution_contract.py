"""执行层接口契约测试。"""

from dataclasses import dataclass
import time

from backend.motion_service import MotionService, _Runtime
from backend.motion_types import ErrorCode


@dataclass
class _ArmState:
    connected: bool = True
    initialized: bool = True


class _FakeController:
    def __init__(self):
        self.arms = {"left": object(), "right": object()}
        self.arm_states = {"left": _ArmState(), "right": _ArmState()}
        self.go_zero_calls = []

    def set_joint_offsets(self, arm_id, joints):
        return True

    def go_to_zero(self, arm_id=None):
        self.go_zero_calls.append(arm_id)


class _FakeCartesian:
    def compute_ik(self, arm_id, x, y, z, current_joints, roll=None, pitch=None, yaw=None):
        return {"joints": list(current_joints), "error_mm": 0.1}

    def compute_fk(self, arm_id, joints):
        return {"x": 0.1, "y": 0.2, "z": 0.3, "roll": 0.0, "pitch": 0.0, "yaw": 0.0}


def _build_service(emits):
    controller = _FakeController()
    cartesian = _FakeCartesian()
    service = MotionService(
        controller=controller,
        cartesian=cartesian,
        emit_fn=lambda event, payload, to=None: emits.append((event, payload, to)),
        get_joints_fn=lambda arm_ctrl, arm_id: [0.0] * 7,
    )
    return service, controller


def test_execute_action_rejects_missing_required_fields():
    emits = []
    service, _ = _build_service(emits)
    result = service.execute_action("sid-1", {"action_id": "reach_pregrasp"})
    assert result["success"] is False
    assert result["error_code"] == ErrorCode.INVALID_REQUEST.value
    assert "missing_fields" in result["detail"]


def test_execute_action_inject_sdk_error_from_hand_adapter():
    emits = []
    service, _ = _build_service(emits)
    service._action_registry = {
        "test_action": {
            "sync_policy": "hand_then_arm",
            "arm_plan": {"motion_type": "CARTESIAN_PTP", "frame": "BASE", "pose": {"x": 0.1, "y": 0.2, "z": 0.3}},
            "hand_plan": {"shape": "pinch_soft"},
        }
    }
    service._refresh_action_registry = lambda: None
    service._hand_adapter.is_ready = lambda: True
    service._hand_adapter.execute_hand_plan = lambda hand_plan, action_id: {"ok": False, "status": "sdk_failed"}
    payload = {
        "request_id": "req-1",
        "run_id": "run-1",
        "step_id": "step-1",
        "action_id": "test_action",
        "arm_id": "left",
        "sync_policy": "hand_then_arm",
    }
    result = service.execute_action("sid-1", payload)
    assert result["success"] is False
    assert result["error_code"] == ErrorCode.SDK_ERROR.value


def test_run_command_timeout_triggers_safe_abort():
    emits = []
    service, controller = _build_service(emits)
    cmd = service._make_command(
        {
            "command_id": "cmd-timeout",
            "arm_id": "left",
            "motion_type": "CARTESIAN_PTP",
            "frame": "BASE",
            "target": {"pose": {"x": 0.1, "y": 0.2, "z": 0.3}},
            "constraints": {},
            "options": {"timeout_ms": 1, "safe_abort_action": "retreat_to_safe_pose"},
        }
    )
    rt = _Runtime(
        command=cmd,
        sid="sid-1",
        created_at=time.time() - 1.0,
        timeout_ms=1,
        safe_abort_action="retreat_to_safe_pose",
    )
    service._run_command(rt)

    assert controller.go_zero_calls
    failed_events = [item for item in emits if item[0] == "motion:failed"]
    assert failed_events
    assert failed_events[-1][1]["error_code"] == ErrorCode.ACTION_TIMEOUT.value


def test_arm_then_hand_executes_after_arm_success():
    emits = []
    service, _ = _build_service(emits)
    called = []
    service._hand_adapter.is_ready = lambda: True
    service._hand_adapter.execute_hand_plan = lambda hand_plan, action_id: called.append((hand_plan, action_id)) or {"ok": True, "status": "executed"}

    cmd = service._make_command(
        {
            "command_id": "cmd-arm-then-hand",
            "arm_id": "left",
            "motion_type": "CARTESIAN_PTP",
            "frame": "BASE",
            "target": {"pose": {"x": 0.1, "y": 0.2, "z": 0.3}},
            "constraints": {},
            "options": {
                "sync_policy": "arm_then_hand",
                "post_arm_hand_plan": {"pose_0_255": [200, 255, 255, 255, 255, 180]},
                "execution_action_id": "reach_pregrasp",
            },
        }
    )
    rt = _Runtime(command=cmd, sid="sid-1", created_at=time.time())
    service._run_command(rt)
    assert called
    completed_events = [item for item in emits if item[0] == "motion:completed"]
    assert completed_events
