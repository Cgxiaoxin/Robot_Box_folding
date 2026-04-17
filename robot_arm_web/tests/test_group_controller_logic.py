"""群控核心逻辑测试：通过 FakeArm 隔离硬件依赖。"""

from __future__ import annotations

import importlib
import sys
import types
from dataclasses import dataclass


def _install_linkerbot_stubs():
    linkerbot_mod = types.ModuleType("linkerbot")

    class _ControlMode:
        PP = 1

    class _A7:
        def __init__(self, *args, **kwargs):
            del args, kwargs

    linkerbot_mod.A7 = _A7
    linkerbot_mod.ControlMode = _ControlMode

    motor_mod = types.ModuleType("linkerbot.arm.a7.motor")

    class _A7Motor:
        _handshake_fallback_installed = False

        def check_alive(self, timeout_s: float = 0.02) -> bool:
            del timeout_s
            return True

    motor_mod.A7Motor = _A7Motor

    sys.modules["linkerbot"] = linkerbot_mod
    sys.modules["linkerbot.arm"] = types.ModuleType("linkerbot.arm")
    sys.modules["linkerbot.arm.a7"] = types.ModuleType("linkerbot.arm.a7")
    sys.modules["linkerbot.arm.a7.motor"] = motor_mod


def _import_group_controller():
    _install_linkerbot_stubs()
    sys.modules.pop("backend.group_controller", None)
    return importlib.import_module("backend.group_controller")


@dataclass
class FakeMotor:
    # 只保留 GroupController 会访问的最小字段集合
    position: float = 0.0
    velocity: float = 0.0
    enabled: bool = True
    mode: int = 5


@dataclass
class _JointState:
    angle: float
    velocity: float = 0.0


class _FakeState:
    def __init__(self, angles):
        self.joint_angles = [_JointState(v) for v in angles]
        self.joint_velocities = [_JointState(0.0) for _ in angles]


class FakeArm:
    def __init__(self, motor_ids):
        # 使用假的 arm 对象隔离真实 CAN/硬件依赖
        self.motor_ids = motor_ids
        self.motors = {mid: FakeMotor(position=float(mid) / 100.0) for mid in motor_ids}
        self.positions_set = []
        self.offset_positions_set = []
        self.zero_offsets = {mid: 0.0 for mid in motor_ids}
        self.profile = []
        self.enabled = False
        self.estopped = False
        self.control_mode = None

    def get_state(self):
        return _FakeState(self.get_angles())

    def get_angles(self):
        return [self.motors[mid].position for mid in self.motor_ids]

    def set_control_mode(self, mode):
        self.control_mode = mode

    def enable(self):
        self.enabled = True

    def disable(self):
        self.enabled = False

    def emergency_stop(self):
        self.estopped = True

    def move_j(self, target_angles, blocking=False):
        self.positions_set.append((tuple(target_angles), blocking))
        for mid, pos in zip(self.motor_ids, target_angles):
            self.motors[mid].position = pos

    def set_position(self, motor_id, position):
        self.positions_set.append((motor_id, position))
        self.motors[motor_id].position = position

    def set_position_with_offset(self, motor_id, position):
        self.offset_positions_set.append((motor_id, position))

    def set_velocities(self, values):
        self.profile.append(("velocity", list(values)))

    def set_accelerations(self, values):
        self.profile.append(("acceleration", list(values)))


def make_controller_with_fake_arms():
    # 构造可测试的 controller，避免触发真实硬件通信
    gc = _import_group_controller()
    controller = gc.GroupController()
    for aid, motor_ids in (("left", [51, 52, 53, 54, 55, 56, 57]), ("right", [61, 62, 63, 64, 65, 66, 67])):
        arm = FakeArm(motor_ids)
        runtime = gc.ArmRuntime(
            arm=arm,
            arm_id=aid,
            sdk_side="right" if aid == "left" else "left",
            can_channel="can0" if aid == "left" else "can1",
            motor_ids=motor_ids,
            motors={mid: gc.MotorCache(position=arm.motors[mid].position) for mid in motor_ids},
            zero_offsets={mid: 0.0 for mid in motor_ids},
            enabled=True,
        )
        controller.arms[aid] = runtime
        controller.arm_states[aid].connected = True
    return controller, gc


def test_set_position_rejects_out_of_limit():
    # 软限位外的目标应被拒绝，且不会下发 set_position
    controller, _ = make_controller_with_fake_arms()
    errors = []
    controller.set_callbacks(on_error=lambda msg: errors.append(msg))

    ok = controller.set_position("left", 51, 99.0)
    assert ok is False
    assert errors
    assert controller.arms["left"].arm.positions_set == []


def test_set_position_accepts_valid_value():
    # 合法位置应通过并转发到底层 arm
    controller, _ = make_controller_with_fake_arms()
    ok = controller.set_position("left", 51, 0.3)
    assert ok is True
    assert controller.arms["left"].arm.positions_set[-1][1] is True
    assert controller.arms["left"].motors[51].position == 0.3


def test_get_target_arms_follows_current_target():
    # target=LEFT/RIGHT/BOTH 时目标臂筛选应正确
    controller, gc = make_controller_with_fake_arms()

    controller.target = gc.ControlTarget.LEFT
    arms = controller._get_target_arms()
    assert [aid for aid, _ in arms] == ["left"]

    controller.target = gc.ControlTarget.RIGHT
    arms = controller._get_target_arms()
    assert [aid for aid, _ in arms] == ["right"]

    controller.target = gc.ControlTarget.BOTH
    arms = controller._get_target_arms()
    assert [aid for aid, _ in arms] == ["left", "right"]


def test_record_point_contains_flat_arm_positions(monkeypatch):
    # 录点结构应包含双臂与字符串电机ID键
    controller, gc = make_controller_with_fake_arms()
    controller.target = gc.ControlTarget.BOTH

    monkeypatch.setattr(controller, "read_positions", lambda: None)
    monkeypatch.setattr(gc.time, "sleep", lambda *_: None)

    point = controller.record_point(name="p_test")
    assert point["name"] == "p_test"
    assert "left" in point["arms"]
    assert "right" in point["arms"]
    assert "51" in point["arms"]["left"]
    assert "61" in point["arms"]["right"]


def test_unknown_speed_preset_emits_error():
    # 非法速度预设应触发错误回调
    controller, _ = make_controller_with_fake_arms()
    errors = []
    controller.set_callbacks(on_error=lambda msg: errors.append(msg))

    controller.set_speed_preset("not_exist")
    assert errors
    assert "未知速度预设" in errors[-1]


def test_prepare_playback_start_rejects_large_first_point_gap():
    controller, _ = make_controller_with_fake_arms()
    errors = []
    controller.set_callbacks(on_error=errors.append)

    ok = controller._prepare_playback_start(
        [{"positions": {"51": controller.playback_start_max_deviation_rad + 0.2}}]
    )

    assert ok is False
    assert controller.arms["left"].arm.positions_set == []
    assert errors


def test_prepare_playback_start_aligns_small_gap_blocking():
    controller, _ = make_controller_with_fake_arms()
    runtime = controller.arms["left"]
    for mid in runtime.motor_ids:
        runtime.motors[mid].position = 0.0
        runtime.arm.motors[mid].position = 0.0

    ok = controller._prepare_playback_start(
        [{"positions": {"51": controller.playback_start_align_tolerance_rad + 0.01}}]
    )

    assert ok is True
    assert controller.arms["left"].arm.positions_set
    assert controller.arms["left"].arm.positions_set[0][1] is True


def test_emergency_stop_requires_recovery_hold_on_reenable():
    controller, _ = make_controller_with_fake_arms()
    runtime = controller.arms["left"]
    arm = runtime.arm

    for mid in runtime.motor_ids:
        runtime.motors[mid].position = 0.1
        arm.motors[mid].position = 0.1

    controller.target = controller.target.LEFT
    controller.emergency_stop()
    assert runtime.recovery_hold_required is True

    ok = controller.init_arm("left")

    assert ok is True
    assert runtime.recovery_hold_required is False
    assert arm.positions_set[-1][0] == tuple([0.1] * 7)
    assert arm.positions_set[-1][1] is True
