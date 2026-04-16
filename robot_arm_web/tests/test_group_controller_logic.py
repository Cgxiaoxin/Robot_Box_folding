"""群控核心逻辑测试：通过 FakeArm 隔离硬件依赖。"""

from dataclasses import dataclass
from enum import Enum
import sys
import types


def _install_linkerbot_stubs():
    linkerbot_mod = types.ModuleType("linkerbot")

    class _ControlMode(Enum):
        PP = 1

    class _A7:
        def __init__(self, *args, **kwargs):
            pass

    linkerbot_mod.A7 = _A7
    linkerbot_mod.ControlMode = _ControlMode

    motor_mod = types.ModuleType("linkerbot.arm.a7.motor")

    class _A7Motor:
        _handshake_fallback_installed = False

        def check_alive(self, timeout_s: float = 0.02) -> bool:
            return True

    motor_mod.A7Motor = _A7Motor

    sys.modules["linkerbot"] = linkerbot_mod
    sys.modules["linkerbot.arm"] = types.ModuleType("linkerbot.arm")
    sys.modules["linkerbot.arm.a7"] = types.ModuleType("linkerbot.arm.a7")
    sys.modules["linkerbot.arm.a7.motor"] = motor_mod


_install_linkerbot_stubs()

import backend.group_controller as gc
from backend.group_controller import ControlTarget, GroupController


@dataclass
class FakeMotor:
    # 只保留 GroupController 会访问的最小字段集合
    position: float = 0.0
    velocity: float = 0.0
    enabled: bool = True
    mode: int = 5


class FakeArm:
    def __init__(self, motor_ids):
        # 使用假的 arm 对象隔离真实 CAN/硬件依赖
        self.motor_ids = motor_ids
        self.motors = {mid: FakeMotor(position=float(mid) / 100.0) for mid in motor_ids}
        self.positions_set = []
        self.offset_positions_set = []
        self.zero_offsets = {mid: 0.0 for mid in motor_ids}
        self.profile = []

    def set_position(self, motor_id, position):
        self.positions_set.append((motor_id, position))
        self.motors[motor_id].position = position

    def set_position_with_offset(self, motor_id, position):
        self.offset_positions_set.append((motor_id, position))

    def set_all_profile_velocity(self, v):
        self.profile.append(("v", v))

    def set_all_profile_acceleration(self, a):
        self.profile.append(("a", a))

    def set_all_profile_deceleration(self, d):
        self.profile.append(("d", d))


def make_controller_with_fake_arms():
    # 构造可测试的 controller，避免触发真实硬件通信
    controller = GroupController()
    controller.arms["left"] = FakeArm([51, 52, 53, 54, 55, 56, 57])
    controller.arms["right"] = FakeArm([61, 62, 63, 64, 65, 66, 67])
    return controller


def test_set_position_rejects_out_of_limit():
    # 软限位外的目标应被拒绝，且不会下发 set_position
    controller = make_controller_with_fake_arms()
    errors = []
    controller.set_callbacks(on_error=lambda msg: errors.append(msg))

    ok = controller.set_position("left", 51, 99.0)
    assert ok is False
    assert errors
    assert controller.arms["left"].positions_set == []


def test_set_position_accepts_valid_value():
    # 合法位置应通过并转发到底层 arm
    controller = make_controller_with_fake_arms()
    ok = controller.set_position("left", 51, 0.3)
    assert ok is True
    assert controller.arms["left"].positions_set[-1] == (51, 0.3)


def test_get_target_arms_follows_current_target():
    # target=LEFT/RIGHT/BOTH 时目标臂筛选应正确
    controller = make_controller_with_fake_arms()

    controller.target = ControlTarget.LEFT
    arms = controller._get_target_arms()
    assert [aid for aid, _ in arms] == ["left"]

    controller.target = ControlTarget.RIGHT
    arms = controller._get_target_arms()
    assert [aid for aid, _ in arms] == ["right"]

    controller.target = ControlTarget.BOTH
    arms = controller._get_target_arms()
    assert [aid for aid, _ in arms] == ["left", "right"]


def test_record_point_contains_flat_arm_positions(monkeypatch):
    # 录点结构应包含双臂与字符串电机ID键
    controller = make_controller_with_fake_arms()
    controller.target = ControlTarget.BOTH

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
    controller = make_controller_with_fake_arms()
    errors = []
    controller.set_callbacks(on_error=lambda msg: errors.append(msg))

    controller.set_speed_preset("not_exist")
    assert errors
    assert "未知速度预设" in errors[-1]
