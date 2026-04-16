"""安全控制器测试：覆盖限位开关、违规检测、事件日志。"""

from backend.safety_controller import JointLimits, SafetyController


def test_position_limit_violations():
    # 超限时应给出对应电机违规信息
    controller = SafetyController()
    controller.set_joint_limits(51, JointLimits(min_position=-1.0, max_position=1.0))

    violations = controller.check_position_limits({51: 1.5})
    assert 51 in violations
    assert "above_max" in violations[51]


def test_limits_can_be_disabled():
    # 关闭限位后，不应再产生越限违规
    controller = SafetyController()
    controller.set_joint_limits(51, JointLimits(min_position=-1.0, max_position=1.0))
    controller.enable_limits(False)

    violations = controller.check_position_limits({51: 10.0})
    assert violations == {}


def test_check_safety_generates_event_and_log():
    # check_safety 应同时产出事件与可查询日志
    controller = SafetyController()
    controller.set_joint_limits(51, JointLimits(min_position=-1.0, max_position=1.0))

    events = controller.check_safety({51: -2.0})
    assert len(events) == 1
    assert events[0].event_type == "position_limit"
    assert events[0].severity == "error"

    logs = controller.get_event_log(limit=10)
    assert len(logs) == 1
    assert logs[0]["type"] == "position_limit"
