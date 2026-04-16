from types import SimpleNamespace

from backend.drag_teach_service import DragTeachService, DragTeachSession, flatten_record_point_to_positions


class _FakeArmHandle:
    def __init__(self):
        self.enable_calls = 0

    def enable(self):
        self.enable_calls += 1


class _FakeRuntime:
    def __init__(self, motor_ids):
        self.arm = _FakeArmHandle()
        self.enabled = False
        self.motors = {mid: SimpleNamespace(enabled=False) for mid in motor_ids}


class _FakeController:
    def __init__(self):
        self.target = SimpleNamespace(value="left")
        self.playback = SimpleNamespace(state="idle")
        self.arms = {
            "left": _FakeRuntime([51, 52]),
            "right": _FakeRuntime([61, 62]),
        }
        self.speed_calls = []
        self.stop_playback_calls = 0
        self.positions = {
            "left": {"51": 0.1, "52": 0.2},
            "right": {"61": -0.1, "62": -0.2},
        }

    def set_speed_preset(self, preset, arm_id=None):
        self.speed_calls.append((preset, arm_id))

    def stop_playback(self):
        self.stop_playback_calls += 1
        self.playback.state = "idle"

    def record_point(self, name=None, arm_id=None):
        if arm_id in ("left", "right"):
            arms = {arm_id: dict(self.positions[arm_id])}
        elif self.target.value in ("left", "right"):
            arms = {self.target.value: dict(self.positions[self.target.value])}
        else:
            arms = {aid: dict(motors) for aid, motors in self.positions.items()}
        return {"name": name or "point", "timestamp": 123.0, "arms": arms}

    def get_state(self):
        return {}


class _FakeMotionService:
    def __init__(self):
        self.cancel_calls = []
        self._active_by_arm = {
            "left": SimpleNamespace(
                completed=False,
                command=SimpleNamespace(command_id="cmd-1"),
            )
        }

    def cancel(self, sid, payload):
        self.cancel_calls.append((sid, payload))
        runtime = self._active_by_arm.get(payload.get("arm_id"))
        if runtime is not None:
            runtime.completed = True


def test_flatten_record_point_to_positions():
    point = {
        "arms": {
            "left": {"51": 0.1, 52: 0.2},
            "right": {"61": -0.3},
        }
    }
    assert flatten_record_point_to_positions(point) == {
        "51": 0.1,
        "52": 0.2,
        "61": -0.3,
    }


def test_enable_stops_playback_and_cancels_motion(tmp_path):
    controller = _FakeController()
    controller.playback.state = "playing"
    motion_service = _FakeMotionService()
    service = DragTeachService(controller, motion_service=motion_service, trajectories_dir=tmp_path)

    state = service.enable(arm_id="left")

    assert controller.stop_playback_calls == 1
    assert motion_service.cancel_calls
    assert state["arms"]["left"]["mode_enabled"] is True
    assert controller.arms["left"].arm.enable_calls == 1


def test_record_keyframe_and_stop_segment_saves_compatible_json(tmp_path):
    controller = _FakeController()
    service = DragTeachService(controller, trajectories_dir=tmp_path)

    service.enable(arm_id="left")
    service.start_segment(name="press_edge", arm_id="left", sample_type="keyframe")
    point = service.record_keyframe(name="p1", arm_id="left", delay=0.8)
    result = service.stop_segment(save=True)

    assert point["sample_type"] == "keyframe"
    assert result["filename"]
    saved = tmp_path / result["filename"]
    assert saved.exists()
    content = saved.read_text(encoding="utf-8")
    assert '"record_mode": "drag_keyframe"' in content
    assert '"positions"' in content
    assert '"delay": 0.8' in content


def test_remove_last_point_updates_session_state(tmp_path):
    controller = _FakeController()
    service = DragTeachService(controller, trajectories_dir=tmp_path)

    service.enable(arm_id="left")
    service.start_segment(name="segment_remove", arm_id="left")
    service.record_keyframe(name="p1", arm_id="left")
    result = service.remove_last_point()

    assert result["remaining"] == 0
    assert service.get_public_state()["arms"]["left"]["keyframe_count"] == 0


def test_stream_threshold_skips_redundant_samples(tmp_path):
    controller = _FakeController()
    service = DragTeachService(controller, trajectories_dir=tmp_path)
    session = DragTeachSession(
        session_id="s1",
        name="stream_seg",
        arm_id="left",
        arm_scope=["left"],
        sample_type="stream",
        record_mode="drag_stream",
    )

    service._session = session
    first = service._append_current_point(session, sample_type="stream", force=True)
    second = service._append_current_point(session, sample_type="stream", force=False)

    assert first["name"] == "stream_seg_1"
    assert second == {}
    assert len(session.points) == 1

    controller.positions["left"]["51"] = 0.2
    third = service._append_current_point(session, sample_type="stream", force=False)
    assert third["sample_type"] == "stream"
    assert len(session.points) == 2


def test_mit_mode_requires_explicit_config(tmp_path):
    controller = _FakeController()
    service = DragTeachService(controller, trajectories_dir=tmp_path)

    try:
        service.enable(arm_id="left", controller_source="mit_compliance")
    except ValueError as exc:
        assert "mit_config_file" in str(exc)
    else:
        raise AssertionError("expected ValueError when MIT config is missing")
