from __future__ import annotations

import importlib
import json
import sys
import threading
import time
import uuid
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional

if __package__ is None or __package__ == "":
    project_root = Path(__file__).resolve().parents[2]
    sys.path.insert(0, str(project_root))
    from robot_arm_web.backend.config import SAFE_RECOVERY_PID_SCALE, TRAJECTORIES_DIR
else:
    from .config import SAFE_RECOVERY_PID_SCALE, TRAJECTORIES_DIR


def flatten_record_point_to_positions(point_data: Dict[str, Any]) -> Dict[str, float]:
    positions: Dict[str, float] = {}
    for _, motors in (point_data or {}).get("arms", {}).items():
        if not isinstance(motors, dict):
            continue
        for mid_str, pos in motors.items():
            positions[str(mid_str)] = float(pos)
    return positions


def _state_value(value: Any) -> str:
    if hasattr(value, "value"):
        return str(value.value)
    return str(value)


def _safe_name(name: str) -> str:
    text = (name or "segment").strip().replace(" ", "_").replace("/", "_")
    return text or "segment"


def _tools_dir() -> Path:
    return Path(__file__).resolve().parent.parent / "tools"


def _load_lansi_arm_controller():
    tools_dir = str(_tools_dir())
    if tools_dir not in sys.path:
        sys.path.insert(0, tools_dir)
    module = importlib.import_module("lansi_arm_controller")
    return module.LansiArmController


@dataclass
class DragTeachArmState:
    arm_id: str
    mode_enabled: bool = False
    status: str = "idle"
    controller_source: str = "soft_compliance"
    record_session_id: Optional[str] = None
    segment_name: str = ""
    sample_type: str = "keyframe"
    last_sample_ts: float = 0.0
    last_positions: Dict[str, float] = field(default_factory=dict)
    keyframe_count: int = 0
    stream_recording: bool = False
    error: Optional[str] = None

    def to_public(self) -> Dict[str, Any]:
        return {
            "arm_id": self.arm_id,
            "mode_enabled": self.mode_enabled,
            "status": self.status,
            "controller_source": self.controller_source,
            "record_session_id": self.record_session_id,
            "segment_name": self.segment_name,
            "sample_type": self.sample_type,
            "last_sample_ts": self.last_sample_ts,
            "last_positions": dict(self.last_positions),
            "keyframe_count": self.keyframe_count,
            "stream_recording": self.stream_recording,
            "error": self.error,
        }


@dataclass
class DragTeachSession:
    session_id: str                 # 会话唯一ID
    name: str                       # 会话名/段名
    arm_id: Optional[str]           # 关联的单侧机械臂ID
    arm_scope: List[str]            # 关联臂的作用域（left/right/双臂等，可多臂录制）
    description: str = ""           # 描述
    segment_type: str = "generic"   # 段类型标签
    sample_type: str = "keyframe"   # 采样类型（关键帧、stream流式）
    record_mode: str = "drag_keyframe"  # 录制模式（如拖动关键帧/流式等）
    points: List[Dict[str, Any]] = field(default_factory=list)    # 录制的轨迹点列表
    hand_timeline: List[Dict[str, Any]] = field(default_factory=list)  # 手部同步动作时序
    created_at: str = field(default_factory=lambda: datetime.now().isoformat())  # 创建时间
    speed_multiplier: float = 1.0   # 回放速度倍率
    loop: bool = False              # 是否循环
    metadata: Dict[str, Any] = field(default_factory=dict)     # 额外元数据
    filename: Optional[str] = None  # 关联的轨迹文件名
    last_record_ts: float = 0.0     # 上一次采样时间戳
    stop_event: threading.Event = field(default_factory=threading.Event, repr=False) # 录制停止事件（线程协作）
    stream_thread: Optional[threading.Thread] = field(default=None, repr=False)      # 后台采样线程

    def to_public(self) -> Dict[str, Any]:
        return {
            "session_id": self.session_id,
            "name": self.name,
            "description": self.description,
            "arm_id": self.arm_id,
            "arm_scope": list(self.arm_scope),
            "segment_type": self.segment_type,
            "sample_type": self.sample_type,
            "record_mode": self.record_mode,
            "points": list(self.points),
            "hand_timeline": list(self.hand_timeline),
            "created_at": self.created_at,
            "speed_multiplier": self.speed_multiplier,
            "loop": self.loop,
            "metadata": dict(self.metadata),
            "filename": self.filename,
        }


class _MitComplianceAdapter:
    def __init__(
        self,
        arm_id: str,
        config_file: str,
        kp: float,
        kd: float,
        torque_bias: Dict[int, float],
        poll_interval_s: float = 0.05,
        max_torque: float = 3.0,
    ):
        LansiArmController = _load_lansi_arm_controller()
        self.arm_id = arm_id
        self.kp = float(kp)
        self.kd = float(kd)
        self.torque_bias = {int(k): float(v) for k, v in torque_bias.items()}
        self.poll_interval_s = float(poll_interval_s)
        self.max_torque = abs(float(max_torque))
        self.controller = LansiArmController(config_file=config_file)
        self._stop_event = threading.Event()
        self._thread: Optional[threading.Thread] = None

    def start(self):
        self.controller.clear_all_alarms()
        self.controller.set_all_mode(2)
        self.controller.enable_all()
        self._send_hold_command()
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self):
        self._stop_event.set()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=1.0)
        try:
            self.controller.stop_all()
        except Exception:
            pass
        try:
            self.controller.set_all_mode(5)
        except Exception:
            pass

    def _loop(self):
        while not self._stop_event.wait(self.poll_interval_s):
            self._send_hold_command()

    def _send_hold_command(self):
        positions = self.controller.get_joint_positions()
        commands: Dict[int, Dict[str, float]] = {}
        for mid, pos in positions.items():
            tor = self.torque_bias.get(int(mid), 0.0)
            tor = max(-self.max_torque, min(self.max_torque, tor))
            commands[int(mid)] = {
                "pos": float(pos),
                "vel": 0.0,
                "kp": self.kp,
                "kd": self.kd,
                "tor": tor,
            }
        if commands:
            self.controller.send_mit_commands(commands)

# 拖拽示教服务
class DragTeachService:
    def __init__(
        self,
        controller,
        motion_service=None,
        trajectories_dir: Path | str = TRAJECTORIES_DIR,
        stop_smooth_fn: Optional[Callable[[], None]] = None,
        state_notifier: Optional[Callable[[], None]] = None,
    ):
        self.controller = controller
        self.motion_service = motion_service
        self.trajectories_dir = Path(trajectories_dir)
        self.stop_smooth_fn = stop_smooth_fn
        self.state_notifier = state_notifier
        self.trajectories_dir.mkdir(parents=True, exist_ok=True)

        self._lock = threading.RLock()
        self._arm_states = {
            "left": DragTeachArmState("left"),
            "right": DragTeachArmState("right"),
        }
        self._session: Optional[DragTeachSession] = None
        self._joint_delta_threshold = 0.01
        self._max_sample_gap_s = 0.1
        self._stream_hz = 20.0
        self._mit_adapters: Dict[str, _MitComplianceAdapter] = {}
        self._soft_original_profiles: Dict[str, Dict[str, List[float]]] = {}
        self._safe_recovery_pid_scale = min(1.0, max(0.0, float(SAFE_RECOVERY_PID_SCALE)))

    def get_public_state(self) -> Dict[str, Any]:
        with self._lock:
            return {
                "enabled": any(state.mode_enabled for state in self._arm_states.values()),
                "active_session": self._session.to_public() if self._session else None,
                "arms": {arm_id: state.to_public() for arm_id, state in self._arm_states.items()},
            }

    def enable(
        self,
        arm_id: Optional[str] = None,
        controller_source: str = "soft_compliance",
        mit_config_file: Optional[str] = None,
        mit_kp: float = 8.0,
        mit_kd: float = 1.5,
        mit_torque_bias: Optional[Dict[int, float]] = None,
        soft_position_kp: float = 6.0,
        soft_velocity_kp: float = 0.08,
        soft_velocity_ki: float = 0.0,
        soft_velocity_limit: float = 0.35,
        soft_acceleration: float = 1.0,
    ) -> Dict[str, Any]:
        with self._lock:
            scope = self._resolve_arm_scope(arm_id)
            self._stop_smooth_motion()
            self._stop_playback_if_needed()
            self._cancel_motion_if_needed(scope)
            self._teardown_soft_compliance(scope, use_safe_recovery=False)
            if controller_source == "mit_compliance":
                self._activate_mit_compliance(
                    scope=scope,
                    config_file=mit_config_file,
                    kp=mit_kp,
                    kd=mit_kd,
                    torque_bias=mit_torque_bias or {},
                )
            elif controller_source == "soft_compliance":
                self._activate_soft_compliance(
                    scope=scope,
                    position_kp=soft_position_kp,
                    velocity_kp=soft_velocity_kp,
                    velocity_ki=soft_velocity_ki,
                    velocity_limit=soft_velocity_limit,
                    acceleration=soft_acceleration,
                )
            else:
                self._activate_pp_fallback(scope)

            for aid in scope:
                state = self._arm_states[aid]
                state.mode_enabled = True
                state.status = "dragging"
                state.controller_source = controller_source
                state.error = None
            self._notify_state()
            return self.get_public_state()

    # && 退出拖拽示教
    # 退出拖拽示教模式（disable）函数
    # 主要逻辑说明：
    # 1. 支持指定 arm_id 关闭单臂或全部，支持可选保留当前机械臂姿态（preserve_pose）。
    # 2. 若 preserve_pose=True，会先抓取每个臂当前电机角度做快照，后续用于 hold。
    # 3. 若处于流式录制（stream）模式，则先终止录制 session（不追加最后一点）。
    # 4. 依次释放 mit 及软顺应控制。
    # 5. 若要求保留姿态，则逐臂下发“保持当前姿态”命令，异常则写入 error 字段。
    # 5.1 若 controller 提供 safe_recover_arm，则优先走统一安全恢复入口，避免刚性/速度瞬间恢复。
    # 6. 最后更新各臂状态到“空闲”，并广播全局状态更新。

    def disable(self, arm_id: Optional[str] = None, *, preserve_pose: bool = True) -> Dict[str, Any]:
        with self._lock:
            scope = self._resolve_arm_scope(arm_id)
            pose_snapshot: Dict[str, List[float]] = {}
            
            # 1. 如需保留姿态，先采集目标臂当前各关节角度
            if preserve_pose:
                self.controller.read_positions()
                for aid in scope:
                    runtime = getattr(self.controller, "arms", {}).get(aid)
                    if runtime is None:
                        continue
                    try:
                        pose_snapshot[aid] = [float(runtime.motors[mid].position) for mid in runtime.motor_ids]
                    except Exception:
                        continue

            # 2. 若当前有拖拽录制 session，先终止流式录制（不保存最后采样点）
            if self._session and any(aid in self._session.arm_scope for aid in scope):
                self._stop_stream_session(record_final_sample=False)

            # 3. 依次释放 MIT、软顺应控制
            self._teardown_mit_compliance(scope)
            recovered_arms = self._teardown_soft_compliance(
                scope,
                pose_snapshot=pose_snapshot if preserve_pose else None,
            )

            # 4. 如需保留姿态，逐臂让机械臂保持当前模式下的快照姿态
            if preserve_pose and pose_snapshot:
                try:
                    for aid in scope:
                        if aid in pose_snapshot:
                            if hasattr(self.controller, "safe_recover_arm"):
                                if aid not in recovered_arms:
                                    self.controller.safe_recover_arm(
                                        aid,
                                        target_angles=pose_snapshot[aid],
                                        final_speed=getattr(self.controller, "speed_params", None),
                                    )
                            else:
                                self.controller.hold_current_pose(arm_id=aid)
                except Exception as exc:
                    # 如持位异常，将异常信息记入对应臂状态
                    for aid in scope:
                        self._arm_states[aid].error = str(exc)

            # 5. 全量重置臂的状态标志及 session 相关属性
            for aid in scope:
                state = self._arm_states[aid]
                state.mode_enabled = False
                state.status = "idle"
                state.record_session_id = None
                state.segment_name = ""
                state.stream_recording = False
                state.error = None

            # 6. 推送一次状态变更，返回公共状态对象
            self._notify_state()
            return self.get_public_state()

    def start_segment(
        self,
        name: str,
        *,
        arm_id: Optional[str] = None,
        description: str = "",
        sample_type: str = "keyframe",
        segment_type: str = "generic",
        speed_multiplier: float = 1.0,
        loop: bool = False,
        metadata: Optional[Dict[str, Any]] = None,
    ) -> Dict[str, Any]:
        with self._lock:
            if self._session is not None:
                raise ValueError("已有未结束的 drag teach segment")

            scope = self._resolve_arm_scope(arm_id)
            if not any(self._arm_states[aid].mode_enabled for aid in scope):
                self.enable(arm_id=arm_id, controller_source="pp_fallback")

            record_mode = "drag_stream" if sample_type == "stream" else "drag_keyframe"
            session = DragTeachSession(
                session_id=str(uuid.uuid4()),
                name=name or f"segment_{int(time.time())}",
                arm_id=arm_id,
                arm_scope=scope,
                description=description,
                segment_type=segment_type,
                sample_type=sample_type,
                record_mode=record_mode,
                speed_multiplier=float(speed_multiplier),
                loop=bool(loop),
                metadata=dict(metadata or {}),
            )
            self._session = session
            for aid in scope:
                state = self._arm_states[aid]
                state.record_session_id = session.session_id
                state.segment_name = session.name
                state.sample_type = sample_type
                state.status = "recording_stream" if sample_type == "stream" else "dragging"
                state.stream_recording = sample_type == "stream"
                state.keyframe_count = 0
            if sample_type == "stream":
                self._append_current_point(session, point_name="stream_start", force=True)
                self._start_stream_session(session)
            self._notify_state()
            return session.to_public()

    def record_keyframe(
        self,
        *,
        name: Optional[str] = None,
        arm_id: Optional[str] = None,
        duration: Optional[float] = None,
        delay: Optional[float] = None,
        hold: float = 0.0,
        hand_action: Optional[Dict[str, Any]] = None,
    ) -> Dict[str, Any]:
        with self._lock:
            if self._session is None:
                self.start_segment(
                    name=f"segment_{int(time.time())}",
                    arm_id=arm_id,
                    sample_type="keyframe",
                )
            if self._session is None:
                raise ValueError("drag teach session 创建失败")
            point = self._append_current_point(
                self._session,
                point_name=name,
                duration=duration,
                delay=delay,
                hold=float(hold),
                sample_type="keyframe",
                force=True,
                hand_action=hand_action,
            )
            self._notify_state()
            return point

    def remove_last_point(self) -> Dict[str, Any]:
        with self._lock:
            if self._session is None:
                raise ValueError("当前没有正在编辑的 drag teach segment")
            if not self._session.points:
                raise ValueError("当前 segment 没有可删除的点位")
            removed = self._session.points.pop()
            self._sync_counts_locked()
            self._notify_state()
            return {
                "removed": removed,
                "remaining": len(self._session.points),
                "session": self._session.to_public(),
            }

    def append_hand_action(self, action: Dict[str, Any]) -> Dict[str, Any]:
        with self._lock:
            if self._session is None:
                raise ValueError("当前没有正在编辑的 drag teach segment")
            event = {
                "timestamp": time.time(),
                "action": dict(action or {}),
            }
            self._session.hand_timeline.append(event)
            if self._session.points:
                self._session.points[-1]["hand_action"] = dict(action or {})
            self._notify_state()
            return event

    def stop_segment(self, *, save: bool = True) -> Dict[str, Any]:
        with self._lock:
            if self._session is None:
                raise ValueError("当前没有正在录制的 drag teach segment")
            session = self._session
        self._stop_stream_session(record_final_sample=True)
        with self._lock:
            if self._session is None:
                raise ValueError("drag teach session 已丢失")
            session = self._session
            payload = session.to_public()
            if save:
                filename = self._save_session_locked(session)
                payload["filename"] = filename
            self._clear_session_locked()
            self._notify_state()
            return payload

    def cancel_segment(self) -> Dict[str, Any]:
        with self._lock:
            if self._session is None:
                raise ValueError("当前没有正在录制的 drag teach segment")
        self._stop_stream_session(record_final_sample=False)
        with self._lock:
            payload = self._session.to_public() if self._session else {}
            self._clear_session_locked()
            self._notify_state()
            return payload

    def preview_segment(self, filename: Optional[str] = None) -> Dict[str, Any]:
        with self._lock:
            if filename is None and self._session is not None:
                return self._session.to_public()
        if not filename:
            raise ValueError("没有可预览的 drag teach segment")
        path = self.trajectories_dir / filename
        if not path.exists():
            raise ValueError(f"轨迹文件不存在: {filename}")
        with open(path, "r", encoding="utf-8") as f:
            return json.load(f)

    def list_segments(self) -> List[Dict[str, Any]]:
        files: List[Dict[str, Any]] = []
        for file_path in sorted(self.trajectories_dir.glob("*.json")):
            try:
                with open(file_path, "r", encoding="utf-8") as f:
                    data = json.load(f)
                if str(data.get("record_mode", "")).startswith("drag_") or data.get("segment_name"):
                    files.append(
                        {
                            "filename": file_path.name,
                            "name": data.get("name", file_path.stem),
                            "segment_name": data.get("segment_name", ""),
                            "segment_type": data.get("segment_type", ""),
                            "record_mode": data.get("record_mode", ""),
                            "points_count": len(data.get("points", [])),
                            "arm_id": data.get("arm_id"),
                        }
                    )
            except Exception:
                continue
        return files

    def _resolve_arm_scope(self, arm_id: Optional[str]) -> List[str]:
        if arm_id in ("left", "right"):
            return [str(arm_id)]
        target = getattr(self.controller, "target", None)
        target_value = getattr(target, "value", target)
        if target_value in ("left", "right"):
            return [str(target_value)]
        connected = [
            aid for aid in ("left", "right")
            if getattr(self.controller, "arms", {}).get(aid) is not None
        ]
        return connected or ["left", "right"]

    def _stop_playback_if_needed(self):
        playback = getattr(self.controller, "playback", None)
        if playback is None:
            return
        state = _state_value(getattr(playback, "state", "idle"))
        if state in {"playing", "paused", "stopping"}:
            self.controller.stop_playback()

    def _cancel_motion_if_needed(self, scope: List[str]):
        if self.motion_service is None:
            return
        active_by_arm = getattr(self.motion_service, "_active_by_arm", {})
        for aid in scope:
            runtime = active_by_arm.get(aid)
            if runtime is not None and not getattr(runtime, "completed", False):
                payload = {"arm_id": aid, "command_id": runtime.command.command_id}
                self.motion_service.cancel("drag_teach", payload)

    def _stop_smooth_motion(self):
        if self.stop_smooth_fn is not None:
            self.stop_smooth_fn()

    def _activate_pp_fallback(self, scope: List[str]):
        for aid in scope:
            try:
                self.controller.set_speed_preset("very_slow", aid)
            except Exception:
                pass
            runtime = getattr(self.controller, "arms", {}).get(aid)
            if runtime is None:
                continue
            arm = getattr(runtime, "arm", None)
            if arm is not None:
                try:
                    arm.enable()
                except Exception:
                    pass
            runtime.enabled = True
            for motor in getattr(runtime, "motors", {}).values():
                try:
                    motor.enabled = True
                except Exception:
                    pass

    def _read_pid_profile(self, arm_obj: Any) -> Optional[Dict[str, List[float]]]:
        motors = getattr(arm_obj, "_motors", None)
        if not motors:
            return None
        try:
            return {
                "position_kp": [float(getattr(m, "position_kp")) for m in motors],
                "velocity_kp": [float(getattr(m, "velocity_kp")) for m in motors],
                "velocity_ki": [float(getattr(m, "velocity_ki")) for m in motors],
                "velocity": [float(getattr(m, "control_velocity").velocity) for m in motors],
                "acceleration": [float(getattr(m, "control_acceleration").acceleration) for m in motors],
            }
        except Exception:
            return None

    def _apply_pid_profile(self, arm_obj: Any, profile: Dict[str, List[float]]):
        if "position_kp" in profile:
            arm_obj.set_position_kps(profile["position_kp"])
        if "velocity_kp" in profile:
            arm_obj.set_velocity_kps(profile["velocity_kp"])
        if "velocity_ki" in profile:
            arm_obj.set_velocity_kis(profile["velocity_ki"])
        if "velocity" in profile:
            arm_obj.set_velocities(profile["velocity"])
        if "acceleration" in profile:
            arm_obj.set_accelerations(profile["acceleration"])

    def _build_safe_restore_profile(self, profile: Dict[str, List[float]]) -> Dict[str, List[float]]:
        safe_profile: Dict[str, List[float]] = {}
        for key in ("position_kp", "velocity_kp", "velocity_ki"):
            values = profile.get(key)
            if values:
                safe_profile[key] = [float(v) * self._safe_recovery_pid_scale for v in values]
        velocities = profile.get("velocity")
        if velocities:
            safe_limit = max(0.01, float(getattr(self.controller, "safe_recovery_velocity", 0.15)))
            safe_profile["velocity"] = [min(float(v), safe_limit) for v in velocities]
        accelerations = profile.get("acceleration")
        if accelerations:
            safe_acc = max(1.0, float(getattr(self.controller, "safe_recovery_accel", 1.0)))
            safe_profile["acceleration"] = [min(float(v), safe_acc) for v in accelerations]
        return safe_profile

    def _activate_soft_compliance(
        self,
        *,
        scope: List[str],
        position_kp: float,
        velocity_kp: float,
        velocity_ki: float,
        velocity_limit: float,
        acceleration: float,
    ):
        pos_kp = max(0.0, float(position_kp))
        vel_kp = max(0.0, float(velocity_kp))
        vel_ki = max(0.0, float(velocity_ki))
        vel_lim = max(0.0, min(50.0, float(velocity_limit)))
        acc_lim = max(1.0, min(50.0, float(acceleration)))

        for aid in scope:
            runtime = getattr(self.controller, "arms", {}).get(aid)
            if runtime is None:
                continue
            arm = getattr(runtime, "arm", None)
            if arm is None:
                continue
            if aid not in self._soft_original_profiles:
                prof = self._read_pid_profile(arm)
                if prof:
                    self._soft_original_profiles[aid] = prof
            try:
                arm.enable()
            except Exception:
                pass
            # PP 模式下通过低增益实现“低阻尼/可拖动”效果。
            self._apply_pid_profile(
                arm,
                {
                    "position_kp": [pos_kp] * 7,
                    "velocity_kp": [vel_kp] * 7,
                    "velocity_ki": [vel_ki] * 7,
                    "velocity": [vel_lim] * 7,
                    "acceleration": [acc_lim] * 7,
                },
            )
            runtime.enabled = True
            for motor in getattr(runtime, "motors", {}).values():
                try:
                    motor.enabled = True
                except Exception:
                    pass

    def _teardown_soft_compliance(
        self,
        scope: List[str],
        pose_snapshot: Optional[Dict[str, List[float]]] = None,
        use_safe_recovery: bool = True,
    ):
        recovered_arms = set()
        for aid in scope:
            runtime = getattr(self.controller, "arms", {}).get(aid)
            if runtime is None:
                self._soft_original_profiles.pop(aid, None)
                continue
            arm = getattr(runtime, "arm", None)
            if arm is None:
                self._soft_original_profiles.pop(aid, None)
                continue
            profile = self._soft_original_profiles.pop(aid, None)
            if not profile:
                continue
            try:
                safe_profile = self._build_safe_restore_profile(profile)
                if safe_profile:
                    self._apply_pid_profile(arm, safe_profile)
                target_angles = None if pose_snapshot is None else pose_snapshot.get(aid)
                if use_safe_recovery and hasattr(self.controller, "safe_recover_arm"):
                    self.controller.safe_recover_arm(
                        aid,
                        target_angles=target_angles,
                        final_speed=getattr(self.controller, "speed_params", None),
                    )
                    recovered_arms.add(aid)
                self._apply_pid_profile(arm, profile)
            except Exception:
                pass
        return recovered_arms

    def _activate_mit_compliance(
        self,
        *,
        scope: List[str],
        config_file: Optional[str],
        kp: float,
        kd: float,
        torque_bias: Dict[int, float],
    ):
        if len(scope) != 1:
            raise ValueError("MIT 顺从控制当前仅支持单臂启用")
        if not config_file:
            raise ValueError("启用 MIT 顺从控制需要提供 mit_config_file")
        aid = scope[0]
        self._teardown_mit_compliance(scope)
        adapter = _MitComplianceAdapter(
            arm_id=aid,
            config_file=config_file,
            kp=kp,
            kd=kd,
            torque_bias=torque_bias,
        )
        adapter.start()
        self._mit_adapters[aid] = adapter

    def _teardown_mit_compliance(self, scope: List[str]):
        for aid in scope:
            adapter = self._mit_adapters.pop(aid, None)
            if adapter is not None:
                adapter.stop()

    def _start_stream_session(self, session: DragTeachSession):
        session.stop_event.clear()
        session.stream_thread = threading.Thread(
            target=self._stream_worker,
            args=(session.session_id,),
            daemon=True,
        )
        session.stream_thread.start()

    def _stop_stream_session(self, record_final_sample: bool):
        with self._lock:
            session = self._session
            if session is None or session.sample_type != "stream":
                return
            thread = session.stream_thread
            session.stop_event.set()
        if thread is not None and thread.is_alive():
            thread.join(timeout=1.0)
        with self._lock:
            if self._session is session and record_final_sample:
                self._append_current_point(
                    session,
                    point_name="stream_end",
                    sample_type="stream",
                    force=True,
                )
            for aid in session.arm_scope:
                self._arm_states[aid].stream_recording = False
                if self._arm_states[aid].mode_enabled:
                    self._arm_states[aid].status = "dragging"

    def _stream_worker(self, session_id: str):
        while True:
            with self._lock:
                session = self._session
                if session is None or session.session_id != session_id:
                    return
                if session.stop_event.is_set():
                    return
            time.sleep(max(1.0 / max(self._stream_hz, 1.0), 0.02))
            try:
                with self._lock:
                    session = self._session
                    if session is None or session.session_id != session_id or session.stop_event.is_set():
                        return
                    self._append_current_point(session, sample_type="stream", force=False)
            except Exception as exc:
                with self._lock:
                    for aid in (session.arm_scope if session else []):
                        self._arm_states[aid].status = "error"
                        self._arm_states[aid].error = str(exc)
                self._notify_state()
                return

    def _append_current_point(
        self,
        session: DragTeachSession,
        *,
        point_name: Optional[str] = None,
        duration: Optional[float] = None,
        delay: Optional[float] = None,
        hold: float = 0.0,
        sample_type: Optional[str] = None,
        force: bool,
        hand_action: Optional[Dict[str, Any]] = None,
    ) -> Dict[str, Any]:
        requested_arm = session.arm_id if session.arm_id in ("left", "right") else None
        point_data = self.controller.record_point(point_name, requested_arm)
        positions = flatten_record_point_to_positions(point_data)
        now = time.time()
        sample_kind = sample_type or session.sample_type
        if sample_kind == "stream" and not force and not self._should_record_stream_sample(session, positions, now):
            return {}

        if sample_kind == "stream":
            point_duration = 0.0
            point_hold = 0.0
            if session.points:
                segment_duration = max(0.0, now - session.last_record_ts)
                session.points[-1]["duration"] = segment_duration
                session.points[-1]["delay"] = segment_duration
        else:
            if duration is None:
                duration = delay
            point_duration = float(duration if duration is not None else 0.1)
            point_hold = max(0.0, float(hold or 0.0))

        point = {
            "name": point_name or f"{session.name}_{len(session.points) + 1}",
            "positions": positions,
            "duration": point_duration,
            "hold": point_hold,
            # 兼容旧读取逻辑：将 delay 映射到 duration。
            "delay": point_duration,
            "sample_type": sample_kind,
            "timestamp": now,
            "segment_marker": session.name,
        }
        if hand_action is not None:
            point["hand_action"] = dict(hand_action)
        session.points.append(point)
        session.last_record_ts = now

        for aid in session.arm_scope:
            state = self._arm_states[aid]
            state.last_sample_ts = now
            state.last_positions = dict(positions)
        self._sync_counts_locked()
        return point

    def _should_record_stream_sample(
        self,
        session: DragTeachSession,
        positions: Dict[str, float],
        now: float,
    ) -> bool:
        if not session.points:
            return True
        last_point = session.points[-1]
        last_positions = last_point.get("positions", {})
        if not last_positions:
            return True
        max_delta = max(
            abs(float(positions.get(mid, 0.0)) - float(last_positions.get(mid, 0.0)))
            for mid in set(positions) | set(last_positions)
        )
        if max_delta >= self._joint_delta_threshold:
            return True
        last_ts = float(last_point.get("timestamp", session.last_record_ts) or 0.0)
        return (now - last_ts) >= self._max_sample_gap_s

    def _sync_counts_locked(self):
        if self._session is None:
            for state in self._arm_states.values():
                state.keyframe_count = 0
            return
        count = len(self._session.points)
        for aid in self._session.arm_scope:
            self._arm_states[aid].keyframe_count = count

    def _save_session_locked(self, session: DragTeachSession) -> str:
        filename = f"drag_{_safe_name(session.name)}_{int(time.time())}.json"
        payload = {
            "name": session.name,
            "description": session.description,
            "points": session.points,
            "loop": session.loop,
            "speed_multiplier": session.speed_multiplier,
            "created_at": session.created_at,
            "segment_type": session.segment_type,
            "segment_name": session.name,
            "record_mode": session.record_mode,
            "arm_id": session.arm_id,
            "arm_scope": list(session.arm_scope),
            "hand_timeline": list(session.hand_timeline),
            "metadata": dict(session.metadata),
        }
        with open(self.trajectories_dir / filename, "w", encoding="utf-8") as f:
            json.dump(payload, f, ensure_ascii=False, indent=2)
        session.filename = filename
        return filename

    def _clear_session_locked(self):
        session = self._session
        self._session = None
        if session is not None:
            for aid in session.arm_scope:
                state = self._arm_states[aid]
                state.record_session_id = None
                state.segment_name = ""
                state.sample_type = "keyframe"
                state.stream_recording = False
                state.keyframe_count = 0
                if state.mode_enabled:
                    state.status = "dragging"
                else:
                    state.status = "idle"

    def _notify_state(self):
        if self.state_notifier is not None:
            try:
                self.state_notifier()
            except Exception:
                pass


_drag_teach_service: Optional[DragTeachService] = None


def get_drag_teach_service(
    controller,
    motion_service=None,
    trajectories_dir: Path | str = TRAJECTORIES_DIR,
    stop_smooth_fn: Optional[Callable[[], None]] = None,
    state_notifier: Optional[Callable[[], None]] = None,
) -> DragTeachService:
    global _drag_teach_service
    if _drag_teach_service is None:
        _drag_teach_service = DragTeachService(
            controller=controller,
            motion_service=motion_service,
            trajectories_dir=trajectories_dir,
            stop_smooth_fn=stop_smooth_fn,
            state_notifier=state_notifier,
        )
    else:
        if motion_service is not None:
            _drag_teach_service.motion_service = motion_service
        if stop_smooth_fn is not None:
            _drag_teach_service.stop_smooth_fn = stop_smooth_fn
        if state_notifier is not None:
            _drag_teach_service.state_notifier = state_notifier
    return _drag_teach_service
