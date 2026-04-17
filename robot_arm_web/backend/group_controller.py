"""
群控管理器 - 基于 linkerbot-py(A7) 的双臂控制。

说明:
- 本文件仅保留 A7 直连实现。
- 已移除旧 arm_control 路径，避免运行时实现混淆。
"""

from __future__ import annotations

import json
import threading
import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Callable, Dict, List, Optional

from linkerbot import A7, ControlMode
from linkerbot.arm.a7.motor import A7Motor

from .config import (
    DEFAULT_LEFT_CAN,
    DEFAULT_RIGHT_CAN,
    INIT_HOME_BLOCKING,
    INIT_HOME_ENABLED,
    JOINT_LIMITS,
    LEFT_MOTOR_IDS,
    RIGHT_MOTOR_IDS,
    SAFE_RECOVERY_ACCEL,
    SAFE_RECOVERY_RAMP_INTERVAL_S,
    SAFE_RECOVERY_RAMP_STEPS,
    SAFE_RECOVERY_SETTLE_S,
    SAFE_RECOVERY_VELOCITY,
    SPEED_PRESETS,
    TRAJECTORY_STREAMING_ENABLED,
    TRAJECTORY_START_ALIGN_TOLERANCE_RAD,
    TRAJECTORY_START_MAX_DEVIATION_RAD,
    TRAJECTORY_MAX_SEGMENT_DURATION_S,
    TRAJECTORY_MIN_SEGMENT_DURATION_S,
    TRAJECTORY_PLAYBACK_CONTROL_HZ,
    TRAJECTORIES_DIR,
    ZERO_OFFSET_FILE,
)


def _install_linkerbot_handshake_fallback():
    """
    兼容不同 linkerbot 构建的握手命令差异。
    """
    if getattr(A7Motor, "_handshake_fallback_installed", False):
        return

    origin_check_alive = A7Motor.check_alive

    def _patched_check_alive(self, timeout_s: float = 0.05) -> bool:
        try:
            if origin_check_alive(self, timeout_s):
                return True
        except Exception:
            pass
        try:
            self._read_register(0x01, timeout_s)  # type: ignore[attr-defined]
            return True
        except Exception:
            return False

    A7Motor.check_alive = _patched_check_alive  # type: ignore[assignment]
    A7Motor._handshake_fallback_installed = True  # type: ignore[attr-defined]


_install_linkerbot_handshake_fallback()


class ControlTarget(Enum):
    LEFT = "left"
    RIGHT = "right"
    BOTH = "both"


class PlaybackState(Enum):
    IDLE = "idle"
    PLAYING = "playing"
    PAUSED = "paused"
    STOPPING = "stopping"


@dataclass
class ArmState:
    connected: bool = False
    initialized: bool = False
    can_channel: str = ""
    motors: Dict[int, Dict[str, Any]] = field(default_factory=dict)
    error: Optional[str] = None


@dataclass
class PlaybackInfo:
    state: PlaybackState = PlaybackState.IDLE
    trajectory_name: str = ""
    current_point: int = 0
    total_points: int = 0
    progress: float = 0.0


@dataclass
class MotorCache:
    position: float = 0.0
    velocity: float = 0.0
    enabled: bool = False
    mode: int = 5


@dataclass
class ArmRuntime:
    arm: A7
    arm_id: str
    sdk_side: str
    can_channel: str
    motor_ids: List[int]
    motors: Dict[int, MotorCache] = field(default_factory=dict)
    zero_offsets: Dict[int, float] = field(default_factory=dict)
    enabled: bool = False
    recovery_hold_required: bool = False


class GroupController:
    def __init__(self):
        self.arms: Dict[str, Optional[ArmRuntime]] = {"left": None, "right": None}
        self.arm_states: Dict[str, ArmState] = {
            "left": ArmState(can_channel=DEFAULT_LEFT_CAN),
            "right": ArmState(can_channel=DEFAULT_RIGHT_CAN),
        }
        self.target = ControlTarget.BOTH
        self.playback = PlaybackInfo()

        self._lock = threading.RLock()
        self._playback_thread: Optional[threading.Thread] = None
        self._stop_playback = threading.Event()
        self._pause_playback = threading.Event()

        default_speed = SPEED_PRESETS.get("slow", {"velocity": 0.5, "accel": 0.5, "decel": 0.5})
        self.speed_params = {
            "velocity": float(default_speed.get("velocity", 1.0)),
            "accel": float(default_speed.get("accel", 1.0)),
            "decel": float(default_speed.get("decel", 1.0)),
        }

        self._on_state_update: Optional[Callable] = None
        self._on_playback_progress: Optional[Callable] = None
        self._on_error: Optional[Callable] = None

        self.joint_limits: Dict[str, Dict[int, Dict[str, float]]] = {"left": {}, "right": {}}
        self._init_joint_limits()

        self._zero_offsets_all: Dict[str, Dict[int, float]] = {"left": {}, "right": {}}
        self._load_zero_offsets_all()
        self.connect_retry_count = 3
        self.connect_retry_delay_s = 0.25
        self.playback_control_hz = max(5.0, float(TRAJECTORY_PLAYBACK_CONTROL_HZ))
        self.trajectory_streaming_enabled = bool(TRAJECTORY_STREAMING_ENABLED)
        self.safe_recovery_velocity = max(0.01, float(SAFE_RECOVERY_VELOCITY))
        self.safe_recovery_accel = max(1.0, float(SAFE_RECOVERY_ACCEL))
        self.safe_recovery_settle_s = max(0.0, float(SAFE_RECOVERY_SETTLE_S))
        self.safe_recovery_ramp_steps = max(1, int(SAFE_RECOVERY_RAMP_STEPS))
        self.safe_recovery_ramp_interval_s = max(0.0, float(SAFE_RECOVERY_RAMP_INTERVAL_S))
        self.min_segment_duration_s = max(0.001, float(TRAJECTORY_MIN_SEGMENT_DURATION_S))
        self.max_segment_duration_s = max(
            self.min_segment_duration_s,
            float(TRAJECTORY_MAX_SEGMENT_DURATION_S),
        )
        self.playback_start_align_tolerance_rad = max(0.0, float(TRAJECTORY_START_ALIGN_TOLERANCE_RAD))
        self.playback_start_max_deviation_rad = max(
            self.playback_start_align_tolerance_rad,
            float(TRAJECTORY_START_MAX_DEVIATION_RAD),
        )

    def set_callbacks(
        self,
        on_state_update: Optional[Callable] = None,
        on_playback_progress: Optional[Callable] = None,
        on_error: Optional[Callable] = None,
    ):
        self._on_state_update = on_state_update
        self._on_playback_progress = on_playback_progress
        self._on_error = on_error

    def _emit_error(self, message: str):
        print(f"[GroupController Error] {message}")
        if self._on_error:
            try:
                self._on_error(message)
            except Exception:
                pass

    def _init_joint_limits(self):
        try:
            left_cfg = JOINT_LIMITS.get("left", {})
            right_cfg = JOINT_LIMITS.get("right", {})
            for idx, mid in enumerate(LEFT_MOTOR_IDS):
                mins = left_cfg.get("position_min") or []
                maxs = left_cfg.get("position_max") or []
                self.joint_limits["left"][mid] = {
                    "min": float(mins[idx]) if idx < len(mins) else -3.14,
                    "max": float(maxs[idx]) if idx < len(maxs) else 3.14,
                }
            for idx, mid in enumerate(RIGHT_MOTOR_IDS):
                mins = right_cfg.get("position_min") or []
                maxs = right_cfg.get("position_max") or []
                self.joint_limits["right"][mid] = {
                    "min": float(mins[idx]) if idx < len(mins) else -3.14,
                    "max": float(maxs[idx]) if idx < len(maxs) else 3.14,
                }
        except Exception as e:
            print(f"[GroupController] 初始化关节限位失败，使用默认值: {e}")
            for mid in LEFT_MOTOR_IDS:
                self.joint_limits["left"][mid] = {"min": -3.14, "max": 3.14}
            for mid in RIGHT_MOTOR_IDS:
                self.joint_limits["right"][mid] = {"min": -3.14, "max": 3.14}

    def _get_joint_limit(self, arm_id: str, motor_id: int) -> Optional[Dict[str, float]]:
        return self.joint_limits.get(arm_id, {}).get(motor_id)

    def _arm_to_sdk_side(self, arm_id: str) -> str:
        # A7 内部: side=right -> 51~57, side=left -> 61~67
        return "right" if arm_id == "left" else "left"

    def _arm_motor_ids(self, arm_id: str) -> List[int]:
        return LEFT_MOTOR_IDS if arm_id == "left" else RIGHT_MOTOR_IDS

    def _load_zero_offsets_all(self):
        data: Dict[str, float] = {}
        if ZERO_OFFSET_FILE.exists():
            try:
                with open(ZERO_OFFSET_FILE, "r", encoding="utf-8") as f:
                    raw = json.load(f)
                    if isinstance(raw, dict):
                        data = raw
            except Exception as e:
                print(f"[GroupController] 读取零点文件失败: {e}")

        for arm_id, mids in (("left", LEFT_MOTOR_IDS), ("right", RIGHT_MOTOR_IDS)):
            offsets: Dict[int, float] = {}
            for mid in mids:
                try:
                    offsets[mid] = float(data.get(str(mid), 0.0))
                except Exception:
                    offsets[mid] = 0.0
            self._zero_offsets_all[arm_id] = offsets

    def _save_zero_offsets_all(self):
        payload: Dict[str, float] = {}
        for arm_id in ("left", "right"):
            for mid, val in self._zero_offsets_all.get(arm_id, {}).items():
                payload[str(mid)] = float(val)
        try:
            ZERO_OFFSET_FILE.parent.mkdir(parents=True, exist_ok=True)
            with open(ZERO_OFFSET_FILE, "w", encoding="utf-8") as f:
                json.dump(payload, f, indent=2)
        except Exception as e:
            self._emit_error(f"保存零点失败: {e}")

    def _create_runtime(self, arm_id: str, channel: str) -> ArmRuntime:
        arm = None
        try:
            arm = A7(side=self._arm_to_sdk_side(arm_id), interface_name=channel)
            runtime = ArmRuntime(
                arm=arm,
                arm_id=arm_id,
                sdk_side=self._arm_to_sdk_side(arm_id),
                can_channel=channel,
                motor_ids=self._arm_motor_ids(arm_id),
                motors={mid: MotorCache() for mid in self._arm_motor_ids(arm_id)},
                zero_offsets=dict(self._zero_offsets_all.get(arm_id, {})),
                enabled=False,
            )
            self._refresh_runtime_state(runtime)
            return runtime
        except Exception:
            if arm is not None:
                try:
                    arm.close()
                except Exception:
                    pass
            raise

    def _refresh_runtime_state(self, runtime: ArmRuntime):
        state = runtime.arm.get_state()
        angles = [x.angle for x in state.joint_angles]
        velocities = [x.velocity for x in state.joint_velocities]
        for i, mid in enumerate(runtime.motor_ids):
            runtime.motors[mid].position = float(angles[i])
            runtime.motors[mid].velocity = float(velocities[i])
            runtime.motors[mid].enabled = bool(runtime.enabled)
            runtime.motors[mid].mode = 5

    def _set_runtime_speed(self, runtime: ArmRuntime, velocity: float, accel: float, decel: float):
        v = max(0.0, float(velocity))
        a = min(50.0, max(1.0, float(max(accel, decel))))
        runtime.arm.set_velocities([v] * len(runtime.motor_ids))
        runtime.arm.set_accelerations([a] * len(runtime.motor_ids))

    def _blend_scalar(self, start: float, end: float, alpha: float) -> float:
        return float(start + (end - start) * alpha)

    def safe_recover_arm(
        self,
        arm_id: str,
        *,
        target_angles: Optional[List[float]] = None,
        final_speed: Optional[Dict[str, float]] = None,
        enable_if_needed: bool = False,
        clear_recovery_hold: bool = False,
    ) -> bool:
        with self._lock:
            runtime = self.arms.get(arm_id)
            if runtime is None:
                self._emit_error(f"{arm_id} 臂未连接")
                return False

            try:
                if enable_if_needed and not runtime.enabled:
                    runtime.arm.set_control_mode(ControlMode.PP)
                    runtime.arm.enable()
                    runtime.enabled = True
                    self.arm_states[arm_id].initialized = True

                pose = [float(v) for v in (target_angles if target_angles is not None else self._read_arm_angles(runtime))]
                target_speed = dict(final_speed or self.speed_params)
                safe_speed = {
                    "velocity": self.safe_recovery_velocity,
                    "accel": self.safe_recovery_accel,
                    "decel": self.safe_recovery_accel,
                }

                self._set_runtime_speed(
                    runtime,
                    safe_speed["velocity"],
                    safe_speed["accel"],
                    safe_speed["decel"],
                )
                self._set_arm_angles_direct(runtime, pose, blocking=True)
                if self.safe_recovery_settle_s > 0.0:
                    time.sleep(self.safe_recovery_settle_s)

                for idx in range(1, self.safe_recovery_ramp_steps + 1):
                    alpha = float(idx) / float(self.safe_recovery_ramp_steps)
                    self._set_runtime_speed(
                        runtime,
                        self._blend_scalar(safe_speed["velocity"], float(target_speed.get("velocity", 0.5)), alpha),
                        self._blend_scalar(safe_speed["accel"], float(target_speed.get("accel", 0.5)), alpha),
                        self._blend_scalar(safe_speed["decel"], float(target_speed.get("decel", 0.5)), alpha),
                    )
                    if idx < self.safe_recovery_ramp_steps and self.safe_recovery_ramp_interval_s > 0.0:
                        time.sleep(self.safe_recovery_ramp_interval_s)

                if clear_recovery_hold:
                    runtime.recovery_hold_required = False
                self._refresh_runtime_state(runtime)
                return True
            except Exception as e:
                self._emit_error(f"{arm_id} 臂安全恢复失败: {e}")
                return False

    def _reset_playback_info(self):
        self.playback.trajectory_name = ""
        self.playback.current_point = 0
        self.playback.total_points = 0
        self.playback.progress = 0.0

    def _validate_joint_target(self, arm_id: str, motor_id: int, position: float, *, context: str) -> bool:
        limits = self._get_joint_limit(arm_id, motor_id)
        if limits is None:
            return True
        if limits["min"] <= position <= limits["max"]:
            return True
        self._emit_error(
            f"{context}: {arm_id} 臂电机 {motor_id} 目标位置 {position:.4f} 超出软限位 "
            f"[{limits['min']:.4f}, {limits['max']:.4f}]"
        )
        return False

    def _validate_arm_targets_within_limits(self, arm_targets: Dict[str, List[float]], *, context: str) -> bool:
        for aid, target in arm_targets.items():
            runtime = self.arms.get(aid)
            if runtime is None:
                continue
            for mid, pos in zip(runtime.motor_ids, target):
                if not self._validate_joint_target(aid, mid, float(pos), context=context):
                    return False
        return True

    def connect(self, arm_id: str, can_channel: Optional[str] = None) -> bool:
        if arm_id not in self.arms:
            self._emit_error(f"未知机械臂ID: {arm_id}")
            return False
        with self._lock:
            if self.arms[arm_id] is not None:
                self.disconnect(arm_id)
            channel = can_channel or self.arm_states[arm_id].can_channel
            self.arm_states[arm_id].can_channel = channel
            last_error: Optional[Exception] = None
            for attempt in range(1, self.connect_retry_count + 1):
                try:
                    print(f"[GroupController] 连接 {arm_id} 臂 ({channel})... [attempt {attempt}/{self.connect_retry_count}]")
                    runtime = self._create_runtime(arm_id, channel)
                    self.arms[arm_id] = runtime
                    self.arm_states[arm_id].connected = True
                    self.arm_states[arm_id].error = None
                    return True
                except Exception as e:
                    last_error = e
                    self.arm_states[arm_id].error = str(e)
                    self.arms[arm_id] = None
                    self.arm_states[arm_id].connected = False
                    self.arm_states[arm_id].initialized = False
                    self.arm_states[arm_id].motors = {}
                    self._emit_error(
                        f"{arm_id} 臂连接异常(第 {attempt}/{self.connect_retry_count} 次): {e}"
                    )
                    if attempt < self.connect_retry_count:
                        time.sleep(self.connect_retry_delay_s)
            if last_error is not None:
                self._emit_error(f"{arm_id} 臂连接失败，已重试 {self.connect_retry_count} 次: {last_error}")
            return False

    def connect_all(self, left_channel: str = None, right_channel: str = None) -> Dict[str, bool]:
        return {"left": self.connect("left", left_channel), "right": self.connect("right", right_channel)}

    def disconnect(self, arm_id: str):
        with self._lock:
            runtime = self.arms.get(arm_id)
            if runtime is not None:
                try:
                    runtime.arm.disable()
                except Exception:
                    pass
                try:
                    runtime.arm.close()
                except Exception:
                    pass
            self.arms[arm_id] = None
            self.arm_states[arm_id].connected = False
            self.arm_states[arm_id].initialized = False
            self.arm_states[arm_id].motors = {}
            print(f"[GroupController] {arm_id} 臂已断开")

    def disconnect_all(self):
        self.disconnect("left")
        self.disconnect("right")

    def _get_target_arms(self, arm_id: Optional[str] = None) -> List[tuple[str, ArmRuntime]]:
        arms: List[tuple[str, ArmRuntime]] = []
        if arm_id:
            runtime = self.arms.get(arm_id)
            if runtime:
                arms.append((arm_id, runtime))
            return arms
        if self.target in (ControlTarget.LEFT, ControlTarget.BOTH) and self.arms.get("left"):
            arms.append(("left", self.arms["left"]))
        if self.target in (ControlTarget.RIGHT, ControlTarget.BOTH) and self.arms.get("right"):
            arms.append(("right", self.arms["right"]))
        return arms

    # 机械臂初始化方法（支持急停后的安全恢复）
    # ---------------------------------------------------
    # 主要逻辑：
    # - 检查目标机械臂是否已连接；
    # - 设置为点到点控制模式（PP模式）并使能；
    # - 设置全局运动速度参数；
    # - 如果启用了自动回零（home），则执行归零操作；
    # - 如果处于“急停恢复”场景（recovery_hold_required=True），
    #   通过统一安全恢复入口先低速持位、再渐进恢复常规速度，并清除该标记；
    # - 最后刷新本地机械臂状态。
    # 若过程中发生异常，记录错误信息并报警。

    def init_arm(self, arm_id: str, mode: int = 5) -> bool:
        """
        初始化指定机械臂，支持急停后的安全恢复。

        参数:
            arm_id: 目标臂 id（"left" 或 "right"）。
            mode: 预留参数，当前未用。

        返回:
            是否初始化成功（True/False）。
        """
        del mode  # mode 参数当前未被使用
        with self._lock:
            runtime = self.arms.get(arm_id)
            if runtime is None:
                self._emit_error(f"{arm_id} 臂未连接")
                return False
            try:
                # 1. 设置为 PP 控制模式，2. 使能
                runtime.arm.set_control_mode(ControlMode.PP)
                runtime.arm.enable()
                runtime.enabled = True
                self.arm_states[arm_id].initialized = True
                # 3. 配置运动速度参数
                # 4. 若配置允许，则自动 home
                if INIT_HOME_ENABLED:
                    runtime.arm.home(blocking=INIT_HOME_BLOCKING)
                # 5. 若处于“急停恢复首次使能”，先保持当前位置并用 very_slow 阻塞一次
                if runtime.recovery_hold_required:
                    # 急停后首次重新使能时，先用 very_slow 速度在当前关节角做一次阻塞保持，再恢复常规速度参数，降低刚恢复时的冲击和抖动风险。
                    current_angles = self._read_arm_angles(runtime)
                    if not self.safe_recover_arm(
                        arm_id,
                        target_angles=current_angles,
                        final_speed=self.speed_params,
                        clear_recovery_hold=True,
                    ):
                        return False
                else:
                    self.set_speed(
                        self.speed_params.get("velocity", 0.5),
                        self.speed_params.get("accel", 0.5),
                        self.speed_params.get("decel", 0.5),
                        arm_id=arm_id,
                    )
                self._refresh_runtime_state(runtime) # 刷新机械臂状态
                return True # 返回初始化成功
            except Exception as e:
                self.arm_states[arm_id].error = str(e)
                self._emit_error(f"{arm_id} 臂初始化失败: {e}")
                return False

    def init_target(self, mode: int = 5) -> Dict[str, bool]:
        results: Dict[str, bool] = {}
        if self.target in (ControlTarget.LEFT, ControlTarget.BOTH):
            results["left"] = self.init_arm("left", mode)
        if self.target in (ControlTarget.RIGHT, ControlTarget.BOTH):
            results["right"] = self.init_arm("right", mode)
        return results

    def disable_arm(self, arm_id: Optional[str] = None):
        with self._lock:
            for aid, runtime in self._get_target_arms(arm_id):
                try:
                    runtime.arm.disable()
                    runtime.enabled = False
                    self.arm_states[aid].initialized = False
                    for mid in runtime.motor_ids:
                        runtime.motors[mid].enabled = False
                except Exception as e:
                    self._emit_error(f"{aid} 臂关闭失败: {e}")

    def deactivate_arm(self, arm_id: Optional[str] = None, target: Optional[str] = None):
        if target:
            try:
                self.target = ControlTarget(target)
            except Exception:
                pass
        self.disable_arm(arm_id)

    def emergency_stop(self):
        self.stop_playback()
        with self._lock:
            for aid, runtime in self._get_target_arms():
                try:
                    runtime.arm.emergency_stop()
                    runtime.arm.disable()
                    runtime.enabled = False
                    runtime.recovery_hold_required = True
                    self.arm_states[aid].initialized = False
                except Exception as e:
                    self._emit_error(f"{aid} 臂急停失败: {e}")

    def go_to_zero(self, arm_id: Optional[str] = None):
        with self._lock:
            for aid, runtime in self._get_target_arms(arm_id):
                try:
                    target = [runtime.zero_offsets.get(mid, 0.0) for mid in runtime.motor_ids]
                    self._set_arm_angles_direct(runtime, target, blocking=False)
                    self._refresh_runtime_state(runtime)
                except Exception as e:
                    self._emit_error(f"{aid} 臂回零失败: {e}")

    def hold_current_pose(self, arm_id: Optional[str] = None):
        with self._lock:
            for aid, runtime in self._get_target_arms(arm_id):
                try:
                    current = self._read_arm_angles(runtime)
                    self._set_arm_angles_direct(runtime, current, blocking=True)
                    self._refresh_runtime_state(runtime)
                except Exception as e:
                    self._emit_error(f"{aid} 臂保持当前姿态失败: {e}")

    def _sync_arm_state_to_public(self, arm_id: str, runtime: ArmRuntime):
        motors_data: Dict[int, Dict[str, Any]] = {}
        for mid, m in runtime.motors.items():
            zero_off = runtime.zero_offsets.get(mid, 0.0)
            motors_data[mid] = {
                "position": m.position,
                "relative_position": m.position - zero_off,
                "velocity": m.velocity,
                "enabled": m.enabled,
                "mode": m.mode,
            }
        self.arm_states[arm_id].motors = motors_data

    def get_state(self) -> Dict[str, Any]:
        with self._lock:
            state = {
                "target": self.target.value,
                "playback": {
                    "state": self.playback.state.value,
                    "trajectory_name": self.playback.trajectory_name,
                    "current_point": self.playback.current_point,
                    "total_points": self.playback.total_points,
                    "progress": self.playback.progress,
                },
                "arms": {},
                "joint_limits": {"left": {}, "right": {}},
            }
            for arm_id in ("left", "right"):
                runtime = self.arms.get(arm_id)
                arm_state = self.arm_states[arm_id]
                if runtime is not None:
                    self._sync_arm_state_to_public(arm_id, runtime)
                state["arms"][arm_id] = {
                    "connected": arm_state.connected,
                    "initialized": arm_state.initialized,
                    "can_channel": arm_state.can_channel,
                    "error": arm_state.error,
                    "motors": arm_state.motors,
                }
                for mid, limits in self.joint_limits.get(arm_id, {}).items():
                    state["joint_limits"][arm_id][str(mid)] = {"min": limits["min"], "max": limits["max"]}
            return state

    def read_positions(self, arm_id: Optional[str] = None):
        with self._lock:
            for current_arm_id, runtime in self._get_target_arms(arm_id):
                try:
                    self._refresh_runtime_state(runtime)
                    self._sync_arm_state_to_public(current_arm_id, runtime)
                except Exception:
                    pass

    def read_single_motor_position(self, arm_id: str, motor_id: int) -> Optional[float]:
        with self._lock:
            runtime = self.arms.get(arm_id)
            if runtime is None or motor_id not in runtime.motors:
                return None
            try:
                self._refresh_runtime_state(runtime)
                return runtime.motors[motor_id].position
            except Exception as e:
                self._emit_error(f"读取电机 {motor_id} 位置失败: {e}")
                return None

    def _read_arm_angles(self, runtime: ArmRuntime) -> List[float]:
        arm_obj = runtime.arm if hasattr(runtime, "arm") else runtime
        if hasattr(arm_obj, "get_angles"):
            try:
                return [float(value) for value in arm_obj.get_angles()]
            except Exception:
                pass
        return [float(runtime.motors[mid].position) for mid in runtime.motor_ids]

    def _set_arm_angles_direct(self, runtime: ArmRuntime, target_angles: List[float], *, blocking: bool = False):
        arm_obj = runtime.arm if hasattr(runtime, "arm") else runtime
        if hasattr(arm_obj, "move_j"):
            arm_obj.move_j(target_angles, blocking=blocking)
            return
        if hasattr(arm_obj, "set_position"):
            for mid, pos in zip(runtime.motor_ids, target_angles):
                arm_obj.set_position(mid, float(pos))
            return
        raise AttributeError("arm object does not support move_j or set_position")

    def set_position(self, arm_id: str, motor_id: int, position: float) -> bool:
        with self._lock:
            runtime = self.arms.get(arm_id)
            if runtime is None:
                self._emit_error(f"{arm_id} 臂未连接")
                return False
            motors = getattr(runtime, "motors", {})
            if motor_id not in motors:
                self._emit_error(f"{arm_id} 臂未知电机ID: {motor_id}")
                return False
            limits = self._get_joint_limit(arm_id, motor_id)
            if limits is not None and (position < limits["min"] or position > limits["max"]):
                self._emit_error(
                    f"{arm_id} 臂电机 {motor_id} 目标位置 {position:.4f} 超出软限位 "
                    f"[{limits['min']:.4f}, {limits['max']:.4f}]"
                )
                return False
            try:
                current = self._read_arm_angles(runtime)
                motor_ids = list(getattr(runtime, "motor_ids", list(motors.keys())))
                idx = motor_ids.index(motor_id)
                current[idx] = float(position)
                self._set_arm_angles_direct(runtime, current, blocking=True)

                if hasattr(motors[motor_id], "position"):
                    motors[motor_id].position = float(position)
                return True
            except Exception as e:
                self._emit_error(f"设置位置失败: {e}")
                return False

    def set_position_offset(self, arm_id: str, motor_id: int, position: float) -> bool:
        with self._lock:
            runtime = self.arms.get(arm_id)
            if runtime is None:
                self._emit_error(f"{arm_id} 臂未连接")
                return False
            limits = self._get_joint_limit(arm_id, motor_id)
            if limits is not None and (position < limits["min"] or position > limits["max"]):
                self._emit_error(
                    f"{arm_id} 臂电机 {motor_id} 相对零点目标位置 {position:.4f} 超出软限位 "
                    f"[{limits['min']:.4f}, {limits['max']:.4f}]"
                )
                return False
            return self.set_position(arm_id, motor_id, float(position) + runtime.zero_offsets.get(motor_id, 0.0))

    def set_joint_offsets(self, arm_id: str, joint_offsets: List[float]) -> bool:
        """
        批量设置 7 关节 URDF 角度(相对零点)。
        该接口用于笛卡尔控制, 避免逐关节下发导致的抖动。
        """
        with self._lock:
            runtime = self.arms.get(arm_id)
            if runtime is None:
                self._emit_error(f"{arm_id} 臂未连接")
                return False
            if len(joint_offsets) != len(runtime.motor_ids):
                self._emit_error(f"{arm_id} 臂关节数量不匹配: 期望 {len(runtime.motor_ids)} 实际 {len(joint_offsets)}")
                return False

            # 先做整组软限位检查，全部通过后再一次性下发。
            for idx, mid in enumerate(runtime.motor_ids):
                joint_offset = float(joint_offsets[idx])
                limits = self._get_joint_limit(arm_id, mid)
                if limits is not None and (joint_offset < limits["min"] or joint_offset > limits["max"]):
                    self._emit_error(
                        f"{arm_id} 臂电机 {mid} 相对零点目标位置 {joint_offset:.4f} 超出软限位 "
                        f"[{limits['min']:.4f}, {limits['max']:.4f}]"
                    )
                    return False

            target_abs = [
                float(joint_offsets[idx]) + runtime.zero_offsets.get(mid, 0.0)
                for idx, mid in enumerate(runtime.motor_ids)
            ]
            try:
                self._set_arm_angles_direct(runtime, target_abs, blocking=False)
                return True
            except Exception as e:
                self._emit_error(f"批量设置关节失败: {e}")
                return False

    def set_speed(self, velocity: float, accel: float, decel: float, arm_id: Optional[str] = None):
        with self._lock:
            self.speed_params = {"velocity": float(velocity), "accel": float(accel), "decel": float(decel)}
            for _, runtime in self._get_target_arms(arm_id):
                try:
                    self._set_runtime_speed(runtime, velocity, accel, decel)
                except Exception as e:
                    self._emit_error(f"设置速度失败: {e}")

    def set_speed_preset(self, preset: str, arm_id: Optional[str] = None):
        if preset not in SPEED_PRESETS:
            self._emit_error(f"未知速度预设: {preset}")
            return
        p = SPEED_PRESETS[preset]
        self.set_speed(p["velocity"], p["accel"], p["decel"], arm_id)

    def get_speed_params(self) -> Dict[str, float]:
        with self._lock:
            return dict(self.speed_params)

    def enable_freedrive(self, arm_id: Optional[str] = None):
        with self._lock:
            for aid, runtime in self._get_target_arms(arm_id):
                try:
                    runtime.arm.disable()
                    runtime.enabled = False
                    for mid in runtime.motor_ids:
                        runtime.motors[mid].enabled = False
                except Exception as e:
                    self._emit_error(f"{aid} 自由拖动启用失败: {e}")

    def disable_freedrive(self, arm_id: Optional[str] = None):
        with self._lock:
            for aid, runtime in self._get_target_arms(arm_id):
                try:
                    runtime.arm.enable()
                    runtime.enabled = True
                    for mid in runtime.motor_ids:
                        runtime.motors[mid].enabled = True
                except Exception as e:
                    self._emit_error(f"{aid} 自由拖动退出失败: {e}")

    def calibrate_zero(self, arm_id: Optional[str] = None) -> Dict[str, Any]:
        with self._lock:
            results: Dict[str, Any] = {}
            for aid, runtime in self._get_target_arms(arm_id):
                try:
                    runtime.arm.calibrate_zero()
                    time.sleep(0.2)
                    angles = runtime.arm.get_angles()
                    runtime.zero_offsets = {mid: float(angles[i]) for i, mid in enumerate(runtime.motor_ids)}
                    self._zero_offsets_all[aid] = dict(runtime.zero_offsets)
                    results[aid] = "success"
                except Exception as e:
                    results[aid] = f"error: {e}"
                    self._emit_error(f"{aid} 臂零点标定失败: {e}")
            self._save_zero_offsets_all()
            return results

    def get_zero_offsets(self) -> Dict[str, Dict]:
        with self._lock:
            out: Dict[str, Dict] = {}
            for aid in ("left", "right"):
                runtime = self.arms.get(aid)
                offsets = runtime.zero_offsets if runtime is not None else self._zero_offsets_all.get(aid, {})
                out[aid] = {str(k): float(v) for k, v in offsets.items()}
            return out

    def record_point(self, name: str = None, arm_id: Optional[str] = None) -> Dict[str, Any]:
        with self._lock:
            self.read_positions()
            point = {"name": name or f"point_{int(time.time())}", "timestamp": time.time(), "arms": {}}
            for aid, runtime in self._get_target_arms(arm_id):
                point["arms"][aid] = {str(mid): runtime.motors[mid].position for mid in runtime.motor_ids}
            return point

    def load_trajectory(self, filename: str) -> Optional[Dict]:
        filepath = TRAJECTORIES_DIR / filename
        if not filepath.exists():
            self._emit_error(f"轨迹文件不存在: {filename}")
            return None
        try:
            with open(filepath, "r", encoding="utf-8") as f:
                return json.load(f)
        except Exception as e:
            self._emit_error(f"加载轨迹失败: {e}")
            return None

    def list_trajectories(self) -> List[str]:
        if not TRAJECTORIES_DIR.exists():
            return []
        return [f.name for f in TRAJECTORIES_DIR.glob("*.json")]

    def play_trajectory(self, filename: str, sync: bool = True, loop_override=None):
        del sync
        if self.playback.state == PlaybackState.PLAYING:
            self._emit_error("已有轨迹正在播放")
            return
        trajectory = self.load_trajectory(filename)
        if trajectory is None:
            return
        points = trajectory.get("points", [])
        if not points:
            self._emit_error("轨迹文件没有轨迹点")
            return
        with self._lock:
            if not self._prepare_playback_start(points):
                self.playback.state = PlaybackState.IDLE
                self._reset_playback_info()
                return
        loop = loop_override if loop_override is not None else trajectory.get("loop", False)
        self._stop_playback.clear()
        self._pause_playback.clear()
        self.playback.state = PlaybackState.PLAYING
        self.playback.trajectory_name = trajectory.get("name", filename)
        self.playback.current_point = 0
        self.playback.total_points = len(points)
        self.playback.progress = 0.0
        self._playback_thread = threading.Thread(
            target=self._playback_worker,
            args=(points, bool(loop), float(trajectory.get("speed_multiplier", 1.0))),
            daemon=True,
        )
        self._playback_thread.start()

    def _playback_worker(self, points: List[Dict], loop: bool, speed_mult: float):
        try:
            while True:
                for i, point in enumerate(points):
                    if self._stop_playback.is_set():
                        break
                    while self._pause_playback.is_set():
                        if self._stop_playback.is_set():
                            break
                        time.sleep(0.05)
                    if self._stop_playback.is_set():
                        break
                    self.playback.current_point = i + 1
                    self.playback.progress = (i + 1) / len(points) * 100
                    self._execute_point(point, points, i, speed_mult)
                    if self._on_playback_progress:
                        try:
                            self._on_playback_progress(
                                {
                                    "current": i + 1,
                                    "total": len(points),
                                    "progress": self.playback.progress,
                                    "point_name": point.get("name", ""),
                                }
                            )
                        except Exception:
                            pass
                if not loop or self._stop_playback.is_set():
                    break
                self.playback.current_point = 0
                self.playback.progress = 0.0
        except Exception as e:
            self._emit_error(f"轨迹播放出错: {e}")
        finally:
            self.playback.state = PlaybackState.IDLE
            if not self._stop_playback.is_set():
                self.playback.progress = 100.0
            self._playback_thread = None

    def _effective_segment_duration(self, point: Dict, speed_mult: float) -> float:
        raw_duration = point.get("duration")
        if raw_duration is None:
            raw_duration = point.get("delay", 0.1)
        duration = float(raw_duration) / max(speed_mult, 1e-6)
        return max(self.min_segment_duration_s, min(self.max_segment_duration_s, duration))

    def _effective_hold_duration(self, point: Dict, speed_mult: float) -> float:
        hold = float(point.get("hold", point.get("dwell", 0.0)))
        return max(0.0, hold) / max(speed_mult, 1e-6)

    def _wait_with_pause(self, duration_s: float):
        end_time = time.perf_counter() + max(0.0, duration_s)
        while time.perf_counter() < end_time:
            if self._stop_playback.is_set():
                return
            while self._pause_playback.is_set():
                if self._stop_playback.is_set():
                    return
                time.sleep(0.01)
            remaining = end_time - time.perf_counter()
            if remaining <= 0.0:
                return
            time.sleep(min(0.01, remaining))

    def _build_arm_targets_for_positions(self, positions: Dict[str, Any]) -> Dict[str, List[float]]:
        arm_targets: Dict[str, List[float]] = {}
        for aid, runtime in self._get_target_arms():
            try:
                target = [float(x) for x in runtime.arm.get_angles()]
                updated = False
                for mid_str, pos in positions.items():
                    mid = int(mid_str)
                    if mid in runtime.motor_ids:
                        target[runtime.motor_ids.index(mid)] = float(pos)
                        updated = True
                if updated:
                    arm_targets[aid] = target
            except Exception as e:
                self._emit_error(f"{aid} 读取当前角度失败: {e}")
        return arm_targets

    def _prepare_playback_start(self, points: List[Dict]) -> bool:
        for idx, point in enumerate(points):
            arm_targets = self._build_arm_targets_for_positions(point.get("positions", {}))
            if not self._validate_arm_targets_within_limits(arm_targets, context=f"轨迹点 {idx + 1}"):
                return False

        if not points:
            return False
        first_targets = self._build_arm_targets_for_positions(points[0].get("positions", {}))
        if not first_targets:
            return True
        if not self._validate_arm_targets_within_limits(first_targets, context="轨迹首点"):
            return False

        needs_align = False
        align_diffs: Dict[str, float] = {}
        for aid, target in first_targets.items():
            runtime = self.arms.get(aid)
            if runtime is None:
                continue
            current = self._read_arm_angles(runtime)
            max_delta = max(abs(float(cur) - float(dst)) for cur, dst in zip(current, target))
            align_diffs[aid] = max_delta
            if max_delta > self.playback_start_max_deviation_rad:
                self._emit_error(
                    f"轨迹首点与当前姿态偏差过大: {aid} 臂最大偏差 {max_delta:.4f}rad，"
                    f"超过允许值 {self.playback_start_max_deviation_rad:.4f}rad"
                )
                return False
            if max_delta > self.playback_start_align_tolerance_rad:
                needs_align = True

        if not needs_align:
            return True

        speed_backup = dict(self.speed_params)
        safe_profile = SPEED_PRESETS.get("very_slow", speed_backup)
        try:
            for aid, target in first_targets.items():
                runtime = self.arms.get(aid)
                if runtime is None:
                    continue
                self._set_runtime_speed(
                    runtime,
                    safe_profile.get("velocity", 0.2),
                    safe_profile.get("accel", 1.0),
                    safe_profile.get("decel", 1.0),
                )
                self._set_arm_angles_direct(runtime, target, blocking=True)
                self._refresh_runtime_state(runtime)
        finally:
            for aid, runtime in self._get_target_arms():
                self._set_runtime_speed(
                    runtime,
                    speed_backup.get("velocity", 0.5),
                    speed_backup.get("accel", 0.5),
                    speed_backup.get("decel", 0.5),
                )
        return True

    def _send_arm_targets(self, arm_targets: Dict[str, List[float]]):
        if not self._validate_arm_targets_within_limits(arm_targets, context="轨迹插值"):
            self._stop_playback.set()
            return
        for aid, target in arm_targets.items():
            runtime = self.arms.get(aid)
            if runtime is None:
                continue
            try:
                runtime.arm.move_j(target, blocking=False)
            except Exception as e:
                self._emit_error(f"{aid} 下发轨迹目标失败: {e}")

    def _execute_segment_blocking(self, end_targets: Dict[str, List[float]], duration_s: float):
        start_ts = time.perf_counter()
        with self._lock:
            if not self._validate_arm_targets_within_limits(end_targets, context="轨迹段目标"):
                self._stop_playback.set()
                return
            for aid, target in end_targets.items():
                runtime = self.arms.get(aid)
                if runtime is None:
                    continue
                try:
                    self._set_arm_angles_direct(runtime, target, blocking=True)
                except Exception as e:
                    self._emit_error(f"{aid} 下发轨迹段目标失败: {e}")
                    self._stop_playback.set()
                    return

        remaining = max(0.0, float(duration_s) - (time.perf_counter() - start_ts))
        if remaining > 0.0:
            self._wait_with_pause(remaining)

    def _interpolate_and_execute_segment(
        self,
        start_targets: Dict[str, List[float]],
        end_targets: Dict[str, List[float]],
        duration_s: float,
    ):
        if not self.trajectory_streaming_enabled:
            del start_targets
            self._execute_segment_blocking(end_targets, duration_s)
            return

        dt = 1.0 / max(self.playback_control_hz, 1.0)
        start_ts = time.perf_counter()
        while True:
            if self._stop_playback.is_set():
                return
            while self._pause_playback.is_set():
                if self._stop_playback.is_set():
                    return
                time.sleep(0.01)
            elapsed = time.perf_counter() - start_ts
            alpha = min(1.0, elapsed / max(duration_s, 1e-6))
            step_targets: Dict[str, List[float]] = {}
            for aid, end_vec in end_targets.items():
                start_vec = start_targets.get(aid)
                if start_vec is None or len(start_vec) != len(end_vec):
                    step_targets[aid] = list(end_vec)
                    continue
                step_targets[aid] = [
                    float(start + (end - start) * alpha)
                    for start, end in zip(start_vec, end_vec)
                ]
            with self._lock:
                self._send_arm_targets(step_targets)
            if alpha >= 1.0:
                break
            time.sleep(dt)
        with self._lock:
            self._send_arm_targets(end_targets)

    def _execute_point(self, point: Dict, points: List[Dict], index: int, speed_mult: float):
        positions = point.get("positions", {})
        hold_duration = self._effective_hold_duration(point, speed_mult)

        # 末点或空点位只执行可选停留，不再注入额外点后等待。
        if index >= len(points) - 1 or not positions:
            if hold_duration > 0.0:
                self._wait_with_pause(hold_duration)
            return

        next_positions = points[index + 1].get("positions", {})
        with self._lock:
            start_targets = self._build_arm_targets_for_positions(positions)
            end_targets = self._build_arm_targets_for_positions(next_positions)

        if end_targets:
            duration_s = self._effective_segment_duration(point, speed_mult)
            self._interpolate_and_execute_segment(start_targets, end_targets, duration_s)

        if hold_duration > 0.0:
            self._wait_with_pause(hold_duration)

        with self._lock:
            for aid, runtime in self._get_target_arms():
                try:
                    self._refresh_runtime_state(runtime)
                except Exception as e:
                    self._emit_error(f"{aid} 刷新轨迹状态失败: {e}")

    def pause_playback(self):
        if self.playback.state == PlaybackState.PLAYING:
            self._pause_playback.set()
            self.playback.state = PlaybackState.PAUSED

    def resume_playback(self):
        if self.playback.state == PlaybackState.PAUSED:
            self._pause_playback.clear()
            self.playback.state = PlaybackState.PLAYING

    def stop_playback(self):
        self._stop_playback.set()
        self._pause_playback.clear()
        self.playback.state = PlaybackState.STOPPING
        if self._playback_thread and self._playback_thread.is_alive():
            self._playback_thread.join(timeout=2.0)
        self.playback.state = PlaybackState.IDLE
        self._playback_thread = None
        self._reset_playback_info()

    def set_target(self, target: str):
        try:
            self.target = ControlTarget(target)
        except ValueError:
            self._emit_error(f"无效的控制目标: {target}")

    def cleanup(self):
        self.stop_playback()
        self.disconnect_all()


_controller_instance: Optional[GroupController] = None


def get_controller() -> GroupController:
    global _controller_instance
    if _controller_instance is None:
        _controller_instance = GroupController()
    return _controller_instance
