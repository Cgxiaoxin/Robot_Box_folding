"""
统一笛卡尔运动服务：协议适配、状态广播与执行控制。
"""

from __future__ import annotations

import math
import time
import uuid
import threading
from copy import deepcopy
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable, Dict, Optional

import eventlet

from .motion_types import (
    ErrorCode,
    ERROR_MESSAGES,
    ExecutionRequest,
    ExecutionResult,
    Frame,
    MotionCommand,
    MotionState,
    MotionType,
)
from .adapters import A7Adapter, HandAdapter


@dataclass
class _Runtime:
    command: MotionCommand
    sid: str
    created_at: float
    cancel_requested: bool = False
    pause_requested: bool = False
    paused_sent: bool = False
    completed: bool = False
    state: str = MotionState.ACCEPTED.value
    timeout_ms: int = 0
    safe_abort_action: Optional[str] = None


class MotionService:
    def __init__(
        self,
        controller,
        cartesian,
        emit_fn: Callable[..., None],
        get_joints_fn: Callable[[Any, str], list],
        hand_config: Optional[Dict[str, Any]] = None,
    ):
        self.controller = controller
        self.cartesian = cartesian
        self.emit = emit_fn
        self.get_joints = get_joints_fn
        self._lock = threading.RLock()
        self._active_by_arm: Dict[str, _Runtime] = {}
        self._a7_adapter = A7Adapter(controller)
        self._hand_adapter = HandAdapter(config=hand_config or {})
        self._registry_path = Path(__file__).resolve().parent / "action_registry.yaml"
        self._action_registry = self._load_action_registry()

    @staticmethod
    def _now_ms() -> int:
        return int(time.time() * 1000)

    def _emit(self, event: str, payload: Dict[str, Any], sid: Optional[str] = None):
        if sid:
            self.emit(event, payload, to=sid)
        self.emit(event, payload)

    def _mk_error(self, code: ErrorCode, detail: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
        return {
            "error_code": code.value,
            "message": ERROR_MESSAGES.get(code.value, code.value),
            "detail": detail or {},
            "server_ts": self._now_ms(),
        }

    def _emit_rejected(self, sid: str, command: MotionCommand, code: ErrorCode, detail: Optional[Dict[str, Any]] = None):
        payload = {
            "command_id": command.command_id,
            "arm_id": command.arm_id,
            "motion_type": command.motion_type.value,
            "state": "rejected",
            "deprecated": command.deprecated,
        }
        payload.update(self._mk_error(code, detail))
        self._emit("motion:rejected", payload, sid=sid)

    def _emit_status(self, rt: _Runtime, state: MotionState, progress: Optional[float] = None, extra: Optional[Dict[str, Any]] = None):
        payload = {
            "command_id": rt.command.command_id,
            "arm_id": rt.command.arm_id,
            "motion_type": rt.command.motion_type.value,
            "state": state.value,
            "deprecated": rt.command.deprecated,
            "server_ts": self._now_ms(),
        }
        if progress is not None:
            payload["progress"] = max(0.0, min(1.0, float(progress)))
        if extra:
            payload.update(extra)

        if state == MotionState.ACCEPTED:
            self._emit("motion:accepted", payload, sid=rt.sid)
        elif state == MotionState.COMPLETED:
            self._emit("motion:completed", payload, sid=rt.sid)
        elif state == MotionState.CANCELLED:
            self._emit("motion:cancelled", payload, sid=rt.sid)
        elif state == MotionState.PAUSED:
            self._emit("motion:paused", payload, sid=rt.sid)
        else:
            self._emit("motion:status", payload, sid=rt.sid)
        rt.state = state.value

    def _emit_failed(self, rt: _Runtime, code: ErrorCode, detail: Optional[Dict[str, Any]] = None):
        payload = {
            "command_id": rt.command.command_id,
            "arm_id": rt.command.arm_id,
            "motion_type": rt.command.motion_type.value,
            "state": MotionState.FAILED.value,
            "deprecated": rt.command.deprecated,
        }
        payload.update(self._mk_error(code, detail))
        self._emit("motion:failed", payload, sid=rt.sid)
        rt.state = MotionState.FAILED.value

    def _is_arm_ready(self, arm_id: str) -> bool:
        arm = self.controller.arms.get(arm_id)
        arm_state = self.controller.arm_states.get(arm_id)
        return bool(arm is not None and arm_state and arm_state.connected and arm_state.initialized)

    def _validate_constraints(self, constraints: Dict[str, Any]) -> bool:
        numeric_fields = (
            "max_velocity",
            "max_acceleration",
            "max_angular_velocity",
            "max_angular_acceleration",
        )
        for key in numeric_fields:
            if key in constraints:
                try:
                    val = float(constraints[key])
                except Exception:
                    return False
                if not math.isfinite(val) or val <= 0:
                    return False
        return True

    def _validate_target(self, cmd: MotionCommand) -> bool:
        if cmd.motion_type in (MotionType.CARTESIAN_PTP, MotionType.CARTESIAN_LINEAR):
            pose = cmd.target.get("pose") if isinstance(cmd.target, dict) else None
            if not isinstance(pose, dict):
                return False
            for k in ("x", "y", "z"):
                if k not in pose:
                    return False
                try:
                    val = float(pose.get(k))
                except Exception:
                    return False
                if not math.isfinite(val):
                    return False
            # 姿态字段可选；若提供需为数值
            for k in ("rx", "ry", "rz"):
                if k in pose and pose.get(k) is not None:
                    try:
                        val = float(pose.get(k))
                    except Exception:
                        return False
                    if not math.isfinite(val):
                        return False
            return True

        if cmd.motion_type == MotionType.CARTESIAN_JOG:
            delta = cmd.target.get("delta") if isinstance(cmd.target, dict) else None
            if not isinstance(delta, dict):
                return False
            keys = ("dx", "dy", "dz", "drx", "dry", "drz")
            has_any = False
            for k in keys:
                if k in delta:
                    has_any = True
                    try:
                        val = float(delta.get(k, 0.0))
                    except Exception:
                        return False
                    if not math.isfinite(val):
                        return False
            return has_any

        return False

    def _make_command(self, payload: Dict[str, Any], legacy_event: Optional[str] = None, deprecated: bool = False) -> MotionCommand:
        motion_type = MotionType(payload.get("motion_type", MotionType.CARTESIAN_PTP.value))
        frame = Frame(payload.get("frame", Frame.BASE.value))
        command_id = payload.get("command_id") or str(uuid.uuid4())
        target = payload.get("target") or {}
        constraints = payload.get("constraints") or {}
        options = payload.get("options") or {}
        return MotionCommand(
            command_id=command_id,
            arm_id=payload.get("arm_id", "left"),
            motion_type=motion_type,
            frame=frame,
            target=target,
            constraints=constraints,
            options=options,
            legacy_event=legacy_event,
            deprecated=deprecated,
        )

    @staticmethod
    def _merge_dict(base: Dict[str, Any], override: Dict[str, Any]) -> Dict[str, Any]:
        result = deepcopy(base)
        for key, value in (override or {}).items():
            if isinstance(value, dict) and isinstance(result.get(key), dict):
                result[key] = MotionService._merge_dict(result[key], value)
            else:
                result[key] = value
        return result

    def _load_action_registry(self) -> Dict[str, Any]:
        if not self._registry_path.exists():
            return {}
        try:
            import yaml  # type: ignore
        except Exception:
            return {}
        try:
            with open(self._registry_path, "r", encoding="utf-8") as f:
                raw = yaml.safe_load(f) or {}
            actions = raw.get("actions") if isinstance(raw, dict) else {}
            return actions if isinstance(actions, dict) else {}
        except Exception:
            return {}

    def _refresh_action_registry(self):
        self._action_registry = self._load_action_registry()

    @staticmethod
    def _to_error_code(code: Any) -> ErrorCode:
        if isinstance(code, ErrorCode):
            return code
        if isinstance(code, str):
            try:
                return ErrorCode(code)
            except Exception:
                return ErrorCode.INTERNAL_ERROR
        return ErrorCode.INTERNAL_ERROR

    def _submit_command(
        self,
        sid: str,
        cmd: MotionCommand,
        allow_preempt: bool = False,
    ) -> Dict[str, Any]:
        if cmd.arm_id not in ("left", "right"):
            detail = {"arm_id": cmd.arm_id}
            self._emit_rejected(sid, cmd, ErrorCode.CONSTRAINT_VIOLATION, detail)
            return {"accepted": False, "error_code": ErrorCode.CONSTRAINT_VIOLATION, "detail": detail}
        if not self._validate_constraints(cmd.constraints):
            detail = {"constraints": cmd.constraints}
            self._emit_rejected(sid, cmd, ErrorCode.CONSTRAINT_VIOLATION, detail)
            return {"accepted": False, "error_code": ErrorCode.CONSTRAINT_VIOLATION, "detail": detail}
        if not self._validate_target(cmd):
            detail = {"target": cmd.target}
            self._emit_rejected(sid, cmd, ErrorCode.CONSTRAINT_VIOLATION, detail)
            return {"accepted": False, "error_code": ErrorCode.CONSTRAINT_VIOLATION, "detail": detail}
        if not self._is_arm_ready(cmd.arm_id):
            self._emit_rejected(sid, cmd, ErrorCode.ARM_NOT_CONNECTED)
            return {"accepted": False, "error_code": ErrorCode.ARM_NOT_CONNECTED, "detail": {}}

        with self._lock:
            active = self._active_by_arm.get(cmd.arm_id)
            if active and not active.completed:
                if allow_preempt:
                    active.cancel_requested = True
                    self._emit_failed(active, ErrorCode.COMMAND_PREEMPTED, {"preempted_by": cmd.command_id})
                else:
                    detail = {"active_command_id": active.command.command_id}
                    self._emit_rejected(sid, cmd, ErrorCode.COMMAND_IN_PROGRESS, detail)
                    return {"accepted": False, "error_code": ErrorCode.COMMAND_IN_PROGRESS, "detail": detail}

            options = cmd.options if isinstance(cmd.options, dict) else {}
            timeout_ms = int(options.get("timeout_ms", 0) or 0)
            safe_abort_action = options.get("safe_abort_action")
            rt = _Runtime(
                command=cmd,
                sid=sid,
                created_at=time.time(),
                timeout_ms=timeout_ms,
                safe_abort_action=safe_abort_action,
            )
            self._active_by_arm[cmd.arm_id] = rt

        self._emit_status(rt, MotionState.ACCEPTED)
        eventlet.spawn_n(self._run_command, rt)
        return {"accepted": True, "command_id": cmd.command_id, "detail": {}}

    def execute(
        self,
        sid: str,
        payload: Dict[str, Any],
        legacy_event: Optional[str] = None,
        deprecated: bool = False,
        allow_preempt: bool = False,
    ):
        cmd = self._make_command(payload, legacy_event=legacy_event, deprecated=deprecated)
        self._submit_command(sid=sid, cmd=cmd, allow_preempt=allow_preempt)

    def execute_action(self, sid: str, payload: Dict[str, Any]) -> Dict[str, Any]:
        started_at_ms = self._now_ms()
        req = ExecutionRequest.from_payload(payload or {})
        self._refresh_action_registry()

        def _mk_result(
            success: bool,
            status: str,
            code: ErrorCode,
            detail: Optional[Dict[str, Any]] = None,
            telemetry: Optional[Dict[str, Any]] = None,
        ) -> Dict[str, Any]:
            ended_at_ms = self._now_ms()
            res = ExecutionResult(
                request_id=req.request_id,
                run_id=req.run_id,
                step_id=req.step_id,
                action_id=req.action_id,
                status=status,
                success=success,
                error_code=code.value,
                message=ERROR_MESSAGES.get(code.value, code.value),
                started_at_ms=started_at_ms,
                ended_at_ms=ended_at_ms,
                duration_ms=max(0, ended_at_ms - started_at_ms),
                sync_policy=req.sync_policy.value,
                telemetry=telemetry or {},
                postcheck={"passed": False, "details": {}},
                retryable=code in (ErrorCode.SDK_ERROR, ErrorCode.ACTION_TIMEOUT, ErrorCode.CAN_TIMEOUT),
                detail=detail or {},
            )
            return res.to_dict()

        required_fields = ("request_id", "run_id", "step_id", "action_id")
        missing = [field for field in required_fields if not getattr(req, field)]
        if missing:
            return _mk_result(False, "rejected", ErrorCode.INVALID_REQUEST, detail={"missing_fields": missing})

        action_cfg = self._action_registry.get(req.action_id)
        if action_cfg is None and not (isinstance(req.params, dict) and req.params):
            return _mk_result(
                False,
                "rejected",
                ErrorCode.UNSUPPORTED_ACTION,
                detail={"reason": "action not found in registry and params missing", "action_id": req.action_id},
            )

        cfg_arm = action_cfg.get("arm_plan", {}) if isinstance(action_cfg, dict) else {}
        cfg_hand = action_cfg.get("hand_plan", {}) if isinstance(action_cfg, dict) else {}
        req_arm = req.params.get("arm", {}) if isinstance(req.params, dict) else {}
        req_hand = req.params.get("hand", {}) if isinstance(req.params, dict) else {}

        arm_plan = self._merge_dict(cfg_arm if isinstance(cfg_arm, dict) else {}, req_arm if isinstance(req_arm, dict) else {})
        hand_plan = self._merge_dict(cfg_hand if isinstance(cfg_hand, dict) else {}, req_hand if isinstance(req_hand, dict) else {})
        if not isinstance(arm_plan, dict) or not arm_plan:
            return _mk_result(False, "rejected", ErrorCode.UNSUPPORTED_ACTION, detail={"reason": "missing arm plan"})

        if not self._a7_adapter.is_ready(req.arm_id):
            return _mk_result(False, "rejected", ErrorCode.ARM_NOT_CONNECTED, detail={"arm_id": req.arm_id})

        if req.sync_policy.value in ("hand_then_arm", "parallel"):
            if not self._hand_adapter.is_ready():
                return _mk_result(False, "rejected", ErrorCode.HAND_NOT_READY)
            hand_first = self._hand_adapter.execute_hand_plan(hand_plan=hand_plan, action_id=req.action_id)
            if not hand_first.get("ok"):
                return _mk_result(
                    False,
                    "rejected",
                    ErrorCode.SDK_ERROR,
                    detail={"hand_result": hand_first},
                    telemetry={"arm": self._a7_adapter.telemetry(req.arm_id), "hand": self._hand_adapter.telemetry()},
                )

        safe_abort_action = req.safe_abort_action
        if not safe_abort_action and isinstance(action_cfg, dict):
            safe_abort_action = action_cfg.get("safe_abort_action")

        timeout_ms = req.timeout_ms
        if isinstance(action_cfg, dict) and action_cfg.get("timeout_ms") is not None:
            timeout_ms = int(action_cfg.get("timeout_ms"))

        cmd_payload = self._a7_adapter.build_motion_payload(
            request_id=req.request_id,
            action_id=req.action_id,
            arm_id=req.arm_id,
            arm_plan=arm_plan,
        )
        cmd_payload["options"] = self._merge_dict(
            cmd_payload.get("options") or {},
            {
                "timeout_ms": timeout_ms,
                "safe_abort_action": safe_abort_action,
                "sync_policy": req.sync_policy.value,
                "post_arm_hand_plan": hand_plan if req.sync_policy.value == "arm_then_hand" else {},
                "execution_action_id": req.action_id,
            },
        )

        try:
            cmd = self._make_command(cmd_payload, legacy_event="execution:execute_action", deprecated=False)
        except Exception as e:
            return _mk_result(False, "rejected", ErrorCode.INVALID_REQUEST, detail={"exception": str(e)})

        submit = self._submit_command(sid=sid, cmd=cmd, allow_preempt=False)
        if not submit.get("accepted"):
            code = self._to_error_code(submit.get("error_code", ErrorCode.INTERNAL_ERROR))
            return _mk_result(False, "rejected", code, detail=submit.get("detail"))

        hand_result = {"ok": True, "status": "deferred"}
        if req.sync_policy.value == "arm_then_hand":
            if hand_plan:
                hand_result = {"ok": True, "status": "deferred_until_arm_done", "detail": {"plan": hand_plan}}
            else:
                hand_result = {"ok": True, "status": "skipped"}
        elif req.sync_policy.value == "parallel":
            hand_result = self._hand_adapter.execute_hand_plan(hand_plan=hand_plan, action_id=req.action_id)
        elif req.sync_policy.value == "hand_then_arm":
            hand_result = {"ok": True, "status": "already_executed_before_arm_submit"}

        return _mk_result(
            True,
            "accepted",
            ErrorCode.OK,
            detail={
                "command_id": submit.get("command_id"),
                "hand_result": hand_result,
                "registry_action": bool(action_cfg),
            },
            telemetry={"arm": self._a7_adapter.telemetry(req.arm_id), "hand": self._hand_adapter.telemetry()},
        )

    def _wait_if_paused(self, rt: _Runtime):
        while rt.pause_requested and not rt.cancel_requested:
            if not rt.paused_sent:
                self._emit_status(rt, MotionState.PAUSED)
                rt.paused_sent = True
            eventlet.sleep(0.05)
        if rt.paused_sent and not rt.cancel_requested:
            self._emit("motion:resumed", {
                "command_id": rt.command.command_id,
                "arm_id": rt.command.arm_id,
                "motion_type": rt.command.motion_type.value,
                "state": "resumed",
                "deprecated": rt.command.deprecated,
                "server_ts": self._now_ms(),
            }, sid=rt.sid)
            rt.paused_sent = False

    def _complete_and_cleanup(self, rt: _Runtime):
        rt.completed = True
        with self._lock:
            active = self._active_by_arm.get(rt.command.arm_id)
            if active and active.command.command_id == rt.command.command_id:
                self._active_by_arm.pop(rt.command.arm_id, None)

    def _run_command(self, rt: _Runtime):
        prev_speed: Optional[Dict[str, float]] = None
        try:
            prev_speed = self._apply_speed_constraints(rt)
            self._emit_status(rt, MotionState.PLANNING, progress=0.0)
            if rt.command.motion_type == MotionType.CARTESIAN_PTP:
                ok, detail = self._run_ptp(rt)
            elif rt.command.motion_type == MotionType.CARTESIAN_LINEAR:
                ok, detail = self._run_linear(rt)
            else:
                ok, detail = self._run_jog(rt)

            if rt.cancel_requested:
                self._emit_status(rt, MotionState.CANCELLED)
                return
            if not ok:
                code = self._to_error_code(detail.get("error_code", ErrorCode.INTERNAL_ERROR))
                if code in (ErrorCode.ACTION_TIMEOUT, ErrorCode.SDK_ERROR, ErrorCode.INTERNAL_ERROR):
                    self._trigger_safe_abort(rt, reason=code.value)
                self._emit_failed(rt, code, detail.get("detail"))
                return

            options = rt.command.options if isinstance(rt.command.options, dict) else {}
            sync_policy = options.get("sync_policy")
            post_arm_hand_plan = options.get("post_arm_hand_plan")
            action_id = str(options.get("execution_action_id") or rt.command.command_id)
            hand_result = {"ok": True, "status": "skipped"}
            if sync_policy == "arm_then_hand" and isinstance(post_arm_hand_plan, dict) and post_arm_hand_plan:
                if not self._hand_adapter.is_ready():
                    self._emit_failed(rt, ErrorCode.HAND_NOT_READY, {"phase": "hand_after_arm"})
                    return
                hand_result = self._hand_adapter.execute_hand_plan(post_arm_hand_plan, action_id=action_id)
                if not hand_result.get("ok"):
                    self._trigger_safe_abort(rt, reason=ErrorCode.SDK_ERROR.value)
                    self._emit_failed(rt, ErrorCode.SDK_ERROR, {"phase": "hand_after_arm", "hand_result": hand_result})
                    return

            duration_ms = int((time.time() - rt.created_at) * 1000)
            payload = {
                "duration_ms": duration_ms,
                "final_error_mm": detail.get("final_error_mm", 0.0),
                "hand_result": hand_result,
            }
            self._emit_status(rt, MotionState.COMPLETED, progress=1.0, extra=payload)

            if rt.command.legacy_event in ("cartesian_move_to", "smooth_cartesian_move"):
                self._emit("cartesian:moved", {
                    "arm_id": rt.command.arm_id,
                    "x": detail.get("target_pose", {}).get("x"),
                    "y": detail.get("target_pose", {}).get("y"),
                    "z": detail.get("target_pose", {}).get("z"),
                    "error_mm": detail.get("final_error_mm", 0.0),
                    "deprecated": True,
                }, sid=rt.sid)
            elif rt.command.legacy_event == "cartesian_jog":
                self._emit("cartesian:jogged", {
                    "arm_id": rt.command.arm_id,
                    "new_pos": detail.get("new_pos"),
                    "error_mm": detail.get("final_error_mm", 0.0),
                    "deprecated": True,
                }, sid=rt.sid)
        except Exception as e:
            self._emit_failed(rt, ErrorCode.INTERNAL_ERROR, {"exception": str(e)})
        finally:
            self._restore_speed_constraints(rt, prev_speed)
            self._complete_and_cleanup(rt)

    def _ik_or_error(self, arm_id: str, pose: Dict[str, float], current_joints: list):
        result = self.cartesian.compute_ik(
            arm_id,
            float(pose.get("x", 0.0)),
            float(pose.get("y", 0.0)),
            float(pose.get("z", 0.0)),
            current_joints,
            roll=pose.get("rx"),
            pitch=pose.get("ry"),
            yaw=pose.get("rz"),
        )
        if result is None:
            return None, ErrorCode.IK_UNREACHABLE
        if float(result.get("error_mm", 999.0)) > 5.0:
            return None, ErrorCode.IK_LOW_ACCURACY
        return result, None

    def _is_timed_out(self, rt: _Runtime) -> bool:
        if rt.timeout_ms <= 0:
            return False
        elapsed_ms = int((time.time() - rt.created_at) * 1000)
        return elapsed_ms >= rt.timeout_ms

    def _constraints_of(self, rt: _Runtime) -> Dict[str, float]:
        raw = rt.command.constraints if isinstance(rt.command.constraints, dict) else {}
        out: Dict[str, float] = {}
        for key in ("max_velocity", "max_acceleration", "max_angular_velocity", "max_angular_acceleration"):
            if key in raw:
                try:
                    out[key] = float(raw[key])
                except Exception:
                    continue
        return out

    def _apply_speed_constraints(self, rt: _Runtime) -> Optional[Dict[str, float]]:
        constraints = self._constraints_of(rt)
        if not constraints:
            return None
        max_v = constraints.get("max_velocity")
        max_a = constraints.get("max_acceleration")
        if max_v is None and max_a is None:
            return None
        try:
            prev = self.controller.get_speed_params()
        except Exception:
            prev = {"velocity": 0.5, "accel": 1.0, "decel": 1.0}

        velocity = float(max_v) if max_v is not None else float(prev.get("velocity", 0.5))
        accel = float(max_a) if max_a is not None else float(max(prev.get("accel", 1.0), prev.get("decel", 1.0)))
        self.controller.set_speed(velocity=velocity, accel=accel, decel=accel, arm_id=rt.command.arm_id)
        return {"velocity": float(prev.get("velocity", 0.5)), "accel": float(prev.get("accel", 1.0)), "decel": float(prev.get("decel", 1.0))}

    def _restore_speed_constraints(self, rt: _Runtime, prev: Optional[Dict[str, float]]):
        if not prev:
            return
        try:
            self.controller.set_speed(
                velocity=float(prev.get("velocity", 0.5)),
                accel=float(prev.get("accel", 1.0)),
                decel=float(prev.get("decel", 1.0)),
                arm_id=rt.command.arm_id,
            )
        except Exception:
            pass

    def _trigger_safe_abort(self, rt: _Runtime, reason: str):
        if not rt.safe_abort_action:
            return
        try:
            action = str(rt.safe_abort_action).strip().lower()
            if action in ("go_to_zero", "return_zero"):
                self.controller.go_to_zero(rt.command.arm_id)
                fallback = False
            elif action in ("retreat_to_safe_pose", "safe_pose"):
                self.controller.go_to_zero(rt.command.arm_id)
                fallback = False
            elif action in ("disable_arm", "deactivate"):
                self.controller.disable_arm(rt.command.arm_id)
                fallback = False
            elif action in ("emergency_stop", "estop"):
                self.controller.emergency_stop()
                fallback = False
            else:
                self.controller.go_to_zero(rt.command.arm_id)
                fallback = True
            self._emit(
                "motion:safe_abort",
                {
                    "command_id": rt.command.command_id,
                    "arm_id": rt.command.arm_id,
                    "action": rt.safe_abort_action,
                    "reason": reason,
                    "fallback": fallback,
                    "server_ts": self._now_ms(),
                },
                sid=rt.sid,
            )
        except Exception as e:
            self._emit(
                "motion:safe_abort",
                {
                    "command_id": rt.command.command_id,
                    "arm_id": rt.command.arm_id,
                    "action": rt.safe_abort_action,
                    "reason": reason,
                    "error": str(e),
                    "server_ts": self._now_ms(),
                },
                sid=rt.sid,
            )

    def _run_ptp(self, rt: _Runtime):
        if self._is_timed_out(rt):
            return False, {"error_code": ErrorCode.ACTION_TIMEOUT, "detail": {"timeout_ms": rt.timeout_ms}}
        arm_ctrl = self.controller.arms.get(rt.command.arm_id)
        current_joints = self.get_joints(arm_ctrl, rt.command.arm_id)
        pose = rt.command.target.get("pose") or {}

        ik_result, err = self._ik_or_error(rt.command.arm_id, pose, current_joints)
        if err:
            return False, {"error_code": err, "detail": {"pose": pose}}

        self._emit_status(rt, MotionState.RUNNING, progress=0.2)
        self._wait_if_paused(rt)
        if self._is_timed_out(rt):
            return False, {"error_code": ErrorCode.ACTION_TIMEOUT, "detail": {"timeout_ms": rt.timeout_ms}}
        if rt.cancel_requested:
            return False, {"error_code": ErrorCode.COMMAND_PREEMPTED}

        ok = self.controller.set_joint_offsets(rt.command.arm_id, ik_result["joints"])
        if not ok:
            return False, {"error_code": ErrorCode.JOINT_LIMIT_REJECTED}

        return True, {"final_error_mm": float(ik_result["error_mm"]), "target_pose": pose}

    def _run_linear_sdk(self, rt: _Runtime, target: Dict[str, float]):
        runtime = self.controller.arms.get(rt.command.arm_id)
        if runtime is None:
            return False, {"error_code": ErrorCode.ARM_NOT_CONNECTED}
        arm = getattr(runtime, "arm", None)
        if arm is None or not hasattr(arm, "move_l"):
            return False, {"error_code": ErrorCode.SDK_ERROR, "detail": {"reason": "move_l unavailable"}}

        constraints = self._constraints_of(rt)
        max_v = float(constraints.get("max_velocity", 0.05))
        max_av = float(constraints.get("max_angular_velocity", 0.3))
        acc = float(constraints.get("max_acceleration", 0.1))
        ang_acc = float(constraints.get("max_angular_acceleration", 0.1))
        try:
            from linkerbot import Pose  # lazy import to avoid hard dependency at module import

            pose = Pose(
                x=float(target["x"]),
                y=float(target["y"]),
                z=float(target["z"]),
                rx=float(target["rx"]),
                ry=float(target["ry"]),
                rz=float(target["rz"]),
            )
            self._emit_status(rt, MotionState.RUNNING, progress=0.1)
            arm.move_l(
                pose,
                max_velocity=max_v,
                max_angular_velocity=max_av,
                acceleration=acc,
                angular_acceleration=ang_acc,
            )
            arm_ctrl = self.controller.arms.get(rt.command.arm_id)
            current_joints = self.get_joints(arm_ctrl, rt.command.arm_id)
            final_fk = self.cartesian.compute_fk(rt.command.arm_id, current_joints)
            final_error_mm = math.sqrt(
                (float(final_fk["x"]) - float(target["x"])) ** 2
                + (float(final_fk["y"]) - float(target["y"])) ** 2
                + (float(final_fk["z"]) - float(target["z"])) ** 2
            ) * 1000.0
            return True, {"final_error_mm": float(final_error_mm), "target_pose": target, "linear_mode": "sdk_move_l"}
        except Exception as e:
            return False, {"error_code": ErrorCode.SDK_ERROR, "detail": {"phase": "sdk_move_l", "exception": str(e)}}

    def _resolve_linear_timing(self, rt: _Runtime, start: Dict[str, float], target: Dict[str, float]) -> tuple[int, float]:
        constraints = self._constraints_of(rt)
        max_v = constraints.get("max_velocity")
        max_a = constraints.get("max_acceleration")
        max_av = constraints.get("max_angular_velocity")
        max_aa = constraints.get("max_angular_acceleration")

        pos_dist = math.sqrt(
            (target["x"] - start["x"]) ** 2 + (target["y"] - start["y"]) ** 2 + (target["z"] - start["z"]) ** 2
        )
        ang_dist = math.sqrt(
            (target["rx"] - start["rx"]) ** 2 + (target["ry"] - start["ry"]) ** 2 + (target["rz"] - start["rz"]) ** 2
        )
        total_time = 0.4  # default ~= 20 * 20ms
        if max_v and pos_dist > 0:
            total_time = max(total_time, pos_dist / max(max_v, 1e-6))
        if max_av and ang_dist > 0:
            total_time = max(total_time, ang_dist / max(max_av, 1e-6))
        if max_a and pos_dist > 0:
            total_time = max(total_time, 2.0 * math.sqrt(pos_dist / max(max_a, 1e-6)))
        if max_aa and ang_dist > 0:
            total_time = max(total_time, 2.0 * math.sqrt(ang_dist / max(max_aa, 1e-6)))

        steps = int(max(5, min(200, math.ceil(total_time / 0.02))))
        step_sleep = max(0.005, total_time / float(steps))
        return steps, step_sleep

    def _run_linear_segmented(self, rt: _Runtime, start: Dict[str, float], target: Dict[str, float], current_joints: list):
        steps, step_sleep = self._resolve_linear_timing(rt, start, target)
        joints_seed = current_joints
        final_error = 0.0
        self._emit_status(rt, MotionState.RUNNING, progress=0.05)

        for i in range(steps):
            if self._is_timed_out(rt):
                return False, {"error_code": ErrorCode.ACTION_TIMEOUT, "detail": {"timeout_ms": rt.timeout_ms, "step": i + 1}}
            if rt.cancel_requested:
                break
            self._wait_if_paused(rt)
            if rt.cancel_requested:
                break

            t = float(i + 1) / float(steps)
            interp = {k: start[k] + (target[k] - start[k]) * t for k in start.keys()}
            ik_result, err = self._ik_or_error(rt.command.arm_id, interp, joints_seed)
            if err:
                return False, {"error_code": err, "detail": {"step": i + 1, "target_pose": interp}}

            ok = self.controller.set_joint_offsets(rt.command.arm_id, ik_result["joints"])
            if not ok:
                return False, {"error_code": ErrorCode.JOINT_LIMIT_REJECTED, "detail": {"step": i + 1}}

            joints_seed = ik_result["joints"]
            final_error = float(ik_result["error_mm"])
            progress = 0.05 + 0.9 * t
            self._emit_status(
                rt,
                MotionState.RUNNING,
                progress=progress,
                extra={
                    "pose_error_mm": final_error,
                    "joint_error_rad": 0.0,
                    "linear_mode": "ik_segmented_fallback",
                    "current_pose": {
                        "x": interp["x"],
                        "y": interp["y"],
                        "z": interp["z"],
                        "rx": interp["rx"],
                        "ry": interp["ry"],
                        "rz": interp["rz"],
                    },
                },
            )
            eventlet.sleep(step_sleep)
        return True, {"final_error_mm": final_error, "target_pose": target, "linear_mode": "ik_segmented_fallback"}

    def _run_linear(self, rt: _Runtime):
        if self._is_timed_out(rt):
            return False, {"error_code": ErrorCode.ACTION_TIMEOUT, "detail": {"timeout_ms": rt.timeout_ms}}
        arm_ctrl = self.controller.arms.get(rt.command.arm_id)
        current_joints = self.get_joints(arm_ctrl, rt.command.arm_id)
        current_fk = self.cartesian.compute_fk(rt.command.arm_id, current_joints)
        pose = rt.command.target.get("pose") or {}

        start = {
            "x": float(current_fk["x"]),
            "y": float(current_fk["y"]),
            "z": float(current_fk["z"]),
            "rx": float(current_fk["roll"]),
            "ry": float(current_fk["pitch"]),
            "rz": float(current_fk["yaw"]),
        }
        target = {
            "x": float(pose.get("x", start["x"])),
            "y": float(pose.get("y", start["y"])),
            "z": float(pose.get("z", start["z"])),
            "rx": float(pose.get("rx", start["rx"])),
            "ry": float(pose.get("ry", start["ry"])),
            "rz": float(pose.get("rz", start["rz"])),
        }

        # 优先走 SDK 原生 move_l；失败时回退到分段 IK 方案，兼容现有暂停/进度机制。
        sdk_ok, sdk_detail = self._run_linear_sdk(rt, target)
        if sdk_ok:
            return True, sdk_detail
        return self._run_linear_segmented(rt, start, target, current_joints)

    def _run_jog(self, rt: _Runtime):
        if self._is_timed_out(rt):
            return False, {"error_code": ErrorCode.ACTION_TIMEOUT, "detail": {"timeout_ms": rt.timeout_ms}}
        arm_ctrl = self.controller.arms.get(rt.command.arm_id)
        current_joints = self.get_joints(arm_ctrl, rt.command.arm_id)
        delta = rt.command.target.get("delta") or {}

        dx = float(delta.get("dx", 0.0))
        dy = float(delta.get("dy", 0.0))
        dz = float(delta.get("dz", 0.0))
        drx = float(delta.get("drx", 0.0))
        dry = float(delta.get("dry", 0.0))
        drz = float(delta.get("drz", 0.0))

        current_fk = self.cartesian.compute_fk(rt.command.arm_id, current_joints)
        target_pose = {
            "x": float(current_fk["x"]) + dx,
            "y": float(current_fk["y"]) + dy,
            "z": float(current_fk["z"]) + dz,
            "rx": float(current_fk["roll"]) + drx,
            "ry": float(current_fk["pitch"]) + dry,
            "rz": float(current_fk["yaw"]) + drz,
        }

        ik_result, err = self._ik_or_error(rt.command.arm_id, target_pose, current_joints)
        if err:
            return False, {"error_code": err, "detail": {"target_pose": target_pose}}

        self._emit_status(rt, MotionState.RUNNING, progress=0.3)
        self._wait_if_paused(rt)
        if self._is_timed_out(rt):
            return False, {"error_code": ErrorCode.ACTION_TIMEOUT, "detail": {"timeout_ms": rt.timeout_ms}}
        if rt.cancel_requested:
            return False, {"error_code": ErrorCode.COMMAND_PREEMPTED}

        ok = self.controller.set_joint_offsets(rt.command.arm_id, ik_result["joints"])
        if not ok:
            return False, {"error_code": ErrorCode.JOINT_LIMIT_REJECTED}

        return True, {
            "final_error_mm": float(ik_result["error_mm"]),
            "new_pos": {"x": target_pose["x"], "y": target_pose["y"], "z": target_pose["z"]},
            "target_pose": target_pose,
        }

    def _resolve_runtime(self, command_id: Optional[str], arm_id: Optional[str]) -> Optional[_Runtime]:
        with self._lock:
            if arm_id in ("left", "right"):
                rt = self._active_by_arm.get(arm_id)
                if not rt:
                    return None
                if command_id and rt.command.command_id != command_id:
                    return None
                return rt
            if command_id:
                for rt in self._active_by_arm.values():
                    if rt.command.command_id == command_id:
                        return rt
            return None

    def cancel(self, sid: str, payload: Dict[str, Any]):
        rt = self._resolve_runtime(payload.get("command_id"), payload.get("arm_id"))
        if not rt:
            cmd = self._make_command({"command_id": payload.get("command_id") or str(uuid.uuid4()), "arm_id": payload.get("arm_id", "left")})
            self._emit_rejected(sid, cmd, ErrorCode.COMMAND_NOT_FOUND)
            return
        rt.cancel_requested = True
        self._emit("motion:status", {
            "command_id": rt.command.command_id,
            "arm_id": rt.command.arm_id,
            "motion_type": rt.command.motion_type.value,
            "state": "cancelling",
            "server_ts": self._now_ms(),
        }, sid=sid)

    def pause(self, sid: str, payload: Dict[str, Any]):
        rt = self._resolve_runtime(payload.get("command_id"), payload.get("arm_id"))
        if not rt:
            cmd = self._make_command({"command_id": payload.get("command_id") or str(uuid.uuid4()), "arm_id": payload.get("arm_id", "left")})
            self._emit_rejected(sid, cmd, ErrorCode.COMMAND_NOT_FOUND)
            return
        rt.pause_requested = True

    def resume(self, sid: str, payload: Dict[str, Any]):
        rt = self._resolve_runtime(payload.get("command_id"), payload.get("arm_id"))
        if not rt:
            cmd = self._make_command({"command_id": payload.get("command_id") or str(uuid.uuid4()), "arm_id": payload.get("arm_id", "left")})
            self._emit_rejected(sid, cmd, ErrorCode.COMMAND_NOT_FOUND)
            return
        rt.pause_requested = False

    def query(self, sid: str, payload: Dict[str, Any]):
        rt = self._resolve_runtime(payload.get("command_id"), payload.get("arm_id"))
        if not rt:
            self._emit("motion:status", {"state": "idle", "server_ts": self._now_ms()}, sid=sid)
            return
        self._emit("motion:status", {
            "command_id": rt.command.command_id,
            "arm_id": rt.command.arm_id,
            "motion_type": rt.command.motion_type.value,
            "state": rt.state,
            "server_ts": self._now_ms(),
        }, sid=sid)
