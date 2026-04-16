"""
统一运动命令类型与错误码定义。
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Dict, List, Optional


class MotionType(str, Enum):
    CARTESIAN_PTP = "CARTESIAN_PTP"
    CARTESIAN_LINEAR = "CARTESIAN_LINEAR"
    CARTESIAN_JOG = "CARTESIAN_JOG"


class Frame(str, Enum):
    BASE = "BASE"
    TOOL = "TOOL"
    USER = "USER"


class MotionState(str, Enum):
    ACCEPTED = "accepted"
    PLANNING = "planning"
    RUNNING = "running"
    COMPLETED = "completed"
    FAILED = "failed"
    CANCELLED = "cancelled"
    PAUSED = "paused"


class ErrorCode(str, Enum):
    OK = "OK"
    INVALID_REQUEST = "INVALID_REQUEST"
    UNSUPPORTED_ACTION = "UNSUPPORTED_ACTION"
    ARM_NOT_CONNECTED = "ARM_NOT_CONNECTED"
    HAND_NOT_READY = "HAND_NOT_READY"
    SDK_ERROR = "SDK_ERROR"
    ACTION_TIMEOUT = "ACTION_TIMEOUT"
    IK_UNREACHABLE = "IK_UNREACHABLE"
    IK_LOW_ACCURACY = "IK_LOW_ACCURACY"
    SINGULARITY_NEAR = "SINGULARITY_NEAR"
    JOINT_LIMIT_REJECTED = "JOINT_LIMIT_REJECTED"
    CONSTRAINT_VIOLATION = "CONSTRAINT_VIOLATION"
    COMMAND_PREEMPTED = "COMMAND_PREEMPTED"
    CAN_TIMEOUT = "CAN_TIMEOUT"
    INTERNAL_ERROR = "INTERNAL_ERROR"
    COMMAND_IN_PROGRESS = "COMMAND_IN_PROGRESS"
    COMMAND_NOT_FOUND = "COMMAND_NOT_FOUND"
    POSTCHECK_FAILED = "POSTCHECK_FAILED"
    SAFETY_ABORTED = "SAFETY_ABORTED"


ERROR_MESSAGES = {
    ErrorCode.OK.value: "执行成功",
    ErrorCode.INVALID_REQUEST.value: "请求参数不完整或格式错误",
    ErrorCode.UNSUPPORTED_ACTION.value: "不支持的动作类型或动作未注册",
    ErrorCode.ARM_NOT_CONNECTED.value: "机械臂未连接，请先连接并初始化",
    ErrorCode.HAND_NOT_READY.value: "灵巧手未就绪",
    ErrorCode.SDK_ERROR.value: "SDK 调用失败",
    ErrorCode.ACTION_TIMEOUT.value: "动作执行超时",
    ErrorCode.IK_UNREACHABLE.value: "目标位姿不可达，请减小位移或调整姿态",
    ErrorCode.IK_LOW_ACCURACY.value: "IK 精度不足，请降低速度并接近可操作区",
    ErrorCode.SINGULARITY_NEAR.value: "接近奇异位形，请调整姿态后再试",
    ErrorCode.JOINT_LIMIT_REJECTED.value: "关节超限，命令已拒绝",
    ErrorCode.CONSTRAINT_VIOLATION.value: "速度/加速度参数超范围",
    ErrorCode.COMMAND_PREEMPTED.value: "命令被新命令抢占",
    ErrorCode.CAN_TIMEOUT.value: "通讯超时，请检查总线与供电",
    ErrorCode.INTERNAL_ERROR.value: "内部错误，请查看系统日志",
    ErrorCode.COMMAND_IN_PROGRESS.value: "执行中禁止重复提交，请先取消/暂停或等待完成",
    ErrorCode.COMMAND_NOT_FOUND.value: "未找到指定命令",
    ErrorCode.POSTCHECK_FAILED.value: "后置条件验证失败",
    ErrorCode.SAFETY_ABORTED.value: "触发安全中断",
}


@dataclass
class MotionCommand:
    command_id: str
    arm_id: str
    motion_type: MotionType
    frame: Frame = Frame.BASE
    target: Dict[str, Any] = field(default_factory=dict)
    constraints: Dict[str, float] = field(default_factory=dict)
    options: Dict[str, Any] = field(default_factory=dict)
    legacy_event: Optional[str] = None
    deprecated: bool = False


class SyncPolicy(str, Enum):
    PARALLEL = "parallel"
    ARM_THEN_HAND = "arm_then_hand"
    HAND_THEN_ARM = "hand_then_arm"


@dataclass
class ExecutionRequest:
    request_id: str
    run_id: str
    step_id: str
    action_id: str
    source: str = "orchestrator"
    priority: int = 5
    timeout_ms: int = 8000
    sync_policy: SyncPolicy = SyncPolicy.ARM_THEN_HAND
    target_scope: str = "both"
    arm_id: str = "left"
    params: Dict[str, Any] = field(default_factory=dict)
    expected_postconditions: List[Dict[str, Any]] = field(default_factory=list)
    safe_abort_action: Optional[str] = None
    metadata: Dict[str, Any] = field(default_factory=dict)

    @staticmethod
    def from_payload(payload: Dict[str, Any]) -> "ExecutionRequest":
        sync_policy_raw = payload.get("sync_policy", SyncPolicy.ARM_THEN_HAND.value)
        try:
            sync_policy = SyncPolicy(sync_policy_raw)
        except Exception:
            sync_policy = SyncPolicy.ARM_THEN_HAND
        return ExecutionRequest(
            request_id=str(payload.get("request_id") or ""),
            run_id=str(payload.get("run_id") or ""),
            step_id=str(payload.get("step_id") or ""),
            action_id=str(payload.get("action_id") or ""),
            source=str(payload.get("source") or "orchestrator"),
            priority=int(payload.get("priority", 5)),
            timeout_ms=int(payload.get("timeout_ms", 8000)),
            sync_policy=sync_policy,
            target_scope=str(payload.get("target_scope") or "both"),
            arm_id=str(payload.get("arm_id") or "left"),
            params=payload.get("params") or {},
            expected_postconditions=payload.get("expected_postconditions") or [],
            safe_abort_action=payload.get("safe_abort_action"),
            metadata=payload.get("metadata") or {},
        )


@dataclass
class ExecutionResult:
    request_id: str
    run_id: str
    step_id: str
    action_id: str
    status: str
    success: bool
    error_code: str
    message: str
    started_at_ms: int
    ended_at_ms: int
    duration_ms: int
    sync_policy: str
    telemetry: Dict[str, Any] = field(default_factory=dict)
    postcheck: Dict[str, Any] = field(default_factory=dict)
    retryable: bool = False
    detail: Dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> Dict[str, Any]:
        return {
            "request_id": self.request_id,
            "run_id": self.run_id,
            "step_id": self.step_id,
            "action_id": self.action_id,
            "status": self.status,
            "success": self.success,
            "error_code": self.error_code,
            "message": self.message,
            "started_at_ms": self.started_at_ms,
            "ended_at_ms": self.ended_at_ms,
            "duration_ms": self.duration_ms,
            "sync_policy": self.sync_policy,
            "telemetry": self.telemetry,
            "postcheck": self.postcheck,
            "retryable": self.retryable,
            "detail": self.detail,
        }

