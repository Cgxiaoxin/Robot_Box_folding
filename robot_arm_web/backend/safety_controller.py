"""
安全控制器 - 提供机械臂安全保护功能
"""
import time
import threading
from typing import Dict, List, Optional, Callable, Any
from dataclasses import dataclass, field
from enum import Enum
import json
from pathlib import Path


class SafetyLevel(Enum):
    DISABLED = 0
    WARNING = 1
    STOP = 2


@dataclass
class JointLimits:
    min_position: float = -3.14
    max_position: float = 3.14
    max_velocity: float = 2.0


@dataclass
class SafetyEvent:
    timestamp: float
    event_type: str
    severity: str
    message: str
    details: Dict[str, Any] = field(default_factory=dict)


class SafetyController:
    """安全控制器"""
    
    def __init__(self, log_dir: Optional[Path] = None):
        self.joint_limits: Dict[int, JointLimits] = {i: JointLimits() for i in range(51, 58)}
        self.limits_enabled = True
        self.collision_detection_enabled = True
        self.collision_threshold_torque = 5.0
        self.velocity_monitoring_enabled = True
        self.max_safe_velocity = 2.0
        
        self._last_positions: Dict[int, float] = {}
        self._last_time: float = 0
        self._velocities: Dict[int, float] = {}
        
        self._on_safety_event: Optional[Callable] = None
        self._event_log: List[SafetyEvent] = []
        self._log_dir = log_dir
        self._log_lock = threading.Lock()
        self._emergency_stop_callback: Optional[Callable] = None
    
    def set_callbacks(self, on_safety_event: Optional[Callable] = None, emergency_stop: Optional[Callable] = None):
        self._on_safety_event = on_safety_event
        self._emergency_stop_callback = emergency_stop
    
    def set_joint_limits(self, motor_id: int, limits: JointLimits):
        if motor_id in self.joint_limits:
            self.joint_limits[motor_id] = limits
    
    def set_all_limits(self, min_pos: float, max_pos: float, max_vel: float = 2.0):
        for mid in self.joint_limits:
            self.joint_limits[mid] = JointLimits(min_position=min_pos, max_position=max_pos, max_velocity=max_vel)
    
    def enable_limits(self, enabled: bool):
        self.limits_enabled = enabled
    
    def check_position_limits(self, positions: Dict[int, float]) -> Dict[int, str]:
        violations = {}
        if not self.limits_enabled:
            return violations
        
        for mid, pos in positions.items():
            if mid not in self.joint_limits:
                continue
            limits = self.joint_limits[mid]
            if pos < limits.min_position:
                violations[mid] = f"below_min ({pos:.4f} < {limits.min_position})"
            elif pos > limits.max_position:
                violations[mid] = f"above_max ({pos:.4f} > {limits.max_position})"
        return violations
    
    def update_state(self, positions: Dict[int, float]):
        current_time = time.time()
        if self._last_time > 0:
            dt = current_time - self._last_time
            if dt > 0:
                for mid, pos in positions.items():
                    if mid in self._last_positions:
                        self._velocities[mid] = (pos - self._last_positions[mid]) / dt
        self._last_positions = positions.copy()
        self._last_time = current_time
    
    def check_safety(self, positions: Dict[int, float], torques: Optional[Dict[int, float]] = None) -> List[SafetyEvent]:
        events = []
        self.update_state(positions)
        
        pos_violations = self.check_position_limits(positions)
        for mid, msg in pos_violations.items():
            event = SafetyEvent(
                timestamp=time.time(),
                event_type="position_limit",
                severity="error",
                message=f"电机 {mid} 位置超限: {msg}",
                details={"motor_id": mid, "position": positions.get(mid)}
            )
            events.append(event)
            self._log_event(event)
        
        return events
    
    def _log_event(self, event: SafetyEvent):
        with self._log_lock:
            self._event_log.append(event)
            if len(self._event_log) > 1000:
                self._event_log = self._event_log[-500:]
    
    def get_event_log(self, limit: int = 100) -> List[Dict]:
        with self._log_lock:
            logs = self._event_log[-limit:]
            return [{"timestamp": e.timestamp, "type": e.event_type, "severity": e.severity, "message": e.message} for e in logs]


class AuditLogger:
    """操作审计日志"""
    
    def __init__(self, log_dir: Path):
        self.log_dir = log_dir
        self.log_dir.mkdir(parents=True, exist_ok=True)
        self._current_session = time.strftime("%Y%m%d_%H%M%S")
        self._log_file = self.log_dir / f"audit_{self._current_session}.jsonl"
        self._operations: List[Dict] = []
        self._lock = threading.Lock()
    
    def log_operation(self, operation: str, user: str = "system", details: Optional[Dict] = None):
        entry = {
            "timestamp": time.time(),
            "datetime": time.strftime("%Y-%m-%d %H:%M:%S"),
            "session": self._current_session,
            "user": user,
            "operation": operation,
            "details": details or {}
        }
        with self._lock:
            self._operations.append(entry)
            try:
                with open(self._log_file, 'a') as f:
                    f.write(json.dumps(entry, ensure_ascii=False) + "\n")
            except Exception as e:
                print(f"[AuditLogger] 写入失败: {e}")
    
    def get_operations(self, start_time: Optional[float] = None, end_time: Optional[float] = None, operation: Optional[str] = None) -> List[Dict]:
        with self._lock:
            results = self._operations.copy()
        if start_time:
            results = [r for r in results if r["timestamp"] >= start_time]
        if end_time:
            results = [r for r in results if r["timestamp"] <= end_time]
        if operation:
            results = [r for r in results if r["operation"] == operation]
        return results
