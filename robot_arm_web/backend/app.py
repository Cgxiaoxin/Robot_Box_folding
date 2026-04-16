"""
机械臂示教控制系统 - Web服务主程序

Flask + Socket.IO 实现实时控制
"""
import eventlet
eventlet.monkey_patch()  # 必须在最前面: 替换 time.sleep/socket/threading 为协作式版本

import json
import math
import os
import sys
import time
import threading
from pathlib import Path
from typing import Optional

import linkerbot.arm.kinetix as _linkerbot_kinetix

from flask import Flask, send_from_directory, jsonify, request
import socketio

# 兼容脚本直接运行与模块运行
if __package__ is None or __package__ == "":
    project_root = Path(__file__).resolve().parents[2]
    sys.path.insert(0, str(project_root))
    from robot_arm_web.backend.config import (
        HOST,
        PORT,
        DEBUG,
        CARTESIAN_LIVE_INTERVAL,
        STATE_UPDATE_INTERVAL,
        TRAJECTORIES_DIR,
        LOG_DIR,
        ZERO_OFFSET_FILE,
        LEFT_MOTOR_IDS,
        RIGHT_MOTOR_IDS,
        HAND_ENABLE,
        HAND_TYPE,
        HAND_JOINT,
        HAND_CAN,
        HAND_MODBUS,
    )
    from robot_arm_web.backend.group_controller import get_controller, ControlTarget, PlaybackState
    from robot_arm_web.backend.safety_controller import AuditLogger
    from robot_arm_web.backend.cartesian_controller import get_cartesian_controller
    from robot_arm_web.backend.motion_service import MotionService
    from robot_arm_web.backend.drag_teach_service import (
        get_drag_teach_service,
        flatten_record_point_to_positions,
    )
else:
    from .config import (
        HOST,
        PORT,
        DEBUG,
        CARTESIAN_LIVE_INTERVAL,
        STATE_UPDATE_INTERVAL,
        TRAJECTORIES_DIR,
        LOG_DIR,
        ZERO_OFFSET_FILE,
        LEFT_MOTOR_IDS,
        RIGHT_MOTOR_IDS,
        HAND_ENABLE,
        HAND_TYPE,
        HAND_JOINT,
        HAND_CAN,
        HAND_MODBUS,
    )
    from .group_controller import get_controller, ControlTarget, PlaybackState
    from .safety_controller import AuditLogger
    from .cartesian_controller import get_cartesian_controller
    from .motion_service import MotionService
    from .drag_teach_service import get_drag_teach_service, flatten_record_point_to_positions

# 创建Flask应用
BASE_DIR = Path(__file__).resolve().parent
FRONTEND_DIR = BASE_DIR.parent / "frontend"
# linkerbot-py 随包 URDF（替代已移除的 lansi_arm_sdk/urdf）
URDF_SERVE_DIR = Path(_linkerbot_kinetix.__file__).resolve().parent / "urdf"
app = Flask(__name__, static_folder=str(FRONTEND_DIR), static_url_path='')

# 创建Socket.IO服务器
sio = socketio.Server(
    async_mode='eventlet',
    cors_allowed_origins='*',
    logger=DEBUG,
    engineio_logger=DEBUG,
    ping_timeout=60,
    ping_interval=25,
)
flask_app = socketio.WSGIApp(sio, app)

# 获取控制器实例
controller = get_controller()

# 笛卡尔控制器
cartesian = get_cartesian_controller()

# 统一运动服务
motion_service = MotionService(
    controller=controller,
    cartesian=cartesian,
    emit_fn=sio.emit,
    get_joints_fn=lambda arm_ctrl, arm_id: _get_urdf_joints(arm_ctrl, arm_id),
    hand_config={
        "enabled": HAND_ENABLE,
        "hand_type": HAND_TYPE,
        "hand_joint": HAND_JOINT,
        "can": HAND_CAN,
        "modbus": HAND_MODBUS,
    },
)
drag_teach = get_drag_teach_service(
    controller=controller,
    motion_service=motion_service,
)

# 审计日志
audit_logger = AuditLogger(LOG_DIR)

# 状态更新定时器
_state_update_running = False


# ==================== 静态文件路由 ====================

@app.route('/')
def index():
    """主页"""
    return send_from_directory(str(FRONTEND_DIR), 'index.html')


@app.route('/urdf/<path:path>')
def urdf_files(path):
    """Serve URDF and mesh files（来自 linkerbot-py 包内资源）"""
    return send_from_directory(str(URDF_SERVE_DIR), path)


@app.route('/<path:path>')
def static_files(path):
    """静态文件"""
    return send_from_directory(str(FRONTEND_DIR), path)


@app.after_request
def add_no_cache_headers(response):
    """开发环境禁用缓存，确保浏览器加载最新代码"""
    response.headers['Cache-Control'] = 'no-cache, no-store, must-revalidate'
    response.headers['Pragma'] = 'no-cache'
    response.headers['Expires'] = '0'
    return response


# ==================== REST API ====================

@app.route('/api/health')
def health():
    """健康检查"""
    return jsonify({"ok": True, "timestamp": time.time()})


@app.route('/api/trajectories')
def list_trajectories():
    """列出所有轨迹文件"""
    files = controller.list_trajectories()
    return jsonify({"trajectories": files})


@app.route('/api/state')
def get_state():
    """获取当前状态（HTTP方式，用于调试）"""
    return jsonify(controller.get_state())


@app.route('/api/execution/execute', methods=['POST'])
def execution_execute():
    """统一执行层入口（HTTP）"""
    payload = request.get_json(silent=True) or {}
    result = motion_service.execute_action(sid="http", payload=payload)
    status = 200 if result.get("success") else 400
    return jsonify(result), status


@app.route('/api/zero_offsets')
def get_zero_offsets_file():
    """从本地文件读取已保存的零点偏移（不依赖臂连接）"""
    try:
        if not ZERO_OFFSET_FILE.exists():
            return jsonify({"left": {}, "right": {}})
        with open(ZERO_OFFSET_FILE, "r", encoding="utf-8") as f:
            data = json.load(f)
        if not isinstance(data, dict):
            return jsonify({"left": {}, "right": {}})
        left = {str(mid): data.get(str(mid), 0.0) for mid in LEFT_MOTOR_IDS}
        right = {str(mid): data.get(str(mid), 0.0) for mid in RIGHT_MOTOR_IDS}
        return jsonify({"left": left, "right": right})
    except Exception as e:
        return jsonify({"left": {}, "right": {}, "error": str(e)}), 500


# ==================== Socket.IO 事件处理 ====================

@sio.event
def connect(sid, environ):
    """客户端连接"""
    print(f"[Socket.IO] 客户端连接: {sid}")
    # 发送当前状态
    sio.emit('state:update', _compose_state(), to=sid)
    sio.emit('cartesian:live', _compose_cartesian_live(), to=sid)


@sio.event
def disconnect(sid):
    """客户端断开"""
    print(f"[Socket.IO] 客户端断开: {sid}")


# ---------- 连接控制 ----------

@sio.event
def arm_connect(sid, data):
    """
    连接机械臂
    
    data: {
        "arm_id": "left" | "right" | "both",
        "left_channel": "can0",  # 可选
        "right_channel": "can1"  # 可选
    }
    """
    arm_id = data.get("arm_id", "both")
    left_ch = data.get("left_channel")
    right_ch = data.get("right_channel")
    
    try:
        if arm_id == "both":
            results = controller.connect_all(left_ch, right_ch)
            success = all(results.values())
        else:
            success = controller.connect(arm_id, left_ch if arm_id == "left" else right_ch)
            results = {arm_id: success}

        audit_logger.log_operation(
            "arm_connect",
            details={"sid": sid, "arm_id": arm_id, "left_channel": left_ch, "right_channel": right_ch, "results": results},
        )
        sio.emit('arm:connected', {"success": success, "results": results}, to=sid)
        _broadcast_state()
    except Exception as e:
        audit_logger.log_operation(
            "arm_connect_error",
            details={"sid": sid, "arm_id": arm_id, "error": str(e)},
        )
        sio.emit('error', {"message": f"连接失败: {str(e)}"}, to=sid)


@sio.event
def arm_disconnect(sid, data):
    """
    断开机械臂
    
    data: {"arm_id": "left" | "right" | "both"}
    """
    arm_id = data.get("arm_id", "both")
    
    try:
        if arm_id == "both":
            controller.disconnect_all()
        else:
            controller.disconnect(arm_id)

        audit_logger.log_operation(
            "arm_disconnect",
            details={"sid": sid, "arm_id": arm_id},
        )
        sio.emit('arm:disconnected', {"arm_id": arm_id}, to=sid)
        _broadcast_state()
    except Exception as e:
        audit_logger.log_operation(
            "arm_disconnect_error",
            details={"sid": sid, "arm_id": arm_id, "error": str(e)},
        )
        sio.emit('error', {"message": f"断开失败: {str(e)}"}, to=sid)


# ---------- 初始化控制 ----------

@sio.event
def arm_init(sid, data):
    """
    初始化并使能机械臂
    
    data: {"arm_id": "left" | "right" | null, "mode": 5}
    """
    arm_id = data.get("arm_id")
    mode = data.get("mode", 5)  # A7 SDK 轨迹位置模式（PP）
    
    try:
        if arm_id:
            success = controller.init_arm(arm_id, mode)
            results = {arm_id: success}
        else:
            results = controller.init_target(mode)

        audit_logger.log_operation(
            "arm_init",
            details={"sid": sid, "arm_id": arm_id or "target", "mode": mode, "results": results},
        )
        sio.emit('arm:initialized', {"results": results}, to=sid)
        _broadcast_state()
    except Exception as e:
        audit_logger.log_operation(
            "arm_init_error",
            details={"sid": sid, "arm_id": arm_id or "target", "mode": mode, "error": str(e)},
        )
        sio.emit('error', {"message": f"启动使能失败: {str(e)}"}, to=sid)


@sio.event
def arm_deactivate(sid, data=None):
    """
    去使能（电机进入待命状态，可手动移动）
    
    data: {"arm_id": "left" | "right" | null, "target": "left" | "right" | "both"}
    """
    arm_id = data.get("arm_id") if data else None
    target = data.get("target") if data else None
    
    try:
        controller.deactivate_arm(arm_id, target)
        audit_logger.log_operation(
            "arm_deactivate",
            details={"sid": sid, "arm_id": arm_id or "all", "target": target},
        )
        sio.emit('arm:deactivated', {"arm_id": arm_id or "all"}, to=sid)
        _broadcast_state()
    except Exception as e:
        audit_logger.log_operation(
            "arm_deactivate_error",
            details={"sid": sid, "arm_id": arm_id or "all", "target": target, "error": str(e)},
        )
        sio.emit('error', {"message": f"去使能失败: {str(e)}"}, to=sid)


@sio.event
def arm_disable(sid, data=None):
    """
    关闭电机使能
    
    data: {"arm_id": "left" | "right" | null}
    """
    arm_id = data.get("arm_id") if data else None
    
    try:
        controller.disable_arm(arm_id)
        audit_logger.log_operation(
            "arm_disable",
            details={"sid": sid, "arm_id": arm_id or "target"},
        )
        sio.emit('arm:disabled', {"arm_id": arm_id}, to=sid)
        _broadcast_state()
    except Exception as e:
        audit_logger.log_operation(
            "arm_disable_error",
            details={"sid": sid, "arm_id": arm_id or "target", "error": str(e)},
        )
        sio.emit('error', {"message": f"关闭电机失败: {str(e)}"}, to=sid)


# ---------- 紧急控制 ----------

@sio.event
def emergency_stop(sid, data=None):
    """紧急停止"""
    global _smooth_abort
    _smooth_abort = True
    try:
        controller.emergency_stop()
        audit_logger.log_operation(
            "emergency_stop",
            details={"sid": sid},
        )
        sio.emit('arm:emergency_stopped', {}, to=sid)
        _broadcast_state()
    except Exception as e:
        audit_logger.log_operation(
            "emergency_stop_error",
            details={"sid": sid, "error": str(e)},
        )
        sio.emit('error', {"message": f"急停失败: {str(e)}"}, to=sid)


@sio.event
def go_to_zero(sid, data):
    """
    安全回零
    
    data: {"arm_id": "left" | "right" | null}
    """
    arm_id = data.get("arm_id")
    
    try:
        controller.go_to_zero(arm_id)
        audit_logger.log_operation(
            "go_to_zero",
            details={"sid": sid, "arm_id": arm_id or "target"},
        )
        sio.emit('arm:went_to_zero', {"arm_id": arm_id}, to=sid)
    except Exception as e:
        audit_logger.log_operation(
            "go_to_zero_error",
            details={"sid": sid, "arm_id": arm_id or "target", "error": str(e)},
        )
        sio.emit('error', {"message": f"回零失败: {str(e)}"}, to=sid)


# ---------- 目标切换 ----------

@sio.event
def set_target(sid, data):
    """
    设置控制目标
    
    data: {"target": "left" | "right" | "both"}
    """
    target = data.get("target", "both")
    
    try:
        controller.set_target(target)
        audit_logger.log_operation(
            "set_target",
            details={"sid": sid, "target": target},
        )
        sio.emit('target:changed', {"target": target}, to=sid)
        _broadcast_state()
    except Exception as e:
        audit_logger.log_operation(
            "set_target_error",
            details={"sid": sid, "target": target, "error": str(e)},
        )
        sio.emit('error', {"message": f"切换目标失败: {str(e)}"}, to=sid)


# ---------- 手动控制 ----------

@sio.event
def manual_set_position(sid, data):
    """
    手动设置位置
    
    data: {"arm_id": "left" | "right", "motor_id": 51, "position": 0.5}
    """
    arm_id = data.get("arm_id")
    motor_id = data.get("motor_id")
    position = data.get("position")
    
    if not all([arm_id, motor_id is not None, position is not None]):
        msg = "缺少参数"
        audit_logger.log_operation(
            "manual_set_position_invalid",
            details={"sid": sid, "data": data, "reason": msg},
        )
        sio.emit('error', {"message": msg}, to=sid)
        return
    
    try:
        success = controller.set_position(arm_id, motor_id, position)
        if not success:
            # 很可能被软限位拒绝
            msg = f"{arm_id} 臂电机 {motor_id} 目标位置被软限位拒绝"
            audit_logger.log_operation(
                "manual_set_position_rejected",
                details={"sid": sid, "arm_id": arm_id, "motor_id": motor_id, "position": position},
            )
            sio.emit('error', {"message": msg}, to=sid)
            return

        # 立即读取该电机位置并广播（优化：只读取单个电机）
        updated_pos = controller.read_single_motor_position(arm_id, motor_id)

        if updated_pos is not None:
            # 获取zero_offset计算relative_position
            arm_ctrl = controller.arms.get(arm_id)
            zero_offset = arm_ctrl.zero_offsets.get(motor_id, 0.0) if arm_ctrl else 0.0

            # 发送单电机位置更新，而非完整状态
            sio.emit('motor:position_update', {
                "arm_id": arm_id,
                "motor_id": motor_id,
                "position": updated_pos,
                "relative_position": updated_pos - zero_offset
            })

        audit_logger.log_operation(
            "manual_set_position",
            details={"sid": sid, "arm_id": arm_id, "motor_id": motor_id, "position": position},
        )
        sio.emit('manual:position_set', {"success": True}, to=sid)
    except Exception as e:
        audit_logger.log_operation(
            "manual_set_position_error",
            details={"sid": sid, "arm_id": arm_id, "motor_id": motor_id, "position": position, "error": str(e)},
        )
        sio.emit('error', {"message": f"设置位置失败: {str(e)}"}, to=sid)


# ---------- 平滑运动 ----------

_smooth_abort = False   # 紧急停止时置 True
_smooth_lock = False    # 防止并发插值


def _abort_smooth_motion():
    global _smooth_abort
    _smooth_abort = True


drag_teach.stop_smooth_fn = _abort_smooth_motion

def _interpolate_move(arm_id, motor_id, start_pos, end_pos, duration_s=0.3, hz=50):
    """线性插值平滑移动（阻塞, 在 eventlet 绿色线程中调用）"""
    global _smooth_abort
    steps = max(1, int(duration_s * hz))
    for i in range(steps):
        if _smooth_abort:
            return False
        t = (i + 1) / steps
        pos = start_pos + t * (end_pos - start_pos)
        controller.set_position(arm_id, motor_id, pos)
        eventlet.sleep(1.0 / hz)
    return True


def _do_smooth_manual(sid, arm_id, motor_id, target, duration):
    """后台执行平滑手动移动"""
    global _smooth_lock, _smooth_abort
    # 新请求取消旧的
    if _smooth_lock:
        _smooth_abort = True
        eventlet.sleep(0.05)
    _smooth_abort = False
    _smooth_lock = True
    try:
        arm_ctrl = controller.arms.get(arm_id)
        if arm_ctrl is None:
            sio.emit('error', {"message": f"{arm_id} 臂未连接"}, to=sid)
            return
        motor = arm_ctrl.motors.get(motor_id)
        current = getattr(motor, 'position', 0.0) if motor else 0.0
        _interpolate_move(arm_id, motor_id, current, target, duration)
        sio.emit('manual:position_set', {"success": True}, to=sid)
    except Exception as e:
        sio.emit('error', {"message": f"平滑移动失败: {str(e)}"}, to=sid)
    finally:
        _smooth_lock = False


@sio.event
def smooth_manual_move(sid, data):
    """
    平滑手动移动单关节

    data: {"arm_id": "left"|"right", "motor_id": 51, "position": 0.5, "duration_ms": 300}
    """
    arm_id = data.get("arm_id")
    motor_id = data.get("motor_id")
    target = data.get("position")
    duration = data.get("duration_ms", 300) / 1000.0

    if not all([arm_id, motor_id is not None, target is not None]):
        sio.emit('error', {"message": "缺少参数"}, to=sid)
        return

    # 在后台绿色线程中执行，不阻塞事件循环
    eventlet.spawn(_do_smooth_manual, sid, arm_id, motor_id, target, duration)


def _do_smooth_cartesian(sid, data):
    """后台执行平滑笛卡尔移动"""
    global _smooth_lock, _smooth_abort
    # 新请求取消旧的
    if _smooth_lock:
        _smooth_abort = True
        eventlet.sleep(0.05)
    _smooth_abort = False
    _smooth_lock = True

    arm_id = data.get("arm_id", "left")
    x_mm = data.get("x", 0)
    y_mm = data.get("y", 0)
    z_mm = data.get("z", 0)
    duration = data.get("duration_ms", 500) / 1000.0
    hz = 50

    try:
        arm_ctrl = controller.arms.get(arm_id)
        if arm_ctrl is None:
            sio.emit('error', {"message": f"{arm_id} 臂未连接"}, to=sid)
            return

        motor_ids = LEFT_MOTOR_IDS if arm_id == "left" else RIGHT_MOTOR_IDS

        # 当前 URDF 关节角
        current_joints = _get_urdf_joints(arm_ctrl, arm_id)

        # 目标 IK
        target_x, target_y, target_z = x_mm / 1000.0, y_mm / 1000.0, z_mm / 1000.0
        ik_result = cartesian.compute_ik(arm_id, target_x, target_y, target_z, current_joints)

        if ik_result is None:
            sio.emit('error', {"message": f"IK求解失败: ({x_mm}, {y_mm}, {z_mm})mm 不可达"}, to=sid)
            return

        if ik_result["error_mm"] > 5.0:
            sio.emit('error', {"message": f"IK精度不足: {ik_result['error_mm']:.2f}mm"}, to=sid)
            return

        target_joints = ik_result["joints"]

        # 使用 URDF 角度做插值，每一步整臂一次性下发，避免逐关节下发造成的抖动。
        start_joints = current_joints

        # 多步插值
        steps = max(1, int(duration * hz))
        for step_i in range(steps):
            if _smooth_abort:
                break
            t = (step_i + 1) / steps
            step_joints = [
                start_joints[j] + t * (target_joints[j] - start_joints[j])
                for j in range(len(motor_ids))
            ]
            ok = controller.set_joint_offsets(arm_id, step_joints)
            if not ok:
                sio.emit('error', {"message": f"{arm_id} 臂平滑笛卡尔移动被拒绝"}, to=sid)
                return
            eventlet.sleep(1.0 / hz)

        sio.emit('cartesian:moved', {
            "arm_id": arm_id,
            "x": target_x, "y": target_y, "z": target_z,
            "error_mm": ik_result["error_mm"],
        }, to=sid)
    except Exception as e:
        sio.emit('error', {"message": f"平滑笛卡尔移动失败: {str(e)}"}, to=sid)
    finally:
        _smooth_lock = False


@sio.event
def smooth_cartesian_move(sid, data):
    """
    平滑笛卡尔移动

    data: {"arm_id": "left"|"right", "x": mm, "y": mm, "z": mm, "duration_ms": 500}
    x/y/z 单位: mm (前端传入)
    """
    # 旧协议兼容: smooth_cartesian_move(mm) -> motion:execute(CARTESIAN_LINEAR,m)
    payload = {
        "command_id": data.get("command_id") or f"legacy-smooth-{int(time.time() * 1000)}",
        "arm_id": data.get("arm_id", "left"),
        "motion_type": "CARTESIAN_LINEAR",
        "frame": "BASE",
        "target": {
            "pose": {
                "x": float(data.get("x", 0.0)) / 1000.0,
                "y": float(data.get("y", 0.0)) / 1000.0,
                "z": float(data.get("z", 0.0)) / 1000.0,
            }
        },
        "constraints": {},
        "options": {"blocking": False},
    }
    motion_service.execute(
        sid=sid,
        payload=payload,
        legacy_event="smooth_cartesian_move",
        deprecated=True,
        allow_preempt=True,
    )


@sio.event
def manual_set_position_offset(sid, data):
    """
    手动设置相对零点位置
    
    data: {"arm_id": "left" | "right", "motor_id": 51, "position": 0.0}
    """
    arm_id = data.get("arm_id")
    motor_id = data.get("motor_id")
    position = data.get("position")
    
    if not all([arm_id, motor_id is not None, position is not None]):
        msg = "缺少参数"
        audit_logger.log_operation(
            "manual_set_position_offset_invalid",
            details={"sid": sid, "data": data, "reason": msg},
        )
        sio.emit('error', {"message": msg}, to=sid)
        return
    
    try:
        success = controller.set_position_offset(arm_id, motor_id, position)
        if not success:
            msg = f"{arm_id} 臂电机 {motor_id} 相对零点目标位置被软限位拒绝"
            audit_logger.log_operation(
                "manual_set_position_offset_rejected",
                details={"sid": sid, "arm_id": arm_id, "motor_id": motor_id, "position": position},
            )
            sio.emit('error', {"message": msg}, to=sid)
            return

        audit_logger.log_operation(
            "manual_set_position_offset",
            details={"sid": sid, "arm_id": arm_id, "motor_id": motor_id, "position": position},
        )
        sio.emit('manual:position_set', {"success": True, "offset": True}, to=sid)
    except Exception as e:
        audit_logger.log_operation(
            "manual_set_position_offset_error",
            details={"sid": sid, "arm_id": arm_id, "motor_id": motor_id, "position": position, "error": str(e)},
        )
        sio.emit('error', {"message": f"设置相对零点位置失败: {str(e)}"}, to=sid)


@sio.event
def set_speed(sid, data):
    """
    设置速度参数
    
    data: {"velocity": 1.0, "accel": 1.0, "decel": 1.0, "arm_id": null}
    或: {"preset": "slow" | "medium" | "fast", "arm_id": null}
    """
    preset = data.get("preset")
    arm_id = data.get("arm_id")
    
    try:
        if preset:
            controller.set_speed_preset(preset, arm_id)
        else:
            velocity = data.get("velocity", 1.0)
            accel = data.get("accel", 1.0)
            decel = data.get("decel", 1.0)
            controller.set_speed(velocity, accel, decel, arm_id)

        params = controller.get_speed_params()
        audit_logger.log_operation(
            "set_speed",
            details={"sid": sid, "preset": preset, "arm_id": arm_id, "params": params},
        )
        sio.emit('speed:set', {"success": True, "params": params}, to=sid)
    except Exception as e:
        audit_logger.log_operation(
            "set_speed_error",
            details={"sid": sid, "preset": preset, "arm_id": arm_id, "error": str(e)},
        )
        sio.emit('error', {"message": f"设置速度失败: {str(e)}"}, to=sid)


# ---------- Drag Teach / 自由拖动桥接 ----------

def _drag_teach_enable_impl(sid, data, legacy_event: Optional[str] = None):
    payload = data or {}
    arm_id = payload.get("arm_id")
    controller_source = payload.get("controller_source", "soft_compliance")
    try:
        state = drag_teach.enable(
            arm_id=arm_id,
            controller_source=controller_source,
            mit_config_file=payload.get("mit_config_file"),
            mit_kp=float(payload.get("mit_kp", 8.0)),
            mit_kd=float(payload.get("mit_kd", 1.5)),
            mit_torque_bias=payload.get("mit_torque_bias") or {},
            soft_position_kp=float(payload.get("soft_position_kp", 6.0)),
            soft_velocity_kp=float(payload.get("soft_velocity_kp", 0.08)),
            soft_velocity_ki=float(payload.get("soft_velocity_ki", 0.0)),
            soft_velocity_limit=float(payload.get("soft_velocity_limit", 0.35)),
            soft_acceleration=float(payload.get("soft_acceleration", 1.0)),
        )
        audit_logger.log_operation(
            legacy_event or "drag_teach_enable",
            details={
                "sid": sid,
                "arm_id": arm_id or "target",
                "controller_source": controller_source,
            },
        )
        sio.emit('drag_teach:enabled', {"arm_id": arm_id, "state": state}, to=sid)
        sio.emit('freedrive:enabled', {"arm_id": arm_id}, to=sid)
        _broadcast_state()
    except Exception as e:
        audit_logger.log_operation(
            f"{legacy_event or 'drag_teach_enable'}_error",
            details={"sid": sid, "arm_id": arm_id or "target", "error": str(e)},
        )
        sio.emit('error', {"message": f"启用拖拽示教失败: {str(e)}"}, to=sid)


def _drag_teach_disable_impl(sid, data, legacy_event: Optional[str] = None):
    payload = data or {}
    arm_id = payload.get("arm_id")
    try:
        state = drag_teach.disable(arm_id=arm_id)
        audit_logger.log_operation(
            legacy_event or "drag_teach_disable",
            details={"sid": sid, "arm_id": arm_id or "target"},
        )
        sio.emit('drag_teach:disabled', {"arm_id": arm_id, "state": state}, to=sid)
        sio.emit('freedrive:disabled', {"arm_id": arm_id}, to=sid)
        _broadcast_state()
    except Exception as e:
        audit_logger.log_operation(
            f"{legacy_event or 'drag_teach_disable'}_error",
            details={"sid": sid, "arm_id": arm_id or "target", "error": str(e)},
        )
        sio.emit('error', {"message": f"退出拖拽示教失败: {str(e)}"}, to=sid)


@sio.event
def freedrive_enable(sid, data):
    """兼容旧事件：桥接到 drag teach enable"""
    _drag_teach_enable_impl(sid, data or {}, legacy_event="freedrive_enable")


@sio.event
def freedrive_disable(sid, data):
    """兼容旧事件：桥接到 drag teach disable"""
    _drag_teach_disable_impl(sid, data or {}, legacy_event="freedrive_disable")


@sio.event
def drag_teach_enable(sid, data):
    _drag_teach_enable_impl(sid, data or {})


@sio.event
def drag_teach_disable(sid, data=None):
    _drag_teach_disable_impl(sid, data or {})


@sio.event
def drag_teach_state(sid, data=None):
    sio.emit('drag_teach:state', drag_teach.get_public_state(), to=sid)


@sio.event
def drag_teach_start_segment(sid, data):
    payload = data or {}
    try:
        segment = drag_teach.start_segment(
            name=payload.get("name") or f"segment_{int(time.time())}",
            arm_id=payload.get("arm_id"),
            description=payload.get("description", ""),
            sample_type=payload.get("sample_type", "keyframe"),
            segment_type=payload.get("segment_type", "generic"),
            speed_multiplier=float(payload.get("speed_multiplier", 1.0)),
            loop=bool(payload.get("loop", False)),
            metadata=payload.get("metadata") or {},
        )
        audit_logger.log_operation(
            "drag_teach_start_segment",
            details={"sid": sid, "name": segment.get("name"), "sample_type": segment.get("sample_type")},
        )
        sio.emit('drag_teach:segment_started', segment, to=sid)
        _broadcast_state()
    except Exception as e:
        sio.emit('error', {"message": f"启动 drag teach segment 失败: {str(e)}"}, to=sid)


@sio.event
def drag_teach_record_keyframe(sid, data=None):
    payload = data or {}
    try:
        point = drag_teach.record_keyframe(
            name=payload.get("name"),
            arm_id=payload.get("arm_id"),
            delay=float(payload.get("delay", 1.0)),
            hand_action=payload.get("hand_action"),
        )
        audit_logger.log_operation(
            "drag_teach_record_keyframe",
            details={"sid": sid, "name": point.get("name"), "delay": point.get("delay")},
        )
        sio.emit('drag_teach:keyframe_recorded', point, to=sid)
        sio.emit('drag_teach:state', drag_teach.get_public_state(), to=sid)
        _broadcast_state()
    except Exception as e:
        sio.emit('error', {"message": f"记录 drag teach 关键帧失败: {str(e)}"}, to=sid)


@sio.event
def drag_teach_remove_last_point(sid, data=None):
    try:
        result = drag_teach.remove_last_point()
        audit_logger.log_operation(
            "drag_teach_remove_last_point",
            details={"sid": sid, "remaining": result.get("remaining")},
        )
        sio.emit('drag_teach:state', drag_teach.get_public_state(), to=sid)
        _broadcast_state()
    except Exception as e:
        sio.emit('error', {"message": f"删除最后一个点位失败: {str(e)}"}, to=sid)


@sio.event
def drag_teach_append_hand_action(sid, data):
    try:
        event = drag_teach.append_hand_action(data or {})
        audit_logger.log_operation(
            "drag_teach_append_hand_action",
            details={"sid": sid, "event": event},
        )
        sio.emit('drag_teach:state', drag_teach.get_public_state(), to=sid)
        _broadcast_state()
    except Exception as e:
        sio.emit('error', {"message": f"追加手动作失败: {str(e)}"}, to=sid)


@sio.event
def drag_teach_stop_segment(sid, data=None):
    payload = data or {}
    try:
        result = drag_teach.stop_segment(save=bool(payload.get("save", True)))
        audit_logger.log_operation(
            "drag_teach_stop_segment",
            details={"sid": sid, "name": result.get("name"), "filename": result.get("filename")},
        )
        sio.emit('drag_teach:segment_stopped', result, to=sid)
        if result.get("filename"):
            sio.emit('drag_teach:segment_saved', result, to=sid)
        _broadcast_state()
    except Exception as e:
        sio.emit('error', {"message": f"停止 drag teach segment 失败: {str(e)}"}, to=sid)


@sio.event
def drag_teach_cancel_segment(sid, data=None):
    try:
        result = drag_teach.cancel_segment()
        audit_logger.log_operation(
            "drag_teach_cancel_segment",
            details={"sid": sid, "name": result.get("name")},
        )
        sio.emit('drag_teach:disabled', {"cancelled": True, "state": drag_teach.get_public_state()}, to=sid)
        _broadcast_state()
    except Exception as e:
        sio.emit('error', {"message": f"取消 drag teach segment 失败: {str(e)}"}, to=sid)


@sio.event
def drag_teach_list_segments(sid, data=None):
    sio.emit('drag_teach:segments', {"segments": drag_teach.list_segments()}, to=sid)


@sio.event
def drag_teach_preview_segment(sid, data=None):
    payload = data or {}
    try:
        preview = drag_teach.preview_segment(filename=payload.get("filename"))
        sio.emit('drag_teach:preview', preview, to=sid)
    except Exception as e:
        sio.emit('error', {"message": f"预览 drag teach segment 失败: {str(e)}"}, to=sid)


# ---------- 零点标定 ----------

@sio.event
def zero_calibrate(sid, data=None):
    """标定零点"""
    try:
        result = controller.calibrate_zero()

        # 更新笛卡尔坐标系原点
        cartesian.update_origins()

        audit_logger.log_operation(
            "zero_calibrate",
            details={"sid": sid, "result": result},
        )
        sio.emit('zero:calibrated', {"result": result}, to=sid)
        _broadcast_state()
    except Exception as e:
        audit_logger.log_operation(
            "zero_calibrate_error",
            details={"sid": sid, "error": str(e)},
        )
        sio.emit('error', {"message": f"零点标定失败: {str(e)}"}, to=sid)


@sio.event
def zero_load(sid, data=None):
    """查看零点偏移"""
    try:
        offsets = controller.get_zero_offsets()
        audit_logger.log_operation(
            "zero_load",
            details={"sid": sid},
        )
        sio.emit('zero:loaded', {"offsets": offsets}, to=sid)
    except Exception as e:
        audit_logger.log_operation(
            "zero_load_error",
            details={"sid": sid, "error": str(e)},
        )
        sio.emit('error', {"message": f"获取零点失败: {str(e)}"}, to=sid)


# ---------- 示教功能 ----------

@sio.event
def teaching_record(sid, data):
    """
    记录当前位置
    
    data: {"name": "point_1", "arm_id": null}
    """
    name = data.get("name")
    arm_id = data.get("arm_id")
    
    try:
        point = controller.record_point(name, arm_id)
        audit_logger.log_operation(
            "teaching_record",
            details={"sid": sid, "name": name, "arm_id": arm_id, "point": point},
        )
        sio.emit('teaching:recorded', {"point": point}, to=sid)
    except Exception as e:
        audit_logger.log_operation(
            "teaching_record_error",
            details={"sid": sid, "name": name, "arm_id": arm_id, "error": str(e)},
        )
        sio.emit('error', {"message": f"记录位置失败: {str(e)}"}, to=sid)


# ---------- 轨迹管理 ----------

# 当前编辑中的轨迹 (服务端状态)
_editing_trajectory = None   # None 表示未在编辑

TRAJ_DIR = Path(__file__).resolve().parent.parent / "trajectories"
TRAJ_DIR.mkdir(exist_ok=True)


def _traj_emit_state(sid):
    """发送当前编辑中轨迹的完整状态"""
    sio.emit('trajectory:edit_state', {
        "editing": _editing_trajectory is not None,
        "trajectory": _editing_trajectory,
    }, to=sid)


@sio.event
def trajectory_new(sid, data):
    """
    新建轨迹

    data: {"name": "自定义轨迹", "description": ""}
    """
    global _editing_trajectory
    name = data.get("name", f"轨迹_{int(time.time())}")
    desc = data.get("description", "")

    _editing_trajectory = {
        "name": name,
        "description": desc,
        "points": [],
        "loop": False,
        "speed_multiplier": 1.0,
    }
    audit_logger.log_operation("trajectory_new", details={"sid": sid, "name": name})
    _traj_emit_state(sid)
    sio.emit('trajectory:created', {"name": name}, to=sid)


@sio.event
def trajectory_add_point(sid, data):
    """
    记录当前位置作为轨迹点

    data: {"name": "point_1", "delay": 1.0, "arm_id": null}
    """
    global _editing_trajectory
    if _editing_trajectory is None:
        sio.emit('error', {"message": "请先新建或加载轨迹"}, to=sid)
        return

    point_name = data.get("name", f"point_{len(_editing_trajectory['points']) + 1}")
    delay = float(data.get("delay", 1.0))
    arm_id = data.get("arm_id")

    try:
        point_data = controller.record_point(point_name, arm_id)
        traj_point = {
            "name": point_name,
            "positions": {},
            "delay": delay,
        }
        traj_point["positions"] = flatten_record_point_to_positions(point_data)

        _editing_trajectory["points"].append(traj_point)
        audit_logger.log_operation("trajectory_add_point", details={
            "sid": sid, "name": point_name, "total": len(_editing_trajectory["points"]),
        })
        _traj_emit_state(sid)
        sio.emit('trajectory:point_added', {
            "index": len(_editing_trajectory["points"]) - 1,
            "name": point_name,
        }, to=sid)
    except Exception as e:
        sio.emit('error', {"message": f"记录点位失败: {str(e)}"}, to=sid)


@sio.event
def trajectory_remove_point(sid, data):
    """删除轨迹点: data: {"index": 0}"""
    global _editing_trajectory
    if _editing_trajectory is None:
        sio.emit('error', {"message": "未在编辑轨迹"}, to=sid)
        return

    idx = int(data.get("index", -1))
    pts = _editing_trajectory["points"]
    if 0 <= idx < len(pts):
        removed = pts.pop(idx)
        audit_logger.log_operation("trajectory_remove_point", details={"sid": sid, "index": idx, "name": removed.get("name")})
        _traj_emit_state(sid)
    else:
        sio.emit('error', {"message": f"点位索引 {idx} 不合法"}, to=sid)


@sio.event
def trajectory_reorder(sid, data):
    """重排点位: data: {"from_index": 0, "to_index": 2}"""
    global _editing_trajectory
    if _editing_trajectory is None:
        sio.emit('error', {"message": "未在编辑轨迹"}, to=sid)
        return

    fi = int(data.get("from_index", -1))
    ti = int(data.get("to_index", -1))
    pts = _editing_trajectory["points"]
    if 0 <= fi < len(pts) and 0 <= ti < len(pts):
        item = pts.pop(fi)
        pts.insert(ti, item)
        _traj_emit_state(sid)
    else:
        sio.emit('error', {"message": f"索引不合法: {fi} → {ti}"}, to=sid)


@sio.event
def trajectory_update_point(sid, data):
    """修改点位属性: data: {"index": 0, "name": "新名称", "delay": 0.5}"""
    global _editing_trajectory
    if _editing_trajectory is None:
        sio.emit('error', {"message": "未在编辑轨迹"}, to=sid)
        return

    idx = int(data.get("index", -1))
    pts = _editing_trajectory["points"]
    if 0 <= idx < len(pts):
        if "name" in data:
            pts[idx]["name"] = data["name"]
        if "delay" in data:
            pts[idx]["delay"] = float(data["delay"])
        _traj_emit_state(sid)
    else:
        sio.emit('error', {"message": f"点位索引 {idx} 不合法"}, to=sid)


@sio.event
def trajectory_update_meta(sid, data):
    """修改轨迹元数据: data: {"name": "...", "description": "...", "loop": false, "speed_multiplier": 1.0}"""
    global _editing_trajectory
    if _editing_trajectory is None:
        sio.emit('error', {"message": "未在编辑轨迹"}, to=sid)
        return

    if "name" in data:
        _editing_trajectory["name"] = data["name"]
    if "description" in data:
        _editing_trajectory["description"] = data["description"]
    if "loop" in data:
        _editing_trajectory["loop"] = bool(data["loop"])
    if "speed_multiplier" in data:
        _editing_trajectory["speed_multiplier"] = float(data["speed_multiplier"])
    _traj_emit_state(sid)


@sio.event
def trajectory_save(sid, data=None):
    """
    保存当前编辑中的轨迹到文件

    data: {"filename": "teach_xxx.json"}  (可选, 自动生成)
    """
    global _editing_trajectory
    if _editing_trajectory is None:
        sio.emit('error', {"message": "没有正在编辑的轨迹"}, to=sid)
        return

    data = data or {}
    filename = data.get("filename")
    if not filename:
        safe_name = _editing_trajectory["name"].replace(" ", "_").replace("/", "_")
        filename = f"teach_{safe_name}_{int(time.time())}.json"

    # 确保 .json 后缀
    if not filename.endswith(".json"):
        filename += ".json"

    save_data = dict(_editing_trajectory)
    save_data["created_at"] = time.strftime("%Y-%m-%dT%H:%M:%S")

    filepath = TRAJ_DIR / filename
    try:
        with open(filepath, "w", encoding="utf-8") as f:
            json.dump(save_data, f, ensure_ascii=False, indent=2)
        audit_logger.log_operation("trajectory_save", details={"sid": sid, "filename": filename})
        sio.emit('trajectory:saved', {"filename": filename, "name": save_data["name"]}, to=sid)
    except Exception as e:
        sio.emit('error', {"message": f"保存轨迹失败: {str(e)}"}, to=sid)


@sio.event
def trajectory_load_edit(sid, data):
    """
    加载已有轨迹到编辑器

    data: {"filename": "teach_xxx.json"}
    """
    global _editing_trajectory
    filename = data.get("filename")
    if not filename:
        sio.emit('error', {"message": "缺少文件名"}, to=sid)
        return

    filepath = TRAJ_DIR / filename
    if not filepath.exists():
        sio.emit('error', {"message": f"轨迹文件不存在: {filename}"}, to=sid)
        return

    try:
        with open(filepath, "r", encoding="utf-8") as f:
            traj = json.load(f)
        _editing_trajectory = {
            "name": traj.get("name", filename),
            "description": traj.get("description", ""),
            "points": traj.get("points", []),
            "loop": traj.get("loop", False),
            "speed_multiplier": traj.get("speed_multiplier", 1.0),
            "_filename": filename,
        }
        audit_logger.log_operation("trajectory_load_edit", details={"sid": sid, "filename": filename})
        _traj_emit_state(sid)
        sio.emit('trajectory:loaded', {"filename": filename, "name": _editing_trajectory["name"]}, to=sid)
    except Exception as e:
        sio.emit('error', {"message": f"加载轨迹失败: {str(e)}"}, to=sid)


@sio.event
def trajectory_delete_file(sid, data):
    """删除轨迹文件: data: {"filename": "teach_xxx.json"}"""
    filename = data.get("filename")
    if not filename:
        sio.emit('error', {"message": "缺少文件名"}, to=sid)
        return

    filepath = TRAJ_DIR / filename
    if filepath.exists():
        try:
            filepath.unlink()
            audit_logger.log_operation("trajectory_delete", details={"sid": sid, "filename": filename})
            sio.emit('trajectory:deleted', {"filename": filename}, to=sid)
        except Exception as e:
            sio.emit('error', {"message": f"删除失败: {str(e)}"}, to=sid)
    else:
        sio.emit('error', {"message": f"文件不存在: {filename}"}, to=sid)


@sio.event
def trajectory_list(sid, data=None):
    """列出所有轨迹文件 (Socket.IO 版)"""
    try:
        files = []
        for f in sorted(TRAJ_DIR.glob("*.json")):
            try:
                with open(f, "r", encoding="utf-8") as fp:
                    meta = json.load(fp)
                files.append({
                    "filename": f.name,
                    "name": meta.get("name", f.stem),
                    "description": meta.get("description", ""),
                    "points_count": len(meta.get("points", [])),
                })
            except Exception:
                files.append({"filename": f.name, "name": f.stem, "description": "", "points_count": 0})
        sio.emit('trajectory:list', {"files": files}, to=sid)
    except Exception as e:
        sio.emit('error', {"message": f"获取轨迹列表失败: {str(e)}"}, to=sid)


@sio.event
def trajectory_close_edit(sid, data=None):
    """关闭编辑器 (不保存)"""
    global _editing_trajectory
    _editing_trajectory = None
    _traj_emit_state(sid)




@sio.event
def trajectory_play(sid, data):
    """
    播放轨迹
    
    data: {"filename": "preset_a.json", "sync": true, "loop": false}
    """
    filename = data.get("filename")
    sync = data.get("sync", True)
    loop = data.get("loop", None)
    
    if drag_teach.get_public_state().get("enabled"):
        sio.emit('error', {"message": "拖拽示教模式进行中，请先退出后再播放轨迹"}, to=sid)
        return

    if not filename:
        msg = "缺少轨迹文件名"
        audit_logger.log_operation(
            "trajectory_play_invalid",
            details={"sid": sid, "data": data, "reason": msg},
        )
        sio.emit('error', {"message": msg}, to=sid)
        return
    
    try:
        controller.play_trajectory(filename, sync, loop_override=loop)
        audit_logger.log_operation(
            "trajectory_play",
            details={"sid": sid, "filename": filename, "sync": sync, "loop": loop},
        )
        sio.emit('trajectory:started', {"filename": filename, "loop": loop}, to=sid)
        _broadcast_state()
    except Exception as e:
        audit_logger.log_operation(
            "trajectory_play_error",
            details={"sid": sid, "filename": filename, "sync": sync, "error": str(e)},
        )
        sio.emit('error', {"message": f"播放轨迹失败: {str(e)}"}, to=sid)


@sio.event
def trajectory_pause(sid, data=None):
    """暂停播放"""
    try:
        controller.pause_playback()
        audit_logger.log_operation(
            "trajectory_pause",
            details={"sid": sid},
        )
        sio.emit('trajectory:paused', {}, to=sid)
        _broadcast_state()
    except Exception as e:
        audit_logger.log_operation(
            "trajectory_pause_error",
            details={"sid": sid, "error": str(e)},
        )
        sio.emit('error', {"message": f"暂停失败: {str(e)}"}, to=sid)


@sio.event
def trajectory_resume(sid, data=None):
    """恢复播放"""
    try:
        controller.resume_playback()
        audit_logger.log_operation(
            "trajectory_resume",
            details={"sid": sid},
        )
        sio.emit('trajectory:resumed', {}, to=sid)
        _broadcast_state()
    except Exception as e:
        audit_logger.log_operation(
            "trajectory_resume_error",
            details={"sid": sid, "error": str(e)},
        )
        sio.emit('error', {"message": f"恢复失败: {str(e)}"}, to=sid)


@sio.event
def trajectory_stop(sid, data=None):
    """停止播放"""
    try:
        controller.stop_playback()
        audit_logger.log_operation(
            "trajectory_stop",
            details={"sid": sid},
        )
        sio.emit('trajectory:stopped', {}, to=sid)
        _broadcast_state()
    except Exception as e:
        audit_logger.log_operation(
            "trajectory_stop_error",
            details={"sid": sid, "error": str(e)},
        )
        sio.emit('error', {"message": f"停止失败: {str(e)}"}, to=sid)


# ---------- 笛卡尔控制 ----------

def _get_urdf_joints(arm_ctrl, arm_id: str):
    """
    获取 URDF 坐标系下的关节角度 (= 绝对编码器位置 - 零点偏移)
    FK/IK 运动学链期望的是 URDF 角度, 不是原始编码器值。
    """
    motor_ids = LEFT_MOTOR_IDS if arm_id == "left" else RIGHT_MOTOR_IDS
    joints = []
    for mid in motor_ids:
        motor = arm_ctrl.motors.get(mid)
        abs_pos = getattr(motor, 'position', 0.0) if motor else 0.0
        zero_offset = arm_ctrl.zero_offsets.get(mid, 0.0)
        joints.append(abs_pos - zero_offset)
    return joints

@sio.event
def cartesian_get_fk(sid, data):
    """
    查询当前末端位置 (FK)

    data: {"arm_id": "left" | "right"}
    返回: {"arm_id", "x", "y", "z", "abs_x", "abs_y", "abs_z"}
    """
    arm_id = data.get("arm_id", "left")

    try:
        # 获取当前关节角度
        arm_controller = controller.arms.get(arm_id)
        if arm_controller is None:
            sio.emit('cartesian:result', {
                "arm_id": arm_id,
                "error": f"{arm_id} 臂未连接"
            }, to=sid)
            return

        joints = _get_urdf_joints(arm_controller, arm_id)

        result = cartesian.compute_fk(arm_id, joints)
        result["arm_id"] = arm_id
        sio.emit('cartesian:result', result, to=sid)
    except Exception as e:
        sio.emit('cartesian:result', {
            "arm_id": arm_id,
            "error": f"FK计算失败: {str(e)}"
        }, to=sid)


@sio.event
def cartesian_move_to(sid, data):
    """
    笛卡尔绝对位置控制

    data: {"arm_id": "left"|"right", "x": 0.1, "y": 0.2, "z": 0.3}
    坐标单位: 米 (末端原点坐标系)
    """
    if drag_teach.get_public_state().get("enabled"):
        sio.emit('error', {"message": "拖拽示教模式进行中，请先退出后再执行自动运动"}, to=sid)
        return
    payload = {
        "command_id": data.get("command_id") or f"legacy-ptp-{int(time.time() * 1000)}",
        "arm_id": data.get("arm_id", "left"),
        "motion_type": "CARTESIAN_PTP",
        "frame": "BASE",
        "target": {
            "pose": {
                "x": float(data.get("x", 0.0)),
                "y": float(data.get("y", 0.0)),
                "z": float(data.get("z", 0.0)),
                "rx": data.get("rx"),
                "ry": data.get("ry"),
                "rz": data.get("rz"),
            }
        },
        "constraints": data.get("constraints", {}),
        "options": {"blocking": bool(data.get("blocking", False))},
    }
    motion_service.execute(
        sid=sid,
        payload=payload,
        legacy_event="cartesian_move_to",
        deprecated=True,
        allow_preempt=True,
    )


@sio.event
def cartesian_jog(sid, data):
    """
    笛卡尔增量控制 (点动)

    data: {"arm_id": "left"|"right", "axis": "x"|"y"|"z", "delta": 0.01}
    delta 单位: 米
    """
    if drag_teach.get_public_state().get("enabled"):
        sio.emit('error', {"message": "拖拽示教模式进行中，请先退出后再执行点动"}, to=sid)
        return
    arm_id = data.get("arm_id", "left")
    axis = data.get("axis", "x")
    delta = float(data.get("delta", 0.01))
    payload = {
        "command_id": data.get("command_id") or f"legacy-jog-{int(time.time() * 1000)}",
        "arm_id": arm_id,
        "motion_type": "CARTESIAN_JOG",
        "frame": "BASE",
        "target": {
            "delta": {
                "dx": delta if axis == "x" else 0.0,
                "dy": delta if axis == "y" else 0.0,
                "dz": delta if axis == "z" else 0.0,
                "drx": 0.0,
                "dry": 0.0,
                "drz": 0.0,
            }
        },
        "constraints": data.get("constraints", {}),
        "options": {"blocking": False},
    }
    motion_service.execute(
        sid=sid,
        payload=payload,
        legacy_event="cartesian_jog",
        deprecated=True,
        allow_preempt=True,
    )


@sio.on("motion:execute")
def motion_execute(sid, data):
    """统一运动执行入口: motion:execute"""
    if drag_teach.get_public_state().get("enabled"):
        sio.emit('error', {"message": "拖拽示教模式进行中，请先退出后再执行 motion:execute"}, to=sid)
        return
    motion_service.execute(sid=sid, payload=data or {}, legacy_event=None, deprecated=False, allow_preempt=False)


@sio.on("execution:execute")
def execution_execute_socket(sid, data):
    """统一执行层入口: execution:execute"""
    if drag_teach.get_public_state().get("enabled"):
        sio.emit('error', {"message": "拖拽示教模式进行中，请先退出后再执行 execution:execute"}, to=sid)
        return
    result = motion_service.execute_action(sid=sid, payload=data or {})
    sio.emit("execution:result", result, to=sid)


@sio.on("motion:cancel")
def motion_cancel(sid, data=None):
    """取消命令: motion:cancel"""
    motion_service.cancel(sid=sid, payload=data or {})


@sio.on("motion:pause")
def motion_pause(sid, data=None):
    """暂停命令: motion:pause"""
    motion_service.pause(sid=sid, payload=data or {})


@sio.on("motion:resume")
def motion_resume(sid, data=None):
    """恢复命令: motion:resume"""
    motion_service.resume(sid=sid, payload=data or {})


@sio.on("motion:query")
def motion_query(sid, data=None):
    """查询命令状态: motion:query"""
    motion_service.query(sid=sid, payload=data or {})


# ---------- 状态请求 ----------

@sio.event
def state_request(sid, data=None):
    """请求当前状态"""
    audit_logger.log_operation(
        "state_request",
        details={"sid": sid},
    )
    sio.emit('state:update', _compose_state(), to=sid)
    sio.emit('cartesian:live', _compose_cartesian_live(), to=sid)


# ==================== 状态广播 ====================

def _compose_state():
    """组合对外状态：控制器状态 + drag teach 摘要 + FK 信息"""
    try:
        state = controller.get_state()
        state["drag_teach"] = drag_teach.get_public_state()

        # 附加笛卡尔坐标信息
        try:
            cartesian_data = {}
            for arm_id in ["left", "right"]:
                arm_ctrl = controller.arms.get(arm_id)
                if arm_ctrl is not None:
                    joints = _get_urdf_joints(arm_ctrl, arm_id)
                    fk = cartesian.compute_fk(arm_id, joints)
                    cartesian_data[arm_id] = {
                        "x": round(fk["x"] * 1000, 2),  # 转为mm
                        "y": round(fk["y"] * 1000, 2),
                        "z": round(fk["z"] * 1000, 2),
                        "roll": round(math.degrees(fk["roll"]), 2),
                        "pitch": round(math.degrees(fk["pitch"]), 2),
                        "yaw": round(math.degrees(fk["yaw"]), 2),
                    }
            state["cartesian"] = cartesian_data
        except Exception:
            pass
        return state
    except Exception as e:
        print(f"[App] 组合状态失败: {e}")
        return controller.get_state()


def _broadcast_state():
    """广播状态给所有客户端"""
    try:
        sio.emit('state:update', _compose_state())
    except Exception as e:
        print(f"[App] 广播状态失败: {e}")


def _compose_cartesian_live():
    """轻量实时笛卡尔状态（双臂）"""
    payload = {
        "server_ts_ms": int(time.time() * 1000),
        "playback_state": getattr(controller.playback.state, "value", str(controller.playback.state)),
        "arms": {},
    }
    try:
        for arm_id in ["left", "right"]:
            arm_ctrl = controller.arms.get(arm_id)
            if arm_ctrl is None:
                payload["arms"][arm_id] = {"connected": False}
                continue
            joints = _get_urdf_joints(arm_ctrl, arm_id)
            fk = cartesian.compute_fk(arm_id, joints)
            payload["arms"][arm_id] = {
                "connected": True,
                "x_mm": float(fk["x"] * 1000.0),
                "y_mm": float(fk["y"] * 1000.0),
                "z_mm": float(fk["z"] * 1000.0),
                "rx_deg": float(math.degrees(fk["roll"])),
                "ry_deg": float(math.degrees(fk["pitch"])),
                "rz_deg": float(math.degrees(fk["yaw"])),
            }
    except Exception as e:
        payload["error"] = str(e)
    return payload


def _broadcast_cartesian_live():
    try:
        sio.emit("cartesian:live", _compose_cartesian_live())
    except Exception as e:
        print(f"[App] 广播 cartesian:live 失败: {e}")



def _run_in_thread(fn):
    """在真实线程中执行函数，同时用 eventlet.sleep 让 greenlet 调度器保持响应"""
    done = [False]
    def _wrapper():
        try:
            fn()
        finally:
            done[0] = True
    t = threading.Thread(target=_wrapper, daemon=True)
    t.start()
    # 轮询等待，期间让出事件循环
    while not done[0]:
        eventlet.sleep(0.01)


def _state_update_loop():
    """状态更新循环 - 在真实线程中执行 read_positions，避免 RLock 阻塞 eventlet 事件循环"""
    global _state_update_running
    _state_update_running = True

    last_state_emit = 0.0
    last_live_emit = 0.0

    while _state_update_running:
        try:
            now = time.time()

            # 始终读取当前位置，保证 moveJ/moveL 期间坐标准实时刷新。
            _run_in_thread(controller.read_positions)

            # 高频：仅推送轻量笛卡尔实时数据
            if (now - last_live_emit) * 1000.0 >= float(CARTESIAN_LIVE_INTERVAL):
                _broadcast_cartesian_live()
                last_live_emit = now

            # 低频：推送全量状态
            if (now - last_state_emit) * 1000.0 >= float(STATE_UPDATE_INTERVAL):
                _broadcast_state()
                last_state_emit = now

            # 主循环 tick
            eventlet.sleep(0.02)
        except Exception as e:
            print(f"[App] 状态更新出错: {e}")
            eventlet.sleep(1)



# ==================== 回调设置 ====================

def _on_error(message):
    """错误回调"""
    audit_logger.log_operation(
        "controller_error",
        details={"message": message},
    )
    sio.emit('error', {"message": message})


def _on_playback_progress(info):
    """播放进度回调"""
    sio.emit('trajectory:progress', info)


# 设置控制器回调
controller.set_callbacks(
    on_error=_on_error,
    on_playback_progress=_on_playback_progress
)


# ==================== 主入口 ====================

def main():
    """主函数"""
    print("=" * 50)
    print("  机械臂演出控制系统")
    print("=" * 50)
    print(f"  访问地址: http://{HOST}:{PORT}")
    print(f"  调试模式: {DEBUG}")
    print("=" * 50)
    print()
    
    # 确保轨迹目录存在
    TRAJECTORIES_DIR.mkdir(parents=True, exist_ok=True)
    
    # 确保零点偏移文件存在（不存在则写入空对象）
    if not ZERO_OFFSET_FILE.exists():
        try:
            with open(ZERO_OFFSET_FILE, "w", encoding="utf-8") as f:
                json.dump({}, f, indent=2)
            print(f"  已创建零点文件: {ZERO_OFFSET_FILE}")
        except Exception as e:
            print(f"  创建零点文件失败: {e}")
    
    # 启动状态更新线程
    eventlet.spawn(_state_update_loop)
    
    # 启动服务
    eventlet.wsgi.server(
        eventlet.listen((HOST, PORT)),
        flask_app,
        log_output=DEBUG
    )


if __name__ == '__main__':
    main()
