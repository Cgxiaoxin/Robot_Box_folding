"""
机械臂3D可视化配置
"""
import json
from typing import Dict, List, Any


class RobotArm3D:
    """机械臂3D模型配置"""
    
    DH_PARAMS = {
        "left": [
            {"theta": 0, "d": 0.1, "a": 0, "alpha": 0},
            {"theta": 0, "d": 0, "a": 0.1, "alpha": 90},
            {"theta": 0, "d": 0, "a": 0.2, "alpha": 0},
            {"theta": 0, "d": 0, "a": 0.1, "alpha": 90},
            {"theta": 0, "d": 0, "a": 0.1, "alpha": -90},
            {"theta": 0, "d": 0, "a": 0.05, "alpha": 90},
            {"theta": 0, "d": 0.1, "a": 0, "alpha": 0},
        ],
        "right": [
            {"theta": 0, "d": 0.1, "a": 0, "alpha": 0},
            {"theta": 0, "d": 0, "a": 0.1, "alpha": 90},
            {"theta": 0, "d": 0, "a": 0.2, "alpha": 0},
            {"theta": 0, "d": 0, "a": 0.1, "alpha": 90},
            {"theta": 0, "d": 0, "a": 0.1, "alpha": -90},
            {"theta": 0, "d": 0, "a": 0.05, "alpha": 90},
            {"theta": 0, "d": 0.1, "a": 0, "alpha": 0},
        ]
    }
    
    MOTOR_TO_JOINT = {
        "left": {51: 0, 52: 1, 53: 2, 54: 3, 55: 4, 56: 5, 57: 6},
        "right": {61: 0, 62: 1, 63: 2, 64: 3, 65: 4, 66: 5, 67: 6}
    }
    
    JOINT_COLORS = [0xFF6B6B, 0x4ECDC4, 0x45B7D1, 0x96CEB4, 0xFFEAA7, 0xDDA0DD, 0x98D8C8]
    END_EFFECTOR_COLOR = 0xFFA500
    
    @staticmethod
    def generate_threejs_config() -> Dict[str, Any]:
        return {
            "dh_params": RobotArm3D.DH_PARAMS,
            "motor_to_joint": RobotArm3D.MOTOR_TO_JOINT,
            "joint_colors": RobotArm3D.JOINT_COLORS,
            "end_effector_color": RobotArm3D.END_EFFECTOR_COLOR,
            "base_color": 0x333333,
            "link_color": 0xCCCCCC,
        }
    
    @staticmethod
    def positions_to_joint_angles(arm_id: str, positions: Dict[str, float]) -> List[float]:
        motor_to_joint = RobotArm3D.MOTOR_TO_JOINT.get(arm_id, {})
        joint_angles = [0.0] * 7
        for motor_id, pos in positions.items():
            joint_idx = motor_to_joint.get(int(motor_id))
            if joint_idx is not None:
                joint_angles[joint_idx] = float(pos)
        return joint_angles


class TrajectoryPreview:
    """轨迹预览生成器"""
    
    @staticmethod
    def generate_preview_data(trajectory_data: Dict, num_frames: int = 60) -> Dict:
        points = trajectory_data.get("points", [])
        if not points:
            return {"frames": [], "duration": 0}
        
        total_delay = sum(p.get("delay", 1.0) for p in points)
        speed_mult = trajectory_data.get("speed_multiplier", 1.0)
        duration = total_delay / speed_mult
        
        frames = []
        frame_time = duration / num_frames
        
        for i in range(num_frames):
            t = i * frame_time
            frame_data = TrajectoryPreview._interpolate_frame(points, t, speed_mult)
            frames.append({
                "time": t,
                "positions": frame_data,
            })
        
        return {"frames": frames, "duration": duration, "total_points": len(points)}
    
    @staticmethod
    def _interpolate_frame(points: List[Dict], t: float, speed_mult: float) -> Dict:
        elapsed = 0
        for i, point in enumerate(points):
            delay = point.get("delay", 1.0) / speed_mult
            if elapsed + delay >= t:
                result = {"left": {}, "right": {}}
                positions = point.get("positions", {})
                for key, val in positions.items():
                    mid = int(key) if isinstance(key, str) else key
                    if 51 <= mid <= 57:
                        result["left"][mid] = val
                    elif 61 <= mid <= 67:
                        result["right"][mid] = val
                return result
            elapsed += delay
        
        last_point = points[-1]
        result = {"left": {}, "right": {}}
        for key, val in last_point.get("positions", {}).items():
            mid = int(key) if isinstance(key, str) else key
            if 51 <= mid <= 57:
                result["left"][mid] = val
            elif 61 <= mid <= 67:
                result["right"][mid] = val
        return result
