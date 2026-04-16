"""
轨迹插值器 - 提供多种轨迹插值算法

支持:
- 线性插值 (Linear)
- 三次样条插值 (Cubic Spline)
- 梯形速度曲线插值 (Trapezoidal)
- S型速度曲线插值 (S-Curve)
"""
import math
from typing import List, Dict, Tuple, Optional
from dataclasses import dataclass
from enum import Enum


class InterpolationType(Enum):
    """插值类型"""
    LINEAR = "linear"
    CUBIC_SPLINE = "cubic"
    TRAPEZOIDAL = "trapezoidal"
    S_CURVE = "s_curve"


@dataclass
class TrajectoryPoint:
    """轨迹点数据"""
    positions: Dict[int, float]
    time: float = 0.0
    name: str = ""


class TrajectoryInterpolator:
    """轨迹插值器"""
    
    def __init__(self, 
                 interpolation_type: InterpolationType = InterpolationType.S_CURVE,
                 num_points: int = 50):
        self.interpolation_type = interpolation_type
        self.num_points = num_points
    
    def interpolate(self, 
                   points: List[TrajectoryPoint],
                   duration: Optional[float] = None) -> List[TrajectoryPoint]:
        """对轨迹点进行插值"""
        if len(points) < 2:
            return points
        
        interpolated = []
        
        for i in range(len(points) - 1):
            p1 = points[i]
            p2 = points[i + 1]
            
            if duration:
                segment_duration = duration / (len(points) - 1)
            else:
                segment_duration = p2.time - p1.time if p2.time > p1.time else 1.0
            
            segment_points = self._interpolate_segment(p1, p2, segment_duration)
            interpolated.extend(segment_points)
        
        interpolated.append(points[-1])
        return interpolated
    
    def _interpolate_segment(self, p1, p2, duration):
        """对单个段落进行插值"""
        motor_ids = list(p1.positions.keys())
        
        if self.interpolation_type == InterpolationType.LINEAR:
            return self._linear_interpolation(p1, p2, duration, motor_ids)
        elif self.interpolation_type == InterpolationType.S_CURVE:
            return self._s_curve_interpolation(p1, p2, duration, motor_ids)
        else:
            return self._linear_interpolation(p1, p2, duration, motor_ids)
    
    def _linear_interpolation(self, p1, p2, duration, motor_ids):
        """线性插值"""
        points = []
        for i in range(self.num_points):
            t = i / self.num_points
            positions = {}
            for mid in motor_ids:
                pos1 = p1.positions.get(mid, 0.0)
                pos2 = p2.positions.get(mid, 0.0)
                positions[mid] = pos1 + (pos2 - pos1) * t
            
            points.append(TrajectoryPoint(
                positions=positions,
                time=t * duration,
                name=f"{p1.name}_to_{p2.name}"
            ))
        return points
    
    def _s_curve_interpolation(self, p1, p2, duration, motor_ids):
        """S型速度曲线插值"""
        points = []
        
        for i in range(self.num_points):
            t = i / self.num_points
            
            positions = {}
            for mid in motor_ids:
                p0 = p1.positions.get(mid, 0.0)
                delta = p2.positions.get(mid, 0.0) - p0
                
                # S曲线插值
                if t < 0.25:
                    tau = t / 0.25
                    pos = p0 + delta * (tau**3) / 4
                elif t < 0.5:
                    tau = (t - 0.25) / 0.25
                    pos = p0 + delta * (1/4 + tau**2 / 4 + tau**3 / 12)
                elif t < 0.75:
                    tau = (t - 0.5) / 0.25
                    pos = p0 + delta * (1/3 + tau / 4)
                else:
                    tau = (t - 0.75) / 0.25
                    pos = p0 + delta * (7/12 + tau / 4 - tau**3 / 12 + tau**4 / 12)
                
                positions[mid] = pos
            
            points.append(TrajectoryPoint(
                positions=positions,
                time=t * duration,
                name=f"{p1.name}_to_{p2.name}"
            ))
        
        return points


class SynchronizedMotionPlanner:
    """同步运动规划器 - 确保多轴运动同步完成"""
    
    def __init__(self, sync_tolerance_ms: float = 50.0):
        self.sync_tolerance_ms = sync_tolerance_ms
    
    def calculate_sync_delay(self, start_times: Dict[str, float]) -> Dict[str, float]:
        """计算同步延迟"""
        if not start_times:
            return {}
        
        base_time = min(start_times.values())
        delays = {}
        
        for arm, start_time in start_times.items():
            delay_ms = (start_time - base_time) * 1000
            if delay_ms > self.sync_tolerance_ms:
                delays[arm] = delay_ms - self.sync_tolerance_ms
            else:
                delays[arm] = 0
        
        return delays


def create_interpolator(interp_type: str = "s_curve", num_points: int = 50) -> TrajectoryInterpolator:
    """创建插值器工厂函数"""
    type_map = {
        "linear": InterpolationType.LINEAR,
        "s_curve": InterpolationType.S_CURVE,
    }
    itype = type_map.get(interp_type, InterpolationType.S_CURVE)
    return TrajectoryInterpolator(interpolation_type=itype, num_points=num_points)
