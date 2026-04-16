"""
轨迹播放引擎

提供轨迹文件管理和高级播放功能
"""
import json
import time
from pathlib import Path
from typing import Dict, List, Optional, Any
from dataclasses import dataclass
from datetime import datetime

from .config import TRAJECTORIES_DIR


@dataclass
class TrajectoryPoint:
    """轨迹点"""
    name: str
    positions: Dict[str, float]
    delay: float = 1.0
    timestamp: Optional[float] = None


@dataclass  
class Trajectory:
    """轨迹数据"""
    name: str
    description: str = ""
    points: List[TrajectoryPoint] = None
    loop: bool = False
    speed_multiplier: float = 1.0
    created_at: Optional[str] = None
    
    def __post_init__(self):
        if self.points is None:
            self.points = []


class TrajectoryEngine:
    """轨迹引擎"""
    
    def __init__(self):
        self._ensure_dir()
    
    def _ensure_dir(self):
        """确保轨迹目录存在"""
        TRAJECTORIES_DIR.mkdir(parents=True, exist_ok=True)
    
    def list_trajectories(self) -> List[Dict[str, Any]]:
        """
        列出所有轨迹文件
        
        Returns:
            轨迹信息列表
        """
        self._ensure_dir()
        files = []
        
        for f in sorted(TRAJECTORIES_DIR.glob("*.json")):
            try:
                with open(f, 'r', encoding='utf-8') as fp:
                    data = json.load(fp)
                    files.append({
                        "filename": f.name,
                        "name": data.get("name", f.stem),
                        "description": data.get("description", ""),
                        "points_count": len(data.get("points", [])),
                        "loop": data.get("loop", False)
                    })
            except Exception:
                files.append({
                    "filename": f.name,
                    "name": f.stem,
                    "description": "无法读取",
                    "points_count": 0,
                    "loop": False
                })
        
        return files
    
    def load(self, filename: str) -> Optional[Trajectory]:
        """
        加载轨迹文件
        
        Args:
            filename: 文件名
            
        Returns:
            Trajectory对象或None
        """
        filepath = TRAJECTORIES_DIR / filename
        if not filepath.exists():
            return None
        
        try:
            with open(filepath, 'r', encoding='utf-8') as f:
                data = json.load(f)
            
            points = []
            for p in data.get("points", []):
                points.append(TrajectoryPoint(
                    name=p.get("name", ""),
                    positions=p.get("positions", {}),
                    delay=p.get("delay", 1.0),
                    timestamp=p.get("timestamp")
                ))
            
            return Trajectory(
                name=data.get("name", filename),
                description=data.get("description", ""),
                points=points,
                loop=data.get("loop", False),
                speed_multiplier=data.get("speed_multiplier", 1.0),
                created_at=data.get("created_at")
            )
        except Exception as e:
            print(f"[TrajectoryEngine] 加载轨迹失败: {e}")
            return None
    
    def save(self, trajectory: Trajectory, filename: str) -> bool:
        """
        保存轨迹文件
        
        Args:
            trajectory: 轨迹对象
            filename: 文件名
            
        Returns:
            是否保存成功
        """
        self._ensure_dir()
        filepath = TRAJECTORIES_DIR / filename
        
        try:
            data = {
                "name": trajectory.name,
                "description": trajectory.description,
                "points": [
                    {
                        "name": p.name,
                        "positions": p.positions,
                        "delay": p.delay,
                        "timestamp": p.timestamp
                    }
                    for p in trajectory.points
                ],
                "loop": trajectory.loop,
                "speed_multiplier": trajectory.speed_multiplier,
                "created_at": trajectory.created_at or datetime.now().isoformat()
            }
            
            with open(filepath, 'w', encoding='utf-8') as f:
                json.dump(data, f, ensure_ascii=False, indent=2)
            
            return True
        except Exception as e:
            print(f"[TrajectoryEngine] 保存轨迹失败: {e}")
            return False
    
    def delete(self, filename: str) -> bool:
        """删除轨迹文件"""
        filepath = TRAJECTORIES_DIR / filename
        try:
            if filepath.exists():
                filepath.unlink()
                return True
            return False
        except Exception as e:
            print(f"[TrajectoryEngine] 删除轨迹失败: {e}")
            return False
    
    def create_from_points(self, 
                           name: str,
                           points: List[Dict],
                           description: str = "",
                           loop: bool = False) -> Trajectory:
        """
        从点位列表创建轨迹
        
        Args:
            name: 轨迹名称
            points: 点位列表 [{"name": "...", "positions": {...}, "delay": 1.0}, ...]
            description: 描述
            loop: 是否循环
            
        Returns:
            Trajectory对象
        """
        traj_points = []
        for p in points:
            traj_points.append(TrajectoryPoint(
                name=p.get("name", f"point_{len(traj_points)}"),
                positions=p.get("positions", {}),
                delay=p.get("delay", 1.0),
                timestamp=time.time()
            ))
        
        return Trajectory(
            name=name,
            description=description,
            points=traj_points,
            loop=loop,
            created_at=datetime.now().isoformat()
        )
    
    def merge_trajectories(self, 
                           trajectory_files: List[str],
                           output_name: str) -> Optional[Trajectory]:
        """
        合并多个轨迹文件
        
        Args:
            trajectory_files: 要合并的文件列表
            output_name: 输出轨迹名称
            
        Returns:
            合并后的Trajectory对象
        """
        all_points = []
        
        for filename in trajectory_files:
            traj = self.load(filename)
            if traj:
                all_points.extend(traj.points)
        
        if not all_points:
            return None
        
        return Trajectory(
            name=output_name,
            description=f"合并自: {', '.join(trajectory_files)}",
            points=all_points,
            created_at=datetime.now().isoformat()
        )
    
    def calculate_duration(self, trajectory: Trajectory) -> float:
        """
        计算轨迹总时长
        
        Args:
            trajectory: 轨迹对象
            
        Returns:
            总时长（秒）
        """
        return sum(p.delay for p in trajectory.points) / trajectory.speed_multiplier


# 全局实例
_engine: Optional[TrajectoryEngine] = None


def get_engine() -> TrajectoryEngine:
    """获取轨迹引擎单例"""
    global _engine
    if _engine is None:
        _engine = TrajectoryEngine()
    return _engine
