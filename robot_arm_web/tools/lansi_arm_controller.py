#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
蓝思机械臂控制器
支持配置文件加载，多电机协同控制
"""
import os
import sys
import time
import yaml
import math
import threading
from typing import List, Dict, Optional, Tuple
from lansi_motor_controller import LansiMotorController


class LansiArmController:
    """
    蓝思机械臂控制器
    支持配置文件加载和多电机协同控制
    """
    
    def __init__(self, config_file: str = "config.yaml"):
        """
        初始化机械臂控制器
        
        参数:
            config_file: 配置文件路径
        """
        self.config_file = config_file
        self.config = {}
        self.motors: Dict[int, LansiMotorController] = {}
        self.motor_names: Dict[int, str] = {}
        self.can_channel = "can0"
        self.baudrate = 1000000
        
        # 加载配置
        self._load_config()
        
        # 初始化电机
        self._init_motors()
        
        print(f"\n机械臂控制器初始化完成，共 {len(self.motors)} 个电机")
    
    def _load_config(self):
        """加载配置文件"""
        if not os.path.exists(self.config_file):
            print(f"警告: 配置文件 {self.config_file} 不存在，使用默认配置")
            return
        
        try:
            with open(self.config_file, 'r', encoding='utf-8') as f:
                self.config = yaml.safe_load(f)
            
            # 解析CAN配置
            can_config = self.config.get('can', {})
            self.can_channel = can_config.get('channel', 'can0')
            self.baudrate = can_config.get('baudrate', 1000000)
            
            print(f"配置文件加载成功: {self.config_file}")
            
        except Exception as e:
            print(f"加载配置文件失败: {e}")
    
    def _init_motors(self):
        """初始化所有电机"""
        motors_config = self.config.get('motors', [])
        
        if not motors_config:
            print("警告: 配置文件中没有电机配置")
            return
        
        for motor_config in motors_config:
            motor_id = motor_config.get('id', 1)
            motor_name = motor_config.get('name', f'电机{motor_id}')
            
            try:
                # 创建电机控制器
                motor = LansiMotorController(
                    motor_id=motor_id,
                    can_channel=self.can_channel,
                    baudrate=self.baudrate
                )
                
                # 应用MIT范围参数
                mit_config = motor_config.get('mit', {})
                if mit_config:
                    motor.mit_pos_upper = mit_config.get('pos_upper', 3.14159)
                    motor.mit_pos_lower = mit_config.get('pos_lower', -3.14159)
                    motor.mit_vel_upper = mit_config.get('vel_upper', 30.0)
                    motor.mit_vel_lower = mit_config.get('vel_lower', -30.0)
                    motor.mit_tor_upper = mit_config.get('tor_upper', 10.0)
                    motor.mit_tor_lower = mit_config.get('tor_lower', -10.0)
                    motor.mit_kp_range = mit_config.get('kp_range', 500.0)
                    motor.mit_kd_range = mit_config.get('kd_range', 5.0)
                
                self.motors[motor_id] = motor
                self.motor_names[motor_id] = motor_name
                
            except Exception as e:
                print(f"初始化电机 {motor_id} ({motor_name}) 失败: {e}")
    
    def get_motor(self, motor_id: int) -> Optional[LansiMotorController]:
        """获取指定ID的电机控制器"""
        return self.motors.get(motor_id)
    
    def get_motor_ids(self) -> List[int]:
        """获取所有电机ID"""
        return list(self.motors.keys())
    
    # ========== 批量控制命令 ==========
    
    def enable_all(self):
        """使能所有电机"""
        print("使能所有电机...")
        for motor_id, motor in self.motors.items():
            motor.enable_motor(True)
            time.sleep(0.01)
    
    def disable_all(self):
        """关闭所有电机"""
        print("关闭所有电机...")
        for motor_id, motor in self.motors.items():
            motor.disable_motor()
            time.sleep(0.01)
    
    def set_all_mode(self, mode: int):
        """设置所有电机的控制模式"""
        mode_names = {1: "力矩", 2: "MIT", 3: "速度", 4: "轨迹速度", 5: "位置", 6: "轨迹位置"}
        print(f"设置所有电机为{mode_names.get(mode, f'模式{mode}')}...")
        for motor_id, motor in self.motors.items():
            motor.set_mode(mode)
            time.sleep(0.01)
    
    def clear_all_alarms(self):
        """清除所有电机报警"""
        print("清除所有电机报警...")
        for motor_id, motor in self.motors.items():
            motor.clear_alarm()
            time.sleep(0.01)
    
    def read_all_status(self):
        """读取所有电机状态"""
        for motor_id, motor in self.motors.items():
            name = self.motor_names.get(motor_id, f"电机{motor_id}")
            print(f"\n--- {name} (ID={motor_id}) ---")
            motor.read_all_status()
            time.sleep(0.2)
    
    def set_all_home(self):
        """将所有电机当前位置设为原点"""
        print("设置所有电机原点...")
        for motor_id, motor in self.motors.items():
            motor.set_home()
            time.sleep(0.01)
    
    # ========== 位置控制 ==========
    
    def set_positions(self, positions: Dict[int, float]):
        """
        设置多个电机的位置
        
        参数:
            positions: {motor_id: position, ...}
        """
        for motor_id, position in positions.items():
            if motor_id in self.motors:
                self.motors[motor_id].set_position(position)
                time.sleep(0.005)
    
    def set_joint_positions(self, positions: List[float]):
        """
        按顺序设置所有关节位置
        
        参数:
            positions: 位置列表，按电机ID顺序
        """
        motor_ids = sorted(self.motors.keys())
        for i, pos in enumerate(positions):
            if i < len(motor_ids):
                self.motors[motor_ids[i]].set_position(pos)
                time.sleep(0.005)
    
    def get_joint_positions(self) -> Dict[int, float]:
        """获取所有关节当前位置"""
        positions = {}
        for motor_id, motor in self.motors.items():
            positions[motor_id] = motor.current_position
        return positions
    
    # ========== 速度控制 ==========
    
    def set_velocities(self, velocities: Dict[int, float]):
        """
        设置多个电机的速度
        
        参数:
            velocities: {motor_id: velocity, ...}
        """
        for motor_id, velocity in velocities.items():
            if motor_id in self.motors:
                self.motors[motor_id].set_velocity(velocity)
                time.sleep(0.005)
    
    def stop_all(self):
        """停止所有电机运动"""
        print("停止所有电机...")
        for motor_id, motor in self.motors.items():
            motor.set_velocity(0.0)
            time.sleep(0.005)
    
    # ========== MIT控制 ==========
    
    def send_mit_commands(self, commands: Dict[int, Dict]):
        """
        发送MIT控制命令到多个电机
        
        参数:
            commands: {motor_id: {'pos': float, 'vel': float, 'kp': float, 'kd': float, 'tor': float}, ...}
        """
        for motor_id, cmd in commands.items():
            if motor_id in self.motors:
                self.motors[motor_id].send_mit_command(
                    position=cmd.get('pos', 0.0),
                    velocity=cmd.get('vel', 0.0),
                    kp=cmd.get('kp', 0.0),
                    kd=cmd.get('kd', 0.0),
                    torque=cmd.get('tor', 0.0)
                )
    
    def mit_position_control(self, positions: Dict[int, float], kp: float = 100.0, kd: float = 2.0):
        """
        使用MIT模式进行位置控制
        
        参数:
            positions: {motor_id: position, ...}
            kp: 位置增益
            kd: 阻尼增益
        """
        commands = {}
        for motor_id, pos in positions.items():
            commands[motor_id] = {
                'pos': pos,
                'vel': 0.0,
                'kp': kp,
                'kd': kd,
                'tor': 0.0
            }
        self.send_mit_commands(commands)
    
    # ========== 轨迹控制 ==========
    
    def move_to_positions(self, target_positions: Dict[int, float], duration: float = 2.0, steps: int = 100):
        """
        平滑移动到目标位置（插值运动）
        
        参数:
            target_positions: {motor_id: target_position, ...}
            duration: 运动时间 (秒)
            steps: 插值步数
        """
        # 获取当前位置
        current_positions = self.get_joint_positions()
        
        # 计算步长
        step_time = duration / steps
        
        print(f"开始平滑运动，时长 {duration}s...")
        
        for step in range(steps + 1):
            # 计算插值系数 (使用S曲线)
            t = step / steps
            # S曲线插值: 3t^2 - 2t^3
            alpha = 3 * t * t - 2 * t * t * t
            
            # 计算中间位置
            positions = {}
            for motor_id in target_positions:
                start_pos = current_positions.get(motor_id, 0.0)
                end_pos = target_positions[motor_id]
                positions[motor_id] = start_pos + alpha * (end_pos - start_pos)
            
            # 发送位置
            self.set_positions(positions)
            
            time.sleep(step_time)
        
        print("运动完成")
    
    def execute_trajectory(self, trajectory: List[Dict[int, float]], time_per_point: float = 0.5):
        """
        执行轨迹
        
        参数:
            trajectory: 轨迹点列表，每个点是 {motor_id: position, ...}
            time_per_point: 每个点的时间间隔
        """
        print(f"执行轨迹，共 {len(trajectory)} 个点...")
        
        for i, point in enumerate(trajectory):
            print(f"  点 {i+1}/{len(trajectory)}")
            self.set_positions(point)
            time.sleep(time_per_point)
        
        print("轨迹执行完成")
    
    # ========== 预定义动作 ==========
    
    def go_home(self):
        """回到零位"""
        print("回到零位...")
        positions = {motor_id: 0.0 for motor_id in self.motors.keys()}
        self.set_positions(positions)
    
    def wave(self, joint_id: int = 1, amplitude: float = 0.5, cycles: int = 3):
        """
        单关节摆动演示
        
        参数:
            joint_id: 关节ID
            amplitude: 摆动幅度 (rad)
            cycles: 摆动次数
        """
        if joint_id not in self.motors:
            print(f"错误: 关节 {joint_id} 不存在")
            return
        
        print(f"关节 {joint_id} 摆动演示...")
        
        for i in range(cycles):
            self.motors[joint_id].set_position(amplitude)
            time.sleep(0.5)
            self.motors[joint_id].set_position(-amplitude)
            time.sleep(0.5)
        
        self.motors[joint_id].set_position(0.0)
        print("演示完成")
    
    # ========== 关闭 ==========
    
    def close(self):
        """关闭所有连接"""
        print("关闭机械臂控制器...")
        self.disable_all()
        time.sleep(0.1)
        
        for motor_id, motor in self.motors.items():
            try:
                motor.close()
            except:
                pass
        
        self.motors.clear()
        print("机械臂控制器已关闭")


def demo():
    """演示程序"""
    print("=" * 60)
    print("蓝思机械臂控制器演示")
    print("=" * 60)
    
    # 创建控制器
    arm = LansiArmController(config_file="config.yaml")
    
    try:
        time.sleep(0.5)
        
        # 清除报警
        print("\n--- 1. 清除报警 ---")
        arm.clear_all_alarms()
        time.sleep(0.3)
        
        # 使能电机
        print("\n--- 2. 使能电机 ---")
        arm.enable_all()
        time.sleep(0.3)
        
        # 设置位置模式
        print("\n--- 3. 设置位置模式 ---")
        arm.set_all_mode(LansiMotorController.MODE_POSITION)
        time.sleep(0.3)
        
        # 读取状态
        print("\n--- 4. 读取状态 ---")
        arm.read_all_status()
        time.sleep(0.5)
        
        # 回到零位
        print("\n--- 5. 回到零位 ---")
        arm.go_home()
        time.sleep(2)
        
        # 平滑运动示例
        print("\n--- 6. 平滑运动示例 ---")
        motor_ids = arm.get_motor_ids()
        if motor_ids:
            target = {motor_ids[0]: 1.0}  # 第一个电机移动到1.0 rad
            arm.move_to_positions(target, duration=2.0)
            time.sleep(1)
            
            target = {motor_ids[0]: 0.0}
            arm.move_to_positions(target, duration=2.0)
        
        # MIT控制示例
        print("\n--- 7. MIT控制示例 ---")
        arm.set_all_mode(LansiMotorController.MODE_MIT)
        time.sleep(0.3)
        
        if motor_ids:
            arm.mit_position_control({motor_ids[0]: 0.5}, kp=100, kd=2)
            time.sleep(2)
            arm.mit_position_control({motor_ids[0]: 0.0}, kp=100, kd=2)
            time.sleep(2)
        
        print("\n演示完成!")
        
    except KeyboardInterrupt:
        print("\n用户中断")
    finally:
        arm.close()


if __name__ == "__main__":
    demo()
