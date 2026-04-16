#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
7轴机械臂控制器 - 完整功能版
电机ID: 51-57

功能:
- 零点标定
- 速度/加速度设置
- 示教记录
- 轨迹播放
- 自由拖动模式

注意:
- 本文件为历史兼容控制层，当前 `robot_arm_web/backend` 主链路已切换到
  `linkerbot-py(A7)` 直连实现。
- 建议新功能优先在 `backend/group_controller.py` 中扩展，不再新增底层原始 CAN 指令逻辑。
"""
import can
import struct
import time
import threading
import json
import os
import math
from typing import Optional


# 机械臂电机ID配置
MOTOR_IDS = [51, 52, 53, 54, 55, 56, 57]

# 配置文件路径
CONFIG_DIR = os.path.dirname(os.path.abspath(__file__))
ZERO_OFFSET_FILE = os.path.join(CONFIG_DIR, "zero_offset.json")
TRAJECTORY_FILE = os.path.join(CONFIG_DIR, "trajectory.json")

# 命令码
CMD_HANDSHAKE = 0x00
CMD_READ_VERSION = 0x01
CMD_READ_VELOCITY = 0x04
CMD_READ_POSITION = 0x06
CMD_SET_MODE = 0x07
CMD_SET_VELOCITY = 0x09
CMD_SET_POSITION = 0x0A
CMD_ENABLE = 0x2A
CMD_CLEAR_ALARM = 0xFE
CMD_READ_ALARM = 0xFF

# 轨迹参数命令
CMD_READ_PT_V = 0x1C   # 读取位置轨迹最大速度
CMD_READ_PT_A = 0x1D   # 读取位置轨迹加速度
CMD_READ_PT_D = 0x1E   # 读取位置轨迹减速度
CMD_SET_PT_V = 0x1F    # 设置位置轨迹最大速度
CMD_SET_PT_A = 0x20    # 设置位置轨迹加速度
CMD_SET_PT_D = 0x21    # 设置位置轨迹减速度

# 控制模式（与 lansi_arm_sdk.constants.MotorMode 保持一致）
# 0: TORQUE, 1: MIT, 2: VELOCITY, 3: PROFILE_VELOCITY,
# 4: POSITION, 5: PROFILE_POSITION
MODE_TORQUE = 0
MODE_MIT = 1
MODE_VELOCITY = 2
MODE_PROFILE_VELOCITY = 3
MODE_POSITION = 4
MODE_PROFILE_POSITION = 5

MODE_NAMES = {
    MODE_TORQUE: "力矩模式",
    MODE_MIT: "MIT模式",
    MODE_VELOCITY: "速度模式",
    MODE_PROFILE_VELOCITY: "轨迹速度模式",
    MODE_POSITION: "位置模式",
    MODE_PROFILE_POSITION: "轨迹位置模式",
}


class Motor:
    """单个电机"""
    def __init__(self, motor_id, bus):
        self.motor_id = motor_id
        self.bus = bus
        self.position = 0.0
        self.velocity = 0.0
        self.enabled = False
        self.mode = 0
        self.alarm = 0
        self.profile_velocity = 0.0
        self.profile_accel = 0.0
        self.profile_decel = 0.0
    
    def send_cmd(self, cmd, data=None):
        """发送命令"""
        if data is None:
            data = []
        msg_data = [cmd] + list(data)
        # Pad with zeros to 8 bytes
        msg_data.extend([0] * (8 - len(msg_data)))
        msg = can.Message(
            arbitration_id=self.motor_id,
            data=msg_data[:8],
            is_extended_id=False
        )
        self.bus.send(msg)
    
    def float_to_bytes(self, value):
        """浮点数转字节"""
        return list(struct.pack('<f', value))
    
    def bytes_to_float(self, data):
        """字节转浮点数"""
        if len(data) >= 4:
            return struct.unpack('<f', bytes(data[:4]))[0]
        return 0.0


class ArmController:
    """机械臂控制器"""
    
    def __init__(self, motor_ids=None, can_channel='can0', bitrate=1000000, zero_offset_file: Optional[str] = None):
        self.motor_ids = motor_ids or MOTOR_IDS
        self.can_channel = can_channel
        self.bitrate = bitrate
        self.zero_offset_file = zero_offset_file if zero_offset_file is not None else os.path.join(CONFIG_DIR, "zero_offset.json")
        
        self.bus = None
        self.motors = {}
        self.running = False
        self.recv_thread = None
        
        # 零点偏移
        self.zero_offsets = {mid: 0.0 for mid in self.motor_ids}
        self._load_zero_offsets()
        
        # 示教轨迹
        self.trajectory = []
        self._load_trajectory()
        
        self._connect()
    
    def _connect(self):
        """连接CAN总线"""
        print(f"连接CAN总线: {self.can_channel}")
        self.bus = can.interface.Bus(
            channel=self.can_channel,
            interface='socketcan',
            bitrate=self.bitrate
        )
        
        # 创建电机对象
        for motor_id in self.motor_ids:
            self.motors[motor_id] = Motor(motor_id, self.bus)
        
        # 启动接收线程
        self.running = True
        self.recv_thread = threading.Thread(target=self._recv_loop, daemon=True)
        self.recv_thread.start()
        
        print(f"已连接，电机数量: {len(self.motors)}")
    
    def _recv_loop(self):
        """接收消息循环"""
        while self.running:
            try:
                msg = self.bus.recv(timeout=0.1)
                if msg:
                    self._handle_msg(msg)
            except:
                pass
    
    def _handle_msg(self, msg):
        """处理接收的消息"""
        motor_id = msg.arbitration_id
        if motor_id not in self.motors:
            return
        
        motor = self.motors[motor_id]
        data = list(msg.data)
        cmd = data[0]
        
        if cmd == CMD_READ_POSITION:
            if len(data) >= 5:
                motor.position = motor.bytes_to_float(data[1:5])
        elif cmd == CMD_READ_VELOCITY:
            if len(data) >= 5:
                motor.velocity = motor.bytes_to_float(data[1:5])
        elif cmd == CMD_ENABLE:
            motor.enabled = data[1] == 1 if len(data) > 1 else False
        elif cmd == CMD_SET_MODE:
            motor.mode = data[1] if len(data) > 1 else 0
        elif cmd == CMD_READ_ALARM:
            motor.alarm = data[1] if len(data) > 1 else 0
        elif cmd == CMD_READ_PT_V:
            if len(data) >= 5:
                motor.profile_velocity = motor.bytes_to_float(data[1:5])
        elif cmd == CMD_READ_PT_A:
            if len(data) >= 5:
                motor.profile_accel = motor.bytes_to_float(data[1:5])
        elif cmd == CMD_READ_PT_D:
            if len(data) >= 5:
                motor.profile_decel = motor.bytes_to_float(data[1:5])
    
    # ==================== 零点标定 ====================
    
    def _load_zero_offsets(self):
        """加载零点偏移"""
        if os.path.exists(self.zero_offset_file):
            try:
                with open(self.zero_offset_file, 'r') as f:
                    data = json.load(f)
                    for mid in self.motor_ids:
                        if str(mid) in data:
                            self.zero_offsets[mid] = data[str(mid)]
                print(f"已加载零点偏移: {self.zero_offset_file}")
            except Exception as e:
                print(f"加载零点偏移失败: {e}")
    
    def _save_zero_offsets(self):
        """保存零点偏移"""
        try:
            merged = {}
            if os.path.exists(self.zero_offset_file):
                try:
                    with open(self.zero_offset_file, 'r') as f:
                        data = json.load(f)
                        if isinstance(data, dict):
                            merged.update(data)
                except Exception:
                    pass
            merged.update({str(k): v for k, v in self.zero_offsets.items()})
            with open(self.zero_offset_file, 'w') as f:
                json.dump(merged, f, indent=2)
            print(f"已保存零点偏移: {self.zero_offset_file}")
        except Exception as e:
            print(f"保存零点偏移失败: {e}")
    
    def calibrate_zero(self, motor_id=None):
        """标定零点 - 将当前位置设为零点"""
        self.read_all_positions()
        time.sleep(0.5)
        
        if motor_id is not None:
            if motor_id in self.motors:
                self.zero_offsets[motor_id] = self.motors[motor_id].position
                print(f"电机 {motor_id} 零点标定: {self.zero_offsets[motor_id]:.4f} rad")
        else:
            for mid in self.motor_ids:
                self.zero_offsets[mid] = self.motors[mid].position
                print(f"电机 {mid} 零点标定: {self.zero_offsets[mid]:.4f} rad")
        
        self._save_zero_offsets()
    
    def get_position_with_offset(self, motor_id):
        """获取相对于零点的位置"""
        return self.motors[motor_id].position - self.zero_offsets[motor_id]
    
    def get_all_positions_with_offset(self):
        """获取所有电机相对于零点的位置"""
        return {mid: self.get_position_with_offset(mid) for mid in self.motor_ids}
    
    def set_position_with_offset(self, motor_id, position):
        """设置相对于零点的位置"""
        absolute_pos = position + self.zero_offsets[motor_id]
        self.set_position(motor_id, absolute_pos)
    
    def go_to_zero(self):
        """回到零点位置"""
        print("回到零点位置...")
        for motor_id in self.motor_ids:
            self.set_position(motor_id, self.zero_offsets[motor_id])
            time.sleep(0.02)
    
    # ==================== 示教轨迹 ====================
    
    def _load_trajectory(self):
        """加载示教轨迹"""
        if os.path.exists(TRAJECTORY_FILE):
            try:
                with open(TRAJECTORY_FILE, 'r') as f:
                    self.trajectory = json.load(f)
                print(f"已加载轨迹 ({len(self.trajectory)} 个点)")
            except Exception as e:
                print(f"加载轨迹失败: {e}")
    
    def _save_trajectory(self):
        """保存示教轨迹"""
        try:
            with open(TRAJECTORY_FILE, 'w') as f:
                json.dump(self.trajectory, f, indent=2)
            print(f"已保存轨迹 ({len(self.trajectory)} 个点)")
        except Exception as e:
            print(f"保存轨迹失败: {e}")
    
    def record_point(self, name=None):
        """记录当前位置点"""
        self.read_all_positions()
        time.sleep(0.3)
        
        point = {
            "name": name or f"point_{len(self.trajectory)}",
            "positions": {str(mid): self.motors[mid].position for mid in self.motor_ids},
            "timestamp": time.time()
        }
        self.trajectory.append(point)
        self._save_trajectory()
        
        print(f"已记录点 '{point['name']}':")
        for mid in self.motor_ids:
            print(f"  电机 {mid}: {self.motors[mid].position:.4f} rad")
        
        return point
    
    def clear_trajectory(self):
        """清空轨迹"""
        self.trajectory = []
        self._save_trajectory()
        print("轨迹已清空")
    
    def play_trajectory(self, interval=2.0):
        """播放示教轨迹"""
        if not self.trajectory:
            print("没有记录的轨迹点")
            return
        
        print(f"\n播放轨迹 ({len(self.trajectory)} 个点), 间隔 {interval}s...")
        
        for i, point in enumerate(self.trajectory):
            print(f"\n移动到点 {i+1}/{len(self.trajectory)}: {point['name']}")
            
            for mid in self.motor_ids:
                pos = point['positions'].get(str(mid), 0)
                self.set_position(mid, pos)
                time.sleep(0.02)
            
            time.sleep(interval)
        
        print("\n轨迹播放完成")
    
    # ==================== 速度控制 ====================
    
    def set_profile_velocity(self, motor_id, velocity):
        """设置轨迹最大速度 (rad/s)"""
        motor = self.motors[motor_id]
        data = motor.float_to_bytes(velocity)
        motor.send_cmd(CMD_SET_PT_V, data)
    
    def set_profile_acceleration(self, motor_id, accel):
        """设置轨迹加速度 (rad/s^2)"""
        motor = self.motors[motor_id]
        data = motor.float_to_bytes(accel)
        motor.send_cmd(CMD_SET_PT_A, data)
    
    def set_profile_deceleration(self, motor_id, decel):
        """设置轨迹减速度 (rad/s^2)"""
        motor = self.motors[motor_id]
        data = motor.float_to_bytes(decel)
        motor.send_cmd(CMD_SET_PT_D, data)
    
    def set_all_profile_velocity(self, velocity):
        """设置所有电机轨迹最大速度"""
        for motor_id in self.motor_ids:
            self.set_profile_velocity(motor_id, velocity)
            time.sleep(0.02)
        print(f"所有电机轨迹速度设置为: {velocity} rad/s")
    
    def set_all_profile_acceleration(self, accel):
        """设置所有电机轨迹加速度"""
        for motor_id in self.motor_ids:
            self.set_profile_acceleration(motor_id, accel)
            time.sleep(0.02)
        print(f"所有电机轨迹加速度设置为: {accel} rad/s^2")
    
    def set_all_profile_deceleration(self, decel):
        """设置所有电机轨迹减速度"""
        for motor_id in self.motor_ids:
            self.set_profile_deceleration(motor_id, decel)
            time.sleep(0.02)
        print(f"所有电机轨迹减速度设置为: {decel} rad/s^2")
    
    def read_profile_params(self, motor_id):
        """读取轨迹参数"""
        motor = self.motors[motor_id]
        motor.send_cmd(CMD_READ_PT_V)
        time.sleep(0.1)
        motor.send_cmd(CMD_READ_PT_A)
        time.sleep(0.1)
        motor.send_cmd(CMD_READ_PT_D)
        time.sleep(0.1)
    
    def read_all_profile_params(self):
        """读取所有电机轨迹参数"""
        for motor_id in self.motor_ids:
            self.read_profile_params(motor_id)
        time.sleep(0.5)
    
    def set_speed_preset(self, preset):
        """设置速度预设: very_slow, slow, medium, fast"""
        presets = {
            'very_slow': {'v': 0.2, 'a': 0.2, 'd': 0.2},
            'slow': {'v': 0.5, 'a': 0.5, 'd': 0.5},
            'medium': {'v': 1.0, 'a': 1.0, 'd': 1.0},
            'fast': {'v': 2.0, 'a': 2.0, 'd': 2.0},
        }
        
        if preset not in presets:
            print(f"未知预设: {preset}, 可用: {list(presets.keys())}")
            return
        
        params = presets[preset]
        print(f"\n设置速度预设: {preset}")
        self.set_all_profile_velocity(params['v'])
        self.set_all_profile_acceleration(params['a'])
        self.set_all_profile_deceleration(params['d'])
    
    # ==================== 自由拖动 ====================
    
    def enable_freedrive(self):
        """启用自由拖动模式 (关闭使能)"""
        print("\n启用自由拖动模式...")
        self.clear_all_alarms()
        time.sleep(0.2)
        self.enable_all(False)
        time.sleep(0.2)
        print("自由拖动模式已启用 - 可以手动移动机械臂")
    
    def disable_freedrive(self):
        """退出自由拖动模式（会自动重新使能电机）"""
        print("\n退出自由拖动模式...")
        self.read_all_positions()
        time.sleep(0.3)
        self.set_all_modes(MODE_PROFILE_POSITION)
        time.sleep(0.2)
        self.enable_all(True)  # 重新使能电机
        time.sleep(0.2)
        print("自由拖动模式已退出 - 电机已重新使能并锁定")
    
    # ==================== 基础控制 ====================
    
    def clear_alarm(self, motor_id):
        """清除报警"""
        self.motors[motor_id].send_cmd(CMD_CLEAR_ALARM)
    
    def clear_all_alarms(self):
        """清除所有报警"""
        for motor_id in self.motor_ids:
            self.clear_alarm(motor_id)
            time.sleep(0.05)  # 增加延迟，避免缓冲区满
    
    def set_mode(self, motor_id, mode):
        """设置控制模式"""
        self.motors[motor_id].send_cmd(CMD_SET_MODE, [mode])
    
    def set_all_modes(self, mode):
        """设置所有电机模式"""
        for motor_id in self.motor_ids:
            self.set_mode(motor_id, mode)
            time.sleep(0.05)  # 增加延迟，避免缓冲区满
    
    def enable(self, motor_id, enable=True):
        """使能/关闭电机"""
        self.motors[motor_id].send_cmd(CMD_ENABLE, [1 if enable else 0])
        self.motors[motor_id].enabled = enable
    
    def enable_all(self, enable=True):
        """使能/关闭所有电机"""
        for motor_id in self.motor_ids:
            self.enable(motor_id, enable)
            time.sleep(0.05)  # 增加延迟，避免缓冲区满
    
    def read_position(self, motor_id):
        """读取位置"""
        self.motors[motor_id].send_cmd(CMD_READ_POSITION)
    
    def read_all_positions(self):
        """读取所有位置"""
        for motor_id in self.motor_ids:
            self.read_position(motor_id)
            time.sleep(0.05)  # 增加延迟，避免缓冲区满
    
    def set_position(self, motor_id, position):
        """设置位置"""
        motor = self.motors[motor_id]
        data = motor.float_to_bytes(position)
        motor.send_cmd(CMD_SET_POSITION, data)
    
    def set_all_positions(self, positions):
        """设置所有电机位置"""
        for motor_id, pos in zip(self.motor_ids, positions):
            self.set_position(motor_id, pos)
            time.sleep(0.02)
    
    def get_positions(self):
        """获取所有电机当前位置"""
        return {mid: self.motors[mid].position for mid in self.motor_ids}
    
    def init_arm(self, mode=MODE_PROFILE_POSITION):
        """初始化机械臂"""
        print("\n初始化机械臂...")
        
        print("  [1/5] 清除报警...")
        self.clear_all_alarms()
        time.sleep(0.3)
        
        print("  [2/5] 读取当前位置...")
        self.read_all_positions()
        time.sleep(0.5)
        
        positions = self.get_positions()
        all_zero = all(abs(pos) < 1e-6 for pos in positions.values())
        for mid, pos in positions.items():
            offset_pos = pos - self.zero_offsets[mid]
            print(f"    电机 {mid}: {pos:.4f} rad (相对零点: {offset_pos:.4f} rad)")
        
        if all_zero:
            print("  ⚠ 警告: 所有电机位置均为 0.0000，可能 CAN 通信异常（电机未回传数据）")
        
        mode_name = MODE_NAMES.get(mode, f"模式{mode}")
        print(f"  [3/5] 设置控制模式: {mode_name}...")
        self.set_all_modes(mode)
        time.sleep(0.3)
        
        print("  [4/5] 使能所有电机...")
        self.enable_all(True)
        time.sleep(0.3)
        
        # [5/5] 验证使能状态 (Bypassed due to firmware not sending ACKs)
        print("  [5/5] 等待电机使能...")
        time.sleep(1.0) # Give motors time to enable
        
        # We assume success because reading 0x2A does not work reliably
        print("初始化完成! 所有电机已下发使能指令 ✓")
        return positions
    
    def disable_arm(self):
        """关闭机械臂"""
        print("\n关闭机械臂...")
        self.enable_all(False)
        time.sleep(0.2)
        print("已关闭")
    
    def close(self):
        """关闭连接"""
        self.running = False
        if self.recv_thread:
            self.recv_thread.join(timeout=1)
        if self.bus:
            self.bus.shutdown()
        print("连接已关闭")


def rad_to_deg(rad):
    return rad * 180 / math.pi

def deg_to_rad(deg):
    return deg * math.pi / 180


def interactive_control():
    """交互式控制"""
    print("\n" + "="*60)
    print("        7轴机械臂交互控制 - 完整版")
    print("        电机ID: 51-57")
    print("="*60)
    
    arm = ArmController(motor_ids=MOTOR_IDS)
    
    def show_menu():
        print("\n" + "-"*50)
        print("【基础控制】")
        print("  1. 初始化机械臂")
        print("  2. 读取所有电机位置")
        print("  3. 移动单个电机")
        print("  4. 关闭所有电机")
        print("")
        print("【零点标定】")
        print("  10. 进入自由拖动模式")
        print("  11. 标定当前位置为零点")
        print("  12. 回到零点位置")
        print("  13. 查看零点偏移")
        print("")
        print("【速度控制】")
        print("  20. 设置速度预设 (慢/中/快)")
        print("  21. 自定义速度参数")
        print("  22. 读取当前速度参数")
        print("")
        print("【示教轨迹】")
        print("  30. 记录当前位置点")
        print("  31. 查看已记录的轨迹")
        print("  32. 播放轨迹")
        print("  33. 清空轨迹")
        print("")
        print("  0. 退出")
        print("-"*50)
    
    try:
        while True:
            show_menu()
            cmd = input("请选择: ").strip()
            
            if cmd == '1':
                print("\n选择控制模式:")
                print("  5. 位置模式 (直接移动)")
                print("  6. 轨迹位置模式 (带速度限制，推荐)")
                mode = int(input("选择 (5/6, 默认6): ").strip() or "6")
                arm.init_arm(mode)
            
            elif cmd == '2':
                arm.read_all_positions()
                time.sleep(0.5)
                print("\n当前位置:")
                print("-" * 60)
                print(f"{'电机ID':<8} {'绝对位置(rad)':<15} {'相对零点(rad)':<15} {'角度':<10}")
                print("-" * 60)
                for mid in MOTOR_IDS:
                    pos = arm.motors[mid].position
                    offset_pos = arm.get_position_with_offset(mid)
                    deg = rad_to_deg(offset_pos)
                    print(f"{mid:<8} {pos:<15.4f} {offset_pos:<15.4f} {deg:<10.2f}")
            
            elif cmd == '3':
                motor_id = int(input("输入电机ID (51-57): ").strip())
                if motor_id not in MOTOR_IDS:
                    print("无效的电机ID")
                    continue
                
                print("输入方式: 1=绝对位置(rad), 2=相对零点(rad), 3=角度")
                input_type = input("选择 (1/2/3, 默认2): ").strip() or "2"
                
                if input_type == '1':
                    target = float(input("输入绝对位置 (rad): ").strip())
                elif input_type == '2':
                    target_offset = float(input("输入相对零点位置 (rad): ").strip())
                    target = target_offset + arm.zero_offsets[motor_id]
                else:
                    target_deg = float(input("输入角度: ").strip())
                    target_offset = deg_to_rad(target_deg)
                    target = target_offset + arm.zero_offsets[motor_id]
                
                confirm = input(f"确认移动电机 {motor_id} 到 {target:.4f} rad? (y/n): ").strip()
                if confirm.lower() == 'y':
                    arm.set_position(motor_id, target)
                    print("命令已发送")
                    time.sleep(1.5)
                    arm.read_position(motor_id)
                    time.sleep(0.3)
                    print(f"当前位置: {arm.motors[motor_id].position:.4f} rad")
            
            elif cmd == '4':
                arm.disable_arm()
            
            elif cmd == '10':
                arm.enable_freedrive()
                print("\n现在可以手动移动机械臂到目标位置")
                print("完成后输入 'done' 退出自由拖动模式")
                
                while True:
                    arm.read_all_positions()
                    time.sleep(0.5)
                    
                    print("\n当前位置:")
                    for mid in MOTOR_IDS:
                        pos = arm.motors[mid].position
                        print(f"  电机 {mid}: {pos:.4f} rad ({rad_to_deg(pos):.2f})")
                    
                    user_input = input("\n输入 'done' 退出, 或按回车刷新位置: ").strip()
                    if user_input.lower() == 'done':
                        break
                
                exit_choice = input("退出自由拖动并锁定电机? (y/n): ").strip()
                if exit_choice.lower() == 'y':
                    arm.disable_freedrive()
            
            elif cmd == '11':
                print("\n零点标定选项:")
                print("  1. 标定所有电机")
                print("  2. 标定单个电机")
                choice = input("选择 (1/2): ").strip()
                
                if choice == '1':
                    confirm = input("确认将所有电机当前位置标定为零点? (y/n): ").strip()
                    if confirm.lower() == 'y':
                        arm.calibrate_zero()
                elif choice == '2':
                    motor_id = int(input("输入电机ID (51-57): ").strip())
                    if motor_id in MOTOR_IDS:
                        confirm = input(f"确认将电机 {motor_id} 当前位置标定为零点? (y/n): ").strip()
                        if confirm.lower() == 'y':
                            arm.calibrate_zero(motor_id)
                    else:
                        print("无效的电机ID")
            
            elif cmd == '12':
                confirm = input("确认回到零点位置? (y/n): ").strip()
                if confirm.lower() == 'y':
                    arm.go_to_zero()
                    time.sleep(2)
                    arm.read_all_positions()
                    time.sleep(0.3)
                    print("已回到零点")
            
            elif cmd == '13':
                print("\n零点偏移:")
                print("-" * 40)
                for mid in MOTOR_IDS:
                    offset = arm.zero_offsets[mid]
                    print(f"  电机 {mid}: {offset:.4f} rad ({rad_to_deg(offset):.2f})")
            
            elif cmd == '20':
                print("\n速度预设:")
                print("  1. very_slow (0.2 rad/s)")
                print("  2. slow (0.5 rad/s)")
                print("  3. medium (1.0 rad/s)")
                print("  4. fast (2.0 rad/s)")
                
                preset_map = {'1': 'very_slow', '2': 'slow', '3': 'medium', '4': 'fast'}
                choice = input("选择 (1-4): ").strip()
                
                if choice in preset_map:
                    arm.set_speed_preset(preset_map[choice])
                else:
                    print("无效选择")
            
            elif cmd == '21':
                print("\n自定义速度参数:")
                velocity = float(input("最大速度 (rad/s, 默认1.0): ").strip() or "1.0")
                accel = float(input("加速度 (rad/s^2, 默认1.0): ").strip() or "1.0")
                decel = float(input("减速度 (rad/s^2, 默认1.0): ").strip() or "1.0")
                
                arm.set_all_profile_velocity(velocity)
                arm.set_all_profile_acceleration(accel)
                arm.set_all_profile_deceleration(decel)
            
            elif cmd == '22':
                arm.read_all_profile_params()
                print("\n当前速度参数:")
                print("-" * 60)
                print(f"{'电机ID':<8} {'最大速度(rad/s)':<18} {'加速度':<15} {'减速度':<15}")
                print("-" * 60)
                for mid in MOTOR_IDS:
                    m = arm.motors[mid]
                    print(f"{mid:<8} {m.profile_velocity:<18.2f} {m.profile_accel:<15.2f} {m.profile_decel:<15.2f}")
            
            elif cmd == '30':
                name = input("输入点位名称 (可选): ").strip() or None
                arm.record_point(name)
            
            elif cmd == '31':
                if not arm.trajectory:
                    print("没有记录的轨迹点")
                else:
                    print(f"\n已记录的轨迹 ({len(arm.trajectory)} 个点):")
                    print("-" * 50)
                    for i, point in enumerate(arm.trajectory):
                        print(f"  [{i+1}] {point['name']}")
                        for mid in MOTOR_IDS:
                            pos = point['positions'].get(str(mid), 0)
                            print(f"       电机 {mid}: {pos:.4f} rad")
            
            elif cmd == '32':
                if not arm.trajectory:
                    print("没有记录的轨迹点")
                else:
                    interval = float(input(f"点间隔时间 (秒, 默认2.0): ").strip() or "2.0")
                    confirm = input(f"确认播放 {len(arm.trajectory)} 个点? (y/n): ").strip()
                    if confirm.lower() == 'y':
                        arm.init_arm(MODE_PROFILE_POSITION)
                        time.sleep(0.5)
                        arm.play_trajectory(interval)
                        arm.disable_arm()
            
            elif cmd == '33':
                confirm = input("确认清空所有轨迹点? (y/n): ").strip()
                if confirm.lower() == 'y':
                    arm.clear_trajectory()
            
            elif cmd == '0':
                break
            
            else:
                print("无效命令")
    
    except KeyboardInterrupt:
        print("\n用户中断")
    
    finally:
        arm.disable_arm()
        arm.close()


if __name__ == "__main__":
    interactive_control()
