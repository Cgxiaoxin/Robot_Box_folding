#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
蓝思电机CAN通信控制器
基于蓝思电机文档V0.5协议实现
支持标准包和广播包MIT控制
"""
import sys
import os
import time
import threading
import struct
from typing import List, Dict, Optional, Tuple

import can
from can.exceptions import CanError


class LansiMotorController:
    """蓝思电机CAN通信控制器"""
    
    # ========== 控制模式枚举 ==========
    # 与 lansi_arm_sdk.constants.MotorMode 保持一致:
    # 0: TORQUE, 1: MIT, 2: VELOCITY, 3: PROFILE_VELOCITY,
    # 4: POSITION, 5: PROFILE_POSITION
    MODE_TORQUE = 0           # 力矩模式
    MODE_MIT = 1              # MIT模式
    MODE_VELOCITY = 2         # 速度模式
    MODE_PROFILE_VELOCITY = 3 # 轨迹速度模式
    MODE_POSITION = 4         # 位置模式
    MODE_PROFILE_POSITION = 5 # 轨迹位置模式
    
    # ========== 命令码定义 ==========
    # 基础命令
    CMD_HANDSHAKE = 0x00
    CMD_R_VERSION = 0x01
    CMD_R_ADDR = 0x02
    CMD_SAVE_PARA = 0x0D
    CMD_SET_ID = 0x3D
    CMD_GO_TO_BOOTLOADER = 0x97
    
    # 电机状态监控命令
    CMD_R_CURRENT = 0x04
    CMD_R_VELOCITY = 0x05
    CMD_R_POSITION = 0x06
    CMD_R_ON_OFF = 0x2B
    CMD_R_VOLTAGE = 0x45
    CMD_R_CURRENT_MODE = 0x55
    CMD_R_CORE_TEMP = 0x5E
    CMD_R_MOTOR_TEMP = 0x5F
    CMD_R_INVERTER_TEMP = 0x60
    CMD_R_CVP = 0x94  # 读取电流、速度、位置
    CMD_R_ALARM = 0xFF
    
    # 电机控制命令
    CMD_SET_MODE = 0x07
    CMD_SET_CURRENT = 0x08
    CMD_SET_VELOCITY = 0x09
    CMD_SET_POSITION = 0x0A
    CMD_MIT_CONTROL = 0x0B  # MIT控制命令（标准包）
    CMD_S_ON_OFF = 0x2A
    
    # 轨迹控制参数
    CMD_R_PT_V = 0x1C   # 读取位置轨迹最大速度
    CMD_R_PT_A = 0x1D   # 读取位置轨迹加速度
    CMD_R_PT_D = 0x1E   # 读取位置轨迹减速度
    CMD_SET_PT_V = 0x1F # 设置位置轨迹最大速度
    CMD_SET_PT_A = 0x20 # 设置位置轨迹加速度
    CMD_SET_PT_D = 0x21 # 设置位置轨迹减速度
    CMD_R_VT_V = 0x22   # 读取速度轨迹最大速度
    CMD_R_VT_A = 0x23   # 读取速度轨迹加速度
    CMD_R_VT_D = 0x24   # 读取速度轨迹减速度
    CMD_SET_VT_V = 0x25 # 设置速度轨迹最大速度
    CMD_SET_VT_A = 0x26 # 设置速度轨迹加速度
    CMD_SET_VT_D = 0x27 # 设置速度轨迹减速度
    
    # 电机参数命令
    CMD_SET_HOME = 0x87
    CMD_S_MOTOR_L = 0xC0
    CMD_R_MOTOR_L = 0xC1
    CMD_S_MOTOR_R = 0xC2
    CMD_R_MOTOR_R = 0xC3
    CMD_S_GEAR_RATIO = 0xC4
    CMD_R_GEAR_RATIO = 0xC5
    CMD_S_CALI_START = 0xC7
    
    # 报警处理命令
    CMD_CLEAR_ALARM = 0xFE
    
    # 广播包ID
    BROADCAST_ID_MIT = 0x00
    BROADCAST_ID_GENERAL = 0xFF
    
    def __init__(self, motor_id: int = 1, can_channel: str = 'can0', baudrate: int = 1000000):
        """
        初始化蓝思电机控制器
        
        参数:
            motor_id: 电机ID (1-128)
            can_channel: CAN接口名称 (如 'can0', 'can1')
            baudrate: CAN总线波特率 (默认1000000，即1Mbps)
        """
        self.motor_id = motor_id
        self.can_channel = can_channel
        self.baudrate = baudrate
        self.running = False
        self.receive_thread = None
        self.bus = None
        
        # MIT模式范围参数（默认值，可通过读取或设置修改）
        self.mit_pos_upper = 3.14159    # 位置上限 (rad)
        self.mit_pos_lower = -3.14159   # 位置下限 (rad)
        self.mit_vel_upper = 30.0       # 速度上限 (rad/s)
        self.mit_vel_lower = -30.0      # 速度下限 (rad/s)
        self.mit_tor_upper = 10.0       # 力矩上限 (Nm)
        self.mit_tor_lower = -10.0      # 力矩下限 (Nm)
        self.mit_kp_range = 500.0       # Kp范围 (0 - kp_range)
        self.mit_kd_range = 5.0         # Kd范围 (0 - kd_range)
        
        # 状态变量
        self.current_position = 0.0
        self.current_velocity = 0.0
        self.current_torque = 0.0
        self.current_current = 0.0
        self.bus_voltage = 0.0
        self.motor_temp = 0.0
        self.core_temp = 0.0
        self.inverter_temp = 0.0
        self.motor_enabled = False
        self.control_mode = 0
        self.motor_error = 0
        self.version_info = ""
        
        # 初始化CAN总线
        self.bus = self._init_can_bus()
        
        # 启动接收线程
        self.running = True
        self.receive_thread = threading.Thread(target=self._receive_messages, daemon=True)
        self.receive_thread.start()
        
        self._print_msg(f"蓝思电机控制器初始化完成 - 电机ID: {motor_id}, CAN通道: {can_channel}", "green")
    
    def _print_msg(self, msg: str, color: str = "white"):
        """打印消息"""
        colors = {
            "red": "\033[91m",
            "green": "\033[92m",
            "yellow": "\033[93m",
            "blue": "\033[94m",
            "cyan": "\033[96m",
            "white": "\033[97m",
            "reset": "\033[0m"
        }
        print(f"{colors.get(color, '')}{msg}{colors['reset']}")
    
    def _init_can_bus(self) -> can.interface.Bus:
        """初始化CAN总线连接"""
        try:
            bus = can.interface.Bus(
                channel=self.can_channel,
                interface="socketcan",
                bitrate=self.baudrate
            )
            self._print_msg(f"成功连接CAN总线: channel='{self.can_channel}', bitrate={self.baudrate}", "green")
            return bus
        except CanError as e:
            self._print_msg(f"CAN总线连接失败: {e}", "red")
            self._print_msg(f"请先配置CAN接口: sudo ip link set {self.can_channel} up type can bitrate {self.baudrate}", "yellow")
            raise
    
    # ========== 数据转换工具函数 ==========
    
    @staticmethod
    def float2uint(x: float, x_min: float, x_max: float, bits: int) -> int:
        """
        将float转换为无符号整数
        
        参数:
            x: 输入的float值
            x_min: 最小值
            x_max: 最大值
            bits: 位数
        返回:
            转换后的整数值
        """
        span = x_max - x_min
        offset = x_min
        x = max(x_min, min(x_max, x))  # 限制范围
        return int((x - offset) * ((1 << bits) - 1) / span)
    
    @staticmethod
    def uint2float(x_int: int, x_min: float, x_max: float, bits: int) -> float:
        """
        将无符号整数转换为float
        
        参数:
            x_int: 输入的整数值
            x_min: 最小值
            x_max: 最大值
            bits: 位数
        返回:
            转换后的float值
        """
        span = x_max - x_min
        offset = x_min
        return float(x_int) * span / ((1 << bits) - 1) + offset
    
    # ========== CAN通信底层函数 ==========
    
    def _send_can_message(self, can_id: int, data: List[int], is_extended: bool = False):
        """发送CAN消息"""
        # 确保数据长度为8字节
        while len(data) < 8:
            data.append(0)
        data = data[:8]
        
        msg = can.Message(
            arbitration_id=can_id,
            data=data,
            is_extended_id=is_extended
        )
        
        try:
            self.bus.send(msg)
        except CanError as e:
            self._print_msg(f"发送CAN消息失败: {e}", "red")
            raise
    
    def send_command(self, cmd: int, data: List[int] = None, log: bool = True):
        """
        发送标准命令包
        
        参数:
            cmd: 命令码 (0x00-0xFF)
            data: 数据列表，每个元素是一个字节 (0-255)，最多7个字节
            log: 是否打印日志
        """
        if data is None:
            data = []
        
        frame_data = [cmd] + list(data)
        self._send_can_message(self.motor_id, frame_data)
        
        if log:
            self._print_msg(f"发送命令: ID=0x{self.motor_id:02X}, CMD=0x{cmd:02X}, 数据={data}", "cyan")
    
    def send_float_command(self, cmd: int, value: float, log: bool = True):
        """发送包含float值的命令"""
        data_bytes = list(struct.pack('<f', value))
        self.send_command(cmd, data_bytes, log)
    
    def send_uint8_command(self, cmd: int, value: int, log: bool = True):
        """发送包含uint8_t值的命令"""
        self.send_command(cmd, [value & 0xFF], log)
    
    # ========== 基础命令 ==========
    
    def handshake(self):
        """握手命令 (0x00)"""
        self.send_command(self.CMD_HANDSHAKE, [])
        self._print_msg("发送握手命令", "green")
    
    def read_version(self):
        """读取版本信息 (0x01)"""
        self.send_command(self.CMD_R_VERSION, [])
    
    def read_address(self):
        """读取地址信息 (0x02)"""
        self.send_command(self.CMD_R_ADDR, [])
    
    def save_parameters(self):
        """保存参数到Flash (0x0D)"""
        self.send_command(self.CMD_SAVE_PARA, [])
        self._print_msg("参数保存命令已发送", "green")
    
    def set_motor_id(self, new_id: int):
        """设置设备ID (0x3D)，范围1-128"""
        if not (1 <= new_id <= 128):
            self._print_msg(f"错误: ID必须在1-128范围内", "red")
            return
        self.send_uint8_command(self.CMD_SET_ID, new_id)
        self._print_msg(f"设置电机ID为: {new_id}", "green")
    
    def enter_bootloader(self):
        """进入引导程序模式 (0x97)"""
        self.send_command(self.CMD_GO_TO_BOOTLOADER, [])
        self._print_msg("进入引导程序模式", "yellow")
    
    # ========== 电机控制命令 ==========
    
    def set_mode(self, mode: int):
        """
        设置控制模式 (0x07)
        
        参数:
            mode: 控制模式
                1 = MODE_TORQUE (力矩模式)
                2 = MODE_MIT (MIT模式)
                3 = MODE_VELOCITY (速度模式)
                4 = MODE_PROFILE_VELOCITY (轨迹速度模式)
                5 = MODE_POSITION (位置模式)
                6 = MODE_PROFILE_POSITION (轨迹位置模式)
        """
        mode_names = {
            1: "力矩模式",
            2: "MIT模式",
            3: "速度模式",
            4: "轨迹速度模式",
            5: "位置模式",
            6: "轨迹位置模式"
        }
        self.send_uint8_command(self.CMD_SET_MODE, mode)
        self._print_msg(f"设置控制模式: {mode_names.get(mode, f'未知模式({mode})')}", "green")
    
    def enable_motor(self, enable: bool = True):
        """设置电机使能 (0x2A)"""
        value = 0x01 if enable else 0x00
        self.send_uint8_command(self.CMD_S_ON_OFF, value)
        self.motor_enabled = enable
        self._print_msg(f"电机{'使能' if enable else '关闭'}", "green")
    
    def disable_motor(self):
        """关闭电机"""
        self.enable_motor(False)
    
    def set_current(self, current: float):
        """设置电流 (0x08)，单位: A"""
        self.send_float_command(self.CMD_SET_CURRENT, current)
        self._print_msg(f"设置电流: {current:.3f} A", "cyan")
    
    def set_velocity(self, velocity: float):
        """设置速度 (0x09)，单位: rad/s"""
        self.send_float_command(self.CMD_SET_VELOCITY, velocity)
        self._print_msg(f"设置速度: {velocity:.3f} rad/s", "cyan")
    
    def set_position(self, position: float):
        """设置位置 (0x0A)，单位: rad"""
        self.send_float_command(self.CMD_SET_POSITION, position)
        self._print_msg(f"设置位置: {position:.3f} rad", "cyan")
    
    def set_home(self):
        """设置当前位置为原点 (0x87)"""
        self.send_command(self.CMD_SET_HOME, [])
        self._print_msg("设置当前位置为原点", "green")
    
    # ========== MIT控制命令 (标准包) ==========
    
    def send_mit_command(self, position: float, velocity: float, kp: float, kd: float, torque: float):
        """
        发送MIT控制命令（标准包，CMD=0x0B）
        
        参数:
            position: 位置指令 (rad)
            velocity: 速度指令 (rad/s)
            kp: Kp参数
            kd: Kd参数
            torque: 转矩指令 (Nm)
        
        数据格式:
            Byte0: CMD (0x0B)
            Byte1-2: 位置 (16位)
            Byte3-4: 速度 (10位)
            Byte4-5: Kp (10位)
            Byte5-6: Kd (10位)
            Byte6-7: 转矩 (10位)
        """
        # 转换为整数
        p_int = self.float2uint(position, self.mit_pos_lower, self.mit_pos_upper, 16)
        v_int = self.float2uint(velocity, self.mit_vel_lower, self.mit_vel_upper, 10)
        kp_int = self.float2uint(kp, 0, self.mit_kp_range, 10)
        kd_int = self.float2uint(kd, 0, self.mit_kd_range, 10)
        t_int = self.float2uint(torque, self.mit_tor_lower, self.mit_tor_upper, 10)
        
        # 打包数据
        data = [0] * 7
        data[0] = (p_int >> 8) & 0xFF
        data[1] = p_int & 0xFF
        data[2] = (v_int >> 2) & 0xFF
        data[3] = ((v_int & 0x03) << 6) | ((kp_int >> 4) & 0x3F)
        data[4] = ((kp_int & 0x0F) << 4) | ((kd_int >> 6) & 0x0F)
        data[5] = ((kd_int & 0x3F) << 2) | ((t_int >> 8) & 0x03)
        data[6] = t_int & 0xFF
        
        self.send_command(self.CMD_MIT_CONTROL, data, log=False)
        self._print_msg(f"MIT控制: pos={position:.3f}, vel={velocity:.3f}, kp={kp:.1f}, kd={kd:.2f}, tor={torque:.3f}", "cyan")
    
    # ========== MIT广播包控制 ==========
    
    def send_mit_broadcast(self, motors_data: List[Dict]):
        """
        发送MIT控制广播包 (广播ID=0x00)
        
        参数:
            motors_data: 电机数据列表，每个元素是一个字典:
                {
                    'position': float,  # 位置指令 (rad)
                    'velocity': float,  # 速度指令 (rad/s)
                    'kp': float,        # Kp参数
                    'kd': float,        # Kd参数
                    'torque': float     # 转矩指令 (Nm)
                }
        
        注意: 广播包每个电机8字节，由于标准CAN帧只有8字节，
              这里每次只发送一个电机的数据到广播ID
        """
        for i, motor_data in enumerate(motors_data):
            if i >= 8:
                self._print_msg("警告: 广播包最多支持8个电机", "yellow")
                break
            
            pos = motor_data.get('position', 0.0)
            vel = motor_data.get('velocity', 0.0)
            kp = motor_data.get('kp', 0.0)
            kd = motor_data.get('kd', 0.0)
            tor = motor_data.get('torque', 0.0)
            
            # 转换为整数 (广播包格式: 16-12-12-12-12)
            p_int = self.float2uint(pos, self.mit_pos_lower, self.mit_pos_upper, 16)
            v_int = self.float2uint(vel, self.mit_vel_lower, self.mit_vel_upper, 12)
            kp_int = self.float2uint(kp, 0, self.mit_kp_range, 12)
            kd_int = self.float2uint(kd, 0, self.mit_kd_range, 12)
            t_int = self.float2uint(tor, self.mit_tor_lower, self.mit_tor_upper, 12)
            
            # 打包数据 (8字节)
            data = [0] * 8
            data[0] = (p_int >> 8) & 0xFF
            data[1] = p_int & 0xFF
            data[2] = (v_int >> 4) & 0xFF
            data[3] = ((v_int & 0x0F) << 4) | ((kp_int >> 8) & 0x0F)
            data[4] = kp_int & 0xFF
            data[5] = (kd_int >> 4) & 0xFF
            data[6] = ((kd_int & 0x0F) << 4) | ((t_int >> 8) & 0x0F)
            data[7] = t_int & 0xFF
            
            # 发送到广播ID (电机通过ID偏移识别自己的数据)
            self._send_can_message(self.BROADCAST_ID_MIT, data)
    
    # ========== 轨迹参数设置 ==========
    
    def set_position_trajectory_params(self, max_velocity: float, acceleration: float, deceleration: float):
        """设置位置轨迹参数"""
        self.send_float_command(self.CMD_SET_PT_V, max_velocity, log=False)
        time.sleep(0.005)
        self.send_float_command(self.CMD_SET_PT_A, acceleration, log=False)
        time.sleep(0.005)
        self.send_float_command(self.CMD_SET_PT_D, deceleration, log=False)
        self._print_msg(f"设置位置轨迹参数: V={max_velocity}, A={acceleration}, D={deceleration}", "green")
    
    def set_velocity_trajectory_params(self, max_velocity: float, acceleration: float, deceleration: float):
        """设置速度轨迹参数"""
        self.send_float_command(self.CMD_SET_VT_V, max_velocity, log=False)
        time.sleep(0.005)
        self.send_float_command(self.CMD_SET_VT_A, acceleration, log=False)
        time.sleep(0.005)
        self.send_float_command(self.CMD_SET_VT_D, deceleration, log=False)
        self._print_msg(f"设置速度轨迹参数: V={max_velocity}, A={acceleration}, D={deceleration}", "green")
    
    # ========== 状态读取命令 ==========
    
    def read_current(self):
        """读取当前电流 (0x04)"""
        self.send_command(self.CMD_R_CURRENT, [], log=False)
    
    def read_velocity(self):
        """读取当前速度 (0x05)"""
        self.send_command(self.CMD_R_VELOCITY, [], log=False)
    
    def read_position(self):
        """读取当前位置 (0x06)"""
        self.send_command(self.CMD_R_POSITION, [], log=False)
    
    def read_enable_status(self):
        """读取电机使能状态 (0x2B)"""
        self.send_command(self.CMD_R_ON_OFF, [], log=False)
    
    def read_voltage(self):
        """读取母线电压 (0x45)"""
        self.send_command(self.CMD_R_VOLTAGE, [], log=False)
    
    def read_control_mode(self):
        """读取控制模式 (0x55)"""
        self.send_command(self.CMD_R_CURRENT_MODE, [], log=False)
    
    def read_temperatures(self):
        """读取所有温度"""
        self.send_command(self.CMD_R_CORE_TEMP, [], log=False)
        time.sleep(0.01)
        self.send_command(self.CMD_R_MOTOR_TEMP, [], log=False)
        time.sleep(0.01)
        self.send_command(self.CMD_R_INVERTER_TEMP, [], log=False)
    
    def read_cvp(self):
        """读取电流、速度、位置 (0x94)"""
        self.send_command(self.CMD_R_CVP, [], log=False)
    
    def read_alarm(self):
        """读取报警信息 (0xFF)"""
        self.send_command(self.CMD_R_ALARM, [], log=False)
    
    def clear_alarm(self):
        """清除报警 (0xFE)"""
        self.send_command(self.CMD_CLEAR_ALARM, [])
        self._print_msg("清除报警命令已发送", "green")
    
    def read_all_status(self):
        """读取所有状态信息"""
        self.read_position()
        time.sleep(0.01)
        self.read_velocity()
        time.sleep(0.01)
        self.read_current()
        time.sleep(0.01)
        self.read_voltage()
        time.sleep(0.01)
        self.read_enable_status()
        time.sleep(0.01)
        self.read_control_mode()
    
    # ========== 消息接收处理 ==========
    
    def _receive_messages(self):
        """接收CAN消息的线程函数"""
        self._print_msg("CAN消息接收线程已启动", "green")
        
        while self.running:
            try:
                msg = self.bus.recv(timeout=1.0)
                if msg:
                    self._process_message(msg)
            except CanError as e:
                if self.running:
                    self._print_msg(f"接收CAN消息错误: {e}", "yellow")
                time.sleep(0.1)
            except Exception as e:
                if self.running:
                    self._print_msg(f"处理CAN消息时发生错误: {e}", "red")
                time.sleep(0.1)
    
    def _process_message(self, msg):
        """处理接收到的CAN消息"""
        msg_id = msg.arbitration_id
        data = list(msg.data)
        
        if len(data) < 1:
            return
        
        cmd = data[0]
        response_data = data[1:] if len(data) > 1 else []
        
        # 只打印重要消息
        if cmd not in [self.CMD_MIT_CONTROL]:
            self._print_msg(f"收到消息: ID=0x{msg_id:02X}, CMD=0x{cmd:02X}, 数据={response_data}", "blue")
        
        self._handle_response(cmd, response_data)
    
    def _handle_response(self, cmd: int, data: List[int]):
        """根据命令码处理响应数据"""
        try:
            if cmd == self.CMD_R_CURRENT and len(data) >= 4:
                self.current_current = struct.unpack('<f', bytes(data[:4]))[0]
                print(f"  当前电流: {self.current_current:.3f} A")
            
            elif cmd == self.CMD_R_VELOCITY and len(data) >= 4:
                self.current_velocity = struct.unpack('<f', bytes(data[:4]))[0]
                print(f"  当前速度: {self.current_velocity:.3f} rad/s")
            
            elif cmd == self.CMD_R_POSITION and len(data) >= 4:
                self.current_position = struct.unpack('<f', bytes(data[:4]))[0]
                print(f"  当前位置: {self.current_position:.3f} rad")
            
            elif cmd == self.CMD_R_ON_OFF and len(data) >= 1:
                self.motor_enabled = data[0] == 0x01
                print(f"  电机使能状态: {'使能' if self.motor_enabled else '关闭'}")
            
            elif cmd == self.CMD_R_VOLTAGE and len(data) >= 4:
                self.bus_voltage = struct.unpack('<f', bytes(data[:4]))[0]
                print(f"  母线电压: {self.bus_voltage:.2f} V")
            
            elif cmd == self.CMD_R_CURRENT_MODE and len(data) >= 1:
                self.control_mode = data[0]
                mode_names = {1: "力矩", 2: "MIT", 3: "速度", 4: "轨迹速度", 5: "位置", 6: "轨迹位置"}
                print(f"  控制模式: {mode_names.get(self.control_mode, f'未知({self.control_mode})')}")
            
            elif cmd == self.CMD_R_MOTOR_TEMP and len(data) >= 4:
                self.motor_temp = struct.unpack('<f', bytes(data[:4]))[0]
                print(f"  电机温度: {self.motor_temp:.1f} °C")
            
            elif cmd == self.CMD_R_CORE_TEMP and len(data) >= 4:
                self.core_temp = struct.unpack('<f', bytes(data[:4]))[0]
                print(f"  核心温度: {self.core_temp:.1f} °C")
            
            elif cmd == self.CMD_R_INVERTER_TEMP and len(data) >= 4:
                self.inverter_temp = struct.unpack('<f', bytes(data[:4]))[0]
                print(f"  逆变器温度: {self.inverter_temp:.1f} °C")
            
            elif cmd == self.CMD_R_CVP and len(data) >= 12:
                self.current_current = struct.unpack('<f', bytes(data[0:4]))[0]
                self.current_velocity = struct.unpack('<f', bytes(data[4:8]))[0]
                self.current_position = struct.unpack('<f', bytes(data[8:12]))[0]
                print(f"  CVP: 电流={self.current_current:.3f}A, 速度={self.current_velocity:.3f}rad/s, 位置={self.current_position:.3f}rad")
            
            elif cmd == self.CMD_R_ALARM and len(data) >= 4:
                self.motor_error = struct.unpack('<I', bytes(data[:4]))[0]
                self._print_alarm_info(self.motor_error)
            
            elif cmd == self.CMD_MIT_CONTROL and len(data) >= 7:
                # MIT控制响应
                self._parse_mit_response(data)
            
            elif cmd == self.CMD_R_VERSION and len(data) >= 1:
                self.version_info = ''.join([chr(b) for b in data if b != 0])
                print(f"  版本信息: {self.version_info}")
        
        except Exception as e:
            self._print_msg(f"解析响应数据时出错: {e}", "red")
    
    def _parse_mit_response(self, data: List[int]):
        """解析MIT控制响应数据"""
        # 数据格式: 位置(16) + 速度(12) + 转矩(12) + 电压(8) + 错误(8)
        p_int = (data[0] << 8) | data[1]
        v_int = (data[2] << 4) | (data[3] >> 4)
        t_int = ((data[3] & 0x0F) << 8) | data[4]
        voltage = data[5]
        error = data[6]
        
        self.current_position = self.uint2float(p_int, self.mit_pos_lower, self.mit_pos_upper, 16)
        self.current_velocity = self.uint2float(v_int, self.mit_vel_lower, self.mit_vel_upper, 12)
        self.current_torque = self.uint2float(t_int, self.mit_tor_lower, self.mit_tor_upper, 12)
        self.motor_error = error
    
    def _print_alarm_info(self, error_code: int):
        """打印报警信息"""
        alarms = []
        alarm_bits = [
            (0x001, "欠压"),
            (0x002, "过压"),
            (0x004, "过流"),
            (0x008, "电机过温"),
            (0x010, "核心过温"),
            (0x020, "逆变器过温"),
            (0x040, "超速"),
            (0x080, "校准错误"),
            (0x100, "编码器错误"),
        ]
        
        for bit, name in alarm_bits:
            if error_code & bit:
                alarms.append(name)
        
        if alarms:
            self._print_msg(f"  报警信息: {', '.join(alarms)} (0x{error_code:08X})", "red")
        else:
            print(f"  无报警 (0x{error_code:08X})")
    
    def get_status(self) -> Dict:
        """获取当前状态信息"""
        return {
            'position': self.current_position,
            'velocity': self.current_velocity,
            'torque': self.current_torque,
            'current': self.current_current,
            'voltage': self.bus_voltage,
            'motor_temp': self.motor_temp,
            'enabled': self.motor_enabled,
            'mode': self.control_mode,
            'error': self.motor_error
        }
    
    def close(self):
        """关闭CAN连接"""
        self._print_msg("正在关闭CAN连接...", "yellow")
        self.running = False
        
        if self.receive_thread and self.receive_thread.is_alive():
            self.receive_thread.join(timeout=2.0)
        
        if self.bus:
            self.bus.shutdown()
        
        self._print_msg("CAN连接已关闭", "green")


# ========== 多电机控制器 ==========

class LansiRobotArmController:
    """蓝思机械臂控制器（多电机）"""
    
    def __init__(self, motor_ids: List[int], can_channel: str = 'can0', baudrate: int = 1000000):
        """
        初始化机械臂控制器
        
        参数:
            motor_ids: 电机ID列表
            can_channel: CAN接口名称
            baudrate: CAN总线波特率
        """
        self.motor_ids = motor_ids
        self.can_channel = can_channel
        self.baudrate = baudrate
        self.motors: Dict[int, LansiMotorController] = {}
        
        # 为每个电机创建控制器
        for motor_id in motor_ids:
            self.motors[motor_id] = LansiMotorController(
                motor_id=motor_id,
                can_channel=can_channel,
                baudrate=baudrate
            )
        
        print(f"\n机械臂控制器初始化完成，共 {len(motor_ids)} 个电机")
    
    def enable_all(self):
        """使能所有电机"""
        for motor_id, motor in self.motors.items():
            motor.enable_motor(True)
            time.sleep(0.01)
    
    def disable_all(self):
        """关闭所有电机"""
        for motor_id, motor in self.motors.items():
            motor.disable_motor()
            time.sleep(0.01)
    
    def set_all_mode(self, mode: int):
        """设置所有电机的控制模式"""
        for motor_id, motor in self.motors.items():
            motor.set_mode(mode)
            time.sleep(0.01)
    
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
    
    def read_all_status(self):
        """读取所有电机状态"""
        for motor_id, motor in self.motors.items():
            print(f"\n--- 电机 {motor_id} ---")
            motor.read_all_status()
            time.sleep(0.1)
    
    def get_motor(self, motor_id: int) -> Optional[LansiMotorController]:
        """获取指定ID的电机控制器"""
        return self.motors.get(motor_id)
    
    def close(self):
        """关闭所有连接"""
        for motor_id, motor in self.motors.items():
            motor.close()


def main():
    """主函数 - 演示如何使用蓝思电机控制器"""
    
    # 单电机控制示例
    print("=" * 60)
    print("蓝思电机控制器演示")
    print("=" * 60)
    
    # 初始化控制器
    motor = LansiMotorController(
        motor_id=1,          # 电机ID
        can_channel='can0',  # CAN接口
        baudrate=1000000     # 波特率 1Mbps
    )
    
    try:
        time.sleep(0.5)
        
        # 1. 握手
        print("\n--- 1. 握手 ---")
        motor.handshake()
        time.sleep(0.3)
        
        # 2. 读取版本
        print("\n--- 2. 读取版本 ---")
        motor.read_version()
        time.sleep(0.3)
        
        # 3. 读取报警信息
        print("\n--- 3. 读取报警信息 ---")
        motor.read_alarm()
        time.sleep(0.3)
        
        # 4. 清除报警
        print("\n--- 4. 清除报警 ---")
        motor.clear_alarm()
        time.sleep(0.3)
        
        # 5. 使能电机
        print("\n--- 5. 使能电机 ---")
        motor.enable_motor(True)
        time.sleep(0.3)
        
        # 6. 设置位置控制模式
        print("\n--- 6. 设置位置控制模式 ---")
        motor.set_mode(motor.MODE_POSITION)
        time.sleep(0.3)
        
        # 7. 读取所有状态
        print("\n--- 7. 读取状态 ---")
        motor.read_all_status()
        time.sleep(0.5)
        
        # 8. 设置位置
        print("\n--- 8. 设置位置 (1.57 rad / 90度) ---")
        motor.set_position(1.57)
        time.sleep(1.0)
        
        # 9. 读取位置
        print("\n--- 9. 读取位置 ---")
        motor.read_position()
        time.sleep(0.3)
        
        # 10. MIT控制示例
        print("\n--- 10. MIT控制模式示例 ---")
        motor.set_mode(motor.MODE_MIT)
        time.sleep(0.3)
        motor.send_mit_command(
            position=0.0,    # 目标位置
            velocity=0.0,    # 目标速度
            kp=50.0,         # 位置增益
            kd=1.0,          # 阻尼增益
            torque=0.0       # 前馈力矩
        )
        time.sleep(0.5)
        
        # 等待接收反馈
        print("\n等待接收反馈消息（3秒）...")
        time.sleep(3)
        
    except KeyboardInterrupt:
        print("\n用户中断")
    except Exception as e:
        print(f"运行时发生错误: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # 关闭电机
        motor.disable_motor()
        time.sleep(0.1)
        motor.close()


if __name__ == "__main__":
    # 使用说明：
    # 1. 确保CAN接口已连接并配置:
    #    sudo ip link set can0 up type can bitrate 1000000
    # 2. 根据你的电机修改motor_id
    # 3. 运行脚本: python3 lansi_motor_controller.py
    
    main()
