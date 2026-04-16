from tools.lansi_motor_controller import LansiMotorController
import time

motor = LansiMotorController(motor_id=51, can_channel="can0", baudrate=1000000)  # 按实际ID/通道改
time.sleep(0.3)
motor.handshake()
time.sleep(0.2)
motor.read_version()
time.sleep(0.5)  # 等待返回
motor.close()
