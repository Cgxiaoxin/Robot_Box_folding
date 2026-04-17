"""
Web控制系统配置
"""
import os
from pathlib import Path

# 项目根目录
PROJECT_ROOT = Path(__file__).resolve().parents[2]
WEB_ROOT = Path(__file__).resolve().parents[1]

# CAN配置
# 结合当前 A7 SDK 实测映射:
# - right side -> can0
# - left side  -> can1
# 项目逻辑臂名 left/right 在 GroupController 内会转换为 SDK side。
DEFAULT_LEFT_CAN = "can0"
DEFAULT_RIGHT_CAN = "can1"
CAN_BITRATE = 1000000

# 灵巧手 L6 配置（第三步：Python 直连 LinkerHandApi + CAN）
HAND_ENABLE = os.getenv("HAND_ENABLE", "true").lower() == "true"
HAND_TYPE = os.getenv("HAND_TYPE", "left")  # left | right
HAND_JOINT = os.getenv("HAND_JOINT", "L6")
HAND_CAN = os.getenv("HAND_CAN", "can2")
HAND_MODBUS = os.getenv("HAND_MODBUS", "None")

# 电机配置
LEFT_MOTOR_IDS = [51, 52, 53, 54, 55, 56, 57]
RIGHT_MOTOR_IDS = [61, 62, 63, 64, 65, 66, 67]
ALL_MOTOR_IDS = LEFT_MOTOR_IDS + RIGHT_MOTOR_IDS

# 服务配置
HOST = "0.0.0.0"
PORT = 5000
DEBUG = os.getenv("DEBUG", "false").lower() == "true"

# 初始化流程是否自动 home（默认关闭，避免上线环境意外动作）
INIT_HOME_ENABLED = os.getenv("INIT_HOME_ENABLED", "false").lower() == "true"
INIT_HOME_BLOCKING = os.getenv("INIT_HOME_BLOCKING", "true").lower() == "true"

# 全量状态推送间隔 (毫秒)
STATE_UPDATE_INTERVAL = int(os.getenv("STATE_UPDATE_INTERVAL", "300"))

# 笛卡尔实时坐标推送间隔 (毫秒)
CARTESIAN_LIVE_INTERVAL = int(os.getenv("CARTESIAN_LIVE_INTERVAL", "80"))

# 轨迹文件目录
TRAJECTORIES_DIR = WEB_ROOT / "trajectories"

# 轨迹播放参数
# A7 当前 SDK 在机械臂处于 moving 状态时，不支持再次启动新的整臂 motion。
# 因此默认关闭高频非阻塞流式插值；若后续 SDK 明确支持运行中覆写目标，可再通过环境变量开启。
TRAJECTORY_STREAMING_ENABLED = os.getenv("TRAJECTORY_STREAMING_ENABLED", "false").lower() == "true"
TRAJECTORY_PLAYBACK_CONTROL_HZ = float(os.getenv("TRAJECTORY_PLAYBACK_CONTROL_HZ", "50.0"))
TRAJECTORY_MIN_SEGMENT_DURATION_S = float(os.getenv("TRAJECTORY_MIN_SEGMENT_DURATION_S", "0.04"))
TRAJECTORY_MAX_SEGMENT_DURATION_S = float(os.getenv("TRAJECTORY_MAX_SEGMENT_DURATION_S", "2.0"))
TRAJECTORY_START_ALIGN_TOLERANCE_RAD = float(os.getenv("TRAJECTORY_START_ALIGN_TOLERANCE_RAD", "0.02"))
TRAJECTORY_START_MAX_DEVIATION_RAD = float(os.getenv("TRAJECTORY_START_MAX_DEVIATION_RAD", "0.12"))

# 统一安全恢复参数
# 急停安全恢复相关参数 **
SAFE_RECOVERY_VELOCITY = float(os.getenv("SAFE_RECOVERY_VELOCITY", "0.15"))  # 恢复初始速度（关节/s）
SAFE_RECOVERY_ACCEL = float(os.getenv("SAFE_RECOVERY_ACCEL", "0.3"))         # 恢复初始加速度
SAFE_RECOVERY_SETTLE_S = float(os.getenv("SAFE_RECOVERY_SETTLE_S", "0.5"))  # 初始持位后等待时间（秒）
SAFE_RECOVERY_RAMP_STEPS = int(os.getenv("SAFE_RECOVERY_RAMP_STEPS", "3"))   # 恢复插值步数
SAFE_RECOVERY_RAMP_INTERVAL_S = float(os.getenv("SAFE_RECOVERY_RAMP_INTERVAL_S", "0.12"))  # 步进插值间隔（秒）
SAFE_RECOVERY_PID_SCALE = float(os.getenv("SAFE_RECOVERY_PID_SCALE", "0.35"))  # 急停PID比例缩放（恢复前松弛）

# 日志目录
LOG_DIR = WEB_ROOT / "logs"

# 零点偏移文件（与 arm_control 共用，连接时自动读入，标定时更新）
ZERO_OFFSET_FILE = WEB_ROOT / "zero_offset.json"

# 预设动作配置
PRESET_ACTIONS = {
    "A": "preset_a.json",
    "B": "preset_b.json",
    "C": "preset_c.json",
    "D": "preset_d.json",
    "E": "preset_e.json",
    "F": "preset_f.json",
}

################################################################################
# 速度与关节限制配置
################################################################################

# 速度预设
SPEED_PRESETS = {
    # A7 SDK acceleration 有效范围 [1.0, 50.0]
    "very_slow": {"velocity": 0.2, "accel": 1.0, "decel": 1.0},
    "slow": {"velocity": 0.5, "accel": 1.0, "decel": 1.0},
    "medium": {"velocity": 1.0, "accel": 2.0, "decel": 2.0},
    "fast": {"velocity": 2.0, "accel": 4.0, "decel": 4.0},
}

# 软关节限位（弧度）
# 说明：
# - 默认采用较为保守的范围，避免在前端误操作导致大幅度摆动
# - 如果后续有更精确的驱动/机构限制，可在此处调整
JOINT_LIMITS = {
    "left": {
        # 对应 LEFT_MOTOR_IDS 顺序：51~57
        "position_min": [-2.5, -2.5, -2.0, -2.0, -2.5, -2.5, -2.5],
        "position_max": [2.5, 2.5, 2.0, 2.0, 2.5, 2.5, 2.5],
    },
    "right": {
        # 对应 RIGHT_MOTOR_IDS 顺序：61~67
        "position_min": [-2.5, -2.5, -2.0, -2.0, -2.5, -2.5, -2.5],
        "position_max": [2.5, 2.5, 2.0, 2.0, 2.5, 2.5, 2.5],
    },
}
