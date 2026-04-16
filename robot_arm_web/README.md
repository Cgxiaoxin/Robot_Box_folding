# 机械臂演出控制系统 (Web版)

基于 Flask + Socket.IO 的机械臂Web控制系统，专为演出/展览场景设计。

## 功能特性

- **轨迹播放**：选择轨迹文件，一键播放/暂停/停止
- **预设动作**：6个大按钮快速执行预设动作（A-F）
- **实时监控**：实时显示左右臂7个关节的位置和状态
- **群控管理**：支持左臂/右臂/双臂同步控制
- **手动控制**：单关节点动控制
- **示教编程**：记录当前位置，创建自定义轨迹
- **速度设置**：预设速度档位 + 自定义速度参数
- **紧急停止**：醒目的急停按钮，一键停止所有电机

## 系统要求

- Linux系统（支持SocketCAN）
- Python 3.8+
- CAN接口（can0, can1）

## 快速开始

### 1. 安装依赖

```bash
cd /data/coding_pro/robot_arm_hans/robot_arm_web
pip install -r requirements.txt
```

### 2. 配置CAN接口

```bash
# 配置左臂 (can0)
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up

# 配置右臂 (can1)
sudo ip link set can1 down
sudo ip link set can1 type can bitrate 1000000
sudo ip link set can1 up
```

### 3. 启动服务

```bash
# Linux
./start.sh

# 或直接运行
python backend/app.py
```

### 4. 打开浏览器

访问：`http://127.0.0.1:5000`

## 目录结构

```
robot_arm_web/
├── backend/
│   ├── __init__.py
│   ├── app.py              # Flask主程序 + Socket.IO
│   ├── config.py           # 配置文件
│   ├── group_controller.py # 群控管理器
│   └── trajectory_engine.py # 轨迹引擎
├── frontend/
│   ├── index.html          # 主界面
│   ├── css/
│   │   └── style.css       # 样式
│   └── js/
│       └── main.js         # 前端逻辑
├── trajectories/           # 轨迹文件
│   ├── preset_a.json       # 待机
│   ├── preset_b.json       # 打鼓
│   ├── preset_c.json       # 弹琴
│   ├── preset_d.json       # 挥手
│   ├── preset_e.json       # 鞠躬
│   └── preset_f.json       # 自定义
├── requirements.txt        # Python依赖
├── start.sh               # Linux启动脚本
├── start.bat              # Windows启动脚本
└── README.md              # 本文档
```

## 使用说明

### 基本操作流程

1. **连接**：点击顶部"连接"按钮
2. **初始化**：点击"初始化"按钮使能电机
3. **选择目标**：在群控管理区选择控制目标（左臂/右臂/双臂同步）
4. **执行动作**：
   - 点击预设动作按钮（A-F）
   - 或选择轨迹文件点击播放
   - 或在手动控制区点动控制

### 紧急停止

如遇紧急情况，点击右上角红色 **"紧急停止"** 按钮，所有电机将立即关闭。

### 创建自定义轨迹

1. 切换到"示教编程"标签
2. 使用"自由拖动"模式手动移动机械臂
3. 输入点位名称，点击"记录当前位置"
4. 重复步骤2-3记录多个点
5. 轨迹自动保存到 `trajectories/` 目录

### 轨迹文件格式

```json
{
  "name": "动作名称",
  "description": "动作描述",
  "points": [
    {
      "name": "点位名称",
      "positions": {
        "51": 0.0,
        "52": 0.5,
        "53": 1.0,
        "54": 0.0,
        "55": 0.0,
        "56": 0.0,
        "57": 0.0
      },
      "delay": 1.0
    }
  ],
  "loop": false,
  "speed_multiplier": 1.0
}
```

## 舞台工作人员操作手册

### 启动步骤

1. 确认CAN线已连接（左臂→can0，右臂→can1）
2. 确认机械臂已上电
3. 双击桌面的 **"启动机械臂控制系统"** 图标
4. 等待浏览器自动打开（约5秒）
5. 看到"已连接"状态后即可操作

### 演出流程

1. 点击 **"连接"** → 等待状态变绿
2. 点击 **"初始化"** → 等待完成
3. 点击预设动作测试（如"动作A"）
4. 确认无误后，开始演出

### 常见问题

| 问题 | 解决方法 |
|------|---------|
| 无法连接 | 检查CAN线是否插好，检查电源 |
| 动作不同步 | 重启程序，确认选择了"双臂同步" |
| 浏览器打不开 | 手动打开浏览器输入 http://127.0.0.1:5000 |
| 电机无响应 | 检查电机是否上电，运行 `python scan_motors.py` 诊断 |

## 配置说明

编辑 `backend/config.py` 可修改：

```python
# CAN通道
DEFAULT_LEFT_CAN = "can0"
DEFAULT_RIGHT_CAN = "can1"

# 服务端口
PORT = 5000

# 状态更新间隔（毫秒）
STATE_UPDATE_INTERVAL = 200
```

## 依赖

```
Flask==3.0.0
python-socketio==5.11.0
eventlet==0.35.0
python-can>=4.0.0
pyyaml>=6.0
```

## 技术架构

```
┌─────────────────┐
│  浏览器 (Web UI) │
└────────┬────────┘
         │ Socket.IO (WebSocket)
         ▼
┌─────────────────┐
│ Flask + SocketIO │
│   (Python后端)   │
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│ GroupController │
│   (群控管理器)   │
├────────┬────────┤
│ 左臂   │ 右臂   │
│ can0   │ can1   │
└────────┴────────┘
         │
         ▼ CAN总线
┌─────────────────┐
│  7轴电机 x 2    │
└─────────────────┘
```

## 注意事项

1. **首次使用前务必设置慢速**
2. **确保机械臂周围无障碍物**
3. **熟悉紧急停止按钮位置**
4. **退出时请先点击"回零"**

## 更新日志

### v1.0.0
- 初始版本
- 支持左右臂双臂同步控制
- 轨迹播放、预设动作、手动控制
- 实时状态监控
- 一键启动脚本
