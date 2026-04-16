# LinkerHand L6 双手联调记录（2026-04-13）

## 1. 联调结论
- 当前现场联调结果：左右手 L6 均可正常控制。
- 控制链路结论：
  - 手部驱动与通信：通过 **ROS2 节点 `linker_hand_ros2_sdk`** 连接 CAN 并下发控制。
  - 测试与控制入口：可用 **ROS2 topic/launch**，也可用 Python 启动 GUI（本质仍发布 ROS2 话题）。
  - `linkerhand-python-sdk` 也可直接连 CAN，但本次“左右手联调成功”的主路径是 ROS2。

## 2. 最终可用配置

### 2.1 右手单手启动（L6）
文件：`src/linkerhand-ros2-sdk/linker_hand_ros2_sdk/launch/linker_hand.launch.py`
- `hand_type: 'right'`
- `hand_joint: 'L6'`
- `can: 'can0'`
- `modbus: 'None'`

### 2.2 双手启动（L6/L6）
文件：`src/linkerhand-ros2-sdk/linker_hand_ros2_sdk/launch/linker_hand_double.launch.py`
- 左手：`hand_type='left'`，`hand_joint='L6'`，`can='can0'`（按现场可改）
- 右手：`hand_type='right'`，`hand_joint='L6'`，`can='can1'`（按现场可改）

### 2.3 GUI 侧配置
文件：`src/linkerhand-ros2-sdk/gui_control/launch/gui_control.launch.py`
- 已改为 `hand_type='right'`、`hand_joint='L6'`（默认右手 L6）

## 3. 本次关键问题与修复

### 3.1 `ModuleNotFoundError: No module named 'can.exceptions'`
- 原因：系统 Python 使用 `python-can 3.3.2`，没有 `can.exceptions` 子模块。
- 修复：在多个 CAN 驱动文件中改为兼容导入：
  - 先尝试 `from can.exceptions import CanError`
  - 失败回退 `from can import CanError`

### 3.2 `TypeError: object of type 'NoneType' has no len()`
- 位置：`linker_hand.py` 中 `embedded_version` 判定逻辑。
- 原因：`embedded_version` 可能为 `None`，直接 `len(None)` 崩溃。
- 修复：对 `embedded_version` 做空值兜底（非 list/tuple 时置空列表）。

### 3.3 `setting.yaml` 找不到导致启动失败
- 现象：`TypeError: 'NoneType' object is not subscriptable`（配置加载失败后继续索引）。
- 原因：`setup.py` 未把 `LinkerHand/config/*.yaml` 正确安装到 `install`。
- 修复：
  - `setup.py` 增加 `package_data` 与 `data_files`，安装配置 YAML。
  - `load_write_yaml.py` 增加多路径兜底：优先包内 `config`，其次 `share` 路径。

### 3.4 CAN 发送失败 `Error Code 105`（没有可用的缓冲区空间）
- 含义：通常为总线无 ACK / 接口 DOWN / 波特率或接线异常导致发送队列堆积。
- 本次观察到：`can0` 出现 `DOWN`，导致 `Network is down`。
- 处理建议：先拉起 CAN，再做节点与动作测试（见第 4 节）。

## 4. 复现实测流程（推荐）

### 4.1 重置 CAN
在 `robot_arm_web` 目录执行：
```bash
cd /data/coding_pro/Robot_Box_folding/robot_arm_web
bash reset_can.sh 1000000 1000
ip -br link show can0
```

### 4.2 启动右手 L6 驱动
```bash
cd /data/coding_pro/Robot_Box_folding/src/linkerhand-ros2-sdk
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch linker_hand_ros2_sdk linker_hand.launch.py
```

### 4.3 发送动作测试（右手 L6）
新终端执行：
```bash
source /opt/ros/humble/setup.bash
source /data/coding_pro/Robot_Box_folding/src/linkerhand-ros2-sdk/install/setup.bash

ros2 topic pub --once /cb_right_hand_control_cmd sensor_msgs/msg/JointState "{name: [], position: [255,255,255,255,255,255], velocity: [0,0,0,0,0,0], effort: [0,0,0,0,0,0]}"
sleep 1
ros2 topic pub --once /cb_right_hand_control_cmd sensor_msgs/msg/JointState "{name: [], position: [0,0,0,0,0,0], velocity: [0,0,0,0,0,0], effort: [0,0,0,0,0,0]}"
```

### 4.4 GUI 测试
- 推荐：
```bash
ros2 launch gui_control gui_control.launch.py
```
- 或 Python 启动右手脚本：
```bash
cd /data/coding_pro/Robot_Box_folding/src/linkerhand-ros2-sdk/gui_control
python gui_control_right.py
```

## 5. “ROS2 还是 Python”说明
- 本次“左右手调试成功”的执行链路是：**ROS2 驱动节点 + ROS2 topic 控制**。
- Python 在本次主要用于 GUI 程序与脚本入口，GUI 底层仍是 ROS2 publisher。
- 若采用 `linkerhand-python-sdk` 直连模式，则可不经过 ROS2 节点，但属于另一条链路，适合做 SDK 级直连调试。

