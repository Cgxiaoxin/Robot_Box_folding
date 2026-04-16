# A7 SDK 使用文档

本文档整理 A7 机械臂常用 Python SDK 接口，包含运动控制、运动学、状态读取与参数配置。

## 1. 快速开始

```python
from linkerbot import A7

with A7(side="left", interface_name="can0") as arm:
    arm.enable()
```

## 2. 运动控制

### 2.1 回零 `home`

将所有关节移动到零位。

```python
from linkerbot import A7

with A7(side="left", interface_name="can0") as arm:
    arm.enable()
    arm.home()  # 阻塞模式
    arm.home(blocking=False)  # 非阻塞模式
```

**参数**

- `blocking`：是否阻塞等待运动完成，默认 `True`。

---

### 2.2 关节运动 `move_j`

通过发送每个关节目标角度控制机械臂运动。

```python
from linkerbot import A7

with A7(side="left", interface_name="can0") as arm:
    arm.enable()
    arm.move_j([0.0, 0.0, 0.0, 1.57, 0.0, 0.0, 0.0])
```

**参数**

- `target_joints`：7 个关节目标角度（`rad`），类型 `list[float]`。
- `blocking`：是否阻塞直到运动完成，默认 `True`，类型 `bool`。

**异常**

- `StateError`：机械臂运动中再次调用时抛出。
- `ValidationError`：目标角度超出关节限位时抛出。

---

### 2.3 位姿运动 `move_p`

通过发送 TCP 6D 位姿（`x, y, z, rx, ry, rz`）控制机械臂运动。  
SDK 会先进行逆运动学求解，再执行关节运动；末端路径不保证为直线。

```python
from linkerbot import A7, Pose

with A7(side="left", interface_name="can0") as arm:
    arm.enable()
    target = Pose(x=0.0, y=0.33, z=-0.25, rx=1.85, ry=0.0, rz=-1.57)
    arm.move_p(target)
```

**参数**

- `target_pose`：目标 TCP 位姿，类型 `linkerbot.arm.Pose`。
- `current_angles`：IK 初始猜测关节角度，类型 `list[float] | None`；为 `None` 时从电机读取。
- `blocking`：是否阻塞直到运动完成，默认 `True`，类型 `bool`。

**异常**

- `StateError`：机械臂运动中再次调用时抛出。
- `RuntimeError`：IK 求解失败时抛出。

---

### 2.4 直线运动 `move_l`

通过发送 TCP 6D 位姿控制末端做笛卡尔直线运动：  
位置使用梯形速度规划，姿态使用球面线性插值（Slerp）。

> `move_l` 始终阻塞直到运动完成，无 `blocking` 参数。

```python
from linkerbot import A7, Pose

with A7(side="left", interface_name="can0") as arm:
    arm.enable()
    target = Pose(x=0.0, y=0.33, z=-0.25, rx=1.85, ry=0.0, rz=-1.57)
    arm.move_l(target)
```

**参数**

- `target_pose`：目标 TCP 位姿，类型 `Pose`。
- `max_velocity`：线性最大速度（`m/s`），默认 `0.05`，范围 `(0, 0.4]`。
- `max_angular_velocity`：最大角速度（`rad/s`），默认 `0.3`，范围 `(0, 3.0]`。
- `acceleration`：线性最大加速度（`m/s²`），默认 `0.1`，范围 `(0, 1.0]`。
- `angular_acceleration`：角加速度（`rad/s²`），默认 `0.1`，范围 `(0, 1.0]`。
- `current_pose`：起始 TCP 位姿；`None` 时由 `current_angles` 或电机读取计算，类型 `Pose | None`。
- `current_angles`：起始关节角度；`None` 时从电机读取，类型 `list[float] | None`。

**异常**

- `StateError`：机械臂运动中再次调用时抛出。
- `ValidationError`：参数超出范围时抛出。
- `RuntimeError`：路径点 IK 求解失败时抛出。

---

### 2.5 运动状态检测 `is_moving`

返回机械臂当前是否处于运动状态。

```python
if arm.is_moving():
    print("正在运动中...")
```

**返回值**

- `bool`：是否在运动中。

---

### 2.6 等待运动完成 `wait_motion_done`

阻塞当前线程，直到运动计时器归零。通常与 `blocking=False` 配合使用。

```python
arm.move_j([0.0, 0.0, 0.0, 1.57, 0.0, 0.0, 0.0], blocking=False)
# ... 执行其他操作 ...
arm.wait_motion_done()  # 等待运动结束
```

### 2.7 示例：非阻塞运动与等待

```python
from linkerbot import A7, Pose

with A7(side="left", interface_name="can0") as arm:
    arm.enable()
    arm.home()

    # 设置速度和加速度
    arm.set_velocities([1.0] * 7)
    arm.set_accelerations([10.0] * 7)

    # 非阻塞 move_j：发送后立即返回，可执行其他操作
    arm.move_j([0.0, 0.0, 0.0, 1.57, 0.0, 0.0, 0.0], blocking=False)
    print("move_j 已发送，正在运动中...")
    arm.wait_motion_done()

    # 阻塞 move_p：调用直接等到运动完成
    target = Pose(x=0.0, y=0.33, z=-0.25, rx=1.85, ry=0.0, rz=-1.57)
    arm.move_p(target)

    # move_l 始终阻塞
    target = Pose(x=0.1, y=0.33, z=-0.2, rx=1.85, ry=0.0, rz=-1.57)
    arm.move_l(target)

    arm.home()
```

## 3. 运动学

### 3.1 正运动学 `forward_kinematics`

根据关节角度计算 TCP 位姿。

```python
pose = arm.forward_kinematics([0.0, 0.0, 0.0, 1.57, 0.0, 0.0, 0.0])
print(f"TCP: ({pose.x:.3f}, {pose.y:.3f}, {pose.z:.3f})")
```

**参数**

- `angles`：7 个关节角度（`rad`），类型 `list[float]`。

**返回值**

- `Pose`：TCP 位姿。

---

### 3.2 逆运动学 `inverse_kinematics`

根据 TCP 位姿求解关节角度。

```python
from linkerbot import Pose

target = Pose(x=0.0, y=0.33, z=-0.25, rx=1.85, ry=0.0, rz=-1.57)
angles = arm.inverse_kinematics(target)
print(f"关节角度：{angles}")
```

**参数**

- `pose`：目标 TCP 位姿，类型 `Pose`。
- `current_angles`：IK 初始猜测关节角度，类型 `list[float] | None`；为 `None` 时从电机读取。

**返回值**

- `list[float]`：7 个关节角度（`rad`）。

---

### 3.3 关节限位

```python
# 获取当前关节限位
limits = arm.get_joint_limits()
print(limits)  # [(lower0, upper0), (lower1, upper1), ...]

# 设置自定义关节限位
arm.set_joint_limits([(-1.0, 1.0)] * 7)
```

## 4. 状态读取

### 4.1 完整状态快照 `get_state`

一次性读取完整状态，包含 TCP 位姿和所有关节的角度、速度、加速度、扭矩、温度。

```python
from linkerbot import A7

with A7(side="left", interface_name="can0") as arm:
    arm.enable()
    state = arm.get_state()
    print(state.pose)
    print(f"第一个关节实际角度：{state.joint_angles[0].angle:.4f} rad")
```

**返回值**

- `State`：包含 `pose`、`joint_angles`、`joint_control_angles`、`joint_velocities`、`joint_control_velocities`、`joint_control_acceleration`、`joint_torques`、`joint_temperatures`。

### 4.2 `State` 数据模型

| 属性 | 说明 |
| --- | --- |
| `pose` | TCP 位姿（`Pose`） |
| `joint_angles` | 7 个关节的实际角度（`list[AngleState]`） |
| `joint_control_angles` | 7 个关节的控制角度（`list[AngleState]`） |
| `joint_velocities` | 7 个关节的实际速度（`list[VelocityState]`） |
| `joint_control_velocities` | 7 个关节的控制速度（`list[VelocityState]`） |
| `joint_control_acceleration` | 7 个关节的控制加速度（`list[AccelerationState]`） |
| `joint_torques` | 7 个关节的实际扭矩（`list[TorqueState]`） |
| `joint_temperatures` | 7 个关节的温度（`list[TemperatureState]`） |

各子状态模型均包含值字段和 `timestamp`（UNIX 时间戳，秒）。

### 4.3 子状态模型字段

| 模型 | 值字段 | 时间戳字段 |
| --- | --- | --- |
| `AngleState` | `.angle` | `.timestamp` |
| `VelocityState` | `.velocity` | `.timestamp` |
| `AccelerationState` | `.acceleration` | `.timestamp` |
| `TorqueState` | `.torque` | `.timestamp` |
| `TemperatureState` | `.temperature` | `.timestamp` |

### 4.4 单项状态读取

除 `get_state()` 外，以下方法返回单一类型状态数据（每项均为 7 关节列表，`get_pose()` 除外）。

```python
from linkerbot import A7

with A7(side="left", interface_name="can0") as arm:
    arm.enable()
    angles = arm.get_angles()
    velocities = arm.get_velocities()
    temps = arm.get_temperatures()
    pose = arm.get_pose()
    print(f"第二个关节实际角度：{angles[1]:.4f} rad")
    print(f"第三个关节实际速度：{velocities[2]:.4f} rad/s")
    print(f"第四个关节实际温度：{temps[3]:.1f} °C")
    print(f"TCP: ({pose.x:.3f}m, {pose.y:.3f}m, {pose.z:.3f}m)")
```

| 方法 | 返回值 | 单位 |
| --- | --- | --- |
| `get_angles()` | 7 个关节的实际角度 | `rad` |
| `get_control_angles()` | 7 个关节的控制角度 | `rad` |
| `get_velocities()` | 7 个关节的实际速度 | `rad/s` |
| `get_control_velocities()` | 7 个关节的控制速度 | `rad/s` |
| `get_control_acceleration()` | 7 个关节的控制加速度 | `rad/s²` |
| `get_torques()` | 7 个关节的实际扭矩 | `Nm` |
| `get_temperatures()` | 7 个关节温度 | `°C` |
| `get_pose()` | TCP 位姿 | `m, rad` |

### 4.5 传感器数据轮询

A7 使用轮询模式采集传感器数据，`get_*()` 返回的是上一次轮询值。

| 数据类型 | 单位 | 默认轮询间隔 | 最大延迟 |
| --- | --- | --- | --- |
| 角度 | `rad` | 10 ms | ≈ 10 ms |
| 速度 | `rad/s` | 10 ms | ≈ 10 ms |
| 扭矩 | `Nm` | 50 ms | ≈ 50 ms |
| 温度 | `°C` | 1000 ms | ≈ 1000 ms |

如需更低延迟，可调用 `start_polling()` 自定义轮询间隔：

```python
from linkerbot import A7
from linkerbot.arm.a7 import SensorType

with A7(side="left", interface_name="can0") as arm:
    arm.start_polling(
        {
            SensorType.POSITION: 0.005,     # 5 ms = 200 Hz
            SensorType.VELOCITY: 0.005,
            SensorType.TORQUE: 0.1,
            SensorType.TEMPERATURE: 2.0,
        }
    )
    arm.enable()
    arm.home()
```

## 5. 参数配置

### 5.1 速度与加速度限制

设置 7 个关节速度和加速度限制。

```python
from linkerbot import A7

with A7(side="left", interface_name="can0") as arm:
    arm.enable()
    arm.set_velocities([1.0] * 7)  # 所有关节速度限制 1.0 rad/s
    arm.set_accelerations([10.0, 9.0, 8.0, 7.0, 6.0, 5.0, 4.0])
```

| 方法 | 参数 | 范围 | 说明 |
| --- | --- | --- | --- |
| `set_velocities(velocities)` | 7 个关节速度上限（`rad/s`） | `[0.0, 50.0]` | 默认 `0.5` |
| `set_accelerations(accelerations)` | 7 个关节加速度上限（`rad/s²`） | `[1.0, 50.0]` | 默认 `10.0`，同时设置加速度和减速度 |

异常：`ValidationError`（值数量不为 7 或数值超范围时）。

---

### 5.2 PID 参数设置

```python
from linkerbot import A7

with A7(side="left", interface_name="can0") as arm:
    arm.set_position_kps([100.0] * 7)
    arm.set_velocity_kps([0.5] * 7)
    arm.set_velocity_kis([0.01] * 7)
```

| 方法 | 参数 | 说明 |
| --- | --- | --- |
| `set_position_kps(kps)` | 7 个关节位置 Kp | 位置环比例增益 |
| `set_velocity_kps(kps)` | 7 个关节速度 Kp | 速度环比例增益 |
| `set_velocity_kis(kis)` | 7 个关节速度 Ki | 速度环积分增益 |

---

### 5.3 参数速查

| 参数类型 | 默认值 | 范围 |
| --- | --- | --- |
| 速度限制 | `0.5 rad/s` | `[0.0, 50.0] rad/s` |
| 加速度限制 | `10.0 rad/s²` | `[1.0, 50.0] rad/s²` |
| `move_l` 最大平移速度 | `0.05 m/s` | `(0, 0.4] m/s` |
| `move_l` 最大角速度 | `0.3 rad/s` | `(0, 3.0] rad/s` |
| `move_l` 平移加速度 | `0.1 m/s²` | `(0, 1.0] m/s²` |
| `move_l` 角加速度 | `0.1 rad/s²` | `(0, 1.0] rad/s²` |

### 5.4 示例：设置参数后运动

```python
from linkerbot import A7

with A7(side="left", interface_name="can0") as arm:
    arm.set_position_kps([80.0] * 7)
    arm.set_velocity_kps([0.4] * 7)
    arm.set_velocity_kis([0.008] * 7)
    arm.enable()
    arm.set_velocities([1.0] * 7)
    arm.set_accelerations([10.0] * 7)
    arm.home(blocking=True)
    arm.move_j([0.0, 0.0, 0.0, 1.57, 0.0, 0.0, 0.0], blocking=True)
```

### 5.5 参数持久化 `save_params`

调用 `save_params()` 将当前 PID、速度、加速度等参数写入电机闪存，断电后仍生效。

```python
from linkerbot import A7

with A7(side="left", interface_name="can0") as arm:
    arm.set_position_kps([80.0] * 7)
    arm.set_velocity_kps([0.4] * 7)
    arm.save_params()  # 保存到闪存
```

注意：每次写入耗时约 1 ms，避免在实时控制循环中调用。
