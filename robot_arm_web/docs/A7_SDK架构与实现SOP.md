# Linkerbot A7 机械臂 Python SDK — 架构与实现 SOP

> **文档性质**：基于 `linkerbot-python-sdk` 源码整理的设计说明与标准作业流程，供学习、对照与自研高可用机械臂 SDK 时参考。  
> **版本说明**：以仓库内 `src/linkerbot/` 为准；具体魔数（寄存器命令字节）以 `arm/a7/motor.py` 为准。

---

## 1. 文档目的与适用范围

| 项目 | 说明 |
|------|------|
| **目的** | 说明 A7 SDK 从 CAN 底层到笛卡尔运动的**分层设计**、**线程模型**、**数据流**与**故障处理**，形成可复用的设计 SOP。 |
| **读者** | 需要维护 A7 集成代码，或计划自研「机械臂 + CAN 驱动」类 SDK 的工程师。 |
| **不覆盖** | 具体硬件电气规范、非 A7 机型差异、上层业务系统（调度、数字孪生等）。 |

---

## 2. 设计目标与原则（高可用视角）

### 2.1 设计目标

1. **可预期**：连接后立刻知道「7 轴是否都在线」；非法输入在**下发前**被拒绝。  
2. **总线友好**：高频下发（如 `move_l`）时**不冲垮 CAN**；发送路径有队列与节拍。  
3. **状态一致**：读反馈与阻塞式读寄存器通过**同一套分发机制**，减少「睡多久再读」的猜测。  
4. **运动安全**：避免在运动未完成时叠加新的整臂运动；急停路径明确。  
5. **运动学可信**：FK/IK 与**随包 URDF + Pinocchio** 绑定，关节限位与模型一致；支持现场坐标系（`urdf` / `maestro`）与 TCP 偏移。

### 2.2 架构原则（可抄作业）

| 原则 | 在本 SDK 中的体现 |
|------|-------------------|
| **单一入口** | 用户主要操作 `A7` 类，而不是直接操作 7 个 `A7Motor`。 |
| **通信与协议解耦** | `CANMessageDispatcher` 只管「收发与分发」；`A7Motor` 只管「字节语义」。 |
| **异步总线 + 同步 API** | 收包在后台线程；用户线程通过 `DataRelay.wait()` 做**请求-响应**同步。 |
| **显式资源生命周期** | `A7.__enter__` / `close()` 停止轮询、等待运动结束、停止 Dispatcher。 |
| **失败快速暴露** | 构造期 `check_alive` / `read_initial_state` 失败即抛 `StateError`，避免半连接状态。 |

---

## 3. 分层架构总览

自上而下可分为五层（逻辑分层，非必须拆成五个 Python 包）：

```
┌─────────────────────────────────────────────────────────────┐
│  L5 应用 API        A7: move_j / move_p / move_l / enable …   │
├─────────────────────────────────────────────────────────────┤
│  L4 运动学与规划     ArmKinetix: FK, IK, plan_move_l           │
├─────────────────────────────────────────────────────────────┤
│  L3 运动状态         MotionTimer, _guard_not_moving            │
├─────────────────────────────────────────────────────────────┤
│  L2 单轴语义         A7Motor: 寄存器读写、轮询、控制量缓存       │
├─────────────────────────────────────────────────────────────┤
│  L1 CAN 基础设施     CANMessageDispatcher + DataRelay          │
└─────────────────────────────────────────────────────────────┘
```

**依赖方向**：L5 → L4 → L3 → L2 → L1（上层依赖下层，下层不依赖上层）。

---

## 4. 标准作业流程（SOP）：从开机到运动

### SOP-1 创建实例（连接与验收）

1. 用户指定 `side`（`left` / `right`）、`interface_name`（如 `can0`）、可选 `tcp_offset`、`world_frame`。  
2. 构造 `CANMessageDispatcher`：拉起 **接收线程** 与 **发送线程**。  
3. 按臂别创建 7 个 `A7Motor`（左臂 ID `61–67`，右臂 `51–57`），每个电机 **subscribe** 总线回调。  
4. 构造 `ArmKinetix("a7", side, ...)`：加载内置 URDF，建立 Pinocchio 模型与 TCP 帧。  
5. **`_check_motors()`**：对每个电机 `check_alive()`（握手寄存器），无响应则 `StateError` 列出 ID。  
6. **`read_initial_state()`**：同步读位置/速度/力矩/温度/使能/控制量/Kp/Ki 等，保证内存中有合法初值。  
7. 若某电机 `has_initial_data()` 失败 → `StateError`。  
8. **`start_polling()`**：按 `DEFAULT_POLL_INTERVALS` 为各传感器起独立轮询线程。

**高可用要点**：验收在**构造期**完成，避免用户先 `move_j` 再发现第 3 轴不通。

### SOP-2 使能与模式

1. `enable()`：`reset_error()` → `set_control_mode`（默认 `ControlMode.PP`）→ 各轴 `enable()` → 短 `sleep` 等待驱动稳定。  
2. 控制模式当前映射为驱动器 PP 模式（`0x07, 0x05`），见 `A7Motor._CONTROL_MODE_MAP`。

### SOP-3 关节运动 `move_j`

1. **`_guard_not_moving`**：若 `MotionTimer` 认为仍在运动，抛 `StateError`，禁止叠运动。  
2. `_set_angles`：可选按 `ArmKinetix.get_joint_limits()` 做限位校验。  
3. 各 `A7Motor.set_angle`：更新本地「控制角」状态并发 CAN（经发送队列限流）。  
4. `_move_duration`：按每轴当前**控制速度/加速度**与关节行程做**梯形时间**估计，取**最慢轴**为整臂等待时间。  
5. `MotionTimer.start(duration)`；若 `blocking`，`wait_motion_done()`。

**说明**：等待时间是**估计值**，不是驱动器回传的「到位」信号；高可用场景下若需硬到位，应扩展为读位置误差或驱动器完成位（若协议支持）。

### SOP-4 点位运动 `move_p`

1. 取 `current_angles`（默认 `get_angles()`）。  
2. `ArmKinetix.inverse_kinematics(pose, current_angles)` 得目标关节角。  
3. 与 `move_j` 相同：`_set_angles`（此处 `check_limits=False`）+ `MotionTimer`。

### SOP-5 直线运动 `move_l`（笛卡尔插补 + 关节流）

1. 参数校验：`max_velocity`、`acceleration`、角速度/角加速度在 `consts` 范围内。  
2. 确定起点位姿：优先 `current_angles` → FK；若同时给 `current_pose` 则与 FK 比对，不一致则 **warn** 并以 FK 为准。  
3. 用位置差、姿态差（SO(3) 测地距离）分别算梯形时间，`blocking_time = max(平移, 旋转) + waypoint_interval`，启动 `MotionTimer`。  
4. **临时** `set_velocities(MAX_VELOCITY)`、`set_accelerations(MAX_ACCELERATION)`，以便密集路点下电机跟随。  
5. `plan_move_l(...)` 生成 `WayPoint` 序列；循环中按 **`perf_counter` 与 10ms 节拍** `sleep` 对齐，逐点 `_set_angles`。  
6. `finally`：恢复原速度/加速度，`wait_motion_done()`。

**高可用要点**：  
- 笛卡尔路径在 **Kinetix** 内完成（Slerp + 独立平移/旋转梯形）；每点 IK **以前一路点为 seed**，平滑且易收敛。  
- 发送侧已有 **CAN 层限流**，与上层节拍配合，降低丢帧概率。

### SOP-6 急停 `emergency_stop`

- 线程池并行：每轴先 **读当前位置**，再 `set_angle` 同一角度（等效「冻结当前指令位置」）。  
- 与「禁用使能」不同，属于**保持使能下的快速 hold** 策略（按现有实现语义理解）。

### SOP-7 关闭 `close`

1. `stop_polling()`：停各电机轮询线程。  
2. `wait_motion_done()`。  
3. `CANMessageDispatcher.stop()`：停收发线程、`bus.shutdown()`、清空订阅。  
4. 标记 `_closed`；总线致命错误时 `on_bus_error` 也会置 `_closed`。

---

## 5. 底层控制详解：CAN 与电机协议

### 5.1 `CANMessageDispatcher`（`comm/can/can.py`）

| 机制 | 说明 |
|------|------|
| **接收线程** | `recv(timeout=0.01)` 循环；拷贝订阅者列表后逐个回调，**隔离单订阅者异常**（记录日志，不拖死总线）。 |
| **发送线程** | `Queue` 出队后 `bus.send`，随后 **busy-wait** 直至 `SEND_INTERVAL_S`（默认约 **0.3 ms**）届满，形成**硬件级发送节拍**。 |
| **队列** | `SEND_QUEUE_SIZE = 2000`；`put_nowait` 满则抛 `queue.Full`（调用方需感知背压）。 |
| **故障策略** | 收/发连续异常达到 `max_consecutive_errors` → `_handle_bus_error`：置 `_running=False`，记录异常，**单次**调用 `on_bus_error`；之后 `send` 抛 `CANError`。 |

**设计意图**：把「多电机 + 多线程同时发」收敛成**单线程顺序发送 + 最小间隔**，这是高可用 CAN SDK 的关键工程细节。

### 5.2 `DataRelay`（`relay.py`）

**职责**：在异步回调环境里实现「**发读命令 → 等到对应回包**」。

- `wait(timeout_s)`：注册 `Event`，阻塞至下一次 `push`。  
- `push(data)`：更新 `_latest`，唤醒**当前所有**等待者（然后清空等待队列）。  
- 可选 `set_sink` 做流式扩展。

**与 A7Motor 的配合**：每个「读命令字节」对应一个 relay；`_on_message` 按 `msg.data[0]` 路由到 relay，`push` 载荷为 `data[1:]`。

### 5.3 `A7Motor` 协议与状态（`arm/a7/motor.py`）

**帧格式（概念）**：`arbitration_id = 电机 ID`，数据区首字节为**命令/寄存器码**，后续为 float（小端 `<f`）或单字节等。

**读类传感器（轮询枚举 `SensorType`）**：

| 含义 | 命令字节（示例） |
|------|------------------|
| 位置 | `0x06` |
| 力矩 | `0x03` |
| 速度 | `0x05` |
| 温度 | `0x5F` |

**内部/控制相关（`InternalSensorType`）**：使能 `0x2B`、加速度读 `0x1D`、位置 Kp `0x19`、速度 Kp `0x17`、速度 Ki `0x1A`、握手 `0x01` 等。

**写类（节选）**：

| 动作 | 形式 |
|------|------|
| 控制模式 | `[0x07, mode_byte]` |
| 使能 | `[0x2A, 0x01/0x00]` |
| 目标角 | `[0x0A] + float` |
| 轨迹速度 | `[0x1F] + float` |
| 加速度 | `[0x20] + float` |
| 减速度 | `[0x21] + float` |
| 清错 | `[0xFE]` |
| 零点标定 | `[0x87]`（后接 `_save_params`） |
| 保存参数 | `[0x0D]` |

**状态模型**：  
- **测量状态**：`angle` / `velocity` / `torque` / `temperature`（轮询更新，带 `timestamp`）。  
- **控制状态**：`control_angle` / `control_velocity` / `control_acceleration`（写指令时同步更新；部分初值来自 `read_initial_state`）。

**轮询线程**：每种 `SensorType` 一线程，循环：`send` 读命令 → `relay.wait(interval_s)` → 解析 float → 更新状态 → `wait(interval_s)` 节流。超时记 debug 日志，不退出线程。

---

## 6. 运动学与笛卡尔规划（`ArmKinetix`）

### 6.1 模型与坐标系

- **URDF**：`arm/kinetix/urdf/a7__{left|right}.urdf` 随包加载，`pin.buildModelFromUrdf`。  
- **TCP**：在末连杆上添加 `OP_FRAME`，平移 `tcp_offset`。  
- **`world_frame`**：`urdf` 为原生；`maestro` 将世界系变换「烘焙」进第一关节 placement（与外部系统对齐）。  
- **关节限位**：自模型 `lowerPositionLimit` / `upperPositionLimit` 初始化，可由 `set_joint_limits` 覆盖。

### 6.2 正解 FK

- `pin.forwardKinematics` + `updateFramePlacements`，取 TCP 帧位姿；姿态输出为 **外旋 ZYX**（`rz, ry, rx`）与 `Pose` 字段一致。

### 6.3 逆解 IK（三级级联）

1. **DLS**：阻尼最小二乘 + Pinocchio 雅可比 + **零空间关节限位梯度**（远离极限）。  
2. **dogbox**：`scipy.optimize.least_squares`，盒约束 + 解析雅可比。  
3. **SLSQP**：`scipy.optimize.minimize`，目标 `0.5‖r‖²`，梯度 `Jᵀr`。

残差：位置误差 + **姿态用 SO(3) log**；姿态项带权重 `ORIENT_WEIGHT`（位置优先）。

`inverse_kinematics_result` 包装为 `IKResult`（Pydantic），供规划里**不抛异常**判断。

### 6.4 `plan_move_l`

- 平移：起点到终点直线，进度 `s_pos` 由**梯形速度曲线**决定。  
- 旋转：`scipy.spatial.transform.Slerp` 球面插值。  
- **总时间** `T = max(T_pos, T_rot)`，保证平移与旋转同时到位。  
- 每个中间位姿：`inverse_kinematics_result`，**seed 链式传递**，失败 `RuntimeError` 带 pose 信息。

---

## 7. 数据类型与 API 契约（`arm/common`）

- **`Pose`**：Pydantic `BaseModel`，`x,y,z`（米）+ `rx,ry,rz`（弧度，外旋 ZYX 与 SciPy 一致）。  
- **`State`**：整机快照（位姿 + 各关节测量/控制量等）。  
- **`WayPoint`**：`pose` + `duration` + `angles`（规划层输出）。  
- **`ControlMode`**：当前仅 `PP`，与驱动映射在 motor 层。

---

## 8. 异常体系（`exceptions.py`）

| 类型 | 用途 |
|------|------|
| `LinkerbotError` | 总基类。 |
| `TimeoutError` | 读寄存器/relay 等待超时。 |
| `CANError` | 总线不可用（致命错误后发送）。 |
| `ValidationError` | 用户参数越界。 |
| `StateError` | 状态不允许（如运动中再 move、电机未响应、初值未就绪）。 |

**自研建议**：区分「可重试」（超时、偶发）与「须停机检查」（总线死、电机缺失）。

---

## 9. 线程与并发模型小结

| 线程 | 职责 |
|------|------|
| Dispatcher recv | 收 CAN → 广播订阅者。 |
| Dispatcher send | 队列发送 + 间隔。 |
| 每电机 × 传感器 | 周期性读，更新缓存。 |
| `MotionTimer` | `threading.Timer` 标记运动结束时刻。 |
| `emergency_stop` | `ThreadPoolExecutor` 并行读位+写位。 |

**注意**：多电机共享同一 Dispatcher；`DataRelay` 按 `(motor_id, cmd)` 逻辑隔离（每电机实例自有 relay 字典）。

---

## 10. 高可用检查清单（可直接当评审表）

**连接与资源**

- [ ] 构造期是否完成**在线检测**与**全状态初读**？  
- [ ] `close()` 是否**先停轮询**再停总线？  
- [ ] 总线错误是否有**单次上报**与**后续发送失败**明确错误？

**CAN**

- [ ] 高频场景是否有**发送队列 + 最小间隔**？  
- [ ] 接收回调是否**不抛未捕获异常**到 recv 线程外？  
- [ ] 队列满是否有背压策略（阻塞/丢弃/报错）？

**协议**

- [ ] 读命令是否**按命令字节关联响应**，避免错帧？  
- [ ] 浮点与字节序是否与驱动一致（此处 `<f`）？

**运动**

- [ ] 是否禁止**运动重叠**（或文档化允许多线程规则）？  
- [ ] `move_l` 是否恢复被临时改写的**速度/加速度**？  
- [ ] IK 是否**链式 seed** 以保证连续路点可解？

**运动学**

- [ ] 模型文件与现场是否一致（左右臂、TCP、world_frame）？  
- [ ] 关节限位是否与机械/控制一致？

---

## 11. 自研 SDK 时的推荐落地顺序（迷你路线图）

1. **L1**：Dispatcher（收线程 + 发队列 + 限流）+ 最小单元测试（mock bus）。  
2. **L2**：单轴 `Motor` + `DataRelay` + 握手与单寄存器读写。  
3. **L3**：7 轴组装 + 构造期验收 + 轮询。  
4. **L4**：URDF + FK；再 IK（先一种 solver，再级联）。  
5. **L5**：`move_j` → `move_p` → `move_l`（先粗后细）。  
6. 最后补充：急停语义、日志与指标、与上层状态机集成。

---

## 12. 关键文件索引（源码阅读顺序建议）

1. `comm/can/can.py` — `CANMessageDispatcher`  
2. `relay.py` — `DataRelay`  
3. `arm/a7/motor.py` — `A7Motor`  
4. `motion_timer.py` — `MotionTimer`  
5. `arm/a7/a7.py` — `A7`  
6. `arm/kinetix/kinetix.py` — `ArmKinetix`  
7. `arm/common/model.py` — `Pose`, `State`, `WayPoint`  
8. `arm/a7/consts.py` — 默认轮询间隔与 MoveL 参数边界  

---

## 13. 附录：与「文档化架构」的差异提示

本 SDK **以代码为准**：若外部宣传「四层/五层架构图」，请对照本章 **第 3 节** 的逻辑分层核对，避免「文档很全、实现很薄」的落差。高可用依赖的是 **Dispatcher + Relay + 构造期验收 + MoveL 恢复参数** 等**可执行细节**，而非仅分层框图。

---

*本文档由源码梳理生成，便于学习；若上游 SDK 更新寄存器或线程策略，请以官方sdk仓库[https://docs.linkerhub.work/sdk/zh-cn/ ]为准同步修订本文。*
