# 机械臂控制系统 — 健壮性深度分析报告

> 分析时间: 2026-03-18 | 系统版本: robot-arm-web v0.1.0

---

## 🚨 当前紧急问题（已修复 / 待修复）

### 问题1：多实例并发冲突 ✅ 已修复（start.sh）

**现象：** 用户多次运行 `bash start.sh`，导致 **3 个 `python app.py` 进程同时运行**，都监听端口 5000，互相抢占 CAN 接口和 Socket.IO 连接。

```
PID 778863  python backend/app.py    # 旧进程（我们后台启动的）
PID 779263  python3 backend/app.py   # 第二次 start.sh 启动
PID 781926  python3 backend/app.py   # 第三次 start.sh 启动
```

**根因：** 旧 [start.sh](file:///data/coding_pro/robot_arm_web/start.sh) 没有端口占用检测，每次直接启动新进程。

**已修复：** 新 [start.sh](file:///data/coding_pro/robot_arm_web/start.sh) 启动前检测端口 5000 是否被占用，自动 `kill -9` 旧进程。

---

### 问题2：Python 环境错误（uv 虚拟环境未使用）✅ 已修复（start.sh）

**现象：** 旧 [start.sh](file:///data/coding_pro/robot_arm_web/start.sh) 使用 `python3`，指向系统 Miniconda 环境（**Python 3.13**）。但项目通过 uv 管理，`.venv` 存在且要求 **Python ≥3.10, <3.13**。

```toml
# pyproject.toml
requires-python = ">=3.10,<3.13"
```

```bash
# 旧 start.sh（错误）
python3 backend/app.py  # → /home/alex/miniconda3/bin/python3 (3.13!)

# 新 start.sh（正确）
uv run python backend/app.py  # → .venv 中的 Python (3.11/3.12)
```

**风险：** Python 3.13 与 `eventlet==0.35.0` 存在兼容性问题（eventlet 官方已废弃，且对 3.13 的支持有已知 bug）。可能导致 `RLock` 无法被 greened 等底层问题。

**已修复：** [start.sh](file:///data/coding_pro/robot_arm_web/start.sh) 改为优先使用 `uv run python`，并在启动时自动运行 `uv sync`。

---

### 问题3：.pyc 缓存导致代码修改不生效 ✅ 已修复（start.sh）

**现象：** 修改 [app.py](file:///data/coding_pro/robot_arm_web/backend/app.py) 后重启，服务仍然运行旧代码（`tpool` 报错在文件已修改后仍然出现）。

**根因：** Python 的 `__pycache__` 字节码缓存在某些情况下不会自动失效（文件时间戳异常时）。

**已修复：** [start.sh](file:///data/coding_pro/robot_arm_web/start.sh) 每次启动前自动清除全部 [.pyc](file:///data/coding_pro/robot_arm_web/__pycache__/arm_control.cpython-313.pyc) / `__pycache__`。

---

### 问题4：eventlet RLock 阻塞事件循环 ✅ 已修复（app.py）

**现象：** 浏览器 Socket.IO 每 1-2 秒断连重连，按钮点击事件丢失，机械臂无法连接。

**根因：**
```
终端警告: 1 RLock(s) were not greened
```
`GroupController.__init__` 创建了 `threading.RLock`，但由于加载时序问题，eventlet 未能将其替换为协作式锁。[_state_update_loop](file:///data/coding_pro/robot_arm_web/backend/app.py#1368-1388) 每 200ms 调用 `controller.read_positions()`，内部 `with self._lock` 真实阻塞 eventlet 事件循环，导致 Socket.IO 心跳超时。

**已修复：** 使用 [_run_in_thread()](file:///data/coding_pro/robot_arm_web/backend/app.py#1353-1366) 在真实 OS 线程中执行 [read_positions()](file:///data/coding_pro/robot_arm_web/backend/group_controller.py#450-459)，同时用 `eventlet.sleep(0.01)` 轮询让出事件循环。更新间隔从 200ms 改为 500ms。

---

## 📊 系统架构分析

```
Browser (Socket.IO client)
    ↕ WebSocket
Flask + python-socketio + eventlet WSGI
    ↕
GroupController (threading.RLock)
    ↕ CAN bus (python-can)
ArmController × 2 (left/right)
    ↕ hardware
Motor 51-57 (can0) | Motor 61-67 (can1)
```

### 架构层面的深层问题

| 问题 | 严重程度 | 说明 |
|------|---------|------|
| **eventlet 已废弃** | 🔴 高 | eventlet 官方声明进入 bugfix-only 模式，强烈建议迁移 |
| **threading + eventlet 混用** | 🔴 高 | `groupcontroller.py` 用 `threading.RLock`，与 eventlet greenlet 模型冲突 |
| **无进程管理** | 🔴 高 | 无 systemd/supervisor，崩溃不自动重启，多实例无防护 |
| **前端依赖 CDN** | 🟡 中 | Socket.IO、Three.js 等都来自 CDN，离线/弱网环境下页面无法使用 |
| **CAN 连接无心跳检测** | 🟡 中 | 机械臂断电后无法自动感知，需要手动重连 |
| **无日志持久化** | 🟡 中 | 所有日志只打印到 stdout，重启即丢失 |
| **无认证** | 🟡 中 | Web 界面完全开放，局域网内任何人可控制机械臂 |

---

## ⚡ 性能分析

### 状态更新循环（已部分修复）

```python
# 修复前：每 200ms 直接阻塞
controller.read_positions()   # 阻塞事件循环 ~50ms

# 修复后：每 500ms 在线程中执行
_run_in_thread(controller.read_positions)  # 非阻塞
```

**仍存在的性能问题：**
1. **[_broadcast_state()](file:///data/coding_pro/robot_arm_web/backend/app.py#1325-1350) 计算 FK（正运动学）** — 每次广播都为两条臂做 FK 计算，计算量不小
2. **广播到所有客户端** — 若多标签页打开，状态重复推送多份
3. **没有状态 diff** — 每次广播完整状态，而不是只推送变化的字段

### CAN 通信

- 读取关节角度为轮询方式（[read_all_positions()](file:///data/coding_pro/robot_arm_web/arm_control.py#468-473)），每次逐个电机查询，无批量读写
- 7 个电机 × 2 条臂 = 14 次 CAN 请求/周期

---

## 🎨 使用体验分析

| 问题 | 影响 |
|------|------|
| 3D 模型加载失败 | URDF 模型无法从 CDN 加载，3D 预览区空白 |
| 连接后需手动点"连接"按钮 | 无法启动时自动连接；重启服务后需要重新手动连接 |
| 错误信息不友好 | 终端报错不会显示在前端；用户看到的只是"未连接" |
| 按钮无防抖 | 连击"连接"可能触发多次连接请求 |
| 无历史曲线 | 实时监控只有当前值，没有历史趋势图 |

---

## 🗺️ 优化路线图（优先级排序）

### P0 — 立即修复（本次已完成）
- [x] [start.sh](file:///data/coding_pro/robot_arm_web/start.sh) 多实例防护
- [x] [start.sh](file:///data/coding_pro/robot_arm_web/start.sh) 切换到 `uv run python`
- [x] [start.sh](file:///data/coding_pro/robot_arm_web/start.sh) 自动清除 pyc 缓存
- [x] [_state_update_loop](file:///data/coding_pro/robot_arm_web/backend/app.py#1368-1388) 阻塞 eventlet 事件循环

### P1 — 近期优化（1-2周）

**1. 迁移离 eventlet → asyncio**
```python
# 推荐方案：用 aiohttp + python-socketio 的 asgi 模式替代 Flask+eventlet
# pyproject.toml 中改为:
"aiohttp>=3.9",
"python-socketio[asyncio]>=5.11",
```
彻底解决 threading/greenlet 冲突，同时兼容 Python 3.13+。

**2. 前端资源本地化**
```html
<!-- 当前（CDN，离线不可用）-->
<script src="https://cdn.socket.io/4.7.2/socket.io.min.js"></script>

<!-- 改为本地 vendor -->
<script src="vendor/socket.io.min.js"></script>
```

**3. 进程管理**
```bash
# 使用 systemd service 或写一个 stop.sh
# stop.sh
pkill -f "uv run python backend/app.py"
```

**4. CAN 连接心跳检测**
```python
# 定期发送 ping 帧，检测机械臂是否在线
# 超时自动标记为断开，UI 显示警告
```

### P2 — 中期优化（1个月）

**5. 状态 diff 广播**
```python
# 只广播变化的字段，减少带宽
if state != last_state:
    sio.emit('state:update', diff(last_state, state))
    last_state = state
```

**6. 前端添加重连后自动恢复连接**
```javascript
socket.on('connect', () => {
    if (appState.wasConnected) {
        connectArms(); // 自动重连机械臂
    }
});
```

**7. 错误信息推送到前端**
```python
# 目前只 print，改为同时推送到前端日志
sio.emit('log', {'level': 'error', 'msg': str(e)})
```

**8. 历史曲线（可选）**
- 使用 Chart.js 为每个关节绘制最近 30s 的位置曲线
- 帮助排查关节抖动、超限等问题

---

## 📋 正确的启动流程

```bash
# 1. 配置 CAN（每次开机后运行一次）
./setup_can.sh

# 2. 启动系统（已修复，自动处理多实例、环境、缓存）
bash start.sh

# 3. 停止系统
Ctrl+C   # 或 pkill -f "uv run python backend/app.py"
```

> **注意：** 不要在同一机器上多次运行 `bash start.sh`！新版本已有保护，但仍需注意。
