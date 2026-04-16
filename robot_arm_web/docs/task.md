# 机械臂 Web 控制系统任务复盘（接口重构版）

> 时间：2026-04-01  
> 项目：`robot_arm_web`  
> 角色：后端 + 前端联动改造（Socket.IO 协议重构、控制链路统一、UI 重构与联调）

---

## 一、项目背景与目标

原有系统在笛卡尔控制链路存在以下问题：

- 事件协议分散（`cartesian_move_to` / `smooth_cartesian_move` / `cartesian_jog` 各自处理）
- 前后端语义不统一，执行状态不可视化不足
- 执行中并发与重复提交缺少统一约束
- 错误提示不标准，缺少错误码体系
- UI 模块虽可用，但不满足“模式化控制 + 生命周期可观测”的交付要求

本次目标是严格对齐两份设计文档，完成**统一接口重构 + 旧协议兼容 + 前端控制面板重构**。

---

## 二、今日完成的核心任务清单（按阶段）

## 阶段 1：后端接口重构与执行链路统一

### 1) 建立统一运动模型

新增后端模块：

- `backend/motion_types.py`
- `backend/motion_service.py`

实现内容：

- 统一 `motion_type`：`CARTESIAN_PTP` / `CARTESIAN_LINEAR` / `CARTESIAN_JOG`
- 统一 `frame`：`BASE` / `TOOL` / `USER`
- 统一生命周期状态：`accepted/planning/running/completed/failed/cancelled/paused`
- 统一错误码体系：`ARM_NOT_CONNECTED`、`IK_UNREACHABLE`、`IK_LOW_ACCURACY`、`JOINT_LIMIT_REJECTED`、`CONSTRAINT_VIOLATION`、`COMMAND_PREEMPTED`、`INTERNAL_ERROR` 等

### 2) 新增统一 Socket 事件

在 `backend/app.py` 新增并接入：

- `motion:execute`
- `motion:cancel`
- `motion:pause`
- `motion:resume`
- `motion:query`

并将旧接口内部映射到新链路：

- `cartesian_move_to` -> `CARTESIAN_PTP`
- `smooth_cartesian_move` -> `CARTESIAN_LINEAR`
- `cartesian_jog` -> `CARTESIAN_JOG`

兼容策略：

- 旧接口保留可用
- 旧接口回包附带 `deprecated: true`

### 3) 执行控制与状态广播

统一由 `MotionService` 管理命令：

- 执行中禁止重复提交（新协议）
- 支持取消/暂停/恢复
- 状态分阶段广播（accepted/planning/running/...）
- 输出统一失败消息（错误码 + 中文 message + detail）

### 4) 参数与目标校验增强

新增目标结构校验逻辑：

- `PTP/LINEAR` 必须是合法 `target.pose`
- `JOG` 必须是合法 `target.delta`
- 非法 payload 统一 `motion:rejected + CONSTRAINT_VIOLATION`

---

## 阶段 2：前端控制台改造（笛卡尔控制页）

涉及文件：

- `frontend/index.html`
- `frontend/js/main.js`
- `frontend/css/style.css`

### 1) UI 结构重构

新增并重构模块：

- 模式栏（PTP / LINEAR / JOG）
- Frame 选择（BASE / TOOL）
- 目标位姿输入（X/Y/Z mm + Rx/Ry/Rz deg）
- 约束参数区（线速度/线加速度/角速度/角加速度）
- 执行控制区（执行/暂停/继续/取消/急停）
- Jog 区（XYZ + RPY，支持长按连续）
- 反馈区（进度、当前姿态、目标姿态、误差、最近错误）

### 2) 单位体系统一

- 前端输入：`mm / deg`
- 发包到后端：`m / rad`

### 3) 生命周期可视化

前端订阅并渲染：

- `motion:accepted`
- `motion:status`
- `motion:paused` / `motion:resumed`
- `motion:completed`
- `motion:failed`
- `motion:cancelled`
- `motion:rejected`

并在执行中控制输入禁用，防止并发误操作。

---

## 阶段 3：联调问题定位与修复（当天追加）

### 问题 1：点击 PTP/LINEAR/JOG 导致页面误切换、控制块消失

原因：

- 模式按钮复用 `.tab-btn`，被全局 Tab 切换监听误识别

修复：

- Tab 监听改为仅绑定 `data-tab` 按钮
- 模式按钮与页面 Tab 逻辑解耦

### 问题 2：模式切换“看起来没变化”

原因：

- 仅切换了 `motion_type` 变量，UI 无显式联动

修复：

- 增加模式说明文案动态切换
- JOG 模式下隐藏位姿输入区，突出点动区
- 模式切换时执行按钮文案和可用状态联动

### 问题 3：JOG 模式语义错配

原因：

- `executeMotion()` 在 JOG 下仍发送 `target.pose`，后端 `_run_jog` 需要 `target.delta`

修复：

- JOG 模式执行按钮禁用并提示“请使用下方点动按钮”
- 点动按钮统一发送 `motion:execute + target.delta`
- 后端补目标结构校验，避免错包进入执行链

---

## 三、代码审查结果（本次改造）

重点审查维度：

- 协议一致性（前后端 payload 结构是否对齐）
- 兼容性（旧事件是否仍可用）
- 状态机一致性（命令生命周期与按钮行为）
- 防并发策略（执行中二次提交处理）

审查结论：

- 已修复核心高优先级问题：JOG 语义错配、模式切换误触页签
- 协议主链路可用，兼容层可用
- 存在可优化项：历史笛卡尔滑杆旧逻辑仍有残留代码，可后续清理

---

## 四、测试与验证

### 1) 静态/语法检查

已执行：

- `python -m py_compile backend/app.py backend/motion_service.py backend/motion_types.py`
- `node --check frontend/js/main.js`
- lints 检查：无新增错误

### 2) 联调验证项（已覆盖/建议回归）

- PTP 小位移执行，状态闭环完整
- LINEAR 执行，进度持续更新
- JOG 按钮长按连续发送
- 执行中重复提交被拦截
- 取消/暂停/恢复事件链可达
- 旧事件调用仍可用（并带 `deprecated`）

---

## 五、产出价值（可用于面试表述）

### 1) 技术价值

- 将分散运动接口收敛为统一协议，降低前后端耦合
- 建立可观测执行生命周期，提升可维护性与可排障性
- 完成旧接口兼容，降低线上迁移风险
- 补齐错误码与文案映射，提高现场可用性

### 2) 业务价值

- 降低“点了没反应/状态不明”的操作风险
- 提升示教与调试效率
- 为后续轨迹抢占、细粒度监控、参数标定提供统一基础

---

## 六、面试 STAR 版本（可直接复述）

### S（Situation）

机械臂 Web 控制项目中，笛卡尔控制接口分散、状态不可视、错误提示不统一，导致联调和现场操作成本高。

### T（Task）

在不破坏旧功能的前提下，完成统一事件协议重构，支持 PTP/LINEAR/JOG 三种语义，补齐生命周期状态、错误码体系与前端可视化。

### A（Action）

- 新建统一命令模型和 `MotionService` 执行层  
- 在后端接入 `motion:execute/cancel/pause/resume/query`  
- 将旧事件映射到新协议并保留兼容回包  
- 重构前端笛卡尔页（模式栏、frame、约束参数、反馈区、点动区）  
- 统一单位转换（mm/deg -> m/rad）  
- 修复联调中的模式误切页与 JOG payload 错配问题  
- 增加目标结构校验，防止非法请求误执行  

### R（Result）

- 协议从“多入口分散”升级到“单入口统一 + 兼容旧协议”  
- 命令状态从不可观测升级为完整生命周期可视化  
- 前端模式交互与后端执行语义一致  
- 通过语法与 lint 检查，联调闭环可达

---

## 七、可标注在简历中的关键词

- Socket.IO 实时协议重构  
- 机械臂任务空间控制（PTP/LINEAR/JOG）  
- 前后端协议设计与兼容迁移  
- 状态机与命令生命周期可视化  
- 错误码体系与可观测性建设  
- 工程化联调与线上兼容策略

