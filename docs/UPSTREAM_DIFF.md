# UPSTREAM_DIFF

本文件用于记录 `Robot_Box_folding/robot_arm_web` 相对原始 `robot_arm_web` 的增量改动，便于后续回迁与对账。

## 2026-04-13

### 本次改动文件
- `robot_arm_web/backend/motion_types.py`
- `robot_arm_web/backend/motion_service.py`
- `robot_arm_web/tests/test_execution_contract.py`
- `robot_arm_web/tests/test_action_registry.py`
- `robot_arm_web/backend/config.py`
- `robot_arm_web/backend/action_registry.yaml`
- `robot_arm_web/backend/adapters/hand_adapter.py`
- `robot_arm_web/backend/app.py`

### 改动目的
- 新增执行层统一数据模型：`ExecutionRequest` / `ExecutionResult`。
- 扩展执行错误码（`OK`、`INVALID_REQUEST`、`UNSUPPORTED_ACTION`、`SDK_ERROR`、`ACTION_TIMEOUT` 等）。
- 在 `MotionService` 中新增 `execute_action()` 统一入口，先打通“上层请求 -> 机械臂动作提交 -> 结构化结果返回”链路。
- 在 `app.py` 暴露统一执行入口：
  - Socket.IO: `execution:execute` -> `execution:result`
  - HTTP: `POST /api/execution/execute`

### 回迁建议
- 可回迁：`motion_types.py` 的执行模型和错误码扩展。
- 需评估后回迁：`motion_service.py` 新接口（是否与原项目协议冲突）。
- 可选回迁：`app.py` 新路由与新事件（取决于原项目前端/上层是否接入执行层协议）。

### 已知限制
- 当前 `execute_action()` 仅完成机械臂路径提交，灵巧手 `HandAdapter` 尚未接入，结果里以 `not_integrated_yet` 标识。
- `execute_action()` 当前返回“已受理/拒绝”即时结果，未等待动作最终完成（后续在阶段B/C增强）。

## 2026-04-13（第1天继续）

### 本次改动文件
- `robot_arm_web/backend/adapters/__init__.py`
- `robot_arm_web/backend/adapters/a7_adapter.py`
- `robot_arm_web/backend/adapters/hand_adapter.py`
- `robot_arm_web/backend/action_registry.yaml`
- `robot_arm_web/backend/motion_service.py`

### 改动目的
- 新增 `A7Adapter` 与 `HandAdapter` 分层，降低执行层与底层实现耦合。
- 增加 `action_registry.yaml`，提供 `action_id -> arm_plan/hand_plan/sync_policy` 的配置入口。
- `execute_action()` 改为优先读取注册表，并支持请求参数覆盖默认配置。
- 增加 `sync_policy` 分支骨架（`arm_then_hand` / `hand_then_arm` / `parallel`）。
- 增加动作超时看门狗与 `safe_abort_action` 降级执行骨架。
- 增加执行层回归测试（契约与注册表路由）。
- 第三步微调落地：L6 + CAN + Python 直连 `LinkerHandApi`。
- `arm_then_hand` 从占位变为“机械臂完成后执行 hand_plan”。

### 回迁建议
- 可回迁：`adapters` 分层与注册表机制（对原项目侵入低）。
- 谨慎回迁：`execute_action` 新路由逻辑（需与原协议对齐后再合并）。

### 当前限制
- 手侧已接入 `LinkerHandApi` 直连，但依赖现场 CAN 与 L6 设备就绪；无设备时 `is_ready=false`。
- `execution:execute` 目前仍返回“受理态”即时结果，动作最终状态通过 `motion:*` 事件观察。
- `parallel` 当前是“先执行手再提交通道”的近似并行，后续可升级为真正并发执行。

## 2026-04-13（CAN 联调排障增强）

### 本次改动文件
- `robot_arm_web/reset_can.sh`
- `robot_arm_web/docs/can_notes.md`

### 改动目的
- 将 `reset_can.sh` 从固定 `can0/can1` 升级为“自动发现所有 `can*` 接口并全量重置”。
- 重置后输出每个口位的 `state/RX/TX`，用于快速判断“接口是否 UP 但无实际通信”。
- 在 `can_notes.md` 明确“单个多通道 USB-CAN 盒下 `canX` 编号可能漂移”，避免继续按旧静态映射联调。
- 记录本次现场已验证口位：`HAND_CAN=can0`，双臂组合可用 `left_channel=can1`、`right_channel=can2`。

### 回迁建议
- 可回迁：`reset_can.sh` 的多口位重置逻辑（通用性更强）。
- 可回迁：`can_notes.md` 的动态映射说明（降低现场误判概率）。
