# Robot Box Folding

用于 A7 双臂机械臂 + LinkerHand 灵巧手的纸箱折叠/演示项目。

当前仓库已经包含一套可直接联调的 Web 控制台，支持双臂控制、执行层动作编排、双手灵巧手实例管理，以及示教/轨迹回放。

## 当前能力

- 双臂 A7 Web 控制：连接、初始化、去使能、回零、急停
- 关节手动控制与笛卡尔控制
- 轨迹录制、编辑、保存、回放
- 执行层统一入口：`ExecutionRequest / ExecutionResult / action_registry`
- 灵巧手子系统：`HandCapabilities / HandCommand / HandState / HandService`
- 支持 LinkerHand 多型号、多实例、多臂绑定
- 前端支持左右手独立手势控制、速度缩放、力度缩放

## 目录说明

- `robot_arm_web/`: 主 Web 控制系统
- `robot_arm_web/backend/`: Flask、Socket.IO、执行层、控制器、手部服务
- `robot_arm_web/frontend/`: Web UI
- `robot_arm_web/tests/`: 后端测试
- `docs/`: 开发文档、任务规划、接口说明
- `src/`: A7 / LinkerHand 相关 SDK 与依赖源码

## 快速开始

### 1. 进入 Web 控制目录

```bash
cd /data/coding_pro2/Robot_Box_folding/robot_arm_web
```

### 2. 安装依赖

优先使用 `uv`：

```bash
uv sync
```

如果本机没有 `uv`，也可以使用：

```bash
pip install -r requirements.txt
```

### 3. 配置 CAN

至少需要双臂 CAN：

- 右臂: `can0`
- 左臂: `can1`

如果还要接灵巧手，可继续使用例如 `can2` / `can3`。

项目内已提供辅助脚本：

```bash
./setup_can.sh
```

### 4. 启动服务

推荐直接使用启动脚本：

```bash
./start.sh
```

也可以手动启动：

```bash
python backend/app.py
```

启动后访问：

`http://127.0.0.1:5000`

## 灵巧手配置

项目兼容两种方式：

### 方式 1：旧单手配置

使用环境变量：

- `HAND_ENABLE`
- `HAND_TYPE`
- `HAND_JOINT`
- `HAND_CAN`
- `HAND_MODBUS`

### 方式 2：推荐的多手配置 `HANDS_JSON`

示例：

```bash
export HANDS_JSON='{
  "hands": [
    {
      "hand_id": "right_hand",
      "vendor": "LinkerHand",
      "model": "L6",
      "side": "right",
      "driver": "linkerhand_python",
      "transport": { "can": "can2", "modbus": "None" },
      "mount_arm_id": "left"
    },
    {
      "hand_id": "left_hand",
      "vendor": "LinkerHand",
      "model": "L10",
      "side": "left",
      "driver": "linkerhand_python",
      "transport": { "can": "can3", "modbus": "None" },
      "mount_arm_id": "right"
    }
  ]
}'
```

说明：

- `side` 表示手的左右手实例
- `mount_arm_id` 表示绑定的机械臂实例，当前使用 `left` / `right`
- 前端会按 `side=left/right` 自动显示为左右手控制卡片

## 执行层接口

后端已经提供统一执行入口：

- HTTP: `POST /api/execution/execute`
- Socket.IO: `execution:execute`

手部调试接口：

- `GET /api/hands/state`
- `GET /api/hands/capabilities`
- `GET /api/hands/bindings`
- `POST /api/hands/execute`

## 前端控制台

`robot_arm_web/frontend/` 当前主要包含：

- 双臂实时监控
- 关节控制
- 笛卡尔控制
- 轨迹与示教
- 双手灵巧手控制

灵巧手控制区支持：

- 左手 / 右手独立控制
- 手势选择
- 仅手部 / 先手后臂 / 先臂后手
- 自动或手动绑定机械臂
- 速度缩放、力度缩放

## 测试

在仓库根目录执行：

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 pytest robot_arm_web/tests
```

## 相关文档

- `docs/A7_SDK_使用文档.md`
- `docs/sdk执行层任务规划_4d9dddf6.plan.md`
- `.cursor/臂手联合集成架构_59b23d99.plan.md`

## 注意事项

- 本项目默认面向本地联调和演示，不是生产部署版本
- CAN、使能、回零、急停均会触发真实硬件动作
- 请务必在安全区域内操作，并确认急停链路可用
- 接入新型号灵巧手时，优先使用 `HANDS_JSON`，不要继续扩展旧的单手全局配置


