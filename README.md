用于 A7 双臂机械臂的折纸盒（纸箱折叠）演示项目。

## 项目目标

- 用于录制、回放和调试折纸盒动作流程
- 支持双臂关节控制、笛卡尔控制、示教编程
- 提供 Web 控制界面，便于快速做 demo

## 主要目录

- `robot_arm_web/`: Web 控制系统（后端 + 前端）
- `docs/`: SDK 文档、协议文档、任务记录
- `src/`: 手部 SDK / 相关依赖代码

## 快速开始

1. 进入项目

```bash
cd /data/coding_pro/Robot_Box_folding2/robot_arm_web
```

2. 安装依赖

```bash
pip install -r requirements.txt
```

3. 启动后端服务

纸箱折叠技能demo
```bash
python backend/app.py
```

4. 浏览器访问

`http://127.0.0.1:5000`

## 说明

- 默认用于本地联调与演示，不是生产部署版本
- CAN、使能、急停等硬件动作请在安全环境下操作
- 详细接口与协议请看 `docs/A7_SDK_使用文档.md`


