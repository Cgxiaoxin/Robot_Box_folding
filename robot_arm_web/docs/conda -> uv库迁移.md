# conda -> uv 库迁移教程（robot_arm_web）

本文档用于把当前项目从“主要依赖 pip + conda 环境管理”迁移到“uv 管理项目依赖”。

## 0. 当前状态确认（你现在的情况）

- 你已经安装了 `uv 0.10.9`。
- 当前项目目录：`/data/coding_pro/robot_arm_web`
- 当前 Python：`3.13.11`
- `pyproject.toml` 中要求：`requires-python = ">=3.13"`
- `.python-version` 中记录：`3.13`

## 1. 如何查看 Python 版本（常用方法）

### 方法 A：查看当前激活解释器版本

```bash
python --version
which python
```

说明：
- `python --version` 看版本号
- `which python` 看当前命令对应的解释器路径

### 方法 B：查看 conda 某个环境里的 Python 版本

```bash
conda activate robot
python --version
```

如果你不想切换环境，也可以直接：

```bash
conda run -n robot python --version
```

### 方法 C：查看 uv 可识别/可下载的 Python 列表

```bash
uv python list
```

## 2. 是否要新建环境？

建议新建，不直接改旧 `robot` 环境。推荐保留双环境：

- `robot`：旧代码稳定运行（回滚保障）
- `robot_arm_web/.venv`：uv 迁移和 SDK 重写开发

## 3. 迁移流程（推荐）

以下命令默认在项目根目录执行：

```bash
cd /data/coding_pro/robot_arm_web
```

### 第 1 步：备份旧依赖（可选但强烈建议）

```bash
conda activate robot
pip freeze > /data/coding_pro/requirements.robot.bak.txt
pip list --not-required > /data/coding_pro/requirements.robot.top.txt
```

### 第 2 步：确保项目有基础配置

你已执行过 `uv init`，可跳过。如果要再次确认：

```bash
uv init
```

### 第 3 步：创建项目虚拟环境（使用 3.13）

```bash
uv venv --python 3.13
source .venv/bin/activate
python --version
```

### 第 4 步：安装核心 SDK（A7/A7 Lite）

从 PyPI：

```bash
uv add linkerbot-py
```

或从 Git 仓库：

```bash
uv add "linkerbot @ git+https://github.com/linker-bot/linkerbot-python-sdk"
```

建议二选一，不要同时装两种来源。

### 第 5 步：迁移旧依赖

如果你有旧依赖文件：

```bash
uv add -r /data/coding_pro/robot_arm_web/requirements.txt
```

如果需要导入旧环境导出的依赖：

```bash
uv add -r /data/coding_pro/requirements.robot.bak.txt
```

> 注意：`pip freeze` 导出的版本可能包含平台绑定包，导入后若报冲突，优先保留业务必需包，逐步精简。

### 第 6 步：锁定与同步

```bash
uv lock
uv sync
```

### 第 7 步：运行项目

推荐用 uv 执行，避免解释器混淆：

```bash
uv run python test.py
uv run python arm_control.py
```

## 4. A7 / A7 Lite 重写建议（简版）

先不要直接大改业务层，建议先做一个 SDK 适配层，例如：

- `sdk_adapter.py`：封装连接、上使能、回零、关节移动、状态读取、急停
- 业务代码先调用适配层接口
- 先跑通 A7 Lite，再切 A7 做参数校准

这样可以降低重写风险，便于回归测试。

## 5. 常见问题

### Q1: 我还需要 conda 吗？

可以保留 conda 作为系统级 Python 管理，但项目依赖建议统一交给 uv 的 `.venv`。

### Q2: 为什么 `python` 版本和预期不一致？

通常是因为没激活目标环境，或当前 shell 仍指向 conda/base。可用以下命令快速确认：

```bash
which python
python --version
```

### Q3: 迁移后如何回滚？

直接切回旧环境运行旧代码：

```bash
conda activate robot
python your_old_script.py
```

---

参考：

- Linkerbot 安装文档：<https://docs.linkerhub.work/sdk/zh-cn/guide/installation.html>
- Linkerbot Python SDK 仓库：<https://github.com/linker-bot/linkerbot-python-sdk>
