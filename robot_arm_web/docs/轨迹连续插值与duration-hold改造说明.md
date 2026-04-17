# 轨迹连续插值与 `duration/hold` 改造说明

## 背景

原轨迹播放逻辑采用逐点阻塞执行：

- 每个点使用 `move_j(..., blocking=True)` 到位；
- 到位后再 `sleep(delay)`。

该方式在 `A -> B -> C` 连续轨迹上会出现点间停顿，导致体感不流畅。

## 改造目标

- 连续运动场景：消除点间停顿，按段连续插值运动。
- 工艺停留场景：只在显式需要时停留。
- 数据语义统一：`duration` 表示段时长，`hold` 表示点后停留时间。
- 兼容历史数据：保留 `delay` 读取与过渡写出能力。

## 语义定义

- `duration`: 从当前点到下一点的运动时长（秒）。
- `hold`/`dwell`: 到达当前点后的额外停留时长（秒）。
- `delay`（兼容字段）:
  - 读取时：若没有 `duration`，回退读取 `delay`。
  - 写入时：暂时写出 `delay = duration`，兼容旧逻辑。

## 场景策略

### 连续运动场景

- `hold = 0`（默认）；
- `duration` 用于连续插值段时长；
- 播放器不再执行每点固定停顿。

### 工艺停留场景（抓取/拍照等）

- 在需要停留的点上设置 `hold > 0`；
- 播放器仅在该点执行停留，其他点不暂停。

## 代码改动概览

### `backend/config.py`

新增轨迹播放参数：

- `TRAJECTORY_PLAYBACK_CONTROL_HZ`（默认 `50`）
- `TRAJECTORY_MIN_SEGMENT_DURATION_S`（默认 `0.04`）
- `TRAJECTORY_MAX_SEGMENT_DURATION_S`（默认 `2.0`）

### `backend/group_controller.py`

- 播放线程改为段插值执行：
  - 根据 `duration`（或回退 `delay`）计算段时长；
  - 按固定控制频率下发 `move_j(..., blocking=False)`；
  - 保留暂停/停止可中断逻辑；
- 显式支持 `hold/dwell` 停留；
- 删除原“每点后 `sleep(delay)`”逻辑。

### `backend/trajectory_engine.py`

- `TrajectoryPoint` 字段升级：
  - `delay` -> `duration` + `hold`
- 加载兼容：
  - `duration` 优先，缺失时回退 `delay`
- 保存过渡：
  - 写 `duration`、`hold`，并兼容写 `delay=duration`
- 总时长计算改为：
  - `sum(duration + hold) / speed_multiplier`

### `backend/drag_teach_service.py`

- 录制关键帧默认值从 `delay=1.0` 调整为 `duration=0.1`；
- 接口兼容 `delay` 入参（自动映射为 `duration`）；
- 录制点写入 `duration`、`hold`，并兼容写 `delay=duration`；
- 流式采样参数优化：
  - `_joint_delta_threshold`: `0.02 -> 0.01`
  - `_max_sample_gap_s`: `0.5 -> 0.1`

## 迁移与兼容建议

1. 前端新建/编辑轨迹时，优先使用 `duration` 与 `hold`。
2. 旧文件无需迁移即可播放（播放器已兼容 `delay`）。
3. 观察稳定后，可在后续版本逐步移除 `delay` 写出。

## 推荐参数

- 控制频率：`50Hz`
- 最小段时长：`0.04s`
- 最大段时长：`2.0s`
- 关键帧默认 `duration`：`0.1s`
- 默认 `hold`：`0.0s`

## 验证要点

- 连续轨迹应无明显 A->B 停顿；
- 指定 `hold` 的点应有可预期停留；
- 旧轨迹文件（仅 `delay`）可正常播放；
- 暂停/恢复/停止在播放过程中可及时响应。
