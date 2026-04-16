# CAN 总线与电机 ID 速记（现场版）

## 先看结论

- 不要默认 `can0/can1` 永远对应双臂。
- 单个多通道 USB-CAN 盒在重插、换 USB 口、重启后，`canX` 编号可能漂移。
- 先扫描再固定映射，避免“之前能通、现在全不通”。

## 电机 ID 归属（固定）

- 一组：`[51, 52, 53, 54, 55, 56, 57]`
- 二组：`[61, 62, 63, 64, 65, 66, 67]`

## 今日现场确认（2026-04-13）

- 灵巧手 L6：`can0`（`find_linker_hand.sh` 有稳定响应）
- 双臂可用组合：`left_channel=can1`、`right_channel=can2`

## 推荐排查顺序

```bash
# 1) 全口位重置（会自动发现 can*）
./reset_can.sh

# 2) 扫描灵巧手所在口位
bash ../src/linkerhand-ros2-sdk/find_linker_hand.sh

# 3) 启动服务后做 arm_connect 口位扫描，再固定 left/right channel
```

## 常用抓包命令

```bash
candump can0
candump can1
candump can2
candump can3
```
