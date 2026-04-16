# 机械臂三大问题深度分析报告

> 分析时间: 2026-03-18

---

## 问题一：点击"回零"后位置偏差较大

### 根本原因

代码链路：`前端 goToZero()` → `socket: go_to_zero` → `controller.go_to_zero()` → `arm_control.py: ArmController.go_to_zero()`

关键问题在 [group_controller.py](file:///data/coding_pro/robot_arm_web/backend/group_controller.py) 第376行的 [go_to_zero](file:///data/coding_pro/robot_arm_web/backend/app.py#340-362)：

```python
def go_to_zero(self, arm_id=None):
    for arm_name, controller in arms_to_control:
        controller.go_to_zero()   # 调用 ArmController 的 go_to_zero
```

而 `ArmController.go_to_zero()` 是让电机运动到**绝对编码器位置 = 0**（物理电机的原始零点），**不是你标定的零点**。

你的标定零点存储在 `zero_offset.json` 中，格式为各电机的偏移量。标定的"零点"实际含义是：

```
逻辑零位 = 物理绝对位置 - zero_offset
→ 物理绝对位置 = 逻辑零位 + zero_offset = 0 + zero_offset = zero_offset
```

所以**回到逻辑零位，应该让每个电机运动到 `zero_offset[motor_id]` 这个绝对位置**，而非 0。

### 修复方案

`GroupController.go_to_zero()` 需修改为：读取 `zero_offset.json`，然后命令每个电机运动到对应的偏移量位置（即 `zero_offset_file` 里存储的绝对位置值）。

```python
def go_to_zero(self, arm_id=None):
    for arm_name, controller in arms_to_control:
        # 正确做法：移动到 zero_offset 存储的绝对位置
        for motor_id, offset in controller.zero_offsets.items():
            controller.move_motor_to(motor_id, offset)   # 移动到物理偏移位置
        print(f"[GroupController] {arm_name} 臂已回到标定零位")
```

---

## 问题二：关节滑块初始位置在边缘，不在中间

### 根本原因

从截图可以看到，关节52、关节56、关节64、关节67等的滑块蓝点在最左或最右边缘。

这是因为滑块的范围是**配置文件中的软限位**（config.py：±2.5 或 ±2.0 rad），而蓝点表示的是**当前关节的逻辑位置**（经zero_offset校正后的值）。

**物理原因：**

```
逻辑位置 = 绝对编码器位置 - zero_offset
```

比如截图中关节52显示 `2.5000 rad`（正好在上限边缘）：
- 说明该关节的物理位置 ≈ `zero_offset[52] + 2.5`
- 也就是**机械臂当前姿态距离零点偏差了2.5rad（约143度）**，接近软限位

这**不是代码bug**，而是：
1. **标定时的姿态就在关节极限附近** → 标定零点选择不合理
2. **或者零点文件和当前机械臂实际姿态不一致**（zero_offset被清空/重置）
3. **或者软件限位范围设置过小**，机械臂正常工作范围就超出了该范围

### 改进建议

1. **重新标定零点**：让机械臂保持在"自然悬挂"或"预设中立位"，再做标定，使各关节处于行程中间
2. **或扩大软限位范围**（config.py中的 `JOINT_LIMITS`），使实际工作范围在软限位中间
3. 可以考虑在UI上加一键显示"当前位置与零点的偏差百分比"，帮助判断

---

## 问题三：笛卡尔控制体验差 — 深度分析与优化方案

### 当前实现的三大缺陷

#### 缺陷1：IK只控制位置，**完全忽略末端姿态（Orientation）**

```python
# cartesian_controller.py 第285行
J = J_full[:3, :]   # 只用线速度部分！舍弃了3行角速度雅可比
error = target - pos  # 只有位置误差，没有姿态误差
```

这意味着：
- 点动 X/Y/Z 时，末端会随意翻转旋转，完全不受控
- 同一个目标位置有无数个姿态解，IK随机收敛到其中一个
- 导致机械臂运动"抽搐"、路径奇怪

#### 缺陷2：IK坐标系**未考虑零点标定偏移**

```python
# CartesianController.__init__ 第331行
pos, rot = forward_kinematics(arm_id, [0.0] * 7)  # 以all-zero关节为原点！
self._origins[arm_id] = pos.copy()
```

但关节实际的"零位"是标定后的位置（[zero_offset](file:///data/coding_pro/robot_arm_web/backend/group_controller.py#621-632) 对应的姿态），不是所有电机编码器值为0时的姿态。这导致：
- FK计算出的坐标系原点与实际零位不一致
- 用滑杆设置坐标时，实际到达位置与预期偏差很大

#### 缺陷3：DLS迭代收敛性差

```python
# 第289行
lambda_sq = damping ** 2   # 固定阻尼 0.01，不自适应
max_step = 0.2             # 步长固定
max_iterations = 200       # 迭代200次但不保证收敛
```

问题：
- 阻尼值固定，在奇异点附近（雅可比退化）会发散
- 无自适应步长，大误差时收敛慢，小误差时过冲
- 无姿态约束，200次迭代可能找到"位置对但姿态奇怪"的解

### 优化方案（分优先级）

#### P0 — 关键修复（最重要）

**修复1：IK改为6-DOF（位置+姿态）**

```python
# 使用全6x7雅可比
J = J_full  # 全部6行

# 误差 = [位置误差3维, 姿态误差3维]
pos_error = target_pos - current_pos
rot_error = rotation_error(target_rot, current_rot)  # 轴角误差
error = np.concatenate([pos_error, rot_error * 0.3])  # 姿态权重可调
```

**修复2：原点坐标系使用实际零位（考虑zero_offset）**

```python
# CartesianController 初始化时，用实际零位关节角而非all-zeros
def _get_calibrated_zero_joints(arm_id):
    # 从 zero_offset.json 读取，计算出 logical_pos=0 时的关节角
    return zero_offsets_as_joint_angles(arm_id)

pos, rot = forward_kinematics(arm_id, _get_calibrated_zero_joints(arm_id))
self._origins[arm_id] = pos.copy()
self._zero_rotations[arm_id] = rot.copy()
```

#### P1 — 质量提升

**改进3：IK使用自适应阻尼（ADLS）**

```python
# 根据雅可比条件数调整阻尼
sv = np.linalg.svd(J, compute_uv=False)
min_sv = sv[-1]
lambda_adaptive = max(0.01, 0.1 * (1 - min_sv / sv[0]))
```

**改进4：IK使用多起始点（避免局部最优）**

```python
# 从当前关节角、多个随机扰动点出发，取误差最小的解
seeds = [current_joints] + [perturb(current_joints) for _ in range(3)]
best = min([ik(seed) for seed in seeds], key=lambda r: r['error_mm'])
```

#### P2 — 体验增强（UI层面）

**改进5：前端实时显示末端位姿（含姿态）**

```
末端位置: X=125.3mm  Y=-45.2mm  Z=310.5mm
末端姿态: Roll=12.5°  Pitch=-8.3°  Yaw=45.0°
```

**改进6：笛卡尔点动加速度插值（平滑运动）**

当前：点动 → 直接发目标关节角 → 电机阶跃运动  
改进：点动 → 插值为多段 → 每段小幅运动 → 视觉上更平滑

**改进7：增加工作空间可视化**

在3D视图中标注当前末端位置的三维坐标轴，显示可达工作空间球面，超出时高亮警告。

---

## 总结优先级

| 优先级 | 问题 | 预计工作量 |
|--------|------|-----------|
| 🔴 P0 | 回零到标定位置而非编码器零位 | 30分钟 |
| 🔴 P0 | IK坐标系考虑zero_offset标定 | 1小时 |
| 🔴 P0 | IK改为6-DOF（位置+姿态同时约束） | 2小时 |
| 🟡 P1 | IK自适应阻尼 | 1小时 |
| 🟡 P1 | 重新标定零点到中立位姿 | 5分钟（用户操作） |
| 🟢 P2 | 末端姿态（Roll/Pitch/Yaw）显示 | 2小时 |
| 🟢 P2 | 工作空间可视化 | 4小时 |
