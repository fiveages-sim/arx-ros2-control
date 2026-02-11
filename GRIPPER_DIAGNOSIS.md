# 夹爪控制诊断报告

## 问题现象

夹爪夹到东西后无法再控制，重启程序依然无法恢复。

---

## 1. 夹爪控制架构总览

```
ROS2 Topic 指令
  │
  ▼
AdaptiveGripperController (控制器层)
  │  两种模式:
  │    - 直接位置控制 (/<joint_name>/position_command, Float64)
  │    - 开关控制 (/<controller_name>/target_command, Int32, 0=关/1=开)
  │  写入: gripper_joint/position (command interface)
  │  读取: gripper_joint/position, effort (state interface)
  │
  ▼
ArxX5Hardware (hardware_interface 层)
  │  read():  SDK.get_joint_state() → gripper_position/velocity/effort_states_
  │  write(): gripper_position_commands_ → SDK.set_joint_cmd()
  │
  ▼
Arx5JointController (ARX SDK 层)
  │  set_joint_cmd() → interpolator → update_output_cmd_() → send_recv_()
  │  后台线程 500Hz 循环执行
  │
  ▼
ArxCan (CAN 通信层)
  │  send_DM_motor_cmd(motor_id=8, kp=5.0, kd=0.2, pos, vel, tor)
  │
  ▼
DM_J4310 夹爪电机 (硬件)
  │  电机位置范围: 0 ~ gripper_open_readout (如 -3.37 rad)
  │  夹爪位置范围: 0 m (全关) ~ 0.088 m (全开)
```

### 关键参数 (X5 机型)

| 参数 | 值 | 来源 |
|------|------|------|
| gripper_motor_id | 8 | config.h |
| gripper_motor_type | DM_J4310 | config.h |
| gripper_width | 0.088 m | config.h |
| gripper_open_readout | -3.37 rad | config.h (需标定) |
| gripper_torque_max | 1.5 Nm | config.h |
| gripper_vel_max | 0.3 m/s | config.h |
| gripper_kp (默认) | 5.0 | config.h |
| gripper_kd (默认) | 0.2 | config.h |

---

## 2. AdaptiveGripperController 两种控制模式

### 2.1 直接位置模式

- **Topic**: `/<joint_name>/position_command` (std_msgs/Float64)
- **行为**: 收到消息后设置 `direct_position_mode_ = true`，目标位置 = clamp(输入值, 关节限位)
- **力反馈**: 永远不启用（无论 `use_effort_interface` 设置如何）
- **适用场景**: 遥操作、精细位置控制

### 2.2 开关控制模式

- **Topic**: `/<controller_name>/target_command` (std_msgs/Int32)
- **行为**:
  - `0` → `target_position_ = closed_position_`（关闭位置，从 URDF 限位自动计算）
  - `1` → `target_position_ = open_position_`（打开位置）
- **力反馈**: 仅当 `use_effort_interface: true` 时启用
  - 只在关闭（target=0）时生效
  - 当 effort 超过 `force_threshold` 时，按 `force_feedback_ratio` 比例调整目标位置
- **适用场景**: 简单抓放操作

### 两个 topic 的关系

两个 topic 互相覆盖，**最后收到哪个 topic 的消息就用哪个模式**。
`use_effort_interface` 只控制开关模式下的力反馈是否生效，不影响 topic 选择。

```
position_command (Float64)       target_command (Int32)
       │                                │
       ▼                                ▼
  去你指定的位置                   0 → closed_position_
  力反馈：永远关                   1 → open_position_
                                        │
                            use_effort_interface?
                            ├── false: 直接去目标，不管力
                            └── true:  关闭时如果力超阈值，
                                       按 ratio 提前停下
```

---

## 3. 控制器与 Hardware Interface 的关系

AdaptiveGripperController 与 hardware_interface 之间通过 ros2_control 的接口机制交互。
夹爪在 hardware_interface 层面就是一个普通的 prismatic joint，没有任何特殊处理。

```
AdaptiveGripperController::update()
  │
  │  position_command_interface.set_value(target_position_)
  │  // 写入 gripper_joint/position command_interface
  │
  ▼
ArxX5Hardware 共享内存
  │
  │  gripper_position_commands_[0]  ← 控制器写入的值
  │
  ▼
ArxX5Hardware::write()
  │  cmd.gripper_pos = gripper_position_commands_[0];
  │  controllers_[arm_idx]->set_joint_cmd(cmd);
  │
  ▼
ARX SDK → CAN → DM_J4310 电机
```

---

## 4. ARX SDK 夹爪控制实现

### 4.1 位置换算

**读取状态** (controller_base.cpp:399-407):
```cpp
// 电机角度(rad) → 夹爪位置(m)
gripper_pos = motor_angle / gripper_open_readout * gripper_width
// X5: motor_angle / (-3.37) * 0.088
```

**发送指令** (controller_base.cpp:600-613):
```cpp
// 夹爪位置(m) → 电机角度(rad)
gripper_motor_pos = gripper_pos / gripper_width * gripper_open_readout
send_DM_motor_cmd(8, gripper_kp, gripper_kd, gripper_motor_pos, vel, tor)
```

### 4.2 SDK 内部保护机制

SDK 内部有三重保护，独立于 ROS2 控制器层，无法从外部绕过：

#### 保护 1: 力矩锁定 (controller_base.cpp:505-519)

```cpp
if (std::abs(gripper_torque) > gripper_torque_max / 2)  // > 0.75 Nm
{
    // 如果运动方向与堵转力矩方向一致，冻结位置指令
    output_joint_cmd_.gripper_pos = prev_output_cmd.gripper_pos;
}
```

- **触发条件**: 夹爪力矩 > 0.75 Nm
- **后果**: 位置指令被冻结，不再更新
- **可恢复**: 反向运动（打开）理论上可以通过

#### 保护 2: 过流保护 → 紧急状态 (controller_base.cpp:311-342)

```cpp
if (std::abs(gripper_torque) > gripper_torque_max)  // > 1.5 Nm
{
    over_current_cnt_++;
    if (over_current_cnt_ > 20)  // 连续 20 次 ≈ 40ms @ 500Hz
    {
        enter_emergency_state_();  // 进入死循环！
    }
}
```

- **触发条件**: 夹爪力矩连续超过 1.5 Nm 达 ~40ms
- **后果**: `enter_emergency_state_()` 是 `while(true)` 死循环，只发阻尼指令，永远不退出
- **不可恢复**: SDK 线程永久阻塞，必须杀掉进程

#### 保护 3: 位置合规性检查 (controller_base.cpp:299-308)

```cpp
if (gripper_pos < -0.005 || gripper_pos > gripper_width + 0.005)  // 超出 [-0.005, 0.093]
{
    enter_emergency_state_();  // 同样进入死循环！
}
```

- **触发条件**: 夹爪位置超出有效范围（容差仅 5mm）
- **后果**: 同样进入紧急状态死循环
- **影响初始化**: `init_robot_()` 阶段也会调用此检查

---

## 5. 故障根因分析

### 运行时夹爪锁死

1. 夹爪夹到物体 → 电机堵转 → 力矩持续升高
2. 力矩超过 1.5 Nm 持续 40ms → 触发 `over_current_protection_()`
3. SDK 进入 `enter_emergency_state_()` — **while(true) 死循环**
4. 所有控制指令被阻塞，夹爪不响应任何命令

### 重启后依然无法控制

1. 杀掉进程，重新启动
2. SDK `init_robot_()` 初始化时调用 `check_joint_state_sanity_()`
3. 如果夹爪在堵转时电机编码器读数发生漂移，计算出的 `gripper_pos` 超出 `[-0.005, 0.093]` 范围
4. **初始化阶段就直接进入紧急状态死循环**，连初始化都无法完成

也可能是 DM_J4310 电机自身进入硬件级过流/过温保护状态，需要断电重置。

---

## 6. 诊断方法

### 6.1 查看日志关键字

复现问题时观察终端输出，寻找以下日志：

**过流保护触发**:
```
[ERROR] Over current detected once on gripper, current: x.xxx
[ERROR] Over current detected, robot is set to damping. Please restart the program.
[ERROR] Emergency state entered. Please restart the program.
```

**位置合规性检查失败**:
```
[ERROR] Gripper position error: got x.xxx but should be in 0~0.088 (m).
         Please close the gripper before turning the arm on or recalibrate gripper home and width.
[ERROR] Emergency state entered. Please restart the program.
```

**力矩锁定（警告级）**:
```
[WARN] Gripper torque is too large, gripper pos cmd is not updated
```

### 6.2 运行时监控

```bash
# 监控夹爪状态
ros2 topic echo /joint_states  # 查看 gripper_joint 的 position, velocity, effort

# 监控 SDK 日志输出（通常在启动终端）
# 关注 spdlog 的 [ERROR] 和 [WARN] 级别输出
```

---

## 7. 解决方案

### 7.1 临时恢复操作

1. 关闭程序 (`Ctrl+C` 或 `kill`)
2. **关闭机械臂电源**
3. 手动将夹爪拨到**半开位置**（确保 gripper_pos 在有效范围内）
4. 重新上电
5. 启动程序

### 7.2 代码层面修复建议

| 问题 | 位置 | 建议修复 |
|------|------|----------|
| `enter_emergency_state_()` 是 while(true) 死循环 | controller_base.cpp:344-363 | 改为设置标志位 + 超时恢复机制，而非死循环 |
| 过流保护阈值太低 | config.h: gripper_torque_max=1.5 | 对夹爪场景考虑提高阈值或区分夹爪/关节的过流策略 |
| 位置合规性容差太小 (5mm) | controller_base.cpp:300 | 增大 gripper_width_tolerance，如 0.01~0.02m |
| 过流计数器阈值太低 (20次=40ms) | config.h: over_current_cnt_max=20 | 考虑增大或为夹爪单独设置更宽松的阈值 |

### 7.3 夹爪重新标定

如果怀疑零点漂移，使用 SDK 的标定功能：

```python
# python/examples/calibrate.py
import arx5_interface as arx5
controller = arx5.Arx5JointController("X5", "can0")
controller.calibrate_gripper()
# 按提示：1. 完全合上夹爪 → Enter  2. 完全张开夹爪 → Enter
# 记录 gripper_open_readout 值并更新 config.h
```

---

## 8. 相关文件索引

| 组件 | 文件路径 |
|------|----------|
| 自适应夹爪控制器 | `controller/adaptive_gripper_controller/src/adaptive_gripper_controller.cpp` |
| 控制器头文件 | `controller/adaptive_gripper_controller/include/adaptive_gripper_controller/adaptive_gripper_controller.h` |
| Hardware Interface 实现 | `hardwares/arx_x5_ros2_control/src/arx_x5_hardware.cpp` |
| Hardware Interface 头文件 | `hardwares/arx_x5_ros2_control/include/arx_x5_ros2_control/arx_x5_hardware.h` |
| SDK 控制器基类 (保护逻辑) | `hardwares/arx_x5_ros2_control/external/arx5-sdk/src/app/controller_base.cpp` |
| SDK 关节控制器 | `hardwares/arx_x5_ros2_control/external/arx5-sdk/src/app/joint_controller.cpp` |
| SDK 数据结构 | `hardwares/arx_x5_ros2_control/external/arx5-sdk/include/app/common.h` |
| SDK 机器人配置 | `hardwares/arx_x5_ros2_control/external/arx5-sdk/include/app/config.h` |
| SDK CAN 通信 | `hardwares/arx_x5_ros2_control/external/arx5-sdk/include/hardware/arx_can.h` |
| 控制器配置 (yaml) | `robot_descriptions/.../config/ros2_control/ros2_controllers.yaml` |

> 路径相对于 `/home/z/ros2_ws/src/arms_ros2_control/`
