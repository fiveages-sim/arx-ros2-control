# ARX ROS2 Control 硬件接口包

Stanford [arx5-sdk](https://github.com/real-stanford/arx5-sdk) + Lift 库封装的 `ros2_control` SystemInterface。

本包作为 **Lift2S 部署工作空间** 的真机 HI 子模块（`src/arx-ros2-control`），同时覆盖：

- 单臂 `arx5` / 双臂 `arx_acone`（`ArxX5Hardware`）
- Lift2S 升降柱（`ArxLiftHardware`，can5）

## 插件

| Plugin | 说明 |
|--------|------|
| `arx_ros2_control/ArxX5Hardware` | 单臂 SystemInterface；双臂时左右各实例化一次 |
| `arx_ros2_control/ArxLiftHardware` | Lift2S 升降（默认 hybrid） |
| `arxlift2s_ros2_control/*` | 旧插件名别名（兼容） |

## 臂控制（仅 `full_control` / MIT MIX）

URDF 声明 MIX 接口；`write()` **始终**下发 `pos + vel + effort`，MIT `kp/kd` 来自 HI `joint_k_gains` / `joint_d_gains`。

**已移除**臂 `position` / `pd_control` 路径。若 URDF 仍写其它 `control_mode`，HI 会告警并忽略，按 full_control 运行。

夹爪保持 **position-only**（`gripper_kp` / `gripper_kd`）。

```xml
<param name="control_mode">full_control</param>
<param name="robot_model">X5</param>
<param name="can_interface">can1</param>  <!-- 单臂右臂用 can3 -->
<param name="joint_k_gains">[80.0, 70.0, 70.0, 30.0, 30.0, 20.0]</param>
<param name="joint_d_gains">[2.0, 2.0, 2.0, 1.0, 1.0, 0.7]</param>
```

## `full_control` 下发映射（OCS2 MIX）

| 量 | 来源 | SDK |
|----|------|-----|
| position | OCS2 轨迹 | `JointState.pos` |
| velocity | OCS2 `future_input` | `JointState.vel` |
| effort（重力/静力学前馈） | OCS2 | `JointState.torque` |
| kp / kd | HI `joint_k_gains` / `joint_d_gains` | `set_gain` |

```bash
./quick_start.sh   # Build → 真机包；Launch → 单臂可选左/右

ros2 launch ocs2_arm_controller demo.launch.py robot:=arx5 hardware:=real xacro_can_interface:=can1
ros2 launch ocs2_arm_controller demo.launch.py robot:=arx5 hardware:=real xacro_can_interface:=can3
ros2 launch ocs2_arm_controller demo.launch.py robot:=arx_acone hardware:=real
ros2 launch ocs2_arm_controller split_body.launch.py robot:=arx_lift2s hardware:=real
```

## 升降（`ArxLiftHardware`）

| `lift_motor_mode` | 说明 |
|-------------------|------|
| `hybrid`（默认） | `sendLiftHybrid`；跟踪 pos+vel；HI 重力/摩擦前馈 |
| `soft_p` / `position` | Soft-P `setHeight`；仅跟踪 position |

## 依赖

### ROS2
- `hardware_interface` / `pluginlib` / `rclcpp` / `rclcpp_lifecycle` / `std_msgs`

### 第三方（`external/`）
- `arx5-sdk`：头文件 + `lib/<arch>/libhardware.so`、`libsolver.so`
- `arx_lift_src`：`lib/<arch>/libarx_lift_src.so`（Lift2S）
- Eigen3 / orocos_kdl / kdl_parser / spdlog
