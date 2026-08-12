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

## 臂控制（仅 `full_control` / MIT MIX）

URDF 声明 `position/velocity/effort`；`write()` **始终**下发 `pos + vel + effort`，MIT `kp/kd` **仅**来自 HI `joint_k_gains` / `joint_d_gains`（无 kp/kd command IF）。

仅支持 `full_control`。若 URDF 写了其它 `control_mode`，HI 会告警并忽略，按 full_control 运行。

夹爪保持 **position-only**（`gripper_kp` / `gripper_kd`）。

```xml
<param name="control_mode">full_control</param>
<!-- robot_model 由 HI 写死为 X5，不导出 ROS 参数（rqt 不可见）；URDF 里可省略 -->
<param name="can_interface">can1</param>  <!-- 单臂右臂用 can3 -->
<!-- 单臂 / ACone 默认；Lift2S 现场默认为 [20,20,20,20,10,10] / [0.8,0.8,0.8,0.8,0.5,0.5] -->
<param name="joint_k_gains">[80.0, 70.0, 70.0, 30.0, 30.0, 20.0]</param>
<param name="joint_d_gains">[2.0, 2.0, 2.0, 1.0, 1.0, 0.7]</param>
<!-- 可选：Ctrl+C / deactivate 时先插值到 shutdown_home 再阻尼（默认 false） -->
<param name="shutdown_return_home">true</param>
<param name="shutdown_home">[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]</param>
<param name="shutdown_home_velocity">0.3</param>
<param name="shutdown_home_timeout">2.0</param>
```

Lift2S 真机 xacro 已打开上述关机回零；单臂 / ACone 默认仅 `set_to_damping`。故障路径（`on_error`）一律只阻尼、不插值。

## `full_control` 下发映射（OCS2 MIX）

| 量 | 来源 | SDK |
|----|------|-----|
| position | OCS2 轨迹 | `JointState.pos` |
| velocity | OCS2 `future_input` | `JointState.vel` |
| effort（重力/静力学前馈） | OCS2 | `JointState.torque` |
| kp / kd | HI `joint_k_gains` / `joint_d_gains` | `set_gain` |

```bash
./quick_start.sh   # Build → 真机包；Launch → 自动预启 Zenoh；单臂可选左/右

# 手动 launch（RMW=zenoh 时先另开终端: ros2 run rmw_zenoh_cpp rmw_zenohd）
source ~/lift2s-ws/install/setup.bash
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

### 底盘 `cmd_vel`（可选）

| hardware 参数 | 默认 | 说明 |
|---------------|------|------|
| `enable_chassis_cmd_vel` | `true`（xacro 默认） | 订阅 Twist → `setChassisCmd` |
| `chassis_cmd_vel_topic` | `/cmd_vel` | 话题名 |
| `chassis_cmd_timeout` | `0.3` | 超时停车（s）；mode=2 |
| `chassis_max_vel_{x,y,z}` | `2/2/4` | `setChassisCmd` 量化上限；**LIFTS 的 .so 未写这些，必须由 HI 补** |

映射：`linear.x/y` → `v_x/v_y`，`angular.z` → `w_z`（ROS Twist 原样）；mode=1 运行 / mode=2 停车。  
**hybrid（OCS2 推荐）**：升降每拍 `sendLiftHybrid`；底盘 `vx/vy/wz` 每拍 `sendChassisOnly`（仅 `0x701/0x703`，**不绑 Soft-P**）。  
soft_p：`loop()` 同时带升降+底盘；全身 OCS2 下易掉柱，仅适合 HOME/点动。  
`chassis_max_vel_{x,y,z}` 须由 HI 写入（LIFTS 的 .so 未初始化）。  
Lift2S xacro 默认开；若与 WBC 底盘规划抢指令可传 `enable_chassis_cmd_vel:=false`。

## 依赖

### ROS2
- `hardware_interface` / `pluginlib` / `rclcpp` / `rclcpp_lifecycle` / `std_msgs`

### Vendored（`external/`）

| 组件 | 路径 | 说明 |
|------|------|------|
| arx5-sdk | `external/arx5-sdk/` | 头文件；`lib/<arch>/libhardware.so`、`libsolver.so`（含 aarch64） |
| arx_lift_src | `external/arx_lift_src/` | Lift2S：`lib/<arch>/libarx_lift_src.so`（目前仅 x86_64；缺档时 CMake 跳过 `ArxLiftHardware`） |
| SOEM | `external/SOEM/lib/<arch>/libsoem.so` | x86 `libhardware.so` 运行时依赖（1.4.x）；aarch64 `libhardware` 已静态内嵌 SOEM，可不提供 |
