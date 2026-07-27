# ARX X5 ROS2 Control 硬件接口包

Stanford [arx5-sdk](https://github.com/real-stanford/arx5-sdk) 封装的 `ros2_control` SystemInterface。

用于 **单臂 `arx5` / 双臂 `arx_acone`** 真机（产品规则：Lift2S 走官方 `arxlift2s_ros2_control`，本包不负责升降）。

控制契约对齐 [panthera-ht](https://github.com/fiveages-sim/open-deploy-ws/tree/panthera-ht) /
[`ht-ros2-control`](https://github.com/fiveages-sim/ht-ros2-control)：URDF **始终**声明 MIX 接口，`control_mode` 只影响 `write()`。

## 插件

| Plugin | 说明 |
|--------|------|
| `arx_ros2_control/ArxX5Hardware` | 单臂 SystemInterface；双臂时左右各实例化一次 |

## 控制模式（`control_mode`）— 参考 panthera-ht

| ARX 模式 | 默认 | 行为（`write()`） | 对应 HT |
|----------|------|-------------------|---------|
| `full_control` | 是（推荐真机） | pos + vel + effort → `set_joint_cmd`；MIT kp/kd 用 HI `joint_k/d_gains` | `full_control`（`pos_vel_tqe_kp_kd`） |
| `position` | 否 | **保留真机位置环**：仅 position；kp/kd 用 HI `joint_k_gains` / `joint_d_gains`；vel/torque=0 | ≈ `pd_control` |
| `pd_control` | 否 | `position` 的 HT 别名（`on_init` 归一化） | `pd_control` |

Stanford SDK 无 HT `position_velocity`（`pos_vel_MAXtqe`）等价路径，故不实现第三种模式。

夹爪保持 **position-only**（`gripper_kp` / `gripper_kd`）。

硬件参数示例：

```xml
<param name="control_mode">full_control</param>
<param name="robot_model">X5</param>
<param name="can_interface">can1</param>
<!-- MIT kp/kd：full_control 与 position 均用此组；可用 rqt / ros2 param 动态改 -->
<param name="joint_k_gains">[80.0, 70.0, 70.0, 30.0, 30.0, 20.0]</param>
<param name="joint_d_gains">[2.0, 2.0, 2.0, 1.0, 1.0, 0.7]</param>
```

增益来源：

| 层级 | 参数 | 何时生效 |
|------|------|----------|
| HI 参数 | `joint_k_gains` / `joint_d_gains` | **`full_control` 与 `position` 全程**；rqt / `ros2 param` 可动态调整 |
| 控制器 | `default_gains` / `pd_gains` | **不再驱动真机 MIT 增益**（HI 忽略 kp/kd command IF） |

动态调参见 [DYNAMIC_PARAMS_USAGE.md](DYNAMIC_PARAMS_USAGE.md)。

## `full_control` 下发映射（OCS2 MIX）

| 量 | 来源 | SDK |
|----|------|-----|
| position | OCS2 轨迹 | `JointState.pos` |
| velocity | OCS2 `future_input` | `JointState.vel` |
| effort（重力/静力学前馈） | OCS2 `calculateStaticTorques()` | `JointState.torque` |
| kp / kd | HI `joint_k_gains` / `joint_d_gains` | `set_gain` |

```bash
# 编译（workspace）
./quick_start.sh   # Build → 单/双臂真机包（Stanford）

# 单臂 X5（推荐 full_control）
ros2 launch ocs2_arm_controller demo.launch.py robot:=arx5 hardware:=real

# 双臂 AC One（can1 / can3）
ros2 launch ocs2_arm_controller demo.launch.py robot:=arx_acone hardware:=real

# 保留真机位置环（经 robot_common_launch 的 xacro_ 前缀）
ros2 launch ocs2_arm_controller demo.launch.py \
  robot:=arx_acone hardware:=real xacro_control_mode:=position
```

| Launch / xacro | 说明 |
|----------------|------|
| `xacro_control_mode:=full_control` | 默认；OCS2 MIX（推荐真机） |
| `xacro_control_mode:=position` | 真机位置环；HI `joint_k/d_gains` |
| `xacro_control_mode:=pd_control` | 同上（HT 别名） |

真机节点：单臂 `/arx5_system`；双臂 `/arx_acone_left_system`、`/arx_acone_right_system`。

## 依赖项

### ROS2 依赖
- `hardware_interface`
- `pluginlib`
- `rclcpp`
- `rclcpp_lifecycle`

### 第三方依赖
- `Eigen3`
- `orocos_kdl`
- `kdl_parser`
- `spdlog`

## 编译步骤

### 1. 编译 external 文件夹下的 SDK

在编译主包之前，需要先编译 `external/arx5-sdk` 目录下的 SDK。

#### 1.1 设置 conda 环境

```bash
cd external/arx5-sdk
mamba env create -f conda_environments/py312_environment.yaml
conda activate arx-py312
```

#### 1.2 编译 SDK

```bash
conda activate arx-py312
cd external/arx5-sdk
mkdir -p build && cd build
cmake ..
make -j$(nproc)
```

### 2. 编译 ROS2 包

```bash
cd ~/arx_lift2s_ws   # 或你的 workspace
colcon build --packages-select arx_ros2_control --symlink-install
```
