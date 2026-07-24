# ARX X5 ROS2 Control 硬件接口包

Stanford [arx5-sdk](https://github.com/real-stanford/arx5-sdk) 封装的 `ros2_control` SystemInterface。

用于 **单臂 `arx5` / 双臂 `arx_acone`** 真机（产品规则：Lift2S 走官方 `arxlift2s_ros2_control`，本包不负责升降）。

## 插件

| Plugin | 说明 |
|--------|------|
| `arx_ros2_control/ArxX5Hardware` | 单臂 SystemInterface；双臂时左右各实例化一次 |

## 控制模式（`control_mode`）

与 panthera-ht 相同：URDF **始终**声明 `position/velocity/effort/kp/kd`，模式只影响 `write()`。

| 模式 | 默认 | 行为 |
|------|------|------|
| `full_control` | 是 | OCS2 MIX：用控制器下发的 pos/vel/effort/kp/kd → `set_gain` + `set_joint_cmd`（等价 HT `pos_vel_tqe_kp_kd`） |
| `position` | 否 | 旧行为：只用 position；kp/kd 来自参数 `joint_k_gains` / `joint_d_gains` |

夹爪保持 **position-only**。

硬件参数示例：

```xml
<param name="control_mode">full_control</param>
<param name="robot_model">X5</param>
<param name="can_interface">can1</param>
<param name="joint_k_gains">[80.0, 70.0, 70.0, 30.0, 30.0, 20.0]</param>
<param name="joint_d_gains">[2.0, 2.0, 2.0, 1.0, 1.0, 0.7]</param>
```

`position` 模式下仍可用动态参数调增益，见 [DYNAMIC_PARAMS_USAGE.md](DYNAMIC_PARAMS_USAGE.md)。`full_control` 下关节 kp/kd 由控制器写入，参数仅作 fallback。

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
