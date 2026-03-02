# 动态参数调整使用指南

本指南说明如何在运行时通过终端动态调整 ARX X5 机械臂的增益参数（kp/kd）。

## 前提条件

1. 已编译并安装 `arx_ros2_control` 包
2. 机械臂硬件已连接并启动
3. ROS2 节点正在运行

## 双臂模式：启动时为左右臂设置不同 kp/kd

在真实双臂模式下，会同时加载 `arx_lift2s_left_system` 和 `arx_lift2s_right_system` 两个 ros2_control system，每个 system 内部都是一个独立的 `ArxX5Hardware` 节点，各自拥有一套 `joint_k_gains` / `joint_d_gains` / `gripper_kp` / `gripper_kd` 参数。

若希望**一启动**就为左臂和右臂使用不同的 kp/kd，只需要在机器人描述的 ros2_control 中，分别在左右两个 `<ros2_control>` 的 `<hardware>` 里配置不同的增益：

**示例（xacro，节选）：**

```xml
<ros2_control name="arx_lift2s_left_system" type="system">
  <hardware>
    <plugin>arx_ros2_control/ArxX5Hardware</plugin>
    <param name="robot_model">X5</param>
    <param name="can_interface">can1</param>
    <param name="joint_k_gains">[80.0, 70.0, 65.0, 22.0, 22.0, 14.0]</param>
    <param name="joint_d_gains">[2.6, 2.6, 2.3, 0.85, 0.85, 0.6]</param>
    <!-- 可选：夹爪增益 -->
    <!-- <param name="gripper_kp">5.0</param> -->
    <!-- <param name="gripper_kd">0.2</param> -->
  </hardware>
</ros2_control>

<ros2_control name="arx_lift2s_right_system" type="system">
  <hardware>
    <plugin>arx_ros2_control/ArxX5Hardware</plugin>
    <param name="robot_model">X5</param>
    <param name="can_interface">can3</param>
    <param name="joint_k_gains">[70.0, 60.0, 60.0, 20.0, 20.0, 12.0]</param>
    <param name="joint_d_gains">[2.2, 2.2, 2.0, 0.75, 0.75, 0.5]</param>
    <!-- 可选：夹爪增益 -->
    <!-- <param name="gripper_kp">5.0</param> -->
    <!-- <param name="gripper_kd">0.2</param> -->
  </hardware>
</ros2_control>
```

不配置时，左右臂会使用同一套默认增益；配置后，启动时两臂即按上述参数使用不同 kp/kd。

## 步骤 2：查找节点名称

硬件接口参数位于硬件接口组件所在的节点下。首先查找节点名称：

```bash
# 查看所有节点
ros2 node list

# 查找硬件接口节点（通常包含硬件名称，如 arx5_system）
ros2 node list | grep arx

# 或者查看所有节点的参数，找到包含 joint_k_gains 的节点
for node in $(ros2 node list); do
    if ros2 param list $node 2>/dev/null | grep -q joint_k_gains; then
        echo "Found hardware interface node: $node"
    fi
done
```

**常见的节点名称：**
- `/arx5_system` - 单臂模式下的 ARX X5 硬件接口节点
- `/arx_lift2s_left_system`、`/arx_lift2s_right_system` - 真实双臂模式下左右臂的硬件接口节点（两个节点）
- `/arx_lift2s_system` - 双臂仿真/非 real 模式下的统一硬件接口节点
- `/controller_manager` - 控制器管理器节点（不包含硬件参数）

## 步骤 3：查看当前参数

在调整参数前，先查看当前的参数值。

### 单臂模式（LEFT / RIGHT）

```bash
# 查看所有参数
ros2 param list /arx5_system

# 查看关节位置增益（kp）
ros2 param get /arx5_system joint_k_gains

# 查看关节阻尼增益（kd）
ros2 param get /arx5_system joint_d_gains

# 查看夹爪位置/阻尼增益
ros2 param get /arx5_system gripper_kp
ros2 param get /arx5_system gripper_kd
```

**示例输出（一行）：**
```
String value is: [80.0, 70.0, 70.0, 30.0, 30.0, 20.0]
```

### 双臂模式（DUAL，两个硬件节点）

真实双臂模式下，会同时出现 `/arx_lift2s_left_system` 和 `/arx_lift2s_right_system` 两个硬件接口节点，左右臂各有独立增益，需要分别在两个节点上查看：

```bash
# 左臂增益
ros2 param get /arx_lift2s_left_system joint_k_gains
ros2 param get /arx_lift2s_left_system joint_d_gains
ros2 param get /arx_lift2s_left_system gripper_kp
ros2 param get /arx_lift2s_left_system gripper_kd

# 右臂增益
ros2 param get /arx_lift2s_right_system joint_k_gains
ros2 param get /arx_lift2s_right_system joint_d_gains
ros2 param get /arx_lift2s_right_system gripper_kp
ros2 param get /arx_lift2s_right_system gripper_kd
```

如果使用仿真（`ros2_control_hardware_type` 不是 `real`），则通常只会有一个统一节点 `/arx_lift2s_system`，其参数用法与单臂类似。

## 步骤 4：动态调整参数

### 4.1 调整关节位置增益（joint_k_gains）

```bash
# 设置6个关节的位置增益
# 格式：[joint1, joint2, joint3, joint4, joint5, joint6]
ros2 param set /arx5_system joint_k_gains "[100.0, 90.0, 80.0, 40.0, 30.0, 25.0]"
```

**参数说明：**
- 必须是6个值的数组（对应6个关节）
- 值越大，关节响应越快、越刚硬
- 典型范围：20.0 ~ 200.0

**示例场景：**
```bash
# 场景1：提高前3个关节的刚度（用于快速运动）
ros2 param set /arx5_system joint_k_gains "[120.0, 100.0, 90.0, 30.0, 30.0, 20.0]"

# 场景2：降低所有关节刚度（用于柔顺控制）
ros2 param set /arx5_system joint_k_gains "[40.0, 35.0, 35.0, 15.0, 15.0, 10.0]"

# 场景3：恢复默认值
ros2 param set /arx5_system joint_k_gains "[80.0, 70.0, 70.0, 30.0, 30.0, 20.0]"
```

### 4.2 调整关节阻尼增益（joint_d_gains）

```bash
# 设置6个关节的阻尼增益
ros2 param set /arx5_system joint_d_gains "[3.0, 2.5, 2.5, 1.5, 1.0, 0.8]"
```

**参数说明：**
- 必须是6个值的数组
- 值越大，阻尼越大，振动越小
- 典型范围：0.5 ~ 5.0

**示例场景：**
```bash
# 场景1：增加阻尼（减少振动）
ros2 param set /arx5_system joint_d_gains "[4.0, 3.0, 3.0, 2.0, 1.5, 1.0]"

# 场景2：降低阻尼（更灵敏）
ros2 param set /arx5_system joint_d_gains "[1.0, 1.0, 1.0, 0.5, 0.5, 0.4]"

# 场景3：恢复默认值
ros2 param set /arx5_system joint_d_gains "[2.0, 2.0, 2.0, 1.0, 1.0, 0.7]"
```

### 4.3 调整夹爪位置增益（gripper_kp）

```bash
# 设置夹爪位置增益
ros2 param set /arx5_system gripper_kp 10.0
```

**参数说明：**
- 单个浮点数值
- 值越大，夹爪响应越快
- 典型范围：2.0 ~ 20.0

**示例场景：**
```bash
# 场景1：提高夹爪响应速度
ros2 param set /arx5_system gripper_kp 10.0

# 场景2：降低夹爪响应速度（更柔顺）
ros2 param set /arx5_system gripper_kp 3.0

# 场景3：恢复默认值
ros2 param set /arx5_system gripper_kp 5.0
```

### 4.4 调整夹爪阻尼增益（gripper_kd）

```bash
# 设置夹爪阻尼增益
ros2 param set /arx5_system gripper_kd 0.5
```

### 4.5 双臂模式：分别调整左/右臂增益

真实双臂模式下，左右臂是两个独立的硬件节点，需要分别对两个节点设置同名参数：

```bash
# 仅设置左臂关节/夹爪增益
ros2 param set /arx_lift2s_left_system joint_k_gains "[100.0, 90.0, 80.0, 40.0, 30.0, 25.0]"
ros2 param set /arx_lift2s_left_system joint_d_gains "[3.0, 2.5, 2.5, 1.5, 1.0, 0.8]"
ros2 param set /arx_lift2s_left_system gripper_kp 10.0
ros2 param set /arx_lift2s_left_system gripper_kd 0.5

# 仅设置右臂关节/夹爪增益
ros2 param set /arx_lift2s_right_system joint_k_gains "[90.0, 80.0, 70.0, 35.0, 25.0, 20.0]"
ros2 param set /arx_lift2s_right_system joint_d_gains "[2.5, 2.0, 2.0, 1.2, 0.8, 0.6]"
ros2 param set /arx_lift2s_right_system gripper_kp 8.0
ros2 param set /arx_lift2s_right_system gripper_kd 0.4
```

若希望左右臂使用相同的增益，只需在 `/arx_lift2s_left_system` 和 `/arx_lift2s_right_system` 上设置同一套参数值即可。

**参数说明：**
- 单个浮点数值
- 值越大，夹爪阻尼越大
- 典型范围：0.1 ~ 2.0

**示例场景：**
```bash
# 场景1：增加夹爪阻尼
ros2 param set /arx5_system gripper_kd 1.0

# 场景2：降低夹爪阻尼
ros2 param set /arx5_system gripper_kd 0.2

# 场景3：恢复默认值
ros2 param set /arx5_system gripper_kd 0.2
```

## 步骤 5：验证参数是否生效

### 5.1 查看日志输出（可选）

参数设置失败或发生异常时，节点会在日志中输出 WARN/ERROR 级别信息，例如参数长度不匹配、数值非法、读写硬件失败等。正常情况下可以主要通过后面的参数查询和机械臂行为来验证是否生效，只有在出现问题时再结合日志排查。

### 5.2 再次查看参数值

```bash
# 确认参数已更新
ros2 param get /arx5_system joint_k_gains
# 应该显示新设置的值
```

### 5.3 观察机械臂行为

参数修改后，机械臂的控制特性会立即改变：
- **增加 kp**：关节响应更快，位置跟踪更准确
- **增加 kd**：振动减少，运动更平滑
- **降低 kp**：关节更柔顺，适合接触任务
- **降低 kd**：响应更灵敏，但可能产生振动

## 完整使用示例

### 示例 1：快速运动模式

```bash
# 1. 查看当前参数
ros2 param get /arx5_system joint_k_gains

# 2. 提高前3个关节的刚度和阻尼（用于快速运动）
ros2 param set /arx5_system joint_k_gains "[120.0, 100.0, 90.0, 30.0, 30.0, 20.0]"
ros2 param set /arx5_system joint_d_gains "[3.0, 2.5, 2.5, 1.0, 1.0, 0.7]"

# 3. 验证
ros2 param get /arx5_system joint_k_gains
```

### 示例 2：柔顺控制模式

```bash
# 1. 降低所有关节的刚度（用于柔顺控制或接触任务）
ros2 param set /arx5_system joint_k_gains "[40.0, 35.0, 35.0, 15.0, 15.0, 10.0]"
ros2 param set /arx5_system joint_d_gains "[1.5, 1.5, 1.5, 0.8, 0.8, 0.5]"

# 2. 验证
ros2 param get /arx5_system joint_k_gains
```

### 示例 3：精细操作模式

```bash
# 1. 调整末端关节（关节5和6）的增益（用于精细操作）
ros2 param set /arx5_system joint_k_gains "[80.0, 70.0, 70.0, 30.0, 15.0, 10.0]"
ros2 param set /arx5_system joint_d_gains "[2.0, 2.0, 2.0, 1.0, 0.5, 0.4]"

# 2. 验证
ros2 param get /arx5_system joint_k_gains
```

### 示例 4：夹爪精细控制

```bash
# 1. 提高夹爪增益（用于快速抓取）
ros2 param set /arx5_system gripper_kp 10.0
ros2 param set /arx5_system gripper_kd 0.5

# 2. 或者降低夹爪增益（用于柔顺抓取）
ros2 param set /arx5_system gripper_kp 3.0
ros2 param set /arx5_system gripper_kd 0.1

# 3. 验证
ros2 param get /arx5_system gripper_kp
```

## 错误处理

### 错误 1：参数长度不匹配

```bash
# 错误示例：只提供了5个值，需要6个
ros2 param set /arx5_system joint_k_gains "[100.0, 90.0, 80.0, 40.0, 30.0]"

# 错误信息：
# Set parameter failed: joint_k_gains must have exactly 6 values (for 6-joint robot)
```

**解决方法：** 确保提供6个值

### 注意 2：硬件未连接时的行为

当硬件尚未连接或未激活时，仍然可以通过 `ros2 param set` 成功设置增益参数，但此时增益不会立即下发到控制器，只有在硬件 configure + activate 之后才真正作用到电机。调整参数前，建议先完成硬件连接与激活。

### 错误 3：节点名称错误

```bash
# 如果节点名称错误，会收到：
# Node not found: /wrong_node_name
```

**解决方法：** 使用 `ros2 node list` 查找正确的节点名称（单臂多为 `/arx5_system`，双臂实机多为 `/arx_lift2s_left_system`、`/arx_lift2s_right_system`，双臂仿真多为 `/arx_lift2s_system`）

## 注意事项

1. **参数立即生效**：参数修改后立即应用到硬件，无需重启节点
2. **参数验证**：系统会自动验证参数类型和长度
3. **硬件连接**：建议在硬件连接并激活后调整增益；在未连接时参数仍可设置，但不会立即下发到硬件
4. **参数持久化**：参数修改不会保存到配置文件，重启节点后会恢复默认值
5. **安全建议**：调整参数时建议从小幅度开始，观察机械臂行为

## 参数调优建议

### 初始值
- `joint_k_gains`: `[80.0, 70.0, 70.0, 30.0, 30.0, 20.0]`
- `joint_d_gains`: `[2.0, 2.0, 2.0, 1.0, 1.0, 0.7]`
- `gripper_kp`: `5.0`
- `gripper_kd`: `0.2`

### 调优步骤
1. 从默认值开始
2. 逐步调整（每次改变10-20%）
3. 观察机械臂行为
4. 记录最佳参数值
5. 如需持久化，更新配置文件

## 相关命令速查

```bash
# 单臂：节点多为 /arx5_system
# 双臂实机：节点为 /arx_lift2s_left_system 和 /arx_lift2s_right_system
# 双臂仿真：节点为 /arx_lift2s_system
NODE=/arx5_system                       # 单臂示例
# NODE=/arx_lift2s_left_system         # 左臂示例
# NODE=/arx_lift2s_right_system        # 右臂示例
# NODE=/arx_lift2s_system              # 双臂仿真示例

# 查看所有参数
ros2 param list $NODE

# 查看单个参数
ros2 param get $NODE <param_name>

# 设置参数
ros2 param set $NODE <param_name> <value>

# 查看节点信息
ros2 node info $NODE

# 查看日志
ros2 topic echo /rosout | grep arx_x5_hardware
```

## 快速参考

**节点名称：**
- 单臂：`/arx5_system`
- 双臂实机：`/arx_lift2s_left_system`、`/arx_lift2s_right_system`
- 双臂仿真：`/arx_lift2s_system`

**单臂 / 统一参数（单臂必用，双臂仿真可用来同时控制左右臂）：**
- `joint_k_gains` - 关节位置增益（6个值的数组）
- `joint_d_gains` - 关节阻尼增益（6个值的数组）
- `gripper_kp` - 夹爪位置增益（单个值）
- `gripper_kd` - 夹爪阻尼增益（单个值）

**双臂实机：左右臂节点与参数：**
- `/arx_lift2s_left_system`  - 左臂：`joint_k_gains`、`joint_d_gains`、`gripper_kp`、`gripper_kd`
- `/arx_lift2s_right_system` - 右臂：`joint_k_gains`、`joint_d_gains`、`gripper_kp`、`gripper_kd`

**快速设置示例：**
```bash
# 单臂（节点 /arx5_system）
ros2 param get /arx5_system joint_k_gains
ros2 param set /arx5_system joint_k_gains "[100.0, 90.0, 80.0, 40.0, 30.0, 25.0]"
ros2 param set /arx5_system joint_d_gains "[3.0, 2.5, 2.5, 1.5, 1.0, 0.8]"
ros2 param set /arx5_system gripper_kp 10.0
ros2 param set /arx5_system gripper_kd 0.5

# 双臂（左右各一个节点）
ros2 param get /arx_lift2s_left_system joint_k_gains
ros2 param get /arx_lift2s_right_system joint_k_gains
ros2 param set /arx_lift2s_left_system joint_k_gains "[100.0, 90.0, 80.0, 40.0, 30.0, 25.0]"
ros2 param set /arx_lift2s_right_system joint_k_gains "[90.0, 80.0, 70.0, 35.0, 25.0, 20.0]"
```
