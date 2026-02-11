# 动态参数调整使用指南

本指南说明如何在运行时通过终端动态调整 ARX X5 机械臂的增益参数（kp/kd）。

## 前提条件

1. 已编译并安装 `arx_x5_ros2_control` 包
2. 机械臂硬件已连接并启动
3. ROS2 节点正在运行


## 步骤 2：查找节点名称

硬件接口参数通常位于 `controller_manager` 节点下。首先查找节点名称：

```bash
# 查看所有节点
ros2 node list

# 查找包含 controller_manager 的节点
ros2 node list | grep controller

# 通常节点名称类似：
# /controller_manager
```

## 步骤 3：查看当前参数

在调整参数前，先查看当前的参数值：

```bash
# 查看所有参数
ros2 param list /controller_manager

# 查看关节位置增益（kp）
ros2 param get /controller_manager joint_k_gains

# 查看关节阻尼增益（kd）
ros2 param get /controller_manager joint_d_gains

# 查看夹爪位置增益
ros2 param get /controller_manager gripper_kp

# 查看夹爪阻尼增益
ros2 param get /controller_manager gripper_kd
```

**示例输出：**
```
String value is: [80.0, 70.0, 70.0, 30.0, 30.0, 20.0]
```

## 步骤 4：动态调整参数

### 4.1 调整关节位置增益（joint_k_gains）

```bash
# 设置6个关节的位置增益
# 格式：[joint1, joint2, joint3, joint4, joint5, joint6]
ros2 param set /controller_manager joint_k_gains "[100.0, 90.0, 80.0, 40.0, 30.0, 25.0]"
```

**参数说明：**
- 必须是6个值的数组（对应6个关节）
- 值越大，关节响应越快、越刚硬
- 典型范围：20.0 ~ 200.0

**示例场景：**
```bash
# 场景1：提高前3个关节的刚度（用于快速运动）
ros2 param set /controller_manager joint_k_gains "[120.0, 100.0, 90.0, 30.0, 30.0, 20.0]"

# 场景2：降低所有关节刚度（用于柔顺控制）
ros2 param set /controller_manager joint_k_gains "[40.0, 35.0, 35.0, 15.0, 15.0, 10.0]"

# 场景3：恢复默认值
ros2 param set /controller_manager joint_k_gains "[80.0, 70.0, 70.0, 30.0, 30.0, 20.0]"
```

### 4.2 调整关节阻尼增益（joint_d_gains）

```bash
# 设置6个关节的阻尼增益
ros2 param set /controller_manager joint_d_gains "[3.0, 2.5, 2.5, 1.5, 1.0, 0.8]"
```

**参数说明：**
- 必须是6个值的数组
- 值越大，阻尼越大，振动越小
- 典型范围：0.5 ~ 5.0

**示例场景：**
```bash
# 场景1：增加阻尼（减少振动）
ros2 param set /controller_manager joint_d_gains "[4.0, 3.0, 3.0, 2.0, 1.5, 1.0]"

# 场景2：降低阻尼（更灵敏）
ros2 param set /controller_manager joint_d_gains "[1.0, 1.0, 1.0, 0.5, 0.5, 0.4]"

# 场景3：恢复默认值
ros2 param set /controller_manager joint_d_gains "[2.0, 2.0, 2.0, 1.0, 1.0, 0.7]"
```

### 4.3 调整夹爪位置增益（gripper_kp）

```bash
# 设置夹爪位置增益
ros2 param set /controller_manager gripper_kp 10.0
```

**参数说明：**
- 单个浮点数值
- 值越大，夹爪响应越快
- 典型范围：2.0 ~ 20.0

**示例场景：**
```bash
# 场景1：提高夹爪响应速度
ros2 param set /controller_manager gripper_kp 10.0

# 场景2：降低夹爪响应速度（更柔顺）
ros2 param set /controller_manager gripper_kp 3.0

# 场景3：恢复默认值
ros2 param set /controller_manager gripper_kp 5.0
```

### 4.4 调整夹爪阻尼增益（gripper_kd）

```bash
# 设置夹爪阻尼增益
ros2 param set /controller_manager gripper_kd 0.5
```

**参数说明：**
- 单个浮点数值
- 值越大，夹爪阻尼越大
- 典型范围：0.1 ~ 2.0

**示例场景：**
```bash
# 场景1：增加夹爪阻尼
ros2 param set /controller_manager gripper_kd 1.0

# 场景2：降低夹爪阻尼
ros2 param set /controller_manager gripper_kd 0.2

# 场景3：恢复默认值
ros2 param set /controller_manager gripper_kd 0.2
```

## 步骤 5：验证参数是否生效

### 5.1 查看日志输出

参数设置成功后，节点会输出日志信息：

```
[INFO] [arx_x5_hardware]: joint_k_gains updated via parameter
[INFO] [arx_x5_hardware]: Left arm gains applied: kp=[100.0, 90.0, ...], kd=[2.0, 2.0, ...], gripper_kp=5.00, gripper_kd=0.20
```

### 5.2 再次查看参数值

```bash
# 确认参数已更新
ros2 param get /controller_manager joint_k_gains
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
ros2 param get /controller_manager joint_k_gains

# 2. 提高前3个关节的刚度和阻尼（用于快速运动）
ros2 param set /controller_manager joint_k_gains "[120.0, 100.0, 90.0, 30.0, 30.0, 20.0]"
ros2 param set /controller_manager joint_d_gains "[3.0, 2.5, 2.5, 1.0, 1.0, 0.7]"

# 3. 验证
ros2 param get /controller_manager joint_k_gains
```

### 示例 2：柔顺控制模式

```bash
# 1. 降低所有关节的刚度（用于柔顺控制或接触任务）
ros2 param set /controller_manager joint_k_gains "[40.0, 35.0, 35.0, 15.0, 15.0, 10.0]"
ros2 param set /controller_manager joint_d_gains "[1.5, 1.5, 1.5, 0.8, 0.8, 0.5]"

# 2. 验证
ros2 param get /controller_manager joint_k_gains
```

### 示例 3：精细操作模式

```bash
# 1. 调整末端关节（关节5和6）的增益（用于精细操作）
ros2 param set /controller_manager joint_k_gains "[80.0, 70.0, 70.0, 30.0, 15.0, 10.0]"
ros2 param set /controller_manager joint_d_gains "[2.0, 2.0, 2.0, 1.0, 0.5, 0.4]"

# 2. 验证
ros2 param get /controller_manager joint_k_gains
```

### 示例 4：夹爪精细控制

```bash
# 1. 提高夹爪增益（用于快速抓取）
ros2 param set /controller_manager gripper_kp 10.0
ros2 param set /controller_manager gripper_kd 0.5

# 2. 或者降低夹爪增益（用于柔顺抓取）
ros2 param set /controller_manager gripper_kp 3.0
ros2 param set /controller_manager gripper_kd 0.1

# 3. 验证
ros2 param get /controller_manager gripper_kp
```

## 错误处理

### 错误 1：参数长度不匹配

```bash
# 错误示例：只提供了5个值，需要6个
ros2 param set /controller_manager joint_k_gains "[100.0, 90.0, 80.0, 40.0, 30.0]"

# 错误信息：
# Set parameter failed: joint_k_gains must have exactly 6 values (for 6-joint robot)
```

**解决方法：** 确保提供6个值

### 错误 2：硬件未连接

```bash
# 如果硬件未激活，会收到错误：
# Set parameter failed: Hardware not connected. Cannot set gains.
```

**解决方法：** 确保硬件已连接并激活

### 错误 3：节点名称错误

```bash
# 如果节点名称错误，会收到：
# Node not found: /wrong_node_name
```

**解决方法：** 使用 `ros2 node list` 查找正确的节点名称

## 注意事项

1. **参数立即生效**：参数修改后立即应用到硬件，无需重启节点
2. **参数验证**：系统会自动验证参数类型和长度
3. **硬件连接**：增益参数只能在硬件连接后设置
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
# 查看所有参数
ros2 param list /controller_manager

# 查看单个参数
ros2 param get /controller_manager <param_name>

# 设置参数
ros2 param set /controller_manager <param_name> <value>

# 查看节点信息
ros2 node info /controller_manager

# 查看日志
ros2 topic echo /rosout | grep arx_x5_hardware
```
