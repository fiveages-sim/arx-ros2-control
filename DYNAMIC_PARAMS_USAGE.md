# 动态参数调整使用指南

本指南说明如何在运行时通过终端动态调整 ARX X5 机械臂的增益参数（kp/kd）。

## 前提条件

1. 已编译并安装 `arx_x5_ros2_control` 包
2. 机械臂硬件已连接并启动
3. ROS2 节点正在运行

## 双臂模式：启动时为左右臂设置不同 kp/kd

若希望**一启动**就为左臂和右臂使用不同的 kp/kd，可在 ros2_control 的硬件参数里为左右臂分别指定初始增益，硬件接口会从这些参数读取初始值。

在机器人描述的 ros2_control 中，在对应 `<hardware>` 的 `<param>` 里增加（双臂时生效）：

- `left_joint_k_gains`、`left_joint_d_gains`：左臂关节 kp/kd（6 个值的数组字符串）
- `right_joint_k_gains`、`right_joint_d_gains`：右臂关节 kp/kd（6 个值的数组字符串）
- 可选：`left_gripper_kp`、`left_gripper_kd`、`right_gripper_kp`、`right_gripper_kd`：左右夹爪增益

**示例（xacro）：**

```xml
<param name="left_joint_k_gains">[80.0, 70.0, 65.0, 22.0, 22.0, 14.0]</param>
<param name="left_joint_d_gains">[2.6, 2.6, 2.3, 0.85, 0.85, 0.6]</param>
<param name="right_joint_k_gains">[70.0, 60.0, 60.0, 20.0, 20.0, 12.0]</param>
<param name="right_joint_d_gains">[2.2, 2.2, 2.0, 0.75, 0.75, 0.5]</param>
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
- `/arx_lift2s_unified_system` - **双臂模式**下的硬件接口节点
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

### 双臂模式（DUAL）

双臂下硬件接口节点名为 **`/arx_lift2s_unified_system`**。左右臂各有独立增益，可分别查看，得到**两行**（左臂一行、右臂一行）：

```bash
# 双臂模式请使用节点 /arx_lift2s_unified_system

# 左臂增益（一行）
ros2 param get /arx_lift2s_unified_system left_joint_k_gains
ros2 param get /arx_lift2s_unified_system left_joint_d_gains
ros2 param get /arx_lift2s_unified_system left_gripper_kp
ros2 param get /arx_lift2s_unified_system left_gripper_kd

# 右臂增益（另一行）
ros2 param get /arx_lift2s_unified_system right_joint_k_gains
ros2 param get /arx_lift2s_unified_system right_joint_d_gains
ros2 param get /arx_lift2s_unified_system right_gripper_kp
ros2 param get /arx_lift2s_unified_system right_gripper_kd
```

**示例输出（两行，分别对应左/右臂）：**
```
# 左臂
String value is: [80.0, 70.0, 70.0, 30.0, 30.0, 20.0]
# 右臂
String value is: [80.0, 70.0, 70.0, 30.0, 30.0, 20.0]
```

双臂下仍保留统一参数 `joint_k_gains` / `joint_d_gains` / `gripper_kp` / `gripper_kd`：设置它们会**同时**更新左右臂为相同值；查看时仍只有一行，表示当前共用值（左右臂独立参数可分别查看）。

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

当 `arm_config` 为 `DUAL` 时，使用节点 **`/arx_lift2s_unified_system`**，可单独设置左臂或右臂的增益，互不影响：

```bash
# 双臂模式请使用节点 /arx_lift2s_unified_system

# 仅设置左臂关节增益
ros2 param set /arx_lift2s_unified_system left_joint_k_gains "[100.0, 90.0, 80.0, 40.0, 30.0, 25.0]"
ros2 param set /arx_lift2s_unified_system left_joint_d_gains "[3.0, 2.5, 2.5, 1.5, 1.0, 0.8]"
ros2 param set /arx_lift2s_unified_system left_gripper_kp 10.0
ros2 param set /arx_lift2s_unified_system left_gripper_kd 0.5

# 仅设置右臂关节增益
ros2 param set /arx_lift2s_unified_system right_joint_k_gains "[90.0, 80.0, 70.0, 35.0, 25.0, 20.0]"
ros2 param set /arx_lift2s_unified_system right_joint_d_gains "[2.5, 2.0, 2.0, 1.2, 0.8, 0.6]"
ros2 param set /arx_lift2s_unified_system right_gripper_kp 8.0
ros2 param set /arx_lift2s_unified_system right_gripper_kd 0.4
```

若设置统一参数 `joint_k_gains` / `joint_d_gains` / `gripper_kp` / `gripper_kd`，则**左右臂会同步为相同值**（与单臂用法一致）。

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

### 5.1 查看日志输出

参数设置成功后，节点会输出日志信息。单臂或设置统一参数时示例：

```
[INFO] [arx_x5_hardware]: joint_k_gains updated via parameter
[INFO] [arx_x5_hardware]: Left arm gains applied: kp=[100.0, 90.0, ...], kd=[2.0, 2.0, ...], gripper_kp=5.00, gripper_kd=0.20
```

双臂下若设置 `left_joint_k_gains` 或 `right_joint_k_gains`，会分别看到 `Left arm gains applied` 或 `Right arm gains applied`。

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

**解决方法：** 使用 `ros2 node list` 查找正确的节点名称（单臂多为 `/arx5_system`，双臂为 `/arx_lift2s_unified_system`）

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
# 单臂：节点多为 /arx5_system；双臂：节点为 /arx_lift2s_unified_system
NODE=/arx5_system                    # 单臂
# NODE=/arx_lift2s_unified_system   # 双臂时改用此节点

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
- 双臂：`/arx_lift2s_unified_system`

**单臂 / 统一参数（单臂必用，双臂可用来同步左右臂）：**
- `joint_k_gains` - 关节位置增益（6个值的数组）
- `joint_d_gains` - 关节阻尼增益（6个值的数组）
- `gripper_kp` - 夹爪位置增益（单个值）
- `gripper_kd` - 夹爪阻尼增益（单个值）

**双臂独立参数（仅当 `arm_config=DUAL` 时可用，查看时可得两行）：**
- `left_joint_k_gains` / `right_joint_k_gains` - 左/右臂关节位置增益
- `left_joint_d_gains` / `right_joint_d_gains` - 左/右臂关节阻尼增益
- `left_gripper_kp` / `right_gripper_kp` - 左/右夹爪位置增益
- `left_gripper_kd` / `right_gripper_kd` - 左/右夹爪阻尼增益

**快速设置示例：**
```bash
# 单臂（节点 /arx5_system）
ros2 param get /arx5_system joint_k_gains
ros2 param set /arx5_system joint_k_gains "[100.0, 90.0, 80.0, 40.0, 30.0, 25.0]"
ros2 param set /arx5_system joint_d_gains "[3.0, 2.5, 2.5, 1.5, 1.0, 0.8]"
ros2 param set /arx5_system gripper_kp 10.0
ros2 param set /arx5_system gripper_kd 0.5

# 双臂（节点 /arx_lift2s_unified_system，分别查看左右臂两行）
ros2 param get /arx_lift2s_unified_system left_joint_k_gains
ros2 param get /arx_lift2s_unified_system right_joint_k_gains
ros2 param set /arx_lift2s_unified_system left_joint_k_gains "[100.0, 90.0, 80.0, 40.0, 30.0, 25.0]"
ros2 param set /arx_lift2s_unified_system right_joint_k_gains "[90.0, 80.0, 70.0, 35.0, 25.0, 20.0]"
```
