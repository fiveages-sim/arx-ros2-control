#include "arx_ros2_control/arx_x5_hardware.h"
#include <pluginlib/class_list_macros.hpp>
#include <algorithm>
#include <unistd.h>  // for usleep
#include <sstream>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

namespace arx_ros2_control {

    // 默认增益值（6关节 X5 机械臂）
    static const std::vector<double> kDefaultJointKGains = {80.0, 70.0, 70.0, 30.0, 30.0, 20.0};
    static const std::vector<double> kDefaultJointDGains = {2.0, 2.0, 2.0, 1.0, 1.0, 0.7};
    static const double kDefaultGripperKP = 5.0;
    static const double kDefaultGripperKD = 0.2;

    // 真机夹爪：SDK 使用 [0, 0.088] m，URDF/仿真使用 [0, 0.044] m。状态上报时 ÷2 使仿真/可视化一致。
    static constexpr double kGripperPosScaleToRos = 0.5;

    // 辅助函数：解析 hardware_parameters 中的数组字符串为 vector<double>
    // 支持形如 "[1,2,3]"、"1, 2, 3"、"1 2 3" 等格式
    static std::vector<double> parseDoubleArrayLoose(const std::string& str, const std::vector<double>& default_val)
    {
        if (str.empty()) {
            return default_val;
        }
        std::string cleaned = str;
        cleaned.erase(std::remove(cleaned.begin(), cleaned.end(), '['), cleaned.end());
        cleaned.erase(std::remove(cleaned.begin(), cleaned.end(), ']'), cleaned.end());
        std::replace(cleaned.begin(), cleaned.end(), ',', ' ');

        std::istringstream iss(cleaned);
        std::vector<double> result;
        double v = 0.0;
        while (iss >> v) {
            result.push_back(v);
        }
        return result.empty() ? default_val : result;
    }

    // rclcpp 参数类型映射（用于 declare_node_parameters 的类型检查）
    template<typename T>
    static constexpr rclcpp::ParameterType paramTypeOf();

    template<>
    constexpr rclcpp::ParameterType paramTypeOf<std::string>() { return rclcpp::ParameterType::PARAMETER_STRING; }
    template<>
    constexpr rclcpp::ParameterType paramTypeOf<int>() { return rclcpp::ParameterType::PARAMETER_INTEGER; }
    template<>
    constexpr rclcpp::ParameterType paramTypeOf<double>() { return rclcpp::ParameterType::PARAMETER_DOUBLE; }
    template<>
    constexpr rclcpp::ParameterType paramTypeOf<bool>() { return rclcpp::ParameterType::PARAMETER_BOOL; }
    template<>
    constexpr rclcpp::ParameterType paramTypeOf<std::vector<double>>() { return rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY; }

/**
 * @brief 声明 ROS2 节点参数
 *
 * 从 URDF 的 hardware_parameters 中读取初始值，并声明为 ROS2 参数。
 * 这样可以通过 `ros2 param set` 动态修改参数值。
 *
 * 声明的参数包括：
 * - robot_model: 机械臂型号 (X5, L5 等)
 * - can_interface: CAN 接口名 (can0, can1 等)
 * - joint_k_gains / joint_d_gains / gripper_kp / gripper_kd: 增益（可动态调整）
 */
void ArxX5Hardware::declare_node_parameters()
{
    // 目标：优先使用 hardware_interface 的 info_.hardware_parameters 作为"初始值来源"，
    // 然后用该初始值声明为 ROS2 node 参数，便于后续 ros2 param set 动态修改。
    //
    // 规则：
    // - 如果参数已存在且类型正确：尊重现有值（可能来自 launch 覆盖/之前声明），不覆盖
    // - 如果参数已存在但类型不对：undeclare 后按正确类型重新 declare（避免类型冲突）
    // - 如果参数不存在：从 hardware_parameters 取值（若有）否则用默认值 declare

    const auto hw_find = [this](const std::string& name) -> const std::string* {
        auto it = info_.hardware_parameters.find(name);
        if (it == info_.hardware_parameters.end()) {
            return nullptr;
        }
        return &it->second;
    };

    // 辅助函数：确保字符串参数已声明
    const auto ensure_string_param = [this](const std::string& name, const std::string& default_val, const std::string* hw_val) {
        if (node_->has_parameter(name)) {
            // 检查类型是否正确
            if (node_->get_parameter(name).get_type() != rclcpp::ParameterType::PARAMETER_STRING) {
                try { 
                    node_->undeclare_parameter(name); 
                } catch (...) {}
            } else {
                return; // 已存在且类型正确：尊重现有值
            }
        }

        // 参数不存在或类型错误，需要声明
        if (!node_->has_parameter(name)) {
            const std::string val = hw_val ? *hw_val : default_val;
            node_->declare_parameter<std::string>(name, val);
        }
    };

    // 辅助函数：确保 double 参数已声明
    const auto ensure_double_param = [this](const std::string& name, double default_val, const std::string* hw_val) {
        if (node_->has_parameter(name)) {
            if (node_->get_parameter(name).get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
                try { 
                    node_->undeclare_parameter(name); 
                } catch (...) {}
            } else {
                return;
            }
        }

        if (!node_->has_parameter(name)) {
            double val = default_val;
            if (hw_val) {
                try {
                    val = std::stod(*hw_val);
                } catch (...) {
                    val = default_val;
                }
            }
            node_->declare_parameter<double>(name, val);
        }
    };

    // 辅助函数：确保 double array 参数已声明（带长度校验）
    const auto ensure_double_array_sized = [this, &hw_find](const std::string& name,
                                                           const std::vector<double>& default_val,
                                                           size_t expected_size) {
        const std::string* hw_val = hw_find(name);

        if (node_->has_parameter(name)) {
            if (node_->get_parameter(name).get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY) {
                try { 
                    node_->undeclare_parameter(name); 
                } catch (...) {}
            } else {
                // 类型正确，校验长度
                try {
                    const auto current = node_->get_parameter(name).get_value<std::vector<double>>();
                    if (current.size() == expected_size) {
                        return; // 已存在且长度正确：尊重现有值
                    }
                } catch (...) {
                    // fallthrough: undeclare + redeclare
                }
                try { 
                    node_->undeclare_parameter(name); 
                } catch (...) {}
            }
        }

        if (!node_->has_parameter(name)) {
            std::vector<double> val = hw_val ? parseDoubleArrayLoose(*hw_val, default_val) : default_val;
            if (val.size() != expected_size) {
                val = default_val;
            }
            node_->declare_parameter<std::vector<double>>(name, val);
        }
    };

    // 单臂配置：机械臂类型 + CAN 口
    ensure_string_param("robot_model", "X5", hw_find("robot_model"));
    ensure_string_param("can_interface", "can0", hw_find("can_interface"));

    // 增益参数（动态可调，默认 6 关节 X5）
    ensure_double_array_sized("joint_k_gains", kDefaultJointKGains, 6);
    ensure_double_array_sized("joint_d_gains", kDefaultJointDGains, 6);
    ensure_double_param("gripper_kp", kDefaultGripperKP, hw_find("gripper_kp"));
    ensure_double_param("gripper_kd", kDefaultGripperKD, hw_find("gripper_kd"));
}

// =============================================================================
// 生命周期回调（按调用顺序：on_init -> on_configure -> on_activate -> on_deactivate
// -> on_cleanup -> on_shutdown；on_error 可在任意状态由框架调用）
// =============================================================================

/**
 * @brief 硬件接口初始化回调（插件初始化）
 * @param params 硬件组件参数，包含 URDF 中定义的关节和参数信息
 * @return SUCCESS 表示初始化成功，ERROR 表示失败
 *
 * 生命周期：ros2_control 特有阶段，不属于标准 ROS2 lifecycle；在插件被加载时由框架调用，
 * 之后会立即调用 on_export_state_interfaces / on_export_command_interfaces 绑定状态与命令数组。
 *
 * 必须做的事：
 * 1. 调用父类 on_init（第一步，失败则直接返回 ERROR）
 * 2. 获取节点与日志器（node_ = get_node(); logger_ = ...）
 * 3. 声明所有 ROS2 参数（集中在 declare_node_parameters()，如 robot_model、can_interface、增益等）
 * 4. 解析 URDF 关节列表（遍历 params.hardware_info.joints），区分机械臂关节与夹爪关节，按单臂/双臂分配
 * 5. 按关节数分配状态/命令数组（resize 所有 *_states_ 与 *_commands_）
 * 6. 初始化 SDK 控制器指针为 nullptr（本阶段不连接硬件）
 *
 * 不做的事：不连接硬件、不注册参数回调、不分配命令 buffer（这些在 on_configure / on_activate 完成）
 */
hardware_interface::CallbackReturn ArxX5Hardware::on_init(
    const hardware_interface::HardwareComponentInterfaceParams& params) {

    // 1. 调用父类 on_init（失败则直接返回 ERROR）
    if (hardware_interface::SystemInterface::on_init(params) !=
        hardware_interface::CallbackReturn::SUCCESS) {
        return hardware_interface::CallbackReturn::ERROR;
    }

    // 2. 获取节点与日志器
    node_ = get_node();
    logger_ = get_node()->get_logger();

    // 3. 声明所有 ROS2 参数（robot_model、can_interface、增益等）
    declare_node_parameters();

    // 4. 读取单臂配置：机械臂类型 + CAN 口
    robot_model_ = get_node_param("robot_model", std::string("X5"));
    can_interface_ = get_node_param("can_interface", std::string("can0"));

    // 5. 解析 URDF 关节列表：区分机械臂关节与夹爪关节（单臂：所有非夹爪关节为一组）
    has_gripper_ = false;
    gripper_joint_names_.clear();
    joint_names_.clear();

    for (const auto& joint : params.hardware_info.joints) {
        std::string joint_name_lower = joint.name;
        std::transform(joint_name_lower.begin(), joint_name_lower.end(),
                       joint_name_lower.begin(), ::tolower);

        if (joint_name_lower.find("gripper") != std::string::npos) {
            has_gripper_ = true;
            gripper_joint_names_.push_back(joint.name);
            RCLCPP_INFO(get_logger(), "Detected gripper joint: %s", joint.name.c_str());
        } else {
            joint_names_.push_back(joint.name);
        }
    }

    joint_count_ = joint_names_.size();
    if (has_gripper_) {
        RCLCPP_INFO(get_logger(), "Found %zu gripper joint(s)", gripper_joint_names_.size());
    }

    // 6. 按关节数分配状态/命令数组（resize 所有 *_states_ 与 *_commands_）
    position_states_.resize(joint_count_, 0.0);
    velocity_states_.resize(joint_count_, 0.0);
    effort_states_.resize(joint_count_, 0.0);
    position_commands_.resize(joint_count_, 0.0);

    if (has_gripper_) {
        const size_t gripper_count = gripper_joint_names_.size();
        gripper_position_states_.resize(gripper_count, 0.0);
        gripper_velocity_states_.resize(gripper_count, 0.0);
        gripper_effort_states_.resize(gripper_count, 0.0);
        gripper_position_commands_.resize(gripper_count, 0.0);
    }

    // 7. 初始化 SDK 控制器指针为 nullptr（本阶段不连接硬件、不注册参数回调、不分配命令 buffer）
    controller_.reset();

    RCLCPP_INFO(get_logger(), "Initialized single-arm: %s on %s, %zu joints",
                robot_model_.c_str(), can_interface_.c_str(), joint_count_);

    return hardware_interface::CallbackReturn::SUCCESS;
}

/**
 * @brief 导出状态接口
 * @return 状态接口列表
 *
 * 生命周期：在 on_init 之后、on_activate 之前调用
 *
 * 导出的状态接口供控制器读取硬件状态：
 * - 每个机械臂关节导出 position, velocity, effort 三个状态
 * - 每个夹爪关节导出 position, velocity, effort 三个状态
 *
 * 控制器通过这些接口获取关节的实时位置、速度和力矩反馈
 */
std::vector<hardware_interface::StateInterface::ConstSharedPtr> ArxX5Hardware::on_export_state_interfaces() {
    std::vector<hardware_interface::StateInterface::ConstSharedPtr> state_interfaces;

    // 导出关节状态接口
    for (size_t i = 0; i < joint_count_; ++i) {
        state_interfaces.push_back(std::make_shared<hardware_interface::StateInterface>(
            joint_names_[i], hardware_interface::HW_IF_POSITION, &position_states_[i]));
        state_interfaces.push_back(std::make_shared<hardware_interface::StateInterface>(
            joint_names_[i], hardware_interface::HW_IF_VELOCITY, &velocity_states_[i]));
        state_interfaces.push_back(std::make_shared<hardware_interface::StateInterface>(
            joint_names_[i], hardware_interface::HW_IF_EFFORT, &effort_states_[i]));
    }

    // 导出夹爪状态接口（支持多个夹爪关节）
    if (has_gripper_) {
        for (size_t i = 0; i < gripper_joint_names_.size(); ++i) {
            state_interfaces.push_back(std::make_shared<hardware_interface::StateInterface>(
                gripper_joint_names_[i], hardware_interface::HW_IF_POSITION, &gripper_position_states_[i]));
            state_interfaces.push_back(std::make_shared<hardware_interface::StateInterface>(
                gripper_joint_names_[i], hardware_interface::HW_IF_VELOCITY, &gripper_velocity_states_[i]));
            state_interfaces.push_back(std::make_shared<hardware_interface::StateInterface>(
                gripper_joint_names_[i], hardware_interface::HW_IF_EFFORT, &gripper_effort_states_[i]));
        }
    }

    return state_interfaces;
}

/**
 * @brief 导出命令接口
 * @return 命令接口列表
 *
 * 生命周期：在 on_init 之后、on_activate 之前调用
 *
 * 导出的命令接口供控制器写入目标命令：
 * - 每个机械臂关节导出 position 命令接口
 * - 每个夹爪关节导出 position 命令接口
 *
 * 控制器通过这些接口发送目标位置指令给硬件
 */
std::vector<hardware_interface::CommandInterface::SharedPtr> ArxX5Hardware::on_export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface::SharedPtr> command_interfaces;

    // 导出关节命令接口
    for (size_t i = 0; i < joint_count_; ++i) {
        command_interfaces.push_back(std::make_shared<hardware_interface::CommandInterface>(
            joint_names_[i], hardware_interface::HW_IF_POSITION, &position_commands_[i]));
    }

    // 导出夹爪命令接口（支持多个夹爪关节）
    if (has_gripper_) {
        for (size_t i = 0; i < gripper_joint_names_.size(); ++i) {
            command_interfaces.push_back(std::make_shared<hardware_interface::CommandInterface>(
                gripper_joint_names_[i], hardware_interface::HW_IF_POSITION, &gripper_position_commands_[i]));
        }
    }

    return command_interfaces;
}

/**
 * @brief 硬件配置回调（Unconfigured → Inactive）
 * @param previous_state 之前的生命周期状态（未使用）
 * @return SUCCESS 表示配置成功，ERROR 表示失败
 *
 * 生命周期：Unconfigured → Inactive 状态转换时调用。与 on_cleanup() 对称：configure 里建立的，
 * cleanup 里释放。
 *
 * 必须做的事：
 * 1. 读取并缓存参数 — get_node_param() 读取已声明的参数（如增益），存入成员变量
 * 2. 注册参数回调 — add_on_set_parameters_callback，在 on_cleanup 中移除
 * 3. 连接硬件 — 创建 SDK 控制器（Arx5JointController）、打开 CAN，连接级校验（通讯通即可）
 * 4. 置连接标志 — hardware_connected_ = true
 *
 * 连接级校验原则：证明链路通（构造成功即可；本实现将“读一次状态”的验证放在 on_activate 的
 * reset_to_home/读状态中）。不要求硬件进入可控状态（使能/归零等在 on_activate 中完成）。
 */
hardware_interface::CallbackReturn ArxX5Hardware::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {

    RCLCPP_INFO(get_logger(), "Configuring ArxX5 Hardware Interface...");

    // 1. 读取并缓存参数（增益等；声明与长度矫正已在 on_init 的 declare_node_parameters 中完成）
    std::vector<double> current_kp = get_node_param("joint_k_gains", kDefaultJointKGains);
    std::vector<double> current_kd = get_node_param("joint_d_gains", kDefaultJointDGains);
    if (current_kp.size() != 6) {
        RCLCPP_WARN(get_logger(), "joint_k_gains size is %zu, expected 6; using default for cache.", current_kp.size());
        current_kp = kDefaultJointKGains;
    }
    if (current_kd.size() != 6) {
        RCLCPP_WARN(get_logger(), "joint_d_gains size is %zu, expected 6; using default for cache.", current_kd.size());
        current_kd = kDefaultJointDGains;
    }
    joint_k_gains_ = current_kp;
    joint_d_gains_ = current_kd;
    gripper_kp_ = get_node_param("gripper_kp", kDefaultGripperKP);
    gripper_kd_ = get_node_param("gripper_kd", kDefaultGripperKD);

    // 2. 注册参数回调（在 on_cleanup 中移除）
    param_callback_handle_ = node_->add_on_set_parameters_callback(
        std::bind(&ArxX5Hardware::paramCallback, this, std::placeholders::_1));
    RCLCPP_INFO(get_logger(), "Parameter callback registered. Use 'ros2 param set' to change gains dynamically.");

    // 3. 连接硬件：创建 SDK 控制器（通讯通验证在 on_activate 中做）
    try {
        RCLCPP_INFO(get_logger(), "Connecting arm: %s on %s", robot_model_.c_str(), can_interface_.c_str());
        controller_ = std::make_shared<arx::Arx5JointController>(robot_model_, can_interface_);
        hardware_connected_ = true;
        RCLCPP_INFO(get_logger(), "Hardware connected. Configuration complete. Ready to activate.");
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Failed to connect or probe hardware: %s", e.what());
        hardware_connected_ = false;
        controller_.reset();
        return hardware_interface::CallbackReturn::ERROR;
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

/**
 * @brief 硬件激活回调（Inactive → Active）
 * @param previous_state 之前的生命周期状态（未使用）
 * @return SUCCESS 表示激活成功，ERROR 表示失败
 *
 * 生命周期：Inactive → Active 状态转换时调用。与 on_deactivate() 对称：activate 里分配的，
 * deactivate 里释放。连接已在 on_configure 建立，此处使硬件进入可控状态并拿到有效状态。
 *
 * 必须做的事：
 * 1. 校验已连接 — hardware_connected_ 为 false 或 controllers_ 为空则直接返回 ERROR
 * 2. 使能/进入可控状态 — 例如 reset_to_home() 或 enable()，根据硬件要求（有的系统只 enable 不强制回 home）
 * 3. 读初始状态并校验 — 循环 get_joint_state()，校验无 NaN/Inf，有限次重试
 * 4. 对齐命令值 — position_commands_[i] = position_states_[i]（或初始 state），防止激活瞬间跳变
 * 5. 预分配命令 buffer — cmd_buffer_.emplace，整个 Active 期间复用
 * 6. 应用初始增益 — applyGains()
 *
 * 最后置 control_active_ = true，此后 read/write 才会访问硬件。若使能/读状态失败，清理控制器并返回 ERROR。
 */
hardware_interface::CallbackReturn ArxX5Hardware::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {

    // 1. 校验已连接（hardware_connected_ 与 controller_）
    if (!hardware_connected_) {
        RCLCPP_ERROR(get_logger(), "Cannot activate: hardware not connected (configure first).");
        return hardware_interface::CallbackReturn::ERROR;
    }
    if (!controller_) {
        RCLCPP_ERROR(get_logger(), "Cannot activate: controller is null.");
        return hardware_interface::CallbackReturn::ERROR;
    }

    const int max_read_attempts = 10;
    const int read_interval_ms = 100;
    auto has_invalid_values = [](const arx::JointState& state, size_t count) -> bool {
        for (size_t i = 0; i < count && i < static_cast<size_t>(state.pos.size()); ++i) {
            if (std::isnan(state.pos[i]) || std::isinf(state.pos[i])) {
                return true;
            }
        }
        return false;
    };

    try {
        RCLCPP_INFO(get_logger(), "Activating: reset_to_home and waiting for valid state...");

        // 2. 使能/进入可控状态（reset_to_home）
        controller_->reset_to_home();
        RCLCPP_INFO(get_logger(), "Arm reset to home position.");

        // 3. 读初始状态并校验（循环读取，无 NaN/Inf 则通过）
        bool initial_read_success = false;
        arx::JointState initial_state(joint_count_);

        for (int attempt = 0; attempt < max_read_attempts; ++attempt) {
            try {
                initial_state = controller_->get_joint_state();
                if (has_invalid_values(initial_state, joint_count_)) {
                    RCLCPP_WARN(get_logger(),
                        "Initial joint state contains NaN/Inf (attempt %d/%d), retrying...",
                        attempt + 1, max_read_attempts);
                    usleep(read_interval_ms * 1000);
                    continue;
                }
                initial_read_success = true;
                RCLCPP_INFO(get_logger(), "Initial joint state read successfully (attempt %d/%d)",
                           attempt + 1, max_read_attempts);
                break;
            } catch (const std::exception& e) {
                RCLCPP_WARN(get_logger(),
                    "Failed to read initial joint state (attempt %d/%d): %s",
                    attempt + 1, max_read_attempts, e.what());
                usleep(read_interval_ms * 1000);
            }
        }

        if (!initial_read_success) {
            RCLCPP_ERROR(get_logger(), "Failed to read valid initial joint state after %d attempts",
                        max_read_attempts);
            controller_.reset();
            return hardware_interface::CallbackReturn::ERROR;
        }

        // 4. 对齐命令值：写入初始状态并令 position_commands_ = 初始位置（防止激活瞬间跳变）
        for (size_t i = 0; i < joint_count_ && i < static_cast<size_t>(initial_state.pos.size()); ++i) {
            position_states_[i] = initial_state.pos[i];
            velocity_states_[i] = initial_state.vel[i];
            effort_states_[i] = initial_state.torque[i];
            position_commands_[i] = initial_state.pos[i];
        }
        if (has_gripper_ && gripper_joint_names_.size() >= 1) {
            gripper_position_states_[0] = initial_state.gripper_pos * kGripperPosScaleToRos;
            gripper_position_commands_[0] = initial_state.gripper_pos * kGripperPosScaleToRos;
        }

        // 5. 预分配命令 buffer（整个 Active 期间复用）
        cmd_buffer_.emplace(joint_count_);
        RCLCPP_DEBUG(get_logger(), "Pre-allocated command buffer: %zu joints", joint_count_);

        // 6. 应用初始增益
        applyGains(joint_k_gains_, joint_d_gains_, gripper_kp_, gripper_kd_);

        control_active_ = true;
        RCLCPP_INFO(get_logger(), "Successfully activated!");
        return hardware_interface::CallbackReturn::SUCCESS;

    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Failed to activate: %s", e.what());
        control_active_ = false;
        hardware_connected_ = false;
        controller_.reset();
        return hardware_interface::CallbackReturn::ERROR;
    }
}

/**
 * @brief 硬件停用回调（Active → Inactive）
 * @param previous_state 之前的生命周期状态（未使用）
 * @return SUCCESS 表示停用成功
 *
 * 生命周期：Active → Inactive 状态转换时调用。与 on_activate() 对称。
 *
 * activate 做了什么（回顾）：
 * - 校验已连接后，使能/进入可控状态（如 reset_to_home）
 * - 读初始状态、对齐命令值，预分配命令 buffer（cmd_buffer_）
 * - 应用初始增益，最后置 control_active_ = true，此后 read/write 才访问硬件
 *
 * deactivate 怎么做（本函数）：
 * - 置 control_active_ = false → read()/write() 直接 return OK，不再访问硬件（noop）
 * - 释放命令 buffer（cmd_buffer_.reset()），与 activate 中 emplace 对称
 * - 不断连、不释放控制器（hardware_connected_ 与 controllers_ 不变），连接与控制器在 on_cleanup 时释放；
 *   下次 activate 无需重新 configure，可快速再次使能。
 */
hardware_interface::CallbackReturn ArxX5Hardware::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {

    RCLCPP_INFO(get_logger(), "Deactivating (keeping connection for next activate)...");

    control_active_ = false;
    cmd_buffer_.reset();

    RCLCPP_INFO(get_logger(), "Successfully deactivated!");
    return hardware_interface::CallbackReturn::SUCCESS;
}

/**
 * @brief 硬件清理回调（Inactive → Unconfigured）
 * @param previous_state 之前的生命周期状态（未使用）
 * @return SUCCESS 表示清理成功
 *
 * 生命周期：Inactive → Unconfigured 状态转换时调用。与 on_configure() 对称。
 *
 * configure 做了什么（回顾）：
 * - 读取并缓存参数，注册参数回调（add_on_set_parameters_callback）
 * - 连接硬件（创建 SDK 控制器），置 hardware_connected_ = true
 *
 * cleanup 怎么做（本函数）：
 * - 移除参数回调 — param_callback_handle_.reset()（本实现仅 reset handle，析构时由 rclcpp 解绑）
 * - 兜底断开并释放 — 置 control_active_ = false；若 hardware_connected_ 仍为 true 打 WARN；
 *   无条件 controller_.reset()、hardware_connected_ = false，幂等、避免泄露
 * - 兜底释放命令 buffer — cmd_buffer_.reset()，防止未走 deactivate 时遗漏
 *
 * 注意：节点参数（declare_parameter）不在 cleanup 中 undeclare，参数保留到节点销毁。
 */
hardware_interface::CallbackReturn ArxX5Hardware::on_cleanup(
    const rclcpp_lifecycle::State& /*previous_state*/) {

    RCLCPP_INFO(get_logger(), "Cleaning up ArxX5 Hardware Interface...");

    // 1. 移除 configure 阶段注册的参数回调（仅 reset handle）
    param_callback_handle_.reset();

    control_active_ = false;
    if (hardware_connected_) {
        RCLCPP_WARN(get_logger(), "Hardware still connected during cleanup, disconnecting...");
    }
    controller_.reset();
    hardware_connected_ = false;
    cmd_buffer_.reset();

    RCLCPP_INFO(get_logger(), "Cleanup complete.");
    return hardware_interface::CallbackReturn::SUCCESS;
}

/**
 * @brief 硬件关闭回调（→ Finalized）
 * @param previous_state 之前的生命周期状态（未使用）
 * @return SUCCESS 表示关闭成功
 *
 * 生命周期：节点关闭时由 ros2_control 调用，对应文档中的 on_shutdown()。进程即将退出，
 * 做最后的安全清场；不保证一定先经过 deactivate/cleanup。
 *
 * 规范语义（on_shutdown）：无条件兜底 + 幂等
 * - 无条件兜底：确保硬件断开、资源释放，不追求“可重新 configure”
 * - 幂等：允许被调用多次而无额外副作用
 *
 * 本实现行为：
 * - 记录关闭日志
 * - 置 control_active_ = false，确保后续即便还有 read/write 调度也不会访问硬件
 * - 若仍处于连接状态则打 WARN 提示，然后无条件 reset 两个 controllers_ 并置 hardware_connected_ = false
 * - 兜底释放命令 buffer（cmd_buffer_.reset()）
 */
hardware_interface::CallbackReturn ArxX5Hardware::on_shutdown(
    const rclcpp_lifecycle::State& /*previous_state*/) {

    // 记录关闭日志
    RCLCPP_INFO(get_logger(), "Shutting down ArxX5 Hardware Interface...");

    // 置 control_active_ = false，后续 read/write 不再访问硬件
    control_active_ = false;
    if (hardware_connected_) {
        RCLCPP_WARN(get_logger(), "Hardware still connected during shutdown, disconnecting...");
    }
    controller_.reset();
    hardware_connected_ = false;
    cmd_buffer_.reset();

    RCLCPP_INFO(get_logger(), "Shutdown complete.");
    return hardware_interface::CallbackReturn::SUCCESS;
}

/**
 * @brief 硬件错误处理回调（错误路径兜底）
 * @param previous_state 之前的生命周期状态（未使用）
 * @return SUCCESS 表示错误处理完成
 *
 * 生命周期：当任意生命周期回调（on_configure/on_activate/on_deactivate/on_cleanup/on_shutdown）
 * 返回 ERROR 时由 ros2_control 调用，对应文档中的 on_error()。
 *
 * 规范语义（on_error）：清理资源 + 告诉框架“是否可恢复”
 * - 触发来源：任一生命周期回调返回 ERROR
 * - 返回值含义：SUCCESS → 转到 Unconfigured，可重新 configure；ERROR → 进入 Finalized，彻底终止
 *
 * 本实现行为：
 * - 记录错误日志
 * - 置 control_active_ = false、hardware_connected_ = false，表示当前不会再进行读写或控制
 * - 安全释放 controller_，并清理命令 buffer（cmd_buffer_.reset()）
 * - 始终返回 SUCCESS：表示错误已通过兜底清理处理完毕，框架可回到 Unconfigured 重新 configure
 */
hardware_interface::CallbackReturn ArxX5Hardware::on_error(
    const rclcpp_lifecycle::State& /*previous_state*/) {

    // 记录错误日志
    RCLCPP_ERROR(get_logger(), "Error in ArxX5 Hardware Interface");

    control_active_ = false;
    hardware_connected_ = false;
    controller_.reset();
    cmd_buffer_.reset();

    // 返回 SUCCESS：错误已兜底清理，框架可回到 Unconfigured 重新 configure
    return hardware_interface::CallbackReturn::SUCCESS;
}

// =============================================================================
// 控制循环接口（active 时由 controller_manager 周期性调用）
// =============================================================================

/**
 * @brief 从硬件读取状态
 * @param time 当前时间（未使用）
 * @param period 控制周期（未使用）
 * @return OK 表示读取成功，ERROR 表示失败
 *
 * 调用频率：由 ros2_control 的控制循环决定（通常 100-1000 Hz）
 *
 * 主要工作：
 * 1. 从 ARX SDK 获取关节状态（位置、速度、力矩）
 * 2. 更新状态数组，供控制器通过状态接口读取
 * 3. 双臂模式下分别读取左右臂状态
 * 4. 同时更新夹爪状态
 *
 * inactive（control_active_ == false）时：直接返回 OK，不访问硬件（noop）。
 * 数据流：ARX SDK -> position_states_/velocity_states_/effort_states_ -> 控制器
 */
hardware_interface::return_type ArxX5Hardware::read(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& /*period*/) {

    if (!control_active_) {
        return hardware_interface::return_type::OK;
    }
    if (!hardware_connected_ || !controller_) {
        return hardware_interface::return_type::ERROR;
    }

    auto validate_and_update = [this](double raw, double& state, const char* name, int idx) {
        if (std::isnan(raw) || std::isinf(raw)) {
            RCLCPP_WARN_THROTTLE(get_logger(), *node_->get_clock(), 1000,
                "%s[%d] is NaN/Inf, keeping previous value: %.3f", name, idx, state);
        } else {
            state = raw;
        }
    };

    try {
        arx::JointState state = controller_->get_joint_state();
        for (size_t i = 0; i < joint_count_ && i < static_cast<size_t>(state.pos.size()); ++i) {
            validate_and_update(state.pos[i], position_states_[i], "Joint position", static_cast<int>(i));
            validate_and_update(state.vel[i], velocity_states_[i], "Joint velocity", static_cast<int>(i));
            validate_and_update(state.torque[i], effort_states_[i], "Joint effort", static_cast<int>(i));
        }
        if (has_gripper_ && gripper_joint_names_.size() >= 1) {
            validate_and_update(state.gripper_pos * kGripperPosScaleToRos, gripper_position_states_[0], "Gripper position", 0);
            validate_and_update(state.gripper_vel * kGripperPosScaleToRos, gripper_velocity_states_[0], "Gripper velocity", 0);
            validate_and_update(state.gripper_torque, gripper_effort_states_[0], "Gripper effort", 0);
        }
        return hardware_interface::return_type::OK;

    } catch (const std::exception& e) {
        RCLCPP_ERROR_THROTTLE(get_logger(), *node_->get_clock(), 1000,
                              "Read failed: %s", e.what());
        return hardware_interface::return_type::ERROR;
    }
}

/**
 * @brief 向硬件写入命令
 * @param time 当前时间（未使用）
 * @param period 控制周期（未使用）
 * @return OK 表示写入成功，ERROR 表示失败
 *
 * 调用频率：由 ros2_control 的控制循环决定（通常 100-1000 Hz）
 *
 * 主要工作：
 * 1. 从命令数组读取目标位置（由控制器通过命令接口写入）
 * 2. 构建 ARX SDK 的 JointState 命令结构
 * 3. 发送命令到 ARX SDK，驱动电机运动
 * 4. 双臂模式下分别向左右臂发送命令
 * 5. 同时发送夹爪命令
 *
 * inactive（control_active_ == false）时：直接返回 OK，不访问硬件（noop）。
 * 数据流：控制器 -> position_commands_ -> ARX SDK -> 电机
 */
hardware_interface::return_type ArxX5Hardware::write(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& /*period*/) {

    if (!control_active_) {
        return hardware_interface::return_type::OK;
    }
    if (!hardware_connected_ || !controller_) {
        return hardware_interface::return_type::ERROR;
    }
    if (!cmd_buffer_) {
        RCLCPP_ERROR_THROTTLE(get_logger(), *node_->get_clock(), 1000, "Command buffer not initialized");
        return hardware_interface::return_type::ERROR;
    }

    try {
        arx::JointState& cmd = *cmd_buffer_;
        for (size_t i = 0; i < joint_count_; ++i) {
            cmd.pos[i] = position_commands_[i];
        }
        if (has_gripper_ && gripper_joint_names_.size() >= 1) {
            cmd.gripper_pos = gripper_position_commands_[0] * 2.0;
        }
        controller_->set_joint_cmd(cmd);
        return hardware_interface::return_type::OK;

    } catch (const std::exception& e) {
        RCLCPP_ERROR_THROTTLE(get_logger(), *node_->get_clock(), 1000,
                              "Write failed: %s", e.what());
        return hardware_interface::return_type::ERROR;
    }
}

// =============================================================================
// 其他：参数回调与增益下发
// =============================================================================

/**
 * @brief 参数回调函数
 * @param params 参数列表
 * @return 设置参数的结果
 *
 * 处理动态参数修改，特别是增益参数（kp/kd）的调整
 */
rcl_interfaces::msg::SetParametersResult ArxX5Hardware::paramCallback(
    const std::vector<rclcpp::Parameter> & params)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    bool gain_cached_not_applied = false;  // 有增益仅缓存、未立即下发时置 true，用于末尾提示

    for (const auto & param : params) {
        // 增益参数：未连接也可设置（仅缓存），连接且 activate 时才 apply；只有“需立即写硬件”时才要求在线
        if (param.get_name() == "joint_k_gains") {
            std::vector<double> new_kp = param.as_double_array();
            // 当前只支持6关节机械臂
            const size_t expected_count = 6;
            
            if (new_kp.size() != expected_count) {
                result.successful = false;
                result.reason = "joint_k_gains must have exactly " + std::to_string(expected_count) + " values (for 6-joint robot)";
                return result;
            }
            
            joint_k_gains_ = new_kp;
            RCLCPP_INFO(get_logger(), "joint_k_gains updated via parameter");
            if (hardware_connected_ && control_active_) {
                applyGains(joint_k_gains_, joint_d_gains_, gripper_kp_, gripper_kd_);
            } else {
                gain_cached_not_applied = true;
            }
        }
        else if (param.get_name() == "joint_d_gains") {
            std::vector<double> new_kd = param.as_double_array();
            const size_t expected_count = 6;
            if (new_kd.size() != expected_count) {
                result.successful = false;
                result.reason = "joint_d_gains must have exactly " + std::to_string(expected_count) + " values (for 6-joint robot)";
                return result;
            }
            joint_d_gains_ = new_kd;
            RCLCPP_INFO(get_logger(), "joint_d_gains updated via parameter");
            if (hardware_connected_ && control_active_) {
                applyGains(joint_k_gains_, joint_d_gains_, gripper_kp_, gripper_kd_);
            } else {
                gain_cached_not_applied = true;
            }
        }
        else if (param.get_name() == "gripper_kp") {
            double new_gripper_kp = param.as_double();
            if (new_gripper_kp < 0.0) {
                result.successful = false;
                result.reason = "gripper_kp must be >= 0";
                return result;
            }
            gripper_kp_ = new_gripper_kp;
            RCLCPP_INFO(get_logger(), "gripper_kp updated to: %.2f", gripper_kp_);
            if (hardware_connected_ && control_active_) {
                applyGains(joint_k_gains_, joint_d_gains_, gripper_kp_, gripper_kd_);
            } else {
                gain_cached_not_applied = true;
            }
        }
        else if (param.get_name() == "gripper_kd") {
            double new_gripper_kd = param.as_double();
            if (new_gripper_kd < 0.0) {
                result.successful = false;
                result.reason = "gripper_kd must be >= 0";
                return result;
            }
            gripper_kd_ = new_gripper_kd;
            RCLCPP_INFO(get_logger(), "gripper_kd updated to: %.2f", gripper_kd_);
            if (hardware_connected_ && control_active_) {
                applyGains(joint_k_gains_, joint_d_gains_, gripper_kp_, gripper_kd_);
            } else {
                gain_cached_not_applied = true;
            }
        }
    }

    if (gain_cached_not_applied) {
        RCLCPP_INFO(get_logger(), "Gain parameter(s) cached; will be applied when connected and activated.");
    }
    return result;
}

/**
 * @brief 应用增益到硬件
 */
void ArxX5Hardware::applyGains(const std::vector<double>& kp, const std::vector<double>& kd,
                               double gripper_kp, double gripper_kd)
{
    if (!controller_) {
        RCLCPP_WARN(get_logger(), "Controller not initialized, cannot apply gains");
        return;
    }
    const size_t joint_count = 6;
    if (kp.size() < joint_count || kd.size() < joint_count) {
        RCLCPP_ERROR(get_logger(),
            "Gain arrays too short: kp.size()=%zu, kd.size()=%zu, expected %zu",
            kp.size(), kd.size(), joint_count);
        return;
    }
    try {
        arx::Gain gain(static_cast<int>(joint_count));
        for (size_t i = 0; i < joint_count; ++i) {
            gain.kp[i] = kp[i];
            gain.kd[i] = kd[i];
        }
        gain.gripper_kp = static_cast<float>(gripper_kp);
        gain.gripper_kd = static_cast<float>(gripper_kd);
        controller_->set_gain(gain);
        RCLCPP_INFO(get_logger(), "Gains applied: kp=[%.1f, %.1f, ...], kd=[%.1f, %.1f, ...], gripper_kp=%.2f, gripper_kd=%.2f",
            kp.empty() ? 0.0 : kp[0], kp.size() > 1 ? kp[1] : 0.0,
            kd.empty() ? 0.0 : kd[0], kd.size() > 1 ? kd[1] : 0.0,
            gripper_kp, gripper_kd);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Failed to apply gains: %s", e.what());
    }
}

}  // namespace arx_ros2_control

/**
 * @brief 注册硬件接口插件
 *
 * 将 ArxX5Hardware 类注册为 ros2_control 的 SystemInterface 插件。
 * ros2_control 通过 pluginlib 动态加载此硬件接口。
 *
 * 配合 arx_ros2_control.xml 插件描述文件使用。
 */
PLUGINLIB_EXPORT_CLASS(arx_ros2_control::ArxX5Hardware, hardware_interface::SystemInterface)
