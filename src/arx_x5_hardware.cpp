#include "arx_x5_ros2_control/arx_x5_hardware.h"
#include <pluginlib/class_list_macros.hpp>
#include <algorithm>
#include <unistd.h>  // for usleep
#include <sstream>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

namespace arx_x5_ros2_control {

    // 默认增益值（6关节 X5 机械臂）
    static const std::vector<double> kDefaultJointKGains = {80.0, 70.0, 70.0, 30.0, 30.0, 20.0};
    static const std::vector<double> kDefaultJointDGains = {2.0, 2.0, 2.0, 1.0, 1.0, 0.7};
    static const double kDefaultGripperKP = 5.0;
    static const double kDefaultGripperKD = 0.2;

    // 辅助函数：字符串标准化（转大写）
    static std::string normalizeString(const std::string& str)
    {
        std::string result = str;
        std::transform(result.begin(), result.end(), result.begin(), ::toupper);
        return result;
    }

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
 * @brief 将字符串转换为大写形式
 * @param str 输入字符串
 * @return 转换后的大写字符串
 *
 * 用于参数解析时统一字符串格式，如 "left" -> "LEFT"
 */
std::string ArxX5Hardware::normalizeString(const std::string& str)
{
    std::string result = str;
    std::transform(result.begin(), result.end(), result.begin(), ::toupper);
    return result;
}

/**
 * @brief 声明 ROS2 节点参数
 *
 * 从 URDF 的 hardware_parameters 中读取初始值，并声明为 ROS2 参数。
 * 这样可以通过 `ros2 param set` 动态修改参数值。
 *
 * 声明的参数包括：
 * - arm_config: 臂配置 ("LEFT", "RIGHT", "DUAL")
 * - robot_model / can_interface: 单臂模式配置
 * - left_robot_model / right_robot_model: 双臂模式型号配置
 * - left_can_interface / right_can_interface: 双臂模式 CAN 接口配置
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

    // arm_config: 臂配置 ("LEFT", "RIGHT", "DUAL")
    ensure_string_param("arm_config", "LEFT", hw_find("arm_config"));
    
    // 单臂配置（向后兼容）
    ensure_string_param("robot_model", "X5", hw_find("robot_model"));
    ensure_string_param("can_interface", "can0", hw_find("can_interface"));
    
    // 双臂配置
    ensure_string_param("left_robot_model", "X5", hw_find("left_robot_model"));
    ensure_string_param("right_robot_model", "X5", hw_find("right_robot_model"));
    ensure_string_param("left_can_interface", "can0", hw_find("left_can_interface"));
    ensure_string_param("right_can_interface", "can1", hw_find("right_can_interface"));

    // 增益参数（动态可调）
    // 默认使用6关节的配置（X5机械臂）
    ensure_double_array_sized("joint_k_gains", kDefaultJointKGains, 6);
    ensure_double_array_sized("joint_d_gains", kDefaultJointDGains, 6);
    ensure_double_param("gripper_kp", kDefaultGripperKP, hw_find("gripper_kp"));
    ensure_double_param("gripper_kd", kDefaultGripperKD, hw_find("gripper_kd"));
}

/**
 * @brief 硬件接口初始化回调
 * @param params 硬件组件参数，包含 URDF 中定义的关节和参数信息
 * @return SUCCESS 表示初始化成功，ERROR 表示失败
 *
 * 生命周期：在 ros2_control 加载硬件插件时调用（unconfigured -> inactive 之前）
 *
 * 主要工作：
 * 1. 调用父类初始化
 * 2. 获取 ROS2 节点和日志器
 * 3. 声明并读取配置参数（arm_config, robot_model, can_interface 等）
 * 4. 解析 URDF 中的关节信息，区分机械臂关节和夹爪关节
 * 5. 根据单臂/双臂模式分配关节到左臂或右臂
 * 6. 初始化状态和命令数组
 */
hardware_interface::CallbackReturn ArxX5Hardware::on_init(
    const hardware_interface::HardwareComponentInterfaceParams& params) {

    if (hardware_interface::SystemInterface::on_init(params) !=
        hardware_interface::CallbackReturn::SUCCESS) {
        return hardware_interface::CallbackReturn::ERROR;
    }

    // 获取节点和日志器
    node_ = get_node();
    logger_ = get_node()->get_logger();

    // 声明节点参数
    declare_node_parameters();

    // 解析arm_config参数
    std::string arm_config_raw = get_node_param("arm_config", std::string("LEFT"));
    arm_config_ = normalizeString(arm_config_raw);
    
    if (arm_config_ == "LEFT") {
        arm_index_ = ARM_LEFT;
    } else if (arm_config_ == "RIGHT") {
        arm_index_ = ARM_RIGHT;
    } else if (arm_config_ == "DUAL") {
        arm_index_ = ARM_DUAL;
    } else {
        RCLCPP_WARN(get_logger(), 
                    "Unknown arm_config '%s', using default LEFT. Valid options: LEFT, RIGHT, DUAL", 
                    arm_config_raw.c_str());
        arm_config_ = "LEFT";
        arm_index_ = ARM_LEFT;
    }

    // 读取配置参数
    if (arm_index_ == ARM_DUAL) {
        // 双臂模式：使用左右臂独立配置
        left_robot_model_ = get_node_param("left_robot_model", std::string("X5"));
        right_robot_model_ = get_node_param("right_robot_model", std::string("X5"));
        left_can_interface_ = get_node_param("left_can_interface", std::string("can0"));
        right_can_interface_ = get_node_param("right_can_interface", std::string("can1"));
    } else {
        // 单臂模式：使用向后兼容的配置
        robot_model_ = get_node_param("robot_model", std::string("X5"));
        can_interface_ = get_node_param("can_interface", std::string("can0"));
        // 同时设置对应的左右臂配置（用于统一处理）
        if (arm_index_ == ARM_LEFT) {
            left_robot_model_ = robot_model_;
            left_can_interface_ = can_interface_;
        } else {
            right_robot_model_ = robot_model_;
            right_can_interface_ = can_interface_;
        }
    }

    // 解析关节：区分左臂和右臂关节
    has_gripper_ = false;
    gripper_joint_names_.clear();
    joint_names_.clear();
    left_joint_names_.clear();
    right_joint_names_.clear();
    
    for (const auto& joint : params.hardware_info.joints) {
        std::string joint_name_lower = joint.name;
        std::transform(joint_name_lower.begin(), joint_name_lower.end(),
                       joint_name_lower.begin(), ::tolower);

        if (joint_name_lower.find("gripper") != std::string::npos) {
            has_gripper_ = true;
            gripper_joint_names_.push_back(joint.name);
            RCLCPP_INFO(get_logger(),
                        "Detected gripper joint: %s", joint.name.c_str());
        } else {
            joint_names_.push_back(joint.name);
            
            // 根据关节名称前缀判断属于左臂还是右臂
            // 假设左臂关节名称包含"left"或"l_"，右臂包含"right"或"r_"
            if (arm_index_ == ARM_DUAL) {
                if (joint_name_lower.find("left") != std::string::npos) {
                    left_joint_names_.push_back(joint.name);
                } else if (joint_name_lower.find("right") != std::string::npos) {
                    right_joint_names_.push_back(joint.name);
                } else {
                    // 如果没有明确的前缀，按顺序分配：前半部分给左臂，后半部分给右臂
                    // 这里假设关节数量是偶数，且按顺序排列
                    if (left_joint_names_.size() < right_joint_names_.size()) {
                        left_joint_names_.push_back(joint.name);
                    } else {
                        right_joint_names_.push_back(joint.name);
                    }
                }
            } else if (arm_index_ == ARM_LEFT) {
                left_joint_names_.push_back(joint.name);
            } else {
                right_joint_names_.push_back(joint.name);
            }
        }
    }
    
    // 计算关节数量
    joint_count_ = joint_names_.size();
    left_joint_count_ = left_joint_names_.size();
    right_joint_count_ = right_joint_names_.size();
    
    if (arm_index_ == ARM_DUAL) {
        RCLCPP_INFO(get_logger(),
                    "Dual-arm configuration: left=%zu joints, right=%zu joints, total=%zu joints",
                    left_joint_count_, right_joint_count_, joint_count_);
    }
    
    if (has_gripper_) {
        const size_t expected_grippers = (arm_index_ == ARM_DUAL) ? 2 : 1;
        RCLCPP_INFO(get_logger(),
                    "Found %zu gripper joint(s), expected %zu gripper(s) for %s arm config",
                    gripper_joint_names_.size(), expected_grippers, arm_config_.c_str());
        if (gripper_joint_names_.size() != expected_grippers) {
            RCLCPP_WARN(get_logger(),
                        "Gripper count mismatch: found %zu but expected %zu for %s mode. "
                        "Right arm gripper may not function correctly.",
                        gripper_joint_names_.size(), expected_grippers, arm_config_.c_str());
        }
    }

    // 初始化状态和命令数组
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

    // 初始化控制器指针
    controllers_[0] = nullptr;
    controllers_[1] = nullptr;

    RCLCPP_INFO(get_logger(),
                "Initialized %s arm config with %zu joints",
                arm_config_.c_str(), joint_count_);

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
std::vector<hardware_interface::StateInterface> ArxX5Hardware::export_state_interfaces() {
    std::vector<hardware_interface::StateInterface> state_interfaces;

    // 导出关节状态接口
    for (size_t i = 0; i < joint_count_; ++i) {
        state_interfaces.emplace_back(
            joint_names_[i], hardware_interface::HW_IF_POSITION, &position_states_[i]);
        state_interfaces.emplace_back(
            joint_names_[i], hardware_interface::HW_IF_VELOCITY, &velocity_states_[i]);
        state_interfaces.emplace_back(
            joint_names_[i], hardware_interface::HW_IF_EFFORT, &effort_states_[i]);
    }

    // 导出夹爪状态接口（支持多个夹爪关节）
    if (has_gripper_) {
        for (size_t i = 0; i < gripper_joint_names_.size(); ++i) {
            state_interfaces.emplace_back(
                gripper_joint_names_[i], hardware_interface::HW_IF_POSITION, &gripper_position_states_[i]);
            state_interfaces.emplace_back(
                gripper_joint_names_[i], hardware_interface::HW_IF_VELOCITY, &gripper_velocity_states_[i]);
            state_interfaces.emplace_back(
                gripper_joint_names_[i], hardware_interface::HW_IF_EFFORT, &gripper_effort_states_[i]);
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
std::vector<hardware_interface::CommandInterface> ArxX5Hardware::export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    // 导出关节命令接口
    for (size_t i = 0; i < joint_count_; ++i) {
        command_interfaces.emplace_back(
            joint_names_[i], hardware_interface::HW_IF_POSITION, &position_commands_[i]);
    }

    // 导出夹爪命令接口（支持多个夹爪关节）
    if (has_gripper_) {
        for (size_t i = 0; i < gripper_joint_names_.size(); ++i) {
            command_interfaces.emplace_back(
                gripper_joint_names_[i], hardware_interface::HW_IF_POSITION, &gripper_position_commands_[i]);
        }
    }

    return command_interfaces;
}

/**
 * @brief 硬件配置回调
 * @param previous_state 之前的生命周期状态（未使用）
 * @return SUCCESS 表示配置成功，ERROR 表示失败
 *
 * 生命周期：unconfigured -> inactive 状态转换时调用
 *
 * 主要工作：
 * 1. 验证配置参数的有效性
 * 2. 记录配置信息
 *
 * 此阶段不连接硬件，仅验证配置是否正确
 */
hardware_interface::CallbackReturn ArxX5Hardware::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {

    RCLCPP_INFO(get_logger(), "Configuring ArxX5 Hardware Interface...");

    // 验证配置参数
    if (arm_index_ == ARM_DUAL) {
        RCLCPP_INFO(get_logger(), "Configuration: Dual-arm mode");
        RCLCPP_INFO(get_logger(), "  Left arm: model=%s, CAN=%s, joints=%zu",
                    left_robot_model_.c_str(), left_can_interface_.c_str(), left_joint_count_);
        RCLCPP_INFO(get_logger(), "  Right arm: model=%s, CAN=%s, joints=%zu",
                    right_robot_model_.c_str(), right_can_interface_.c_str(), right_joint_count_);
    } else {
        const std::string& model = (arm_index_ == ARM_LEFT) ? left_robot_model_ : right_robot_model_;
        const std::string& can_if = (arm_index_ == ARM_LEFT) ? left_can_interface_ : right_can_interface_;
        RCLCPP_INFO(get_logger(), "Configuration: %s arm mode, model=%s, CAN=%s, joints=%zu",
                    arm_config_.c_str(), model.c_str(), can_if.c_str(), joint_count_);
    }

    if (has_gripper_) {
        RCLCPP_INFO(get_logger(), "  Gripper: %zu joint(s)", gripper_joint_names_.size());
    }

    // 根据实际关节数量调整增益参数长度
    // 注意：在 on_configure 时，关节数量已经确定（在 on_init 中解析）
    // 当前只支持6关节机械臂（X5）
    size_t expected_joint_count = (arm_index_ == ARM_DUAL) ? 
        std::max(left_joint_count_, right_joint_count_) : joint_count_;
    
    // 验证关节数量是否为6（当前只支持6关节）
    if (expected_joint_count != 6) {
        RCLCPP_WARN(get_logger(), 
                    "Expected 6 joints for X5 robot, but got %zu joints. Using 6-joint default gains.",
                    expected_joint_count);
        expected_joint_count = 6;
    }
    
    // 获取当前增益参数，如果长度不匹配则调整为6关节默认值
    std::vector<double> current_kp = get_node_param("joint_k_gains", kDefaultJointKGains);
    std::vector<double> current_kd = get_node_param("joint_d_gains", kDefaultJointDGains);
    
    // 如果长度不匹配，使用6关节默认值并更新参数
    if (current_kp.size() != 6) {
        current_kp = kDefaultJointKGains;
        // 重新声明参数以更新长度
        try {
            node_->undeclare_parameter("joint_k_gains");
        } catch (...) {}
        node_->declare_parameter<std::vector<double>>("joint_k_gains", current_kp);
        RCLCPP_INFO(get_logger(), "Adjusted joint_k_gains to 6-joint default values");
    }
    if (current_kd.size() != 6) {
        current_kd = kDefaultJointDGains;
        try {
            node_->undeclare_parameter("joint_d_gains");
        } catch (...) {}
        node_->declare_parameter<std::vector<double>>("joint_d_gains", current_kd);
        RCLCPP_INFO(get_logger(), "Adjusted joint_d_gains to 6-joint default values");
    }
    
    // 初始化增益缓存
    joint_k_gains_ = current_kp;
    joint_d_gains_ = current_kd;
    gripper_kp_ = get_node_param("gripper_kp", kDefaultGripperKP);
    gripper_kd_ = get_node_param("gripper_kd", kDefaultGripperKD);

    // 注册参数回调
    param_callback_handle_ = node_->add_on_set_parameters_callback(
        std::bind(&ArxX5Hardware::paramCallback, this, std::placeholders::_1));
    RCLCPP_INFO(get_logger(), "Parameter callback registered. Use 'ros2 param set' to change gains dynamically.");

    RCLCPP_INFO(get_logger(), "Configuration complete. Ready to activate.");
    return hardware_interface::CallbackReturn::SUCCESS;
}

/**
 * @brief 硬件清理回调
 * @param previous_state 之前的生命周期状态（未使用）
 * @return SUCCESS 表示清理成功
 *
 * 生命周期：inactive -> unconfigured 状态转换时调用
 *
 * 主要工作：
 * 1. 确保硬件已断开连接
 * 2. 重置状态，准备重新配置
 */
hardware_interface::CallbackReturn ArxX5Hardware::on_cleanup(
    const rclcpp_lifecycle::State& /*previous_state*/) {

    RCLCPP_INFO(get_logger(), "Cleaning up ArxX5 Hardware Interface...");

    // 确保硬件已断开
    if (hardware_connected_) {
        RCLCPP_WARN(get_logger(), "Hardware still connected during cleanup, disconnecting...");
        hardware_connected_ = false;
        controllers_[0].reset();
        controllers_[1].reset();
    }

    // 清理命令缓冲区
    left_cmd_buffer_.reset();
    right_cmd_buffer_.reset();
    single_cmd_buffer_.reset();

    RCLCPP_INFO(get_logger(), "Cleanup complete.");
    return hardware_interface::CallbackReturn::SUCCESS;
}

/**
 * @brief 硬件激活回调
 * @param previous_state 之前的生命周期状态（未使用）
 * @return SUCCESS 表示激活成功，ERROR 表示失败
 *
 * 生命周期：inactive -> active 状态转换时调用
 *
 * 主要工作：
 * 1. 根据配置创建 ARX SDK 控制器实例
 *    - 单臂模式：创建一个控制器
 *    - 双臂模式：创建左右两个控制器
 * 2. 连接到真实硬件（通过 CAN 接口）
 * 3. 读取当前关节状态作为初始值
 * 4. 将初始位置同时设为命令值（避免激活时跳变）
 *
 * 错误处理：如果连接失败，清理已创建的控制器并返回 ERROR
 */
hardware_interface::CallbackReturn ArxX5Hardware::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {

    // 初始状态验证参数
    const int max_read_attempts = 10;
    const int read_interval_ms = 100;

    // 辅助函数：检查关节状态是否全零（可能表示硬件未就绪）
    auto is_all_zeros = [](const arx::JointState& state, size_t count) -> bool {
        for (size_t i = 0; i < count && i < static_cast<size_t>(state.pos.size()); ++i) {
            if (std::abs(state.pos[i]) > 1e-6) {
                return false;
            }
        }
        return true;
    };

    // 辅助函数：检查关节状态是否包含 NaN/Inf
    auto has_invalid_values = [](const arx::JointState& state, size_t count) -> bool {
        for (size_t i = 0; i < count && i < static_cast<size_t>(state.pos.size()); ++i) {
            if (std::isnan(state.pos[i]) || std::isinf(state.pos[i])) {
                return true;
            }
        }
        return false;
    };

    try {
        if (arm_index_ == ARM_DUAL) {
            // 双臂模式：创建两个控制器
            RCLCPP_INFO(get_logger(),
                        "Activating dual-arm: Left=%s on %s, Right=%s on %s",
                        left_robot_model_.c_str(), left_can_interface_.c_str(),
                        right_robot_model_.c_str(), right_can_interface_.c_str());

            controllers_[ARM_LEFT] = std::make_shared<arx::Arx5JointController>(
                left_robot_model_, left_can_interface_);
            controllers_[ARM_RIGHT] = std::make_shared<arx::Arx5JointController>(
                right_robot_model_, right_can_interface_);

            // 重置到 HOME 位置（确保机械臂处于已知安全位置）
            RCLCPP_INFO(get_logger(), "Resetting left arm to home position...");
            controllers_[ARM_LEFT]->reset_to_home();
            RCLCPP_INFO(get_logger(), "Resetting right arm to home position...");
            controllers_[ARM_RIGHT]->reset_to_home();
            RCLCPP_INFO(get_logger(), "Both arms reset to home position.");

            // 读取左臂初始状态（带验证和重试）
            bool left_success = false;
            bool left_all_zeros_detected = false;
            arx::JointState left_state(left_joint_count_);

            for (int attempt = 0; attempt < max_read_attempts; ++attempt) {
                try {
                    left_state = controllers_[ARM_LEFT]->get_joint_state();

                    if (has_invalid_values(left_state, left_joint_count_)) {
                        RCLCPP_WARN(get_logger(),
                            "Left arm initial state contains NaN/Inf (attempt %d/%d), retrying...",
                            attempt + 1, max_read_attempts);
                        usleep(read_interval_ms * 1000);
                        continue;
                    }

                    if (is_all_zeros(left_state, left_joint_count_)) {
                        left_all_zeros_detected = true;
                        RCLCPP_WARN(get_logger(),
                            "Left arm initial positions are all zeros (attempt %d/%d), retrying...",
                            attempt + 1, max_read_attempts);
                        usleep(read_interval_ms * 1000);
                        continue;
                    }

                    left_success = true;
                    RCLCPP_INFO(get_logger(), "Left arm initial state read successfully (attempt %d/%d)",
                               attempt + 1, max_read_attempts);
                    break;

                } catch (const std::exception& e) {
                    RCLCPP_WARN(get_logger(),
                        "Failed to read left arm initial state (attempt %d/%d): %s",
                        attempt + 1, max_read_attempts, e.what());
                    usleep(read_interval_ms * 1000);
                }
            }

            if (!left_success) {
                RCLCPP_ERROR(get_logger(), "Failed to read valid left arm initial state after %d attempts",
                            max_read_attempts);
                if (left_all_zeros_detected) {
                    RCLCPP_ERROR(get_logger(), "Left arm hardware appears to be returning all zeros. "
                               "Please check: 1) Hardware power, 2) CAN connection, 3) Motor initialization.");
                }
                controllers_[0].reset();
                controllers_[1].reset();
                return hardware_interface::CallbackReturn::ERROR;
            }

            // 读取右臂初始状态（带验证和重试）
            bool right_success = false;
            bool right_all_zeros_detected = false;
            arx::JointState right_state(right_joint_count_);

            for (int attempt = 0; attempt < max_read_attempts; ++attempt) {
                try {
                    right_state = controllers_[ARM_RIGHT]->get_joint_state();

                    if (has_invalid_values(right_state, right_joint_count_)) {
                        RCLCPP_WARN(get_logger(),
                            "Right arm initial state contains NaN/Inf (attempt %d/%d), retrying...",
                            attempt + 1, max_read_attempts);
                        usleep(read_interval_ms * 1000);
                        continue;
                    }

                    if (is_all_zeros(right_state, right_joint_count_)) {
                        right_all_zeros_detected = true;
                        RCLCPP_WARN(get_logger(),
                            "Right arm initial positions are all zeros (attempt %d/%d), retrying...",
                            attempt + 1, max_read_attempts);
                        usleep(read_interval_ms * 1000);
                        continue;
                    }

                    right_success = true;
                    RCLCPP_INFO(get_logger(), "Right arm initial state read successfully (attempt %d/%d)",
                               attempt + 1, max_read_attempts);
                    break;

                } catch (const std::exception& e) {
                    RCLCPP_WARN(get_logger(),
                        "Failed to read right arm initial state (attempt %d/%d): %s",
                        attempt + 1, max_read_attempts, e.what());
                    usleep(read_interval_ms * 1000);
                }
            }

            if (!right_success) {
                RCLCPP_ERROR(get_logger(), "Failed to read valid right arm initial state after %d attempts",
                            max_read_attempts);
                if (right_all_zeros_detected) {
                    RCLCPP_ERROR(get_logger(), "Right arm hardware appears to be returning all zeros. "
                               "Please check: 1) Hardware power, 2) CAN connection, 3) Motor initialization.");
                }
                controllers_[0].reset();
                controllers_[1].reset();
                return hardware_interface::CallbackReturn::ERROR;
            }

            // 更新左臂状态
            for (size_t i = 0; i < left_joint_count_ && i < static_cast<size_t>(left_state.pos.size()); ++i) {
                position_states_[i] = left_state.pos[i];
                velocity_states_[i] = left_state.vel[i];
                effort_states_[i] = left_state.torque[i];
                position_commands_[i] = left_state.pos[i];
            }

            // 更新右臂状态
            for (size_t i = 0; i < right_joint_count_ && i < static_cast<size_t>(right_state.pos.size()); ++i) {
                const size_t dst_idx = left_joint_count_ + i;
                if (dst_idx < joint_count_) {
                    position_states_[dst_idx] = right_state.pos[i];
                    velocity_states_[dst_idx] = right_state.vel[i];
                    effort_states_[dst_idx] = right_state.torque[i];
                    position_commands_[dst_idx] = right_state.pos[i];
                }
            }

            // 初始化夹爪状态（双臂模式：左臂和右臂各一个夹爪）
            if (has_gripper_) {
                if (gripper_joint_names_.size() >= 1) {
                    gripper_position_states_[0] = left_state.gripper_pos;
                    gripper_position_commands_[0] = left_state.gripper_pos;
                }
                if (gripper_joint_names_.size() >= 2) {
                    gripper_position_states_[1] = right_state.gripper_pos;
                    gripper_position_commands_[1] = right_state.gripper_pos;
                }
            }

        } else {
            // 单臂模式
            const int arm_idx = arm_index_;
            const std::string& model = (arm_idx == ARM_LEFT) ? left_robot_model_ : right_robot_model_;
            const std::string& can_if = (arm_idx == ARM_LEFT) ? left_can_interface_ : right_can_interface_;

            RCLCPP_INFO(get_logger(),
                        "Activating %s arm: %s on %s",
                        arm_config_.c_str(), model.c_str(), can_if.c_str());

            controllers_[arm_idx] = std::make_shared<arx::Arx5JointController>(model, can_if);

            // 重置到 HOME 位置（确保机械臂处于已知安全位置）
            RCLCPP_INFO(get_logger(), "Resetting %s arm to home position...", arm_config_.c_str());
            controllers_[arm_idx]->reset_to_home();
            RCLCPP_INFO(get_logger(), "Arm reset to home position.");

            // 读取初始状态（带验证和重试）
            bool initial_read_success = false;
            bool all_zeros_detected = false;
            arx::JointState initial_state(joint_count_);

            for (int attempt = 0; attempt < max_read_attempts; ++attempt) {
                try {
                    initial_state = controllers_[arm_idx]->get_joint_state();

                    // 检查 NaN/Inf
                    if (has_invalid_values(initial_state, joint_count_)) {
                        RCLCPP_WARN(get_logger(),
                            "Initial joint state contains NaN/Inf (attempt %d/%d), retrying...",
                            attempt + 1, max_read_attempts);
                        usleep(read_interval_ms * 1000);
                        continue;
                    }

                    // 检查全零
                    if (is_all_zeros(initial_state, joint_count_)) {
                        all_zeros_detected = true;
                        RCLCPP_WARN(get_logger(),
                            "Initial joint positions are all zeros (attempt %d/%d), "
                            "this may indicate uninitialized hardware. Retrying...",
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
                if (all_zeros_detected) {
                    RCLCPP_ERROR(get_logger(), "Hardware appears to be returning all zeros. "
                               "Please check: 1) Hardware power, 2) CAN connection, 3) Motor initialization.");
                }
                controllers_[arm_idx].reset();
                return hardware_interface::CallbackReturn::ERROR;
            }

            // 更新关节状态
            for (size_t i = 0; i < joint_count_ && i < static_cast<size_t>(initial_state.pos.size()); ++i) {
                position_states_[i] = initial_state.pos[i];
                velocity_states_[i] = initial_state.vel[i];
                effort_states_[i] = initial_state.torque[i];
                position_commands_[i] = initial_state.pos[i];
            }

            // 初始化夹爪状态
            if (has_gripper_ && gripper_joint_names_.size() >= 1) {
                gripper_position_states_[0] = initial_state.gripper_pos;
                gripper_position_commands_[0] = initial_state.gripper_pos;
            }
        }

        // 预分配命令缓冲区（避免在控制循环中频繁分配内存）
        if (arm_index_ == ARM_DUAL) {
            left_cmd_buffer_.emplace(left_joint_count_);
            right_cmd_buffer_.emplace(right_joint_count_);
            RCLCPP_DEBUG(get_logger(), "Pre-allocated command buffers: left=%zu, right=%zu joints",
                        left_joint_count_, right_joint_count_);
        } else {
            single_cmd_buffer_.emplace(joint_count_);
            RCLCPP_DEBUG(get_logger(), "Pre-allocated command buffer: %zu joints", joint_count_);
        }

        hardware_connected_ = true;
        
        // 应用初始增益值
        if (arm_index_ == ARM_DUAL) {
            applyGains(ARM_LEFT, joint_k_gains_, joint_d_gains_, gripper_kp_, gripper_kd_);
            applyGains(ARM_RIGHT, joint_k_gains_, joint_d_gains_, gripper_kp_, gripper_kd_);
        } else {
            applyGains(arm_index_, joint_k_gains_, joint_d_gains_, gripper_kp_, gripper_kd_);
        }
        
        RCLCPP_INFO(get_logger(), "Successfully activated!");
        return hardware_interface::CallbackReturn::SUCCESS;

    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(),
                     "Failed to activate: %s", e.what());
        // 清理已创建的控制器
        hardware_connected_ = false;
        controllers_[0].reset();
        controllers_[1].reset();
        return hardware_interface::CallbackReturn::ERROR;
    }
}

/**
 * @brief 硬件停用回调
 * @param previous_state 之前的生命周期状态（未使用）
 * @return SUCCESS 表示停用成功
 *
 * 生命周期：active -> inactive 状态转换时调用
 *
 * 主要工作：
 * 1. 释放 ARX SDK 控制器实例
 * 2. 断开与硬件的连接
 *
 * 停用后硬件将停止接收命令，关节保持当前位置
 */
hardware_interface::CallbackReturn ArxX5Hardware::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {

    RCLCPP_INFO(get_logger(), "Deactivating...");

    // 标记硬件已断开连接
    hardware_connected_ = false;

    // 释放控制器
    if (arm_index_ == ARM_DUAL) {
        controllers_[ARM_LEFT].reset();
        controllers_[ARM_RIGHT].reset();
    } else {
        controllers_[arm_index_].reset();
    }

    // 清理命令缓冲区
    left_cmd_buffer_.reset();
    right_cmd_buffer_.reset();
    single_cmd_buffer_.reset();

    RCLCPP_INFO(get_logger(), "Successfully deactivated!");
    return hardware_interface::CallbackReturn::SUCCESS;
}

/**
 * @brief 硬件错误处理回调
 * @param previous_state 之前的生命周期状态（未使用）
 * @return SUCCESS 表示错误处理完成
 *
 * 生命周期：当硬件接口发生错误时由 ros2_control 调用
 *
 * 主要工作：
 * 1. 记录错误日志
 * 2. 标记硬件已断开连接
 * 3. 安全释放控制器资源
 */
hardware_interface::CallbackReturn ArxX5Hardware::on_error(
    const rclcpp_lifecycle::State& /*previous_state*/) {

    RCLCPP_ERROR(get_logger(), "Error in ArxX5 Hardware Interface");

    // 标记硬件已断开连接
    hardware_connected_ = false;

    // 安全释放控制器
    controllers_[0].reset();
    controllers_[1].reset();

    // 清理命令缓冲区
    left_cmd_buffer_.reset();
    right_cmd_buffer_.reset();
    single_cmd_buffer_.reset();

    return hardware_interface::CallbackReturn::SUCCESS;
}

/**
 * @brief 硬件关闭回调
 * @param previous_state 之前的生命周期状态（未使用）
 * @return SUCCESS 表示关闭成功
 *
 * 生命周期：节点关闭时由 ros2_control 调用
 *
 * 主要工作：
 * 1. 记录关闭日志
 * 2. 确保硬件已断开连接
 * 3. 释放所有资源
 */
hardware_interface::CallbackReturn ArxX5Hardware::on_shutdown(
    const rclcpp_lifecycle::State& /*previous_state*/) {

    RCLCPP_INFO(get_logger(), "Shutting down ArxX5 Hardware Interface...");

    // 确保硬件已断开
    if (hardware_connected_) {
        hardware_connected_ = false;
        controllers_[0].reset();
        controllers_[1].reset();
    }

    // 清理命令缓冲区
    left_cmd_buffer_.reset();
    right_cmd_buffer_.reset();
    single_cmd_buffer_.reset();

    RCLCPP_INFO(get_logger(), "Shutdown complete.");
    return hardware_interface::CallbackReturn::SUCCESS;
}

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
 * 数据流：ARX SDK -> position_states_/velocity_states_/effort_states_ -> 控制器
 */
hardware_interface::return_type ArxX5Hardware::read(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& /*period*/) {

    if (!hardware_connected_) {
        return hardware_interface::return_type::ERROR;
    }

    // 辅助 lambda：校验并更新状态值，无效值保留上一次的值
    auto validate_and_update = [this](double raw, double& state, const char* name, int idx) {
        if (std::isnan(raw) || std::isinf(raw)) {
            RCLCPP_WARN_THROTTLE(get_logger(), *node_->get_clock(), 1000,
                "%s[%d] is NaN/Inf, keeping previous value: %.3f", name, idx, state);
        } else {
            state = raw;
        }
    };

    try {
        if (arm_index_ == ARM_DUAL) {
            // 双臂模式：从两个控制器读取状态
            // 读取左臂状态
            arx::JointState left_state = controllers_[ARM_LEFT]->get_joint_state();
            for (size_t i = 0; i < left_joint_count_ && i < static_cast<size_t>(left_state.pos.size()); ++i) {
                validate_and_update(left_state.pos[i], position_states_[i], "Left joint position", i);
                validate_and_update(left_state.vel[i], velocity_states_[i], "Left joint velocity", i);
                validate_and_update(left_state.torque[i], effort_states_[i], "Left joint effort", i);
            }

            // 读取右臂状态
            arx::JointState right_state = controllers_[ARM_RIGHT]->get_joint_state();
            for (size_t i = 0; i < right_joint_count_ && i < static_cast<size_t>(right_state.pos.size()); ++i) {
                const size_t dst_idx = left_joint_count_ + i;
                if (dst_idx < joint_count_) {
                    validate_and_update(right_state.pos[i], position_states_[dst_idx], "Right joint position", i);
                    validate_and_update(right_state.vel[i], velocity_states_[dst_idx], "Right joint velocity", i);
                    validate_and_update(right_state.torque[i], effort_states_[dst_idx], "Right joint effort", i);
                }
            }

            // 更新夹爪状态（双臂模式：左臂和右臂各一个夹爪）
            if (has_gripper_) {
                if (gripper_joint_names_.size() >= 1) {
                    validate_and_update(left_state.gripper_pos, gripper_position_states_[0], "Left gripper position", 0);
                    validate_and_update(left_state.gripper_vel, gripper_velocity_states_[0], "Left gripper velocity", 0);
                    validate_and_update(left_state.gripper_torque, gripper_effort_states_[0], "Left gripper effort", 0);
                }
                if (gripper_joint_names_.size() >= 2) {
                    validate_and_update(right_state.gripper_pos, gripper_position_states_[1], "Right gripper position", 1);
                    validate_and_update(right_state.gripper_vel, gripper_velocity_states_[1], "Right gripper velocity", 1);
                    validate_and_update(right_state.gripper_torque, gripper_effort_states_[1], "Right gripper effort", 1);
                }
            }

        } else {
            // 单臂模式
            const int arm_idx = arm_index_;

            // 从SDK读取关节状态
            arx::JointState state = controllers_[arm_idx]->get_joint_state();

            // 更新关节状态（带 NaN/Inf 校验）
            for (size_t i = 0; i < joint_count_ && i < static_cast<size_t>(state.pos.size()); ++i) {
                validate_and_update(state.pos[i], position_states_[i], "Joint position", i);
                validate_and_update(state.vel[i], velocity_states_[i], "Joint velocity", i);
                validate_and_update(state.torque[i], effort_states_[i], "Joint effort", i);
            }

            // 更新夹爪状态
            if (has_gripper_ && gripper_joint_names_.size() >= 1) {
                validate_and_update(state.gripper_pos, gripper_position_states_[0], "Gripper position", 0);
                validate_and_update(state.gripper_vel, gripper_velocity_states_[0], "Gripper velocity", 0);
                validate_and_update(state.gripper_torque, gripper_effort_states_[0], "Gripper effort", 0);
            }
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
 * 数据流：控制器 -> position_commands_ -> ARX SDK -> 电机
 */
hardware_interface::return_type ArxX5Hardware::write(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& /*period*/) {

    if (!hardware_connected_) {
        return hardware_interface::return_type::ERROR;
    }

    try {
        if (arm_index_ == ARM_DUAL) {
            // 双臂模式：向两个控制器发送命令
            // 使用预分配的缓冲区（避免在控制循环中分配内存）
            if (!left_cmd_buffer_ || !right_cmd_buffer_) {
                RCLCPP_ERROR_THROTTLE(get_logger(), *node_->get_clock(), 1000,
                    "Command buffers not initialized");
                return hardware_interface::return_type::ERROR;
            }

            // 更新左臂命令缓冲区
            arx::JointState& left_cmd = *left_cmd_buffer_;
            for (size_t i = 0; i < left_joint_count_; ++i) {
                left_cmd.pos[i] = position_commands_[i];
            }
            if (has_gripper_ && gripper_joint_names_.size() >= 1) {
                left_cmd.gripper_pos = gripper_position_commands_[0];
            }
            controllers_[ARM_LEFT]->set_joint_cmd(left_cmd);

            // 更新右臂命令缓冲区
            arx::JointState& right_cmd = *right_cmd_buffer_;
            for (size_t i = 0; i < right_joint_count_; ++i) {
                const size_t src_idx = left_joint_count_ + i;
                if (src_idx < joint_count_) {
                    right_cmd.pos[i] = position_commands_[src_idx];
                }
            }
            if (has_gripper_ && gripper_joint_names_.size() >= 2) {
                right_cmd.gripper_pos = gripper_position_commands_[1];
            }
            controllers_[ARM_RIGHT]->set_joint_cmd(right_cmd);

        } else {
            // 单臂模式
            // 使用预分配的缓冲区（避免在控制循环中分配内存）
            if (!single_cmd_buffer_) {
                RCLCPP_ERROR_THROTTLE(get_logger(), *node_->get_clock(), 1000,
                    "Command buffer not initialized");
                return hardware_interface::return_type::ERROR;
            }

            const int arm_idx = arm_index_;

            // 更新命令缓冲区
            arx::JointState& cmd = *single_cmd_buffer_;
            for (size_t i = 0; i < joint_count_; ++i) {
                cmd.pos[i] = position_commands_[i];
            }

            // 设置夹爪命令
            if (has_gripper_ && gripper_joint_names_.size() >= 1) {
                cmd.gripper_pos = gripper_position_commands_[0];
            }

            // 发送命令到SDK
            controllers_[arm_idx]->set_joint_cmd(cmd);
        }

        return hardware_interface::return_type::OK;

    } catch (const std::exception& e) {
        RCLCPP_ERROR_THROTTLE(get_logger(), *node_->get_clock(), 1000,
                              "Write failed: %s", e.what());
        return hardware_interface::return_type::ERROR;
    }
}

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

    for (const auto & param : params) {
        // 检查硬件是否已连接（增益参数需要硬件连接）
        if (!hardware_connected_) {
            // 对于增益参数，需要硬件连接
            if (param.get_name() == "joint_k_gains" || param.get_name() == "joint_d_gains" ||
                param.get_name() == "gripper_kp" || param.get_name() == "gripper_kd") {
                result.successful = false;
                result.reason = "Hardware not connected. Cannot set gains.";
                return result;
            }
        }

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
            
            // 应用到硬件
            if (hardware_connected_) {
                if (arm_index_ == ARM_DUAL) {
                    applyGains(ARM_LEFT, joint_k_gains_, joint_d_gains_, gripper_kp_, gripper_kd_);
                    applyGains(ARM_RIGHT, joint_k_gains_, joint_d_gains_, gripper_kp_, gripper_kd_);
                } else {
                    applyGains(arm_index_, joint_k_gains_, joint_d_gains_, gripper_kp_, gripper_kd_);
                }
            }
        }
        else if (param.get_name() == "joint_d_gains") {
            std::vector<double> new_kd = param.as_double_array();
            // 当前只支持6关节机械臂
            const size_t expected_count = 6;
            
            if (new_kd.size() != expected_count) {
                result.successful = false;
                result.reason = "joint_d_gains must have exactly " + std::to_string(expected_count) + " values (for 6-joint robot)";
                return result;
            }
            
            joint_d_gains_ = new_kd;
            RCLCPP_INFO(get_logger(), "joint_d_gains updated via parameter");
            
            // 应用到硬件
            if (hardware_connected_) {
                if (arm_index_ == ARM_DUAL) {
                    applyGains(ARM_LEFT, joint_k_gains_, joint_d_gains_, gripper_kp_, gripper_kd_);
                    applyGains(ARM_RIGHT, joint_k_gains_, joint_d_gains_, gripper_kp_, gripper_kd_);
                } else {
                    applyGains(arm_index_, joint_k_gains_, joint_d_gains_, gripper_kp_, gripper_kd_);
                }
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
            
            // 应用到硬件
            if (hardware_connected_) {
                if (arm_index_ == ARM_DUAL) {
                    applyGains(ARM_LEFT, joint_k_gains_, joint_d_gains_, gripper_kp_, gripper_kd_);
                    applyGains(ARM_RIGHT, joint_k_gains_, joint_d_gains_, gripper_kp_, gripper_kd_);
                } else {
                    applyGains(arm_index_, joint_k_gains_, joint_d_gains_, gripper_kp_, gripper_kd_);
                }
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
            
            // 应用到硬件
            if (hardware_connected_) {
                if (arm_index_ == ARM_DUAL) {
                    applyGains(ARM_LEFT, joint_k_gains_, joint_d_gains_, gripper_kp_, gripper_kd_);
                    applyGains(ARM_RIGHT, joint_k_gains_, joint_d_gains_, gripper_kp_, gripper_kd_);
                } else {
                    applyGains(arm_index_, joint_k_gains_, joint_d_gains_, gripper_kp_, gripper_kd_);
                }
            }
        }
    }

    return result;
}

/**
 * @brief 应用增益到硬件
 * @param arm_index 臂索引 (ARM_LEFT, ARM_RIGHT)
 * @param kp 位置增益数组
 * @param kd 阻尼增益数组
 * @param gripper_kp 夹爪位置增益
 * @param gripper_kd 夹爪阻尼增益
 *
 * 将增益参数应用到指定的机械臂控制器
 */
void ArxX5Hardware::applyGains(int arm_index, const std::vector<double>& kp, 
                                const std::vector<double>& kd, 
                                double gripper_kp, double gripper_kd)
{
    if (!controllers_[arm_index]) {
        RCLCPP_WARN(get_logger(), "Controller[%d] not initialized, cannot apply gains", arm_index);
        return;
    }

    try {
        // 当前只支持6关节机械臂
        const size_t joint_count = 6;
        
        // 验证输入数组长度
        if (kp.size() < joint_count || kd.size() < joint_count) {
            RCLCPP_ERROR(get_logger(), 
                        "Gain arrays too short: kp.size()=%zu, kd.size()=%zu, expected %zu",
                        kp.size(), kd.size(), joint_count);
            return;
        }
        
        // 创建 Gain 结构（6关节）
        arx::Gain gain(static_cast<int>(joint_count));
        
        // 设置关节增益（6个关节）
        for (size_t i = 0; i < joint_count; ++i) {
            gain.kp[i] = kp[i];
            gain.kd[i] = kd[i];
        }
        
        // 设置夹爪增益
        gain.gripper_kp = static_cast<float>(gripper_kp);
        gain.gripper_kd = static_cast<float>(gripper_kd);
        
        // 应用到控制器
        controllers_[arm_index]->set_gain(gain);
        
        const char* arm_name = (arm_index == ARM_LEFT) ? "Left" : 
                              (arm_index == ARM_RIGHT) ? "Right" : "Unknown";
        RCLCPP_INFO(get_logger(), "%s arm gains applied: kp=[%.1f, %.1f, ...], kd=[%.1f, %.1f, ...], gripper_kp=%.2f, gripper_kd=%.2f",
                    arm_name, 
                    kp.empty() ? 0.0 : kp[0], kp.size() > 1 ? kp[1] : 0.0,
                    kd.empty() ? 0.0 : kd[0], kd.size() > 1 ? kd[1] : 0.0,
                    gripper_kp, gripper_kd);
                    
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Failed to apply gains to arm[%d]: %s", arm_index, e.what());
    }
}

}  // namespace arx_x5_ros2_control

/**
 * @brief 注册硬件接口插件
 *
 * 将 ArxX5Hardware 类注册为 ros2_control 的 SystemInterface 插件。
 * ros2_control 通过 pluginlib 动态加载此硬件接口。
 *
 * 配合 arx_x5_ros2_control.xml 插件描述文件使用。
 */
PLUGINLIB_EXPORT_CLASS(arx_x5_ros2_control::ArxX5Hardware, hardware_interface::SystemInterface)
