#pragma once

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include "app/joint_controller.h"

#include <cmath>
#include <memory>
#include <string>
#include <vector>
#include <optional>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

namespace arx_ros2_control {

class ArxX5Hardware : public hardware_interface::SystemInterface {
public:
    RCLCPP_SHARED_PTR_DEFINITIONS(ArxX5Hardware)

    // 初始化
    hardware_interface::CallbackReturn on_init(
        const hardware_interface::HardwareComponentInterfaceParams& params) override;

    // 导出状态接口 (position, velocity, effort)
    std::vector<hardware_interface::StateInterface::ConstSharedPtr> on_export_state_interfaces() override;

    // 导出命令接口 (position)
    std::vector<hardware_interface::CommandInterface::SharedPtr> on_export_command_interfaces() override;

    // 配置 (解析参数、验证配置，但不连接硬件)
    hardware_interface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State& previous_state) override;

    // 清理 (重置到未配置状态)
    hardware_interface::CallbackReturn on_cleanup(
        const rclcpp_lifecycle::State& previous_state) override;

    // 激活 (连接机械臂)
    hardware_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State& previous_state) override;

    // 停用 (停控不断连，read/write 不再访问硬件)
    hardware_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State& previous_state) override;

    // 关闭 (最终清理)
    hardware_interface::CallbackReturn on_shutdown(
        const rclcpp_lifecycle::State& previous_state) override;

    // 错误处理 (硬件错误时调用)
    hardware_interface::CallbackReturn on_error(
        const rclcpp_lifecycle::State& previous_state) override;

    // 读取状态 (从SDK获取关节状态)
    hardware_interface::return_type read(
        const rclcpp::Time& time,
        const rclcpp::Duration& period) override;

    // 写入命令 (发送关节命令到SDK)
    hardware_interface::return_type write(
        const rclcpp::Time& time,
        const rclcpp::Duration& period) override;

private:
    // ROS2 节点和日志器
    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Logger get_logger() const 
    { 
        return logger_.value();
    }
    std::optional<rclcpp::Logger> logger_;

    // ARX5 SDK 控制器（单臂）
    std::shared_ptr<arx::Arx5JointController> controller_;

    // 硬件连接状态标志（configure 建连为 true，cleanup/error/shutdown 才置 false）
    bool hardware_connected_ = false;

    // 控制使能标志（activate 为 true，deactivate 为 false）；inactive 时 read/write 均 noop
    bool control_active_ = false;

    // 参数回调句柄
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

    // 增益参数缓存（用于动态调整）
    std::vector<double> joint_k_gains_;   // 关节位置增益
    std::vector<double> joint_d_gains_;   // 关节阻尼增益
    double gripper_kp_ = 5.0;             // 夹爪位置增益
    double gripper_kd_ = 0.2;            // 夹爪阻尼增益

    // 配置参数（仅单臂：机械臂类型 + CAN 口）
    std::string robot_model_;    // 机器人型号 (X5, L5 等)
    std::string can_interface_;  // CAN 接口名 (can0, can1 等)

    size_t joint_count_;  // 关节数量

    // 关节名称
    std::vector<std::string> joint_names_;

    // 状态数据 (从SDK读取)
    std::vector<double> position_states_;
    std::vector<double> velocity_states_;
    std::vector<double> effort_states_;

    // 命令数据 (发送到SDK)
    std::vector<double> position_commands_;

    // 夹爪
    bool has_gripper_;
    std::vector<std::string> gripper_joint_names_;  // 夹爪关节名称列表（支持多个夹爪关节）
    std::vector<double> gripper_position_states_;    // 夹爪位置状态（每个夹爪关节独立的值）
    std::vector<double> gripper_velocity_states_;   // 夹爪速度状态
    std::vector<double> gripper_effort_states_;      // 夹爪力矩状态
    std::vector<double> gripper_position_commands_;  // 夹爪位置命令

    // 预分配的命令缓冲区（避免在控制循环中频繁分配内存）
    std::optional<arx::JointState> cmd_buffer_;

    // 参数管理辅助函数
    template<typename T>
    T get_node_param(const std::string& name, const T& default_val)
    {
        if (!node_->has_parameter(name)) {
            node_->declare_parameter<T>(name, default_val);
        }
        return node_->get_parameter(name).get_value<T>();
    }

    // 声明节点参数
    void declare_node_parameters();

    // 参数回调函数
    rcl_interfaces::msg::SetParametersResult paramCallback(const std::vector<rclcpp::Parameter> & params);

    // 应用增益到硬件
    void applyGains(const std::vector<double>& kp, const std::vector<double>& kd,
                    double gripper_kp, double gripper_kd);
};

}  // namespace arx_ros2_control
