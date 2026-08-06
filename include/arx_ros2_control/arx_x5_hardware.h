#pragma once

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include "app/joint_controller.h"

#include <atomic>
#include <cmath>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

namespace arx_ros2_control {

class ArxX5Hardware : public hardware_interface::SystemInterface {
public:
    RCLCPP_SHARED_PTR_DEFINITIONS(ArxX5Hardware)

    hardware_interface::CallbackReturn on_init(
        const hardware_interface::HardwareComponentInterfaceParams& params) override;

    std::vector<hardware_interface::StateInterface::ConstSharedPtr> on_export_state_interfaces() override;

    std::vector<hardware_interface::CommandInterface::SharedPtr> on_export_command_interfaces() override;

    hardware_interface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State& previous_state) override;

    hardware_interface::CallbackReturn on_cleanup(
        const rclcpp_lifecycle::State& previous_state) override;

    hardware_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State& previous_state) override;

    hardware_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State& previous_state) override;

    hardware_interface::CallbackReturn on_shutdown(
        const rclcpp_lifecycle::State& previous_state) override;

    hardware_interface::CallbackReturn on_error(
        const rclcpp_lifecycle::State& previous_state) override;

    hardware_interface::return_type read(
        const rclcpp::Time& time,
        const rclcpp::Duration& period) override;

    hardware_interface::return_type write(
        const rclcpp::Time& time,
        const rclcpp::Duration& period) override;

private:
    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Logger get_logger() const
    {
        return logger_.value();
    }
    std::optional<rclcpp::Logger> logger_;

    std::shared_ptr<arx::Arx5JointController> controller_;

    bool hardware_connected_ = false;
    bool control_active_ = false;

    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

    std::vector<double> joint_k_gains_;
    std::vector<double> joint_d_gains_;
    mutable std::mutex gains_mutex_;
    double gripper_kp_ = 5.0;
    double gripper_kd_ = 0.2;

    std::vector<double> last_applied_kp_;
    std::vector<double> last_applied_kd_;
    double last_applied_gripper_kp_ = -1.0;
    double last_applied_gripper_kd_ = -1.0;

    std::string robot_model_;    // Fixed to "X5" (not exposed as ROS param)
    std::string can_interface_;

    size_t joint_count_;

    std::vector<std::string> joint_names_;

    std::vector<double> position_states_;
    std::vector<double> velocity_states_;
    std::vector<double> effort_states_;

    std::vector<double> position_commands_;
    std::vector<double> velocity_commands_;
    std::vector<double> effort_commands_;
    std::vector<double> kp_commands_;
    std::vector<double> kd_commands_;

    bool has_gripper_;
    std::vector<std::string> gripper_joint_names_;
    std::vector<double> gripper_position_states_;
    std::vector<double> gripper_velocity_states_;
    std::vector<double> gripper_effort_states_;
    std::vector<double> gripper_position_commands_;

    std::optional<arx::JointState> cmd_buffer_;

    // Ctrl+C / deactivate: optional interpolate to shutdown_home then damping
    // (URDF: shutdown_return_home / shutdown_home / velocity / timeout).
    bool shutdown_return_home_{false};
    std::vector<double> shutdown_home_;
    double shutdown_home_velocity_{0.3};
    double shutdown_home_timeout_sec_{2.0};
    std::atomic<bool> safe_exit_done_{false};

    template<typename T>
    T get_node_param(const std::string& name, const T& default_val)
    {
        if (!node_->has_parameter(name)) {
            node_->declare_parameter<T>(name, default_val);
        }
        return node_->get_parameter(name).get_value<T>();
    }

    void declare_node_parameters();

    void enterSafeExit(bool allow_return_home);
    void moveToShutdownHomeThenDamping();
    void enterDampingOnly();

    rcl_interfaces::msg::SetParametersResult paramCallback(const std::vector<rclcpp::Parameter> & params);

    void applyGains(const std::vector<double>& kp, const std::vector<double>& kd,
                    double gripper_kp, double gripper_kd, bool force = false);
};

}  // namespace arx_ros2_control
