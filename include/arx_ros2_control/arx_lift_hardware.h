// Copyright 2026 FiveAges Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include "arx_lift_src/lift_head_control_loop.h"

#include <atomic>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace arx_ros2_control
{

/**
 * @brief Lift2S 升降柱 ros2_control 硬件接口。
 *
 * ``lift_motor_mode``（运行时可改 ``arx_lift.motor_mode``）：
 * - ``soft_p`` / ``position`` — Soft-P ``setHeight`` / ``loop()``，仅跟踪 position（功能保留）
 * - ``hybrid``（默认）— ``sendLiftHybrid``；跟踪 position+velocity；
 *   kp/kd = ``arx_lift.hybrid_kp/kd``；
 *   ``τ_ff = gravity - coulomb * sign(v_cmd)``（忽略上层 effort）
 *
 * 底盘（可选，URDF ``enable_chassis_cmd_vel``）：
 * - 订阅 ``chassis_cmd_vel_topic``（默认 ``/cmd_vel``）→ ``setChassisCmd``
 * - 运行 mode=1；超时 / 退出 / soft-stop → mode=2 停车
 */
class ArxLiftHardware : public hardware_interface::SystemInterface
{
public:
  enum class MotorMode : int
  {
    SoftP = 0,
    Hybrid = 1,
  };

  RCLCPP_SHARED_PTR_DEFINITIONS(ArxLiftHardware)

  ArxLiftHardware() = default;
  ~ArxLiftHardware() override;

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_error(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface::ConstSharedPtr>
  on_export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface::SharedPtr>
  on_export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  void stop_loop_thread();
  static bool parseMotorMode(const std::string & raw, MotorMode & out);
  static const char * motorModeName(MotorMode mode);
  void setupDynamicParameters(
    const std::string & initial_mode, double soft_p_kp, double hybrid_kp,
    double hybrid_kd, double gravity_comp, double coulomb_friction,
    double friction_vel_eps_mps, bool status_debug);
  double computeHybridFeedforward(double v_cmd_sdk) const;
  void sendHybridHoldOrTrack(double q_target_sdk, double dt_s);
  void enterSafeExit(bool allow_return_home);
  void interpolateLiftToShutdownHeight();
  void softStopLift();
  void setupChassisCmdVelSubscription();
  void teardownChassisCmdVelSubscription();
  void applyChassisCmd(bool force_park);
  void flushChassisWithHybridLift(
    double k_p, double k_d, double p_motor, double v_motor, double t_ff);

  double rosToSdk(double ros_m) const
  {
    const double m = std::clamp(ros_m, 0.0, height_span_m_);
    const double sdk = m * height_rad_per_meter_;
    return std::clamp(sdk, 0.0, sdk_max_rad_);
  }
  double sdkToRos(double sdk_rad) const
  {
    if (height_rad_per_meter_ <= 0.0) {
      return 0.0;
    }
    return sdk_rad / height_rad_per_meter_;
  }

  std::string lift_joint_name_;
  double lift_position_{0.0};
  double lift_velocity_{0.0};
  double lift_effort_{0.0};
  double lift_position_command_{0.0};
  double lift_velocity_command_{0.0};
  double lift_effort_command_{0.0};

  std::string can_name_{"can5"};
  int robot_type_{0};
  std::atomic<double> gravity_compensation_torque_{-1.01};
  std::atomic<double> coulomb_friction_torque_{0.32};
  std::atomic<double> friction_vel_eps_mps_{0.01};
  double lift_max_vel_{0.20};
  double lift_max_torque_{15.0};
  double cmd_ramp_vel_mps_{0.12};

  std::atomic<double> soft_p_kp_{50.0};
  std::atomic<double> hybrid_kp_{5.0};
  std::atomic<double> hybrid_kd_{2.0};

  std::string motor_mode_param_{"hybrid"};
  std::atomic<int> motor_mode_{static_cast<int>(MotorMode::Hybrid)};
  int last_applied_mode_{-1};

  double height_rad_per_meter_{41.54};
  double height_span_m_{0.48};
  double sdk_max_rad_{20.0};

  double ramp_q_sdk_{0.0};
  bool ramp_initialized_{false};

  std::shared_ptr<arx::LiftHeadControlLoop> lift_;

  std::thread loop_thread_;
  std::atomic<bool> loop_running_{false};
  std::atomic<bool> command_enabled_{false};
  std::atomic<bool> soft_stop_active_{false};
  std::atomic<bool> status_debug_{false};
  std::atomic<double> last_written_height_{0.0};
  std::atomic<double> last_written_vel_{0.0};

  bool shutdown_return_home_{false};
  double shutdown_height_m_{0.0};
  double shutdown_home_velocity_{0.10};
  double shutdown_home_timeout_sec_{2.0};
  std::atomic<bool> safe_exit_done_{false};

  /** URDF：是否订阅 cmd_vel 并映射到底盘（默认关）。 */
  bool enable_chassis_cmd_vel_{false};
  std::string chassis_cmd_vel_topic_{"/cmd_vel"};
  double chassis_cmd_timeout_sec_{0.3};
  std::atomic<double> chassis_vx_{0.0};
  std::atomic<double> chassis_vy_{0.0};
  std::atomic<double> chassis_wz_{0.0};
  /** steady_clock ns；0 表示尚未收到指令（按超时停车）。 */
  std::atomic<int64_t> chassis_cmd_stamp_ns_{0};
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr chassis_cmd_sub_;

  std::atomic<double> motor_position_{0.0};
  std::atomic<double> motor_velocity_{0.0};
  std::atomic<double> motor_torque_{0.0};
  std::atomic<double> motor_current_{0.0};
  std::atomic<int> motor_online_{0};
  std::atomic<int> motor_error_{0};
  std::atomic<double> sdk_get_height_{0.0};

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr motor_pub_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;
};

}  // namespace arx_ros2_control
