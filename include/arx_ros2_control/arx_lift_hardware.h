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
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "arx_lift_src/lift_head_control_loop.h"

#include <Eigen/Dense>

#include <atomic>
#include <cmath>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace arx_ros2_control
{

/**
 * @brief Lift2S 升降柱 ros2_control 硬件接口。
 *
 * ``lift_motor_mode``（运行时可改 ``arx_lift.motor_mode``）：
 * - ``soft_p`` / ``position`` — 仅跟踪 position（直跟，无 HI 斜坡）；
 *   Type3 位置环 + **常值**重力 ``τ_g``（不加库仑摩擦）；不走 SDK ``loop()`` 混发底盘
 * - ``hybrid``（默认）— ``sendLiftHybrid``；跟踪 position+velocity；
 *   kp/kd = ``arx_lift.hybrid_kp/kd``；
 *   ``τ_ff = gravity - coulomb * sign(v_cmd)``（忽略上层 effort）
 *
 * 底盘（可选，URDF ``enable_chassis_cmd_vel``）：
 * - 订阅 ``chassis_cmd_vel_topic``（默认 ``/cmd_vel``）→ ``setChassisCmd``
 * - Twist ``vx/vy/wz`` 原样（对齐 body_communicator；勿套 PosCmd ``-chy``）
 * - 两种模式：升降与底盘分开发；底盘 ``sendChassisOnly``（0x701/0x703）
 * - 运行 mode=1；超时 / 退出 / soft-stop → mode=2 停车
 *
 * 底盘反馈 / 里程计（可选；官方无现成 odom，仅发 IMU+轮速）：
 * - ``enable_chassis_feedback``：对齐官方 lift_controller —
 *   ``getOrientation/AngularVel/Accel`` → ``/arx_imu``；
 *   ``getWheelVel`` → ``/arx_lift/wheel_vel``
 * - ``enable_chassis_odom``：用 **轮速逆解（Isaac Holonomic 正向之逆）+ IMU yaw**
 *   积分发 ``/arx_lift/odom``（**不用** ``cmd_vel`` 积分）
 * - ``enable_chassis_odom_tf``：持续广播 ``world→base_link``（默认关；关时 WBC 发 identity 占位 TF）
 * - ``enable_chassis_odom_debug``：用最近 ``cmd_vel`` 正解发期望轮速（默认关）
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
  /** @param chassis_active 本周期底盘是否 mode=1。 */
  void flushChassisForControlCycle(bool chassis_active);
  void sendHybridHoldOrTrack(
    double q_target_sdk, double dt_s, bool chassis_active);
  /** Soft-P：直跟 SDK 高度目标；忽略上层 vel/effort。 */
  void sendSoftPHoldOrTrack(double q_sdk, bool chassis_active);
  void enterSafeExit(bool allow_return_home);
  void interpolateLiftToShutdownHeight();
  void softStopLift();
  void setupChassisCmdVelSubscription();
  void teardownChassisCmdVelSubscription();
  /** @return true 若底盘为运行 mode=1（有有效 cmd_vel）。 */
  bool applyChassisCmd(bool force_park);
  /** 只发 0x701/0x703（不发升降）。 */
  void flushChassisCanOnly();
  /** 停车：最多刷一次 mode=2 底盘帧。 */
  void flushChassisParkOnce();

  /** 创建 IMU / 轮速 / odom / TF 发布者（activate）。 */
  void setupChassisFeedbackPublishers();
  void teardownChassisFeedbackPublishers();
  /**
   * 读 SDK 底盘反馈；发布 IMU/轮速；可选由轮速+IMU 积分解算里程计。
   * SDK 读失败时仍发布上次位姿的 odom/TF，避免 world→base_link 断桥。
   * @param dt_s 本周期时长。
   */
  void updateChassisFeedbackAndOdom(double dt_s);
  /**
   * 发布当前 odom 位姿到 ``/arx_lift/odom`` 与（可选）``world→base_link`` TF。
   * @param twist_ok 本拍是否有有效 body twist（写入 odom.twist）。
   */
  void publishChassisOdomAndTf(
    const rclcpp::Time & stamp, double x, double y, double yaw, double vx_b,
    double vy_b, double wz_b, bool twist_ok);
  /** 按 Lift2S / Isaac Holonomic（mecanum=90°）几何预计算 3×3 雅可比。 */
  bool buildChassisKinematics();
  /**
   * 逆解：ω[rad/s] → body (vx,vy,wz)。对齐 Isaac HolonomicController 正向的逆。
   * @return false 若几何未就绪或输入非有限。
   */
  bool bodyTwistFromWheelVel(
    const double wheel_vel[4], double & vx, double & vy, double & wz) const;
  /** 正解：body twist → 期望轮速 ω[rad/s]（与 Isaac cmd_vel→wheel 同模型）。 */
  bool wheelVelFromBodyTwist(
    double vx, double vy, double wz, double wheel_out[3]) const;

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

  /** URDF：是否订阅 cmd_vel 并映射到底盘（Lift2S xacro 默认开）。 */
  bool enable_chassis_cmd_vel_{false};
  std::string chassis_cmd_vel_topic_{"/cmd_vel"};
  double chassis_cmd_timeout_sec_{0.3};
  /** setChassisCmd 量化上限；LIFTS 的 SDK 未初始化，须 HI 写入。 */
  double chassis_max_vel_x_{2.0};
  double chassis_max_vel_y_{2.0};
  double chassis_max_vel_z_{4.0};
  std::atomic<double> chassis_vx_{0.0};
  std::atomic<double> chassis_vy_{0.0};
  std::atomic<double> chassis_wz_{0.0};
  /** steady_clock ns；0 表示尚未收到指令（按超时停车）。 */
  std::atomic<int64_t> chassis_cmd_stamp_ns_{0};
  /** Hybrid 下 mode=2 停车帧是否已刷过。 */
  bool chassis_park_flushed_{false};
  /** 最近下发的底盘 mode：1=运行，2=停车（官方：运动控制启动后才有轮速反馈）。 */
  std::atomic<int> chassis_mode_cmd_{2};
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr chassis_cmd_sub_;

  /** 对齐官方 lift_controller：发 /arx_imu + 轮速；可选轮速+IMU 里程计。 */
  bool enable_chassis_feedback_{true};
  bool enable_chassis_odom_{false};
  bool enable_chassis_odom_tf_{false};
  bool enable_chassis_odom_debug_{false};
  std::string chassis_odom_parent_frame_{"world"};
  std::string chassis_odom_child_frame_{"base_link"};
  std::string chassis_imu_topic_{"/arx_imu"};
  std::string chassis_wheel_vel_topic_{"/arx_lift/wheel_vel"};
  /** 对齐官方 /body_information.temp_float_data：[0]=腰占位，[1..4]=轮速。 */
  std::string chassis_body_temp_float_topic_{"/arx_lift/body_temp_float_data"};
  std::string chassis_wheel_vel_expected_topic_{"/arx_lift/wheel_vel_expected"};
  std::string chassis_odom_topic_{"/arx_lift/odom"};

  /**
   * Lift2S 三全向轮几何（chassis.xacro / FaSim ARX_LIFT2S.usda）：
   * Omnia150 r=0.075；mecanumAngles=90° → 驱动方向 = R_z(θ)·Ŷ。
   * 与 Isaac HolonomicController 正向模型一致，此处做逆解。
   */
  double chassis_wheel_radius_m_{0.075};
  double chassis_mecanum_angle_deg_{90.0};
  double wheel_x_[3]{0.15361, 0.15361, -0.35293};
  double wheel_y_[3]{0.29246, -0.29245, 0.0};
  double wheel_yaw_[3]{1.0472, -1.0472, 3.1415};
  double wheel_vel_sign_[3]{1.0, 1.0, 1.0};
  bool chassis_kinematics_ok_{false};
  /** r*ω = J_rw_from_body_ * [vx,vy,wz]；body = J_body_from_rw_ * (r*ω)。 */
  Eigen::Matrix3d J_rw_from_body_{Eigen::Matrix3d::Zero()};
  Eigen::Matrix3d J_body_from_rw_{Eigen::Matrix3d::Zero()};

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr wheel_vel_pub_;
  /** 官方文档同布局：temp_float_data[1..3]=LIFT 三轮（[4] 常 0）。 */
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
    body_temp_float_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
    wheel_vel_expected_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  /** 航迹：IMU yaw 相对零位 + 轮速逆解得到的 body vx/vy 积分。 */
  std::mutex odom_mutex_;
  bool odom_yaw_initialized_{false};
  double odom_yaw0_{0.0};
  double odom_x_{0.0};
  double odom_y_{0.0};
  double odom_yaw_{0.0};

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
