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

/**
 * @file arx_lift_hardware.cpp
 * @brief Lift2S 升降柱 SystemInterface 实现。
 *
 * 控制路径概要：
 * - 后台线程 ~400 Hz：soft_p 直跟 position；hybrid 直跟 position+velocity
 * - soft_p（position）：只用 position；Type3 位置环 + 常值 τ_g（无摩擦项）；忽略 vel/effort
 * - hybrid：直接跟 position + velocity；HI τ_ff（重力 + 库仑摩擦·(-sign(v_cmd))）；
 *   忽略控制器 effort（防 OCS2 RNEA 双重前馈）；增益来自 arx_lift.* 热调
 * - 底盘（两种模式相同）：vx/vy/wz → setChassisCmd + sendChassisOnly；与升降分开发
 * - SDK loop() 仅用于校准期（lift 未 command_enabled）
 * - 可选反馈：SDK IMU/轮速；可选轮速逆解+IMU yaw → odom/TF
 */

#include "arx_ros2_control/arx_lift_hardware.h"

#include <chrono>
#include <cmath>
#include <cctype>
#include <pluginlib/class_list_macros.hpp>
#include <sstream>
#include <stdexcept>
#include <thread>

#include <Eigen/Dense>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>

namespace arx_ros2_control
{
namespace
{
/** @brief 热调参数名：电机模式 soft_p | hybrid。 */
constexpr const char * kMotorModeParam = "arx_lift.motor_mode";
/** @brief 热调参数名：Soft-P 位置增益。 */
constexpr const char * kSoftPKpParam = "arx_lift.soft_p_kp";
/** @brief 热调参数名：Hybrid MIT kp。 */
constexpr const char * kHybridKpParam = "arx_lift.hybrid_kp";
/** @brief 热调参数名：Hybrid MIT kd。 */
constexpr const char * kHybridKdParam = "arx_lift.hybrid_kd";
/** @brief 热调参数名：HI 重力前馈（Hybrid τ_ff 重力项；Soft-P 仅用该常值，不加摩擦）。 */
constexpr const char * kGravityCompParam = "arx_lift.gravity_compensation_torque";
/** @brief 热调参数名：Hybrid 库仑摩擦幅值（≥0；τ_f = -μ·sign(v_cmd)）。 */
constexpr const char * kCoulombFrictionParam = "arx_lift.coulomb_friction_torque";
/** @brief 热调参数名：摩擦速度死区 [m/s]（|v_cmd| 以下不加库仑项）。 */
constexpr const char * kFrictionVelEpsParam = "arx_lift.friction_vel_eps_mps";
/** @brief Hybrid Type3 打包 kd 上限（见 libarx_lift_src.so）。 */
constexpr double kHybridKdMax = 5.0;

/** @brief 读取 URDF/xacro hardware_parameters 字符串，缺省返回 default_value。 */
std::string get_hw_param(
  const hardware_interface::HardwareInfo & info, const std::string & key,
  const std::string & default_value = "")
{
  const auto it = info.hardware_parameters.find(key);
  if (it == info.hardware_parameters.end()) {
    return default_value;
  }
  return it->second;
}

/** @brief 解析布尔型 hardware 参数。 */
bool parse_bool_param(
  const hardware_interface::HardwareInfo & info, const std::string & key,
  bool default_value, bool & out)
{
  const std::string raw = get_hw_param(info, key, "");
  if (raw.empty()) {
    out = default_value;
    return true;
  }
  std::string s = raw;
  for (char & c : s) {
    c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  }
  out = (s == "true" || s == "1" || s == "yes" || s == "on");
  return true;
}

/** @brief 解析浮点型 hardware 参数；非法字符串返回 false。 */
bool parse_double_param(
  const hardware_interface::HardwareInfo & info, const std::string & key,
  double default_value, double & out, const rclcpp::Logger & logger)
{
  const std::string raw = get_hw_param(info, key, "");
  if (raw.empty()) {
    out = default_value;
    return true;
  }
  try {
    out = std::stod(raw);
    return true;
  } catch (const std::exception &) {
    RCLCPP_ERROR(
      logger, "Invalid %s parameter: '%s'", key.c_str(), raw.c_str());
    return false;
  }
}

/** @brief 解析 3 个 double（空格/逗号分隔）；空串则保留 out 默认值。 */
bool parse_triple_param(
  const hardware_interface::HardwareInfo & info, const std::string & key,
  double out[3], const rclcpp::Logger & logger)
{
  const std::string raw = get_hw_param(info, key, "");
  if (raw.empty()) {
    return true;
  }
  std::string normalized = raw;
  for (char & c : normalized) {
    if (c == ',' || c == ';' || c == '[' || c == ']') {
      c = ' ';
    }
  }
  std::istringstream iss(normalized);
  double a = 0.0;
  double b = 0.0;
  double c = 0.0;
  if (!(iss >> a >> b >> c)) {
    RCLCPP_ERROR(
      logger, "Invalid %s parameter (need 3 numbers): '%s'", key.c_str(),
      raw.c_str());
    return false;
  }
  out[0] = a;
  out[1] = b;
  out[2] = c;
  return true;
}

}  // namespace

/** @brief 将 "soft_p"/"hybrid"（及别名）解析为 MotorMode。 */
bool ArxLiftHardware::parseMotorMode(const std::string & raw, MotorMode & out)
{
  std::string s = raw;
  for (char & c : s) {
    c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  }
  if (s == "soft_p" || s == "softp" || s == "soft-p" || s == "position") {
    out = MotorMode::SoftP;
    return true;
  }
  if (s == "hybrid") {
    out = MotorMode::Hybrid;
    return true;
  }
  return false;
}

/** @brief MotorMode → 参数字符串。 */
const char * ArxLiftHardware::motorModeName(MotorMode mode)
{
  return mode == MotorMode::SoftP ? "soft_p" : "hybrid";
}

/**
 * @brief 注册升降热调参数（模式 / soft_p_kp / hybrid_kp|kd / τ_ff / 摩擦死区 / status_debug）。
 * @note 挂在 controller_manager 节点上，可用 ros2 param set 在线修改（同 hybrid_kp/kd）。
 */
void ArxLiftHardware::setupDynamicParameters(
  const std::string & initial_mode, double soft_p_kp, double hybrid_kp,
  double hybrid_kd, double gravity_comp, double coulomb_friction,
  double friction_vel_eps_mps, bool status_debug)
{
  auto node = get_node();
  if (!node) {
    RCLCPP_WARN(
      get_logger(),
      "No ROS node; arx_lift.* params will not be dynamically settable");
    return;
  }

  auto declare_or_set_string =
    [&](const char * name, const std::string & value) {
      if (!node->has_parameter(name)) {
        node->declare_parameter<std::string>(name, value);
      } else {
        node->set_parameter(rclcpp::Parameter(name, value));
      }
    };
  auto declare_or_set_double = [&](const char * name, double value) {
    if (!node->has_parameter(name)) {
      node->declare_parameter<double>(name, value);
    } else {
      node->set_parameter(rclcpp::Parameter(name, value));
    }
  };

  auto declare_or_set_bool = [&](const char * name, bool value) {
    if (!node->has_parameter(name)) {
      node->declare_parameter<bool>(name, value);
    } else {
      node->set_parameter(rclcpp::Parameter(name, value));
    }
  };

  declare_or_set_string(kMotorModeParam, initial_mode);
  declare_or_set_double(kSoftPKpParam, soft_p_kp);
  declare_or_set_double(kHybridKpParam, hybrid_kp);
  declare_or_set_double(kHybridKdParam, hybrid_kd);
  declare_or_set_double(kGravityCompParam, gravity_comp);
  declare_or_set_double(kCoulombFrictionParam, coulomb_friction);
  declare_or_set_double(kFrictionVelEpsParam, friction_vel_eps_mps);
  declare_or_set_bool("status_debug", status_debug);
  status_debug_.store(status_debug);

  auto parse_nonneg = [](const rclcpp::Parameter & p, double & out,
                         std::string & reason, const char * label) -> bool {
    if (p.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE &&
      p.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER)
    {
      reason = std::string(label) + " must be a number";
      return false;
    }
    out = p.as_double();
    if (!(out >= 0.0) || !std::isfinite(out)) {
      reason = std::string(label) + " must be finite and >= 0";
      return false;
    }
    return true;
  };

  auto parse_finite = [](const rclcpp::Parameter & p, double & out,
                         std::string & reason, const char * label) -> bool {
    if (p.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE &&
      p.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER)
    {
      reason = std::string(label) + " must be a number";
      return false;
    }
    out = p.as_double();
    if (!std::isfinite(out)) {
      reason = std::string(label) + " must be finite";
      return false;
    }
    return true;
  };

  param_cb_ = node->add_on_set_parameters_callback(
    [this, parse_nonneg, parse_finite](const std::vector<rclcpp::Parameter> & params) {
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = true;
      for (const auto & p : params) {
        const auto & name = p.get_name();
        if (name == kMotorModeParam) {
          if (p.get_type() != rclcpp::ParameterType::PARAMETER_STRING) {
            result.successful = false;
            result.reason = "arx_lift.motor_mode must be a string";
            return result;
          }
          MotorMode mode;
          if (!parseMotorMode(p.as_string(), mode)) {
            result.successful = false;
            result.reason = "arx_lift.motor_mode must be 'soft_p'|'position' or 'hybrid'";
            return result;
          }
          motor_mode_.store(static_cast<int>(mode));
          RCLCPP_INFO(
            get_logger(), "%s -> %s", kMotorModeParam, motorModeName(mode));
        } else if (name == kSoftPKpParam) {
          double v = 0.0;
          if (!parse_nonneg(p, v, result.reason, kSoftPKpParam)) {
            result.successful = false;
            return result;
          }
          soft_p_kp_.store(v);
          RCLCPP_INFO(get_logger(), "%s -> %.3f", kSoftPKpParam, v);
        } else if (name == kHybridKpParam) {
          double v = 0.0;
          if (!parse_nonneg(p, v, result.reason, kHybridKpParam)) {
            result.successful = false;
            return result;
          }
          hybrid_kp_.store(v);
          RCLCPP_INFO(get_logger(), "%s -> %.3f", kHybridKpParam, v);
        } else if (name == kHybridKdParam) {
          double v = 0.0;
          if (!parse_nonneg(p, v, result.reason, kHybridKdParam)) {
            result.successful = false;
            return result;
          }
          if (v > kHybridKdMax) {
            RCLCPP_WARN(
              get_logger(),
              "%s=%.3f exceeds Hybrid MIT pack max %.1f; will clamp when sending",
              kHybridKdParam, v, kHybridKdMax);
          }
          hybrid_kd_.store(v);
          RCLCPP_INFO(get_logger(), "%s -> %.3f", kHybridKdParam, v);
        } else if (name == kGravityCompParam) {
          double v = 0.0;
          if (!parse_finite(p, v, result.reason, kGravityCompParam)) {
            result.successful = false;
            return result;
          }
          gravity_compensation_torque_.store(v);
          RCLCPP_INFO(get_logger(), "%s -> %.3f", kGravityCompParam, v);
        } else if (name == kCoulombFrictionParam) {
          double v = 0.0;
          if (!parse_nonneg(p, v, result.reason, kCoulombFrictionParam)) {
            result.successful = false;
            return result;
          }
          coulomb_friction_torque_.store(v);
          RCLCPP_INFO(get_logger(), "%s -> %.3f", kCoulombFrictionParam, v);
        } else if (name == kFrictionVelEpsParam) {
          double v = 0.0;
          if (!parse_nonneg(p, v, result.reason, kFrictionVelEpsParam)) {
            result.successful = false;
            return result;
          }
          friction_vel_eps_mps_.store(v);
          RCLCPP_INFO(get_logger(), "%s -> %.4f", kFrictionVelEpsParam, v);
        } else if (name == "status_debug") {
          if (p.get_type() != rclcpp::ParameterType::PARAMETER_BOOL) {
            result.successful = false;
            result.reason = "status_debug must be a bool";
            return result;
          }
          status_debug_.store(p.as_bool());
          RCLCPP_INFO(
            get_logger(), "status_debug -> %s",
            status_debug_.load() ? "true" : "false");
        }
      }
      return result;
    });

  RCLCPP_INFO(
    get_logger(),
    "Dynamic params: %s=%s %s=%.3f %s=%.3f %s=%.3f %s=%.3f %s=%.3f %s=%.4f "
    "status_debug=%s",
    kMotorModeParam, initial_mode.c_str(), kSoftPKpParam, soft_p_kp,
    kHybridKpParam, hybrid_kp, kHybridKdParam, hybrid_kd,
    kGravityCompParam, gravity_comp, kCoulombFrictionParam, coulomb_friction,
    kFrictionVelEpsParam, friction_vel_eps_mps,
    status_debug ? "true" : "false");
}

/**
 * @brief Hybrid τ_ff = τ_g - μ·sign(v_cmd)；|v_cmd| 死区内仅重力。
 *
 * 符号按上层速度指令（高度系上行>0），不用反馈速度。
 * 死区：arx_lift.friction_vel_eps_mps（米/秒）换算为 SDK rad/s。
 */
double ArxLiftHardware::computeHybridFeedforward(double v_cmd_sdk) const
{
  const double tau_g = gravity_compensation_torque_.load();
  const double mu = coulomb_friction_torque_.load();
  const double eps_sdk =
    friction_vel_eps_mps_.load() * height_rad_per_meter_;
  double tau_f = 0.0;
  if (std::abs(v_cmd_sdk) > eps_sdk && mu > 0.0) {
    tau_f = -mu * ((v_cmd_sdk > 0.0) ? 1.0 : -1.0);
  }
  return std::clamp(tau_g + tau_f, -lift_max_torque_, lift_max_torque_);
}

/**
 * @brief Hybrid：直接跟 OCS2 位置 + 速度，sendLiftHybrid。
 *
 * τ_ff = gravity − μ·sign(v_cmd)（热调 arx_lift.*），再限幅。
 * **故意忽略** 上层 effort（全身 OCS2 RNEA），避免与 HI 前馈双重叠加过流。
 *
 * 底盘 vx/vy/wz：setChassisCmd 后 sendChassisOnly()（仅 0x701/0x703）。
 */
void ArxLiftHardware::sendHybridHoldOrTrack(
  double q_target_sdk, double dt_s, bool chassis_active)
{
  if (!lift_) {
    return;
  }
  (void)dt_s;

  ramp_q_sdk_ = std::clamp(q_target_sdk, 0.0, sdk_max_rad_);
  ramp_initialized_ = true;

  double v_d = lift_velocity_command_ * height_rad_per_meter_;
  const double v_max = lift_max_vel_ * height_rad_per_meter_;
  v_d = std::clamp(v_d, -v_max, v_max);

  const double hy_kp = hybrid_kp_.load();
  const double hy_kd = hybrid_kd_.load();
  const double t_ff = computeHybridFeedforward(v_d);
  (void)lift_effort_command_;

  lift_->read();
  lift_->exchangeLiftMotorMsg();
  const double p_motor = -ramp_q_sdk_;
  const double v_motor = -v_d;
  const double kd_send = std::min(hy_kd, kHybridKdMax);

  flushChassisForControlCycle(chassis_active);
  lift_->sendLiftHybrid(hy_kp, kd_send, p_motor, v_motor, t_ff);

  last_written_height_.store(ramp_q_sdk_);
  last_written_vel_.store(v_d);
}

/**
 * @brief Soft-P：直跟位置 + 常值重力，升降与底盘分开发。
 *
 * τ = kp·(p_des−p) + kd·(0−v) + τ_g。τ_g 与跟踪误差无关，BODY_FREE 下
 * p_des 跟着测量走时仍能持高。不加库仑摩擦：Soft-P 不跟速度，
 * −μ·sign(v) 在零速附近会抖，对悬停掉柱也无帮助。
 */
void ArxLiftHardware::sendSoftPHoldOrTrack(double q_sdk, bool chassis_active)
{
  if (!lift_) {
    return;
  }

  ramp_q_sdk_ = std::clamp(q_sdk, 0.0, sdk_max_rad_);
  ramp_initialized_ = true;

  const double soft_kp = soft_p_kp_.load();
  const double t_g = std::clamp(
    gravity_compensation_torque_.load(), -lift_max_torque_, lift_max_torque_);
  (void)lift_effort_command_;
  (void)lift_velocity_command_;

  lift_->config_.lift_kp = soft_kp;
  lift_->config_.gravity_compensation_torque = t_g;
  lift_->setHeight(ramp_q_sdk_);
  lift_->read();
  lift_->exchangeLiftMotorMsg();

  const double p_motor = -ramp_q_sdk_;
  constexpr double kSoftPKd = 0.5;
  const double kd_send = std::min(kSoftPKd, kHybridKdMax);

  flushChassisForControlCycle(chassis_active);
  lift_->sendLiftHybrid(soft_kp, kd_send, p_motor, 0.0, t_g);

  last_written_height_.store(ramp_q_sdk_);
  last_written_vel_.store(0.0);
}

void ArxLiftHardware::flushChassisForControlCycle(bool chassis_active)
{
  if (!enable_chassis_cmd_vel_) {
    return;
  }
  if (chassis_active) {
    chassis_park_flushed_ = false;
    flushChassisCanOnly();
  } else if (!chassis_park_flushed_) {
    flushChassisParkOnce();
  }
}

void ArxLiftHardware::flushChassisCanOnly()
{
  if (!lift_) {
    return;
  }
  lift_->setWheelVel(0.0, 0.0, 0.0, 0.0);
  lift_->sendChassisOnly();
}

void ArxLiftHardware::flushChassisParkOnce()
{
  if (!lift_ || chassis_park_flushed_) {
    return;
  }
  flushChassisCanOnly();
  chassis_park_flushed_ = true;
}

void ArxLiftHardware::setupChassisFeedbackPublishers()
{
  teardownChassisFeedbackPublishers();
  if (
    !enable_chassis_feedback_ && !enable_chassis_odom_ &&
    !enable_chassis_odom_tf_ && !enable_chassis_odom_debug_)
  {
    return;
  }
  auto node = get_node();
  if (!node) {
    RCLCPP_WARN(
      get_logger(),
      "Chassis feedback/odom enabled but no ROS node; publishers skipped");
    return;
  }

  {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    odom_yaw_initialized_ = false;
    odom_yaw0_ = 0.0;
    odom_x_ = 0.0;
    odom_y_ = 0.0;
    odom_yaw_ = 0.0;
  }

  if (!buildChassisKinematics()) {
    RCLCPP_ERROR(
      get_logger(),
      "Chassis kinematics build failed; odom/TF/debug will stay disabled");
  }

  if (enable_chassis_feedback_) {
    imu_pub_ = node->create_publisher<sensor_msgs::msg::Imu>(
      chassis_imu_topic_, rclcpp::SystemDefaultsQoS());
    wheel_vel_pub_ = node->create_publisher<std_msgs::msg::Float64MultiArray>(
      chassis_wheel_vel_topic_, rclcpp::SystemDefaultsQoS());
    RCLCPP_INFO(
      get_logger(),
      "Chassis feedback enabled (official lift_controller style): imu=%s "
      "wheel_vel=%s",
      chassis_imu_topic_.c_str(), chassis_wheel_vel_topic_.c_str());
  }
  if (enable_chassis_odom_ || enable_chassis_odom_tf_) {
    odom_pub_ = node->create_publisher<nav_msgs::msg::Odometry>(
      chassis_odom_topic_, rclcpp::SystemDefaultsQoS());
    RCLCPP_INFO(
      get_logger(),
      "Chassis odom = inverse of Isaac Holonomic (mecanum=90°) + IMU yaw "
      "(NOT cmd_vel): topic=%s frames %s→%s wheel_r=%.4f m",
      chassis_odom_topic_.c_str(), chassis_odom_parent_frame_.c_str(),
      chassis_odom_child_frame_.c_str(), chassis_wheel_radius_m_);
  }
  if (enable_chassis_odom_tf_) {
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node);
    RCLCPP_INFO(
      get_logger(),
      "Chassis odom TF enabled: %s→%s (continuous; replaces WBC identity placeholder)",
      chassis_odom_parent_frame_.c_str(), chassis_odom_child_frame_.c_str());
  }
  if (enable_chassis_odom_debug_) {
    wheel_vel_expected_pub_ =
      node->create_publisher<std_msgs::msg::Float64MultiArray>(
        chassis_wheel_vel_expected_topic_, rclcpp::SystemDefaultsQoS());
    RCLCPP_INFO(
      get_logger(),
      "Chassis odom debug: cmd_vel→expected wheel ω on %s (compare with %s)",
      chassis_wheel_vel_expected_topic_.c_str(),
      chassis_wheel_vel_topic_.c_str());
  }

  // Activate 立刻发一帧 identity，保证 WBC 探测窗内能看到外部 TF，从而跳过占位 TF。
  if (enable_chassis_odom_ || enable_chassis_odom_tf_) {
    rclcpp::Time stamp;
    try {
      stamp = get_clock()->now();
    } catch (...) {
      stamp = rclcpp::Clock(RCL_ROS_TIME).now();
    }
    publishChassisOdomAndTf(stamp, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, false);
  }
}

void ArxLiftHardware::teardownChassisFeedbackPublishers()
{
  imu_pub_.reset();
  wheel_vel_pub_.reset();
  wheel_vel_expected_pub_.reset();
  odom_pub_.reset();
  tf_broadcaster_.reset();
}

bool ArxLiftHardware::buildChassisKinematics()
{
  chassis_kinematics_ok_ = false;
  J_rw_from_body_.setZero();
  J_body_from_rw_.setZero();

  if (!std::isfinite(chassis_wheel_radius_m_) || chassis_wheel_radius_m_ <= 0.0) {
    return false;
  }
  // Isaac HolonomicController + isaacmecanumwheel:angle=90°：
  //   roller / drive dir = R_z(mecanum) * wheel_axis(X) = R_z(90°)*X = Ŷ
  //   再乘轮关节 R_z(θ)：d = R_z(θ)·Ŷ = [-sinθ, cosθ]
  // 接触速度：v = [vx − ωz·y, vy + ωz·x]
  // 约束：r·ω_i = d_i · v  ⇒  r·ω = J · [vx,vy,wz]
  // （与 chassis.xacro / FaSim ARX_LIFT2S 几何一致；官方 MCU 内部正向同模型）
  const double deg = chassis_mecanum_angle_deg_;
  if (!std::isfinite(deg)) {
    return false;
  }
  // 仅支持 90°（Lift2S Omnia）；其它角度需完整 Isaac 旋转链。
  if (std::abs(deg - 90.0) > 1e-3) {
    RCLCPP_WARN(
      get_logger(),
      "chassis_mecanum_angle_deg=%.3f not 90; using classic Ŷ drive model "
      "anyway (Lift2S asset uses 90)",
      deg);
  }

  for (int i = 0; i < 3; ++i) {
    const double th = wheel_yaw_[i];
    const double s = std::sin(th);
    const double c = std::cos(th);
    J_rw_from_body_(i, 0) = -s;
    J_rw_from_body_(i, 1) = c;
    J_rw_from_body_(i, 2) = wheel_x_[i] * c + wheel_y_[i] * s;
  }

  Eigen::ColPivHouseholderQR<Eigen::Matrix3d> qr(J_rw_from_body_);
  if (qr.rank() < 3) {
    RCLCPP_ERROR(get_logger(), "Chassis wheel Jacobian is singular");
    return false;
  }
  J_body_from_rw_ = qr.solve(Eigen::Matrix3d::Identity());
  if (!J_body_from_rw_.allFinite()) {
    return false;
  }
  chassis_kinematics_ok_ = true;
  RCLCPP_INFO(
    get_logger(),
    "Chassis kinematics OK (3-omni / Isaac mecanum=90 inverse): "
    "r=%.4f m wheels xyz=(%.4f,%.4f)/(%.4f,%.4f)/(%.4f,%.4f) "
    "yaw=%.4f/%.4f/%.4f sign=%.0f/%.0f/%.0f",
    chassis_wheel_radius_m_, wheel_x_[0], wheel_y_[0], wheel_x_[1], wheel_y_[1],
    wheel_x_[2], wheel_y_[2], wheel_yaw_[0], wheel_yaw_[1], wheel_yaw_[2],
    wheel_vel_sign_[0], wheel_vel_sign_[1], wheel_vel_sign_[2]);
  return true;
}

bool ArxLiftHardware::wheelVelFromBodyTwist(
  double vx, double vy, double wz, double wheel_out[3]) const
{
  if (!chassis_kinematics_ok_ || !wheel_out) {
    return false;
  }
  if (
    !std::isfinite(vx) || !std::isfinite(vy) || !std::isfinite(wz) ||
    chassis_wheel_radius_m_ <= 0.0)
  {
    return false;
  }
  const Eigen::Vector3d body(vx, vy, wz);
  const Eigen::Vector3d rw = J_rw_from_body_ * body;
  for (int i = 0; i < 3; ++i) {
    // 期望轮速与反馈同号约定：测量侧乘 sign，故命令侧也乘 sign。
    wheel_out[i] =
      (rw(i) / chassis_wheel_radius_m_) * wheel_vel_sign_[i];
    if (!std::isfinite(wheel_out[i])) {
      return false;
    }
  }
  return true;
}

bool ArxLiftHardware::bodyTwistFromWheelVel(
  const double wheel_vel[4], double & vx, double & vy, double & wz) const
{
  if (!chassis_kinematics_ok_) {
    return false;
  }
  Eigen::Vector3d rw;
  for (int i = 0; i < 3; ++i) {
    if (!std::isfinite(wheel_vel[i])) {
      return false;
    }
    const double omega = wheel_vel[i] * wheel_vel_sign_[i];
    rw(i) = chassis_wheel_radius_m_ * omega;
  }
  const Eigen::Vector3d twist = J_body_from_rw_ * rw;
  if (!twist.allFinite()) {
    return false;
  }
  vx = twist(0);
  vy = twist(1);
  wz = twist(2);
  return true;
}

void ArxLiftHardware::publishChassisOdomAndTf(
  const rclcpp::Time & stamp, double x, double y, double yaw, double vx_b,
  double vy_b, double wz_b, bool twist_ok)
{
  tf2::Quaternion q_yaw;
  q_yaw.setRPY(0.0, 0.0, yaw);

  if (enable_chassis_odom_tf_ && tf_broadcaster_) {
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = stamp;
    tf_msg.header.frame_id = chassis_odom_parent_frame_;
    tf_msg.child_frame_id = chassis_odom_child_frame_;
    tf_msg.transform.translation.x = x;
    tf_msg.transform.translation.y = y;
    tf_msg.transform.translation.z = 0.0;
    tf_msg.transform.rotation.x = q_yaw.x();
    tf_msg.transform.rotation.y = q_yaw.y();
    tf_msg.transform.rotation.z = q_yaw.z();
    tf_msg.transform.rotation.w = q_yaw.w();
    tf_broadcaster_->sendTransform(tf_msg);
  }

  if (odom_pub_) {
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = chassis_odom_parent_frame_;
    odom.child_frame_id = chassis_odom_child_frame_;
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation.x = q_yaw.x();
    odom.pose.pose.orientation.y = q_yaw.y();
    odom.pose.pose.orientation.z = q_yaw.z();
    odom.pose.pose.orientation.w = q_yaw.w();
    if (twist_ok) {
      odom.twist.twist.linear.x = vx_b;
      odom.twist.twist.linear.y = vy_b;
      odom.twist.twist.angular.z = wz_b;
    }
    odom_pub_->publish(odom);
  }
}

void ArxLiftHardware::updateChassisFeedbackAndOdom(double dt_s)
{
  if (
    !lift_ ||
    (!enable_chassis_feedback_ && !enable_chassis_odom_ &&
     !enable_chassis_odom_tf_ && !enable_chassis_odom_debug_))
  {
    return;
  }

  rclcpp::Time stamp;
  try {
    stamp = get_clock()->now();
  } catch (...) {
    stamp = rclcpp::Clock(RCL_ROS_TIME).now();
  }

  double wheel_vel[4] = {0.0, 0.0, 0.0, 0.0};
  double orientation[3] = {0.0, 0.0, 0.0};
  double angular_vel[3] = {0.0, 0.0, 0.0};
  double accel[3] = {0.0, 0.0, 0.0};
  bool sdk_ok = false;
  try {
    lift_->getWheelVel(wheel_vel);
    lift_->getOrientation(orientation);
    lift_->getAngularVel(angular_vel);
    lift_->getAccel(accel);
    sdk_ok = true;
  } catch (const std::exception & e) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Chassis SDK feedback read failed: %s (holding last odom/TF)", e.what());
  }

  // SDK 失败时仍刷新 world→base_link，避免 RViz Fixed Frame=world 断桥，
  // 也避免 WBC 因短暂缺 TF 回退到 identity 占位。
  if (!sdk_ok) {
    if (enable_chassis_odom_ || enable_chassis_odom_tf_) {
      double x = 0.0;
      double y = 0.0;
      double yaw = 0.0;
      {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        x = odom_x_;
        y = odom_y_;
        yaw = odom_yaw_;
      }
      publishChassisOdomAndTf(stamp, x, y, yaw, 0.0, 0.0, 0.0, false);
    }
    return;
  }

  if (enable_chassis_feedback_) {
    if (imu_pub_) {
      sensor_msgs::msg::Imu imu_msg;
      imu_msg.header.stamp = stamp;
      imu_msg.header.frame_id = chassis_odom_child_frame_;
      tf2::Quaternion q;
      q.setRPY(orientation[0], orientation[1], orientation[2]);
      imu_msg.orientation.x = q.x();
      imu_msg.orientation.y = q.y();
      imu_msg.orientation.z = q.z();
      imu_msg.orientation.w = q.w();
      imu_msg.angular_velocity.x = angular_vel[0];
      imu_msg.angular_velocity.y = angular_vel[1];
      imu_msg.angular_velocity.z = angular_vel[2];
      imu_msg.linear_acceleration.x = accel[0];
      imu_msg.linear_acceleration.y = accel[1];
      imu_msg.linear_acceleration.z = accel[2];
      imu_pub_->publish(imu_msg);
    }
    if (wheel_vel_pub_) {
      std_msgs::msg::Float64MultiArray wheels;
      wheels.data = {wheel_vel[0], wheel_vel[1], wheel_vel[2], wheel_vel[3]};
      wheel_vel_pub_->publish(wheels);
    }
  }

  // 用最近 cmd_vel 正解期望轮速，对比实测（验证符号/几何，不参与积分）。
  if (enable_chassis_odom_debug_ && wheel_vel_expected_pub_) {
    double w_exp[3] = {0.0, 0.0, 0.0};
    if (wheelVelFromBodyTwist(
        chassis_vx_.load(), chassis_vy_.load(), chassis_wz_.load(), w_exp))
    {
      std_msgs::msg::Float64MultiArray exp_msg;
      exp_msg.data = {w_exp[0], w_exp[1], w_exp[2], 0.0};
      wheel_vel_expected_pub_->publish(exp_msg);
    }
  }

  if (!enable_chassis_odom_ && !enable_chassis_odom_tf_) {
    return;
  }

  double vx_b = 0.0;
  double vy_b = 0.0;
  double wz_b = 0.0;
  const bool twist_ok = bodyTwistFromWheelVel(wheel_vel, vx_b, vy_b, wz_b);
  if (!twist_ok) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Chassis wheel inverse kinematics failed; odom twist held at last pose");
  } else {
    // Prefer IMU yaw-rate when available; else kinematic wz.
    if (std::isfinite(angular_vel[2])) {
      wz_b = angular_vel[2];
    }
  }

  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
  {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    if (!odom_yaw_initialized_) {
      odom_yaw0_ = orientation[2];
      odom_yaw_initialized_ = true;
      odom_x_ = 0.0;
      odom_y_ = 0.0;
      odom_yaw_ = 0.0;
    }

    yaw = orientation[2] - odom_yaw0_;
    while (yaw > M_PI) {
      yaw -= 2.0 * M_PI;
    }
    while (yaw < -M_PI) {
      yaw += 2.0 * M_PI;
    }
    odom_yaw_ = yaw;

    if (twist_ok && dt_s > 0.0) {
      const double c = std::cos(yaw);
      const double s = std::sin(yaw);
      odom_x_ += (vx_b * c - vy_b * s) * dt_s;
      odom_y_ += (vx_b * s + vy_b * c) * dt_s;
    }
    x = odom_x_;
    y = odom_y_;
  }

  publishChassisOdomAndTf(stamp, x, y, yaw, vx_b, vy_b, wz_b, twist_ok);
}

void ArxLiftHardware::setupChassisCmdVelSubscription()
{
  teardownChassisCmdVelSubscription();
  if (!enable_chassis_cmd_vel_) {
    return;
  }
  auto node = get_node();
  if (!node) {
    RCLCPP_WARN(
      get_logger(),
      "enable_chassis_cmd_vel=true but no ROS node; chassis mapping disabled");
    return;
  }

  chassis_vx_.store(0.0);
  chassis_vy_.store(0.0);
  chassis_wz_.store(0.0);
  chassis_cmd_stamp_ns_.store(0);

  // ROS FLU Twist → setChassisCmd 原样（与 body_communicator / joy 一致）。
  // 勿套用 PosCmd VR 的 -chy：那是 PosCmd 桥接约定，SDK 量化已对 Y 含符号。
  chassis_cmd_sub_ = node->create_subscription<geometry_msgs::msg::Twist>(
    chassis_cmd_vel_topic_, rclcpp::SystemDefaultsQoS(),
    [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
      chassis_vx_.store(msg->linear.x);
      chassis_vy_.store(msg->linear.y);
      chassis_wz_.store(msg->angular.z);
      chassis_cmd_stamp_ns_.store(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now().time_since_epoch())
          .count());
    });

  RCLCPP_INFO(
    get_logger(),
    "Chassis cmd_vel mapping enabled: topic=%s timeout=%.2f s "
    "(mode 1 run / 2 park; Twist vx/vy/wz passthrough)",
    chassis_cmd_vel_topic_.c_str(), chassis_cmd_timeout_sec_);
}

void ArxLiftHardware::teardownChassisCmdVelSubscription()
{
  chassis_cmd_sub_.reset();
  chassis_vx_.store(0.0);
  chassis_vy_.store(0.0);
  chassis_wz_.store(0.0);
  chassis_cmd_stamp_ns_.store(0);
}

bool ArxLiftHardware::applyChassisCmd(bool force_park)
{
  if (!lift_) {
    return false;
  }

  if (
    !enable_chassis_cmd_vel_ || force_park || soft_stop_active_.load() ||
    !command_enabled_.load())
  {
    lift_->setChassisCmd(0.0, 0.0, 0.0, 2);
    return false;
  }

  const int64_t stamp_ns = chassis_cmd_stamp_ns_.load();
  bool timed_out = (stamp_ns <= 0);
  if (!timed_out) {
    const auto now_ns =
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch())
        .count();
    const double age_s =
      static_cast<double>(now_ns - stamp_ns) * 1e-9;
    timed_out = age_s > chassis_cmd_timeout_sec_;
  }

  if (timed_out) {
    lift_->setChassisCmd(0.0, 0.0, 0.0, 2);
    return false;
  }

  // 官方 joy/VR：运行 mode=1；停车 mode=2
  lift_->setChassisCmd(
    chassis_vx_.load(), chassis_vy_.load(), chassis_wz_.load(), 1);
  return true;
}

/** @brief 关闭指令门控并 join 后台循环线程。 */
void ArxLiftHardware::stop_loop_thread()
{
  command_enabled_ = false;
  loop_running_ = false;
  if (loop_thread_.joinable()) {
    loop_thread_.join();
  }
}

/**
 * @brief 析构：停线程；故意不销毁 lift_，避免厂商 SocketCAN 线程纯虚崩溃。
 */
ArxLiftHardware::~ArxLiftHardware()
{
  stop_loop_thread();
  teardownChassisCmdVelSubscription();
  param_cb_.reset();
  motor_pub_.reset();
  // 故意泄漏 lift_：厂商 SocketCAN 收包线程仍在跑时销毁 LiftHeadControlLoop
  // 会触发 "pure virtual method called"。
}

/**
 * @brief 从 URDF hardware 参数加载 CAN、单位换算、增益与电机模式。
 * @return SUCCESS / ERROR
 */
hardware_interface::CallbackReturn ArxLiftHardware::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (
    hardware_interface::SystemInterface::on_init(params) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (info_.joints.size() != 1) {
    RCLCPP_ERROR(
      get_logger(),
      "ArxLiftHardware expects exactly 1 joint, got %zu", info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  lift_joint_name_ = info_.joints.front().name;
  can_name_ = get_hw_param(info_, "can_name", "can5");
  const std::string robot_type_str = get_hw_param(info_, "robot_type", "2");
  try {
    robot_type_ = std::stoi(robot_type_str);
  } catch (const std::exception &) {
    RCLCPP_ERROR(
      get_logger(), "Invalid robot_type parameter: '%s'",
      robot_type_str.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (robot_type_ < 0 || robot_type_ > 2) {
    RCLCPP_ERROR(
      get_logger(), "robot_type must be 0(LIFT)/1(X7S)/2(LIFTS), got %d",
      robot_type_);
    return hardware_interface::CallbackReturn::ERROR;
  }

  double gravity = -1.01;
  double coulomb_friction = 0.32;
  double friction_vel_eps = 0.01;
  double max_vel = 0.20;
  double max_torque = 15.0;
  double soft_p_kp = 50.0;
  double hybrid_kp = 5.0;
  double hybrid_kd = 2.0;
  double height_scale = 41.54;
  double span_m = 0.48;
  double sdk_max = 20.0;
  double ramp_vel = 0.04;

  if (!parse_double_param(
      info_, "gravity_compensation_torque", -1.01, gravity, get_logger()) ||
    !parse_double_param(
      info_, "coulomb_friction_torque", 0.32, coulomb_friction, get_logger()) ||
    !parse_double_param(
      info_, "friction_vel_eps_mps", 0.01, friction_vel_eps, get_logger()) ||
    !parse_double_param(info_, "lift_max_vel", 0.20, max_vel, get_logger()) ||
    !parse_double_param(
      info_, "lift_max_torque", 15.0, max_torque, get_logger()) ||
    !parse_double_param(info_, "sdk_max_rad", 20.0, sdk_max, get_logger()) ||
    !parse_double_param(info_, "cmd_ramp_vel", 0.04, ramp_vel, get_logger()) ||
    !parse_double_param(info_, "hybrid_kp", 5.0, hybrid_kp, get_logger()))
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // 米 ↔ 电机弧度（优先 height_rad_per_meter；别名 height_to_motor）
  if (!get_hw_param(info_, "height_rad_per_meter", "").empty()) {
    if (!parse_double_param(
        info_, "height_rad_per_meter", 41.54, height_scale, get_logger()))
    {
      return hardware_interface::CallbackReturn::ERROR;
    }
  } else if (!parse_double_param(
      info_, "height_to_motor", 41.54, height_scale, get_logger()))
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!get_hw_param(info_, "height_span_m", "").empty()) {
    if (!parse_double_param(info_, "height_span_m", 0.48, span_m, get_logger())) {
      return hardware_interface::CallbackReturn::ERROR;
    }
  } else if (!parse_double_param(info_, "max_height_m", 0.48, span_m, get_logger())) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // soft_p_kp（别名：lift_kp / kp）
  if (!get_hw_param(info_, "soft_p_kp", "").empty()) {
    if (!parse_double_param(info_, "soft_p_kp", 50.0, soft_p_kp, get_logger())) {
      return hardware_interface::CallbackReturn::ERROR;
    }
  } else if (!get_hw_param(info_, "lift_kp", "").empty()) {
    if (!parse_double_param(info_, "lift_kp", 50.0, soft_p_kp, get_logger())) {
      return hardware_interface::CallbackReturn::ERROR;
    }
  } else if (!get_hw_param(info_, "kp", "").empty()) {
    if (!parse_double_param(info_, "kp", 50.0, soft_p_kp, get_logger())) {
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  // 优先 hybrid_kd；别名 kd
  if (!get_hw_param(info_, "hybrid_kd", "").empty()) {
    if (!parse_double_param(info_, "hybrid_kd", 2.0, hybrid_kd, get_logger())) {
      return hardware_interface::CallbackReturn::ERROR;
    }
  } else if (!get_hw_param(info_, "kd", "").empty()) {
    if (!parse_double_param(info_, "kd", 2.0, hybrid_kd, get_logger())) {
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  auto ok_gain = [](double v) { return std::isfinite(v) && v >= 0.0; };
  if (!ok_gain(soft_p_kp) || !ok_gain(hybrid_kp) || !ok_gain(hybrid_kd) ||
    !ok_gain(coulomb_friction) || !ok_gain(friction_vel_eps))
  {
    RCLCPP_ERROR(
      get_logger(),
      "soft_p_kp/hybrid_kp/hybrid_kd/coulomb_friction_torque/"
      "friction_vel_eps_mps must be finite and >= 0 "
      "(got %.3f / %.3f / %.3f / %.3f / %.4f)",
      soft_p_kp, hybrid_kp, hybrid_kd, coulomb_friction, friction_vel_eps);
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (!std::isfinite(gravity)) {
    RCLCPP_ERROR(get_logger(), "gravity_compensation_torque must be finite");
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (height_scale <= 0.0 || span_m <= 0.0 || sdk_max <= 0.0 || ramp_vel <= 0.0) {
    RCLCPP_ERROR(
      get_logger(),
      "height scale / span / sdk_max / cmd_ramp_vel must be > 0");
    return hardware_interface::CallbackReturn::ERROR;
  }

  gravity_compensation_torque_.store(gravity);
  coulomb_friction_torque_.store(coulomb_friction);
  friction_vel_eps_mps_.store(friction_vel_eps);
  lift_max_vel_ = max_vel;
  lift_max_torque_ = max_torque;
  height_rad_per_meter_ = height_scale;
  height_span_m_ = span_m;
  sdk_max_rad_ = sdk_max;
  cmd_ramp_vel_mps_ = ramp_vel;
  soft_p_kp_.store(soft_p_kp);
  hybrid_kp_.store(hybrid_kp);
  hybrid_kd_.store(hybrid_kd);

  motor_mode_param_ = get_hw_param(info_, "lift_motor_mode", "hybrid");
  MotorMode mode;
  if (!parseMotorMode(motor_mode_param_, mode)) {
    RCLCPP_ERROR(
      get_logger(),
      "Invalid lift_motor_mode '%s' (use soft_p|position or hybrid)",
      motor_mode_param_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }
  motor_mode_.store(static_cast<int>(mode));
  motor_mode_param_ = motorModeName(mode);

  {
    bool dbg = false;
    parse_bool_param(info_, "status_debug", false, dbg);
    status_debug_.store(dbg);
  }

  parse_bool_param(info_, "shutdown_return_home", false, shutdown_return_home_);
  if (!parse_double_param(
      info_, "shutdown_height_m", 0.0, shutdown_height_m_, get_logger()) ||
    !parse_double_param(
      info_, "shutdown_home_velocity", 0.10, shutdown_home_velocity_,
      get_logger()) ||
    !parse_double_param(
      info_, "shutdown_home_timeout", 2.0, shutdown_home_timeout_sec_,
      get_logger()))
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (!std::isfinite(shutdown_home_velocity_) || shutdown_home_velocity_ <= 0.0) {
    shutdown_home_velocity_ = 0.10;
  }
  if (!std::isfinite(shutdown_home_timeout_sec_) ||
    shutdown_home_timeout_sec_ <= 0.0)
  {
    shutdown_home_timeout_sec_ = 2.0;
  }
  shutdown_height_m_ = std::clamp(shutdown_height_m_, 0.0, span_m);
  safe_exit_done_ = false;
  soft_stop_active_ = false;

  parse_bool_param(
    info_, "enable_chassis_cmd_vel", false, enable_chassis_cmd_vel_);
  chassis_cmd_vel_topic_ =
    get_hw_param(info_, "chassis_cmd_vel_topic", "/cmd_vel");
  if (chassis_cmd_vel_topic_.empty()) {
    chassis_cmd_vel_topic_ = "/cmd_vel";
  }
  if (!parse_double_param(
      info_, "chassis_cmd_timeout", 0.3, chassis_cmd_timeout_sec_,
      get_logger()))
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (
    !std::isfinite(chassis_cmd_timeout_sec_) ||
    chassis_cmd_timeout_sec_ <= 0.0)
  {
    chassis_cmd_timeout_sec_ = 0.3;
  }

  // LIFTS(setRobotType=2) 的 .so 不写 max_vel_*；缺省用 X7S 同档，可用 URDF 覆盖。
  if (!parse_double_param(
      info_, "chassis_max_vel_x", 2.0, chassis_max_vel_x_, get_logger()) ||
    !parse_double_param(
      info_, "chassis_max_vel_y", 2.0, chassis_max_vel_y_, get_logger()) ||
    !parse_double_param(
      info_, "chassis_max_vel_z", 4.0, chassis_max_vel_z_, get_logger()))
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
  auto ok_max = [](double v) { return std::isfinite(v) && v > 0.0; };
  if (
    !ok_max(chassis_max_vel_x_) || !ok_max(chassis_max_vel_y_) ||
    !ok_max(chassis_max_vel_z_))
  {
    RCLCPP_ERROR(
      get_logger(),
      "chassis_max_vel_{x,y,z} must be finite and > 0 (got %.3f / %.3f / %.3f)",
      chassis_max_vel_x_, chassis_max_vel_y_, chassis_max_vel_z_);
    return hardware_interface::CallbackReturn::ERROR;
  }

  // 官方无 odom：只发 IMU+轮速。本 HI 对齐 raw，另用轮速逆解+IMU 自算 /arx_lift/odom。
  parse_bool_param(
    info_, "enable_chassis_feedback", true, enable_chassis_feedback_);
  parse_bool_param(info_, "enable_chassis_odom", true, enable_chassis_odom_);
  parse_bool_param(
    info_, "enable_chassis_odom_tf", true, enable_chassis_odom_tf_);
  parse_bool_param(
    info_, "enable_chassis_odom_debug", true, enable_chassis_odom_debug_);
  chassis_odom_parent_frame_ =
    get_hw_param(info_, "chassis_odom_parent_frame", "world");
  chassis_odom_child_frame_ =
    get_hw_param(info_, "chassis_odom_child_frame", "base_link");
  chassis_imu_topic_ = get_hw_param(info_, "chassis_imu_topic", "/arx_imu");
  chassis_wheel_vel_topic_ =
    get_hw_param(info_, "chassis_wheel_vel_topic", "/arx_lift/wheel_vel");
  chassis_wheel_vel_expected_topic_ = get_hw_param(
    info_, "chassis_wheel_vel_expected_topic",
    "/arx_lift/wheel_vel_expected");
  chassis_odom_topic_ =
    get_hw_param(info_, "chassis_odom_topic", "/arx_lift/odom");
  if (!parse_double_param(
      info_, "chassis_wheel_radius_m", 0.075, chassis_wheel_radius_m_,
      get_logger()))
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (!parse_double_param(
      info_, "chassis_mecanum_angle_deg", 90.0, chassis_mecanum_angle_deg_,
      get_logger()))
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (
    !std::isfinite(chassis_wheel_radius_m_) || chassis_wheel_radius_m_ <= 0.0)
  {
    RCLCPP_ERROR(
      get_logger(), "chassis_wheel_radius_m must be finite and > 0 (got %.4f)",
      chassis_wheel_radius_m_);
    return hardware_interface::CallbackReturn::ERROR;
  }
  // 默认几何 = chassis.xacro / FaSim ARX_LIFT2S（可覆盖）。
  if (!parse_triple_param(
      info_, "chassis_wheel_x_m", wheel_x_, get_logger()) ||
    !parse_triple_param(
      info_, "chassis_wheel_y_m", wheel_y_, get_logger()) ||
    !parse_triple_param(
      info_, "chassis_wheel_yaw_rad", wheel_yaw_, get_logger()) ||
    !parse_triple_param(
      info_, "chassis_wheel_vel_sign", wheel_vel_sign_, get_logger()))
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
  for (int i = 0; i < 3; ++i) {
    if (
      !std::isfinite(wheel_x_[i]) || !std::isfinite(wheel_y_[i]) ||
      !std::isfinite(wheel_yaw_[i]) || !std::isfinite(wheel_vel_sign_[i]) ||
      wheel_vel_sign_[i] == 0.0)
    {
      RCLCPP_ERROR(
        get_logger(),
        "Invalid chassis wheel geometry/sign at index %d", i);
      return hardware_interface::CallbackReturn::ERROR;
    }
    // 归一到 ±1，避免误写成半径之类。
    wheel_vel_sign_[i] = (wheel_vel_sign_[i] < 0.0) ? -1.0 : 1.0;
  }
  if (!buildChassisKinematics()) {
    RCLCPP_ERROR(get_logger(), "Failed to build chassis kinematics at init");
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (chassis_odom_parent_frame_.empty()) {
    chassis_odom_parent_frame_ = "world";
  }
  if (chassis_odom_child_frame_.empty()) {
    chassis_odom_child_frame_ = "base_link";
  }
  if (chassis_imu_topic_.empty()) {
    chassis_imu_topic_ = "/arx_imu";
  }
  if (chassis_wheel_vel_topic_.empty()) {
    chassis_wheel_vel_topic_ = "/arx_lift/wheel_vel";
  }
  if (chassis_wheel_vel_expected_topic_.empty()) {
    chassis_wheel_vel_expected_topic_ = "/arx_lift/wheel_vel_expected";
  }
  if (chassis_odom_topic_.empty()) {
    chassis_odom_topic_ = "/arx_lift/odom";
  }

  lift_position_ = 0.0;
  lift_velocity_ = 0.0;
  lift_effort_ = 0.0;
  lift_position_command_ = 0.0;
  lift_velocity_command_ = 0.0;
  lift_effort_command_ = 0.0;
  command_enabled_ = false;
  last_written_height_ = 0.0;
  last_written_vel_ = 0.0;
  last_applied_mode_ = -1;
  lift_.reset();

  RCLCPP_INFO(
    get_logger(),
    "ArxLiftHardware init: joint=%s can=%s robot_type=%d "
    "motor_mode=%s soft_p_kp=%.3f hybrid_kp=%.3f hybrid_kd=%.3f "
    "gravity=%.3f coulomb_friction=%.3f friction_vel_eps=%.4f m/s "
    "ramp=%.3f m/s shutdown_return_home=%s height=%.3f "
    "enable_chassis_cmd_vel=%s topic=%s timeout=%.2f s "
    "chassis_max_vel=%.2f/%.2f/%.2f "
    "chassis_feedback=%s odom=%s odom_tf=%s odom_debug=%s (%s→%s) "
    "wheel_r=%.4f mecanum_deg=%.1f",
    lift_joint_name_.c_str(), can_name_.c_str(), robot_type_,
    motor_mode_param_.c_str(), soft_p_kp_.load(), hybrid_kp_.load(),
    hybrid_kd_.load(), gravity_compensation_torque_.load(),
    coulomb_friction_torque_.load(), friction_vel_eps_mps_.load(),
    cmd_ramp_vel_mps_, shutdown_return_home_ ? "true" : "false",
    shutdown_height_m_, enable_chassis_cmd_vel_ ? "true" : "false",
    chassis_cmd_vel_topic_.c_str(), chassis_cmd_timeout_sec_,
    chassis_max_vel_x_, chassis_max_vel_y_, chassis_max_vel_z_,
    enable_chassis_feedback_ ? "true" : "false",
    enable_chassis_odom_ ? "true" : "false",
    enable_chassis_odom_tf_ ? "true" : "false",
    enable_chassis_odom_debug_ ? "true" : "false",
    chassis_odom_parent_frame_.c_str(), chassis_odom_child_frame_.c_str(),
    chassis_wheel_radius_m_, chassis_mecanum_angle_deg_);

  return hardware_interface::CallbackReturn::SUCCESS;
}

/** @brief 导出 position / velocity / effort 状态接口。 */
std::vector<hardware_interface::StateInterface::ConstSharedPtr>
ArxLiftHardware::on_export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface::ConstSharedPtr> state_interfaces;
  state_interfaces.push_back(
    std::make_shared<hardware_interface::StateInterface>(
      lift_joint_name_, hardware_interface::HW_IF_POSITION, &lift_position_));
  state_interfaces.push_back(
    std::make_shared<hardware_interface::StateInterface>(
      lift_joint_name_, hardware_interface::HW_IF_VELOCITY, &lift_velocity_));
  state_interfaces.push_back(
    std::make_shared<hardware_interface::StateInterface>(
      lift_joint_name_, hardware_interface::HW_IF_EFFORT, &lift_effort_));
  return state_interfaces;
}

/**
 * @brief 导出 pos/vel/effort 指令接口，供 ocs2_wbc claim。
 * @note soft_p：只用 position；velocity/effort 可写但不下发电机。
 *       hybrid：直跟 position+velocity + HI τ_ff；增益由 arx_lift.* 决定（无 kp/kd command IF）。
 */
std::vector<hardware_interface::CommandInterface::SharedPtr>
ArxLiftHardware::on_export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface::SharedPtr> command_interfaces;
  command_interfaces.push_back(
    std::make_shared<hardware_interface::CommandInterface>(
      lift_joint_name_, hardware_interface::HW_IF_POSITION,
      &lift_position_command_));
  command_interfaces.push_back(
    std::make_shared<hardware_interface::CommandInterface>(
      lift_joint_name_, hardware_interface::HW_IF_VELOCITY,
      &lift_velocity_command_));
  command_interfaces.push_back(
    std::make_shared<hardware_interface::CommandInterface>(
      lift_joint_name_, hardware_interface::HW_IF_EFFORT,
      &lift_effort_command_));
  return command_interfaces;
}

/**
 * @brief 创建 SDK、启后台循环、等待校准后打开指令门控。
 *
 * 流程：构造 LiftHeadControlLoop → 底盘停车 → 注册热调参数 →
 * 循环线程（校准期 Soft-P）→ 等待约 2.5s → 指令对齐当前高度 → command_enabled。
 */
hardware_interface::CallbackReturn ArxLiftHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  stop_loop_thread();
  param_cb_.reset();

  try {
    const auto type =
      static_cast<arx::LiftHeadControlLoop::RobotType>(robot_type_);
    lift_ = std::make_shared<arx::LiftHeadControlLoop>(can_name_.c_str(), type);

    lift_->config_.lift_kp = soft_p_kp_.load();
    lift_->config_.gravity_compensation_torque =
      gravity_compensation_torque_.load();
    lift_->config_.lift_max_vel = lift_max_vel_;
    lift_->config_.lift_max_torque = lift_max_torque_;

    // LIFTS: SDK setRobotType 不写 max_vel_*，必须补上，否则 vy/wz 量化失效。
    lift_->setChassisVelocityLimits(
      chassis_max_vel_x_, chassis_max_vel_y_, chassis_max_vel_z_);
    RCLCPP_INFO(
      get_logger(),
      "Chassis velocity limits set to vx=%.2f vy=%.2f wz=%.2f (LIFTS SDK omits these)",
      chassis_max_vel_x_, chassis_max_vel_y_, chassis_max_vel_z_);

    // 底盘停车模式（mode=2）；轮速清零，避免 0x703 带未初始化值
    lift_->setWheelVel(0.0, 0.0, 0.0, 0.0);
    lift_->setChassisCmd(0.0, 0.0, 0.0, 2);

    setupDynamicParameters(
      motor_mode_param_, soft_p_kp_.load(), hybrid_kp_.load(),
      hybrid_kd_.load(), gravity_compensation_torque_.load(),
      coulomb_friction_torque_.load(), friction_vel_eps_mps_.load(),
      status_debug_.load());

    setupChassisCmdVelSubscription();
    setupChassisFeedbackPublishers();

    if (auto node = get_node()) {
      motor_pub_ = node->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/arx_lift/motor_status", rclcpp::SystemDefaultsQoS());
      RCLCPP_INFO(
        get_logger(),
        "Publishing raw lift motor status on /arx_lift/motor_status");
    }

    loop_running_ = true;
    command_enabled_ = false;  // 校准完成前不跟踪外部指令
    last_applied_mode_ = -1;
    ramp_initialized_ = false;
    safe_exit_done_ = false;
    soft_stop_active_ = false;
    chassis_park_flushed_ = false;
    using namespace std::chrono_literals;
    // 后台控制循环：soft_p 直跟 position / hybrid 直跟 pos+vel
    loop_thread_ = std::thread([this]() {
      auto last = std::chrono::steady_clock::now();
      while (loop_running_.load()) {
        try {
          if (!lift_) {
            std::this_thread::sleep_for(5ms);
            continue;
          }

          const auto now = std::chrono::steady_clock::now();
          double dt_s = std::chrono::duration<double>(now - last).count();
          last = now;
          // 限制 dt，避免长时间阻塞后一步跳过大
          if (dt_s < 1e-4) {
            dt_s = 1e-4;
          } else if (dt_s > 0.05) {
            dt_s = 0.05;
          }

          bool chassis_active = false;
          if (soft_stop_active_.load()) {
            // Soft stop at height: keep gravity τ_ff (τ=0 while elevated buzzes/drops).
            applyChassisCmd(/*force_park=*/true);
            const double q_hold = sdk_get_height_.load();
            ramp_q_sdk_ = q_hold;
            const int mode_i = motor_mode_.load();
            if (mode_i == static_cast<int>(MotorMode::SoftP)) {
              sendSoftPHoldOrTrack(q_hold, /*chassis_active=*/false);
            } else {
              // Hybrid soft-stop：重力 τ_ff 持高；底盘 park 最多刷一次
              const double hy_kd =
                std::min(std::max(hybrid_kd_.load(), 0.5), kHybridKdMax);
              const double t_ff = std::clamp(
                gravity_compensation_torque_.load(), -lift_max_torque_,
                lift_max_torque_);
              lift_->read();
              lift_->exchangeLiftMotorMsg();
              if (enable_chassis_cmd_vel_ && !chassis_park_flushed_) {
                flushChassisParkOnce();
              }
              lift_->sendLiftHybrid(0.0, hy_kd, -q_hold, 0.0, t_ff);
            }
            last_written_height_.store(q_hold);
            last_written_vel_.store(0.0);
          } else if (!command_enabled_.load()) {
            // 校准 / 停车：始终 Soft-P loop()
            applyChassisCmd(/*force_park=*/true);
            lift_->config_.lift_kp = soft_p_kp_.load();
            lift_->loop();
            chassis_park_flushed_ = true;
            ramp_initialized_ = false;
            last_applied_mode_ = -1;
          } else {
            chassis_active = applyChassisCmd(/*force_park=*/false);
            const double q_target = rosToSdk(lift_position_command_);
            if (!ramp_initialized_) {
              ramp_q_sdk_ = sdk_get_height_.load();
              ramp_initialized_ = true;
            }

            const int mode_i = motor_mode_.load();
            const double soft_kp = soft_p_kp_.load();
            const double hy_kp = hybrid_kp_.load();
            const double hy_kd = hybrid_kd_.load();
            if (mode_i != last_applied_mode_) {
              ramp_q_sdk_ = sdk_get_height_.load();
              last_applied_mode_ = mode_i;
              RCLCPP_INFO(
                get_logger(),
                "Lift motor mode applied: %s (soft_p_kp=%.3f hybrid_kp=%.3f kd=%.3f)",
                motorModeName(static_cast<MotorMode>(mode_i)), soft_kp, hy_kp,
                hy_kd);
            }

            if (mode_i == static_cast<int>(MotorMode::SoftP)) {
              sendSoftPHoldOrTrack(q_target, chassis_active);
            } else {
              sendHybridHoldOrTrack(q_target, dt_s, chassis_active);
            }
          }

          // 刷新状态供 read()/话题使用
          const double sdk_h = lift_->getHeight();
          const auto motor = lift_->getLiftMotorStatus();
          sdk_get_height_.store(sdk_h);
          motor_position_.store(motor.position);
          motor_velocity_.store(motor.velocity);
          motor_torque_.store(motor.torque);
          motor_current_.store(motor.current);
          motor_online_.store(lift_->isLiftMotorOnline() ? 1 : 0);
          motor_error_.store(lift_->getLiftMotorErrorCode());

          lift_position_ = sdkToRos(sdk_h);
          lift_velocity_ = sdkToRos(-motor.velocity);
          lift_effort_ = motor.torque;

          updateChassisFeedbackAndOdom(dt_s);
        } catch (const std::exception & e) {
          RCLCPP_ERROR_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "ArxLiftHardware loop thread failed: %s", e.what());
        }
        std::this_thread::sleep_for(2500us);  // 约 400 Hz
      }
    });

    // 等待 SDK 内部升降校准
    constexpr auto kCalibWait = std::chrono::milliseconds(2500);
    std::this_thread::sleep_for(kCalibWait);

    // 指令对齐当前反馈，再打开外部跟踪
    lift_position_command_ = lift_position_;
    ramp_q_sdk_ = rosToSdk(lift_position_command_);
    ramp_initialized_ = true;
    last_written_height_.store(ramp_q_sdk_);
    last_written_vel_.store(0.0);
    command_enabled_ = true;

    RCLCPP_INFO(
      get_logger(),
      "ArxLiftHardware activated on %s (ros_height=%.3f m, sdk_rad=%.3f, "
      "motor_mode=%s soft_p_kp=%.3f hybrid_kp=%.3f hybrid_kd=%.3f "
      "gravity=%.3f coulomb_friction=%.3f friction_vel_eps=%.4f)",
      can_name_.c_str(), lift_position_, sdk_get_height_.load(),
      motorModeName(static_cast<MotorMode>(motor_mode_.load())),
      soft_p_kp_.load(), hybrid_kp_.load(), hybrid_kd_.load(),
      gravity_compensation_torque_.load(), coulomb_friction_torque_.load(),
      friction_vel_eps_mps_.load());
    return hardware_interface::CallbackReturn::SUCCESS;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Failed to activate ArxLiftHardware: %s", e.what());
    stop_loop_thread();
    teardownChassisCmdVelSubscription();
    teardownChassisFeedbackPublishers();
    param_cb_.reset();
    lift_.reset();
    return hardware_interface::CallbackReturn::ERROR;
  }
}

/**
 * @brief 可选降高 + soft stop，再停循环、底盘停车；不销毁 lift_。
 */
hardware_interface::CallbackReturn ArxLiftHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "ArxLiftHardware on_deactivate: enterSafeExit");
  enterSafeExit(/*allow_return_home=*/true);
  teardownChassisCmdVelSubscription();
  teardownChassisFeedbackPublishers();
  param_cb_.reset();
  motor_pub_.reset();
  RCLCPP_INFO(get_logger(), "ArxLiftHardware deactivated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

/**
 * @brief 有序退出（幂等）；同样保留 lift_ 存活，避免 CAN 线程崩溃。
 */
hardware_interface::CallbackReturn ArxLiftHardware::on_shutdown(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "ArxLiftHardware on_shutdown: enterSafeExit");
  enterSafeExit(/*allow_return_home=*/true);
  teardownChassisCmdVelSubscription();
  teardownChassisFeedbackPublishers();
  param_cb_.reset();
  motor_pub_.reset();
  RCLCPP_INFO(
    get_logger(),
    "ArxLiftHardware shutdown (lift SDK left alive to avoid CAN thread crash)");
  return hardware_interface::CallbackReturn::SUCCESS;
}

/**
 * @brief 故障：仅 soft stop + 停线程，不降高度。
 */
hardware_interface::CallbackReturn ArxLiftHardware::on_error(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_ERROR(get_logger(), "ArxLiftHardware on_error: soft stop only");
  enterSafeExit(/*allow_return_home=*/false);
  teardownChassisCmdVelSubscription();
  teardownChassisFeedbackPublishers();
  param_cb_.reset();
  motor_pub_.reset();
  return hardware_interface::CallbackReturn::SUCCESS;
}

void ArxLiftHardware::enterSafeExit(bool allow_return_home)
{
  if (safe_exit_done_.exchange(true)) {
    return;
  }

  try {
    if (allow_return_home && shutdown_return_home_ && loop_running_.load() &&
      lift_)
    {
      interpolateLiftToShutdownHeight();
    }
    softStopLift();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      get_logger(), "enterSafeExit failed: %s; forcing soft stop + stop thread",
      e.what());
    try {
      softStopLift();
    } catch (...) {
    }
  }

  stop_loop_thread();
  soft_stop_active_ = false;

  if (lift_) {
    try {
      lift_->setWheelVel(0.0, 0.0, 0.0, 0.0);
      lift_->setChassisCmd(0.0, 0.0, 0.0, 2);
      lift_->sendChassisOnly();
    } catch (const std::exception & e) {
      RCLCPP_WARN(get_logger(), "Chassis park on safe exit failed: %s", e.what());
    }
  }
}

void ArxLiftHardware::interpolateLiftToShutdownHeight()
{
  if (!lift_ || !loop_running_.load()) {
    return;
  }

  const double goal =
    std::clamp(shutdown_height_m_, 0.0, height_span_m_);
  const double start = lift_position_;
  const double travel = std::abs(goal - start);
  constexpr double kMinDurationSec = 0.5;
  const double speed = std::max(0.02, std::abs(shutdown_home_velocity_));
  const double duration_sec = std::clamp(
    std::max(travel / speed, kMinDurationSec),
    kMinDurationSec,
    std::max(kMinDurationSec, shutdown_home_timeout_sec_));

  RCLCPP_WARN(
    get_logger(),
    "Shutdown interpolate lift: %.3f -> %.3f m, travel=%.3f, duration=%.2fs "
    "(ramp<=%.2f m/s)",
    start, goal, travel, duration_sec, speed);

  if (travel <= 0.01) {
    RCLCPP_INFO(get_logger(), "Lift already near shutdown height");
    lift_position_command_ = goal;
    return;
  }

  // Use shutdown velocity for the ramp while loop_thread_ tracks.
  const double old_ramp = cmd_ramp_vel_mps_;
  cmd_ramp_vel_mps_ = std::min(speed, lift_max_vel_);
  lift_position_command_ = goal;
  command_enabled_ = true;

  const auto t0 = std::chrono::steady_clock::now();
  constexpr auto kPoll = std::chrono::milliseconds(20);
  constexpr double kTolM = 0.015;
  while (loop_running_.load()) {
    const double elapsed =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - t0)
        .count();
    if (elapsed >= duration_sec) {
      break;
    }
    if (std::abs(lift_position_ - goal) <= kTolM) {
      break;
    }
    std::this_thread::sleep_for(kPoll);
  }

  cmd_ramp_vel_mps_ = old_ramp;
  RCLCPP_INFO(
    get_logger(),
    "Lift shutdown interpolate done (feedback=%.3f m, goal=%.3f m)",
    lift_position_, goal);
}

void ArxLiftHardware::softStopLift()
{
  if (!lift_ || !loop_running_.load()) {
    return;
  }

  // Hold current height; hybrid path uses kp≈0 / τ=0 via soft_stop_active_.
  lift_position_command_ = lift_position_;
  RCLCPP_INFO(
    get_logger(),
    "Lift soft stop (mode=%s, hold=%.3f m)",
    motorModeName(static_cast<MotorMode>(motor_mode_.load())),
    lift_position_command_);
  soft_stop_active_ = true;
  std::this_thread::sleep_for(std::chrono::milliseconds(250));
}

/**
 * @brief 状态已由后台线程写入缓冲；此处仅做存活检查。
 */
hardware_interface::return_type ArxLiftHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!lift_) {
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

/**
 * @brief 不向电机发指令；status_debug 时打日志并发布 /arx_lift/motor_status。
 */
hardware_interface::return_type ArxLiftHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!lift_) {
    return hardware_interface::return_type::ERROR;
  }

  if (!status_debug_.load()) {
    return hardware_interface::return_type::OK;
  }

  const double err = lift_position_command_ - lift_position_;
  const double motor_pos = motor_position_.load();
  const double sdk_h = sdk_get_height_.load();
  const int err_code = motor_error_.load();
  const std::string err_code_str =
    (err_code >= 0 && err_code < 256) ? std::to_string(err_code) : "n/a";
  const auto mode = static_cast<MotorMode>(motor_mode_.load());
  const double v_cmd_sdk = last_written_vel_.load();
  const double v_cmd_mps = sdkToRos(v_cmd_sdk);
  const double v_state_mps = lift_velocity_;
  double sign_v = 0.0;
  const double eps_sdk =
    friction_vel_eps_mps_.load() * height_rad_per_meter_;
  if (std::abs(v_cmd_sdk) > eps_sdk) {
    sign_v = (v_cmd_sdk > 0.0) ? 1.0 : -1.0;
  }

  RCLCPP_INFO_THROTTLE(
    get_logger(), *get_clock(), 2000,
    "ArxLiftHardware mode=%s cmd=%.4f m feedback=%.4f m err=%.4f m | "
    "sdk_rad=%.4f motor_pos=%.4f "
    "vel_rad=%.4f torq=%.4f curr=%.4f online=%d err_code=%s "
    "(last q=%.4f v_cmd=%.4f rad/s=%.4f m/s sign=%.0f v_state=%.4f m/s, "
    "soft_p_kp=%.3f hybrid_kp=%.3f hybrid_kd=%.3f)",
    motorModeName(mode), lift_position_command_, lift_position_, err, sdk_h,
    motor_pos, motor_velocity_.load(), motor_torque_.load(),
    motor_current_.load(), motor_online_.load(), err_code_str.c_str(),
    last_written_height_.load(), v_cmd_sdk, v_cmd_mps, sign_v, v_state_mps,
    soft_p_kp_.load(), hybrid_kp_.load(), hybrid_kd_.load());

  // data: 0-11 同前；12 v_cmd_sdk, 13 v_cmd_mps, 14 v_state_mps, 15 sign(v_cmd)
  if (motor_pub_) {
    std_msgs::msg::Float64MultiArray msg;
    msg.data = {
      motor_pos,
      motor_velocity_.load(),
      motor_torque_.load(),
      motor_current_.load(),
      sdk_h,
      static_cast<double>(motor_online_.load()),
      static_cast<double>(motor_error_.load()),
      -motor_pos,
      static_cast<double>(motor_mode_.load()),
      soft_p_kp_.load(),
      hybrid_kp_.load(),
      hybrid_kd_.load(),
      v_cmd_sdk,
      v_cmd_mps,
      v_state_mps,
      sign_v};
    motor_pub_->publish(msg);
  }

  return hardware_interface::return_type::OK;
}

}  // namespace arx_ros2_control

PLUGINLIB_EXPORT_CLASS(
  arx_ros2_control::ArxLiftHardware, hardware_interface::SystemInterface)
