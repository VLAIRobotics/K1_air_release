// Copyright 2025 vlai.
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

#include "xarm_hardware/v10_simple_hardware.hpp"
#include "xarm_hardware/command_mode_switch.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <cctype>
#include <chrono>
#include <thread>
#include <vector>

#include <kdl_parser/kdl_parser.hpp>
#include <urdf_parser/urdf_parser.h>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"

namespace xarm_hardware {

namespace {

// SDK positions are float. Accept the rounded representation of an exact URDF
// endpoint, but no additional float step beyond it. Commands still clamp to
// the exact double URDF interval before SDK conversion.
bool within_urdf_bounds(double position, const UrdfPositionLimits& limits) {
  const double lower = std::min(limits.lower, static_cast<double>(static_cast<float>(limits.lower)));
  const double upper = std::max(limits.upper, static_cast<double>(static_cast<float>(limits.upper)));
  return position >= lower && position <= upper;
}

bool sdk_call_ok(int ret, const char* api_name) {
  if (ret == XARM_SDK_OK) {
    return true;
  }
  char err_buf[512] = {0};
  xarm_sdk_get_last_error(err_buf, static_cast<int>(sizeof(err_buf)));
  RCLCPP_ERROR(rclcpp::get_logger("XArm_v10HW"), "%s failed ret=%d, last_error=%s", api_name,
               ret, err_buf);
  return false;
}

}  // namespace

XArm_v10HW::XArm_v10HW() : sdk_handle_(nullptr) {}

XArm_v10HW::~XArm_v10HW() {
  if (sdk_handle_) {
    xarm_sdk_destroy(sdk_handle_);
    sdk_handle_ = nullptr;
  }
}

bool XArm_v10HW::parse_config(const hardware_interface::HardwareInfo& info) {
  // Parse CAN interface (default: can0)
  auto it = info.hardware_parameters.find("can_interface");
  can_interface_ = (it != info.hardware_parameters.end()) ? it->second : "can0";

  // Parse arm prefix (default: empty for single arm, "left_" or "right_" for
  // bimanual)
  it = info.hardware_parameters.find("arm_prefix");
  arm_prefix_ = (it != info.hardware_parameters.end()) ? it->second : "";

  // Parse gripper enable (default: true for V10)
  it = info.hardware_parameters.find("hand");
  if (it == info.hardware_parameters.end()) {
    hand_ = true;  // Default to true for V10
  } else {
    // Handle both "true"/"True" and "false"/"False"
    std::string value = it->second;
    std::transform(value.begin(), value.end(), value.begin(), ::tolower);
    hand_ = (value == "true");
  }

  // Parse CAN-FD enable (default: true for V10)
  it = info.hardware_parameters.find("can_fd");
  if (it == info.hardware_parameters.end()) {
    can_fd_ = true;  // Default to true for V10
  } else {
    // Handle both "true"/"True" and "false"/"False"
    std::string value = it->second;
    std::transform(value.begin(), value.end(), value.begin(), ::tolower);
    can_fd_ = (value == "true");
  }

  it = info.hardware_parameters.find("feedback_wait_us");
  if (it != info.hardware_parameters.end()) {
    try {
      size_t consumed = 0;
      feedback_wait_us_ = std::stoi(it->second, &consumed);
      if (consumed != it->second.size() || feedback_wait_us_ < 0 ||
          feedback_wait_us_ > 2500) {
        throw std::invalid_argument("range");
      }
    } catch (const std::exception&) {
      RCLCPP_ERROR(rclcpp::get_logger("XArm_v10HW"),
                   "feedback_wait_us must be an integer in [0, 2500]");
      return false;
    }
  }
  RCLCPP_INFO(rclcpp::get_logger("XArm_v10HW"),
               "Feedback query window: %d us (before controller update)",
               feedback_wait_us_);

  RCLCPP_INFO(rclcpp::get_logger("XArm_v10HW"),
              "Configuration: CAN=%s, arm_prefix=%s, hand=%s, can_fd=%s",
              can_interface_.c_str(), arm_prefix_.c_str(),
              hand_ ? "enabled" : "disabled", can_fd_ ? "enabled" : "disabled");
  return true;
}

void XArm_v10HW::generate_joint_names() {
  joint_names_.clear();
  // TODO: read from urdf properly and sort in the future.
  // Currently, the joint names are hardcoded for order consistency to align
  // with hardware. Generate arm joint names: xarm_{arm_prefix}joint{N}
  for (size_t i = 1; i <= ARM_DOF; ++i) {
    std::string joint_name =
        "xarm_" + arm_prefix_ + "joint" + std::to_string(i);
    joint_names_.push_back(joint_name);
  }

  // Generate gripper joint name if enabled
  if (hand_) {
    std::string gripper_joint_name = "xarm_" + arm_prefix_ + "finger_joint1";
    joint_names_.push_back(gripper_joint_name);
    RCLCPP_INFO(rclcpp::get_logger("XArm_v10HW"), "Added gripper joint: %s",
                gripper_joint_name.c_str());
  } else {
    RCLCPP_INFO(rclcpp::get_logger("XArm_v10HW"),
                "Gripper joint NOT added because hand_=false");
  }

  RCLCPP_INFO(rclcpp::get_logger("XArm_v10HW"),
              "Generated %zu joint names for arm prefix '%s'",
              joint_names_.size(), arm_prefix_.c_str());
}

hardware_interface::CallbackReturn XArm_v10HW::on_init(
    const hardware_interface::HardwareInfo& info) {
  if (hardware_interface::SystemInterface::on_init(info) !=
      CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }
  // Parse configuration
  if (!parse_config(info)) {
    return CallbackReturn::ERROR;
  }

  // Generate joint names based on arm prefix
  generate_joint_names();

  // Validate joint count (7 arm joints + optional gripper)
  size_t expected_joints = ARM_DOF + (hand_ ? 1 : 0);
  if (joint_names_.size() != expected_joints) {
    RCLCPP_ERROR(rclcpp::get_logger("XArm_v10HW"),
                 "Generated %zu joint names, expected %zu", joint_names_.size(),
                 expected_joints);
    return CallbackReturn::ERROR;
  }

  // Match each CAN channel to the actual URDF joint before opening CAN.
  // Arm channels are always J1..J7 followed by the optional gripper.
  if (info.joints.size() != expected_joints) {
    RCLCPP_ERROR(rclcpp::get_logger("XArm_v10HW"),
                 "Hardware declares %zu joints, expected %zu", info.joints.size(), expected_joints);
    return CallbackReturn::ERROR;
  }
  for (size_t i = 0; i < expected_joints; ++i) {
    if (info.joints[i].name != joint_names_[i]) {
      RCLCPP_ERROR(rclcpp::get_logger("XArm_v10HW"),
                   "Joint channel %zu mismatch: declared '%s', expected '%s'", i,
                   info.joints[i].name.c_str(), joint_names_[i].c_str());
      return CallbackReturn::ERROR;
    }
  }
  const auto urdf_model = urdf::parseURDF(info.original_xml);
  if (!urdf_model) return CallbackReturn::ERROR;
  for (size_t i = 0; i < ARM_DOF; ++i) {
    const auto joint = urdf_model->getJoint(joint_names_[i]);
    if (!joint || joint->type != urdf::Joint::REVOLUTE || joint->mimic || !joint->limits ||
        !std::isfinite(joint->limits->lower) || !std::isfinite(joint->limits->upper) ||
        joint->limits->lower >= joint->limits->upper) {
      RCLCPP_ERROR(rclcpp::get_logger("XArm_v10HW"), "Invalid URDF position limits for %s; refusing CAN initialization", joint_names_[i].c_str());
      return CallbackReturn::ERROR;
    }
    position_limits_[i] = {joint->limits->lower, joint->limits->upper};
    RCLCPP_INFO(rclcpp::get_logger("XArm_v10HW"), "URDF position limits %s: [%.9f, %.9f]", joint_names_[i].c_str(), position_limits_[i].lower, position_limits_[i].upper);
  }
  // Initialize XArm with configurable CAN-FD setting
  RCLCPP_INFO(rclcpp::get_logger("XArm_v10HW"),
              "Initializing XArm on %s with CAN-FD %s...",
              can_interface_.c_str(), can_fd_ ? "enabled" : "disabled");
  if (sdk_handle_) {
    xarm_sdk_destroy(sdk_handle_);
    sdk_handle_ = nullptr;
  }
  if (!sdk_call_ok(
          xarm_sdk_create(can_interface_.c_str(), can_fd_ ? 1 : 0, &sdk_handle_),
          "xarm_sdk_create")) {
    return CallbackReturn::ERROR;
  }

  // Initialize arm motors with V10 defaults
  if (!sdk_call_ok(xarm_sdk_init_arm_motors(
                       sdk_handle_, DEFAULT_MOTOR_TYPES.data(),
                       DEFAULT_SEND_CAN_IDS.data(), DEFAULT_RECV_CAN_IDS.data(),
                       static_cast<int>(DEFAULT_MOTOR_TYPES.size())),
                   "xarm_sdk_init_arm_motors")) {
    return CallbackReturn::ERROR;
  }

  // Initialize gripper if enabled
  if (hand_) {
    RCLCPP_INFO(rclcpp::get_logger("XArm_v10HW"), "Initializing gripper...");
    if (!sdk_call_ok(
            xarm_sdk_init_gripper_motor(sdk_handle_, DEFAULT_GRIPPER_MOTOR_TYPE,
                                        DEFAULT_GRIPPER_SEND_CAN_ID,
                                        DEFAULT_GRIPPER_RECV_CAN_ID),
            "xarm_sdk_init_gripper_motor")) {
      return CallbackReturn::ERROR;
    }
  }

  // Initialize state and command vectors based on generated joint count
  const size_t total_joints = joint_names_.size();
  const double nan_value = std::numeric_limits<double>::quiet_NaN();


  pos_commands_.resize(total_joints, 0.0);
  vel_commands_.resize(total_joints, 0.0);
  tau_commands_.resize(total_joints, 0.0);
  pos_states_.assign(total_joints, nan_value);
  vel_states_.resize(total_joints, 0.0);
  tau_states_.resize(total_joints, 0.0);

  // Hold the measured arm pose until a controller writes its first command.
  // The position-mode fallback replaces these NaNs with the current state on
  // the first write cycle. Keep the gripper command at its explicit default.
  std::fill_n(pos_commands_.begin(), ARM_DOF, nan_value);

  // write() runs before the controller spawners finish, so every joint needs a
  // valid mode from the first real-time cycle onward.
  control_modes_.resize(total_joints, hardware_interface::HW_IF_POSITION);

  // Build KDL gravity compensation solver from URDF
  KDL::Tree kdl_tree;
  if (kdl_parser::treeFromString(info.original_xml, kdl_tree)) {
    std::string root = kdl_tree.getRootSegment()->first;
    // Include the end-effector (hand) rigid body in the gravity chain when
    // present: it is attached via a fixed joint (no extra DOF) but its mass
    // (~0.35kg) is otherwise silently dropped, causing gravity torque to be
    // underestimated, especially for the wrist joints. Verified numerically
    // against Pinocchio: including it brings gravity torque estimates from
    // up to 3x off down to within ~5-12% (residual = gripper finger mass,
    // which depends on gripper open/close state and isn't modeled).
    std::string tip = hand_ ? ("xarm_" + arm_prefix_ + "hand")
                             : ("xarm_" + arm_prefix_ + "link7");
    KDL::Chain chain;
    if (kdl_tree.getChain(root, tip, chain)) {
      arm_chain_ = chain;
      gravity_solver_ = std::make_unique<KDL::ChainDynParam>(
          arm_chain_, KDL::Vector(0.0, 0.0, -9.81));
      RCLCPP_INFO(rclcpp::get_logger("XArm_v10HW"),
                  "Gravity compensation enabled: %s -> %s (%u joints)",
                  root.c_str(), tip.c_str(), arm_chain_.getNrOfJoints());
    } else {
      RCLCPP_WARN(rclcpp::get_logger("XArm_v10HW"),
                  "KDL chain %s->%s not found, gravity compensation disabled",
                  root.c_str(), tip.c_str());
    }
  } else {
    RCLCPP_WARN(rclcpp::get_logger("XArm_v10HW"),
                "Failed to parse URDF, gravity compensation disabled");
  }

  RCLCPP_INFO(rclcpp::get_logger("XArm_v10HW"),
              "XArm V10 Simple HW initialized successfully");

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn XArm_v10HW::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  // Set callback mode to ignore during configuration
  if (!sdk_call_ok(xarm_sdk_refresh_all(sdk_handle_), "xarm_sdk_refresh_all")) {
    return CallbackReturn::ERROR;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  if (!sdk_call_ok(xarm_sdk_recv_all(sdk_handle_, 0), "xarm_sdk_recv_all")) {
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
XArm_v10HW::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        joint_names_[i], hardware_interface::HW_IF_POSITION, &pos_states_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        joint_names_[i], hardware_interface::HW_IF_VELOCITY, &vel_states_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        joint_names_[i], hardware_interface::HW_IF_EFFORT, &tau_states_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
XArm_v10HW::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  // TODO: consider exposing only needed interfaces to avoid undefined behavior.
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        joint_names_[i], hardware_interface::HW_IF_POSITION,
        &pos_commands_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        joint_names_[i], hardware_interface::HW_IF_VELOCITY,
        &vel_commands_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        joint_names_[i], hardware_interface::HW_IF_EFFORT, &tau_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::return_type XArm_v10HW::prepare_command_mode_switch(
    const std::vector<std::string>& start_interfaces,
    const std::vector<std::string>& stop_interfaces) {
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    const auto prefix = joint_names_[i] + "/";
    const bool position = std::find(start_interfaces.begin(), start_interfaces.end(),
        prefix + hardware_interface::HW_IF_POSITION) != start_interfaces.end();
    const bool effort = std::find(start_interfaces.begin(), start_interfaces.end(),
        prefix + hardware_interface::HW_IF_EFFORT) != start_interfaces.end();
    const bool velocity = std::find(start_interfaces.begin(), start_interfaces.end(),
        prefix + hardware_interface::HW_IF_VELOCITY) != start_interfaces.end();
    const bool stopping = std::find(stop_interfaces.begin(), stop_interfaces.end(),
        prefix + hardware_interface::HW_IF_POSITION) != stop_interfaces.end() ||
        std::find(stop_interfaces.begin(), stop_interfaces.end(),
        prefix + hardware_interface::HW_IF_EFFORT) != stop_interfaces.end();
    if (velocity || (position && effort)) return hardware_interface::return_type::ERROR;
    if ((position || effort || stopping) &&
        (!std::isfinite(pos_states_[i]) ||
         (i < ARM_DOF && !within_urdf_bounds(pos_states_[i], position_limits_[i])))) {
      RCLCPP_ERROR(rclcpp::get_logger("XArm_v10HW"),
                   "Cannot switch joint %zu: feedback unavailable or outside configured limits", i);
      return hardware_interface::return_type::ERROR;
    }
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type XArm_v10HW::perform_command_mode_switch(
    const std::vector<std::string>& start_interfaces,
    const std::vector<std::string>& stop_interfaces) {
  // Validate the whole request before changing any joint (no partial switch).
  if (prepare_command_mode_switch(start_interfaces, stop_interfaces) !=
      hardware_interface::return_type::OK) return hardware_interface::return_type::ERROR;
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    const auto prefix = joint_names_[i] + "/";
    const bool position = std::find(start_interfaces.begin(), start_interfaces.end(),
        prefix + hardware_interface::HW_IF_POSITION) != start_interfaces.end();
    const bool effort = std::find(start_interfaces.begin(), start_interfaces.end(),
        prefix + hardware_interface::HW_IF_EFFORT) != start_interfaces.end();
    const bool stopping = std::find(stop_interfaces.begin(), stop_interfaces.end(),
        prefix + hardware_interface::HW_IF_POSITION) != stop_interfaces.end() ||
        std::find(stop_interfaces.begin(), stop_interfaces.end(),
        prefix + hardware_interface::HW_IF_EFFORT) != stop_interfaces.end();
    if (!(position || effort || stopping)) continue;
    const auto mode = effort ? CommandMode::kEffort : CommandMode::kPosition;
    synchronize_commands_for_mode_switch(mode, pos_states_[i], pos_commands_[i],
                                         vel_commands_[i], tau_commands_[i]);
    control_modes_[i] = effort ? hardware_interface::HW_IF_EFFORT : hardware_interface::HW_IF_POSITION;
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::CallbackReturn XArm_v10HW::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_INFO(rclcpp::get_logger("XArm_v10HW"), "Activating XArm V10...");
  if (!sdk_call_ok(xarm_sdk_set_callback_mode_state_all(sdk_handle_),
                   "xarm_sdk_set_callback_mode_state_all")) {
    return CallbackReturn::ERROR;
  }
  if (!sdk_call_ok(xarm_sdk_enable_all(sdk_handle_), "xarm_sdk_enable_all")) {
    return CallbackReturn::ERROR;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  if (!sdk_call_ok(xarm_sdk_recv_all(sdk_handle_, 0), "xarm_sdk_recv_all")) {
    return CallbackReturn::ERROR;
  }

  // Activation never commands a zero trajectory. Seed from feedback instead.
  if (read(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)) !=
      hardware_interface::return_type::OK) {
    xarm_sdk_disable_all(sdk_handle_);
    return CallbackReturn::ERROR;
  }
  for (size_t i = 0; i < pos_states_.size(); ++i) {
    if (!std::isfinite(pos_states_[i]) ||
        (i < ARM_DOF && !within_urdf_bounds(pos_states_[i], position_limits_[i]))) {
      RCLCPP_ERROR(rclcpp::get_logger("XArm_v10HW"),
                   "Activation refused: joint %zu feedback invalid or outside configured limits", i);
      xarm_sdk_disable_all(sdk_handle_);
      return CallbackReturn::ERROR;
    }
    synchronize_commands_for_mode_switch(CommandMode::kPosition, pos_states_[i],
        pos_commands_[i], vel_commands_[i], tau_commands_[i]);
    control_modes_[i] = hardware_interface::HW_IF_POSITION;
  }

  RCLCPP_INFO(rclcpp::get_logger("XArm_v10HW"), "XArm V10 activated");
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn XArm_v10HW::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_INFO(rclcpp::get_logger("XArm_v10HW"),
              "Deactivating XArm V10...");

  // Disable all motors (like full_arm.cpp exit)
  if (!sdk_call_ok(xarm_sdk_disable_all(sdk_handle_), "xarm_sdk_disable_all")) {
    return CallbackReturn::ERROR;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  if (!sdk_call_ok(xarm_sdk_recv_all(sdk_handle_, 0), "xarm_sdk_recv_all")) {
    return CallbackReturn::ERROR;
  }

  RCLCPP_INFO(rclcpp::get_logger("XArm_v10HW"), "XArm V10 deactivated");
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type XArm_v10HW::read(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& period) {
  // Drain old replies, then request this cycle's feedback. SDK recv_all(t)
  // restarts its timeout after EACH packet, so a positive timeout there is
  // not a bounded total wait. Sleep once, then drain nonblocking instead.
  if (!sdk_call_ok(xarm_sdk_recv_all(sdk_handle_, 0), "xarm_sdk_recv_all") ||
      !sdk_call_ok(xarm_sdk_refresh_all(sdk_handle_), "xarm_sdk_refresh_all")) {
    return hardware_interface::return_type::ERROR;
  }
  // Each arm may consume at most 1/4 of the advertised cycle for this wait.
  // At 100 Hz the default is 2.5 ms/arm; no controller-rate change is needed.
  const auto cycle_us = period.nanoseconds() / 1000;
  const auto wait_us = std::min<int64_t>(
      feedback_wait_us_, std::max<int64_t>(0, cycle_us / 4));
  if (wait_us > 0) {
    std::this_thread::sleep_until(std::chrono::steady_clock::now() +
                                  std::chrono::microseconds(wait_us));
  }
  if (!sdk_call_ok(xarm_sdk_recv_all(sdk_handle_, 0), "xarm_sdk_recv_all")) {
    return hardware_interface::return_type::ERROR;
  }

  // Read arm joint states
  if (!sdk_call_ok(xarm_sdk_get_arm_joint_states(
                       sdk_handle_, arm_feedback_.data(),
                       static_cast<int>(arm_feedback_.size())),
                   "xarm_sdk_get_arm_joint_states")) {
    return hardware_interface::return_type::ERROR;
  }
  for (const auto& state : arm_feedback_) {
    if (!std::isfinite(state.pos) || !std::isfinite(state.vel) ||
        !std::isfinite(state.torque)) return hardware_interface::return_type::ERROR;
  }
  for (size_t i = 0; i < ARM_DOF; ++i) {
    pos_states_[i] = arm_feedback_[i].pos;
    vel_states_[i] = arm_feedback_[i].vel;
    tau_states_[i] = arm_feedback_[i].torque;
  }

  // Read gripper state if enabled
  if (hand_ && joint_names_.size() > ARM_DOF) {
    xarm_sdk_joint_state_t gripper_state{};
    if (sdk_call_ok(xarm_sdk_get_gripper_state(sdk_handle_, &gripper_state),
                    "xarm_sdk_get_gripper_state")) {
      pos_states_[ARM_DOF] = motor_radians_to_joint(gripper_state.pos);
      vel_states_[ARM_DOF] = gripper_state.vel;
      tau_states_[ARM_DOF] = gripper_state.torque;
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type XArm_v10HW::write(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  // Never send commands before valid feedback or propagate NaN to a motor.
  for (size_t i = 0; i < ARM_DOF; ++i) {
    if (!std::isfinite(pos_states_[i]) || !std::isfinite(tau_commands_[i]) ||
        !std::isfinite(vel_commands_[i])) return hardware_interface::return_type::ERROR;
  }
  gravity_torques_.data.setZero();
  const bool position_mode = std::any_of(
      control_modes_.begin(), control_modes_.begin() + ARM_DOF,
      [](const std::string& mode) {return mode == hardware_interface::HW_IF_POSITION;});
  if (gravity_solver_ && position_mode) {
    for (size_t i = 0; i < ARM_DOF; ++i) gravity_q_(i) = pos_states_[i];
    if (gravity_solver_->JntToGravity(gravity_q_, gravity_torques_) < 0)
      return hardware_interface::return_type::ERROR;
  }
  for (size_t i = 0; i < ARM_DOF; ++i) {
    if (control_modes_[i] == hardware_interface::HW_IF_EFFORT) {
      // The effort controller owns torque; do not add gravity a second time.
      arm_commands_[i] = {0.0F, 0.0F, 0.0F, 0.0F, static_cast<float>(tau_commands_[i])};
    } else {
      // NaN means no command; an explicit zero is still a legitimate target.
      if (std::isnan(pos_commands_[i])) pos_commands_[i] = pos_states_[i];
      if (!std::isfinite(pos_commands_[i])) return hardware_interface::return_type::ERROR;
      const double position = std::clamp(pos_commands_[i], position_limits_[i].lower, position_limits_[i].upper);
      const double torque = tau_commands_[i] + gravity_torques_(i);
      if (!std::isfinite(torque)) return hardware_interface::return_type::ERROR;
      arm_commands_[i] = {static_cast<float>(position), static_cast<float>(vel_commands_[i]),
          static_cast<float>(DEFAULT_KP[i]), static_cast<float>(DEFAULT_KD[i]), static_cast<float>(torque)};
    }
  }
  if (hand_ && joint_names_.size() > ARM_DOF &&
      (!std::isfinite(pos_commands_[ARM_DOF]) || !std::isfinite(tau_commands_[ARM_DOF])))
    return hardware_interface::return_type::ERROR;
  if (!sdk_call_ok(xarm_sdk_arm_mit_control(sdk_handle_, arm_commands_.data(),
                                            static_cast<int>(arm_commands_.size())),
                   "xarm_sdk_arm_mit_control")) {
    return hardware_interface::return_type::ERROR;
  }
  // Control gripper if enabled
  if (hand_ && joint_names_.size() > ARM_DOF) {
    // TODO the true mappings are unimplemented.
    double motor_command = joint_to_motor_radians(pos_commands_[ARM_DOF]);
    xarm_sdk_mit_param_t gripper_param{static_cast<float>(motor_command), 0.0F,
                                       static_cast<float>(GRIPPER_DEFAULT_KP),
                                       static_cast<float>(GRIPPER_DEFAULT_KD),
                                       0.0F};
    if (!sdk_call_ok(xarm_sdk_gripper_mit_control(sdk_handle_, &gripper_param),
                     "xarm_sdk_gripper_mit_control")) {
      return hardware_interface::return_type::ERROR;
    }
  }
  // Do not wait after sending torque: acquire fresh feedback in read(),
  // immediately before controller update, rather than early in the old cycle.
  if (!sdk_call_ok(xarm_sdk_recv_all(sdk_handle_, 0), "xarm_sdk_recv_all")) {
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

// Gripper mapping helper functions
double XArm_v10HW::joint_to_motor_radians(double joint_value) {
  // Joint 0=closed -> motor 0 rad, Joint 0.044=open -> motor -1.0472 rad
  return (joint_value / GRIPPER_JOINT_0_POSITION) *
         GRIPPER_MOTOR_1_RADIANS;  // Scale from 0-0.044 to 0 to -1.0472
}

double XArm_v10HW::motor_radians_to_joint(double motor_radians) {
  // Motor 0 rad=closed -> joint 0, Motor -1.0472 rad=open -> joint 0.044
  return GRIPPER_JOINT_0_POSITION *
         (motor_radians /
          GRIPPER_MOTOR_1_RADIANS);  // Scale from 0 to -1.0472 to 0-0.044
}

}  // namespace xarm_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(xarm_hardware::XArm_v10HW,
                       hardware_interface::SystemInterface)
