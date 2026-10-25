// BSD 3-Clause License
//
// Copyright (c) 2023, Ekumen Inc.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice, this
//    list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from
//    this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#include "andino_base/diffdrive_andino.h"

#include <algorithm>
#include <cmath>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>

#include "andino_base/serial_mcu.h"

namespace andino_base {

namespace {

using HardwareParameters = decltype(hardware_interface::HardwareInfo::hardware_parameters);

// Creates a SerialMcu connected to the configured serial device.
std::unique_ptr<Mcu> make_serial_mcu(const DiffDriveAndino::Config& config) {
  return SerialMcu::open(config.serial_device, config.baud_rate, config.timeout);
}

// Reads the string parameter `name` into `value`, which keeps its default if the parameter is not set.
void read_parameter(const HardwareParameters& parameters, const std::string& name, std::string& value,
                    const rclcpp::Logger& logger) {
  const auto it = parameters.find(name);
  if (it == parameters.end()) {
    RCLCPP_WARN(logger, "Parameter '%s' is not set, using default value '%s'.", name.c_str(), value.c_str());
    return;
  }
  value = it->second;
  RCLCPP_DEBUG(logger, "%s: %s", name.c_str(), value.c_str());
}

// Reads the positive integer parameter `name` into `value`, which keeps its default if the parameter is not set.
// Returns false if the parameter is set but is not a positive integer.
bool read_parameter(const HardwareParameters& parameters, const std::string& name, int& value,
                    const rclcpp::Logger& logger) {
  const auto it = parameters.find(name);
  if (it == parameters.end()) {
    RCLCPP_WARN(logger, "Parameter '%s' is not set, using default value '%d'.", name.c_str(), value);
    return true;
  }
  try {
    size_t parsed_chars = 0;
    const int parsed_value = std::stoi(it->second, &parsed_chars);
    if (parsed_chars != it->second.size() || parsed_value <= 0) {
      throw std::invalid_argument(it->second);
    }
    value = parsed_value;
  } catch (const std::exception&) {
    RCLCPP_FATAL(logger, "Parameter '%s' must be a positive integer, got '%s'.", name.c_str(), it->second.c_str());
    return false;
  }
  RCLCPP_DEBUG(logger, "%s: %d", name.c_str(), value);
  return true;
}

// Returns true if `interfaces` holds exactly the interfaces named in `expected_names`, in any order.
bool has_interfaces(const std::vector<hardware_interface::InterfaceInfo>& interfaces,
                    std::vector<std::string> expected_names) {
  std::vector<std::string> names;
  for (const hardware_interface::InterfaceInfo& interface : interfaces) {
    names.push_back(interface.name);
  }
  std::sort(names.begin(), names.end());
  std::sort(expected_names.begin(), expected_names.end());
  return names == expected_names;
}

}  // namespace

DiffDriveAndino::DiffDriveAndino() : DiffDriveAndino(make_serial_mcu) {}

DiffDriveAndino::DiffDriveAndino(McuFactory mcu_factory) : mcu_factory_(std::move(mcu_factory)) {}

hardware_interface::CallbackReturn DiffDriveAndino::on_init(const hardware_interface::HardwareInfo& info) {
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(logger_, "On init...");

  const HardwareParameters& parameters = info_.hardware_parameters;
  read_parameter(parameters, kLeftWheelNameParam, config_.left_wheel_name, logger_);
  read_parameter(parameters, kRightWheelNameParam, config_.right_wheel_name, logger_);
  read_parameter(parameters, kSerialDeviceParam, config_.serial_device, logger_);
  if (!read_parameter(parameters, kBaudRateParam, config_.baud_rate, logger_) ||
      !read_parameter(parameters, kTimeoutParam, config_.timeout, logger_) ||
      !read_parameter(parameters, kEncTicksPerRevParam, config_.enc_ticks_per_rev, logger_)) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  for (const hardware_interface::ComponentInfo& joint : info.joints) {
    // DiffDriveAndino has exactly two states and one command interface on each joint
    if (!has_interfaces(joint.command_interfaces, {hardware_interface::HW_IF_VELOCITY})) {
      RCLCPP_FATAL(logger_, "Joint '%s' must have exactly one '%s' command interface.", joint.name.c_str(),
                   hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (!has_interfaces(joint.state_interfaces,
                        {hardware_interface::HW_IF_POSITION, hardware_interface::HW_IF_VELOCITY})) {
      RCLCPP_FATAL(logger_, "Joint '%s' must have exactly one '%s' and one '%s' state interface.", joint.name.c_str(),
                   hardware_interface::HW_IF_POSITION, hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  // Set up the wheels
  left_wheel_.Setup(config_.left_wheel_name, config_.enc_ticks_per_rev);
  right_wheel_.Setup(config_.right_wheel_name, config_.enc_ticks_per_rev);

  RCLCPP_INFO(logger_, "Finished On init.");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffDriveAndino::on_configure(const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_INFO(logger_, "On configure...");

  // Set up communication with the microcontroller.
  mcu_ = mcu_factory_(config_);
  if (!mcu_) {
    RCLCPP_FATAL(logger_, "Failed to set up the communication with the microcontroller.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(logger_, "Finished Configuration");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffDriveAndino::on_cleanup(const rclcpp_lifecycle::State& /* previous_state */) {
  RCLCPP_INFO(logger_, "On cleanup...");

  // Close the communication with the microcontroller.
  mcu_.reset();

  RCLCPP_INFO(logger_, "Finished Cleanup");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffDriveAndino::on_shutdown(const rclcpp_lifecycle::State& /* previous_state */) {
  RCLCPP_INFO(logger_, "On shutdown...");
  // Shutdown may come straight from the active state, so make sure the robot stops.
  if (is_mcu_connected() && !mcu_->set_motors_speed(0, 0)) {
    RCLCPP_ERROR(logger_, "Failed to stop the motors.");
  }
  mcu_.reset();
  RCLCPP_INFO(logger_, "Finished Shutdown");

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> DiffDriveAndino::export_state_interfaces() {
  // We need to set up a position and a velocity interface for each wheel
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // TODO(francocipollone): Probably we could use the information obtained via info_ variable about the joint name
  // directly.
  state_interfaces.emplace_back(
      hardware_interface::StateInterface(left_wheel_.name_, hardware_interface::HW_IF_VELOCITY, &left_wheel_.vel_));
  state_interfaces.emplace_back(
      hardware_interface::StateInterface(left_wheel_.name_, hardware_interface::HW_IF_POSITION, &left_wheel_.pos_));
  state_interfaces.emplace_back(
      hardware_interface::StateInterface(right_wheel_.name_, hardware_interface::HW_IF_VELOCITY, &right_wheel_.vel_));
  state_interfaces.emplace_back(
      hardware_interface::StateInterface(right_wheel_.name_, hardware_interface::HW_IF_POSITION, &right_wheel_.pos_));

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DiffDriveAndino::export_command_interfaces() {
  // We need to set up a velocity command interface for each wheel

  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(
      hardware_interface::CommandInterface(left_wheel_.name_, hardware_interface::HW_IF_VELOCITY, &left_wheel_.cmd_));
  command_interfaces.emplace_back(
      hardware_interface::CommandInterface(right_wheel_.name_, hardware_interface::HW_IF_VELOCITY, &right_wheel_.cmd_));

  return command_interfaces;
}

hardware_interface::CallbackReturn DiffDriveAndino::on_activate(const rclcpp_lifecycle::State& /* previous_state */) {
  RCLCPP_INFO(logger_, "On activate...");

  if (!is_mcu_connected()) {
    RCLCPP_ERROR(logger_, "MCU is not connected.");
    return hardware_interface::CallbackReturn::ERROR;
  }
  // Start from the current encoder reading so the first read does not report a jump in position.
  const std::optional<Mcu::EncodersData> encoders = mcu_->read_encoders();
  if (!encoders.has_value()) {
    RCLCPP_ERROR(logger_, "Failed to read the encoders.");
    return hardware_interface::CallbackReturn::ERROR;
  }
  for (auto [wheel, enc] : {std::pair{&left_wheel_, (*encoders)[0]}, std::pair{&right_wheel_, (*encoders)[1]}}) {
    wheel->enc_ = enc;
    wheel->pos_ = wheel->Angle();
    wheel->vel_ = 0.;
    wheel->cmd_ = 0.;
  }

  RCLCPP_INFO(logger_, "Finished Activation");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffDriveAndino::on_deactivate(const rclcpp_lifecycle::State& /* previous_state */) {
  RCLCPP_INFO(logger_, "On deactivate...");

  if (!is_mcu_connected() || !mcu_->set_motors_speed(0, 0)) {
    RCLCPP_ERROR(logger_, "Failed to stop the motors.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(logger_, "Finished Deactivation");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type DiffDriveAndino::read(const rclcpp::Time& /* time */, const rclcpp::Duration& period) {
  const double delta_secs = period.seconds();

  if (!is_mcu_connected()) {
    RCLCPP_ERROR(logger_, "MCU is not connected.");
    return hardware_interface::return_type::ERROR;
  }

  const std::optional<Mcu::EncodersData> encoders = mcu_->read_encoders();
  if (!encoders.has_value()) {
    RCLCPP_ERROR(logger_, "Failed to read the encoders.");
    return hardware_interface::return_type::ERROR;
  }

  for (auto [wheel, enc] : {std::pair{&left_wheel_, (*encoders)[0]}, std::pair{&right_wheel_, (*encoders)[1]}}) {
    const double pos_prev = wheel->pos_;
    wheel->enc_ = enc;
    wheel->pos_ = wheel->Angle();
    // Without elapsed time the velocity cannot be estimated, so the previous estimate is kept.
    if (delta_secs > 0.) {
      wheel->vel_ = (wheel->pos_ - pos_prev) / delta_secs;
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type DiffDriveAndino::write(const rclcpp::Time& /* time */,
                                                       const rclcpp::Duration& /* period */) {
  if (!is_mcu_connected()) {
    RCLCPP_ERROR(logger_, "MCU is not connected.");
    return hardware_interface::return_type::ERROR;
  }

  // The command is in rad/sec (rps), we need to convert it to ticks/sec (tps)
  // Using the rads per tick(rpt) of the motor information
  // Formula: ticks/sec = rads/sec / rads/tick

  const int left_value_target = static_cast<int>(std::lround(left_wheel_.cmd_ / left_wheel_.rads_per_tick_));
  const int right_value_target = static_cast<int>(std::lround(right_wheel_.cmd_ / right_wheel_.rads_per_tick_));
  if (!mcu_->set_motors_speed(left_value_target, right_value_target)) {
    RCLCPP_ERROR(logger_, "Failed to set the motors speed.");
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

bool DiffDriveAndino::is_mcu_connected() const { return mcu_ && mcu_->is_connected(); }

}  // namespace andino_base

PLUGINLIB_EXPORT_CLASS(andino_base::DiffDriveAndino, hardware_interface::SystemInterface)
