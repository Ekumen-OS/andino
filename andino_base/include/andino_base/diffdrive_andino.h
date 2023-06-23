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
#pragma once

#include <string>
#include <vector>

#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include "andino_base/motor_driver.h"
#include "andino_base/wheel.h"

namespace andino_base {

/// @brief Hardware interface for andino robot.
/// This class is a hardware interface implementation for the andino robot. It is responsible for
/// abstracting away the specifics of the hardware and exposing interfaces that are easy to work with.
class DiffDriveAndino : public hardware_interface::SystemInterface {
 public:
  /// @brief Default constructor for the DiffDriveAndino class.
  DiffDriveAndino() = default;

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;

  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

 private:
  const std::string kLeftWheelNameParam{"left_wheel_name"};
  const std::string kRightWheelNameParam{"right_wheel_name"};
  const std::string kSerialDeviceParam{"serial_device"};
  const std::string kBaudRateParam{"baud_rate"};
  const std::string kTimeoutParam{"timeout"};
  const std::string kEncTicksPerRevParam{"enc_ticks_per_rev"};

  // Configuration parameters for the DiffDriveAndino class.
  struct Config {
    // Name of the left and right wheels.
    std::string left_wheel_name = "left_wheel";
    std::string right_wheel_name = "right_wheel";
    // Encoder parameters.
    int enc_ticks_per_rev = 700;
    // Communication parameters.
    std::string serial_device = "/dev/ttyUSB0";
    int baud_rate = 57600;
    int timeout = 1000;
  };

  // Configuration parameters.
  Config config_;
  // Communication with the firmware in charge of controlling the motors.
  MotorDriver motor_driver_;
  // Left wheel of the robot.
  Wheel left_wheel_;
  // Right wheel of the robot.
  Wheel right_wheel_;
  // Logger.
  rclcpp::Logger logger_{rclcpp::get_logger("DiffDriveAndino")};
};

}  // namespace andino_base
