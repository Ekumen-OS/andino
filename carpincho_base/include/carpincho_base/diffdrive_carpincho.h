// Copyright 2023 Franco Cipollone
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

#include "carpincho_base/motor_driver.h"
#include "carpincho_base/wheel.h"

namespace carpincho_base {

/// @brief Hardware interface for carpincho robot.
/// This class is a hardware interface implementation for the carpincho robot. It is responsible for
/// abstracting away the specifics of the hardware and exposing interfaces that are easy to work with.
class DiffDriveCarpincho : public hardware_interface::SystemInterface {
 public:
  /// @brief Default constructor for the DiffDriveCarpincho class.
  DiffDriveCarpincho() = default;

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

  // Configuration parameters for the DiffDriveCarpincho class.
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
  rclcpp::Logger logger_{rclcpp::get_logger("DiffDriveCarpincho")};
};

}  // namespace carpincho_base
