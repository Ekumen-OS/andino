// BSD 3-Clause License
//
// Copyright (c) 2026, Ekumen Inc.
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

#include <array>

namespace andino_base {

/// \brief Interface to the microcontroller running andino_firmware.
/// It exposes the full set of commands supported by andino_firmware.
class Mcu {
 public:
  /// @brief Holds encoder sensors reading. First value is the left encoder, second is the right one.
  using EncodersData = std::array<int, 2>;

  /// @brief Holds IMU sensor reading.
  struct ImuData {
    /// @brief Absolute orientation as a quaternion, in (x, y, z, w) order.
    std::array<double, 4> orientation{};
    /// @brief Angular velocity [rad/s], in (x, y, z) order.
    std::array<double, 3> angular_velocity{};
    /// @brief Linear acceleration [m/s^2], in (x, y, z) order.
    std::array<double, 3> linear_acceleration{};
  };

  /// @brief Holds encoders and IMU sensors reading.
  struct EncodersAndImuData {
    /// @brief The encoder values.
    EncodersData encoders_data{};
    /// @brief The IMU sensor reading.
    ImuData imu_data{};
  };

  virtual ~Mcu() = default;

  /// @brief Checks if the microcontroller is connected.
  /// @return True if the microcontroller is connected, false otherwise.
  virtual bool is_connected() const = 0;

  /// @brief Resets the encoder sensors.
  virtual void reset_encoders() = 0;

  /// @brief Returns the encoder sensors data.
  /// @returns The encoder sensors data.
  virtual EncodersData read_encoders() = 0;

  /// @brief Checks if there is an IMU sensor available.
  /// @returns True if there is an IMU sensor available, false otherwise.
  virtual bool is_imu_available() = 0;

  /// @brief Returns the encoder sensors and IMU sensor data.
  /// @returns The encoder sensors and IMU sensor data.
  virtual EncodersAndImuData read_encoders_and_imu() = 0;

  /// @brief Sets the motors speed [ticks/s].
  /// @param left_motor_speed Speed value for the left motor.
  /// @param right_motor_speed Speed value for the right motor.
  virtual void set_motors_speed(int left_motor_speed, int right_motor_speed) = 0;

  /// @brief Sets the motors PWM [duty range: 0-255].
  /// @param left_motor_pwm PWM value for the left motor.
  /// @param right_motor_pwm PWM value for the right motor.
  virtual void set_motors_pwm(int left_motor_pwm, int right_motor_pwm) = 0;

  /// @brief Sets the PID tuning gains.
  /// @param kp Proportional gain.
  /// @param kd Derivative gain.
  /// @param ki Integral gain.
  /// @param ko Offset gain.
  virtual void set_pid_tuning_gains(float kp, float kd, float ki, float ko) = 0;
};

}  // namespace andino_base
