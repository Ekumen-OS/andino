// Code in this file is inspired by:
// https://github.com/hbrobotics/ros_arduino_bridge/blob/indigo-devel/ros_arduino_firmware/src/libraries/ROSArduinoBridge/ROSArduinoBridge.ino
//
// ----------------------------------------------------------------------------
// ros_arduino_bridge's license follows:
//
// Software License Agreement (BSD License)
//
// Copyright (c) 2012, Patrick Goebel.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//   * Redistributions of source code must retain the above copyright
//     notice, this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above
//     copyright notice, this list of conditions and the following
//     disclaimer in the documentation and/or other materials provided
//     with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

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
#include "andino/app/app.h"

#include <stdint.h>
#include <stdlib.h>

#include "andino/app/commands.h"
#include "andino/app/version.h"

namespace andino {

void App::setup() {
  serial_stream_.begin();

  left_encoder_.begin();
  right_encoder_.begin();

  left_motor_.begin();
  left_motor_.enable(true);
  right_motor_.begin();
  right_motor_.enable(true);

  reset_pids();
  pid_period_ms_ = left_pid_controller_.period_ms();

  // Initialize command shell.
  shell_.set_serial_stream(&serial_stream_);
  shell_.set_default_callback(&trampoline<&App::cmd_unknown>, this);
  shell_.set_overflow_callback(&overflow_trampoline<&App::message_too_long>, this);
  const bool commands_registered =
      register_command<&App::cmd_read_encoder_channel>(Commands::kReadEncoderChannel) &&
      register_command<&App::cmd_read_encoders>(Commands::kReadEncoders) &&
      register_command<&App::cmd_reset_encoders>(Commands::kResetEncoders) &&
      register_command<&App::cmd_set_motors_speed>(Commands::kSetMotorsSpeed) &&
      register_command<&App::cmd_set_motors_pwm>(Commands::kSetMotorsPwm) &&
      register_command<&App::cmd_set_pid_gains>(Commands::kSetPidGains) &&
      register_command<&App::cmd_is_imu_connected>(Commands::kIsImuConnected) &&
      register_command<&App::cmd_read_encoders_and_imu>(Commands::kReadEncodersAndImu) &&
      register_command<&App::cmd_get_version>(Commands::kGetVersion);
  if (!commands_registered) {
    reply_error("Failed to register commands");
  }

  // Initialize IMU sensor.
  is_imu_connected_ = imu_.begin();
}

void App::loop() {
  // Process command prompt input.
  shell_.process_input();

  const unsigned long now = clock_.millis();

  // Compute PID output at the configured rate.
  if ((now - last_pid_computation_) > pid_period_ms_) {
    last_pid_computation_ = now;
    adjust_motors_speed();
  }

  // Stop the motors if auto-stop interval has been reached.
  if ((now - last_set_motors_speed_cmd_) > auto_stop_window_ms_) {
    last_set_motors_speed_cmd_ = now;
    stop_motors();
  }
}

bool App::parse_int(const char* str, int16_t& value) {
  char* end = nullptr;
  const long parsed = strtol(str, &end, 10);
  if (end == str || *end != '\0' || parsed < INT16_MIN || parsed > INT16_MAX) {
    return false;
  }
  value = static_cast<int16_t>(parsed);
  return true;
}

void App::message_too_long() {
  reply_error("Command too long");
}

void App::cmd_unknown() {
  reply_error("Unknown command");
}

void App::cmd_read_encoder_channel(int argc, char** argv) {
  int16_t encoder = 0;
  int16_t channel = 0;
  if (argc != 3 || !parse_int(argv[1], encoder) || !parse_int(argv[2], channel) ||
      (encoder != 0 && encoder != 1) || (channel != 0 && channel != 1)) {
    reply_error("Invalid arguments");
    return;
  }

  Encoder& selected_encoder = (encoder == 0) ? left_encoder_ : right_encoder_;
  const int value =
      (channel == 0) ? selected_encoder.read_channel_a() : selected_encoder.read_channel_b();
  // The serial stream prints fixed-width integers. On some platforms `int` is none of them, which
  // would make the call ambiguous.
  serial_stream_.println(static_cast<int16_t>(value));
}

void App::cmd_read_encoders() {
  serial_stream_.print(left_encoder_.read());
  serial_stream_.print(" ");
  serial_stream_.println(right_encoder_.read());
}

void App::cmd_reset_encoders() {
  left_encoder_.reset();
  right_encoder_.reset();
  reset_pids();
  reply_ok();
}

void App::cmd_set_motors_speed(int argc, char** argv) {
  int16_t left_motor_speed = 0;
  int16_t right_motor_speed = 0;
  if (argc != 3 || !parse_int(argv[1], left_motor_speed) ||
      !parse_int(argv[2], right_motor_speed)) {
    reply_error("Invalid arguments");
    return;
  }

  // Reset the auto stop timer.
  last_set_motors_speed_cmd_ = clock_.millis();
  if (left_motor_speed == 0 && right_motor_speed == 0) {
    left_motor_.set_speed(0);
    right_motor_.set_speed(0);
    reset_pids();
    disable_pids();
  } else {
    left_pid_controller_.enable();
    right_pid_controller_.enable();
  }

  left_pid_controller_.set_target_speed(left_motor_speed);
  right_pid_controller_.set_target_speed(right_motor_speed);
  reply_ok();
}

void App::cmd_set_motors_pwm(int argc, char** argv) {
  int16_t left_motor_pwm = 0;
  int16_t right_motor_pwm = 0;
  if (argc != 3 || !parse_int(argv[1], left_motor_pwm) || !parse_int(argv[2], right_motor_pwm)) {
    reply_error("Invalid arguments");
    return;
  }

  // Driving the motors with raw PWM values bypasses the PID.
  reset_pids();
  disable_pids();

  // Reset the auto stop timer.
  last_set_motors_speed_cmd_ = clock_.millis();

  left_motor_.set_speed(left_motor_pwm);
  right_motor_.set_speed(right_motor_pwm);
  reply_ok();
}

void App::cmd_set_pid_gains(int argc, char** argv) {
  int16_t kp = 0;
  int16_t kd = 0;
  int16_t ki = 0;
  int16_t ko = 0;
  if (argc != 5 || !parse_int(argv[1], kp) || !parse_int(argv[2], kd) || !parse_int(argv[3], ki) ||
      !parse_int(argv[4], ko) || ko <= 0) {
    // The output gain is used as a divisor by the PID, so it must be strictly positive.
    reply_error("Invalid arguments");
    return;
  }

  const Pid::Gains gains{kp, kd, ki, ko};
  left_pid_controller_.set_tunings(gains);
  right_pid_controller_.set_tunings(gains);
  reply_ok();
}

void App::cmd_is_imu_connected() {
  serial_stream_.println(static_cast<int16_t>(is_imu_connected_));
}

void App::cmd_read_encoders_and_imu() {
  if (!is_imu_connected_) {
    reply_error("IMU unavailable");
    return;
  }

  serial_stream_.print(left_encoder_.read());
  serial_stream_.print(" ");
  serial_stream_.print(right_encoder_.read());
  serial_stream_.print(" ");

  // Retrieve absolute orientation (quaternion).
  Imu::Orientation orientation = imu_.get_orientation();
  serial_stream_.print(orientation.x, 4);
  serial_stream_.print(" ");
  serial_stream_.print(orientation.y, 4);
  serial_stream_.print(" ");
  serial_stream_.print(orientation.z, 4);
  serial_stream_.print(" ");
  serial_stream_.print(orientation.w, 4);
  serial_stream_.print(" ");

  // Retrieve angular velocity (rad/s).
  Imu::Vector3 angular_velocity = imu_.get_angular_velocity();
  serial_stream_.print(angular_velocity.x, 4);
  serial_stream_.print(" ");
  serial_stream_.print(angular_velocity.y, 4);
  serial_stream_.print(" ");
  serial_stream_.print(angular_velocity.z, 4);
  serial_stream_.print(" ");

  // Retrieve linear acceleration (m/s^2).
  Imu::Vector3 linear_acceleration = imu_.get_linear_acceleration();
  serial_stream_.print(linear_acceleration.x, 4);
  serial_stream_.print(" ");
  serial_stream_.print(linear_acceleration.y, 4);
  serial_stream_.print(" ");
  serial_stream_.print(linear_acceleration.z, 4);
  serial_stream_.println("");
}

void App::reply_ok() {
  serial_stream_.println("[OK]");
}

void App::reply_error(const char* reason) {
  serial_stream_.print("[ERROR] ");
  serial_stream_.println(reason);
}

void App::cmd_get_version() {
  serial_stream_.println(Version::kFirmware);
}

void App::adjust_motors_speed() {
  int16_t left_motor_speed = 0;
  int16_t right_motor_speed = 0;
  left_pid_controller_.compute(left_encoder_.read(), left_motor_speed);
  right_pid_controller_.compute(right_encoder_.read(), right_motor_speed);
  if (left_pid_controller_.enabled()) {
    left_motor_.set_speed(left_motor_speed);
  }
  if (right_pid_controller_.enabled()) {
    right_motor_.set_speed(right_motor_speed);
  }
}

void App::stop_motors() {
  left_motor_.set_speed(0);
  right_motor_.set_speed(0);
  disable_pids();
}

void App::reset_pids() {
  left_pid_controller_.reset(left_encoder_.read());
  right_pid_controller_.reset(right_encoder_.read());
}

void App::disable_pids() {
  left_pid_controller_.disable();
  right_pid_controller_.disable();
}

}  // namespace andino
