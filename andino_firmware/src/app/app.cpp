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
#include "andino/app/constants.h"

namespace andino {

void App::setup() {
  serial_stream_.begin(Constants::kBaudrate);

  left_encoder_.begin();
  right_encoder_.begin();

  left_motor_.begin();
  left_motor_.enable(true);
  right_motor_.begin();
  right_motor_.enable(true);

  left_pid_controller_.reset(left_encoder_.read());
  right_pid_controller_.reset(right_encoder_.read());

  // Initialize command shell.
  shell_.set_serial_stream(&serial_stream_);
  shell_.set_default_callback(cmd_unknown_cb, this);
  shell_.register_command(Commands::kReadEncoderChannel, cmd_read_encoder_channel_cb, this);
  shell_.register_command(Commands::kReadEncoders, cmd_read_encoders_cb, this);
  shell_.register_command(Commands::kResetEncoders, cmd_reset_encoders_cb, this);
  shell_.register_command(Commands::kSetMotorsSpeed, cmd_set_motors_speed_cb, this);
  shell_.register_command(Commands::kSetMotorsPwm, cmd_set_motors_pwm_cb, this);
  shell_.register_command(Commands::kSetPidGains, cmd_set_pid_gains_cb, this);
  shell_.register_command(Commands::kIsImuConnected, cmd_is_imu_connected_cb, this);
  shell_.register_command(Commands::kReadEncodersAndImu, cmd_read_encoders_and_imu_cb, this);

  // Initialize IMU sensor.
  is_imu_connected = imu_.begin();
}

void App::loop() {
  // Process command prompt input.
  shell_.process_input();

  // Compute PID output at the configured rate.
  if ((clock_.millis() - last_pid_computation_) > Constants::kPidPeriod) {
    last_pid_computation_ = clock_.millis();
    adjust_motors_speed();
  }

  // Stop the motors if auto-stop interval has been reached.
  if ((clock_.millis() - last_set_motors_speed_cmd_) > Constants::kAutoStopWindow) {
    last_set_motors_speed_cmd_ = clock_.millis();
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

void App::cmd_unknown_cb(void* context, int, char**) {
  App* app = static_cast<App*>(context);
  app->reply_error("Unknown command");
}

void App::cmd_read_encoder_channel_cb(void* context, int argc, char** argv) {
  App* app = static_cast<App*>(context);
  int16_t encoder = 0;
  int16_t channel = 0;
  if (argc != 3 || !parse_int(argv[1], encoder) || !parse_int(argv[2], channel) ||
      (encoder != 0 && encoder != 1) || (channel != 0 && channel != 1)) {
    app->reply_error("Invalid arguments");
    return;
  }

  Encoder& selected_encoder = (encoder == 0) ? app->left_encoder_ : app->right_encoder_;
  const int value =
      (channel == 0) ? selected_encoder.read_channel_a() : selected_encoder.read_channel_b();
  app->serial_stream_.println(value);
}

void App::cmd_read_encoders_cb(void* context, int, char**) {
  App* app = static_cast<App*>(context);
  app->serial_stream_.print(app->left_encoder_.read());
  app->serial_stream_.print(" ");
  app->serial_stream_.println(app->right_encoder_.read());
}

void App::cmd_reset_encoders_cb(void* context, int, char**) {
  App* app = static_cast<App*>(context);
  app->left_encoder_.reset();
  app->right_encoder_.reset();
  app->left_pid_controller_.reset(app->left_encoder_.read());
  app->right_pid_controller_.reset(app->right_encoder_.read());
  app->reply_ok();
}

void App::cmd_set_motors_speed_cb(void* context, int argc, char** argv) {
  App* app = static_cast<App*>(context);
  int16_t left_motor_speed = 0;
  int16_t right_motor_speed = 0;
  if (argc != 3 || !parse_int(argv[1], left_motor_speed) ||
      !parse_int(argv[2], right_motor_speed)) {
    app->reply_error("Invalid arguments");
    return;
  }

  // Reset the auto stop timer.
  app->last_set_motors_speed_cmd_ = app->clock_.millis();
  if (left_motor_speed == 0 && right_motor_speed == 0) {
    app->left_motor_.set_speed(0);
    app->right_motor_.set_speed(0);
    app->left_pid_controller_.reset(app->left_encoder_.read());
    app->right_pid_controller_.reset(app->right_encoder_.read());
    app->left_pid_controller_.disable();
    app->right_pid_controller_.disable();
  } else {
    app->left_pid_controller_.enable();
    app->right_pid_controller_.enable();
  }

  // The target speeds are in ticks per second, so we need to convert them to ticks per
  // Constants::kPidRate.
  app->left_pid_controller_.set_setpoint(
      static_cast<int16_t>(left_motor_speed / Constants::kPidRate));
  app->right_pid_controller_.set_setpoint(
      static_cast<int16_t>(right_motor_speed / Constants::kPidRate));
  app->reply_ok();
}

void App::cmd_set_motors_pwm_cb(void* context, int argc, char** argv) {
  App* app = static_cast<App*>(context);
  int16_t left_motor_pwm = 0;
  int16_t right_motor_pwm = 0;
  if (argc != 3 || !parse_int(argv[1], left_motor_pwm) || !parse_int(argv[2], right_motor_pwm)) {
    app->reply_error("Invalid arguments");
    return;
  }

  app->left_pid_controller_.reset(app->left_encoder_.read());
  app->right_pid_controller_.reset(app->right_encoder_.read());
  // Sneaky way to temporarily disable the PID.
  app->left_pid_controller_.disable();
  app->right_pid_controller_.disable();

  // Reset the auto stop timer.
  app->last_set_motors_speed_cmd_ = app->clock_.millis();

  app->left_motor_.set_speed(left_motor_pwm);
  app->right_motor_.set_speed(right_motor_pwm);
  app->reply_ok();
}

void App::cmd_set_pid_gains_cb(void* context, int argc, char** argv) {
  App* app = static_cast<App*>(context);
  int16_t kp = 0;
  int16_t kd = 0;
  int16_t ki = 0;
  int16_t ko = 0;
  if (argc != 5 || !parse_int(argv[1], kp) || !parse_int(argv[2], kd) || !parse_int(argv[3], ki) ||
      !parse_int(argv[4], ko)) {
    app->reply_error("Invalid arguments");
    return;
  }

  app->left_pid_controller_.set_tunings(kp, kd, ki, ko);
  app->right_pid_controller_.set_tunings(kp, kd, ki, ko);
  app->reply_ok();
}

void App::cmd_is_imu_connected_cb(void* context, int, char**) {
  App* app = static_cast<App*>(context);
  app->serial_stream_.println(app->is_imu_connected);
}

void App::cmd_read_encoders_and_imu_cb(void* context, int, char**) {
  App* app = static_cast<App*>(context);
  if (!app->is_imu_connected) {
    app->reply_error("IMU unavailable");
    return;
  }

  app->serial_stream_.print(app->left_encoder_.read());
  app->serial_stream_.print(" ");
  app->serial_stream_.print(app->right_encoder_.read());
  app->serial_stream_.print(" ");

  // Retrieve absolute orientation (quaternion).
  Imu::Orientation orientation = app->imu_.get_orientation();
  app->serial_stream_.print(orientation.x, 4);
  app->serial_stream_.print(" ");
  app->serial_stream_.print(orientation.y, 4);
  app->serial_stream_.print(" ");
  app->serial_stream_.print(orientation.z, 4);
  app->serial_stream_.print(" ");
  app->serial_stream_.print(orientation.w, 4);
  app->serial_stream_.print(" ");

  // Retrieve angular velocity (rad/s).
  Imu::Vector3 angular_velocity = app->imu_.get_angular_velocity();
  app->serial_stream_.print(angular_velocity.x, 4);
  app->serial_stream_.print(" ");
  app->serial_stream_.print(angular_velocity.y, 4);
  app->serial_stream_.print(" ");
  app->serial_stream_.print(angular_velocity.z, 4);
  app->serial_stream_.print(" ");

  // Retrieve linear acceleration (m/s^2).
  Imu::Vector3 linear_acceleration = app->imu_.get_linear_acceleration();
  app->serial_stream_.print(linear_acceleration.x, 4);
  app->serial_stream_.print(" ");
  app->serial_stream_.print(linear_acceleration.y, 4);
  app->serial_stream_.print(" ");
  app->serial_stream_.print(linear_acceleration.z, 4);
  app->serial_stream_.println("");
}

void App::reply_ok() {
  serial_stream_.println("[OK]");
}

void App::reply_error(const char* reason) {
  serial_stream_.print("[ERROR] ");
  serial_stream_.println(reason);
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
  left_pid_controller_.disable();
  right_pid_controller_.disable();
}

}  // namespace andino
