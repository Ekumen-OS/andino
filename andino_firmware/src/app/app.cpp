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

#include <stdio.h>

#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <Arduino.h>
#include <Wire.h>
#include <utility/imumaths.h>

#include "andino/app/commands.h"
#include "andino/app/constants.h"

namespace andino {

void App::setup() {
  // Required by Arduino libraries to work.
  init();

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
  shell_.set_default_callback(cmd_unknown_cb);
  shell_.register_command(Commands::kReadAnalogGpio, cmd_read_analog_gpio_cb, this);
  shell_.register_command(Commands::kReadDigitalGpio, cmd_read_digital_gpio_cb, this);
  shell_.register_command(Commands::kReadEncoders, cmd_read_encoders_cb, this);
  shell_.register_command(Commands::kResetEncoders, cmd_reset_encoders_cb, this);
  shell_.register_command(Commands::kSetMotorsSpeed, cmd_set_motors_speed_cb, this);
  shell_.register_command(Commands::kSetMotorsPwm, cmd_set_motors_pwm_cb, this);
  shell_.register_command(Commands::kSetPidsTuningGains, cmd_set_pid_tuning_gains_cb, this);
  shell_.register_command(Commands::kGetIsImuConnected, cmd_get_is_imu_connected_cb, this);
  shell_.register_command(Commands::kReadEncodersAndImu, cmd_read_encoders_and_imu_cb, this);

  // Initialize IMU sensor.
  if (bno055_imu_.begin()) {
    bno055_imu_.setExtCrystalUse(true);
    is_imu_connected = true;
  }
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

  // Required by Arduino libraries to work.
  if (serialEventRun) {
    serialEventRun();
  }
}

void App::cmd_unknown_cb(void*, int, char**) {
  Serial.println("Unknown command.");
}

void App::cmd_read_analog_gpio_cb(void*, int argc, char** argv) {
  if (argc < 2) {
    return;
  }

  const int pin = atoi(argv[1]);
  Serial.println(analogRead(pin));
}

void App::cmd_read_digital_gpio_cb(void*, int argc, char** argv) {
  if (argc < 2) {
    return;
  }

  const int pin = atoi(argv[1]);
  Serial.println(digitalRead(pin));
}

void App::cmd_read_encoders_cb(void* context, int, char**) {
  App* app = static_cast<App*>(context);
  Serial.print(app->left_encoder_.read());
  Serial.print(" ");
  Serial.println(app->right_encoder_.read());
}

void App::cmd_reset_encoders_cb(void* context, int, char**) {
  App* app = static_cast<App*>(context);
  app->left_encoder_.reset();
  app->right_encoder_.reset();
  app->left_pid_controller_.reset(app->left_encoder_.read());
  app->right_pid_controller_.reset(app->right_encoder_.read());
  Serial.println("OK");
}

void App::cmd_set_motors_speed_cb(void* context, int argc, char** argv) {
  if (argc < 3) {
    return;
  }

  App* app = static_cast<App*>(context);
  const int left_motor_speed = atoi(argv[1]);
  const int right_motor_speed = atoi(argv[2]);

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
  app->left_pid_controller_.set_setpoint(left_motor_speed / Constants::kPidRate);
  app->right_pid_controller_.set_setpoint(right_motor_speed / Constants::kPidRate);
  Serial.println("OK");
}

void App::cmd_set_motors_pwm_cb(void* context, int argc, char** argv) {
  if (argc < 3) {
    return;
  }

  App* app = static_cast<App*>(context);
  const int left_motor_pwm = atoi(argv[1]);
  const int right_motor_pwm = atoi(argv[2]);

  app->left_pid_controller_.reset(app->left_encoder_.read());
  app->right_pid_controller_.reset(app->right_encoder_.read());
  // Sneaky way to temporarily disable the PID.
  app->left_pid_controller_.disable();
  app->right_pid_controller_.disable();

  // Reset the auto stop timer.
  app->last_set_motors_speed_cmd_ = app->clock_.millis();

  app->left_motor_.set_speed(left_motor_pwm);
  app->right_motor_.set_speed(right_motor_pwm);
  Serial.println("OK");
}

void App::cmd_set_pid_tuning_gains_cb(void* context, int argc, char** argv) {
  if (argc < 5) {
    return;
  }

  App* app = static_cast<App*>(context);
  const int kp = atoi(argv[1]);
  const int kd = atoi(argv[2]);
  const int ki = atoi(argv[3]);
  const int ko = atoi(argv[4]);

  // Example: "u 30 20 10 50".
  app->left_pid_controller_.set_tunings(kp, kd, ki, ko);
  app->right_pid_controller_.set_tunings(kp, kd, ki, ko);
  Serial.print("PID Updated: ");
  Serial.print(kp);
  Serial.print(" ");
  Serial.print(kd);
  Serial.print(" ");
  Serial.print(ki);
  Serial.print(" ");
  Serial.println(ko);
  Serial.println("OK");
}

void App::cmd_get_is_imu_connected_cb(void* context, int, char**) {
  App* app = static_cast<App*>(context);
  Serial.println(app->is_imu_connected);
}

void App::cmd_read_encoders_and_imu_cb(void* context, int, char**) {
  App* app = static_cast<App*>(context);
  Serial.print(app->left_encoder_.read());
  Serial.print(" ");
  Serial.print(app->right_encoder_.read());
  Serial.print(" ");

  // Retrieve absolute orientation (quaternion). See
  // https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor/overview for further
  // information.
  imu::Quaternion orientation = app->bno055_imu_.getQuat();
  Serial.print(orientation.x(), 4);
  Serial.print(" ");
  Serial.print(orientation.y(), 4);
  Serial.print(" ");
  Serial.print(orientation.z(), 4);
  Serial.print(" ");
  Serial.print(orientation.w(), 4);
  Serial.print(" ");

  // Retrieve angular velocity (rad/s). See
  // https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor/overview for further
  // information.
  imu::Vector<3> angular_velocity = app->bno055_imu_.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  Serial.print(angular_velocity.x());
  Serial.print(" ");
  Serial.print(angular_velocity.y());
  Serial.print(" ");
  Serial.print(angular_velocity.z());
  Serial.print(" ");

  // Retrieve linear acceleration (m/s^2). See
  // https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor/overview for further
  // information.
  imu::Vector<3> linear_acceleration =
      app->bno055_imu_.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  Serial.print(linear_acceleration.x());
  Serial.print(" ");
  Serial.print(linear_acceleration.y());
  Serial.print(" ");
  Serial.print(linear_acceleration.z());
}

void App::adjust_motors_speed() {
  int left_motor_speed = 0;
  int right_motor_speed = 0;
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
