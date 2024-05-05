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

#include <Adafruit_BNO055.h>

#include "digital_out_arduino.h"
#include "encoder.h"
#include "interrupt_in_arduino.h"
#include "motor.h"
#include "pid.h"
#include "pwm_out_arduino.h"
#include "serial_stream_arduino.h"
#include "shell.h"

namespace andino {

/// @brief This class wraps the MCU main application.
class App {
 public:
  /// This class only contains static members.
  App() = delete;

  /// @brief Configures and sets the application up. Meant to be called once at startup.
  static void setup();

  /// @brief Application main run loop. Meant to be called continously.
  static void loop();

 private:
  /// Computes the PID output and updates the motors speed accordingly.
  static void adjust_motors_speed();

  /// Stops the motors and disables the PID.
  static void stop_motors();

  /// Callback method for an unknown command (default).
  static void cmd_unknown_cb(int argc, char** argv);

  /// Callback method for the `Commands::kReadAnalogGpio` command.
  static void cmd_read_analog_gpio_cb(int argc, char** argv);

  /// Callback method for the `Commands::kReadDigitalGpio` command.
  static void cmd_read_digital_gpio_cb(int argc, char** argv);

  /// Callback method for the `Commands::kReadEncoders` command.
  static void cmd_read_encoders_cb(int argc, char** argv);

  /// Callback method for the `Commands::kResetEncoders` command.
  static void cmd_reset_encoders_cb(int argc, char** argv);

  /// Callback method for the `Commands::kSetMotorsSpeed` command.
  static void cmd_set_motors_speed_cb(int argc, char** argv);

  /// Callback method for the `Commands::kSetMotorsPwm` command.
  static void cmd_set_motors_pwm_cb(int argc, char** argv);

  /// Callback method for the `Commands::kSetPidsTuningGains` command.
  static void cmd_set_pid_tuning_gains_cb(int argc, char** argv);

  /// Callback method for the `Commands::kGetIsImuConnected` command.
  static void cmd_get_is_imu_connected_cb(int argc, char** argv);

  /// Callback method for the `Commands::kReadEncodersAndImu` command.
  static void cmd_read_encoders_and_imu_cb(int argc, char** argv);

  /// Serial stream.
  static SerialStreamArduino serial_stream_;

  /// Application command shell.
  static Shell shell_;

  /// Left wheel motor.
  static DigitalOutArduino left_motor_enable_digital_out_;
  static PwmOutArduino left_motor_forward_pwm_out_;
  static PwmOutArduino left_motor_backward_pwm_out_;
  static Motor left_motor_;

  /// Right wheel motor.
  static DigitalOutArduino right_motor_enable_digital_out_;
  static PwmOutArduino right_motor_forward_pwm_out_;
  static PwmOutArduino right_motor_backward_pwm_out_;
  static Motor right_motor_;

  /// Left wheel encoder.
  static InterruptInArduino left_encoder_channel_a_interrupt_in_;
  static InterruptInArduino left_encoder_channel_b_interrupt_in_;
  static Encoder left_encoder_;

  /// Right wheel encoder.
  static InterruptInArduino right_encoder_channel_a_interrupt_in_;
  static InterruptInArduino right_encoder_channel_b_interrupt_in_;
  static Encoder right_encoder_;

  /// PID controllers (one per wheel).
  static Pid left_pid_controller_;
  static Pid right_pid_controller_;

  /// Adafruit BNO055 IMU sensor.
  static Adafruit_BNO055 bno055_imu_;

  /// Tracks the last time the PID computation was made.
  static unsigned long last_pid_computation_;

  /// Tracks the last time a `Commands::kSetMotorsSpeed` command was received.
  static unsigned long last_set_motors_speed_cmd_;

  /// Tracks whether there is an IMU sensor connected.
  static bool is_imu_connected;
};

}  // namespace andino
