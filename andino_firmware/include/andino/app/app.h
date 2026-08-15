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

#include "andino/app/constants.h"
#include "andino/app/pid.h"
#include "andino/app/shell.h"
#include "andino/drivers/encoder.h"
#include "andino/drivers/imu.h"
#include "andino/drivers/motor.h"
#include "andino/hal/clock.h"
#include "andino/hal/digital_out.h"
#include "andino/hal/interrupt_in.h"
#include "andino/hal/pwm_out.h"
#include "andino/hal/serial_stream.h"

namespace andino {

/// @brief Controller class encapsulating all Andino coordination logic.
class App {
 public:
  /**
   * @brief Constructs a new App instance.
   *
   * @param clock The clock service.
   * @param serial_stream The serial communication stream.
   * @param left_motor_enable The left motor enable digital output.
   * @param left_motor_forward The left motor forward PWM output.
   * @param left_motor_backward The left motor backward PWM output.
   * @param right_motor_enable The right motor enable digital output.
   * @param right_motor_forward The right motor forward PWM output.
   * @param right_motor_backward The right motor backward PWM output.
   * @param left_encoder_a The left encoder channel A interrupt input.
   * @param left_encoder_b The left encoder channel B interrupt input.
   * @param right_encoder_a The right encoder channel A interrupt input.
   * @param right_encoder_b The right encoder channel B interrupt input.
   */
  App(const Clock& clock, SerialStream& serial_stream, DigitalOut& left_motor_enable,
      PwmOut& left_motor_forward, PwmOut& left_motor_backward, DigitalOut& right_motor_enable,
      PwmOut& right_motor_forward, PwmOut& right_motor_backward, InterruptIn& left_encoder_a,
      InterruptIn& left_encoder_b, InterruptIn& right_encoder_a, InterruptIn& right_encoder_b)
      : clock_(clock),
        serial_stream_(serial_stream),
        left_motor_(&left_motor_enable, &left_motor_forward, &left_motor_backward),
        right_motor_(&right_motor_enable, &right_motor_forward, &right_motor_backward),
        left_encoder_(&left_encoder_a, &left_encoder_b),
        right_encoder_(&right_encoder_a, &right_encoder_b),
        left_pid_controller_(Constants::kPidKp, Constants::kPidKd, Constants::kPidKi,
                             Constants::kPidKo, -Constants::kPwmMax, Constants::kPwmMax),
        right_pid_controller_(Constants::kPidKp, Constants::kPidKd, Constants::kPidKi,
                              Constants::kPidKo, -Constants::kPwmMax, Constants::kPwmMax) {
  }

  // Delete copy and move operations to enforce unique reference ownership.
  App(const App&) = delete;
  App& operator=(const App&) = delete;
  App(App&&) = delete;
  App& operator=(App&&) = delete;

  /// @brief Configures and sets the application up. Meant to be called once at startup.
  void setup();

  /// @brief Application main run loop. Meant to be called continuously.
  void loop();

 private:
  /// Callback method for an unknown command (default).
  static void cmd_unknown_cb(void* context, int argc, char** argv);

  /// Callback method for the `Commands::kReadAnalogGpio` command.
  static void cmd_read_analog_gpio_cb(void* context, int argc, char** argv);

  /// Callback method for the `Commands::kReadDigitalGpio` command.
  static void cmd_read_digital_gpio_cb(void* context, int argc, char** argv);

  /// Callback method for the `Commands::kReadEncoders` command.
  static void cmd_read_encoders_cb(void* context, int argc, char** argv);

  /// Callback method for the `Commands::kResetEncoders` command.
  static void cmd_reset_encoders_cb(void* context, int argc, char** argv);

  /// Callback method for the `Commands::kSetMotorsSpeed` command.
  static void cmd_set_motors_speed_cb(void* context, int argc, char** argv);

  /// Callback method for the `Commands::kSetMotorsPwm` command.
  static void cmd_set_motors_pwm_cb(void* context, int argc, char** argv);

  /// Callback method for the `Commands::kSetPidsTuningGains` command.
  static void cmd_set_pid_tuning_gains_cb(void* context, int argc, char** argv);

  /// Callback method for the `Commands::kGetIsImuConnected` command.
  static void cmd_get_is_imu_connected_cb(void* context, int argc, char** argv);

  /// Callback method for the `Commands::kReadEncodersAndImu` command.
  static void cmd_read_encoders_and_imu_cb(void* context, int argc, char** argv);

  /// Computes the PID output and updates the motors speed accordingly.
  void adjust_motors_speed();

  /// Stops the motors and disables the PID.
  void stop_motors();

  const Clock& clock_;

  SerialStream& serial_stream_;

  /// Left wheel motor.
  Motor left_motor_;

  /// Right wheel motor.
  Motor right_motor_;

  /// Left wheel encoder.
  Encoder left_encoder_;

  /// Right wheel encoder.
  Encoder right_encoder_;

  /// IMU sensor.
  Imu imu_;

  /// PID controllers (one per wheel).
  Pid left_pid_controller_;
  Pid right_pid_controller_;

  /// Application command shell.
  Shell shell_;

  /// Tracks the last time the PID computation was made.
  unsigned long last_pid_computation_{0};

  /// Tracks the last time a `Commands::kSetMotorsSpeed` command was received.
  unsigned long last_set_motors_speed_cmd_{0};

  /// Tracks whether there is an IMU sensor connected.
  bool is_imu_connected{false};
};

}  // namespace andino
