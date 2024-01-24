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

#include "encoder.h"
#include "motor.h"
#include "pid.h"

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
  /// Clears the current command parameters.
  // TODO(jballoffet): Move this method to a different module.
  static void reset_command();

  /// Runs a command.
  // TODO(jballoffet): Move this method to a different module.
  static void run_command();

  /// Callback method for an unknown command (default).
  // TODO(jballoffet): Parse arguments within callback method.
  static void cmd_unknown(const char* arg1, const char* arg2);

  /// Callback method for the `Commands::kReadAnalogGpio` command.
  // TODO(jballoffet): Parse arguments within callback method.
  static void cmd_read_analog_gpio(const char* arg1, const char* arg2);

  /// Callback method for the `Commands::kReadDigitalGpio` command.
  // TODO(jballoffet): Parse arguments within callback method.
  static void cmd_read_digital_gpio(const char* arg1, const char* arg2);

  /// Callback method for the `Commands::kReadEncoders` command.
  // TODO(jballoffet): Parse arguments within callback method.
  static void cmd_read_encoders(const char* arg1, const char* arg2);

  /// Callback method for the `Commands::kResetEncoders` command.
  // TODO(jballoffet): Parse arguments within callback method.
  static void cmd_reset_encoders(const char* arg1, const char* arg2);

  /// Callback method for the `Commands::kSetMotorsSpeed` command.
  // TODO(jballoffet): Parse arguments within callback method.
  static void cmd_set_motors_speed(const char* arg1, const char* arg2);

  /// Callback method for the `Commands::kSetMotorsPwm` command.
  // TODO(jballoffet): Parse arguments within callback method.
  static void cmd_set_motors_pwm(const char* arg1, const char* arg2);

  /// Callback method for the `Commands::kSetPidsTuningGains` command.
  // TODO(jballoffet): Parse arguments within callback method.
  static void cmd_set_pid_tuning_gains(const char* arg1, const char* arg2);

  /// Motors (one per wheel).
  static Motor left_motor_;
  static Motor right_motor_;

  /// Encoders (one per wheel).
  static Encoder left_encoder_;
  static Encoder right_encoder_;

  /// PID controllers (one per wheel).
  static PID left_pid_controller_;
  static PID right_pid_controller_;
};

}  // namespace andino
