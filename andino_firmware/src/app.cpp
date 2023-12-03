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
#include "app.h"

#include "Arduino.h"
#include "commands.h"
#include "constants.h"
#include "encoder.h"
#include "hw.h"
#include "motor.h"
#include "pid.h"

// TODO(jballoffet): Move this variables to a different module.

/* Track the next time we make a PID calculation */
unsigned long nextPID = andino::Constants::kPidPeriod;

long lastMotorCommand = andino::Constants::kAutoStopWindow;

// A pair of varibles to help parse serial commands
int arg = 0;
int index = 0;

// Variable to hold an input character
char chr;

// Variable to hold the current single-character command
char cmd;

// Character arrays to hold the first and second arguments
char argv1[16];
char argv2[16];

namespace andino {

Motor App::left_motor_(Hw::kLeftMotorEnableGpioPin, Hw::kLeftMotorForwardGpioPin,
                       Hw::kLeftMotorBackwardGpioPin);
Motor App::right_motor_(Hw::kRightMotorEnableGpioPin, Hw::kRightMotorForwardGpioPin,
                        Hw::kRightMotorBackwardGpioPin);

Encoder App::left_encoder_(Hw::kLeftEncoderChannelAGpioPin, Hw::kLeftEncoderChannelBGpioPin);
Encoder App::right_encoder_(Hw::kRightEncoderChannelAGpioPin, Hw::kRightEncoderChannelBGpioPin);

PID App::left_pid_controller_(Constants::kPidKp, Constants::kPidKd, Constants::kPidKi,
                              Constants::kPidKo, -Constants::kPwmMax, Constants::kPwmMax);
PID App::right_pid_controller_(Constants::kPidKp, Constants::kPidKd, Constants::kPidKi,
                               Constants::kPidKo, -Constants::kPwmMax, Constants::kPwmMax);

void App::setup() {
  // Required by Arduino libraries to work.
  init();

  Serial.begin(Constants::kBaudrate);

  left_encoder_.init();
  right_encoder_.init();

  // Enable motors.
  left_motor_.set_state(true);
  right_motor_.set_state(true);

  left_pid_controller_.reset(left_encoder_.read());
  right_pid_controller_.reset(right_encoder_.read());
}

void App::loop() {
  while (Serial.available() > 0) {
    // Read the next character
    chr = Serial.read();

    // Terminate a command with a CR
    if (chr == 13) {
      if (arg == 1)
        argv1[index] = 0;
      else if (arg == 2)
        argv2[index] = 0;
      run_command();
      reset_command();
    }
    // Use spaces to delimit parts of the command
    else if (chr == ' ') {
      // Step through the arguments
      if (arg == 0)
        arg = 1;
      else if (arg == 1) {
        argv1[index] = 0;
        arg = 2;
        index = 0;
      }
      continue;
    } else {
      if (arg == 0) {
        // The first arg is the single-letter command
        cmd = chr;
      } else if (arg == 1) {
        // Subsequent arguments can be more than one character
        argv1[index] = chr;
        index++;
      } else if (arg == 2) {
        argv2[index] = chr;
        index++;
      }
    }
  }

  // Run a PID calculation at the appropriate intervals
  if (millis() > nextPID) {
    int left_motor_speed = 0;
    int right_motor_speed = 0;
    left_pid_controller_.compute(left_encoder_.read(), left_motor_speed);
    right_pid_controller_.compute(right_encoder_.read(), right_motor_speed);
    left_motor_.set_speed(left_motor_speed);
    right_motor_.set_speed(right_motor_speed);
    nextPID += Constants::kPidPeriod;
  }

  // Check to see if we have exceeded the auto-stop interval
  if ((millis() - lastMotorCommand) > Constants::kAutoStopWindow) {
    lastMotorCommand = millis();
    left_motor_.set_speed(0);
    right_motor_.set_speed(0);
    left_pid_controller_.enable(false);
    right_pid_controller_.enable(false);
  }

  // Required by Arduino libraries to work.
  if (serialEventRun) {
    serialEventRun();
  }
}

void App::reset_command() {
  cmd = 0;
  memset(argv1, 0, sizeof(argv1));
  memset(argv2, 0, sizeof(argv2));
  arg = 0;
  index = 0;
}

void App::run_command() {
  switch (cmd) {
    case Commands::kreadAnalogGpio:
      cmd_read_analog_gpio(argv1, argv2);
      break;
    case Commands::kreadDigitalGpio:
      cmd_read_digital_gpio(argv1, argv2);
      break;
    case Commands::kreadEncoders:
      cmd_read_encoders(argv1, argv2);
      break;
    case Commands::kresetEncoders:
      cmd_reset_encoders(argv1, argv2);
      break;
    case Commands::ksetMotorsSpeed:
      cmd_set_motors_speed(argv1, argv2);
      break;
    case Commands::ksetMotorsPwm:
      cmd_set_motors_pwm(argv1, argv2);
      break;
    case Commands::ksetPidsTuningGains:
      cmd_set_pid_tuning_gains(argv1, argv2);
      break;
    default:
      cmd_unknown(argv1, argv2);
      break;
  }
}

void App::cmd_unknown(const char* arg1, const char* arg2) {
  (void)arg1;
  (void)arg2;
  Serial.println("Unknown command.");
}

void App::cmd_read_analog_gpio(const char* arg1, const char* arg2) {
  (void)arg2;
  int pin = atoi(arg1);
  Serial.println(analogRead(pin));
}

void App::cmd_read_digital_gpio(const char* arg1, const char* arg2) {
  (void)arg2;
  int pin = atoi(arg1);
  Serial.println(digitalRead(pin));
}

void App::cmd_read_encoders(const char* arg1, const char* arg2) {
  (void)arg1;
  (void)arg2;
  Serial.print(left_encoder_.read());
  Serial.print(" ");
  Serial.println(right_encoder_.read());
}

void App::cmd_reset_encoders(const char* arg1, const char* arg2) {
  (void)arg1;
  (void)arg2;
  left_encoder_.reset();
  right_encoder_.reset();
  left_pid_controller_.reset(left_encoder_.read());
  right_pid_controller_.reset(right_encoder_.read());
  Serial.println("OK");
}

void App::cmd_set_motors_speed(const char* arg1, const char* arg2) {
  int left_motor_speed = atoi(arg1);
  int right_motor_speed = atoi(arg2);

  // Reset the auto stop timer.
  lastMotorCommand = millis();
  if (left_motor_speed == 0 && right_motor_speed == 0) {
    left_motor_.set_speed(0);
    right_motor_.set_speed(0);
    left_pid_controller_.reset(left_encoder_.read());
    right_pid_controller_.reset(right_encoder_.read());
    left_pid_controller_.enable(false);
    right_pid_controller_.enable(false);
  } else {
    left_pid_controller_.enable(true);
    right_pid_controller_.enable(true);
  }

  // The target speeds are in ticks per second, so we need to convert them to ticks per
  // Constants::kPidRate.
  left_pid_controller_.set_setpoint(left_motor_speed / Constants::kPidRate);
  right_pid_controller_.set_setpoint(right_motor_speed / Constants::kPidRate);
  Serial.println("OK");
}

void App::cmd_set_motors_pwm(const char* arg1, const char* arg2) {
  int left_motor_pwm = atoi(arg1);
  int right_motor_pwm = atoi(arg2);

  // Reset the auto stop timer.
  lastMotorCommand = millis();
  left_pid_controller_.reset(left_encoder_.read());
  right_pid_controller_.reset(right_encoder_.read());
  // Sneaky way to temporarily disable the PID.
  left_pid_controller_.enable(false);
  right_pid_controller_.enable(false);
  left_motor_.set_speed(left_motor_pwm);
  right_motor_.set_speed(right_motor_pwm);
  Serial.println("OK");
}

void App::cmd_set_pid_tuning_gains(const char* arg1, const char* arg2) {
  (void)arg2;

  int i = 0;
  char arg[20];
  char* str;
  int pid_args[4];

  // Example: "u 30:20:10:50".
  strcpy(arg, arg1);
  char* p = arg;
  while ((str = strtok_r(p, ":", &p)) != NULL) {
    pid_args[i] = atoi(str);
    i++;
  }
  left_pid_controller_.set_tunings(pid_args[0], pid_args[1], pid_args[2], pid_args[3]);
  right_pid_controller_.set_tunings(pid_args[0], pid_args[1], pid_args[2], pid_args[3]);
  Serial.print("PID Updated: ");
  Serial.print(pid_args[0]);
  Serial.print(" ");
  Serial.print(pid_args[1]);
  Serial.print(" ");
  Serial.print(pid_args[2]);
  Serial.print(" ");
  Serial.println(pid_args[3]);
  Serial.println("OK");
}

}  // namespace andino
