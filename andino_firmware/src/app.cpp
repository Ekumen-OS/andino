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

// The arguments converted to integers
long arg1;
long arg2;

namespace andino {

Motor App::left_motor_(LEFT_MOTOR_ENABLE_GPIO_PIN, LEFT_MOTOR_FORWARD_GPIO_PIN,
                       LEFT_MOTOR_BACKWARD_GPIO_PIN);
Motor App::right_motor_(RIGHT_MOTOR_ENABLE_GPIO_PIN, RIGHT_MOTOR_FORWARD_GPIO_PIN,
                        RIGHT_MOTOR_BACKWARD_GPIO_PIN);

Encoder App::left_encoder_(LEFT_ENCODER_A_GPIO_PIN, LEFT_ENCODER_B_GPIO_PIN);
Encoder App::right_encoder_(RIGHT_ENCODER_A_GPIO_PIN, RIGHT_ENCODER_B_GPIO_PIN);

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
  arg1 = 0;
  arg2 = 0;
  arg = 0;
  index = 0;
}

void App::run_command() {
  int i = 0;
  char* p = argv1;
  char* str;
  int pid_args[4];
  arg1 = atoi(argv1);
  arg2 = atoi(argv2);

  switch (cmd) {
    case GET_BAUDRATE:
      Serial.println(Constants::kBaudrate);
      break;
    case ANALOG_READ:
      Serial.println(analogRead(arg1));
      break;
    case DIGITAL_READ:
      Serial.println(digitalRead(arg1));
      break;
    case ANALOG_WRITE:
      analogWrite(arg1, arg2);
      Serial.println("OK");
      break;
    case DIGITAL_WRITE:
      if (arg2 == 0)
        digitalWrite(arg1, LOW);
      else if (arg2 == 1)
        digitalWrite(arg1, HIGH);
      Serial.println("OK");
      break;
    case PIN_MODE:
      if (arg2 == 0)
        pinMode(arg1, INPUT);
      else if (arg2 == 1)
        pinMode(arg1, OUTPUT);
      Serial.println("OK");
      break;
    case READ_ENCODERS:
      Serial.print(left_encoder_.read());
      Serial.print(" ");
      Serial.println(right_encoder_.read());
      break;
    case RESET_ENCODERS:
      left_encoder_.reset();
      right_encoder_.reset();
      left_pid_controller_.reset(left_encoder_.read());
      right_pid_controller_.reset(right_encoder_.read());
      Serial.println("OK");
      break;
    case MOTOR_SPEEDS:
      /* Reset the auto stop timer */
      lastMotorCommand = millis();
      if (arg1 == 0 && arg2 == 0) {
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
      // The target speeds are in ticks per second, so we need to convert them
      // to ticks per PID_INTERVAL
      left_pid_controller_.set_setpoint(arg1 / Constants::kPidRate);
      right_pid_controller_.set_setpoint(arg2 / Constants::kPidRate);
      Serial.println("OK");
      break;
    case MOTOR_RAW_PWM:
      /* Reset the auto stop timer */
      lastMotorCommand = millis();
      left_pid_controller_.reset(left_encoder_.read());
      right_pid_controller_.reset(right_encoder_.read());
      // Sneaky way to temporarily disable the PID
      left_pid_controller_.enable(false);
      right_pid_controller_.enable(false);
      left_motor_.set_speed(arg1);
      right_motor_.set_speed(arg2);
      Serial.println("OK");
      break;
    case UPDATE_PID:
      /* Example: "u 30:20:10:50" */
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
      break;
    default:
      Serial.println("Invalid Command");
      break;
  }
}

}  // namespace andino
