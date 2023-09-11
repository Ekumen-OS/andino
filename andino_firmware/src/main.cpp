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

/* Serial port baud rate */
#define BAUDRATE 57600
/* Maximum PWM signal */
#define MAX_PWM 255

#include "Arduino.h"
#include "hw.h"

/* Include definition of serial commands */
#include "commands.h"

/* Motor driver function definitions */
#include "motor.h"

/* Encoder driver function definitions */
#include "encoder_driver.h"

/* PID parameters and functions */
#include "pid.h"

/* Run the PID loop at 30 times per second */
#define PID_RATE 30  // Hz

/* Convert the rate into an interval in milliseconds */
const int PID_INTERVAL = 1000 / PID_RATE;

/* Track the next time we make a PID calculation */
unsigned long nextPID = PID_INTERVAL;

/* Stop the robot if it hasn't received a movement command
  in this number of milliseconds */
#define AUTO_STOP_INTERVAL 3000
long lastMotorCommand = AUTO_STOP_INTERVAL;

/* Variable initialization */

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

// TODO(jballoffet): Make these objects local to the main function.
andino::Motor left_motor(LEFT_MOTOR_ENABLE_GPIO_PIN, LEFT_MOTOR_FORWARD_GPIO_PIN,
                         LEFT_MOTOR_BACKWARD_GPIO_PIN);
andino::Motor right_motor(RIGHT_MOTOR_ENABLE_GPIO_PIN, RIGHT_MOTOR_FORWARD_GPIO_PIN,
                          RIGHT_MOTOR_BACKWARD_GPIO_PIN);

// TODO(jballoffet): Make these objects local to the main function.
andino::PID left_pid_controller(30, 10, 0, 10, -MAX_PWM, MAX_PWM);
andino::PID right_pid_controller(30, 10, 0, 10, -MAX_PWM, MAX_PWM);

/* Clear the current command parameters */
void resetCommand() {
  cmd = 0;
  memset(argv1, 0, sizeof(argv1));
  memset(argv2, 0, sizeof(argv2));
  arg1 = 0;
  arg2 = 0;
  arg = 0;
  index = 0;
}

/* Run a command.  Commands are defined in commands.h */
int runCommand() {
  int i = 0;
  char* p = argv1;
  char* str;
  int pid_args[4];
  arg1 = atoi(argv1);
  arg2 = atoi(argv2);

  switch (cmd) {
    case GET_BAUDRATE:
      Serial.println(BAUDRATE);
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
      Serial.print(readEncoder(LEFT));
      Serial.print(" ");
      Serial.println(readEncoder(RIGHT));
      break;
    case RESET_ENCODERS:
      resetEncoders();
      left_pid_controller.reset(readEncoder(LEFT));
      right_pid_controller.reset(readEncoder(RIGHT));
      Serial.println("OK");
      break;
    case MOTOR_SPEEDS:
      /* Reset the auto stop timer */
      lastMotorCommand = millis();
      if (arg1 == 0 && arg2 == 0) {
        left_motor.set_speed(0);
        right_motor.set_speed(0);
        left_pid_controller.reset(readEncoder(LEFT));
        right_pid_controller.reset(readEncoder(RIGHT));
        left_pid_controller.enable(false);
        right_pid_controller.enable(false);
      } else {
        left_pid_controller.enable(true);
        right_pid_controller.enable(true);
      }
      // The target speeds are in ticks per second, so we need to convert them
      // to ticks per PID_INTERVAL
      left_pid_controller.set_setpoint(arg1 / PID_RATE);
      right_pid_controller.set_setpoint(arg2 / PID_RATE);
      Serial.println("OK");
      break;
    case MOTOR_RAW_PWM:
      /* Reset the auto stop timer */
      lastMotorCommand = millis();
      left_pid_controller.reset(readEncoder(LEFT));
      right_pid_controller.reset(readEncoder(RIGHT));
      // Sneaky way to temporarily disable the PID
      left_pid_controller.enable(false);
      right_pid_controller.enable(false);
      left_motor.set_speed(arg1);
      right_motor.set_speed(arg2);
      Serial.println("OK");
      break;
    case UPDATE_PID:
      /* Example: "u 30:20:10:50" */
      while ((str = strtok_r(p, ":", &p)) != NULL) {
        pid_args[i] = atoi(str);
        i++;
      }
      left_pid_controller.set_tunings(pid_args[0], pid_args[1], pid_args[2], pid_args[3]);
      right_pid_controller.set_tunings(pid_args[0], pid_args[1], pid_args[2], pid_args[3]);
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

/* Setup function--runs once at startup. */
void setup() {
  Serial.begin(BAUDRATE);

  initEncoders();

  // Enable motors.
  left_motor.set_state(true);
  right_motor.set_state(true);

  left_pid_controller.reset(readEncoder(LEFT));
  right_pid_controller.reset(readEncoder(RIGHT));
}

/* Enter the main loop.  Read and parse input from the serial port
   and run any valid commands. Run a PID calculation at the target
   interval and check for auto-stop conditions.
*/
void loop() {
  while (Serial.available() > 0) {
    // Read the next character
    chr = Serial.read();

    // Terminate a command with a CR
    if (chr == 13) {
      if (arg == 1)
        argv1[index] = 0;
      else if (arg == 2)
        argv2[index] = 0;
      runCommand();
      resetCommand();
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
    left_pid_controller.compute(readEncoder(LEFT), left_motor_speed);
    right_pid_controller.compute(readEncoder(RIGHT), right_motor_speed);
    left_motor.set_speed(left_motor_speed);
    right_motor.set_speed(right_motor_speed);
    nextPID += PID_INTERVAL;
  }

  // Check to see if we have exceeded the auto-stop interval
  if ((millis() - lastMotorCommand) > AUTO_STOP_INTERVAL) {
    lastMotorCommand = millis();
    left_motor.set_speed(0);
    right_motor.set_speed(0);
    left_pid_controller.enable(false);
    right_pid_controller.enable(false);
  }
}
