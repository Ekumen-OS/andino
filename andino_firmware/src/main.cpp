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
#include <Adafruit_BNO055.h>
#include <Wire.h>

#include "andino/app/app.h"
#include "andino/app/hw.h"
#include "andino/bsp/clock_arduino.h"
#include "andino/bsp/digital_out_arduino.h"
#include "andino/bsp/interrupt_in_arduino.h"
#include "andino/bsp/pwm_out_arduino.h"
#include "andino/bsp/serial_stream_arduino.h"

// BSP implementations.
static andino::ClockArduino sys_clock;
static andino::SerialStreamArduino serial_stream;
static andino::DigitalOutArduino left_motor_enable(andino::Hw::kLeftMotorEnableGpioPin);
static andino::PwmOutArduino left_motor_forward(andino::Hw::kLeftMotorForwardGpioPin);
static andino::PwmOutArduino left_motor_backward(andino::Hw::kLeftMotorBackwardGpioPin);
static andino::DigitalOutArduino right_motor_enable(andino::Hw::kRightMotorEnableGpioPin);
static andino::PwmOutArduino right_motor_forward(andino::Hw::kRightMotorForwardGpioPin);
static andino::PwmOutArduino right_motor_backward(andino::Hw::kRightMotorBackwardGpioPin);
static andino::InterruptInArduino left_encoder_a(andino::Hw::kLeftEncoderChannelAGpioPin);
static andino::InterruptInArduino left_encoder_b(andino::Hw::kLeftEncoderChannelBGpioPin);
static andino::InterruptInArduino right_encoder_a(andino::Hw::kRightEncoderChannelAGpioPin);
static andino::InterruptInArduino right_encoder_b(andino::Hw::kRightEncoderChannelBGpioPin);
static Adafruit_BNO055 bno055_imu(55, BNO055_ADDRESS_A, &Wire);

// Main application.
static andino::App app(sys_clock, serial_stream, left_motor_enable, left_motor_forward,
                       left_motor_backward, right_motor_enable, right_motor_forward,
                       right_motor_backward, left_encoder_a, left_encoder_b, right_encoder_a,
                       right_encoder_b, bno055_imu);

/// @brief Application entry point.
///
/// @return Execution final status (never reached).
int main(void) {
  // Application configuration.
  app.setup();

  // Application main run loop.
  while (1) {
    app.loop();
  }

  return 0;
}
