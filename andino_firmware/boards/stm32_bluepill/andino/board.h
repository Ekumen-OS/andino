// BSD 3-Clause License
//
// Copyright (c) 2026, Ekumen Inc.
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

#include <stdint.h>

#include "andino/app/app.h"
#include "andino/bsp/arduino/arduino_core.h"

namespace andino {

/// @brief Board package of the STM32 Blue Pill (STM32F103C8T6).
///
/// See the package of the Arduino Uno for what a board package is. The pins were chosen with the
/// following constraints:
///  - The encoder pins use external interrupts, which are shared by the pins with the same number
///    of every port: the four of them must have different numbers (here, 12 to 15).
///  - The motor driver PWM outputs must be timer outputs. All of them are channels of TIM2, so they
///    share the PWM frequency.
///  - The IMU uses the pins of the default I2C bus (I2C1).
///  - PA11 and PA12 are used by USB, and PA13 and PA14 by the debugger (SWD).
struct Board {
  /// @brief Serial port baud rate. It is not used, because the serial port is the USB CDC one
  /// (which doesn't have a baud rate), but the serial stream still needs one.
  static constexpr unsigned long kSerialBaudRate{57600};

  /// @brief Left encoder channel A pin.
  static constexpr uint8_t kLeftEncoderChannelAGpioPin{PB12};
  /// @brief Left encoder channel B pin.
  static constexpr uint8_t kLeftEncoderChannelBGpioPin{PB13};

  /// @brief Right encoder channel A pin.
  static constexpr uint8_t kRightEncoderChannelAGpioPin{PB14};
  /// @brief Right encoder channel B pin.
  static constexpr uint8_t kRightEncoderChannelBGpioPin{PB15};

  /// @brief Left motor driver backward pin (TIM2_CH1).
  static constexpr uint8_t kLeftMotorBackwardGpioPin{PA0};
  /// @brief Left motor driver forward pin (TIM2_CH2).
  static constexpr uint8_t kLeftMotorForwardGpioPin{PA1};
  /// @brief Left motor driver enable pin.
  /// @note The enable input of the L298N motor driver may be directly jumped to 5V if the board has
  /// a jumper to do so.
  static constexpr uint8_t kLeftMotorEnableGpioPin{PA4};

  /// @brief Right motor driver backward pin (TIM2_CH3).
  static constexpr uint8_t kRightMotorBackwardGpioPin{PA2};
  /// @brief Right motor driver forward pin (TIM2_CH4).
  static constexpr uint8_t kRightMotorForwardGpioPin{PA3};
  /// @brief Right motor driver enable pin.
  /// @note The enable input of the L298N motor driver may be directly jumped to 5V if the board has
  /// a jumper to do so.
  static constexpr uint8_t kRightMotorEnableGpioPin{PA5};

  /// @brief IMU sensor I2C SCL pin (I2C1).
  static constexpr uint8_t kImuI2cSclPin{PB6};
  /// @brief IMU sensor I2C SDA pin (I2C1).
  static constexpr uint8_t kImuI2cSdaPin{PB7};
};

/// @brief Application configuration for this board: auto-stop window [ms], PID rate [Hz] and PID
/// gains {kp, kd, ki, ko}.
static constexpr App::Config kAppConfig{3000, 30, {30, 10, 0, 10}};

}  // namespace andino
