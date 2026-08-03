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

namespace andino {

/// @brief Hardware configuration.
struct Hw {
  /// @brief Left encoder channel A pin. Connected to PD2 (digital pin 2).
  static constexpr int kLeftEncoderChannelAGpioPin{2};
  /// @brief Left encoder channel B pin. Connected to PD3 (digital pin 3).
  static constexpr int kLeftEncoderChannelBGpioPin{3};

  /// @brief Right encoder channel A pin. Connected to PC2 (digital pin 16, analog pin A2).
  static constexpr int kRightEncoderChannelAGpioPin{16};
  /// @brief Right encoder channel B pin. Connected to PC3 (digital pin 17, analog pin A3).
  static constexpr int kRightEncoderChannelBGpioPin{17};

  /// @brief Left motor driver backward pin. Connected to PD6 (digital pin 6).
  static constexpr int kLeftMotorBackwardGpioPin{6};
  /// @brief Left motor driver forward pin. Connected to PB2 (digital pin 10).
  static constexpr int kLeftMotorForwardGpioPin{10};
  /// @brief Left motor driver enable pin. Connected to PB5 (digital pin 13).
  /// @note The enable input of the L298N motor driver may be directly jumped to 5V if the board has
  /// a jumper to do so.
  static constexpr int kLeftMotorEnableGpioPin{13};

  /// @brief Right motor driver backward pin. Connected to PD5 (digital pin 5).
  static constexpr int kRightMotorBackwardGpioPin{5};
  /// @brief Right motor driver forward pin. Connected to PB1 (digital pin 9).
  static constexpr int kRightMotorForwardGpioPin{9};
  /// @brief Right motor driver enable pin. Connected to PB4 (digital pin 12).
  /// @note The enable input of the L298N motor driver may be directly jumped to 5V if the board has
  /// a jumper to do so.
  static constexpr int kRightMotorEnableGpioPin{12};

  /// @brief IMU sensor I2C SCL pin. Connected to PC5 (digital pin 19, analog pin A5).
  static constexpr int kImuI2cSclPin{19};
  /// @brief IMU sensor I2C SDA pin. Connected to PC4 (digital pin 18, analog pin A4).
  static constexpr int kImuI2cSdaPin{18};
};

}  // namespace andino
