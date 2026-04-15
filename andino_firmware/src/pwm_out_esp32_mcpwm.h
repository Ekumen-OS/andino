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

#if defined(ARDUINO_ARCH_ESP32)

#include "pwm_out.h"

#include "driver/mcpwm.h"

namespace andino {

/// @brief ESP32 implementation of the PWM output interface using MCPWM.
///
/// This maps the logical 0-255 value used in the firmware to a 0-100% duty
/// cycle on the selected MCPWM unit/timer/operator, similar to the example
/// code you provided.
class PwmOutEsp32Mcpwm : public PwmOut {
 public:
  /// @brief Constructs a PwmOutEsp32Mcpwm using the specified GPIO pin,
  /// MCPWM unit and IO signal.
  ///
  /// @param gpio_pin GPIO pin.
  /// @param unit MCPWM unit (e.g. MCPWM_UNIT_0).
  /// @param io_signal MCPWM IO signal (e.g. MCPWM0A, MCPWM0B, ...).
  PwmOutEsp32Mcpwm(const int gpio_pin, mcpwm_unit_t unit, mcpwm_io_signals_t io_signal)
      : PwmOut(gpio_pin), unit_(unit), io_signal_(io_signal) {}

  void begin() const override;

  void write(int value) const override;

 private:
  /// MCPWM unit used by this PWM output.
  const mcpwm_unit_t unit_;

  /// MCPWM IO signal used by this PWM output.
  const mcpwm_io_signals_t io_signal_;
};

}  // namespace andino

#endif  // ARDUINO_ARCH_ESP32
