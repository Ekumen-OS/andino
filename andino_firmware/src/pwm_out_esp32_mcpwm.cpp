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
#include "pwm_out_esp32_mcpwm.h"

#if defined(ARDUINO_ARCH_ESP32)

#include <Arduino.h>
#include <constants.h>

#include "driver/mcpwm.h"

namespace andino {

namespace {

mcpwm_timer_t timer_from_signal(mcpwm_io_signals_t signal) {
  switch (signal) {
    case MCPWM0A:
    case MCPWM0B:
      return MCPWM_TIMER_0;
    case MCPWM1A:
    case MCPWM1B:
      return MCPWM_TIMER_1;
    case MCPWM2A:
    case MCPWM2B:
      return MCPWM_TIMER_2;
    default:
      return MCPWM_TIMER_0;
  }
}

mcpwm_operator_t operator_from_signal(mcpwm_io_signals_t signal) {
  switch (signal) {
    case MCPWM0A:
    case MCPWM1A:
    case MCPWM2A:
      return MCPWM_OPR_A;
    case MCPWM0B:
    case MCPWM1B:
    case MCPWM2B:
      return MCPWM_OPR_B;
    default:
      return MCPWM_OPR_A;
  }
}

// Track whether a given (unit, timer) pair has already been initialized so we
// don't call mcpwm_init more than once per timer.
bool initialized[MCPWM_UNIT_MAX][MCPWM_TIMER_MAX] = {{false}};

}  // namespace

void PwmOutEsp32Mcpwm::begin() const {
  const mcpwm_timer_t timer = timer_from_signal(io_signal_);

  // Attach the MCPWM output to the selected pin.
  mcpwm_gpio_init(unit_, io_signal_, gpio_pin_);

  if (!initialized[unit_][timer]) {
    mcpwm_config_t config;
    config.frequency = Constants::kMcpwmFrequency;
    config.cmpr_a = 0.0f;
    config.cmpr_b = 0.0f;
    config.counter_mode = MCPWM_UP_COUNTER;
    config.duty_mode = MCPWM_DUTY_MODE_0;

    mcpwm_init(unit_, timer, &config);
    initialized[unit_][timer] = true;
  }

  // Start with the output stopped.
  write(0);
}

void PwmOutEsp32Mcpwm::write(int value) const {
  if (value < 0) {
    value = 0;
  } else if (value > 255) {
    value = 255;
  }

  const float duty = (static_cast<float>(value) * 100.0f) / 255.0f;
  const mcpwm_timer_t timer = timer_from_signal(io_signal_);
  const mcpwm_operator_t op = operator_from_signal(io_signal_);

  mcpwm_set_duty(unit_, timer, op, duty);
}

}  // namespace andino

#endif  // ARDUINO_ARCH_ESP32
