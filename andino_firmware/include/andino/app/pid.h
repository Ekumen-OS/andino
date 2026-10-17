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

#include <stdint.h>

namespace andino {

/// @brief This class provides a simple PID controller implementation.
class Pid {
 public:
  /// @brief Tuning gains.
  struct Gains {
    /// Proportional gain.
    int16_t kp;
    /// Derivative gain.
    int16_t kd;
    /// Integral gain.
    int16_t ki;
    /// Output gain (the output is divided by it, so it must be greater than zero).
    int16_t ko;
  };

  /// @brief Constructs a new PID object.
  ///
  /// @param gains Tuning gains.
  /// @param output_min Output minimum limit.
  /// @param output_max Output maximum limit.
  /// @param rate_hz Rate at which the controller output is computed [Hz]. A rate of zero is
  /// treated as 1 Hz.
  Pid(const Gains& gains, int16_t output_min, int16_t output_max, uint16_t rate_hz)
      : gains_(gains),
        output_min_(output_min),
        output_max_(output_max),
        rate_hz_(rate_hz == 0 ? static_cast<uint16_t>(1) : rate_hz) {
  }

  /// @brief Gets the period at which the controller output must be computed.
  ///
  /// @return Period [ms].
  unsigned long period_ms() const {
    return 1000UL / rate_hz_;
  }

  /// @brief Resets the PID controller.
  ///
  /// @param encoder_count Current encoder value.
  void reset(int32_t encoder_count);

  /// @brief Returns if the PID controller is enabled or not.
  bool enabled() const;

  /// @brief Enables the PID controller.
  void enable();

  /// @brief Disables the PID controller.
  void disable();

  /// @brief Computes a new output.
  ///
  /// @param encoder_count Current encoder value.
  /// @param computed_output Computed output value.
  void compute(int32_t encoder_count, int16_t& computed_output);

  /// @brief Sets the target speed.
  ///
  /// The controller works with the ticks counted in each computation period, so the speed is
  /// converted to ticks per period using integer division. The speed resolution is therefore the
  /// controller rate, in ticks per second.
  ///
  /// @param ticks_per_second Desired speed [ticks/s].
  void set_target_speed(int16_t ticks_per_second);

  /// @brief Sets the tuning gains.
  ///
  /// @param gains Tuning gains.
  void set_tunings(const Gains& gains);

 private:
  /// Tuning gains.
  Gains gains_;

  /// Output minimum limit.
  int16_t output_min_{0};
  /// Output maximum limit.
  int16_t output_max_{0};

  /// Rate at which the controller output is computed [Hz].
  uint16_t rate_hz_{1};

  /// True if the PID is enabled, false otherwise.
  bool enabled_{false};

  /// Setpoint value [ticks/period].
  int16_t setpoint_{0};
  /// Accumulated integral term.
  int16_t integral_term_{0};
  /// Last received encoder value.
  int32_t last_encoder_count_{0};
  /// Last computed input value.
  int16_t last_input_{0};
  /// Last computed output value.
  int32_t last_output_{0};
};

}  // namespace andino
