// Code in this file is inspired by:
// https://github.com/hbrobotics/ros_arduino_bridge/blob/indigo-devel/ros_arduino_firmware/src/libraries/ROSArduinoBridge/diff_controller.h
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
#include "andino/app/pid.h"

namespace andino {

// Note: see the following links for more information regarding this class implementation:
//  - http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-initialization/
//  - http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-derivative-kick/
//  - http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/

void Pid::reset(int32_t encoder_count) {
  // Since we can assume that the PID is only turned on when going from stop to moving, we can init
  // everything on zero.
  setpoint_ = 0;
  integral_term_ = 0;
  last_encoder_count_ = encoder_count;
  last_input_ = 0;
  last_output_ = 0;
}

/// @brief Enable PID
void Pid::enable() {
  enabled_ = true;
}

/// @brief Is the PID controller enabled?
bool Pid::enabled() {
  return enabled_;
}

/// @brief Disable PID
void Pid::disable() {
  enabled_ = false;
}

void Pid::compute(int32_t encoder_count, int16_t& computed_output) {
  if (!enabled_) {
    // Reset PID once to prevent startup spikes.
    if (last_input_ != 0) {
      reset(encoder_count);
    }
    return;
  }

  // The tick count itself is unbounded, but the delta between two consecutive PID cycles is
  // always small (a few dozen ticks at most), so it safely fits back into an int16_t.
  const int16_t input = static_cast<int16_t>(encoder_count - last_encoder_count_);
  const int32_t error = setpoint_ - input;

  // Products are computed in 32 bits explicitly so that the result doesn't depend on the platform
  // `int` width (16 bits on AVR).
  const int32_t input_delta = static_cast<int32_t>(input) - last_input_;
  int32_t output = (kp_ * error - static_cast<int32_t>(kd_) * input_delta + integral_term_) / ko_;
  output += last_output_;

  // Accumulate integral term as long as output doesn't saturate.
  if (output >= output_max_) {
    output = output_max_;
  } else if (output <= output_min_) {
    output = output_min_;
  } else {
    // ki_ * error safely fits into an int16_t: it is only accumulated while output (which is
    // derived from the same magnitudes) is within the int16_t-sized output_min_/output_max_
    // bounds.
    integral_term_ = static_cast<int16_t>(integral_term_ + ki_ * error);
  }

  // output is clamped to output_min_/output_max_ above, both of which are int16_t, so this
  // narrowing is safe.
  computed_output = static_cast<int16_t>(output);

  // Store obtained values.
  last_encoder_count_ = encoder_count;
  last_input_ = input;
  last_output_ = output;
}

void Pid::set_setpoint(int16_t setpoint) {
  setpoint_ = setpoint;
}

void Pid::set_tunings(int16_t kp, int16_t kd, int16_t ki, int16_t ko) {
  kp_ = kp;
  kd_ = kd;
  ki_ = ki;
  ko_ = ko;
}

}  // namespace andino
