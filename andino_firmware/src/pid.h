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

/// @brief This class provides a simple PID controller implementation.
class PID {
 public:
  /// @brief Constructs a new PID object.
  ///
  /// @param kp Tuning proportional gain.
  /// @param kd Tuning derivative gain.
  /// @param ki Tuning integral gain.
  /// @param ko Tuning output gain.
  /// @param output_min Output minimum limit.
  /// @param output_max Output maximum limit.
  PID(int kp, int kd, int ki, int ko, int output_min, int output_max)
      : kp_(kp),
        kd_(kd),
        ki_(ki),
        ko_(ko),
        output_min_(output_min),
        output_max_(output_max),
        enabled_(false) {}

  /// @brief Resets the PID controller.
  ///
  /// @param encoder_count Current encoder value.
  void reset(int encoder_count);

  /// @brief Enables the PID controller.
  ///
  /// @param enabled True to enable the PID, false otherwise.
  void enable(bool enabled);

  /// @brief Computes a new output.
  ///
  /// @param encoder_count Current encoder value.
  /// @param computed_output Computed output value.
  void compute(int encoder_count, int& computed_output);

  /// @brief Sets the setpoint.
  ///
  /// @param setpoint Desired setpoint value.
  void set_setpoint(int setpoint);

  /// @brief Sets the tuning gains.
  ///
  /// @param kp Tuning proportional gain.
  /// @param kd Tuning derivative gain.
  /// @param ki Tuning integral gain.
  /// @param ko Tuning output gain.
  void set_tunings(int kp, int kd, int ki, int ko);

 private:
  /// Tuning proportional gain.
  int kp_;
  /// Tuning derivative gain.
  int kd_;
  /// Tuning integral gain.
  int ki_;
  /// Tuning output gain.
  int ko_;

  /// Output minimum limit.
  int output_min_;
  /// Output maximum limit.
  int output_max_;

  /// True if the PID is enabled, false otherwise.
  bool enabled_;

  /// Setpoint value.
  int setpoint_;
  /// Accumulated integral term.
  int integral_term_;
  /// Last received encoder value.
  long last_encoder_count_;
  /// Last computed input value.
  int last_input_;
  /// Last computed output value.
  long last_output_;
};

}  // namespace andino
