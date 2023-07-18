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

// TODO(jballoffet): Revisit overall logic and add proper documentation.
class PID {
 public:
  void reset(int encoder_count);
  void compute();
  void update(int encoder_count, int& motor_speed);
  void set_output_limits(int min, int max);
  void set_tunings(int kp, int kd, int ki, int ko);
  void set_state(bool enabled);
  void set_setpoint(int setpoint);

 private:
  int target_ticks_per_frame_;  // target speed in ticks per frame
  long encoder_;                // encoder count
  long prev_enc_;               // last encoder count
  int prev_input_;              // last input
  int i_term_;                  // integrated term
  long output_;                 // last motor setting

  /* PID Parameters */
  int kp_ = 30;
  int kd_ = 10;
  int ki_ = 0;
  int ko_ = 10;

  bool enabled_ = false;  // is the base in motion?

  int output_limit_min_;
  int output_limit_max_;
};

}  // namespace andino
