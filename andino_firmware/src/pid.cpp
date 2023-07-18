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
#include "pid.h"

// TODO(jballoffet): Revisit overall logic and add proper documentation.
namespace andino {

/*
 * Initialize PID variables to zero to prevent startup spikes
 * when turning PID on to start moving
 * In particular, assign both encoder_ and prev_enc_ the current encoder value
 * See http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-initialization/
 * Note that the assumption here is that PID is only turned on
 * when going from stop to moving, that's why we can init everything on zero.
 */
void PID::reset(int encoder_count) {
  target_ticks_per_frame_ = 0.0;
  encoder_ = encoder_count;
  prev_enc_ = encoder_;
  output_ = 0;
  prev_input_ = 0;
  i_term_ = 0;
}

/* PID routine to compute the next motor commands */
void PID::compute() {
  long Perror;
  long output;
  int input;

  // Perror = target_ticks_per_frame_ - (encoder_ - prev_enc_);
  input = encoder_ - prev_enc_;
  Perror = target_ticks_per_frame_ - input;

  /*
   * Avoid derivative kick and allow tuning changes,
   * see
   * http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-derivative-kick/
   * see
   * http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
   */
  // output = (Kp * Perror + Kd * (Perror - PrevErr) + Ki * Ierror) / Ko;
  //  PrevErr = Perror;
  output = (kp_ * Perror - kd_ * (input - prev_input_) + i_term_) / ko_;
  prev_enc_ = encoder_;

  output += output_;
  // Accumulate Integral error *or* Limit output.
  // Stop accumulating when output saturates
  if (output >= output_limit_max_)
    output = output_limit_max_;
  else if (output <= output_limit_min_)
    output = output_limit_min_;
  else
    /*
     * allow turning changes, see
     * http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
     */
    i_term_ += ki_ * Perror;

  output_ = output;
  prev_input_ = input;
}

/* Read the encoder values and call the PID routine */
void PID::update(int encoder_count, int& motor_speed) {
  /* Read the encoders */
  encoder_ = encoder_count;

  /* If we're not moving there is nothing more to do */
  if (!enabled_) {
    /*
     * Reset PIDs once, to prevent startup spikes,
     * see
     * http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-initialization/
     * prev_input_ is considered a good proxy to detect
     * whether reset has already happened
     */
    if (prev_input_ != 0) reset(encoder_count);
    return;
  }

  /* Compute PID update for each motor */
  compute();

  /* Set the motor speeds accordingly */
  motor_speed = output_;
}

void PID::set_output_limits(int min, int max) {
  output_limit_min_ = min;
  output_limit_max_ = max;
}

void PID::set_tunings(int kp, int kd, int ki, int ko) {
  kp_ = kp;
  kd_ = kd;
  ki_ = ki;
  ko_ = ko;
}

void PID::set_state(bool enabled) { enabled_ = enabled; }

void PID::set_setpoint(int setpoint) { target_ticks_per_frame_ = setpoint; }

}  // namespace andino
