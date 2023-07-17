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

// TODO(jballoffet): Turn this module into a class.

/* PID setpoint info For a Motor */
typedef struct {
  int TargetTicksPerFrame;  // target speed in ticks per frame
  long Encoder;             // encoder count
  long PrevEnc;             // last encoder count

  /*
   * Using previous input (PrevInput) instead of PrevError to avoid derivative kick,
   * see
   * http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-derivative-kick/
   */
  int PrevInput;  // last input
  // int PrevErr;                   // last error

  /*
   * Using integrated term (ITerm) instead of integrated error (Ierror),
   * to allow tuning changes,
   * see
   * http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
   */
  // int Ierror;
  int ITerm;  // integrated term

  long output;  // last motor setting
} SetPointInfo;

void resetPID(int left_encoder_count, int right_encoder_count);
void doPID(SetPointInfo* p);
void updatePID(int left_encoder_count, int right_encoder_count, int& left_motor_speed,
               int& right_motor_speed);
void set_output_limits(int min, int max);
void set_tunings(int kp, int kd, int ki, int ko);
void set_state(bool enabled);
void set_setpoints(int left_setpoint, int right_setpoint);
