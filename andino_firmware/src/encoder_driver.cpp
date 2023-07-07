// Code in this file is inspired by:
// https://github.com/hbrobotics/ros_arduino_bridge/blob/indigo-devel/ros_arduino_firmware/src/libraries/ROSArduinoBridge/encoder_driver.ino
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

#include "encoder_driver.h"

#include <stdint.h>

#include "Arduino.h"
#include "commands.h"
#include "hw.h"

volatile long left_enc_pos = 0L;
volatile long right_enc_pos = 0L;
static const int8_t ENC_STATES[] = {0, 1, -1, 0, -1, 0, 0, 1, 1, 0, 0, -1, 0, -1, 1, 0};  // encoder lookup table

/* Interrupt routine for LEFT encoder, taking care of actual counting */
ISR(PCINT2_vect) {
  static uint8_t enc_last = 0;

  enc_last <<= 2;                      // shift previous state two places
  enc_last |= (PIND & (3 << 2)) >> 2;  // read the current state into lowest 2 bits

  left_enc_pos += ENC_STATES[(enc_last & 0x0f)];
}

/* Interrupt routine for RIGHT encoder, taking care of actual counting */
ISR(PCINT1_vect) {
  static uint8_t enc_last = 0;

  enc_last <<= 2;                      // shift previous state two places
  enc_last |= (PINC & (3 << 4)) >> 4;  // read the current state into lowest 2 bits

  right_enc_pos += ENC_STATES[(enc_last & 0x0f)];
}

void initEncoders() {
  // set as inputs
  DDRD &= ~(1 << LEFT_ENCODER_A_GPIO_PIN);
  DDRD &= ~(1 << LEFT_ENCODER_B_GPIO_PIN);
  DDRC &= ~(1 << RIGHT_ENCODER_A_GPIO_PIN);
  DDRC &= ~(1 << RIGHT_ENCODER_B_GPIO_PIN);

  // enable pull up resistors
  PORTD |= (1 << LEFT_ENCODER_A_GPIO_PIN);
  PORTD |= (1 << LEFT_ENCODER_B_GPIO_PIN);
  PORTC |= (1 << RIGHT_ENCODER_A_GPIO_PIN);
  PORTC |= (1 << RIGHT_ENCODER_B_GPIO_PIN);

  // tell pin change mask to listen to left encoder pins
  PCMSK2 |= (1 << LEFT_ENCODER_A_GPIO_PIN) | (1 << LEFT_ENCODER_B_GPIO_PIN);
  // tell pin change mask to listen to right encoder pins
  PCMSK1 |= (1 << RIGHT_ENCODER_A_GPIO_PIN) | (1 << RIGHT_ENCODER_B_GPIO_PIN);

  // enable PCINT1 and PCINT2 interrupt in the general interrupt mask
  PCICR |= (1 << PCIE1) | (1 << PCIE2);
}

/* Wrap the encoder reading function */
long readEncoder(int i) {
  if (i == LEFT)
    return left_enc_pos;
  else
    return right_enc_pos;
}

/* Wrap the encoder reset function */
void resetEncoder(int i) {
  if (i == LEFT) {
    left_enc_pos = 0L;
    return;
  } else {
    right_enc_pos = 0L;
    return;
  }
}

/* Wrap the encoder reset function */
void resetEncoders() {
  resetEncoder(LEFT);
  resetEncoder(RIGHT);
}
