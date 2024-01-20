// BSD 3-Clause License
//
// Copyright (c) 2024, Ekumen Inc.
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
#include "interrupt_in_arduino.h"

#include <Arduino.h>

/// Holds the attached callbacks.
static andino::InterruptIn::InterruptCallback g_callbacks[3] = {nullptr};

/// Pin change interrupt request 0 interrupt service routine.
ISR(PCINT0_vect) {
  if (g_callbacks[0] != nullptr) {
    g_callbacks[0]();
  }
}

/// Pin change interrupt request 1 interrupt service routine.
ISR(PCINT1_vect) {
  if (g_callbacks[1] != nullptr) {
    g_callbacks[1]();
  }
}

/// Pin change interrupt request 2 interrupt service routine.
ISR(PCINT2_vect) {
  if (g_callbacks[2] != nullptr) {
    g_callbacks[2]();
  }
}

namespace andino {

/// Map between ports and Pin Change Mask registers.
static constexpr volatile uint8_t* kPortToPCMask[]{&PCMSK0, &PCMSK1, &PCMSK2};

void InterruptInArduino::begin() const { pinMode(gpio_pin_, INPUT_PULLUP); }

int InterruptInArduino::read() const { return digitalRead(gpio_pin_); }

void InterruptInArduino::attach(InterruptCallback callback) const {
  uint8_t bit_mask = digitalPinToBitMask(gpio_pin_);
  uint8_t port = digitalPinToPort(gpio_pin_);

  if (port == NOT_A_PORT) {
    return;
  }

  // Ports B, C and D values are 2, 3 and 4 correspondingly.
  port -= 2;

  // Set corresponding bit in the appropriate Pin Change Mask register.
  *(kPortToPCMask[port]) |= bit_mask;

  // Set corresponding bit in the Pin Change Interrupt Control register.
  PCICR |= static_cast<uint8_t>(0x01 << port);

  // Set callback function.
  g_callbacks[port] = callback;
}

}  // namespace andino
