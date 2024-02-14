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
#include "serial_stream_arduino.h"

#include <stdio.h>

#include <Arduino.h>

namespace andino {

void SerialStreamArduino::begin(unsigned long baud) const { Serial.begin(baud); }

int SerialStreamArduino::available() const { return Serial.available(); }

int SerialStreamArduino::read() const { return Serial.read(); }

size_t SerialStreamArduino::print(const char* c) const { return Serial.print(c); }

size_t SerialStreamArduino::print(char c) const { return Serial.print(c); }

size_t SerialStreamArduino::print(unsigned char b, int base) const { return Serial.print(b, base); }

size_t SerialStreamArduino::print(int num, int base) const { return Serial.print(num, base); }

size_t SerialStreamArduino::print(unsigned int num, int base) const {
  return Serial.print(num, base);
}

size_t SerialStreamArduino::print(long num, int base) const { return Serial.print(num, base); }

size_t SerialStreamArduino::print(unsigned long num, int base) const {
  return Serial.print(num, base);
}

size_t SerialStreamArduino::print(double num, int digits) const {
  return Serial.print(num, digits);
}

size_t SerialStreamArduino::println(const char* c) const { return Serial.println(c); }

size_t SerialStreamArduino::println(char c) const { return Serial.println(c); }

size_t SerialStreamArduino::println(unsigned char b, int base) const {
  return Serial.println(b, base);
}

size_t SerialStreamArduino::println(int num, int base) const { return Serial.println(num, base); }

size_t SerialStreamArduino::println(unsigned int num, int base) const {
  return Serial.println(num, base);
}

size_t SerialStreamArduino::println(long num, int base) const { return Serial.println(num, base); }

size_t SerialStreamArduino::println(unsigned long num, int base) const {
  return Serial.println(num, base);
}

size_t SerialStreamArduino::println(double num, int digits) const {
  return Serial.println(num, digits);
}

}  // namespace andino
