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
#pragma once

#include <stdio.h>

#include "serial_stream.h"

namespace andino {

/// @brief This class provides an Arduino implementation of the serial stream interface.
class SerialStreamArduino : public SerialStream {
 public:
  /// @brief Constructs a SerialStreamArduino.
  explicit SerialStreamArduino() : SerialStream() {}

  void begin(unsigned long baud) const override;

  int available() const override;

  int read() const override;

  size_t print(const char* c) const override;

  size_t print(char c) const override;

  size_t print(unsigned char b, int base) const override;

  size_t print(int num, int base) const override;

  size_t print(unsigned int num, int base) const override;

  size_t print(long num, int base) const override;

  size_t print(unsigned long num, int base) const override;

  size_t print(double num, int digits) const override;

  size_t println(const char* c) const override;

  size_t println(char c) const override;

  size_t println(unsigned char b, int base) const override;

  size_t println(int num, int base) const override;

  size_t println(unsigned int num, int base) const override;

  size_t println(long num, int base) const override;

  size_t println(unsigned long num, int base) const override;

  size_t println(double num, int digits) const override;
};

}  // namespace andino
