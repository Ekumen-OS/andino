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

#include <stdint.h>
#include <stdio.h>

namespace andino {

/// @brief This class defines an interface for serial streams.
class SerialStream {
 public:
  /// @brief Constructs a SerialStream.
  explicit SerialStream() = default;

  /// @brief Destructs the serial stream.
  virtual ~SerialStream() = default;

  /// @brief Initializes the serial stream.
  ///
  /// @param baud Data rate in bits per second (baud).
  virtual void begin(unsigned long baud) const = 0;

  /// @brief Gets the number of bytes available for reading.
  ///
  /// @return Number of bytes available for reading.
  virtual int available() const = 0;

  /// @brief Gets the incoming data.
  ///
  /// @return First byte of incoming data, or -1 if no data is available.
  virtual int read() const = 0;

  /// @brief Sends data as human-readable ASCII text.
  ///
  /// @param c String to send.
  /// @return Number of bytes sent.
  virtual size_t print(const char* c) const = 0;

  /// @brief Sends data as human-readable ASCII text.
  ///
  /// @param num Number to send.
  /// @return Number of bytes sent.
  virtual size_t print(int16_t num) const = 0;

  /// @brief Sends data as human-readable ASCII text.
  ///
  /// @param num Number to send.
  /// @return Number of bytes sent.
  virtual size_t print(int32_t num) const = 0;

  /// @brief Sends data as human-readable ASCII text.
  ///
  /// @param num Number to send.
  /// @param digits Number of digits after the decimal point.
  /// @return Number of bytes sent.
  virtual size_t print(double num, int digits = 2) const = 0;

  /// @brief Sends data as human-readable ASCII text, followed by a newline.
  ///
  /// @param c String to send.
  /// @return Number of bytes sent.
  virtual size_t println(const char* c) const = 0;

  /// @brief Sends data as human-readable ASCII text, followed by a newline.
  ///
  /// @param num Number to send.
  /// @return Number of bytes sent.
  virtual size_t println(int16_t num) const = 0;

  /// @brief Sends data as human-readable ASCII text, followed by a newline.
  ///
  /// @param num Number to send.
  /// @return Number of bytes sent.
  virtual size_t println(int32_t num) const = 0;

  /// @brief Sends data as human-readable ASCII text, followed by a newline.
  ///
  /// @param num Number to send.
  /// @param digits Number of digits after the decimal point.
  /// @return Number of bytes sent.
  virtual size_t println(double num, int digits = 2) const = 0;
};

}  // namespace andino
