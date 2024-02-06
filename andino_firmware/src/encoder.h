// Code in this file is inspired by:
// https://github.com/hbrobotics/ros_arduino_bridge/blob/indigo-devel/ros_arduino_firmware/src/libraries/ROSArduinoBridge/encoder_driver.h
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
#pragma once

#include <stdint.h>

#include "interrupt_in.h"

namespace andino {

/// @brief This class allows to use a quadrature encoder by configuring it and then getting its
/// ticks count value.
/// @note The current implementation only supports two instances of this class to be constructed.
class Encoder {
 public:
  /// @brief Constructs a new Encoder object.
  ///
  /// @param channel_a_interrupt_in Digital interrupt input connected to encoder channel A pin.
  /// @param channel_b_interrupt_in Digital interrupt input connected to encoder channel B pin.
  Encoder(const InterruptIn* channel_a_interrupt_in, const InterruptIn* channel_b_interrupt_in)
      : channel_a_interrupt_in_(channel_a_interrupt_in),
        channel_b_interrupt_in_(channel_b_interrupt_in) {}

  /// @brief Initializes the encoder.
  void begin();

  /// @brief Gets the ticks count value.
  ///
  /// @return Ticks count value.
  long read();

  /// @brief Sets the ticks count value to zero.
  void reset();

 private:
  /// Ticks delta lookup table. Its content is defined as follows:
  ///   +--------+-----+-----+--------+-----------+
  ///   | Number | Old | New | Binary | Direction |
  ///   |        | A B | A B |        |           |
  ///   +--------+-----+-----+--------+-----------+
  ///   |    0   | 0 0 | 0 0 |  0000  |     0     |
  ///   |    1   | 0 0 | 0 1 |  0001  |    +1     |
  ///   |    2   | 0 0 | 1 0 |  0010  |    -1     |
  ///   |    3   | 0 0 | 1 1 |  0011  |     0     |
  ///   |    4   | 0 1 | 0 0 |  0100  |    -1     |
  ///   |    5   | 0 1 | 0 1 |  0101  |     0     |
  ///   |    6   | 0 1 | 1 0 |  0110  |     0     |
  ///   |    7   | 0 1 | 1 1 |  0111  |    +1     |
  ///   |    8   | 1 0 | 0 0 |  1000  |    +1     |
  ///   |    9   | 1 0 | 0 1 |  1001  |     0     |
  ///   |   10   | 1 0 | 1 0 |  1010  |     0     |
  ///   |   11   | 1 0 | 1 1 |  1011  |    -1     |
  ///   |   12   | 1 1 | 0 0 |  1100  |     0     |
  ///   |   13   | 1 1 | 0 1 |  1101  |    -1     |
  ///   |   14   | 1 1 | 1 0 |  1110  |    +1     |
  ///   |   15   | 1 1 | 1 1 |  1111  |     0     |
  ///   +--------+-----+-----+--------+-----------+
  static constexpr int8_t kTicksDelta[]{0, 1, -1, 0, -1, 0, 0, 1, 1, 0, 0, -1, 0, -1, 1, 0};

  /// Maximum number of instances of this class that can be instantiated.
  static constexpr int kInstancesMax{2};

  /// Static wrappers that redirect to instance callback methods.
  static const InterruptIn::InterruptCallback kCallbacks[kInstancesMax];

  /// Static wrapper that redirects to the first instance callback method.
  static void callback_0();

  /// Static wrapper that redirects to the second instance callback method.
  static void callback_1();

  /// Channels interrupt callback.
  void callback();

  /// Holds references to the constructed Encoder instances.
  static Encoder* instances_[kInstancesMax];

  /// Number of constructed Encoder instances.
  static int instance_count_;

  /// Digital interrupt input connected to encoder channel A pin.
  const InterruptIn* channel_a_interrupt_in_;

  /// Digital interrupt input connected to encoder channel B pin.
  const InterruptIn* channel_b_interrupt_in_;

  /// Encoder state. It contains both the current and previous channels state readings:
  ///   +------+-----+-----+-----+-----+-----+-----+-----+-----+
  ///   | Bits |  7  |  6  |  5  |  4  |  3  |  2  |  1  |  0  |
  ///   +------+-----+-----+-----+-----+-----+-----+-----+-----+
  ///   |      |  x  |  x  |  x  |  x  | PREVIOUS  |  CURRENT  |
  ///   +------+-----+-----+-----+-----+-----+-----+-----+-----+
  uint8_t state_{0x00};

  /// Ticks count.
  volatile long count_{0L};
};

}  // namespace andino
