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

/// @brief Common constants.
struct Constants {
  /// @brief Serial port baud rate.
  static constexpr long kBaudrate{57600};

  /// @brief Time window to automatically stop the robot if no command has been received [ms].
  static constexpr long kAutoStopWindow{3000};

  /// @brief Minimum PWM wave duty cycle (0%) (see
  /// https://www.arduino.cc/reference/en/language/functions/analog-io/analogwrite/).
  static constexpr int kPwmMin{0};
  /// @brief Maximum PWM wave duty cycle (100%) (see
  /// https://www.arduino.cc/reference/en/language/functions/analog-io/analogwrite/).
  static constexpr int kPwmMax{255};

  /// @brief PID computation rate [Hz].
  static constexpr int kPidRate{30};
  /// @brief PID computation period [ms].
  static constexpr double kPidPeriod{1000 / kPidRate};
  /// @brief PID default tuning proportional gain.
  static constexpr int kPidKp{30};
  /// @brief PID default tuning derivative gain.
  static constexpr int kPidKd{10};
  /// @brief PID default tuning integral gain.
  static constexpr int kPidKi{0};
  /// @brief PID default tuning output gain.
  static constexpr int kPidKo{10};
};

}  // namespace andino
