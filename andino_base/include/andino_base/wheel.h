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

#include <string>

namespace andino_base {

/// Represents a wheel on the robot.
/// TODO(francocipollone): Use better practices
///    1 - Make it a struct
///    2 - Move Angle method to DiffDriveAndino class or implementation.
///
class Wheel {
 public:
  /// @brief Default constructor for the Wheel class
  Wheel() = default;

  /// @brief Setup the wheel.
  /// @param wheel_name name of the wheel.
  /// @param ticks_per_rev number of encoder ticks per wheel revolution.
  void Setup(const std::string& wheel_name, int ticks_per_rev);

  /// @brief Calculate the angle of the wheel.
  /// @return The angle of the wheel in radians.
  double Angle();

  std::string name_ = "";
  unsigned int enc_ = 0;
  double cmd_ = 0;
  double pos_ = 0;
  double vel_ = 0;
  double eff_ = 0;
  double vel_set_pt_ = 0;
  double rads_per_tick_ = 0;
};

}  // namespace andino_base
