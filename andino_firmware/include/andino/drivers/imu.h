// BSD 3-Clause License
//
// Copyright (c) 2026, Ekumen Inc.
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

/// @brief This class allows to use an IMU sensor by configuring it and then getting its
/// orientation, angular velocity and linear acceleration readings. It is backed by an Adafruit
/// BNO055 sensor.
class Imu {
 public:
  /// @brief Absolute orientation as a quaternion.
  struct Orientation {
    /// @brief Constructs a zeroed-out Orientation.
    Orientation() = default;

    /// @brief Constructs an Orientation from its components.
    Orientation(double orientation_x, double orientation_y, double orientation_z,
                double orientation_w)
        : x(orientation_x), y(orientation_y), z(orientation_z), w(orientation_w) {
    }

    /// Quaternion x component.
    double x{0.0};
    /// Quaternion y component.
    double y{0.0};
    /// Quaternion z component.
    double z{0.0};
    /// Quaternion w component.
    double w{0.0};
  };

  /// @brief A 3-axis vector reading (e.g. angular velocity or linear acceleration).
  struct Vector3 {
    /// @brief Constructs a zeroed-out Vector3.
    Vector3() = default;

    /// @brief Constructs a Vector3 from its components.
    Vector3(double vector_x, double vector_y, double vector_z)
        : x(vector_x), y(vector_y), z(vector_z) {
    }

    /// X axis component.
    double x{0.0};
    /// Y axis component.
    double y{0.0};
    /// Z axis component.
    double z{0.0};
  };

  /// @brief Initializes the IMU sensor.
  ///
  /// @return True if the IMU sensor was successfully initialized, false otherwise.
  bool begin() const;

  /// @brief Gets the absolute orientation.
  ///
  /// @return Absolute orientation as a quaternion.
  Orientation get_orientation() const;

  /// @brief Gets the angular velocity.
  ///
  /// @return Angular velocity vector [rad/s].
  Vector3 get_angular_velocity() const;

  /// @brief Gets the linear acceleration.
  ///
  /// @return Linear acceleration vector [m/s^2].
  Vector3 get_linear_acceleration() const;
};

}  // namespace andino
