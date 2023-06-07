// Copyright 2023 Franco Cipollone
#pragma once

#include <string>

namespace carpincho_base {

/// Represents a wheel on the robot.
/// TODO(francocipollone): Use better practices
///    1 - Make it a struct
///    2 - Move Angle method to DiffDriveCarpincho class or implementation.
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

}  // namespace carpincho_base
