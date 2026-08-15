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
#include "andino/drivers/imu.h"

#include <Adafruit_BNO055.h>
#include <Wire.h>

/// Adafruit BNO055 IMU sensor instance.
static Adafruit_BNO055 g_bno055_imu(55, BNO055_ADDRESS_A, &Wire);

namespace andino {

bool Imu::begin() const {
  if (!g_bno055_imu.begin()) {
    return false;
  }
  g_bno055_imu.setExtCrystalUse(true);
  return true;
}

Imu::Orientation Imu::get_orientation() const {
  // See https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor/overview for
  // further information.
  imu::Quaternion orientation = g_bno055_imu.getQuat();
  return Orientation{orientation.x(), orientation.y(), orientation.z(), orientation.w()};
}

Imu::Vector3 Imu::get_angular_velocity() const {
  // See https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor/overview for
  // further information.
  imu::Vector<3> angular_velocity = g_bno055_imu.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  return Vector3{angular_velocity.x(), angular_velocity.y(), angular_velocity.z()};
}

Imu::Vector3 Imu::get_linear_acceleration() const {
  // See https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor/overview for
  // further information.
  imu::Vector<3> linear_acceleration = g_bno055_imu.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  return Vector3{linear_acceleration.x(), linear_acceleration.y(), linear_acceleration.z()};
}

}  // namespace andino
