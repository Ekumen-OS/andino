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
#include "andino/app/pid.h"

#include <gtest/gtest.h>

namespace andino {
namespace test {
namespace {

/// Rate used by the controllers in the tests [Hz].
constexpr uint16_t kRateHz{10};

/// Target speed [ticks/s] that is 15 ticks per period at `kRateHz`.
constexpr int16_t kTargetSpeed{150};

TEST(PidTest, Initialize) {
  andino::Pid pid_controller({1, 0, 0, 1}, -100, 100, kRateHz);
  int16_t output = 0;

  // Controller is disabled by default, so output variable should remain unchanged.
  pid_controller.compute(5, output);
  EXPECT_EQ(output, 0);
}

TEST(PidTest, ComputeOutputProportionalGain) {
  andino::Pid pid_controller({3, 0, 0, 1}, -100, 100, kRateHz);
  int16_t output = 0;
  pid_controller.set_target_speed(kTargetSpeed);
  pid_controller.enable();

  pid_controller.compute(5, output);
  EXPECT_EQ(output, 30);

  pid_controller.compute(10, output);
  EXPECT_EQ(output, 60);
}

TEST(PidTest, ComputeOutputProportionalAndDerivativeGain) {
  andino::Pid pid_controller({3, 2, 0, 1}, -100, 100, kRateHz);
  int16_t output = 0;
  pid_controller.set_target_speed(kTargetSpeed);
  pid_controller.enable();

  pid_controller.compute(5, output);
  EXPECT_EQ(output, 20);

  pid_controller.compute(10, output);
  EXPECT_EQ(output, 50);
}

TEST(PidTest, ComputeOutputProportionalAndIntegralGain) {
  andino::Pid pid_controller({3, 0, 1, 1}, -100, 100, kRateHz);
  int16_t output = 0;
  pid_controller.set_target_speed(kTargetSpeed);
  pid_controller.enable();

  pid_controller.compute(5, output);
  EXPECT_EQ(output, 30);

  pid_controller.compute(10, output);
  EXPECT_EQ(output, 70);
}

TEST(PidTest, ComputeOutputProportionalDerivativeAndIntegralGain) {
  andino::Pid pid_controller({3, 2, 1, 1}, -100, 100, kRateHz);
  int16_t output = 0;
  pid_controller.set_target_speed(kTargetSpeed);
  pid_controller.enable();

  pid_controller.compute(5, output);
  EXPECT_EQ(output, 20);

  pid_controller.compute(10, output);
  EXPECT_EQ(output, 60);
}

TEST(PidTest, Reset) {
  andino::Pid pid_controller({3, 0, 0, 1}, -100, 100, kRateHz);
  int16_t output = 0;
  pid_controller.set_target_speed(kTargetSpeed);
  pid_controller.enable();

  pid_controller.compute(5, output);
  EXPECT_EQ(output, 30);

  // Controller reset, obtained output should be the same as before.
  pid_controller.reset(0);
  pid_controller.set_target_speed(kTargetSpeed);
  pid_controller.compute(5, output);
  EXPECT_EQ(output, 30);
}

TEST(PidTest, SetTunings) {
  andino::Pid pid_controller({3, 0, 0, 1}, -100, 100, kRateHz);
  int16_t output = 0;
  pid_controller.set_target_speed(kTargetSpeed);
  pid_controller.enable();

  pid_controller.compute(5, output);
  EXPECT_EQ(output, 30);

  // Proportional gain doubled, obtained output should be doubled.
  pid_controller.reset(0);
  pid_controller.set_target_speed(kTargetSpeed);
  pid_controller.set_tunings({6, 0, 0, 1});
  pid_controller.compute(5, output);
  EXPECT_EQ(output, 60);
}

TEST(PidTest, PeriodIsDerivedFromTheRate) {
  EXPECT_EQ(andino::Pid({1, 0, 0, 1}, -100, 100, 10).period_ms(), 100UL);
  EXPECT_EQ(andino::Pid({1, 0, 0, 1}, -100, 100, 30).period_ms(), 33UL);
  EXPECT_EQ(andino::Pid({1, 0, 0, 1}, -100, 100, 1000).period_ms(), 1UL);
}

TEST(PidTest, ZeroRateIsTreatedAsOneHertz) {
  EXPECT_EQ(andino::Pid({1, 0, 0, 1}, -100, 100, 0).period_ms(), 1000UL);
}

// With a proportional gain of 1 and no previous output, the first output equals the setpoint, which
// makes the conversion of the target speed to ticks per period observable.
int16_t FirstOutputForTargetSpeed(int16_t ticks_per_second, uint16_t rate_hz) {
  andino::Pid pid_controller({1, 0, 0, 1}, -100, 100, rate_hz);
  int16_t output = 0;
  pid_controller.set_target_speed(ticks_per_second);
  pid_controller.enable();
  pid_controller.compute(0, output);
  return output;
}

TEST(PidTest, TargetSpeedIsConvertedToTicksPerPeriod) {
  EXPECT_EQ(FirstOutputForTargetSpeed(90, 30), 3);
  EXPECT_EQ(FirstOutputForTargetSpeed(150, 10), 15);
  EXPECT_EQ(FirstOutputForTargetSpeed(0, 30), 0);
}

TEST(PidTest, TargetSpeedConversionTruncatesTowardsZero) {
  EXPECT_EQ(FirstOutputForTargetSpeed(95, 30), 3);
  EXPECT_EQ(FirstOutputForTargetSpeed(29, 30), 0);
  EXPECT_EQ(FirstOutputForTargetSpeed(-95, 30), -3);
  EXPECT_EQ(FirstOutputForTargetSpeed(-29, 30), 0);
}

TEST(PidTest, TargetSpeedConversionSupportsTheFullSpeedRange) {
  EXPECT_EQ(FirstOutputForTargetSpeed(INT16_MAX, 1000), 32);
  EXPECT_EQ(FirstOutputForTargetSpeed(INT16_MIN, 1000), -32);
}

}  // namespace
}  // namespace test
}  // namespace andino

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  if (RUN_ALL_TESTS()) {
  }

  // Always return zero-code and allow PlatformIO to parse results.
  return 0;
}
