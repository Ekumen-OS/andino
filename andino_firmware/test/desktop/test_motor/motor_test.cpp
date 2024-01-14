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
#include "motor.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "digital_out.h"
#include "pwm_out.h"

namespace andino {
namespace test {
namespace {

class MockDigitalOut : public andino::DigitalOut {
 public:
  MockDigitalOut(const int gpio_pin) : andino::DigitalOut(gpio_pin) {}
  MOCK_METHOD(void, begin, (), (const, override));
  MOCK_METHOD(void, write, (int value), (const, override));
};

class MockPwmOut : public andino::PwmOut {
 public:
  MockPwmOut(const int gpio_pin) : andino::PwmOut(gpio_pin) {}
  MOCK_METHOD(void, begin, (), (const, override));
  MOCK_METHOD(void, write, (int value), (const, override));
};

class MotorTest : public testing::Test {
 protected:
  MockDigitalOut enable_digital_out_{0};
  MockPwmOut forward_pwm_out_{0};
  MockPwmOut backward_pwm_out_{0};
};

TEST_F(MotorTest, Initialize) {
  EXPECT_CALL(enable_digital_out_, begin()).Times(1);
  EXPECT_CALL(forward_pwm_out_, begin()).Times(1);
  EXPECT_CALL(backward_pwm_out_, begin()).Times(1);

  andino::Motor motor(&enable_digital_out_, &forward_pwm_out_, &backward_pwm_out_);
  motor.begin();
}

TEST_F(MotorTest, Enable) {
  EXPECT_CALL(enable_digital_out_, write(1)).Times(1);

  andino::Motor motor(&enable_digital_out_, &forward_pwm_out_, &backward_pwm_out_);
  motor.enable(true);
}

TEST_F(MotorTest, Disable) {
  EXPECT_CALL(enable_digital_out_, write(0)).Times(1);

  andino::Motor motor(&enable_digital_out_, &forward_pwm_out_, &backward_pwm_out_);
  motor.enable(false);
}

TEST_F(MotorTest, SetPositiveSpeed) {
  EXPECT_CALL(forward_pwm_out_, write(100)).Times(1);
  EXPECT_CALL(backward_pwm_out_, write(0)).Times(1);

  andino::Motor motor(&enable_digital_out_, &forward_pwm_out_, &backward_pwm_out_);
  motor.set_speed(100);
}

TEST_F(MotorTest, SetNegativeSpeed) {
  EXPECT_CALL(forward_pwm_out_, write(0)).Times(1);
  EXPECT_CALL(backward_pwm_out_, write(100)).Times(1);

  andino::Motor motor(&enable_digital_out_, &forward_pwm_out_, &backward_pwm_out_);
  motor.set_speed(-100);
}

TEST_F(MotorTest, SetZeroSpeed) {
  EXPECT_CALL(forward_pwm_out_, write(0)).Times(1);
  EXPECT_CALL(backward_pwm_out_, write(0)).Times(1);

  andino::Motor motor(&enable_digital_out_, &forward_pwm_out_, &backward_pwm_out_);
  motor.set_speed(0);
}

TEST_F(MotorTest, SetHigherThanMaximumPositiveSpeed) {
  EXPECT_CALL(forward_pwm_out_, write(255)).Times(1);
  EXPECT_CALL(backward_pwm_out_, write(0)).Times(1);

  andino::Motor motor(&enable_digital_out_, &forward_pwm_out_, &backward_pwm_out_);
  motor.set_speed(500);
}

TEST_F(MotorTest, SetLowerThanMinimumNegativeSpeed) {
  EXPECT_CALL(forward_pwm_out_, write(0)).Times(1);
  EXPECT_CALL(backward_pwm_out_, write(255)).Times(1);

  andino::Motor motor(&enable_digital_out_, &forward_pwm_out_, &backward_pwm_out_);
  motor.set_speed(-500);
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
