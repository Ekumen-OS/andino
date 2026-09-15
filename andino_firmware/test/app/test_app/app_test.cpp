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
#include "andino/app/app.h"

#include <stdio.h>

#include <string>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "andino/app/constants.h"
#include "andino/hal/clock.h"
#include "andino/hal/digital_out.h"
#include "andino/hal/imu.h"
#include "andino/hal/interrupt_in.h"
#include "andino/hal/pwm_out.h"
#include "andino/hal/serial_stream.h"

namespace andino {
namespace test {
namespace {

using ::testing::NiceMock;
using ::testing::Return;

class MockClock : public andino::Clock {
 public:
  MOCK_METHOD(unsigned long, millis, (), (const, override));
  MOCK_METHOD(void, delay, (unsigned long ms), (const, override));
};

class MockDigitalOut : public andino::DigitalOut {
 public:
  MOCK_METHOD(void, begin, (), (const, override));
  MOCK_METHOD(void, write, (uint8_t value), (const, override));
};

class MockPwmOut : public andino::PwmOut {
 public:
  MOCK_METHOD(void, begin, (), (const, override));
  MOCK_METHOD(void, write, (int value), (const, override));
};

class MockInterruptIn : public andino::InterruptIn {
 public:
  MOCK_METHOD(void, begin, (), (const, override));
  MOCK_METHOD(int, read, (), (const, override));
  MOCK_METHOD(void, attach, (andino::InterruptIn::InterruptCallback callback), (const, override));
};

class MockImu : public andino::Imu {
 public:
  MOCK_METHOD(bool, begin, (), (const, override));
  MOCK_METHOD(andino::Imu::Orientation, get_orientation, (), (const, override));
  MOCK_METHOD(andino::Imu::Vector3, get_angular_velocity, (), (const, override));
  MOCK_METHOD(andino::Imu::Vector3, get_linear_acceleration, (), (const, override));
};

/// @brief Serial stream fake that replays a canned input and records everything written to it.
class FakeSerialStream : public andino::SerialStream {
 public:
  /// @brief Queues the given text as incoming data.
  void set_input(const std::string& input) {
    input_ = input;
    input_index_ = 0;
  }

  /// @brief Gets the text written to the stream so far.
  const std::string& output() const {
    return output_;
  }

  /// @brief Discards the text written to the stream so far.
  void clear_output() {
    output_.clear();
  }

  /// @brief Gets the baud rate the stream was initialized with.
  unsigned long baud() const {
    return baud_;
  }

  void begin(unsigned long baud) const override {
    baud_ = baud;
  }

  int available() const override {
    return static_cast<int>(input_.size() - input_index_);
  }

  int read() const override {
    return (input_index_ < input_.size()) ? input_.at(input_index_++) : -1;
  }

  size_t print(const char* c) const override {
    return append(std::string(c));
  }

  size_t print(char c) const override {
    return append(std::string(1, c));
  }

  size_t print(unsigned char b, int base) const override {
    return append(to_string(static_cast<unsigned long>(b), base));
  }

  size_t print(int num, int base) const override {
    return append(to_string(static_cast<long>(num), base));
  }

  size_t print(unsigned int num, int base) const override {
    return append(to_string(static_cast<unsigned long>(num), base));
  }

  size_t print(long num, int base) const override {
    return append(to_string(num, base));
  }

  size_t print(unsigned long num, int base) const override {
    return append(to_string(num, base));
  }

  size_t print(double num, int digits) const override {
    return append(to_string(num, digits));
  }

  size_t println(const char* c) const override {
    return append(std::string(c) + "\n");
  }

  size_t println(char c) const override {
    return append(std::string(1, c) + "\n");
  }

  size_t println(unsigned char b, int base) const override {
    return append(to_string(static_cast<unsigned long>(b), base) + "\n");
  }

  size_t println(int num, int base) const override {
    return append(to_string(static_cast<long>(num), base) + "\n");
  }

  size_t println(unsigned int num, int base) const override {
    return append(to_string(static_cast<unsigned long>(num), base) + "\n");
  }

  size_t println(long num, int base) const override {
    return append(to_string(num, base) + "\n");
  }

  size_t println(unsigned long num, int base) const override {
    return append(to_string(num, base) + "\n");
  }

  size_t println(double num, int digits) const override {
    return append(to_string(num, digits) + "\n");
  }

 private:
  static std::string to_string(long num, int base) {
    char buffer[34]{};
    snprintf(buffer, sizeof(buffer), (base == kHex) ? "%lx" : "%ld", num);
    return std::string(buffer);
  }

  static std::string to_string(unsigned long num, int base) {
    char buffer[34]{};
    snprintf(buffer, sizeof(buffer), (base == kHex) ? "%lx" : "%lu", num);
    return std::string(buffer);
  }

  static std::string to_string(double num, int digits) {
    char buffer[64]{};
    snprintf(buffer, sizeof(buffer), "%.*f", digits, num);
    return std::string(buffer);
  }

  size_t append(const std::string& text) const {
    output_ += text;
    return text.size();
  }

  std::string input_;
  mutable size_t input_index_{0};
  mutable std::string output_;
  mutable unsigned long baud_{0};
};

class AppTest : public testing::Test {
 protected:
  void SetUp() override {
    ON_CALL(clock_, millis()).WillByDefault(Return(0UL));
    ON_CALL(imu_, begin()).WillByDefault(Return(true));
  }

  /// @brief Runs a command through the command prompt and returns everything it wrote back.
  std::string run_command(const std::string& command) {
    serial_stream_.set_input(command + "\r");
    serial_stream_.clear_output();
    app_.loop();
    return serial_stream_.output();
  }

  NiceMock<MockClock> clock_;
  FakeSerialStream serial_stream_;
  NiceMock<MockDigitalOut> left_motor_enable_;
  NiceMock<MockPwmOut> left_motor_forward_;
  NiceMock<MockPwmOut> left_motor_backward_;
  NiceMock<MockDigitalOut> right_motor_enable_;
  NiceMock<MockPwmOut> right_motor_forward_;
  NiceMock<MockPwmOut> right_motor_backward_;
  NiceMock<MockInterruptIn> left_encoder_a_;
  NiceMock<MockInterruptIn> left_encoder_b_;
  NiceMock<MockInterruptIn> right_encoder_a_;
  NiceMock<MockInterruptIn> right_encoder_b_;
  NiceMock<MockImu> imu_;

  andino::App app_{clock_,
                   serial_stream_,
                   left_motor_enable_,
                   left_motor_forward_,
                   left_motor_backward_,
                   right_motor_enable_,
                   right_motor_forward_,
                   right_motor_backward_,
                   left_encoder_a_,
                   left_encoder_b_,
                   right_encoder_a_,
                   right_encoder_b_,
                   imu_};
};

TEST_F(AppTest, SetupInitializesSerialStream) {
  app_.setup();

  EXPECT_EQ(serial_stream_.baud(), static_cast<unsigned long>(Constants::kBaudrate));
}

TEST_F(AppTest, SetupInitializesMotors) {
  EXPECT_CALL(left_motor_enable_, begin()).Times(1);
  EXPECT_CALL(left_motor_forward_, begin()).Times(1);
  EXPECT_CALL(left_motor_backward_, begin()).Times(1);
  EXPECT_CALL(right_motor_enable_, begin()).Times(1);
  EXPECT_CALL(right_motor_forward_, begin()).Times(1);
  EXPECT_CALL(right_motor_backward_, begin()).Times(1);

  // Both motors are enabled upon startup.
  EXPECT_CALL(left_motor_enable_, write(1)).Times(1);
  EXPECT_CALL(right_motor_enable_, write(1)).Times(1);

  app_.setup();
}

TEST_F(AppTest, SetupInitializesEncoders) {
  EXPECT_CALL(left_encoder_a_, begin()).Times(1);
  EXPECT_CALL(left_encoder_b_, begin()).Times(1);
  EXPECT_CALL(right_encoder_a_, begin()).Times(1);
  EXPECT_CALL(right_encoder_b_, begin()).Times(1);
  EXPECT_CALL(left_encoder_a_, attach(::testing::_)).Times(1);
  EXPECT_CALL(left_encoder_b_, attach(::testing::_)).Times(1);
  EXPECT_CALL(right_encoder_a_, attach(::testing::_)).Times(1);
  EXPECT_CALL(right_encoder_b_, attach(::testing::_)).Times(1);

  app_.setup();
}

TEST_F(AppTest, SetupInitializesImu) {
  EXPECT_CALL(imu_, begin()).Times(1).WillOnce(Return(true));

  app_.setup();
}

TEST_F(AppTest, UnknownCommand) {
  app_.setup();

  EXPECT_EQ(run_command("z"), "Unknown command.\n");
}

TEST_F(AppTest, ReadEncodersCommand) {
  app_.setup();

  EXPECT_EQ(run_command("e"), "0 0\n");
}

TEST_F(AppTest, ResetEncodersCommand) {
  app_.setup();

  EXPECT_EQ(run_command("r"), "OK\n");
  EXPECT_EQ(run_command("e"), "0 0\n");
}

TEST_F(AppTest, ReadDigitalGpioCommand) {
  app_.setup();

  ON_CALL(left_encoder_a_, read()).WillByDefault(Return(1));
  ON_CALL(left_encoder_b_, read()).WillByDefault(Return(0));
  ON_CALL(right_encoder_a_, read()).WillByDefault(Return(0));
  ON_CALL(right_encoder_b_, read()).WillByDefault(Return(1));

  EXPECT_EQ(run_command("d 0 0"), "1\n");
  EXPECT_EQ(run_command("d 0 1"), "0\n");
  EXPECT_EQ(run_command("d 1 0"), "0\n");
  EXPECT_EQ(run_command("d 1 1"), "1\n");
}

TEST_F(AppTest, GetIsImuConnectedCommandWhenConnected) {
  EXPECT_CALL(imu_, begin()).WillOnce(Return(true));
  app_.setup();

  EXPECT_EQ(run_command("h"), "1\n");
}

TEST_F(AppTest, GetIsImuConnectedCommandWhenNotConnected) {
  EXPECT_CALL(imu_, begin()).WillOnce(Return(false));
  app_.setup();

  EXPECT_EQ(run_command("h"), "0\n");
}

TEST_F(AppTest, SetMotorsPwmCommand) {
  app_.setup();

  // A positive value drives the motor forward, while a negative one drives it backward.
  EXPECT_CALL(left_motor_forward_, write(100)).Times(1);
  EXPECT_CALL(left_motor_backward_, write(0)).Times(1);
  EXPECT_CALL(right_motor_backward_, write(150)).Times(1);
  EXPECT_CALL(right_motor_forward_, write(0)).Times(1);

  EXPECT_EQ(run_command("o 100 -150"), "OK\n");
}

TEST_F(AppTest, SetMotorsSpeedCommandWithZeroSpeedStopsMotors) {
  app_.setup();

  EXPECT_CALL(left_motor_forward_, write(0)).Times(1);
  EXPECT_CALL(left_motor_backward_, write(0)).Times(1);
  EXPECT_CALL(right_motor_forward_, write(0)).Times(1);
  EXPECT_CALL(right_motor_backward_, write(0)).Times(1);

  EXPECT_EQ(run_command("m 0 0"), "OK\n");
}

TEST_F(AppTest, SetPidsTuningGainsCommand) {
  app_.setup();

  EXPECT_EQ(run_command("u 30 20 10 50"), "PID Updated: 30 20 10 50\nOK\n");
}

TEST_F(AppTest, ReadEncodersAndImuCommand) {
  app_.setup();

  ON_CALL(imu_, get_orientation())
      .WillByDefault(Return(andino::Imu::Orientation{0.1, 0.2, 0.3, 0.4}));
  ON_CALL(imu_, get_angular_velocity()).WillByDefault(Return(andino::Imu::Vector3{1.5, 2.5, 3.5}));
  ON_CALL(imu_, get_linear_acceleration())
      .WillByDefault(Return(andino::Imu::Vector3{4.5, 5.5, 6.5}));

  // Ticks count, orientation quaternion, angular velocity and linear acceleration.
  EXPECT_EQ(run_command("i"), "0 0 0.1000 0.2000 0.3000 0.4000 1.50 2.50 3.50 4.50 5.50 6.50");
}

TEST_F(AppTest, LoopStopsMotorsOnceAutoStopWindowElapses) {
  app_.setup();

  // No set motors speed command was received, so the motors must be stopped once the auto stop
  // time window elapses.
  EXPECT_CALL(left_motor_forward_, write(0)).Times(1);
  EXPECT_CALL(left_motor_backward_, write(0)).Times(1);
  EXPECT_CALL(right_motor_forward_, write(0)).Times(1);
  EXPECT_CALL(right_motor_backward_, write(0)).Times(1);

  ON_CALL(clock_, millis())
      .WillByDefault(Return(static_cast<unsigned long>(Constants::kAutoStopWindow) + 1UL));
  app_.loop();
}

TEST_F(AppTest, LoopKeepsMotorsRunningWithinAutoStopWindow) {
  app_.setup();

  EXPECT_CALL(left_motor_forward_, write(::testing::_)).Times(0);
  EXPECT_CALL(left_motor_backward_, write(::testing::_)).Times(0);
  EXPECT_CALL(right_motor_forward_, write(::testing::_)).Times(0);
  EXPECT_CALL(right_motor_backward_, write(::testing::_)).Times(0);

  ON_CALL(clock_, millis())
      .WillByDefault(Return(static_cast<unsigned long>(Constants::kAutoStopWindow) - 1UL));
  app_.loop();
}

TEST_F(AppTest, LoopDrivesMotorsWhilePidIsEnabled) {
  app_.setup();
  run_command("m 100 100");

  // The PID controllers are enabled by a non zero set motors speed command, so the computed output
  // reaches the motors once the PID computation period elapses.
  EXPECT_CALL(left_motor_forward_, write(::testing::Gt(0))).Times(1);
  EXPECT_CALL(left_motor_backward_, write(0)).Times(1);
  EXPECT_CALL(right_motor_forward_, write(::testing::Gt(0))).Times(1);
  EXPECT_CALL(right_motor_backward_, write(0)).Times(1);

  ON_CALL(clock_, millis())
      .WillByDefault(Return(static_cast<unsigned long>(Constants::kPidPeriod) + 1UL));
  app_.loop();
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
