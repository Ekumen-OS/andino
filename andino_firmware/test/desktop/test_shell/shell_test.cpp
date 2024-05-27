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
#include "shell.h"

#include <string>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "serial_stream.h"

namespace andino {
namespace test {
namespace {

using ::testing::Return;

class MockSerialStream : public andino::SerialStream {
 public:
  MockSerialStream() : andino::SerialStream() {}
  MOCK_METHOD(void, begin, (unsigned long baud), (const, override));
  MOCK_METHOD(int, available, (), (const, override));
  MOCK_METHOD(int, read, (), (const, override));
  MOCK_METHOD(size_t, print, (const char* c), (const, override));
  MOCK_METHOD(size_t, print, (char c), (const, override));
  MOCK_METHOD(size_t, print, (unsigned char b, int base), (const, override));
  MOCK_METHOD(size_t, print, (int num, int base), (const, override));
  MOCK_METHOD(size_t, print, (unsigned int num, int base), (const, override));
  MOCK_METHOD(size_t, print, (long num, int base), (const, override));
  MOCK_METHOD(size_t, print, (unsigned long num, int base), (const, override));
  MOCK_METHOD(size_t, print, (double num, int digits), (const, override));
  MOCK_METHOD(size_t, println, (const char* c), (const, override));
  MOCK_METHOD(size_t, println, (char c), (const, override));
  MOCK_METHOD(size_t, println, (unsigned char b, int base), (const, override));
  MOCK_METHOD(size_t, println, (int num, int base), (const, override));
  MOCK_METHOD(size_t, println, (unsigned int num, int base), (const, override));
  MOCK_METHOD(size_t, println, (long num, int base), (const, override));
  MOCK_METHOD(size_t, println, (unsigned long num, int base), (const, override));
  MOCK_METHOD(size_t, println, (double num, int digits), (const, override));
};

class ShellTest : public testing::Test {
 protected:
  void SetUp() override {
    shell.set_serial_stream(&serial_stream_);
    shell.set_default_callback(cmd_unknown_cb);
    shell.register_command(kCommand1, cmd_1_cb);
    shell.register_command(kCommand2, cmd_2_cb);
    shell.register_command(kCommand3, cmd_3_cb);
  }

  static void cmd_unknown_cb(int argc, char** argv) {
    called_callback_ = 0;
    save_arguments(argc, argv);
  }

  static void cmd_1_cb(int argc, char** argv) {
    called_callback_ = 1;
    save_arguments(argc, argv);
  }

  static void cmd_2_cb(int argc, char** argv) {
    called_callback_ = 2;
    save_arguments(argc, argv);
  }

  static void cmd_3_cb(int argc, char** argv) {
    called_callback_ = 3;
    save_arguments(argc, argv);
  }

  static void save_arguments(int argc, char** argv) {
    argc_ = argc;
    argv_.assign(argv, argv + argc);
  }

  static constexpr const char* kCommand1{"a"};
  static constexpr const char* kCommand2{"ab"};
  static constexpr const char* kCommand3{"cde"};

  static int called_callback_;
  static int argc_;
  static std::vector<std::string> argv_;

  andino::Shell shell;
  MockSerialStream serial_stream_;
};

int ShellTest::called_callback_{-1};
int ShellTest::argc_{0};
std::vector<std::string> ShellTest::argv_;

TEST_F(ShellTest, ProcessInputEmpty) {
  EXPECT_CALL(serial_stream_, available()).Times(1).WillOnce(Return(0));

  shell.process_input();
}

TEST_F(ShellTest, ProcessInputMessageSingleCharacterCommandSingleArg) {
  const std::string input_message{"a\r"};
  const std::vector<std::string> expected_argv{"a"};

  int available_call_count = input_message.size() + 1;
  EXPECT_CALL(serial_stream_, available()).Times(available_call_count);
  ON_CALL(serial_stream_, available())
      .WillByDefault(
          testing::Invoke([&available_call_count]() -> int { return --available_call_count; }));

  int input_index = 0;
  EXPECT_CALL(serial_stream_, read()).Times(input_message.size());
  ON_CALL(serial_stream_, read())
      .WillByDefault(testing::Invoke(
          [input_message, &input_index]() -> int { return input_message.at(input_index++); }));

  shell.process_input();

  ASSERT_EQ(called_callback_, 1);
  ASSERT_EQ(argc_, 1);
  EXPECT_THAT(argv_, ::testing::ElementsAreArray(expected_argv));
}

TEST_F(ShellTest, ProcessInputMessageUnknownCommand) {
  const std::string input_message{"z\r"};
  const std::vector<std::string> expected_argv{"z"};

  int available_call_count = input_message.size() + 1;
  EXPECT_CALL(serial_stream_, available()).Times(available_call_count);
  ON_CALL(serial_stream_, available())
      .WillByDefault(
          testing::Invoke([&available_call_count]() -> int { return --available_call_count; }));

  int input_index = 0;
  EXPECT_CALL(serial_stream_, read()).Times(input_message.size());
  ON_CALL(serial_stream_, read())
      .WillByDefault(testing::Invoke(
          [input_message, &input_index]() -> int { return input_message.at(input_index++); }));

  shell.process_input();

  ASSERT_EQ(called_callback_, 0);
  ASSERT_EQ(argc_, 1);
  EXPECT_THAT(argv_, ::testing::ElementsAreArray(expected_argv));
}

TEST_F(ShellTest, ProcessInputMessageTwoCharacterCommandSingleArg) {
  const std::string input_message{"ab\r"};
  const std::vector<std::string> expected_argv{"ab"};

  int available_call_count = input_message.size() + 1;
  EXPECT_CALL(serial_stream_, available()).Times(available_call_count);
  ON_CALL(serial_stream_, available())
      .WillByDefault(
          testing::Invoke([&available_call_count]() -> int { return --available_call_count; }));

  int input_index = 0;
  EXPECT_CALL(serial_stream_, read()).Times(input_message.size());
  ON_CALL(serial_stream_, read())
      .WillByDefault(testing::Invoke(
          [input_message, &input_index]() -> int { return input_message.at(input_index++); }));

  shell.process_input();

  ASSERT_EQ(called_callback_, 2);
  ASSERT_EQ(argc_, 1);
  EXPECT_EQ(argv_.at(0), expected_argv.at(0));
}

TEST_F(ShellTest, ProcessInputMessageThreeCharacterCommandSingleArg) {
  const std::string input_message{"cde\r"};
  const std::vector<std::string> expected_argv{"cde"};

  int available_call_count = input_message.size() + 1;
  EXPECT_CALL(serial_stream_, available()).Times(available_call_count);
  ON_CALL(serial_stream_, available())
      .WillByDefault(
          testing::Invoke([&available_call_count]() -> int { return --available_call_count; }));

  int input_index = 0;
  EXPECT_CALL(serial_stream_, read()).Times(input_message.size());
  ON_CALL(serial_stream_, read())
      .WillByDefault(testing::Invoke(
          [input_message, &input_index]() -> int { return input_message.at(input_index++); }));

  shell.process_input();

  ASSERT_EQ(called_callback_, 3);
  ASSERT_EQ(argc_, 1);
  EXPECT_THAT(argv_, ::testing::ElementsAreArray(expected_argv));
}

TEST_F(ShellTest, ProcessInputMessageTwoArgs) {
  const std::string input_message{"a 12\r"};
  const std::vector<std::string> expected_argv{"a", "12"};

  int available_call_count = input_message.size() + 1;
  EXPECT_CALL(serial_stream_, available()).Times(available_call_count);
  ON_CALL(serial_stream_, available())
      .WillByDefault(
          testing::Invoke([&available_call_count]() -> int { return --available_call_count; }));

  int input_index = 0;
  EXPECT_CALL(serial_stream_, read()).Times(input_message.size());
  ON_CALL(serial_stream_, read())
      .WillByDefault(testing::Invoke(
          [input_message, &input_index]() -> int { return input_message.at(input_index++); }));

  shell.process_input();

  ASSERT_EQ(called_callback_, 1);
  ASSERT_EQ(argc_, 2);
  EXPECT_THAT(argv_, ::testing::ElementsAreArray(expected_argv));
}

TEST_F(ShellTest, ProcessInputMessageThreeArgs) {
  const std::string input_message{"ab 12 3\r"};
  const std::vector<std::string> expected_argv{"ab", "12", "3"};

  int available_call_count = input_message.size() + 1;
  EXPECT_CALL(serial_stream_, available()).Times(available_call_count);
  ON_CALL(serial_stream_, available())
      .WillByDefault(
          testing::Invoke([&available_call_count]() -> int { return --available_call_count; }));

  int input_index = 0;
  EXPECT_CALL(serial_stream_, read()).Times(input_message.size());
  ON_CALL(serial_stream_, read())
      .WillByDefault(testing::Invoke(
          [input_message, &input_index]() -> int { return input_message.at(input_index++); }));

  shell.process_input();

  ASSERT_EQ(called_callback_, 2);
  ASSERT_EQ(argc_, 3);
  EXPECT_THAT(argv_, ::testing::ElementsAreArray(expected_argv));
}

TEST_F(ShellTest, ProcessInputMessageFourArgs) {
  const std::string input_message{"cde 12 3 456\r"};
  const std::vector<std::string> expected_argv{"cde", "12", "3", "456"};

  int available_call_count = input_message.size() + 1;
  EXPECT_CALL(serial_stream_, available()).Times(available_call_count);
  ON_CALL(serial_stream_, available())
      .WillByDefault(
          testing::Invoke([&available_call_count]() -> int { return --available_call_count; }));

  int input_index = 0;
  EXPECT_CALL(serial_stream_, read()).Times(input_message.size());
  ON_CALL(serial_stream_, read())
      .WillByDefault(testing::Invoke(
          [input_message, &input_index]() -> int { return input_message.at(input_index++); }));

  shell.process_input();

  ASSERT_EQ(called_callback_, 3);
  ASSERT_EQ(argc_, 4);
  EXPECT_THAT(argv_, ::testing::ElementsAreArray(expected_argv));
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
