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
#include "andino_base/mcu_reply_parser.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

namespace andino_base {
namespace mcu_reply_parser {
namespace {

using ::testing::ElementsAre;
using ::testing::Optional;

TEST(McuReplyParserTest, IsError) {
  EXPECT_TRUE(is_error("[ERROR] Invalid arguments\r\n"));
  EXPECT_FALSE(is_error("[OK]\r\n"));
  EXPECT_FALSE(is_error("10 20\r\n"));
  EXPECT_FALSE(is_error(" [ERROR] Not at the start\r\n"));
}

TEST(McuReplyParserTest, IsOk) {
  EXPECT_TRUE(is_ok("[OK]\r\n"));
  EXPECT_TRUE(is_ok("[OK]"));
  EXPECT_FALSE(is_ok(""));
  EXPECT_FALSE(is_ok("[ERROR] Unknown command\r\n"));
  EXPECT_FALSE(is_ok("[OK] trailing\r\n"));
}

TEST(McuReplyParserTest, ParseEncoders) {
  EXPECT_THAT(parse_encoders("123 -456\r\n"), Optional(ElementsAre(123, -456)));
  EXPECT_THAT(parse_encoders("0 0"), Optional(ElementsAre(0, 0)));
}

TEST(McuReplyParserTest, ParseEncodersRejectsMalformedReplies) {
  EXPECT_EQ(parse_encoders(""), std::nullopt);
  EXPECT_EQ(parse_encoders("\r\n"), std::nullopt);
  EXPECT_EQ(parse_encoders("123\r\n"), std::nullopt);
  EXPECT_EQ(parse_encoders("123 abc\r\n"), std::nullopt);
  EXPECT_EQ(parse_encoders("123 456 789\r\n"), std::nullopt);
  EXPECT_EQ(parse_encoders("[ERROR] Unknown command\r\n"), std::nullopt);
}

TEST(McuReplyParserTest, ParseImuAvailable) {
  EXPECT_THAT(parse_imu_available("1\r\n"), Optional(true));
  EXPECT_THAT(parse_imu_available("0\r\n"), Optional(false));
  EXPECT_EQ(parse_imu_available(""), std::nullopt);
  EXPECT_EQ(parse_imu_available("2\r\n"), std::nullopt);
  EXPECT_EQ(parse_imu_available("yes\r\n"), std::nullopt);
}

TEST(McuReplyParserTest, ParseEncodersAndImu) {
  const std::optional<Mcu::EncodersAndImuData> data =
      parse_encoders_and_imu("10 -20 0.0000 0.0000 0.7071 0.7071 0.1000 -0.2000 0.3000 0.0100 0.0200 9.8100\r\n");

  ASSERT_TRUE(data.has_value());
  EXPECT_THAT(data->encoders_data, ElementsAre(10, -20));
  EXPECT_THAT(data->imu_data.orientation, ElementsAre(0.0, 0.0, 0.7071, 0.7071));
  EXPECT_THAT(data->imu_data.angular_velocity, ElementsAre(0.1, -0.2, 0.3));
  EXPECT_THAT(data->imu_data.linear_acceleration, ElementsAre(0.01, 0.02, 9.81));
}

TEST(McuReplyParserTest, ParseEncodersAndImuRejectsMalformedReplies) {
  EXPECT_EQ(parse_encoders_and_imu(""), std::nullopt);
  // Missing the last linear acceleration value.
  EXPECT_EQ(parse_encoders_and_imu("10 -20 0 0 0.7071 0.7071 0.1 -0.2 0.3 0.01 0.02\r\n"), std::nullopt);
  // One extra value.
  EXPECT_EQ(parse_encoders_and_imu("10 -20 0 0 0.7071 0.7071 0.1 -0.2 0.3 0.01 0.02 9.81 1\r\n"), std::nullopt);
  EXPECT_EQ(parse_encoders_and_imu("[ERROR] IMU unavailable\r\n"), std::nullopt);
}

}  // namespace
}  // namespace mcu_reply_parser
}  // namespace andino_base
