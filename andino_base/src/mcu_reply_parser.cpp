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

#include <sstream>
#include <string>

namespace andino_base {
namespace mcu_reply_parser {

namespace {

// Removes the leading and trailing whitespace, including the line terminator.
std::string trim(const std::string& reply) {
  constexpr char kWhitespace[] = " \t\r\n";
  const size_t begin = reply.find_first_not_of(kWhitespace);
  if (begin == std::string::npos) {
    return "";
  }
  const size_t end = reply.find_last_not_of(kWhitespace);
  return reply.substr(begin, end - begin + 1);
}

// Returns true if every extraction succeeded and nothing but whitespace is left in the stream.
bool fully_consumed(std::istringstream& iss) {
  if (iss.fail()) {
    return false;
  }
  iss >> std::ws;
  return iss.eof();
}

}  // namespace

bool is_error(const std::string& reply) { return reply.rfind("[ERROR]", 0) == 0; }

bool is_ok(const std::string& reply) { return trim(reply) == "[OK]"; }

std::optional<Mcu::EncodersData> parse_encoders(const std::string& reply) {
  std::istringstream iss(reply);
  Mcu::EncodersData encoders_data{};
  iss >> encoders_data[0] >> encoders_data[1];
  if (!fully_consumed(iss)) {
    return std::nullopt;
  }
  return encoders_data;
}

std::optional<bool> parse_imu_available(const std::string& reply) {
  const std::string value = trim(reply);
  if (value == "1") {
    return true;
  }
  if (value == "0") {
    return false;
  }
  return std::nullopt;
}

std::optional<Mcu::EncodersAndImuData> parse_encoders_and_imu(const std::string& reply) {
  std::istringstream iss(reply);
  Mcu::EncodersAndImuData data{};
  for (int& value : data.encoders_data) {
    iss >> value;
  }
  for (double& value : data.imu_data.orientation) {
    iss >> value;
  }
  for (double& value : data.imu_data.angular_velocity) {
    iss >> value;
  }
  for (double& value : data.imu_data.linear_acceleration) {
    iss >> value;
  }
  if (!fully_consumed(iss)) {
    return std::nullopt;
  }
  return data;
}

}  // namespace mcu_reply_parser
}  // namespace andino_base
