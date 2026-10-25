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

#include <optional>
#include <string>

#include "andino_base/mcu.h"

namespace andino_base {

/// @brief Functions that parse the replies sent by andino_firmware.
/// Every parser rejects replies with missing, malformed or trailing values.
namespace mcu_reply_parser {

/// @brief Checks whether a reply is an error, which has the form "[ERROR] <description>".
/// @param reply Reply received from the microcontroller.
/// @returns True if the reply is an error, false otherwise.
bool is_error(const std::string& reply);

/// @brief Checks whether a reply is an acknowledgement, which has the form "[OK]".
/// @param reply Reply received from the microcontroller.
/// @returns True if the reply is an acknowledgement, false otherwise.
bool is_ok(const std::string& reply);

/// @brief Parses the reply to the encoders read command, which has the form "<left> <right>".
/// @param reply Reply received from the microcontroller.
/// @returns The encoder sensors data, or std::nullopt if the reply is malformed.
std::optional<Mcu::EncodersData> parse_encoders(const std::string& reply);

/// @brief Parses the reply to the IMU availability command, which is either "0" or "1".
/// @param reply Reply received from the microcontroller.
/// @returns Whether there is an IMU sensor available, or std::nullopt if the reply is malformed.
std::optional<bool> parse_imu_available(const std::string& reply);

/// @brief Parses the reply to the encoders and IMU read command, which has the form
/// "<left> <right> <qx> <qy> <qz> <qw> <wx> <wy> <wz> <ax> <ay> <az>".
/// @param reply Reply received from the microcontroller.
/// @returns The encoder sensors and IMU sensor data, or std::nullopt if the reply is malformed.
std::optional<Mcu::EncodersAndImuData> parse_encoders_and_imu(const std::string& reply);

}  // namespace mcu_reply_parser
}  // namespace andino_base
