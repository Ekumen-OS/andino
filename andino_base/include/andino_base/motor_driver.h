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
#pragma once

#include <array>
#include <cstdint>

#include <libserial/SerialPort.h>

namespace andino_base {

/// \brief Class to handle serial communication with the motor driver
/// It is used to send commands to the motor driver and read encoder values.
/// The use:
/// 1. Create an instance of the class.
/// 2. Call Setup() to initialize the serial connection.
/// 3. Use api to send commands to the motor driver.
class MotorDriver {
 public:
  /// @brief Type to store the encoder values.
  using Encoders = std::array<int, 2>;

  /// @brief Default constructor.
  MotorDriver() = default;

  /// @param[in] serial_device Path to the serial device(eg. /dev/ttyACM0)
  /// @param[in] baud_rate Baud rate of the serial connection(eg. 57600)
  /// @param[in] timeout_ms Timeout in milliseconds.
  void Setup(const std::string& serial_device, int32_t baud_rate, int32_t timeout_ms);

  /// @brief Send an empty message to the motor driver. The use of this function is to
  ///       ensure that the motor driver is ready to receive a new command.
  void SendEmptyMsg();

  /// @brief Read the encoder values from the motor driver.
  ///       First value is the left encoder, second value is the right encoder.
  /// @returns The encoder values.
  Encoders ReadEncoderValues();

  /// @brief Set the motor values.
  ///        The unit of the values is in encoder ticks per revolution.
  /// @param val_1 Value for the first motor.
  /// @param val_2 Value for the second motor.
  void SetMotorValues(int val_1, int val_2);

  /// @brief Set the PID values.
  /// @param k_p Proportional gain.
  /// @param k_d Derivative gain.
  /// @param k_i Integral gain.
  /// @param k_o Offset gain.
  void SetPidValues(float k_p, float k_d, float k_i, float k_o);

  /// @brief Check if the serial connection is open.
  /// @return True if the serial connection is open, false otherwise.
  bool is_connected() const;

  /// @brief Send a message to the motor driver and read the response.
  ///        The message is sent with a carriage return appended to it.
  /// @param[in] msg_to_send Message to send to the motor driver.
  /// @returns The response from the motor driver.
  std::string SendMsg(const std::string& msg_to_send);

 private:
  // Underlying serial connection.
  LibSerial::SerialPort serial_port_;

  int32_t timeout_ms_{25};
};

}  // namespace andino_base
