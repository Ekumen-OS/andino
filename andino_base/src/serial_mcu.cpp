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
#include "andino_base/serial_mcu.h"

#include <chrono>
#include <iostream>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <thread>

#include "andino_base/mcu_reply_parser.h"

namespace andino_base {

namespace {

// Maps a baud rate value to the matching LibSerial enumerator.
std::optional<LibSerial::BaudRate> to_libserial_baud_rate(int32_t baud_rate) {
  switch (baud_rate) {
    case 9600:
      return LibSerial::BaudRate::BAUD_9600;
    case 19200:
      return LibSerial::BaudRate::BAUD_19200;
    case 38400:
      return LibSerial::BaudRate::BAUD_38400;
    case 57600:
      return LibSerial::BaudRate::BAUD_57600;
    case 115200:
      return LibSerial::BaudRate::BAUD_115200;
    default:
      return std::nullopt;
  }
}

}  // namespace

std::unique_ptr<SerialMcu> SerialMcu::open(const std::string& serial_device, int32_t baud_rate, int32_t timeout_ms) {
  const std::optional<LibSerial::BaudRate> libserial_baud_rate = to_libserial_baud_rate(baud_rate);
  if (!libserial_baud_rate.has_value()) {
    std::cerr << "Unsupported baud rate: " << baud_rate << std::endl;
    return nullptr;
  }

  // The constructor is private, so std::make_unique cannot be used.
  std::unique_ptr<SerialMcu> serial_mcu(new SerialMcu());
  serial_mcu->timeout_ms_ = timeout_ms;
  LibSerial::SerialPort& serial_port = serial_mcu->serial_port_;
  try {
    serial_port.Open(serial_device);

    std::cout << "Waiting 2 seconds for the Microcontroller to be ready..." << std::endl;
    // When the Microcontroller is reset, it takes a few seconds to be ready.
    // And tipically when the serial port is opened, the Microcontroller is reset.
    std::this_thread::sleep_for(std::chrono::seconds(2));

    // Configure the serial port.
    serial_port.SetBaudRate(*libserial_baud_rate);
    serial_port.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
    serial_port.SetParity(LibSerial::Parity::PARITY_NONE);
    serial_port.SetStopBits(LibSerial::StopBits::STOP_BITS_1);
    serial_port.SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_NONE);
    serial_port.SetDTR(false);
    serial_port.SetRTS(false);
    // Flush buffers.
    serial_port.FlushIOBuffers();
  } catch (const std::exception& e) {
    std::cerr << "Failed to open serial device " << serial_device << ": " << e.what() << std::endl;
    return nullptr;
  }
  return serial_mcu;
}

bool SerialMcu::is_connected() const { return serial_port_.IsOpen(); }

bool SerialMcu::reset_encoders() { return send_command("rstenc"); }

std::optional<SerialMcu::EncodersData> SerialMcu::read_encoders() {
  const std::optional<std::string> response = send_message("getenc");
  if (!response.has_value()) {
    return std::nullopt;
  }
  return mcu_reply_parser::parse_encoders(*response);
}

std::optional<bool> SerialMcu::is_imu_available() {
  const std::optional<std::string> response = send_message("hasimu");
  if (!response.has_value()) {
    return std::nullopt;
  }
  return mcu_reply_parser::parse_imu_available(*response);
}

std::optional<SerialMcu::EncodersAndImuData> SerialMcu::read_encoders_and_imu() {
  const std::optional<std::string> response = send_message("getencimu");
  if (!response.has_value()) {
    return std::nullopt;
  }
  return mcu_reply_parser::parse_encoders_and_imu(*response);
}

bool SerialMcu::set_motors_speed(int left_motor_speed, int right_motor_speed) {
  std::stringstream ss;
  ss << "setspd " << left_motor_speed << " " << right_motor_speed;
  return send_command(ss.str());
}

bool SerialMcu::set_motors_pwm(int left_motor_pwm, int right_motor_pwm) {
  std::stringstream ss;
  ss << "setpwm " << left_motor_pwm << " " << right_motor_pwm;
  return send_command(ss.str());
}

bool SerialMcu::set_pid_tuning_gains(float kp, float kd, float ki, float ko) {
  std::stringstream ss;
  ss << "setpid " << kp << " " << kd << " " << ki << " " << ko;
  return send_command(ss.str());
}

std::optional<std::string> SerialMcu::send_message(const std::string& msg) {
  if (!serial_port_.IsOpen()) {
    std::cerr << "Cannot send " << msg << ": the serial port is not open." << std::endl;
    return std::nullopt;
  }

  std::string response;
  try {
    // Add carriage return to the message and send it.
    serial_port_.Write(msg + '\r');
    // Get response from the microcontroller.
    serial_port_.ReadLine(response, '\n', timeout_ms_);
  } catch (const LibSerial::ReadTimeout&) {
    std::cerr << "Response to " << msg << " timed out." << std::endl;
    return std::nullopt;
  } catch (const std::exception& e) {
    std::cerr << "Failed to send " << msg << ": " << e.what() << std::endl;
    return std::nullopt;
  }
  if (mcu_reply_parser::is_error(response)) {
    std::cerr << "Command " << msg << " failed: " << response;
    return std::nullopt;
  }
  return response;
}

bool SerialMcu::send_command(const std::string& msg) {
  const std::optional<std::string> response = send_message(msg);
  if (!response.has_value()) {
    return false;
  }
  if (!mcu_reply_parser::is_ok(*response)) {
    std::cerr << "Unexpected response to " << msg << ": " << *response;
    return false;
  }
  return true;
}

}  // namespace andino_base
