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
#include <cstdlib>
#include <iostream>
#include <sstream>
#include <thread>

namespace andino_base {

void SerialMcu::setup(const std::string& serial_device, int32_t baud_rate, int32_t timeout_ms) {
  timeout_ms_ = timeout_ms;
  try {
    serial_port_.Open(serial_device);
  } catch (std::exception& e) {
    std::cout << e.what() << std::endl;
  }

  std::cout << "Waiting 2 seconds for the Microcontroller to be ready..." << std::endl;
  // When the Microcontroller is reset, it takes a few seconds to be ready.
  // And tipically when the serial port is opened, the Microcontroller is reset.
  std::this_thread::sleep_for(std::chrono::seconds(2));

  // TODO: Use baud_rate from parameter.
  if (baud_rate != 57600) {
    std::cerr << "A baudrate different than 57600 is not supported yet." << std::endl;
  }
  // Configure the serial port.
  serial_port_.SetBaudRate(LibSerial::BaudRate::BAUD_57600);
  serial_port_.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
  serial_port_.SetParity(LibSerial::Parity::PARITY_NONE);
  serial_port_.SetStopBits(LibSerial::StopBits::STOP_BITS_1);
  serial_port_.SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_NONE);
  serial_port_.SetDTR(false);
  serial_port_.SetRTS(false);
  // Flush buffers.
  serial_port_.FlushIOBuffers();
}

bool SerialMcu::is_connected() const { return serial_port_.IsOpen(); }

void SerialMcu::reset_encoders() { send_message("r"); }

SerialMcu::EncodersData SerialMcu::read_encoders() {
  static const std::string delimiter = " ";

  const std::string response = send_message("e");
  const size_t del_pos = response.find(delimiter);
  const std::string token_1 = response.substr(0, del_pos).c_str();
  const std::string token_2 = response.substr(del_pos + delimiter.length()).c_str();
  return {std::atoi(token_1.c_str()), std::atoi(token_2.c_str())};
}

bool SerialMcu::is_imu_available() { return std::atoi(send_message("h").c_str()) != 0; }

SerialMcu::EncodersAndImuData SerialMcu::read_encoders_and_imu() {
  static const std::string delimiter = " ";

  const std::string response = send_message("i");

  std::istringstream iss(response);
  EncodersAndImuData encoders_and_imu_data;
  iss >> encoders_and_imu_data.encoders_data[0] >> encoders_and_imu_data.encoders_data[1];
  iss >> encoders_and_imu_data.imu_data.orientation[0] >> encoders_and_imu_data.imu_data.orientation[1] >>
      encoders_and_imu_data.imu_data.orientation[2] >> encoders_and_imu_data.imu_data.orientation[3];
  iss >> encoders_and_imu_data.imu_data.angular_velocity[0] >> encoders_and_imu_data.imu_data.angular_velocity[1] >>
      encoders_and_imu_data.imu_data.angular_velocity[2];
  iss >> encoders_and_imu_data.imu_data.linear_acceleration[0] >>
      encoders_and_imu_data.imu_data.linear_acceleration[1] >> encoders_and_imu_data.imu_data.linear_acceleration[2];
  return encoders_and_imu_data;
}

void SerialMcu::set_motors_speed(int left_motor_speed, int right_motor_speed) {
  std::stringstream ss;
  ss << "m " << left_motor_speed << " " << right_motor_speed;
  send_message(ss.str());
}

void SerialMcu::set_motors_pwm(int left_motor_pwm, int right_motor_pwm) {
  std::stringstream ss;
  ss << "o " << left_motor_pwm << " " << right_motor_pwm;
  send_message(ss.str());
}

void SerialMcu::set_pid_tuning_gains(float kp, float kd, float ki, float ko) {
  std::stringstream ss;
  ss << "u " << kp << ":" << kd << ":" << ki << ":" << ko;
  send_message(ss.str());
}

std::string SerialMcu::send_message(const std::string& msg) {
  // Add carriage return to the message.
  const std::string msg_to_send = msg + '\r';
  // Send the message.
  serial_port_.Write(msg_to_send);

  // Get response from the microcontroller.
  std::string response;
  try {
    serial_port_.ReadLine(response, '\n', timeout_ms_);
  } catch (LibSerial::ReadTimeout&) {
    std::cerr << "Response to " << msg << " timed out." << std::endl;
  }
  return response;
}

}  // namespace andino_base
