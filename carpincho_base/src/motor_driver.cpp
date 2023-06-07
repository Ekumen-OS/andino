// Copyright 2023 Franco Cipollone
#include "carpincho_base/motor_driver.h"

#include <cstdlib>
#include <iostream>
#include <sstream>

namespace carpincho_base {

void MotorDriver::Setup(const std::string& serial_device, int32_t baud_rate, int32_t timeout_ms) {
  timeout_ms_ = timeout_ms;
  try {
    serial_port_.Open(serial_device);
  } catch (std::exception& e) {
    std::cout << e.what() << std::endl;
  }
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

bool MotorDriver::is_connected() const { return serial_port_.IsOpen(); }

void MotorDriver::SendEmptyMsg() { std::string response = SendMsg(""); }

MotorDriver::Encoders MotorDriver::ReadEncoderValues() {
  static const std::string delimiter = " ";

  const std::string response = SendMsg("e");
  const size_t del_pos = response.find(delimiter);
  const std::string token_1 = response.substr(0, del_pos).c_str();
  const std::string token_2 = response.substr(del_pos + delimiter.length()).c_str();
  return {std::atoi(token_1.c_str()), std::atoi(token_2.c_str())};
}

void MotorDriver::SetMotorValues(int val_1, int val_2) {
  std::stringstream ss;
  ss << "m " << val_1 << " " << val_2;
  SendMsg(ss.str());
}

void MotorDriver::SetPidValues(float k_p, float k_d, float k_i, float k_o) {
  std::stringstream ss;
  ss << "u " << k_p << ":" << k_d << ":" << k_i << ":" << k_o;
  SendMsg(ss.str());
}

std::string MotorDriver::SendMsg(const std::string& msg) {
  // Add carriage return to the message.
  const std::string msg_to_send = msg + '\r';
  // Send the message.
  serial_port_.Write(msg_to_send);

  // Get response from the motor driver.
  std::string response;
  try {
    serial_port_.ReadLine(response, '\n', timeout_ms_);
  } catch (LibSerial::ReadTimeout&) {
    std::cerr << "Response to " << msg << " timed out." << std::endl;
  }
  return response;
}

}  // namespace carpincho_base
