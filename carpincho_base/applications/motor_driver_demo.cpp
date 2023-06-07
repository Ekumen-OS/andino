// Copyright 2023 Franco Cipollone
#include <chrono>
#include <iostream>
#include <sstream>
#include <thread>

#include <gflags/gflags.h>

#include "carpincho_base/motor_driver.h"

DEFINE_string(serial_port, "/dev/ttyUSB0", "Serial port");
DEFINE_int32(baud_rate, 57600, "Baud rate");
DEFINE_int32(timeout_ms, 1000, "Timeout in milliseconds for receiving a response from the Microcontroller");

DEFINE_string(msg, "e", "Motor driver message(default read encoders)");

namespace carpincho_base {
namespace applications {

// Returns a string with the usage message.
std::string GetUsageMessage() {
  std::stringstream ss;
  ss << "CLI for easy test of the MotorDriver class" << std::endl << std::endl;
  ss << "  motor_driver_demo --serial_port=/dev/ttyUSB0 --msg='e' " << std::endl << std::endl;
  ss << "  motor_driver_demo --msg='o 255 255' " << std::endl << std::endl;
  return ss.str();
}

int Main(int argc, char* argv[]) {
  gflags::SetUsageMessage(GetUsageMessage());
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  MotorDriver motor_driver;
  motor_driver.Setup(FLAGS_serial_port, FLAGS_baud_rate, FLAGS_timeout_ms);

  std::cout << "Motor driver is connected: " << (motor_driver.is_connected() ? "True" : "False") << std::endl;
  std::cout << "Waiting 2 seconds for the Microcontroller to be ready..." << std::endl;
  // When the Microcontroller is reset, it takes a few seconds to be ready.
  // And tipically when the serial port is opened, the Microcontroller is reset.
  std::this_thread::sleep_for(std::chrono::seconds(2));
  // Send message
  std::cout << "Sending message: " << FLAGS_msg << std::endl;
  const std::string response = motor_driver.SendMsg(FLAGS_msg);
  std::cout << "Response: " << response << std::endl;
  return 0;
}

}  // namespace applications
}  // namespace carpincho_base

int main(int argc, char* argv[]) { return carpincho_base::applications::Main(argc, argv); }
