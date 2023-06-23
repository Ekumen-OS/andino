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
#include <chrono>
#include <iostream>
#include <sstream>
#include <thread>

#include <gflags/gflags.h>

#include "andino_base/motor_driver.h"

DEFINE_string(serial_port, "/dev/ttyUSB0", "Serial port");
DEFINE_int32(baud_rate, 57600, "Baud rate");
DEFINE_int32(timeout_ms, 1000, "Timeout in milliseconds for receiving a response from the Microcontroller");

DEFINE_string(msg, "e", "Motor driver message(default read encoders)");

namespace andino_base {
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
}  // namespace andino_base

int main(int argc, char* argv[]) { return andino_base::applications::Main(argc, argv); }
