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
#include "andino_base/diffdrive_andino.h"

#include <cmath>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp_lifecycle/state.hpp>

namespace andino_base {
namespace {

using ::testing::_;
using ::testing::NiceMock;
using ::testing::Return;
using CallbackReturn = hardware_interface::CallbackReturn;
using ReturnType = hardware_interface::return_type;

class MockMcu : public Mcu {
 public:
  MOCK_METHOD(bool, is_connected, (), (const, override));
  MOCK_METHOD(bool, reset_encoders, (), (override));
  MOCK_METHOD(std::optional<EncodersData>, read_encoders, (), (override));
  MOCK_METHOD(std::optional<bool>, is_imu_available, (), (override));
  MOCK_METHOD(std::optional<EncodersAndImuData>, read_encoders_and_imu, (), (override));
  MOCK_METHOD(bool, set_motors_speed, (int left_motor_speed, int right_motor_speed), (override));
  MOCK_METHOD(bool, set_motors_pwm, (int left_motor_pwm, int right_motor_pwm), (override));
  MOCK_METHOD(bool, set_pid_tuning_gains, (float kp, float kd, float ki, float ko), (override));
  // Called on destruction, to check when the MCU is released.
  MOCK_METHOD(void, Die, ());
  ~MockMcu() override { Die(); }
};

constexpr int kTicksPerRev = 360;
constexpr double kRadsPerTick = 2. * M_PI / kTicksPerRev;
constexpr double kTolerance = 1e-9;

hardware_interface::ComponentInfo MakeJoint(const std::string& name, const std::vector<std::string>& command_interfaces,
                                            const std::vector<std::string>& state_interfaces) {
  hardware_interface::ComponentInfo joint;
  joint.name = name;
  joint.type = "joint";
  for (const std::string& interface_name : command_interfaces) {
    hardware_interface::InterfaceInfo interface_info;
    interface_info.name = interface_name;
    joint.command_interfaces.push_back(interface_info);
  }
  for (const std::string& interface_name : state_interfaces) {
    hardware_interface::InterfaceInfo interface_info;
    interface_info.name = interface_name;
    joint.state_interfaces.push_back(interface_info);
  }
  return joint;
}

hardware_interface::ComponentInfo MakeWheelJoint(const std::string& name) {
  return MakeJoint(name, {hardware_interface::HW_IF_VELOCITY},
                   {hardware_interface::HW_IF_POSITION, hardware_interface::HW_IF_VELOCITY});
}

hardware_interface::HardwareInfo MakeHardwareInfo() {
  hardware_interface::HardwareInfo info;
  info.name = "DiffDriveAndino";
  info.type = "system";
  info.hardware_parameters = {
      {"left_wheel_name", "left_wheel_joint"},
      {"right_wheel_name", "right_wheel_joint"},
      {"serial_device", "/dev/ttyTEST"},
      {"baud_rate", "115200"},
      {"timeout", "500"},
      {"enc_ticks_per_rev", std::to_string(kTicksPerRev)},
  };
  info.joints = {MakeWheelJoint("left_wheel_joint"), MakeWheelJoint("right_wheel_joint")};
  return info;
}

// The value accessors of the interface handles changed between ROS 2 distributions. These helpers
// use the ones available in all of the supported distributions.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
template <typename InterfaceT>
double GetValue(const InterfaceT& interface) {
  return interface.get_value();
}

void SetValue(hardware_interface::CommandInterface& interface, double value) {
  static_cast<void>(interface.set_value(value));
}
#pragma GCC diagnostic pop

class DiffDriveAndinoTest : public ::testing::Test {
 protected:
  DiffDriveAndinoTest()
      : mcu_owner_(std::make_unique<NiceMock<MockMcu>>()),
        mcu_(mcu_owner_.get()),
        diffdrive_([this](const DiffDriveAndino::Config& config) {
          factory_config_ = config;
          return std::move(mcu_owner_);
        }) {
    ON_CALL(*mcu_, is_connected()).WillByDefault(Return(true));
    ON_CALL(*mcu_, read_encoders()).WillByDefault(Return(Mcu::EncodersData{0, 0}));
    ON_CALL(*mcu_, set_motors_speed(_, _)).WillByDefault(Return(true));
  }

  // Initializes and configures the hardware interface, and exports its interfaces.
  void Configure() {
    ASSERT_EQ(diffdrive_.on_init(MakeHardwareInfo()), CallbackReturn::SUCCESS);
    ASSERT_EQ(diffdrive_.on_configure(state_), CallbackReturn::SUCCESS);
    for (hardware_interface::StateInterface& interface : diffdrive_.export_state_interfaces()) {
      state_interfaces_.emplace(interface.get_name(), std::move(interface));
    }
    for (hardware_interface::CommandInterface& interface : diffdrive_.export_command_interfaces()) {
      command_interfaces_.emplace(interface.get_name(), std::move(interface));
    }
  }

  double State(const std::string& name) const { return GetValue(state_interfaces_.at(name)); }

  void Command(const std::string& name, double value) { SetValue(command_interfaces_.at(name), value); }

  ReturnType Read(double period_secs = 0.1) {
    return diffdrive_.read(rclcpp::Time(0), rclcpp::Duration::from_seconds(period_secs));
  }

  ReturnType Write() { return diffdrive_.write(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.1)); }

  std::unique_ptr<NiceMock<MockMcu>> mcu_owner_;
  NiceMock<MockMcu>* mcu_;
  std::optional<DiffDriveAndino::Config> factory_config_;
  DiffDriveAndino diffdrive_;
  rclcpp_lifecycle::State state_;
  std::map<std::string, hardware_interface::StateInterface> state_interfaces_;
  std::map<std::string, hardware_interface::CommandInterface> command_interfaces_;
};

TEST_F(DiffDriveAndinoTest, InitSucceedsWithValidHardwareInfo) {
  EXPECT_EQ(diffdrive_.on_init(MakeHardwareInfo()), CallbackReturn::SUCCESS);
}

TEST_F(DiffDriveAndinoTest, InitUsesDefaultsForMissingParameters) {
  hardware_interface::HardwareInfo info = MakeHardwareInfo();
  info.hardware_parameters.erase("serial_device");
  info.hardware_parameters.erase("baud_rate");
  info.hardware_parameters.erase("timeout");

  ASSERT_EQ(diffdrive_.on_init(info), CallbackReturn::SUCCESS);
  ASSERT_EQ(diffdrive_.on_configure(state_), CallbackReturn::SUCCESS);

  const DiffDriveAndino::Config defaults;
  ASSERT_TRUE(factory_config_.has_value());
  EXPECT_EQ(factory_config_->serial_device, defaults.serial_device);
  EXPECT_EQ(factory_config_->baud_rate, defaults.baud_rate);
  EXPECT_EQ(factory_config_->timeout, defaults.timeout);
}

TEST_F(DiffDriveAndinoTest, InitFailsWithNonNumericParameter) {
  hardware_interface::HardwareInfo info = MakeHardwareInfo();
  info.hardware_parameters["baud_rate"] = "fast";

  EXPECT_EQ(diffdrive_.on_init(info), CallbackReturn::ERROR);
}

TEST_F(DiffDriveAndinoTest, InitFailsWithNonPositiveEncoderTicksPerRevolution) {
  hardware_interface::HardwareInfo info = MakeHardwareInfo();
  info.hardware_parameters["enc_ticks_per_rev"] = "0";

  EXPECT_EQ(diffdrive_.on_init(info), CallbackReturn::ERROR);
}

TEST_F(DiffDriveAndinoTest, InitFailsWithWrongNumberOfStateInterfaces) {
  hardware_interface::HardwareInfo info = MakeHardwareInfo();
  info.joints[0] =
      MakeJoint("left_wheel_joint", {hardware_interface::HW_IF_VELOCITY}, {hardware_interface::HW_IF_POSITION});

  EXPECT_EQ(diffdrive_.on_init(info), CallbackReturn::ERROR);
}

TEST_F(DiffDriveAndinoTest, InitFailsWithWrongNumberOfCommandInterfaces) {
  hardware_interface::HardwareInfo info = MakeHardwareInfo();
  info.joints[0] =
      MakeJoint("left_wheel_joint", {}, {hardware_interface::HW_IF_POSITION, hardware_interface::HW_IF_VELOCITY});

  EXPECT_EQ(diffdrive_.on_init(info), CallbackReturn::ERROR);
}

TEST_F(DiffDriveAndinoTest, InitFailsWithWrongCommandInterfaceType) {
  hardware_interface::HardwareInfo info = MakeHardwareInfo();
  info.joints[0] = MakeJoint("left_wheel_joint", {hardware_interface::HW_IF_POSITION},
                             {hardware_interface::HW_IF_POSITION, hardware_interface::HW_IF_VELOCITY});

  EXPECT_EQ(diffdrive_.on_init(info), CallbackReturn::ERROR);
}

TEST_F(DiffDriveAndinoTest, InitFailsWithWrongStateInterfaceTypes) {
  hardware_interface::HardwareInfo info = MakeHardwareInfo();
  info.joints[0] = MakeJoint("left_wheel_joint", {hardware_interface::HW_IF_VELOCITY},
                             {hardware_interface::HW_IF_POSITION, hardware_interface::HW_IF_EFFORT});

  EXPECT_EQ(diffdrive_.on_init(info), CallbackReturn::ERROR);
}

TEST_F(DiffDriveAndinoTest, ConfigurePassesParametersToMcuFactory) {
  ASSERT_EQ(diffdrive_.on_init(MakeHardwareInfo()), CallbackReturn::SUCCESS);
  ASSERT_EQ(diffdrive_.on_configure(state_), CallbackReturn::SUCCESS);

  ASSERT_TRUE(factory_config_.has_value());
  EXPECT_EQ(factory_config_->left_wheel_name, "left_wheel_joint");
  EXPECT_EQ(factory_config_->right_wheel_name, "right_wheel_joint");
  EXPECT_EQ(factory_config_->serial_device, "/dev/ttyTEST");
  EXPECT_EQ(factory_config_->baud_rate, 115200);
  EXPECT_EQ(factory_config_->timeout, 500);
  EXPECT_EQ(factory_config_->enc_ticks_per_rev, kTicksPerRev);
}

TEST_F(DiffDriveAndinoTest, ConfigureFailsWhenMcuCannotBeCreated) {
  DiffDriveAndino diffdrive([](const DiffDriveAndino::Config&) { return nullptr; });
  ASSERT_EQ(diffdrive.on_init(MakeHardwareInfo()), CallbackReturn::SUCCESS);

  EXPECT_EQ(diffdrive.on_configure(state_), CallbackReturn::ERROR);
}

TEST_F(DiffDriveAndinoTest, ExportsInterfacesForEachWheel) {
  Configure();

  EXPECT_EQ(state_interfaces_.size(), 4u);
  EXPECT_EQ(state_interfaces_.count("left_wheel_joint/position"), 1u);
  EXPECT_EQ(state_interfaces_.count("left_wheel_joint/velocity"), 1u);
  EXPECT_EQ(state_interfaces_.count("right_wheel_joint/position"), 1u);
  EXPECT_EQ(state_interfaces_.count("right_wheel_joint/velocity"), 1u);
  EXPECT_EQ(command_interfaces_.size(), 2u);
  EXPECT_EQ(command_interfaces_.count("left_wheel_joint/velocity"), 1u);
  EXPECT_EQ(command_interfaces_.count("right_wheel_joint/velocity"), 1u);
}

TEST_F(DiffDriveAndinoTest, ActivateStartsFromCurrentEncoderReading) {
  Configure();
  EXPECT_CALL(*mcu_, read_encoders()).WillOnce(Return(Mcu::EncodersData{100, 200}));

  ASSERT_EQ(diffdrive_.on_activate(state_), CallbackReturn::SUCCESS);

  EXPECT_NEAR(State("left_wheel_joint/position"), 100 * kRadsPerTick, kTolerance);
  EXPECT_NEAR(State("right_wheel_joint/position"), 200 * kRadsPerTick, kTolerance);

  // The first read after activation must not report the jump from zero as a velocity.
  EXPECT_CALL(*mcu_, read_encoders()).WillOnce(Return(Mcu::EncodersData{100, 200}));
  ASSERT_EQ(Read(), ReturnType::OK);
  EXPECT_NEAR(State("left_wheel_joint/velocity"), 0., kTolerance);
  EXPECT_NEAR(State("right_wheel_joint/velocity"), 0., kTolerance);
}

TEST_F(DiffDriveAndinoTest, ActivateFailsWhenEncodersCannotBeRead) {
  Configure();
  EXPECT_CALL(*mcu_, read_encoders()).WillOnce(Return(std::nullopt));

  EXPECT_EQ(diffdrive_.on_activate(state_), CallbackReturn::ERROR);
}

TEST_F(DiffDriveAndinoTest, ReadComputesPositionAndVelocity) {
  Configure();
  ASSERT_EQ(diffdrive_.on_activate(state_), CallbackReturn::SUCCESS);
  EXPECT_CALL(*mcu_, read_encoders()).WillOnce(Return(Mcu::EncodersData{36, 72}));

  ASSERT_EQ(Read(0.5), ReturnType::OK);

  EXPECT_NEAR(State("left_wheel_joint/position"), 36 * kRadsPerTick, kTolerance);
  EXPECT_NEAR(State("right_wheel_joint/position"), 72 * kRadsPerTick, kTolerance);
  EXPECT_NEAR(State("left_wheel_joint/velocity"), 36 * kRadsPerTick / 0.5, kTolerance);
  EXPECT_NEAR(State("right_wheel_joint/velocity"), 72 * kRadsPerTick / 0.5, kTolerance);
}

TEST_F(DiffDriveAndinoTest, ReadHandlesNegativeEncoderCounts) {
  Configure();
  ASSERT_EQ(diffdrive_.on_activate(state_), CallbackReturn::SUCCESS);
  EXPECT_CALL(*mcu_, read_encoders()).WillOnce(Return(Mcu::EncodersData{-10, -20}));

  ASSERT_EQ(Read(0.1), ReturnType::OK);

  EXPECT_NEAR(State("left_wheel_joint/position"), -10 * kRadsPerTick, kTolerance);
  EXPECT_NEAR(State("right_wheel_joint/position"), -20 * kRadsPerTick, kTolerance);
  EXPECT_NEAR(State("left_wheel_joint/velocity"), -10 * kRadsPerTick / 0.1, kTolerance);
  EXPECT_NEAR(State("right_wheel_joint/velocity"), -20 * kRadsPerTick / 0.1, kTolerance);
}

TEST_F(DiffDriveAndinoTest, ReadFailsAndKeepsStateWhenEncodersCannotBeRead) {
  Configure();
  ASSERT_EQ(diffdrive_.on_activate(state_), CallbackReturn::SUCCESS);
  EXPECT_CALL(*mcu_, read_encoders()).WillOnce(Return(Mcu::EncodersData{36, 72}));
  ASSERT_EQ(Read(0.5), ReturnType::OK);
  EXPECT_CALL(*mcu_, read_encoders()).WillOnce(Return(std::nullopt));

  EXPECT_EQ(Read(0.5), ReturnType::ERROR);

  EXPECT_NEAR(State("left_wheel_joint/position"), 36 * kRadsPerTick, kTolerance);
  EXPECT_NEAR(State("right_wheel_joint/position"), 72 * kRadsPerTick, kTolerance);
  EXPECT_NEAR(State("left_wheel_joint/velocity"), 36 * kRadsPerTick / 0.5, kTolerance);
  EXPECT_NEAR(State("right_wheel_joint/velocity"), 72 * kRadsPerTick / 0.5, kTolerance);
}

TEST_F(DiffDriveAndinoTest, ReadWithZeroPeriodKeepsVelocityFinite) {
  Configure();
  ASSERT_EQ(diffdrive_.on_activate(state_), CallbackReturn::SUCCESS);
  EXPECT_CALL(*mcu_, read_encoders()).WillOnce(Return(Mcu::EncodersData{36, 72}));

  ASSERT_EQ(Read(0.), ReturnType::OK);

  EXPECT_NEAR(State("left_wheel_joint/position"), 36 * kRadsPerTick, kTolerance);
  EXPECT_TRUE(std::isfinite(State("left_wheel_joint/velocity")));
  EXPECT_TRUE(std::isfinite(State("right_wheel_joint/velocity")));
}

TEST_F(DiffDriveAndinoTest, ReadFailsWhenMcuIsDisconnected) {
  Configure();
  ON_CALL(*mcu_, is_connected()).WillByDefault(Return(false));

  EXPECT_EQ(Read(), ReturnType::ERROR);
}

TEST_F(DiffDriveAndinoTest, ReadAndWriteFailWhenNotConfigured) {
  ASSERT_EQ(diffdrive_.on_init(MakeHardwareInfo()), CallbackReturn::SUCCESS);

  EXPECT_EQ(Read(), ReturnType::ERROR);
  EXPECT_EQ(Write(), ReturnType::ERROR);
}

TEST_F(DiffDriveAndinoTest, WriteConvertsVelocityToTicksPerSecond) {
  Configure();
  ASSERT_EQ(diffdrive_.on_activate(state_), CallbackReturn::SUCCESS);
  Command("left_wheel_joint/velocity", 10.7 * kRadsPerTick);
  Command("right_wheel_joint/velocity", -10.7 * kRadsPerTick);
  EXPECT_CALL(*mcu_, set_motors_speed(11, -11)).WillOnce(Return(true));

  EXPECT_EQ(Write(), ReturnType::OK);
}

TEST_F(DiffDriveAndinoTest, WriteFailsWhenMotorsSpeedIsNotAcknowledged) {
  Configure();
  ASSERT_EQ(diffdrive_.on_activate(state_), CallbackReturn::SUCCESS);
  EXPECT_CALL(*mcu_, set_motors_speed(_, _)).WillOnce(Return(false));

  EXPECT_EQ(Write(), ReturnType::ERROR);
}

TEST_F(DiffDriveAndinoTest, WriteFailsWhenMcuIsDisconnected) {
  Configure();
  ON_CALL(*mcu_, is_connected()).WillByDefault(Return(false));
  EXPECT_CALL(*mcu_, set_motors_speed(_, _)).Times(0);

  EXPECT_EQ(Write(), ReturnType::ERROR);
}

TEST_F(DiffDriveAndinoTest, DeactivateStopsTheMotors) {
  Configure();
  ASSERT_EQ(diffdrive_.on_activate(state_), CallbackReturn::SUCCESS);
  EXPECT_CALL(*mcu_, set_motors_speed(0, 0)).WillOnce(Return(true));

  EXPECT_EQ(diffdrive_.on_deactivate(state_), CallbackReturn::SUCCESS);
}

TEST_F(DiffDriveAndinoTest, DeactivateFailsWhenMotorsCannotBeStopped) {
  Configure();
  ASSERT_EQ(diffdrive_.on_activate(state_), CallbackReturn::SUCCESS);
  EXPECT_CALL(*mcu_, set_motors_speed(0, 0)).WillOnce(Return(false));

  EXPECT_EQ(diffdrive_.on_deactivate(state_), CallbackReturn::ERROR);
}

TEST_F(DiffDriveAndinoTest, ShutdownStopsTheMotorsAndReleasesTheMcu) {
  Configure();
  ASSERT_EQ(diffdrive_.on_activate(state_), CallbackReturn::SUCCESS);
  EXPECT_CALL(*mcu_, set_motors_speed(0, 0)).WillOnce(Return(true));

  ASSERT_EQ(diffdrive_.on_shutdown(state_), CallbackReturn::SUCCESS);

  EXPECT_EQ(Read(), ReturnType::ERROR);
}

TEST_F(DiffDriveAndinoTest, CleanupReleasesTheMcu) {
  Configure();
  ::testing::MockFunction<void()> cleanup_done;
  {
    ::testing::InSequence sequence;
    EXPECT_CALL(*mcu_, Die());
    EXPECT_CALL(cleanup_done, Call());
  }

  EXPECT_EQ(diffdrive_.on_cleanup(state_), CallbackReturn::SUCCESS);
  cleanup_done.Call();
}

}  // namespace
}  // namespace andino_base
