// Copyright 2023 Giovanni Remigi
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Giovanni Remigi nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <gtest/gtest.h>

#include <cmath>
#include <map>
#include <memory>
#include <numeric>
#include <string>
#include <tuple>
#include <vector>

#include <stepit_hardware/stepit_hardware.hpp>

#include <cobs_serial/data_utils.hpp>

#include <fake/fake_hardware_info.hpp>
#include <mock/mock_driver.hpp>
#include <mock/mock_driver_factory.hpp>

#include <hardware_interface/resource_manager.hpp>
#include <hardware_interface/types/hardware_component_interface_params.hpp>
#include <hardware_interface/types/lifecycle_state_names.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <ros2_control_test_assets/components_urdfs.hpp>
#include <ros2_control_test_assets/descriptions.hpp>

namespace stepit_driver::test
{
using ::testing::_;
using ::testing::DoAll;
using ::testing::Matcher;
using ::testing::Return;
using ::testing::SaveArg;

/**
 * The info response of a controller reporting the limits FakeHardwareInfo
 * declares, which is what a healthy handshake returns.
 */
InfoResponse handshake_info(double max_acceleration = 12.5663706143592, double max_velocity = 18.8495559215388)
{
  std::vector<MotorLimits> limits;
  for (uint8_t id = 0; id < 5; id++)
  {
    limits.emplace_back(MotorLimits{ id, max_acceleration, max_velocity });
  }
  return InfoResponse{ Response::Status::Success, "STEPIT", Version{ 1, 0, 0 }, limits };
}

/**
 * A successful status response with as many motors as the joints declared
 * in FakeHardwareInfo, as returned by the controller during the handshake.
 */
StatusResponse handshake_status()
{
  // clang-format off
  return StatusResponse{
      Response::Status::Success,
      {
          MotorState{ 0, 0, 0, 0 },  // Motor 0 status
          MotorState{ 1, 0, 0, 0 },  // Motor 1 status
          MotorState{ 2, 0, 0, 0 },  // Motor 2 status
          MotorState{ 3, 0, 0, 0 },  // Motor 3 status
          MotorState{ 4, 0, 0, 0 }   // Motor 4 status
      }
  };
  // clang-format on
}

/**
 * Build a minimal HardwareInfo with one joint per entry in ids, so tests can
 * exercise custom motor-id assignments (e.g. a shuffled 0..N-1 order or an
 * out-of-range id).
 */
hardware_interface::HardwareInfo make_hardware_info(const std::vector<int>& ids)
{
  hardware_interface::HardwareInfo info;
  info.name = "StepitHardware";
  info.type = "system";
  info.hardware_plugin_name = "stepit_driver/StepitHardware";
  info.hardware_parameters = {
    { "usb_port", "/dev/ttyUSB0" },
    { "baud_rate", "9600" },
    { "timeout", "0.5" },
    { "use_dummy", "true" },
  };
  for (std::size_t i = 0; i < ids.size(); i++)
  {
    hardware_interface::ComponentInfo joint;
    joint.name = "joint" + std::to_string(i + 1);
    joint.type = "joint";
    hardware_interface::InterfaceInfo pos{ .name = "position", .size = 0, .parameters = {}, .enable_limits = false };
    hardware_interface::InterfaceInfo vel{ .name = "velocity", .size = 0, .parameters = {}, .enable_limits = false };
    joint.command_interfaces = { pos, vel };
    joint.state_interfaces = { pos, vel };
    joint.parameters = { { "id", std::to_string(ids[i]) },
                         { "acceleration", "3.14159" },
                         { "max_velocity", "6.28319" } };
    info.joints.emplace_back(joint);
  }
  return info;
}

/**
 * This test generates a minimal xacro robot configuration and loads the
 * hardware interface plugin.
 */
TEST(TestStepitHardware, load_urdf)
{
  // ros2_control_test_assets::urdf_head only defines joint1-joint3; build a full 5-joint URDF.
  const std::string urdf =
      R"(<?xml version="1.0" encoding="utf-8"?>
        <robot name="StepitRobot">

          <link name="world"/>
          <link name="link0"/>
          <joint name="joint1" type="revolute">
            <parent link="world"/><child link="link0"/>
            <limit lower="-3.14159" upper="3.14159" effort="10" velocity="6.28319"/>
          </joint>
          <link name="link1"/>
          <joint name="joint2" type="revolute">
            <parent link="link0"/><child link="link1"/>
            <limit lower="-3.14159" upper="3.14159" effort="10" velocity="6.28319"/>
          </joint>
          <link name="link2"/>
          <joint name="joint3" type="revolute">
            <parent link="link1"/><child link="link2"/>
            <limit lower="-3.14159" upper="3.14159" effort="10" velocity="6.28319"/>
          </joint>
          <link name="link3"/>
          <joint name="joint4" type="revolute">
            <parent link="link2"/><child link="link3"/>
            <limit lower="-3.14159" upper="3.14159" effort="10" velocity="6.28319"/>
          </joint>
          <link name="link4"/>
          <joint name="joint5" type="revolute">
            <parent link="link3"/><child link="link4"/>
            <limit lower="-3.14159" upper="3.14159" effort="10" velocity="6.28319"/>
          </joint>

          <ros2_control name="StepitHardware" type="system">
            <hardware>
              <plugin>stepit_driver/StepitHardware</plugin>
              <param name="usb_port">/dev/whatever</param>
              <param name="baud_rate">9600</param>
              <param name="use_dummy">true</param>
            </hardware>
            <joint name="joint1">
              <param name="id">0</param>
              <param name="acceleration">3.14159</param>
              <param name="max_velocity">6.28319</param>
              <command_interface name="position"/>
              <command_interface name="velocity"/>
              <state_interface name="position"/>
              <state_interface name="velocity"/>
            </joint>
            <joint name="joint2">
              <param name="id">1</param>
              <param name="acceleration">3.14159</param>
              <param name="max_velocity">6.28319</param>
              <command_interface name="position"/>
              <command_interface name="velocity"/>
              <state_interface name="position"/>
              <state_interface name="velocity"/>
            </joint>
            <joint name="joint3">
              <param name="id">2</param>
              <param name="acceleration">3.14159</param>
              <param name="max_velocity">6.28319</param>
              <command_interface name="position"/>
              <command_interface name="velocity"/>
              <state_interface name="position"/>
              <state_interface name="velocity"/>
            </joint>
            <joint name="joint4">
              <param name="id">3</param>
              <param name="acceleration">3.14159</param>
              <param name="max_velocity">6.28319</param>
              <command_interface name="position"/>
              <command_interface name="velocity"/>
              <state_interface name="position"/>
              <state_interface name="velocity"/>
            </joint>
            <joint name="joint5">
              <param name="id">4</param>
              <param name="acceleration">3.14159</param>
              <param name="max_velocity">6.28319</param>
              <command_interface name="position"/>
              <command_interface name="velocity"/>
              <state_interface name="position"/>
              <state_interface name="velocity"/>
            </joint>
          </ros2_control>
        </robot>
      )";

  auto clock = std::make_shared<rclcpp::Clock>();
  hardware_interface::ResourceManager rm(urdf, clock, rclcpp::get_logger("test_stepit_hardware"));

  // Check interfaces
  EXPECT_EQ(1u, rm.system_components_size());

  // 5 position and 5 velocity interfaces.
  ASSERT_EQ(10u, rm.state_interface_keys().size());

  EXPECT_TRUE(rm.state_interface_exists("joint1/position"));
  EXPECT_TRUE(rm.state_interface_exists("joint2/position"));
  EXPECT_TRUE(rm.state_interface_exists("joint3/position"));
  EXPECT_TRUE(rm.state_interface_exists("joint4/position"));
  EXPECT_TRUE(rm.state_interface_exists("joint5/position"));

  EXPECT_TRUE(rm.state_interface_exists("joint1/velocity"));
  EXPECT_TRUE(rm.state_interface_exists("joint2/velocity"));
  EXPECT_TRUE(rm.state_interface_exists("joint3/velocity"));
  EXPECT_TRUE(rm.state_interface_exists("joint4/velocity"));
  EXPECT_TRUE(rm.state_interface_exists("joint5/velocity"));

  EXPECT_TRUE(rm.command_interface_exists("joint1/position"));
  EXPECT_TRUE(rm.command_interface_exists("joint2/position"));
  EXPECT_TRUE(rm.command_interface_exists("joint3/position"));
  EXPECT_TRUE(rm.command_interface_exists("joint4/position"));
  EXPECT_TRUE(rm.command_interface_exists("joint5/position"));

  EXPECT_TRUE(rm.command_interface_exists("joint1/velocity"));
  EXPECT_TRUE(rm.command_interface_exists("joint2/velocity"));
  EXPECT_TRUE(rm.command_interface_exists("joint3/velocity"));
  EXPECT_TRUE(rm.command_interface_exists("joint4/velocity"));
  EXPECT_TRUE(rm.command_interface_exists("joint5/velocity"));
}

/**
 * Test the read method using a mocked data interface. The interface returns a
 * hard-wired response containing information about position and velocity of
 * two motors. The read method parses this information and use it to populate
 * position and velocity of its state interface.
 * The test check if the state interface contains the expected values.
 */
TEST(TestStepitHardware, read_status)
{
  // clang-format off
  const StatusResponse mocked_response{
      Response::Status::Success,
      {
          MotorState{ 0, 32100, 0.5, 150 },      // Motor 0 status
          MotorState{ 1, -6500, 0.75, 150000 },  // Motor 1 status
          MotorState{ 2, -6500, 0.75, 150000 },  // Motor 2 status
          MotorState{ 3, -6500, 0.75, 150000 },  // Motor 3 status
          MotorState{ 4, -6500, 0.75, 150000 }   // Motor 4 status
      }
  };
  // clang-format on

  auto mock_driver = std::make_unique<MockDriver>();
  ON_CALL(*mock_driver, connect()).WillByDefault(Return(true));
  ON_CALL(*mock_driver, get_info(_)).WillByDefault(Return(handshake_info()));
  ON_CALL(*mock_driver, configure(Matcher<const ConfigCommand&>(_)))
      .WillByDefault(Return(AcknowledgeResponse{ Response::Status::Success }));
  EXPECT_CALL(*mock_driver, get_status(_)).Times(2).WillRepeatedly(Return(mocked_response));
  auto mock_driver_factory = std::make_unique<MockDriverFactory>(std::move(mock_driver));

  auto stepit_hardware = std::make_unique<stepit_driver::StepitHardware>(std::move(mock_driver_factory));

  // Initialize the hardware.
  hardware_interface::HardwareComponentInterfaceParams init_params;
  init_params.hardware_info = FakeHardwareInfo{};
  ASSERT_EQ(hardware_interface::CallbackReturn::SUCCESS, stepit_hardware->on_init(init_params));

  // Capture state interface pointers (indices: 0=joint1/pos, 1=joint1/vel, 2=joint2/pos, ...).
  auto state_interfaces = stepit_hardware->on_export_state_interfaces();

  // Configure (opens serial port and sends config to hardware).
  rclcpp_lifecycle::State unconfigured{ lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED,
                                        hardware_interface::lifecycle_state_names::UNCONFIGURED };
  ASSERT_EQ(hardware_interface::CallbackReturn::SUCCESS, stepit_hardware->on_configure(unconfigured));

  // Activate.
  rclcpp_lifecycle::State inactive{ lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
                                    hardware_interface::lifecycle_state_names::INACTIVE };
  ASSERT_EQ(hardware_interface::CallbackReturn::SUCCESS, stepit_hardware->on_activate(inactive));

  // Invoke a read command.
  const rclcpp::Time time;
  const rclcpp::Duration period = rclcpp::Duration::from_seconds(0);
  ASSERT_EQ(hardware_interface::return_type::OK, stepit_hardware->read(time, period));

  // Verify state interface values (position index: 0, 2, 4, 6, 8; velocity index: 1, 3, 5, 7, 9).
  ASSERT_EQ(32100, state_interfaces[0]->get_optional().value());  // joint1/position
  ASSERT_EQ(-6500, state_interfaces[2]->get_optional().value());  // joint2/position
  ASSERT_EQ(-6500, state_interfaces[4]->get_optional().value());  // joint3/position
  ASSERT_EQ(-6500, state_interfaces[6]->get_optional().value());  // joint4/position
  ASSERT_EQ(-6500, state_interfaces[8]->get_optional().value());  // joint5/position

  ASSERT_EQ(0.5, state_interfaces[1]->get_optional().value());   // joint1/velocity
  ASSERT_EQ(0.75, state_interfaces[3]->get_optional().value());  // joint2/velocity
  ASSERT_EQ(0.75, state_interfaces[5]->get_optional().value());  // joint3/velocity
  ASSERT_EQ(0.75, state_interfaces[7]->get_optional().value());  // joint4/velocity
  ASSERT_EQ(0.75, state_interfaces[9]->get_optional().value());  // joint5/velocity
}

/**
 * In this test we set velocities goals on the hardware interface.
 * We execute a write operation and expect to see one data frame delivered
 * to the actual hardware, setting velocities.
 */
TEST(TestStepitHardware, write_velocities)
{
  // clang-format off
  const VelocityCommand expected_request{
    {
        VelocityGoal{ 0, 0.5 },   // Motor 0 goal
        VelocityGoal{ 1, 0.75 },  // Motor 1 goal
        VelocityGoal{ 2, 0.75 },  // Motor 2 goal
        VelocityGoal{ 3, 0.75 },  // Motor 3 goal
        VelocityGoal{ 4, 0.75 }   // Motor 4 goal
    }
  };
  // clang-format on

  VelocityCommand actual_request{ {} };

  auto mock_driver = std::make_unique<MockDriver>();
  ON_CALL(*mock_driver, connect()).WillByDefault(Return(true));
  ON_CALL(*mock_driver, get_info(_)).WillByDefault(Return(handshake_info()));
  ON_CALL(*mock_driver, get_status(_)).WillByDefault(Return(handshake_status()));
  ON_CALL(*mock_driver, configure(Matcher<const ConfigCommand&>(_)))
      .WillByDefault(Return(AcknowledgeResponse{ Response::Status::Success }));
  EXPECT_CALL(*mock_driver, set_velocity(_, Matcher<const VelocityCommand&>(_)))
      .WillOnce(DoAll(SaveArg<1>(&actual_request), Return(AcknowledgeResponse{ Response::Status::Success })));
  auto mock_driver_factory = std::make_unique<MockDriverFactory>(std::move(mock_driver));

  auto stepit_hardware = std::make_unique<stepit_driver::StepitHardware>(std::move(mock_driver_factory));

  // Initialize the hardware.
  hardware_interface::HardwareComponentInterfaceParams init_params;
  init_params.hardware_info = FakeHardwareInfo{};
  ASSERT_EQ(hardware_interface::CallbackReturn::SUCCESS, stepit_hardware->on_init(init_params));

  // Capture command interface pointers (indices: 0=joint1/pos, 1=joint1/vel, 2=joint2/pos, ...).
  auto command_interfaces = stepit_hardware->on_export_command_interfaces();

  // Configure and activate.
  rclcpp_lifecycle::State unconfigured{ lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED,
                                        hardware_interface::lifecycle_state_names::UNCONFIGURED };
  ASSERT_EQ(hardware_interface::CallbackReturn::SUCCESS, stepit_hardware->on_configure(unconfigured));
  rclcpp_lifecycle::State inactive{ lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
                                    hardware_interface::lifecycle_state_names::INACTIVE };
  ASSERT_EQ(hardware_interface::CallbackReturn::SUCCESS, stepit_hardware->on_activate(inactive));

  // Simulate a controller claiming the velocity command interfaces, as the
  // resource manager would do when activating a real velocity controller.
  ASSERT_EQ(hardware_interface::return_type::OK,
            stepit_hardware->perform_command_mode_switch(
                { "joint1/velocity", "joint2/velocity", "joint3/velocity", "joint4/velocity", "joint5/velocity" }, {}));

  // Write velocity values (velocity index: 1, 3, 5, 7, 9).
  std::ignore = command_interfaces[1]->set_value(0.5);
  std::ignore = command_interfaces[3]->set_value(0.75);
  std::ignore = command_interfaces[5]->set_value(0.75);
  std::ignore = command_interfaces[7]->set_value(0.75);
  std::ignore = command_interfaces[9]->set_value(0.75);

  // Invoke a write command.
  const rclcpp::Time time;
  const rclcpp::Duration period = rclcpp::Duration::from_seconds(0);
  ASSERT_EQ(hardware_interface::return_type::OK, stepit_hardware->write(time, period));

  ASSERT_EQ(expected_request.goals().size(), actual_request.goals().size());

  ASSERT_EQ(expected_request.goals()[0].motor_id(), actual_request.goals()[0].motor_id());
  ASSERT_EQ(expected_request.goals()[0].velocity(), actual_request.goals()[0].velocity());

  ASSERT_EQ(expected_request.goals()[1].motor_id(), actual_request.goals()[1].motor_id());
  ASSERT_EQ(expected_request.goals()[1].velocity(), actual_request.goals()[1].velocity());

  ASSERT_EQ(expected_request.goals()[2].motor_id(), actual_request.goals()[2].motor_id());
  ASSERT_EQ(expected_request.goals()[2].velocity(), actual_request.goals()[2].velocity());

  ASSERT_EQ(expected_request.goals()[3].motor_id(), actual_request.goals()[3].motor_id());
  ASSERT_EQ(expected_request.goals()[3].velocity(), actual_request.goals()[3].velocity());

  ASSERT_EQ(expected_request.goals()[4].motor_id(), actual_request.goals()[4].motor_id());
  ASSERT_EQ(expected_request.goals()[4].velocity(), actual_request.goals()[4].velocity());
}

/**
 * In this test we set positions goals on the hardware interface.
 * We execute a write operation and expect to see one data frame delivered
 * to the actual hardware, setting positions.
 */
TEST(TestStepitHardware, write_positions)
{
  // clang-format off
  const PositionCommand expected_request{
    {
        PositionGoal{ 0, 0.5 },   // Motor 0 goal
        PositionGoal{ 1, 0.75 },  // Motor 1 goal
        PositionGoal{ 2, 0.75 },  // Motor 2 goal
        PositionGoal{ 3, 0.75 },  // Motor 3 goal
        PositionGoal{ 4, 0.75 }   // Motor 4 goal
    }
  };
  // clang-format on

  PositionCommand actual_request{ {} };

  auto mock_driver = std::make_unique<MockDriver>();
  ON_CALL(*mock_driver, connect()).WillByDefault(Return(true));
  ON_CALL(*mock_driver, get_info(_)).WillByDefault(Return(handshake_info()));
  ON_CALL(*mock_driver, get_status(_)).WillByDefault(Return(handshake_status()));
  ON_CALL(*mock_driver, configure(Matcher<const ConfigCommand&>(_)))
      .WillByDefault(Return(AcknowledgeResponse{ Response::Status::Success }));
  EXPECT_CALL(*mock_driver, set_position(_, Matcher<const PositionCommand&>(_)))
      .WillOnce(DoAll(SaveArg<1>(&actual_request), Return(AcknowledgeResponse{ Response::Status::Success })));
  auto mock_driver_factory = std::make_unique<MockDriverFactory>(std::move(mock_driver));

  auto stepit_hardware = std::make_unique<stepit_driver::StepitHardware>(std::move(mock_driver_factory));

  // Initialize the hardware.
  hardware_interface::HardwareComponentInterfaceParams init_params;
  init_params.hardware_info = FakeHardwareInfo{};
  ASSERT_EQ(hardware_interface::CallbackReturn::SUCCESS, stepit_hardware->on_init(init_params));

  // Capture command interface pointers (indices: 0=joint1/pos, 1=joint1/vel, 2=joint2/pos, ...).
  auto command_interfaces = stepit_hardware->on_export_command_interfaces();

  // Configure and activate.
  rclcpp_lifecycle::State unconfigured{ lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED,
                                        hardware_interface::lifecycle_state_names::UNCONFIGURED };
  ASSERT_EQ(hardware_interface::CallbackReturn::SUCCESS, stepit_hardware->on_configure(unconfigured));
  rclcpp_lifecycle::State inactive{ lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
                                    hardware_interface::lifecycle_state_names::INACTIVE };
  ASSERT_EQ(hardware_interface::CallbackReturn::SUCCESS, stepit_hardware->on_activate(inactive));

  // Simulate a controller claiming the position command interfaces, as the
  // resource manager would do when activating a real position controller.
  ASSERT_EQ(hardware_interface::return_type::OK,
            stepit_hardware->perform_command_mode_switch(
                { "joint1/position", "joint2/position", "joint3/position", "joint4/position", "joint5/position" }, {}));

  // Write position values (position index: 0, 2, 4, 6, 8).
  std::ignore = command_interfaces[0]->set_value(0.5);
  std::ignore = command_interfaces[2]->set_value(0.75);
  std::ignore = command_interfaces[4]->set_value(0.75);
  std::ignore = command_interfaces[6]->set_value(0.75);
  std::ignore = command_interfaces[8]->set_value(0.75);

  // Invoke a write command.
  const rclcpp::Time time;
  const rclcpp::Duration period = rclcpp::Duration::from_seconds(0);
  ASSERT_EQ(hardware_interface::return_type::OK, stepit_hardware->write(time, period));

  ASSERT_EQ(expected_request.goals().size(), actual_request.goals().size());

  ASSERT_EQ(expected_request.goals()[0].motor_id(), actual_request.goals()[0].motor_id());
  ASSERT_EQ(expected_request.goals()[0].position(), actual_request.goals()[0].position());

  ASSERT_EQ(expected_request.goals()[1].motor_id(), actual_request.goals()[1].motor_id());
  ASSERT_EQ(expected_request.goals()[1].position(), actual_request.goals()[1].position());

  ASSERT_EQ(expected_request.goals()[2].motor_id(), actual_request.goals()[2].motor_id());
  ASSERT_EQ(expected_request.goals()[2].position(), actual_request.goals()[2].position());

  ASSERT_EQ(expected_request.goals()[3].motor_id(), actual_request.goals()[3].motor_id());
  ASSERT_EQ(expected_request.goals()[3].position(), actual_request.goals()[3].position());

  ASSERT_EQ(expected_request.goals()[4].motor_id(), actual_request.goals()[4].motor_id());
  ASSERT_EQ(expected_request.goals()[4].position(), actual_request.goals()[4].position());
}

/**
 * In this test we set positions goals on the hardware interface.
 * We execute a write operation and expect to see one data frame delivered
 * to the actual hardware, setting positions.
 */
TEST(TestStepitHardware, configuration)
{
  // FakeHardwareInfo sets acceleration=3.14159, max_velocity=6.28319 for all joints.
  // clang-format off
  const ConfigCommand expected_request{
    {
        ConfigParam{ 0, 3.14159, 6.28319 },  // Motor 0
        ConfigParam{ 1, 3.14159, 6.28319 },  // Motor 1
        ConfigParam{ 2, 3.14159, 6.28319 },  // Motor 2
        ConfigParam{ 3, 3.14159, 6.28319 },  // Motor 3
        ConfigParam{ 4, 3.14159, 6.28319 }   // Motor 4
    }
  };
  // clang-format on

  const AcknowledgeResponse mocked_response{ Response::Status::Success };
  ConfigCommand actual_request{ {} };

  auto mock_driver = std::make_unique<MockDriver>();
  ON_CALL(*mock_driver, connect()).WillByDefault(Return(true));
  ON_CALL(*mock_driver, get_info(_)).WillByDefault(Return(handshake_info()));
  ON_CALL(*mock_driver, get_status(_)).WillByDefault(Return(handshake_status()));
  EXPECT_CALL(*mock_driver, configure(Matcher<const ConfigCommand&>(_)))
      .WillOnce(DoAll(SaveArg<0>(&actual_request), Return(mocked_response)));
  auto mock_driver_factory = std::make_unique<MockDriverFactory>(std::move(mock_driver));

  auto stepit_hardware = std::make_unique<stepit_driver::StepitHardware>(std::move(mock_driver_factory));

  // Initialize the hardware.
  hardware_interface::HardwareComponentInterfaceParams init_params;
  init_params.hardware_info = FakeHardwareInfo{};
  ASSERT_EQ(hardware_interface::CallbackReturn::SUCCESS, stepit_hardware->on_init(init_params));

  // Configure (triggers driver->configure() call with joint parameters).
  rclcpp_lifecycle::State unconfigured{ lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED,
                                        hardware_interface::lifecycle_state_names::UNCONFIGURED };
  ASSERT_EQ(hardware_interface::CallbackReturn::SUCCESS, stepit_hardware->on_configure(unconfigured));

  ASSERT_EQ(expected_request.params().size(), actual_request.params().size());

  ASSERT_EQ(expected_request.params()[0].motor_id(), actual_request.params()[0].motor_id());
  ASSERT_EQ(expected_request.params()[0].acceleration(), actual_request.params()[0].acceleration());
  ASSERT_EQ(expected_request.params()[0].max_velocity(), actual_request.params()[0].max_velocity());

  ASSERT_EQ(expected_request.params()[1].motor_id(), actual_request.params()[1].motor_id());
  ASSERT_EQ(expected_request.params()[1].acceleration(), actual_request.params()[1].acceleration());
  ASSERT_EQ(expected_request.params()[1].max_velocity(), actual_request.params()[1].max_velocity());

  ASSERT_EQ(expected_request.params()[2].motor_id(), actual_request.params()[2].motor_id());
  ASSERT_EQ(expected_request.params()[2].acceleration(), actual_request.params()[2].acceleration());
  ASSERT_EQ(expected_request.params()[2].max_velocity(), actual_request.params()[2].max_velocity());

  ASSERT_EQ(expected_request.params()[3].motor_id(), actual_request.params()[3].motor_id());
  ASSERT_EQ(expected_request.params()[3].acceleration(), actual_request.params()[3].acceleration());
  ASSERT_EQ(expected_request.params()[3].max_velocity(), actual_request.params()[3].max_velocity());

  ASSERT_EQ(expected_request.params()[4].motor_id(), actual_request.params()[4].motor_id());
  ASSERT_EQ(expected_request.params()[4].acceleration(), actual_request.params()[4].acceleration());
  ASSERT_EQ(expected_request.params()[4].max_velocity(), actual_request.params()[4].max_velocity());
}

/**
 * In this test the driver cannot identify a StepIt controller on the serial
 * port: configuration must fail before anything is sent to the hardware.
 */
TEST(TestStepitHardware, configure_fails_when_not_connected)
{
  auto mock_driver = std::make_unique<MockDriver>();
  EXPECT_CALL(*mock_driver, connect()).WillOnce(Return(false));
  EXPECT_CALL(*mock_driver, get_info(_)).Times(0);
  EXPECT_CALL(*mock_driver, get_status(_)).Times(0);
  EXPECT_CALL(*mock_driver, configure(Matcher<const ConfigCommand&>(_))).Times(0);
  auto mock_driver_factory = std::make_unique<MockDriverFactory>(std::move(mock_driver));

  auto stepit_hardware = std::make_unique<stepit_driver::StepitHardware>(std::move(mock_driver_factory));

  hardware_interface::HardwareComponentInterfaceParams init_params;
  init_params.hardware_info = FakeHardwareInfo{};
  ASSERT_EQ(hardware_interface::CallbackReturn::SUCCESS, stepit_hardware->on_init(init_params));

  rclcpp_lifecycle::State unconfigured{ lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED,
                                        hardware_interface::lifecycle_state_names::UNCONFIGURED };
  ASSERT_EQ(hardware_interface::CallbackReturn::FAILURE, stepit_hardware->on_configure(unconfigured));
}

/**
 * In this test the controller answers the handshake and accepts the
 * configuration, but then reports a failure on the status query:
 * configuration must fail.
 */
TEST(TestStepitHardware, configure_fails_when_status_fails)
{
  const StatusResponse mocked_response{ Response::Status::Failure, {} };

  auto mock_driver = std::make_unique<MockDriver>();
  ON_CALL(*mock_driver, connect()).WillByDefault(Return(true));
  ON_CALL(*mock_driver, get_info(_)).WillByDefault(Return(handshake_info()));
  ON_CALL(*mock_driver, configure(Matcher<const ConfigCommand&>(_)))
      .WillByDefault(Return(AcknowledgeResponse{ Response::Status::Success }));
  EXPECT_CALL(*mock_driver, get_status(_)).WillOnce(Return(mocked_response));
  auto mock_driver_factory = std::make_unique<MockDriverFactory>(std::move(mock_driver));

  auto stepit_hardware = std::make_unique<stepit_driver::StepitHardware>(std::move(mock_driver_factory));

  hardware_interface::HardwareComponentInterfaceParams init_params;
  init_params.hardware_info = FakeHardwareInfo{};
  ASSERT_EQ(hardware_interface::CallbackReturn::SUCCESS, stepit_hardware->on_init(init_params));

  rclcpp_lifecycle::State unconfigured{ lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED,
                                        hardware_interface::lifecycle_state_names::UNCONFIGURED };
  ASSERT_EQ(hardware_interface::CallbackReturn::FAILURE, stepit_hardware->on_configure(unconfigured));
}

/**
 * In this test the controller accepts the configuration but drives fewer
 * motors than the joints declared in the URDF: configuration must fail.
 */
TEST(TestStepitHardware, configure_fails_on_motor_count_mismatch)
{
  // clang-format off
  const StatusResponse mocked_response{
      Response::Status::Success,
      {
          MotorState{ 0, 0, 0, 0 },  // Motor 0 status
          MotorState{ 1, 0, 0, 0 }   // Motor 1 status
      }
  };
  // clang-format on

  auto mock_driver = std::make_unique<MockDriver>();
  ON_CALL(*mock_driver, connect()).WillByDefault(Return(true));
  ON_CALL(*mock_driver, get_info(_)).WillByDefault(Return(handshake_info()));
  ON_CALL(*mock_driver, configure(Matcher<const ConfigCommand&>(_)))
      .WillByDefault(Return(AcknowledgeResponse{ Response::Status::Success }));
  EXPECT_CALL(*mock_driver, get_status(_)).WillOnce(Return(mocked_response));
  auto mock_driver_factory = std::make_unique<MockDriverFactory>(std::move(mock_driver));

  auto stepit_hardware = std::make_unique<stepit_driver::StepitHardware>(std::move(mock_driver_factory));

  hardware_interface::HardwareComponentInterfaceParams init_params;
  init_params.hardware_info = FakeHardwareInfo{};
  ASSERT_EQ(hardware_interface::CallbackReturn::SUCCESS, stepit_hardware->on_init(init_params));

  rclcpp_lifecycle::State unconfigured{ lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED,
                                        hardware_interface::lifecycle_state_names::UNCONFIGURED };
  ASSERT_EQ(hardware_interface::CallbackReturn::FAILURE, stepit_hardware->on_configure(unconfigured));
}

/**
 * In this test the handshake sees a valid set of motors, but a later read()
 * reports an id that is not one of the configured joints. The motor count
 * still matches, so only the id-range check can catch this; indexing joints_
 * with such an id would run past the end of the vector.
 */
TEST(TestStepitHardware, read_fails_on_out_of_range_motor_id)
{
  // clang-format off
  const StatusResponse bad_status{
      Response::Status::Success,
      {
          MotorState{ 0, 0, 0, 0 },
          MotorState{ 1, 0, 0, 0 },
          MotorState{ 2, 0, 0, 0 },
          MotorState{ 3, 0, 0, 0 },
          MotorState{ 99, 0, 0, 0 }  // id 99 is not a configured joint
      }
  };
  // clang-format on

  auto mock_driver = std::make_unique<MockDriver>();
  ON_CALL(*mock_driver, connect()).WillByDefault(Return(true));
  ON_CALL(*mock_driver, get_info(_)).WillByDefault(Return(handshake_info()));
  ON_CALL(*mock_driver, configure(Matcher<const ConfigCommand&>(_)))
      .WillByDefault(Return(AcknowledgeResponse{ Response::Status::Success }));
  EXPECT_CALL(*mock_driver, get_status(_))
      .WillOnce(Return(handshake_status()))  // on_configure handshake
      .WillOnce(Return(bad_status));         // read()
  auto mock_driver_factory = std::make_unique<MockDriverFactory>(std::move(mock_driver));

  auto stepit_hardware = std::make_unique<stepit_driver::StepitHardware>(std::move(mock_driver_factory));

  hardware_interface::HardwareComponentInterfaceParams init_params;
  init_params.hardware_info = FakeHardwareInfo{};
  ASSERT_EQ(hardware_interface::CallbackReturn::SUCCESS, stepit_hardware->on_init(init_params));

  rclcpp_lifecycle::State unconfigured{ lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED,
                                        hardware_interface::lifecycle_state_names::UNCONFIGURED };
  ASSERT_EQ(hardware_interface::CallbackReturn::SUCCESS, stepit_hardware->on_configure(unconfigured));
  rclcpp_lifecycle::State inactive{ lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
                                    hardware_interface::lifecycle_state_names::INACTIVE };
  ASSERT_EQ(hardware_interface::CallbackReturn::SUCCESS, stepit_hardware->on_activate(inactive));

  const rclcpp::Time time;
  const rclcpp::Duration period = rclcpp::Duration::from_seconds(0);
  EXPECT_EQ(hardware_interface::return_type::ERROR, stepit_hardware->read(time, period));
}

/**
 * In this test a later read() reports one id twice and omits another, while
 * the motor count still matches. Without a uniqueness check the duplicated id
 * would overwrite one joint and leave the omitted joint stale.
 */
TEST(TestStepitHardware, read_fails_on_duplicate_motor_id)
{
  // clang-format off
  const StatusResponse bad_status{
      Response::Status::Success,
      {
          MotorState{ 0, 0, 0, 0 },
          MotorState{ 0, 0, 0, 0 },  // id 0 reported twice
          MotorState{ 2, 0, 0, 0 },
          MotorState{ 3, 0, 0, 0 },
          MotorState{ 4, 0, 0, 0 }   // id 1 never reported
      }
  };
  // clang-format on

  auto mock_driver = std::make_unique<MockDriver>();
  ON_CALL(*mock_driver, connect()).WillByDefault(Return(true));
  ON_CALL(*mock_driver, get_info(_)).WillByDefault(Return(handshake_info()));
  ON_CALL(*mock_driver, configure(Matcher<const ConfigCommand&>(_)))
      .WillByDefault(Return(AcknowledgeResponse{ Response::Status::Success }));
  EXPECT_CALL(*mock_driver, get_status(_))
      .WillOnce(Return(handshake_status()))  // on_configure handshake
      .WillOnce(Return(bad_status));         // read()
  auto mock_driver_factory = std::make_unique<MockDriverFactory>(std::move(mock_driver));

  auto stepit_hardware = std::make_unique<stepit_driver::StepitHardware>(std::move(mock_driver_factory));

  hardware_interface::HardwareComponentInterfaceParams init_params;
  init_params.hardware_info = FakeHardwareInfo{};
  ASSERT_EQ(hardware_interface::CallbackReturn::SUCCESS, stepit_hardware->on_init(init_params));

  rclcpp_lifecycle::State unconfigured{ lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED,
                                        hardware_interface::lifecycle_state_names::UNCONFIGURED };
  ASSERT_EQ(hardware_interface::CallbackReturn::SUCCESS, stepit_hardware->on_configure(unconfigured));
  rclcpp_lifecycle::State inactive{ lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
                                    hardware_interface::lifecycle_state_names::INACTIVE };
  ASSERT_EQ(hardware_interface::CallbackReturn::SUCCESS, stepit_hardware->on_activate(inactive));

  const rclcpp::Time time;
  const rclcpp::Duration period = rclcpp::Duration::from_seconds(0);
  EXPECT_EQ(hardware_interface::return_type::ERROR, stepit_hardware->read(time, period));
}

/**
 * In this test the joints are configured with a shuffled 0..N-1 id order (a
 * real hardware scenario: the controller reports ids 0..N-1 but a joint may
 * own any of them) and the status reports them out of order. read() must
 * route each state to the joint that owns the id, not to the id's position in
 * the reported list.
 */
TEST(TestStepitHardware, read_routes_states_by_configured_id)
{
  // clang-format off
  const StatusResponse mocked_response{
      Response::Status::Success,
      {
          MotorState{ 3, 4000, 0.5, 0 },
          MotorState{ 0, 1000, 0.5, 0 },
          MotorState{ 4, 5000, 0.5, 0 },
          MotorState{ 1, 2000, 0.5, 0 },
          MotorState{ 2, 3000, 0.5, 0 }
      }
  };
  // clang-format on

  auto mock_driver = std::make_unique<MockDriver>();
  ON_CALL(*mock_driver, connect()).WillByDefault(Return(true));
  ON_CALL(*mock_driver, get_info(_)).WillByDefault(Return(handshake_info()));
  ON_CALL(*mock_driver, configure(Matcher<const ConfigCommand&>(_)))
      .WillByDefault(Return(AcknowledgeResponse{ Response::Status::Success }));
  EXPECT_CALL(*mock_driver, get_status(_)).Times(2).WillRepeatedly(Return(mocked_response));
  auto mock_driver_factory = std::make_unique<MockDriverFactory>(std::move(mock_driver));

  auto stepit_hardware = std::make_unique<stepit_driver::StepitHardware>(std::move(mock_driver_factory));

  hardware_interface::HardwareComponentInterfaceParams init_params;
  init_params.hardware_info = make_hardware_info({ 2, 0, 4, 1, 3 });
  ASSERT_EQ(hardware_interface::CallbackReturn::SUCCESS, stepit_hardware->on_init(init_params));

  auto state_interfaces = stepit_hardware->on_export_state_interfaces();

  rclcpp_lifecycle::State unconfigured{ lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED,
                                        hardware_interface::lifecycle_state_names::UNCONFIGURED };
  ASSERT_EQ(hardware_interface::CallbackReturn::SUCCESS, stepit_hardware->on_configure(unconfigured));
  rclcpp_lifecycle::State inactive{ lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
                                    hardware_interface::lifecycle_state_names::INACTIVE };
  ASSERT_EQ(hardware_interface::CallbackReturn::SUCCESS, stepit_hardware->on_activate(inactive));

  const rclcpp::Time time;
  const rclcpp::Duration period = rclcpp::Duration::from_seconds(0);
  ASSERT_EQ(hardware_interface::return_type::OK, stepit_hardware->read(time, period));

  // Each joint holds the position of the motor whose id it was configured with.
  ASSERT_EQ(3000, state_interfaces[0]->get_optional().value());  // joint1 (id 2)
  ASSERT_EQ(1000, state_interfaces[2]->get_optional().value());  // joint2 (id 0)
  ASSERT_EQ(5000, state_interfaces[4]->get_optional().value());  // joint3 (id 4)
  ASSERT_EQ(2000, state_interfaces[6]->get_optional().value());  // joint4 (id 1)
  ASSERT_EQ(4000, state_interfaces[8]->get_optional().value());  // joint5 (id 3)
}

/**
 * In this test the URDF declares a motor id outside the 0..N-1 range the
 * controller can report. on_init() must reject it with a clear error before
 * the hardware is ever opened, rather than failing later at configure time.
 */
TEST(TestStepitHardware, init_fails_on_out_of_range_motor_id)
{
  auto mock_driver = std::make_unique<MockDriver>();
  auto mock_driver_factory = std::make_unique<MockDriverFactory>(std::move(mock_driver));

  auto stepit_hardware = std::make_unique<stepit_driver::StepitHardware>(std::move(mock_driver_factory));

  hardware_interface::HardwareComponentInterfaceParams init_params;
  // 5 joints, but the last has id 5 which is out of range [0, 4].
  init_params.hardware_info = make_hardware_info({ 0, 1, 2, 3, 5 });
  EXPECT_EQ(hardware_interface::CallbackReturn::ERROR, stepit_hardware->on_init(init_params));
}

/**
 * In this test on_init() is called twice on the same instance. The derived
 * id-to-joint mapping must be rebuilt from scratch: were it kept, every
 * configured id would look like a duplicate of itself on the second call.
 */
TEST(TestStepitHardware, init_can_run_twice)
{
  auto mock_driver = std::make_unique<MockDriver>();
  auto mock_driver_factory = std::make_unique<MockDriverFactory>(std::move(mock_driver));

  auto stepit_hardware = std::make_unique<stepit_driver::StepitHardware>(std::move(mock_driver_factory));

  hardware_interface::HardwareComponentInterfaceParams init_params;
  init_params.hardware_info = FakeHardwareInfo{};
  ASSERT_EQ(hardware_interface::CallbackReturn::SUCCESS, stepit_hardware->on_init(init_params));
  EXPECT_EQ(hardware_interface::CallbackReturn::SUCCESS, stepit_hardware->on_init(init_params));
}

/**
 * In this test the URDF declares an acceleration above what the controller
 * reports it tolerates. Configuration must fail before anything is sent to
 * the hardware: an open loop stepper pushed beyond its limit loses steps
 * without anything noticing.
 */
TEST(TestStepitHardware, configure_fails_when_a_joint_exceeds_the_reported_limits)
{
  auto mock_driver = std::make_unique<MockDriver>();
  ON_CALL(*mock_driver, connect()).WillByDefault(Return(true));
  // FakeHardwareInfo declares acceleration 3.14159, so report half of it.
  EXPECT_CALL(*mock_driver, get_info(_)).WillOnce(Return(handshake_info(3.14159 / 2)));
  EXPECT_CALL(*mock_driver, configure(Matcher<const ConfigCommand&>(_))).Times(0);
  auto mock_driver_factory = std::make_unique<MockDriverFactory>(std::move(mock_driver));

  auto stepit_hardware = std::make_unique<stepit_driver::StepitHardware>(std::move(mock_driver_factory));

  hardware_interface::HardwareComponentInterfaceParams init_params;
  init_params.hardware_info = FakeHardwareInfo{};
  ASSERT_EQ(hardware_interface::CallbackReturn::SUCCESS, stepit_hardware->on_init(init_params));

  rclcpp_lifecycle::State unconfigured{ lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED,
                                        hardware_interface::lifecycle_state_names::UNCONFIGURED };
  ASSERT_EQ(hardware_interface::CallbackReturn::FAILURE, stepit_hardware->on_configure(unconfigured));
}

/**
 * In this test the controller reports no limits for one of the configured
 * motors, so the description cannot be checked against anything.
 */
TEST(TestStepitHardware, configure_fails_when_a_joint_has_no_reported_limits)
{
  // Limits for two motors only, while five joints are configured.
  const InfoResponse partial{ Response::Status::Success,
                              "STEPIT",
                              Version{ 1, 0, 0 },
                              { MotorLimits{ 0, 12.5663706143592, 18.8495559215388 },
                                MotorLimits{ 1, 12.5663706143592, 18.8495559215388 } } };

  auto mock_driver = std::make_unique<MockDriver>();
  ON_CALL(*mock_driver, connect()).WillByDefault(Return(true));
  EXPECT_CALL(*mock_driver, get_info(_)).WillOnce(Return(partial));
  EXPECT_CALL(*mock_driver, configure(Matcher<const ConfigCommand&>(_))).Times(0);
  auto mock_driver_factory = std::make_unique<MockDriverFactory>(std::move(mock_driver));

  auto stepit_hardware = std::make_unique<stepit_driver::StepitHardware>(std::move(mock_driver_factory));

  hardware_interface::HardwareComponentInterfaceParams init_params;
  init_params.hardware_info = FakeHardwareInfo{};
  ASSERT_EQ(hardware_interface::CallbackReturn::SUCCESS, stepit_hardware->on_init(init_params));

  rclcpp_lifecycle::State unconfigured{ lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED,
                                        hardware_interface::lifecycle_state_names::UNCONFIGURED };
  ASSERT_EQ(hardware_interface::CallbackReturn::FAILURE, stepit_hardware->on_configure(unconfigured));
}

/**
 * In this test the controller fails to report its limits at all.
 */
TEST(TestStepitHardware, configure_fails_when_limits_cannot_be_read)
{
  auto mock_driver = std::make_unique<MockDriver>();
  ON_CALL(*mock_driver, connect()).WillByDefault(Return(true));
  EXPECT_CALL(*mock_driver, get_info(_)).WillOnce(Return(InfoResponse{ Response::Status::Failure, "", Version{}, {} }));
  EXPECT_CALL(*mock_driver, configure(Matcher<const ConfigCommand&>(_))).Times(0);
  auto mock_driver_factory = std::make_unique<MockDriverFactory>(std::move(mock_driver));

  auto stepit_hardware = std::make_unique<stepit_driver::StepitHardware>(std::move(mock_driver_factory));

  hardware_interface::HardwareComponentInterfaceParams init_params;
  init_params.hardware_info = FakeHardwareInfo{};
  ASSERT_EQ(hardware_interface::CallbackReturn::SUCCESS, stepit_hardware->on_init(init_params));

  rclcpp_lifecycle::State unconfigured{ lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED,
                                        hardware_interface::lifecycle_state_names::UNCONFIGURED };
  ASSERT_EQ(hardware_interface::CallbackReturn::FAILURE, stepit_hardware->on_configure(unconfigured));
}

/**
 * In this test the URDF declares more joints than motor_states_are_valid()
 * can track in its bitmask. on_init() must reject it rather than shift past
 * the width of the mask.
 */
TEST(TestStepitHardware, init_fails_beyond_the_supported_joint_count)
{
  auto mock_driver = std::make_unique<MockDriver>();
  auto mock_driver_factory = std::make_unique<MockDriverFactory>(std::move(mock_driver));

  auto stepit_hardware = std::make_unique<stepit_driver::StepitHardware>(std::move(mock_driver_factory));

  // 33 joints with ids 0..32: every id is in range and unique, so only the
  // bitmask bound can reject this.
  std::vector<int> ids(33);
  std::iota(ids.begin(), ids.end(), 0);

  hardware_interface::HardwareComponentInterfaceParams init_params;
  init_params.hardware_info = make_hardware_info(ids);
  EXPECT_EQ(hardware_interface::CallbackReturn::ERROR, stepit_hardware->on_init(init_params));
}

/**
 * In this test the controller rejects a velocity command. write() has to
 * report the failure so that ros2_control deactivates the component: the
 * acknowledgement used to be discarded, leaving the control loop running as
 * though the goal had been accepted.
 */
TEST(TestStepitHardware, write_reports_a_rejected_velocity_command)
{
  auto mock_driver = std::make_unique<MockDriver>();
  ON_CALL(*mock_driver, connect()).WillByDefault(Return(true));
  ON_CALL(*mock_driver, get_info(_)).WillByDefault(Return(handshake_info()));
  ON_CALL(*mock_driver, get_status(_)).WillByDefault(Return(handshake_status()));
  ON_CALL(*mock_driver, configure(Matcher<const ConfigCommand&>(_)))
      .WillByDefault(Return(AcknowledgeResponse{ Response::Status::Success }));
  EXPECT_CALL(*mock_driver, set_velocity(_, Matcher<const VelocityCommand&>(_)))
      .WillOnce(Return(AcknowledgeResponse{ Response::Status::Failure }));
  auto mock_driver_factory = std::make_unique<MockDriverFactory>(std::move(mock_driver));

  auto stepit_hardware = std::make_unique<stepit_driver::StepitHardware>(std::move(mock_driver_factory));

  hardware_interface::HardwareComponentInterfaceParams init_params;
  init_params.hardware_info = FakeHardwareInfo{};
  ASSERT_EQ(hardware_interface::CallbackReturn::SUCCESS, stepit_hardware->on_init(init_params));
  auto command_interfaces = stepit_hardware->on_export_command_interfaces();

  rclcpp_lifecycle::State unconfigured{ lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED,
                                        hardware_interface::lifecycle_state_names::UNCONFIGURED };
  ASSERT_EQ(hardware_interface::CallbackReturn::SUCCESS, stepit_hardware->on_configure(unconfigured));
  rclcpp_lifecycle::State inactive{ lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
                                    hardware_interface::lifecycle_state_names::INACTIVE };
  ASSERT_EQ(hardware_interface::CallbackReturn::SUCCESS, stepit_hardware->on_activate(inactive));

  ASSERT_EQ(hardware_interface::return_type::OK,
            stepit_hardware->perform_command_mode_switch(
                { "joint1/velocity", "joint2/velocity", "joint3/velocity", "joint4/velocity", "joint5/velocity" }, {}));

  std::ignore = command_interfaces[1]->set_value(0.5);

  const rclcpp::Time time;
  const rclcpp::Duration period = rclcpp::Duration::from_seconds(0);
  EXPECT_EQ(hardware_interface::return_type::ERROR, stepit_hardware->write(time, period));
}

/**
 * The same for a position command.
 */
TEST(TestStepitHardware, write_reports_a_rejected_position_command)
{
  auto mock_driver = std::make_unique<MockDriver>();
  ON_CALL(*mock_driver, connect()).WillByDefault(Return(true));
  ON_CALL(*mock_driver, get_info(_)).WillByDefault(Return(handshake_info()));
  ON_CALL(*mock_driver, get_status(_)).WillByDefault(Return(handshake_status()));
  ON_CALL(*mock_driver, configure(Matcher<const ConfigCommand&>(_)))
      .WillByDefault(Return(AcknowledgeResponse{ Response::Status::Success }));
  EXPECT_CALL(*mock_driver, set_position(_, Matcher<const PositionCommand&>(_)))
      .WillOnce(Return(AcknowledgeResponse{ Response::Status::Failure }));
  auto mock_driver_factory = std::make_unique<MockDriverFactory>(std::move(mock_driver));

  auto stepit_hardware = std::make_unique<stepit_driver::StepitHardware>(std::move(mock_driver_factory));

  hardware_interface::HardwareComponentInterfaceParams init_params;
  init_params.hardware_info = FakeHardwareInfo{};
  ASSERT_EQ(hardware_interface::CallbackReturn::SUCCESS, stepit_hardware->on_init(init_params));
  auto command_interfaces = stepit_hardware->on_export_command_interfaces();

  rclcpp_lifecycle::State unconfigured{ lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED,
                                        hardware_interface::lifecycle_state_names::UNCONFIGURED };
  ASSERT_EQ(hardware_interface::CallbackReturn::SUCCESS, stepit_hardware->on_configure(unconfigured));
  rclcpp_lifecycle::State inactive{ lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
                                    hardware_interface::lifecycle_state_names::INACTIVE };
  ASSERT_EQ(hardware_interface::CallbackReturn::SUCCESS, stepit_hardware->on_activate(inactive));

  ASSERT_EQ(hardware_interface::return_type::OK,
            stepit_hardware->perform_command_mode_switch(
                { "joint1/position", "joint2/position", "joint3/position", "joint4/position", "joint5/position" }, {}));

  std::ignore = command_interfaces[0]->set_value(1.0);

  const rclcpp::Time time;
  const rclcpp::Duration period = rclcpp::Duration::from_seconds(0);
  EXPECT_EQ(hardware_interface::return_type::ERROR, stepit_hardware->write(time, period));
}

/**
 * Motion commands received by the driver of make_active_hardware().
 */
struct SentCommands
{
  std::vector<VelocityCommand> velocities;
  std::vector<PositionCommand> positions;
};

/**
 * Build, configure and activate a StepitHardware on FakeHardwareInfo, with a
 * driver that records every motion command in sent and acknowledges it with
 * the given status.
 */
std::unique_ptr<StepitHardware> make_active_hardware(SentCommands& sent,
                                                     Response::Status ack = Response::Status::Success)
{
  auto mock_driver = std::make_unique<MockDriver>();
  ON_CALL(*mock_driver, connect()).WillByDefault(Return(true));
  ON_CALL(*mock_driver, get_info(_)).WillByDefault(Return(handshake_info()));
  ON_CALL(*mock_driver, get_status(_)).WillByDefault(Return(handshake_status()));
  ON_CALL(*mock_driver, configure(Matcher<const ConfigCommand&>(_)))
      .WillByDefault(Return(AcknowledgeResponse{ Response::Status::Success }));
  ON_CALL(*mock_driver, set_velocity(_, Matcher<const VelocityCommand&>(_)))
      .WillByDefault([&sent, ack](const rclcpp::Time&, const VelocityCommand& command) {
        sent.velocities.push_back(command);
        return AcknowledgeResponse{ ack };
      });
  ON_CALL(*mock_driver, set_position(_, Matcher<const PositionCommand&>(_)))
      .WillByDefault([&sent, ack](const rclcpp::Time&, const PositionCommand& command) {
        sent.positions.push_back(command);
        return AcknowledgeResponse{ ack };
      });
  auto mock_driver_factory = std::make_unique<MockDriverFactory>(std::move(mock_driver));

  auto stepit_hardware = std::make_unique<stepit_driver::StepitHardware>(std::move(mock_driver_factory));

  hardware_interface::HardwareComponentInterfaceParams init_params;
  init_params.hardware_info = FakeHardwareInfo{};
  EXPECT_EQ(hardware_interface::CallbackReturn::SUCCESS, stepit_hardware->on_init(init_params));

  rclcpp_lifecycle::State unconfigured{ lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED,
                                        hardware_interface::lifecycle_state_names::UNCONFIGURED };
  EXPECT_EQ(hardware_interface::CallbackReturn::SUCCESS, stepit_hardware->on_configure(unconfigured));
  rclcpp_lifecycle::State inactive{ lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
                                    hardware_interface::lifecycle_state_names::INACTIVE };
  EXPECT_EQ(hardware_interface::CallbackReturn::SUCCESS, stepit_hardware->on_activate(inactive));
  return stepit_hardware;
}

/**
 * The goals of a velocity command, by motor id.
 */
std::map<uint8_t, double> goals_of(const VelocityCommand& command)
{
  std::map<uint8_t, double> goals;
  for (const auto& goal : command.goals())
  {
    goals[goal.motor_id()] = goal.velocity();
  }
  return goals;
}

const std::vector<std::string> kVelocityInterfaces = { "joint1/velocity", "joint2/velocity", "joint3/velocity",
                                                       "joint4/velocity", "joint5/velocity" };
const std::vector<std::string> kPositionInterfaces = { "joint1/position", "joint2/position", "joint3/position",
                                                       "joint4/position", "joint5/position" };
const std::map<uint8_t, double> kAllStopped = { { 0, 0.0 }, { 1, 0.0 }, { 2, 0.0 }, { 3, 0.0 }, { 4, 0.0 } };

/**
 * Deactivating the hardware must bring every motor to rest: the firmware
 * keeps executing the last goal it received, and its watchdog does not fire
 * while the status is being polled.
 */
TEST(TestStepitHardware, deactivate_stops_every_motor)
{
  SentCommands sent;
  auto stepit_hardware = make_active_hardware(sent);
  auto command_interfaces = stepit_hardware->on_export_command_interfaces();

  ASSERT_EQ(hardware_interface::return_type::OK, stepit_hardware->perform_command_mode_switch(kVelocityInterfaces, {}));
  std::ignore = command_interfaces[1]->set_value(0.5);
  const rclcpp::Time time;
  const rclcpp::Duration period = rclcpp::Duration::from_seconds(0);
  ASSERT_EQ(hardware_interface::return_type::OK, stepit_hardware->write(time, period));
  sent.velocities.clear();

  rclcpp_lifecycle::State active{ lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
                                  hardware_interface::lifecycle_state_names::ACTIVE };
  ASSERT_EQ(hardware_interface::CallbackReturn::SUCCESS, stepit_hardware->on_deactivate(active));

  ASSERT_EQ(1u, sent.velocities.size());
  EXPECT_EQ(kAllStopped, goals_of(sent.velocities[0]));
}

/**
 * A stop the controller refuses leaves the motors in an unknown state, so
 * deactivation reports it.
 */
TEST(TestStepitHardware, deactivate_reports_a_rejected_stop)
{
  SentCommands sent;
  auto stepit_hardware = make_active_hardware(sent, Response::Status::Failure);

  rclcpp_lifecycle::State active{ lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
                                  hardware_interface::lifecycle_state_names::ACTIVE };
  EXPECT_EQ(hardware_interface::CallbackReturn::ERROR, stepit_hardware->on_deactivate(active));
}

/**
 * When a controller releases one joint, the next write stops that motor once
 * and leaves the joints still under control alone.
 */
TEST(TestStepitHardware, releasing_a_velocity_interface_stops_the_motor)
{
  SentCommands sent;
  auto stepit_hardware = make_active_hardware(sent);
  auto command_interfaces = stepit_hardware->on_export_command_interfaces();

  ASSERT_EQ(hardware_interface::return_type::OK, stepit_hardware->perform_command_mode_switch(kVelocityInterfaces, {}));
  std::ignore = command_interfaces[1]->set_value(0.5);
  std::ignore = command_interfaces[3]->set_value(0.75);
  std::ignore = command_interfaces[5]->set_value(0.75);
  std::ignore = command_interfaces[7]->set_value(0.75);
  std::ignore = command_interfaces[9]->set_value(0.75);
  const rclcpp::Time time;
  const rclcpp::Duration period = rclcpp::Duration::from_seconds(0);
  ASSERT_EQ(hardware_interface::return_type::OK, stepit_hardware->write(time, period));
  sent.velocities.clear();

  // Release joint1 only. Its last command, 0.5, stays in the interface.
  ASSERT_EQ(hardware_interface::return_type::OK,
            stepit_hardware->perform_command_mode_switch({}, { "joint1/velocity" }));

  ASSERT_EQ(hardware_interface::return_type::OK, stepit_hardware->write(time, period));
  ASSERT_EQ(1u, sent.velocities.size());
  const std::map<uint8_t, double> expected = { { 0, 0.0 }, { 1, 0.75 }, { 2, 0.75 }, { 3, 0.75 }, { 4, 0.75 } };
  EXPECT_EQ(expected, goals_of(sent.velocities[0]));

  // The stop is sent once: afterwards joint1 is left out.
  ASSERT_EQ(hardware_interface::return_type::OK, stepit_hardware->write(time, period));
  ASSERT_EQ(2u, sent.velocities.size());
  EXPECT_EQ(0u, goals_of(sent.velocities[1]).count(0));
  EXPECT_EQ(4u, sent.velocities[1].goals().size());
}

/**
 * When the only controller is deactivated, every motor is stopped even though
 * no joint is commanded any more.
 */
TEST(TestStepitHardware, releasing_every_interface_stops_the_motors)
{
  SentCommands sent;
  auto stepit_hardware = make_active_hardware(sent);
  auto command_interfaces = stepit_hardware->on_export_command_interfaces();

  ASSERT_EQ(hardware_interface::return_type::OK, stepit_hardware->perform_command_mode_switch(kVelocityInterfaces, {}));
  std::ignore = command_interfaces[1]->set_value(0.5);
  const rclcpp::Time time;
  const rclcpp::Duration period = rclcpp::Duration::from_seconds(0);
  ASSERT_EQ(hardware_interface::return_type::OK, stepit_hardware->write(time, period));
  sent.velocities.clear();

  ASSERT_EQ(hardware_interface::return_type::OK, stepit_hardware->perform_command_mode_switch({}, kVelocityInterfaces));

  ASSERT_EQ(hardware_interface::return_type::OK, stepit_hardware->write(time, period));
  ASSERT_EQ(1u, sent.velocities.size());
  EXPECT_EQ(kAllStopped, goals_of(sent.velocities[0]));

  ASSERT_EQ(hardware_interface::return_type::OK, stepit_hardware->write(time, period));
  EXPECT_EQ(1u, sent.velocities.size());
  EXPECT_TRUE(sent.positions.empty());
}

/**
 * Switching from a velocity to a position controller that already commands
 * the joints hands them over without stopping them first.
 */
TEST(TestStepitHardware, switching_to_a_commanding_controller_does_not_stop_the_motors)
{
  SentCommands sent;
  auto stepit_hardware = make_active_hardware(sent);
  auto command_interfaces = stepit_hardware->on_export_command_interfaces();

  ASSERT_EQ(hardware_interface::return_type::OK, stepit_hardware->perform_command_mode_switch(kVelocityInterfaces, {}));
  std::ignore = command_interfaces[1]->set_value(0.5);
  const rclcpp::Time time;
  const rclcpp::Duration period = rclcpp::Duration::from_seconds(0);
  ASSERT_EQ(hardware_interface::return_type::OK, stepit_hardware->write(time, period));
  sent.velocities.clear();

  ASSERT_EQ(hardware_interface::return_type::OK,
            stepit_hardware->perform_command_mode_switch(kPositionInterfaces, kVelocityInterfaces));
  for (std::size_t i = 0; i < 10; i += 2)
  {
    std::ignore = command_interfaces[i]->set_value(1.0);
  }

  ASSERT_EQ(hardware_interface::return_type::OK, stepit_hardware->write(time, period));
  EXPECT_TRUE(sent.velocities.empty());
  ASSERT_EQ(1u, sent.positions.size());
  EXPECT_EQ(5u, sent.positions[0].goals().size());
}

/**
 * A controller that claims the joints but has not written a goal yet leaves
 * them uncommanded, so the motors are stopped rather than left running the
 * released controller's last goal.
 */
TEST(TestStepitHardware, switching_to_a_silent_controller_stops_the_motors)
{
  SentCommands sent;
  auto stepit_hardware = make_active_hardware(sent);
  auto command_interfaces = stepit_hardware->on_export_command_interfaces();

  ASSERT_EQ(hardware_interface::return_type::OK, stepit_hardware->perform_command_mode_switch(kVelocityInterfaces, {}));
  std::ignore = command_interfaces[1]->set_value(0.5);
  const rclcpp::Time time;
  const rclcpp::Duration period = rclcpp::Duration::from_seconds(0);
  ASSERT_EQ(hardware_interface::return_type::OK, stepit_hardware->write(time, period));
  sent.velocities.clear();

  // The position interfaces were never written, so they still hold NaN.
  ASSERT_EQ(hardware_interface::return_type::OK,
            stepit_hardware->perform_command_mode_switch(kPositionInterfaces, kVelocityInterfaces));

  ASSERT_EQ(hardware_interface::return_type::OK, stepit_hardware->write(time, period));
  ASSERT_EQ(1u, sent.velocities.size());
  EXPECT_EQ(kAllStopped, goals_of(sent.velocities[0]));
  EXPECT_TRUE(sent.positions.empty());
}

}  // namespace stepit_driver::test
