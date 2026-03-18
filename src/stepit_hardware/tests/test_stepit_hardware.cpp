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
  ON_CALL(*mock_driver, configure(Matcher<const ConfigCommand&>(_)))
      .WillByDefault(Return(AcknowledgeResponse{ Response::Status::Success }));
  EXPECT_CALL(*mock_driver, get_status(_)).WillOnce(Return(mocked_response));
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
}  // namespace stepit_driver::test
