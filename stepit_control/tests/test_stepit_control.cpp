/*
 * Copyright (c) 2022, Giovanni Remigi
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <gtest/gtest.h>

#include <stepit_control/data_utils.hpp>
#include <stepit_control/msgs/msgs.hpp>
#include <stepit_control/stepit_control.hpp>

#include <fake/fake_hardware_info.hpp>
#include <mock/mock_command_interface.hpp>

#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <hardware_interface/resource_manager.hpp>
#include <hardware_interface/types/lifecycle_state_names.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <ros2_control_test_assets/components_urdfs.hpp>
#include <ros2_control_test_assets/descriptions.hpp>

#include <cmath>
#include <string>
#include <unordered_map>
#include <vector>

namespace stepit_control::test
{
using ::testing::_;
using ::testing::An;
using ::testing::Matcher;
using ::testing::Return;
using ::testing::SaveArg;

/**
 * This test generates a minimal xacro robot configuration and loads the
 * hardware interface plugin.
 */
TEST(TestStepitControl, load_urdf)
{
  std::string urdf_control_ =
      R"(
        <ros2_control name="StepitControl" type="system">
          <hardware>
            <plugin>stepit_control/StepitControl</plugin>
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
        </ros2_control>
      )";

  auto urdf = ros2_control_test_assets::urdf_head + urdf_control_ + ros2_control_test_assets::urdf_tail;
  hardware_interface::ResourceManager rm(urdf);

  // Check interfaces
  EXPECT_EQ(1u, rm.system_components_size());
  ASSERT_EQ(4u, rm.state_interface_keys().size());
  EXPECT_TRUE(rm.state_interface_exists("joint1/position"));
  EXPECT_TRUE(rm.state_interface_exists("joint2/position"));
  EXPECT_TRUE(rm.state_interface_exists("joint1/velocity"));
  EXPECT_TRUE(rm.state_interface_exists("joint2/velocity"));

  ASSERT_EQ(4u, rm.command_interface_keys().size());
  EXPECT_TRUE(rm.command_interface_exists("joint1/position"));
  EXPECT_TRUE(rm.command_interface_exists("joint2/position"));
  EXPECT_TRUE(rm.command_interface_exists("joint1/velocity"));
  EXPECT_TRUE(rm.command_interface_exists("joint2/velocity"));
}

/**
 * Test the read method using a mocked data interface. The interface returns a
 * hard-wired response containing information about position and velocity of
 * two motors. The read method parses this information and use it to populate
 * position and velocity of its state interface.
 * The test check if the state interface contains the expected values.
 */
TEST(TestStepitControl, read_status)
{
  // clang-format off
  const MotorStatusResponse mocked_response{
      1,     // request ID
      Response::Status::Success,
      {
          MotorStatusResponse::MotorState{ 0, 32100, 0.5, 150 },     // Motor 0 status
          MotorStatusResponse::MotorState{ 1, -6500, 0.75, 150000 }  // Motor 1 status
      }
  };
  // clang-format on

  auto mock_command_interface = std::make_unique<MockCommandInterface>();
  ON_CALL(*mock_command_interface, init()).WillByDefault(Return());
  ON_CALL(*mock_command_interface, send(Matcher<const MotorConfigCommand&>(_)))
      .WillByDefault(Return(AcknowledgeResponse{ 0x00, Response::Status::Success }));
  EXPECT_CALL(*mock_command_interface, send(_, An<const MotorStatusQuery&>())).WillOnce(Return(mocked_response));

  auto stepit_control = std::make_unique<stepit_control::StepitControl>(std::move(mock_command_interface));

  // Load the component.
  hardware_interface::ResourceManager rm;
  rm.import_component(std::move(stepit_control), FakeHardwareInfo{});

  // Connect the hardware.
  rclcpp_lifecycle::State state{ lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
                                 hardware_interface::lifecycle_state_names::ACTIVE };
  rm.set_component_state("StepitControl", state);

  // Invoke a read command.
  const rclcpp::Time time;
  const rclcpp::Duration period = rclcpp::Duration::from_seconds(0);
  rm.read(time, period);

  // Read state interface values.
  hardware_interface::LoanedStateInterface joint1_position_state = rm.claim_state_interface("joint1/position");
  hardware_interface::LoanedStateInterface joint2_position_state = rm.claim_state_interface("joint2/position");
  hardware_interface::LoanedStateInterface joint1_velocity_state = rm.claim_state_interface("joint1/velocity");
  hardware_interface::LoanedStateInterface joint2_velocity_state = rm.claim_state_interface("joint2/velocity");

  ASSERT_EQ(32100, joint1_position_state.get_value());
  ASSERT_EQ(-6500, joint2_position_state.get_value());
  ASSERT_EQ(0.5, joint1_velocity_state.get_value());
  ASSERT_EQ(0.75, joint2_velocity_state.get_value());
}

/**
 * In this test we set velocities goals on the hardware interface.
 * We execute a write operation and expect to see one data frame delivered
 * to the actual hardware, setting velocities.
 */
TEST(TestStepitControl, write_velocities)
{
  // clang-format off
  const MotorVelocityCommand expected_request{
    1,  // request ID
    {
        MotorVelocityCommand::Goal{ 0, 0.5 },  // Motor 0 goal
        MotorVelocityCommand::Goal{ 1, 0.75 }  // Motor 1 goal
    }
  };
  // clang-format on

  MotorVelocityCommand actual_request{ 0, {} };

  auto mock_command_interface = std::make_unique<MockCommandInterface>();
  ON_CALL(*mock_command_interface, init()).WillByDefault(Return());
  ON_CALL(*mock_command_interface, send(Matcher<const MotorConfigCommand&>(_)))
      .WillByDefault(Return(AcknowledgeResponse{ 0x00, Response::Status::Success }));
  EXPECT_CALL(*mock_command_interface, send(_, Matcher<const MotorVelocityCommand&>(_)))
      .WillOnce(DoAll(SaveArg<1>(&actual_request), Return(AcknowledgeResponse{ 0x01, Response::Status::Success })));

  auto stepit_control = std::make_unique<stepit_control::StepitControl>(std::move(mock_command_interface));

  // Load the component.
  hardware_interface::ResourceManager rm;
  rm.import_component(std::move(stepit_control), FakeHardwareInfo{});

  // Connect the hardware.
  rclcpp_lifecycle::State state{ lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
                                 hardware_interface::lifecycle_state_names::ACTIVE };
  rm.set_component_state("StepitControl", state);

  // Write velocity values.
  hardware_interface::LoanedCommandInterface joint1_velocity_command = rm.claim_command_interface("joint1/velocity");
  hardware_interface::LoanedCommandInterface joint2_velocity_command = rm.claim_command_interface("joint2/velocity");
  joint1_velocity_command.set_value(0.5);
  joint2_velocity_command.set_value(0.75);

  // Invoke a write command.
  const rclcpp::Time time;
  const rclcpp::Duration period = rclcpp::Duration::from_seconds(0);
  rm.write(time, period);

  ASSERT_EQ(actual_request.request_id(), expected_request.request_id());
  ASSERT_EQ(actual_request.goals().size(), actual_request.goals().size());
  ASSERT_EQ(actual_request.goals()[0].motor_id(), actual_request.goals()[0].motor_id());
  ASSERT_EQ(actual_request.goals()[0].velocity(), actual_request.goals()[0].velocity());
  ASSERT_EQ(actual_request.goals()[1].motor_id(), actual_request.goals()[1].motor_id());
  ASSERT_EQ(actual_request.goals()[1].velocity(), actual_request.goals()[1].velocity());
}

/**
 * In this test we set positions goals on the hardware interface.
 * We execute a write operation and expect to see one data frame delivered
 * to the actual hardware, setting positions.
 */
TEST(TestStepitControl, write_positions)
{
  // clang-format off
  const MotorPositionCommand expected_request{
    1,  // request ID
    {
        MotorPositionCommand::Goal{ 0, 0.5 },  // Motor 0 goal
        MotorPositionCommand::Goal{ 1, 0.75 }  // Motor 1 goal
    }
  };
  // clang-format on

  MotorPositionCommand actual_request{ 0, {} };

  auto mock_command_interface = std::make_unique<MockCommandInterface>();
  ON_CALL(*mock_command_interface, init()).WillByDefault(Return());
  ON_CALL(*mock_command_interface, send(Matcher<const MotorConfigCommand&>(_)))
      .WillByDefault(Return(AcknowledgeResponse{ 0x00, Response::Status::Success }));
  EXPECT_CALL(*mock_command_interface, send(_, Matcher<const MotorPositionCommand&>(_)))
      .WillOnce(DoAll(SaveArg<1>(&actual_request), Return(AcknowledgeResponse{ 0x01, Response::Status::Success })));

  auto stepit_control = std::make_unique<stepit_control::StepitControl>(std::move(mock_command_interface));

  // Load the component.
  hardware_interface::ResourceManager rm;
  rm.import_component(std::move(stepit_control), FakeHardwareInfo{});

  // Connect the hardware.
  rclcpp_lifecycle::State state{ lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
                                 hardware_interface::lifecycle_state_names::ACTIVE };
  rm.set_component_state("StepitControl", state);

  // Write position values.
  hardware_interface::LoanedCommandInterface joint1_position_command = rm.claim_command_interface("joint1/position");
  hardware_interface::LoanedCommandInterface joint2_position_command = rm.claim_command_interface("joint2/position");
  joint1_position_command.set_value(0.5);
  joint2_position_command.set_value(0.75);

  // Invoke a write command.
  const rclcpp::Time time;
  const rclcpp::Duration period = rclcpp::Duration::from_seconds(0);
  rm.write(time, period);

  ASSERT_EQ(actual_request.request_id(), expected_request.request_id());
  ASSERT_EQ(actual_request.goals().size(), actual_request.goals().size());
  ASSERT_EQ(actual_request.goals()[0].motor_id(), actual_request.goals()[0].motor_id());
  ASSERT_EQ(actual_request.goals()[0].position(), actual_request.goals()[0].position());
  ASSERT_EQ(actual_request.goals()[1].motor_id(), actual_request.goals()[1].motor_id());
  ASSERT_EQ(actual_request.goals()[1].position(), actual_request.goals()[1].position());
}

/**
 * In this test we set positions goals on the hardware interface.
 * We execute a write operation and expect to see one data frame delivered
 * to the actual hardware, setting positions.
 */
TEST(TestStepitControl, configuration)
{
  // clang-format off
   const MotorConfigCommand expected_request{
     0,  // request ID
     {
         MotorConfigCommand::Param{ 0, 0.1, 0.2 },  // Motor 0 goal
         MotorConfigCommand::Param{ 1, 0.3, 0.4}    // Motor 1 goal
     }
   };
  // clang-format on

  const AcknowledgeResponse mocked_response{ 0x00, Response::Status::Success };
  MotorConfigCommand actual_request{ 0, {} };

  auto mock_command_interface = std::make_unique<MockCommandInterface>();
  ON_CALL(*mock_command_interface, init()).WillByDefault(Return());
  EXPECT_CALL(*mock_command_interface, send(Matcher<const MotorConfigCommand&>(_)))
      .WillOnce(DoAll(SaveArg<0>(&actual_request), Return(mocked_response)));

  auto stepit_control = std::make_unique<stepit_control::StepitControl>(std::move(mock_command_interface));

  // Load the component.
  hardware_interface::ResourceManager rm;
  rm.import_component(std::move(stepit_control), FakeHardwareInfo{});

  // Connect the hardware.
  rclcpp_lifecycle::State state{ lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
                                 hardware_interface::lifecycle_state_names::ACTIVE };
  rm.set_component_state("StepitControl", state);

  ASSERT_EQ(actual_request.request_id(), expected_request.request_id());
  ASSERT_EQ(actual_request.params().size(), actual_request.params().size());
  ASSERT_EQ(actual_request.params()[0].motor_id(), actual_request.params()[0].motor_id());
  ASSERT_EQ(actual_request.params()[0].acceleration(), actual_request.params()[0].acceleration());
  ASSERT_EQ(actual_request.params()[0].max_velocity(), actual_request.params()[0].max_velocity());
  ASSERT_EQ(actual_request.params()[1].motor_id(), actual_request.params()[1].motor_id());
  ASSERT_EQ(actual_request.params()[1].acceleration(), actual_request.params()[1].acceleration());
  ASSERT_EQ(actual_request.params()[1].max_velocity(), actual_request.params()[1].max_velocity());
}
}  // namespace stepit_control::test