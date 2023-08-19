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

#include <stepit_driver/msgs/msgs.hpp>
#include <stepit_driver/stepit_hardware.hpp>

#include <data_interface/data_utils.hpp>

#include <fake/fake_hardware_info.hpp>
#include <mock/mock_driver.hpp>
#include <mock/mock_driver_factory.hpp>

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

namespace stepit_driver::test
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
TEST(TestStepitHardware, load_urdf)
{
  std::string urdf_control_ =
      R"(
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
      )";

  auto urdf = ros2_control_test_assets::urdf_head + urdf_control_ + ros2_control_test_assets::urdf_tail;
  hardware_interface::ResourceManager rm(urdf);

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
          MotorState{ 0, 32100, 0.5, 150 },     // Motor 0 status
          MotorState{ 1, -6500, 0.75, 150000 }, // Motor 1 status
          MotorState{ 2, -6500, 0.75, 150000 }, // Motor 1 status
          MotorState{ 3, -6500, 0.75, 150000 }, // Motor 1 status
          MotorState{ 4, -6500, 0.75, 150000 }  // Motor 1 status
      }
  };
  // clang-format on

  auto mock_driver = std::make_unique<MockDriver>();
  ON_CALL(*mock_driver, connect()).WillByDefault(Return(true));
  ON_CALL(*mock_driver, send(Matcher<const ConfigCommand&>(_)))
      .WillByDefault(Return(AcknowledgeResponse{ Response::Status::Success }));
  EXPECT_CALL(*mock_driver, send(_, An<const StatusQuery&>())).WillOnce(Return(mocked_response));
  auto mock_driver_factory = std::make_unique<MockDriverFactory>(std::move(mock_driver));

  auto stepit_hardware = std::make_unique<stepit_driver::StepitHardware>(std::move(mock_driver_factory));

  // Load the component.
  hardware_interface::ResourceManager rm;
  rm.import_component(std::move(stepit_hardware), FakeHardwareInfo{});

  // Connect the hardware.
  rclcpp_lifecycle::State state{ lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
                                 hardware_interface::lifecycle_state_names::ACTIVE };
  rm.set_component_state("StepitHardware", state);

  // Invoke a read command.
  const rclcpp::Time time;
  const rclcpp::Duration period = rclcpp::Duration::from_seconds(0);
  rm.read(time, period);

  // Read state interface values.
  hardware_interface::LoanedStateInterface joint1_position_state = rm.claim_state_interface("joint1/position");
  hardware_interface::LoanedStateInterface joint2_position_state = rm.claim_state_interface("joint2/position");
  hardware_interface::LoanedStateInterface joint3_position_state = rm.claim_state_interface("joint3/position");
  hardware_interface::LoanedStateInterface joint4_position_state = rm.claim_state_interface("joint4/position");
  hardware_interface::LoanedStateInterface joint5_position_state = rm.claim_state_interface("joint5/position");

  hardware_interface::LoanedStateInterface joint1_velocity_state = rm.claim_state_interface("joint1/velocity");
  hardware_interface::LoanedStateInterface joint2_velocity_state = rm.claim_state_interface("joint2/velocity");
  hardware_interface::LoanedStateInterface joint3_velocity_state = rm.claim_state_interface("joint3/velocity");
  hardware_interface::LoanedStateInterface joint4_velocity_state = rm.claim_state_interface("joint4/velocity");
  hardware_interface::LoanedStateInterface joint5_velocity_state = rm.claim_state_interface("joint5/velocity");

  ASSERT_EQ(32100, joint1_position_state.get_value());
  ASSERT_EQ(-6500, joint2_position_state.get_value());
  ASSERT_EQ(-6500, joint3_position_state.get_value());
  ASSERT_EQ(-6500, joint4_position_state.get_value());
  ASSERT_EQ(-6500, joint5_position_state.get_value());

  ASSERT_EQ(0.5, joint1_velocity_state.get_value());
  ASSERT_EQ(0.75, joint2_velocity_state.get_value());
  ASSERT_EQ(0.75, joint3_velocity_state.get_value());
  ASSERT_EQ(0.75, joint4_velocity_state.get_value());
  ASSERT_EQ(0.75, joint5_velocity_state.get_value());
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
        VelocityGoal{ 0, 0.5 },  // Motor 0 goal
        VelocityGoal{ 1, 0.75 }, // Motor 1 goal
        VelocityGoal{ 2, 0.75 }, // Motor 2 goal
        VelocityGoal{ 3, 0.75 }, // Motor 3 goal
        VelocityGoal{ 4, 0.75 }  // Motor 4 goal
    }
  };
  // clang-format on

  VelocityCommand actual_request{ {} };

  auto mock_driver = std::make_unique<MockDriver>();
  ON_CALL(*mock_driver, connect()).WillByDefault(Return(true));
  ON_CALL(*mock_driver, send(Matcher<const ConfigCommand&>(_)))
      .WillByDefault(Return(AcknowledgeResponse{ Response::Status::Success }));
  EXPECT_CALL(*mock_driver, send(_, Matcher<const VelocityCommand&>(_)))
      .WillOnce(DoAll(SaveArg<1>(&actual_request), Return(AcknowledgeResponse{ Response::Status::Success })));
  auto mock_driver_factory = std::make_unique<MockDriverFactory>(std::move(mock_driver));

  auto stepit_hardware = std::make_unique<stepit_driver::StepitHardware>(std::move(mock_driver_factory));

  // Load the component.
  hardware_interface::ResourceManager rm;
  rm.import_component(std::move(stepit_hardware), FakeHardwareInfo{});

  // Connect the hardware.
  rclcpp_lifecycle::State state{ lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
                                 hardware_interface::lifecycle_state_names::ACTIVE };
  rm.set_component_state("StepitHardware", state);

  // Write velocity values.
  hardware_interface::LoanedCommandInterface joint1_velocity_command = rm.claim_command_interface("joint1/velocity");
  hardware_interface::LoanedCommandInterface joint2_velocity_command = rm.claim_command_interface("joint2/velocity");
  hardware_interface::LoanedCommandInterface joint3_velocity_command = rm.claim_command_interface("joint3/velocity");
  hardware_interface::LoanedCommandInterface joint4_velocity_command = rm.claim_command_interface("joint4/velocity");
  hardware_interface::LoanedCommandInterface joint5_velocity_command = rm.claim_command_interface("joint5/velocity");

  joint1_velocity_command.set_value(0.5);
  joint2_velocity_command.set_value(0.75);
  joint3_velocity_command.set_value(0.75);
  joint4_velocity_command.set_value(0.75);
  joint5_velocity_command.set_value(0.75);

  // Invoke a write command.
  const rclcpp::Time time;
  const rclcpp::Duration period = rclcpp::Duration::from_seconds(0);
  rm.write(time, period);

  ASSERT_EQ(actual_request.goals().size(), actual_request.goals().size());

  ASSERT_EQ(actual_request.goals()[0].motor_id(), actual_request.goals()[0].motor_id());
  ASSERT_EQ(actual_request.goals()[0].velocity(), actual_request.goals()[0].velocity());

  ASSERT_EQ(actual_request.goals()[1].motor_id(), actual_request.goals()[1].motor_id());
  ASSERT_EQ(actual_request.goals()[1].velocity(), actual_request.goals()[1].velocity());

  ASSERT_EQ(actual_request.goals()[2].motor_id(), actual_request.goals()[2].motor_id());
  ASSERT_EQ(actual_request.goals()[2].velocity(), actual_request.goals()[2].velocity());

  ASSERT_EQ(actual_request.goals()[3].motor_id(), actual_request.goals()[3].motor_id());
  ASSERT_EQ(actual_request.goals()[3].velocity(), actual_request.goals()[3].velocity());

  ASSERT_EQ(actual_request.goals()[4].motor_id(), actual_request.goals()[4].motor_id());
  ASSERT_EQ(actual_request.goals()[4].velocity(), actual_request.goals()[4].velocity());
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
        PositionGoal{ 0, 0.5 },  // Motor 0 goal
        PositionGoal{ 1, 0.75 }, // Motor 1 goal
        PositionGoal{ 2, 0.75 }, // Motor 2 goal
        PositionGoal{ 3, 0.75 }, // Motor 3 goal
        PositionGoal{ 4, 0.75 }  // Motor 4 goal
    }
  };
  // clang-format on

  PositionCommand actual_request{ {} };

  auto mock_driver = std::make_unique<MockDriver>();
  ON_CALL(*mock_driver, connect()).WillByDefault(Return(true));
  ON_CALL(*mock_driver, send(Matcher<const ConfigCommand&>(_)))
      .WillByDefault(Return(AcknowledgeResponse{ Response::Status::Success }));
  EXPECT_CALL(*mock_driver, send(_, Matcher<const PositionCommand&>(_)))
      .WillOnce(DoAll(SaveArg<1>(&actual_request), Return(AcknowledgeResponse{ Response::Status::Success })));
  auto mock_driver_factory = std::make_unique<MockDriverFactory>(std::move(mock_driver));

  auto stepit_hardware = std::make_unique<stepit_driver::StepitHardware>(std::move(mock_driver_factory));

  // Load the component.
  hardware_interface::ResourceManager rm;
  rm.import_component(std::move(stepit_hardware), FakeHardwareInfo{});

  // Connect the hardware.
  rclcpp_lifecycle::State state{ lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
                                 hardware_interface::lifecycle_state_names::ACTIVE };
  rm.set_component_state("StepitHardware", state);

  // Write position values.
  hardware_interface::LoanedCommandInterface joint1_position_command = rm.claim_command_interface("joint1/position");
  hardware_interface::LoanedCommandInterface joint2_position_command = rm.claim_command_interface("joint2/position");
  hardware_interface::LoanedCommandInterface joint3_position_command = rm.claim_command_interface("joint3/position");
  hardware_interface::LoanedCommandInterface joint4_position_command = rm.claim_command_interface("joint4/position");
  hardware_interface::LoanedCommandInterface joint5_position_command = rm.claim_command_interface("joint5/position");
  joint1_position_command.set_value(0.5);
  joint2_position_command.set_value(0.75);
  joint3_position_command.set_value(0.75);
  joint4_position_command.set_value(0.75);
  joint5_position_command.set_value(0.75);

  // Invoke a write command.
  const rclcpp::Time time;
  const rclcpp::Duration period = rclcpp::Duration::from_seconds(0);
  rm.write(time, period);

  ASSERT_EQ(actual_request.goals().size(), actual_request.goals().size());

  ASSERT_EQ(actual_request.goals()[0].motor_id(), actual_request.goals()[0].motor_id());
  ASSERT_EQ(actual_request.goals()[0].position(), actual_request.goals()[0].position());

  ASSERT_EQ(actual_request.goals()[1].motor_id(), actual_request.goals()[1].motor_id());
  ASSERT_EQ(actual_request.goals()[1].position(), actual_request.goals()[1].position());

  ASSERT_EQ(actual_request.goals()[2].motor_id(), actual_request.goals()[2].motor_id());
  ASSERT_EQ(actual_request.goals()[2].position(), actual_request.goals()[2].position());

  ASSERT_EQ(actual_request.goals()[3].motor_id(), actual_request.goals()[3].motor_id());
  ASSERT_EQ(actual_request.goals()[3].position(), actual_request.goals()[3].position());

  ASSERT_EQ(actual_request.goals()[4].motor_id(), actual_request.goals()[4].motor_id());
  ASSERT_EQ(actual_request.goals()[4].position(), actual_request.goals()[4].position());
}

/**
 * In this test we set positions goals on the hardware interface.
 * We execute a write operation and expect to see one data frame delivered
 * to the actual hardware, setting positions.
 */
TEST(TestStepitHardware, configuration)
{
  // clang-format off
   const ConfigCommand expected_request{
     {
         ConfigParam{ 0, 0.1, 0.2 },  // Motor 0 goal
         ConfigParam{ 1, 0.3, 0.4},   // Motor 1 goal
         ConfigParam{ 2, 0.3, 0.4},   // Motor 1 goal
         ConfigParam{ 3, 0.3, 0.4},   // Motor 1 goal
         ConfigParam{ 4, 0.3, 0.4}    // Motor 1 goal
     }
   };
  // clang-format on

  const AcknowledgeResponse mocked_response{ Response::Status::Success };
  ConfigCommand actual_request{ {} };

  auto mock_driver = std::make_unique<MockDriver>();
  ON_CALL(*mock_driver, connect()).WillByDefault(Return(true));
  EXPECT_CALL(*mock_driver, send(Matcher<const ConfigCommand&>(_)))
      .WillOnce(DoAll(SaveArg<0>(&actual_request), Return(mocked_response)));
  auto mock_driver_factory = std::make_unique<MockDriverFactory>(std::move(mock_driver));

  auto stepit_hardware = std::make_unique<stepit_driver::StepitHardware>(std::move(mock_driver_factory));

  // Load the component.
  hardware_interface::ResourceManager rm;
  rm.import_component(std::move(stepit_hardware), FakeHardwareInfo{});

  // Connect the hardware.
  rclcpp_lifecycle::State state{ lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
                                 hardware_interface::lifecycle_state_names::ACTIVE };
  rm.set_component_state("StepitHardware", state);

  ASSERT_EQ(actual_request.params().size(), actual_request.params().size());
  ASSERT_EQ(actual_request.params()[0].motor_id(), actual_request.params()[0].motor_id());
  ASSERT_EQ(actual_request.params()[0].acceleration(), actual_request.params()[0].acceleration());
  ASSERT_EQ(actual_request.params()[0].max_velocity(), actual_request.params()[0].max_velocity());

  ASSERT_EQ(actual_request.params()[1].motor_id(), actual_request.params()[1].motor_id());
  ASSERT_EQ(actual_request.params()[1].acceleration(), actual_request.params()[1].acceleration());
  ASSERT_EQ(actual_request.params()[1].max_velocity(), actual_request.params()[1].max_velocity());

  ASSERT_EQ(actual_request.params()[2].motor_id(), actual_request.params()[2].motor_id());
  ASSERT_EQ(actual_request.params()[2].acceleration(), actual_request.params()[2].acceleration());
  ASSERT_EQ(actual_request.params()[2].max_velocity(), actual_request.params()[2].max_velocity());

  ASSERT_EQ(actual_request.params()[3].motor_id(), actual_request.params()[3].motor_id());
  ASSERT_EQ(actual_request.params()[3].acceleration(), actual_request.params()[3].acceleration());
  ASSERT_EQ(actual_request.params()[3].max_velocity(), actual_request.params()[3].max_velocity());

  ASSERT_EQ(actual_request.params()[4].motor_id(), actual_request.params()[4].motor_id());
  ASSERT_EQ(actual_request.params()[4].acceleration(), actual_request.params()[4].acceleration());
  ASSERT_EQ(actual_request.params()[4].max_velocity(), actual_request.params()[4].max_velocity());
}
}  // namespace stepit_driver::test
