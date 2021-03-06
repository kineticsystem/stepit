/*
 * Copyright (C) 2022 Remigi Giovanni
 * g.remigi@kineticsystem.org
 * www.kineticsystem.org
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU Lesser General Public License as published by the Free
 * Software Foundation; either version 2 of the License, or (at your option) any
 * later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License for more
 * details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program; if not, write to the Free Software Foundation, Inc.,
 * 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <gtest/gtest.h>

#include <stepit_hardware/stepit_hardware.hpp>
#include <stepit_hardware/data_utils.hpp>
#include <stepit_hardware/msgs/motor_status_query.hpp>
#include <mock/mock_data_interface.hpp>
#include <fake/fake_hardware_info.hpp>

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

namespace stepit_hardware::test
{
using ::testing::_;
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
            <plugin>stepit_hardware/StepitHardware</plugin>
            <param name="usb_port">/dev/whatever</param>
            <param name="baud_rate">9600</param>
          </hardware>
          <joint name="joint1">
            <param name="id">0</param>
            <command_interface name="position"/>
            <command_interface name="velocity"/>
            <state_interface name="position"/>
            <state_interface name="velocity"/>
          </joint>
          <joint name="joint2">
            <param name="id">1</param>
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
TEST(TestStepitHardware, read)
{
  std::vector<uint8_t> motor_status_response{
    0x00,  // request ID
    0x11,  // status success
    0x00,  // motor ID
    0x00,  // position = 32100
    0x00,  // position
    0x7D,  // position
    0x64,  // position
    0x3F,  // speed = 0.5
    0x00,  // speed
    0x00,  // speed
    0x00,  // speed
    0x00,  // distance to go = 150
    0x00,  // distance to go
    0x00,  // distance to go
    0x96,  // distance to go
    0x01,  // motor ID
    0xFF,  // position = -6500
    0xFF,  // position
    0xE6,  // position
    0x9C,  // position
    0x3F,  // speed = 0.75
    0x40,  // speed
    0x00,  // speed
    0x00,  // speed
    0x00,  // distance to go = 150000
    0x02,  // distance to go
    0x49,  // distance to go
    0xF0,  // distance to go
  };

  auto mock_data_interface = std::make_unique<MockDataInterface>();
  EXPECT_CALL(*mock_data_interface, read()).WillOnce(Return(motor_status_response));

  auto stepit_hardware = std::make_unique<stepit_hardware::StepitHardware>();
  stepit_hardware->set_data_interface(std::move(mock_data_interface));

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
  hardware_interface::LoanedStateInterface joint1_velocity_state = rm.claim_state_interface("joint1/velocity");
  hardware_interface::LoanedStateInterface joint2_velocity_state = rm.claim_state_interface("joint2/velocity");

  ASSERT_EQ(32100, joint1_position_state.get_value());
  ASSERT_EQ(-6500, joint2_position_state.get_value());
  ASSERT_EQ(0.5, joint1_velocity_state.get_value());
  ASSERT_EQ(0.75, joint2_velocity_state.get_value());
}

/**
 * In this test we set velocities and positions goals on the hardware interface.
 * We execute a write operation and expect to see two packets of data delivered
 * to the actual hardware, one setting velocities and one setting positions.
 */
TEST(TestStepitHardware, write)
{
  std::vector<uint8_t> velocity_request;
  std::vector<uint8_t> position_request;
  auto mock_data_interface = std::make_unique<MockDataInterface>();
  EXPECT_CALL(*mock_data_interface, write(_))
      .WillOnce(SaveArg<0>(&velocity_request))
      .WillOnce(SaveArg<0>(&position_request));

  std::vector<uint8_t> velocity_response{ 0x00, 0x11 };
  std::vector<uint8_t> position_response{ 0x01, 0x11 };
  EXPECT_CALL(*mock_data_interface, read()).WillOnce(Return(velocity_response)).WillOnce(Return(position_response));

  auto stepit_hardware = std::make_unique<stepit_hardware::StepitHardware>();
  stepit_hardware->set_data_interface(std::move(mock_data_interface));

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
  joint1_velocity_command.set_value(0.5);
  joint2_velocity_command.set_value(0.75);

  // Write position values.
  hardware_interface::LoanedCommandInterface joint1_position_command = rm.claim_command_interface("joint1/position");
  hardware_interface::LoanedCommandInterface joint2_position_command = rm.claim_command_interface("joint2/position");
  joint1_position_command.set_value(0.5);
  joint2_position_command.set_value(0.75);

  // Invoke a write command.
  const rclcpp::Time time;
  const rclcpp::Duration period = rclcpp::Duration::from_seconds(0);
  rm.write(time, period);

  std::vector<uint8_t> expected_velocity_request{
    0x00,  // request ID
    0x77,  // command ID
    0x00,  // motor ID
    0x3F,  // velocity = 0.5
    0x00,  // velocity
    0x00,  // velocity
    0x00,  // velocity
    0x01,  // motor ID
    0x3F,  // velocity = 0.75
    0x40,  // velocity
    0x00,  // velocity
    0x00   // velocity
  };
  ASSERT_THAT(stepit_hardware::data_utils::to_hex(velocity_request),
              stepit_hardware::data_utils::to_hex(expected_velocity_request));

  std::vector<uint8_t> expected_position_request{
    0x01,  // request ID
    0x71,  // command ID
    0x00,  // motor ID
    0x3F,  // position = 0.5
    0x00,  // position
    0x00,  // position
    0x00,  // position
    0x01,  // motor ID
    0x3F,  // position = 0.75
    0x40,  // position
    0x00,  // position
    0x00   // position
  };
  ASSERT_THAT(stepit_hardware::data_utils::to_hex(position_request),
              stepit_hardware::data_utils::to_hex(expected_position_request));
}
}  // namespace stepit_hardware::test
