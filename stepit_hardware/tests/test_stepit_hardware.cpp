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

#include <cmath>
#include <string>
#include <unordered_map>
#include <vector>

#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <hardware_interface/resource_manager.hpp>
#include <hardware_interface/types/lifecycle_state_names.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <ros2_control_test_assets/components_urdfs.hpp>
#include <ros2_control_test_assets/descriptions.hpp>

namespace
{
const auto TIME = rclcpp::Time(0);
const auto PERIOD = rclcpp::Duration::from_seconds(0.01);
}  // namespace

using namespace hardware_interface;

/**
 * https://control.ros.org/master/doc/ros2_control/hardware_interface/doc/writing_new_hardware_interface.html
 */
class TestStepitHardware : public ::testing::Test
{
protected:
  void SetUp() override
  {
    stepit_hardware_ =
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
  }

  std::string stepit_hardware_;
};

void set_components_state(hardware_interface::ResourceManager& rm, const std::vector<std::string>& components,
                          const uint8_t state_id, const std::string& state_name)
{
  for (const auto& component : components)
  {
    rclcpp_lifecycle::State state(state_id, state_name);
    rm.set_component_state(component, state);
  }
}

auto activate_components = [](hardware_interface::ResourceManager& rm, const std::vector<std::string>& components) {
  set_components_state(rm, components, lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
                       hardware_interface::lifecycle_state_names::ACTIVE);
};

/**
 * This test generates a minimal xacro robot configuration and load plugin.
 */
TEST_F(TestStepitHardware, load_stepit_hardware)
{
  auto urdf = ros2_control_test_assets::urdf_head + stepit_hardware_ + ros2_control_test_assets::urdf_tail;
  hardware_interface::ResourceManager rm(urdf);
}

/**
 * Activate the component, check all interfaces and their current values.
 */
TEST_F(TestStepitHardware, activate_stepit_hardware)
{
  auto urdf = ros2_control_test_assets::urdf_head + stepit_hardware_ + ros2_control_test_assets::urdf_tail;
  hardware_interface::ResourceManager rm(urdf);

  // Activate the given component to retrieve all available interfaces.
  activate_components(rm, { "StepitHardware" });

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

  // Check initial values
  hardware_interface::LoanedStateInterface joint1_position_state = rm.claim_state_interface("joint1/position");
  hardware_interface::LoanedStateInterface joint2_position_state = rm.claim_state_interface("joint2/position");
  hardware_interface::LoanedCommandInterface joint1_position_command = rm.claim_command_interface("joint1/position");
  hardware_interface::LoanedCommandInterface joint2_position_command = rm.claim_command_interface("joint2/position");

  ASSERT_EQ(1.57, joint1_position_state.get_value());
  ASSERT_EQ(0.7854, joint2_position_state.get_value());
  ASSERT_TRUE(std::isnan(joint1_position_command.get_value()));
  ASSERT_TRUE(std::isnan(joint2_position_command.get_value()));

  hardware_interface::LoanedStateInterface joint1_velocity_state = rm.claim_state_interface("joint1/velocity");
  hardware_interface::LoanedStateInterface joint2_velocity_state = rm.claim_state_interface("joint2/velocity");
  hardware_interface::LoanedCommandInterface joint1_velocity_command = rm.claim_command_interface("joint1/velocity");
  hardware_interface::LoanedCommandInterface joint2_velocity_command = rm.claim_command_interface("joint2/velocity");
}

/**
 * Test the read method by providing a mocked serial interface.
 */
TEST_F(TestStepitHardware, test_read)
{
  // clang-format off
  HardwareInfo hardware_info
    {
      "StepitHardware",
      "system",
      "stepit_hardware/StepitHardware",
      {
        {"usb_port", "/dev/whatever"},
        {"baud_rate", "9600"}
      },
      {
        ComponentInfo{"joint1",
          "joint",
          {
            InterfaceInfo{"position", "", "", "", "double", 1},
            InterfaceInfo{"velocity", "", "", "", "double", 1}
          },
          {
            InterfaceInfo{"position", "", "", "", "double", 1},
            InterfaceInfo{"velocity", "", "", "", "double", 1}
          },
          {{{"id","0"}}}
        },
        ComponentInfo{"joint2",
         "joint",
         {
           InterfaceInfo{"position", "", "", "", "double", 1},
           InterfaceInfo{"velocity", "", "", "", "double", 1}
         },
         {
           InterfaceInfo{"position", "", "", "", "double", 1},
           InterfaceInfo{"velocity", "", "", "", "double", 1}
         },
         {{{ "id", "1" }}}
        }
      },
      {},
      {},
      {},
      ""
    };
  // clang-format on

  auto stepit_hardware = std::make_unique<stepit_hardware::StepitHardware>();

  hardware_interface::ResourceManager rm;
  rm.import_component(std::move(stepit_hardware), hardware_info);
}
