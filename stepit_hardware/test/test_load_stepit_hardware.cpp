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

#include <gmock/gmock.h>

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

/** https://control.ros.org/master/doc/ros2_control/hardware_interface/doc/writing_new_hardware_interface.html */
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
            <param name="port">/dev/whatever</param>
          </hardware>
          <joint name="joint1">
            <command_interface name="position"/>
            <command_interface name="velocity"/>
            <state_interface name="position"/>
            <state_interface name="velocity"/>
            <param name="initial_position">3.45</param>
          </joint>
          <joint name="joint2">
            <command_interface name="position"/>
            <command_interface name="velocity"/>
            <state_interface name="position"/>
            <state_interface name="velocity"/>
            <param name="initial_position">3.45</param>
          </joint>
        </ros2_control>
      )";
  }

  std::string stepit_hardware_;
};

/**
 * This test generates a minimal xacro robot configuration with instructions
 * to load the hardware interface plugin and processes it.
 */
TEST_F(TestStepitHardware, load_stepit_hardware)
{
  auto urdf = ros2_control_test_assets::urdf_head + stepit_hardware_ + ros2_control_test_assets::urdf_tail;

  hardware_interface::ResourceManager rm(urdf);
  ASSERT_NO_THROW(hardware_interface::ResourceManager rm(urdf));
}
