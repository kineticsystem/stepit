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
#include <gmock/gmock.h>

#include <stepit_hardware/msgs/msgs.hpp>
#include <stepit_hardware/stepit_hardware.hpp>

#include <stepit_hardware_info.hpp>
#include <test_utils.hpp>

#include <data_interface/data_utils.hpp>
#include <data_interface/crc_utils.hpp>

#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <hardware_interface/resource_manager.hpp>
#include <hardware_interface/types/lifecycle_state_names.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <ros2_control_test_assets/components_urdfs.hpp>
#include <ros2_control_test_assets/descriptions.hpp>

#include <thread>
#include <chrono>

namespace stepit_hardware::test
{

class TestStepitHardware : public ::testing::Test
{
public:
  void SetUp()
  {
    if (skip_test())
    {
      GTEST_SKIP() << "Skipping all tests for this fixture";
    }
  }
};

/**
 * This test requires connection to a real hardware.
 */
TEST_F(TestStepitHardware, test_connection)
{
  auto stepit_hardware = std::make_unique<stepit_hardware::StepitHardware>();

  StepitHardwareInfo info;
  info.hardware_parameters["use_dummy"] = false;

  // Load the component.
  hardware_interface::ResourceManager rm;
  rm.import_component(std::move(stepit_hardware), info);

  // Connect the hardware.
  rclcpp_lifecycle::State state{ lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
                                 hardware_interface::lifecycle_state_names::ACTIVE };
  rm.set_component_state("StepitHardware", state);

  // Invoke a read command.
  rm.read(rclcpp::Time{}, rclcpp::Duration::from_seconds(0));

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

  ASSERT_EQ(0, joint1_position_state.get_value());
  ASSERT_EQ(0, joint2_position_state.get_value());
  ASSERT_EQ(0, joint3_position_state.get_value());
  ASSERT_EQ(0, joint4_position_state.get_value());
  ASSERT_EQ(0, joint5_position_state.get_value());

  ASSERT_EQ(0, joint1_velocity_state.get_value());
  ASSERT_EQ(0, joint2_velocity_state.get_value());
  ASSERT_EQ(0, joint3_velocity_state.get_value());
  ASSERT_EQ(0, joint4_velocity_state.get_value());
  ASSERT_EQ(0, joint5_velocity_state.get_value());

  // Write velocity values.
  // Arduino uses its max internal speed if a too high velocity value is given.
  hardware_interface::LoanedCommandInterface joint1_velocity_command = rm.claim_command_interface("joint1/velocity");
  hardware_interface::LoanedCommandInterface joint2_velocity_command = rm.claim_command_interface("joint2/velocity");
  hardware_interface::LoanedCommandInterface joint3_velocity_command = rm.claim_command_interface("joint3/velocity");
  hardware_interface::LoanedCommandInterface joint4_velocity_command = rm.claim_command_interface("joint4/velocity");
  hardware_interface::LoanedCommandInterface joint5_velocity_command = rm.claim_command_interface("joint5/velocity");
  joint1_velocity_command.set_value(4);
  joint2_velocity_command.set_value(1);
  joint3_velocity_command.set_value(1);
  joint4_velocity_command.set_value(1);
  joint5_velocity_command.set_value(1);

  for (int i = 0; i < 10; ++i)
  {
    // Invoke a write command.
    rm.write(rclcpp::Time{}, rclcpp::Duration::from_seconds(0));
    rm.read(rclcpp::Time{}, rclcpp::Duration::from_seconds(0));
    RCLCPP_INFO(rclcpp::get_logger("TestStepitHardware"),
                "%d, p1:%f, v1:%f, p2:%f, v2:%f, p2:%f, v2:%f, p2:%f, v2:%f, p2:%f, v2:%f", i,
                joint1_position_state.get_value(), joint1_velocity_state.get_value(), joint2_position_state.get_value(),
                joint2_velocity_state.get_value(), joint3_position_state.get_value(), joint3_velocity_state.get_value(),
                joint4_position_state.get_value(), joint4_velocity_state.get_value(), joint5_position_state.get_value(),
                joint5_velocity_state.get_value()

    );
  }
}

}  // namespace stepit_hardware::test
