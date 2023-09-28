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
#include <gmock/gmock.h>

#include <cmath>
#include <string>
#include <unordered_map>
#include <vector>

#include <freezer_driver/freezer_hardware.hpp>

#include <cobs_serial/data_utils.hpp>

#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <hardware_interface/resource_manager.hpp>
#include <hardware_interface/types/lifecycle_state_names.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <ros2_control_test_assets/components_urdfs.hpp>
#include <ros2_control_test_assets/descriptions.hpp>

namespace freezer_driver::test
{
/**
 * This test generates a minimal xacro robot configuration and loads the
 * hardware interface plugin.
 */
TEST(TestFreezerHardware, load_urdf)
{
  std::string urdf_control_ =
      R"(
        <ros2_control name="FreezerHardware" type="system">
          <hardware>
            <plugin>freezer_driver/FreezerHardware</plugin>
            <param name="usb_port">/dev/ttyUSB0</param>
            <param name="baud_rate">9600</param>
            <param name="timeout">1</param>
            <param name="connection_timeout">5</param>
          </hardware>
        </ros2_control>
      )";

  auto urdf = ros2_control_test_assets::urdf_head + urdf_control_ + ros2_control_test_assets::urdf_tail;
  hardware_interface::ResourceManager rm(urdf);

  EXPECT_EQ(1u, rm.system_components_size());
  ASSERT_EQ(0u, rm.state_interface_keys().size());
  ASSERT_EQ(0u, rm.command_interface_keys().size());
}
}  // namespace freezer_driver::test
