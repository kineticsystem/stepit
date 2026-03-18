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

#pragma once

#include <hardware_interface/hardware_info.hpp>

namespace stepit_driver::test
{
/**
 * Simplified Hardware Interface configuration for testing.
 */
class FakeHardwareInfo : public hardware_interface::HardwareInfo
{
public:
  FakeHardwareInfo()
  {
    name = "StepitHardware";
    type = "system";
    hardware_plugin_name = "stepit_driver/StepitHardware";
    hardware_parameters = {
      { "usb_port", "/dev/ttyUSB0" },
      { "baud_rate", "9600" },
      { "timeout", "0.5" },
      { "use_dummy", "true" },
    };

    auto make_joint = [](const std::string& joint_name, const std::string& id) {
      hardware_interface::ComponentInfo joint;
      joint.name = joint_name;
      joint.type = "joint";
      hardware_interface::InterfaceInfo pos{ .name = "position", .size = 0, .enable_limits = false, .parameters = {} };
      hardware_interface::InterfaceInfo vel{ .name = "velocity", .size = 0, .enable_limits = false, .parameters = {} };
      joint.command_interfaces = { pos, vel };
      joint.state_interfaces = { pos, vel };
      joint.parameters = { { "id", id }, { "acceleration", "3.14159" }, { "max_velocity", "6.28319" } };
      return joint;
    };

    joints = {
      make_joint("joint1", "0"), make_joint("joint2", "1"), make_joint("joint3", "2"),
      make_joint("joint4", "3"), make_joint("joint5", "4"),
    };
  }
};
}  // namespace stepit_driver::test
