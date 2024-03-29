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
  // clang-format off
  FakeHardwareInfo() : HardwareInfo{
    "StepitHardware",
    "system",
    "stepit_driver/StepitHardware",
    {
      {"usb_port", "/dev/ttyUSB0"},
      {"baud_rate", "9600"},
      {"timeout", "0.5"},
      {"use_dummy", "true"}
    },
    {
      {
        "joint1",
        "joint",
        {
          {"position", "", "", "", "double", 1},
          {"velocity", "", "", "", "double", 1}
        },
        {
          {"position", "", "", "", "double", 1},
          {"velocity", "", "", "", "double", 1}
        },
        {{{"id", "0"}, {"acceleration", "3.14159"}, {"max_velocity", "6.28319"}}}
      },
      {
        "joint2",
        "joint",
        {
          {"position", "", "", "", "double", 1},
          {"velocity", "", "", "", "double", 1}
        },
        {
          {"position", "", "", "", "double", 1},
          {"velocity", "", "", "", "double", 1}
        },
        {{{ "id", "1" }, {"acceleration", "3.14159"}, {"max_velocity", "6.28319"}}}
      },
      {
        "joint3",
        "joint",
        {
          {"position", "", "", "", "double", 1},
          {"velocity", "", "", "", "double", 1}
        },
        {
          {"position", "", "", "", "double", 1},
          {"velocity", "", "", "", "double", 1}
        },
        {{{ "id", "2" }, {"acceleration", "3.14159"}, {"max_velocity", "6.28319"}}}
      },
      {
        "joint4",
        "joint",
        {
          {"position", "", "", "", "double", 1},
          {"velocity", "", "", "", "double", 1}
        },
        {
          {"position", "", "", "", "double", 1},
          {"velocity", "", "", "", "double", 1}
        },
        {{{ "id", "3" }, {"acceleration", "3.14159"}, {"max_velocity", "6.28319"}}}
      },
      {
        "joint5",
        "joint",
        {
          {"position", "", "", "", "double", 1},
          {"velocity", "", "", "", "double", 1}
        },
        {
          {"position", "", "", "", "double", 1},
          {"velocity", "", "", "", "double", 1}
        },
        {{{ "id", "4" }, {"acceleration", "3.14159"}, {"max_velocity", "6.28319"}}}
      }
    },
    {},
    {},
    {},
    ""
  } {};
  // clang-format on
};
}  // namespace stepit_driver::test
