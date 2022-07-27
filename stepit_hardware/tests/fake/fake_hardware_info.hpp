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

#pragma once

#include <hardware_interface/hardware_info.hpp>

namespace stepit_hardware::test
{
/**
 * Simplified Hardware Interface configuration for testing.
 */
class FakeHardwareInfo : public hardware_interface::HardwareInfo
{
public:
  // clang-format off
  FakeHardwareInfo() : HardwareInfo{
    HardwareInfo{
      "StepitHardware",
      "system",
      "stepit_hardware/StepitHardware",
      {
          {"usb_port", "/dev/whatever"},
          {"baud_rate", "9600"}
      },
      {
          hardware_interface::ComponentInfo{"joint1",
              "joint",
              {
                  hardware_interface::InterfaceInfo{"position", "", "", "", "double", 1},
                  hardware_interface::InterfaceInfo{"velocity", "", "", "", "double", 1}
              },
              {
                  hardware_interface::InterfaceInfo{"position", "", "", "", "double", 1},
                  hardware_interface::InterfaceInfo{"velocity", "", "", "", "double", 1}
              },
              {{{"id","0"}}}
          },
          hardware_interface::ComponentInfo{"joint2",
              "joint",
              {
                  hardware_interface::InterfaceInfo{"position", "", "", "", "double", 1},
                  hardware_interface::InterfaceInfo{"velocity", "", "", "", "double", 1}
              },
              {
                  hardware_interface::InterfaceInfo{"position", "", "", "", "double", 1},
                  hardware_interface::InterfaceInfo{"velocity", "", "", "", "double", 1}
              },
              {{{ "id", "1" }}}
          }
      },
      {},
      {},
      {},
      ""
    }
  } {};
  // clang-format on
};
}  // namespace stepit_hardware::test
