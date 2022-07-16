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

#include <stepit_hardware/stepit_hardware.hpp>

#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>

namespace stepit_hardware
{
constexpr auto kStepitHardware = "StepitHardware";

CallbackReturn StepitHardware::on_init(const hardware_interface::HardwareInfo& info)
{
  auto usb_port = info_.hardware_parameters.at("usb_port");
  auto baud_rate = std::stoi(info_.hardware_parameters.at("baud_rate"));

  RCLCPP_INFO(rclcpp::get_logger(kStepitHardware), "usb_port: %s", usb_port.c_str());
  RCLCPP_INFO(rclcpp::get_logger(kStepitHardware), "baud_rate: %d", baud_rate);
  return {};
}

std::vector<hardware_interface::StateInterface> StepitHardware::export_state_interfaces()
{
  return {};
}

std::vector<hardware_interface::CommandInterface> StepitHardware::export_command_interfaces()
{
  return {};
}

CallbackReturn StepitHardware::on_activate(const rclcpp_lifecycle::State& previous_state)
{
  return {};
}

CallbackReturn StepitHardware::on_deactivate(const rclcpp_lifecycle::State& previous_state)
{
  return {};
}

return_type StepitHardware::read()
{
  return {};
}

return_type StepitHardware::write()
{
  return {};
}

}  // namespace stepit_hardware
