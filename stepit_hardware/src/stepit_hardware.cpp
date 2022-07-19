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

#include <pluginlib/class_list_macros.hpp>

#include <limits>
#include <vector>
#include <string>

namespace stepit_hardware
{
constexpr auto kStepitHardware = "StepitHardware";

hardware_interface::CallbackReturn StepitHardware::on_init(const hardware_interface::HardwareInfo& info)
{
  RCLCPP_DEBUG(rclcpp::get_logger(kStepitHardware), "on_init");
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  // Initialize as many joint states as specified in the configuration.

  joint_ids_.resize(info_.joints.size(), 0);
  joints_.resize(info_.joints.size(), Joint{});

  for (uint i = 0; i < info_.joints.size(); i++)
  {
    joint_ids_[i] = std::stoi(info_.joints[i].parameters.at("id"));
    joints_[i].state.position = std::numeric_limits<double>::quiet_NaN();
    joints_[i].state.velocity = std::numeric_limits<double>::quiet_NaN();
    joints_[i].command.position = std::numeric_limits<double>::quiet_NaN();
    joints_[i].command.velocity = std::numeric_limits<double>::quiet_NaN();
    RCLCPP_INFO(rclcpp::get_logger(kStepitHardware), "joint_id %d: %d", i, joint_ids_[i]);
  }

  // Read serial port connection parameters.

  auto usb_port = info_.hardware_parameters.at("usb_port");
  auto baud_rate = std::stoi(info_.hardware_parameters.at("baud_rate"));

  RCLCPP_INFO(rclcpp::get_logger(kStepitHardware), "usb_port: %s", usb_port.c_str());
  RCLCPP_INFO(rclcpp::get_logger(kStepitHardware), "baud_rate: %d", baud_rate);

  // TODO: initialize the serial connection and check the steppers.

  //  if (!data_interface_.init(usb_port.c_str(), baud_rate, &log))
  //  {
  //    RCLCPP_FATAL(rclcpp::get_logger(kStepitHardware), "%s", log);
  //    return CallbackReturn::ERROR;
  //  }

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> StepitHardware::export_state_interfaces()
{
  RCLCPP_DEBUG(rclcpp::get_logger(kStepitHardware), "export_state_interfaces");
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joints_[i].state.position));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joints_[i].state.velocity));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> StepitHardware::export_command_interfaces()
{
  RCLCPP_DEBUG(rclcpp::get_logger(kStepitHardware), "export_command_interfaces");
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joints_[i].command.position));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joints_[i].command.velocity));
  }
  return command_interfaces;
}

hardware_interface::CallbackReturn
StepitHardware::on_activate([[maybe_unused]] const rclcpp_lifecycle::State& previous_state)
{
  RCLCPP_DEBUG(rclcpp::get_logger(kStepitHardware), "start");

  // Make goals match current joint states.
  //  read();
  //  for (auto& joint : joints_)
  //  {
  //    joint.command.position = joint.state.position;
  //    joint.command.velocity = 0.0;
  //  }
  //  write();

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
StepitHardware::on_deactivate([[maybe_unused]] const rclcpp_lifecycle::State& previous_state)
{
  RCLCPP_DEBUG(rclcpp::get_logger(kStepitHardware), "stop");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type StepitHardware::read(const rclcpp::Time& time, const rclcpp::Duration& period)
{
  std::vector<uint8_t> ids(info_.joints.size(), 0);
  std::vector<int32_t> positions(info_.joints.size(), 0);
  std::vector<int32_t> velocities(info_.joints.size(), 0);

  std::copy(joint_ids_.begin(), joint_ids_.end(), ids.begin());

  // TODO: call the command interface and read the stepper motor states.
  // Update the joint states with the information coming from the hardware.

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type StepitHardware::write(const rclcpp::Time& time, const rclcpp::Duration& period)
{
  std::vector<uint8_t> joint_ids(info_.joints.size(), 0);
  std::vector<int32_t> joint_commands(info_.joints.size(), 0);

  std::copy(joint_ids_.begin(), joint_ids_.end(), joint_ids.begin());

  // TODO: send goals to the hardware.

  return hardware_interface::return_type::OK;
}

}  // namespace stepit_hardware

PLUGINLIB_EXPORT_CLASS(stepit_hardware::StepitHardware, hardware_interface::SystemInterface)
