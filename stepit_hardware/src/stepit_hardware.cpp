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
#include <stepit_hardware/data_utils.hpp>
#include <stepit_hardware/msgs/motor_status_query.hpp>
#include <stepit_hardware/msgs/motor_velocity_command.hpp>
#include <stepit_hardware/msgs/motor_position_command.hpp>
#include <stepit_hardware/msgs/acknowledge_response.hpp>
#include <stepit_hardware/msgs/motor_status_response.hpp>

#include <stepit_hardware/data_handler.hpp>
#include <stepit_hardware/serial_handler.hpp>

#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

#include <pluginlib/class_list_macros.hpp>

#include <limits>
#include <vector>
#include <string>
#include <cmath>

namespace stepit_hardware
{
constexpr auto kStepitHardware = "StepitHardware";
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

hardware_interface::CallbackReturn StepitHardware::on_init(const hardware_interface::HardwareInfo& info)
{
  RCLCPP_DEBUG(rclcpp::get_logger(kStepitHardware), "on_init");
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  // Initialize as many joint states as specified in the configuration.

  joints_.resize(info_.joints.size(), Joint{});

  for (uint i = 0; i < info_.joints.size(); i++)
  {
    joints_[i].id = static_cast<uint8_t>(std::stoi(info_.joints[i].parameters.at("id")));
    joints_[i].state.position = kNaN;
    joints_[i].state.velocity = kNaN;
    joints_[i].command.position = kNaN;
    joints_[i].command.velocity = kNaN;
    RCLCPP_INFO(rclcpp::get_logger(kStepitHardware), "joint_id %d: %d", i, joints_[i].id);
  }

  // Read serial port connection parameters.

  std::string usb_port = info_.hardware_parameters.at("usb_port");
  RCLCPP_INFO(rclcpp::get_logger(kStepitHardware), "usb_port: %s", usb_port.c_str());

  uint32_t baud_rate = data_utils::stoui32(info_.hardware_parameters.at("baud_rate"));

  //  uint32_t baud_rate = std::sto(info_.hardware_parameters.at("baud_rate"));
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

  //  const rclcpp::Time time;
  //  const rclcpp::Duration period = rclcpp::Duration::from_seconds(0);

  //  // Make goals match current joint states.
  //  read(time, period);
  //  for (auto& joint : joints_)
  //  {
  //    joint.command.position = joint.state.position;
  //    joint.command.velocity = 0.0;
  //  }
  //  write(time, period);

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
StepitHardware::on_deactivate([[maybe_unused]] const rclcpp_lifecycle::State& previous_state)
{
  RCLCPP_DEBUG(rclcpp::get_logger(kStepitHardware), "stop");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type StepitHardware::read([[maybe_unused]] const rclcpp::Time& time,
                                                     [[maybe_unused]] const rclcpp::Duration& period)
{
  MotorStatusQuery query{ request_id++ };
  data_interface_->write(query.bytes());
  auto data = data_interface_->read();
  MotorStatusResponse response{ data };

  auto motor_states = response.motor_states();
  if (motor_states.size() != joints_.size())
  {
    RCLCPP_ERROR(rclcpp::get_logger(kStepitHardware), "incorrect number of joints");
    return hardware_interface::return_type::ERROR;
  }

  for (std::size_t i = 0; i < motor_states.size(); ++i)
  {
    uint8_t motor_id = motor_states[i].id();
    joints_[motor_id].state.position = motor_states[i].position();
    joints_[motor_id].state.velocity = motor_states[i].velocity();
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type StepitHardware::write([[maybe_unused]] const rclcpp::Time& time,
                                                      [[maybe_unused]] const rclcpp::Duration& period)
{
  if (std::any_of(joints_.cbegin(), joints_.cend(), [](auto joint) { return !std::isnan(joint.command.velocity); }))
  {
    // Set velocities.

    std::vector<MotorVelocityCommand::Goal> velocities;
    for (const auto& joint : joints_)
    {
      if (!std::isnan(joint.command.velocity))
      {
        MotorVelocityCommand::Goal velocity{ joint.id, joint.command.velocity };
        velocities.push_back(velocity);
      }
    }
    MotorVelocityCommand velocity_command{ request_id++, velocities };
    data_interface_->write(velocity_command.bytes());
    AcknowledgeResponse veocity_command_response{ data_interface_->read() };
  }
  else
  {
    // Set positions.

    std::vector<MotorPositionCommand::Goal> positions;
    for (const auto& joint : joints_)
    {
      if (!std::isnan(joint.command.position))
      {
        MotorPositionCommand::Goal position{ joint.id, joint.command.position };
        positions.push_back(position);
      }
    }
    MotorPositionCommand position_command{ request_id++, positions };
    data_interface_->write(position_command.bytes());
    AcknowledgeResponse position_command_response{ data_interface_->read() };
  }
  return hardware_interface::return_type::OK;
}

void StepitHardware::set_data_interface(std::unique_ptr<DataInterface> data_interface)
{
  data_interface_ = std::move(data_interface);
}

}  // namespace stepit_hardware

PLUGINLIB_EXPORT_CLASS(stepit_hardware::StepitHardware, hardware_interface::SystemInterface)
