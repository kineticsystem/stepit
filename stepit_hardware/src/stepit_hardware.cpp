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

#include <stepit_hardware/stepit_hardware.hpp>
#include <stepit_hardware/fake/fake_command_handler.hpp>
#include <stepit_hardware/msgs/status_query.hpp>
#include <stepit_hardware/msgs/velocity_command.hpp>
#include <stepit_hardware/msgs/position_command.hpp>
#include <stepit_hardware/msgs/acknowledge_response.hpp>
#include <stepit_hardware/msgs/status_response.hpp>

#include <stepit_hardware/command_handler.hpp>
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
constexpr auto kLogger = "StepitHardware";
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

// This constructor is use for testing only.
StepitHardware::StepitHardware(std::unique_ptr<CommandInterface> command_interface)
  : command_interface_{ std::move(command_interface) }
{
}

hardware_interface::CallbackReturn StepitHardware::on_init(const hardware_interface::HardwareInfo& info)
{
  RCLCPP_DEBUG(rclcpp::get_logger(kLogger), "on_init");
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  // Initialize all joints.

  joints_.resize(info_.joints.size(), Joint{});

  for (uint i = 0; i < info_.joints.size(); i++)
  {
    joints_[i].id = static_cast<uint8_t>(std::stoi(info_.joints[i].parameters.at("id")));
    joints_[i].acceleration = std::stod(info_.joints[i].parameters.at("acceleration"));
    joints_[i].max_velocity = std::stod(info_.joints[i].parameters.at("max_velocity"));
    joints_[i].state.position = kNaN;
    joints_[i].state.velocity = kNaN;
    joints_[i].command.position = kNaN;
    joints_[i].command.velocity = kNaN;
    RCLCPP_INFO(rclcpp::get_logger(kLogger), "joint_id %d: %d", i, joints_[i].id);
  }

  // If "use_dummy" is true return a fake hardware otherwise return a real one.

  if (!command_interface_)
  {
    if (info_.hardware_parameters.find("use_dummy") != info_.hardware_parameters.end() &&
        info_.hardware_parameters.at("use_dummy") == "true")
    {
      command_interface_ = std::make_unique<FakeCommandHandler>();
    }
    else
    {
      std::string usb_port = info.hardware_parameters.at("usb_port");
      RCLCPP_INFO(rclcpp::get_logger(kLogger), "usb_port: %s", usb_port.c_str());

      uint32_t baud_rate = static_cast<uint32_t>(std::stoul(info.hardware_parameters.at("baud_rate")));
      RCLCPP_INFO(rclcpp::get_logger(kLogger), "baud_rate: %d", baud_rate);

      double timeout = std::stod(info.hardware_parameters.at("timeout"));
      uint32_t timeout_ms = static_cast<uint32_t>(round(timeout * 1e3));
      RCLCPP_INFO(rclcpp::get_logger(kLogger), "timeout: %f", timeout);

      auto serial_handler = std::make_unique<SerialHandler>();
      serial_handler->set_port(usb_port);
      serial_handler->set_baudrate(baud_rate);
      serial_handler->set_timeout(timeout_ms);

      command_interface_ = std::make_unique<CommandHandler>(std::make_unique<DataHandler>(std::move(serial_handler)));
    }
  }

  // Open the serial port and handshake.
  command_interface_->init();

  // Send configuration parameters to the hardware.
  std::vector<ConfigCommand::Param> params;
  for (const auto joint : joints_)
  {
    params.emplace_back(ConfigCommand::Param{ joint.id, joint.acceleration, joint.max_velocity });
  }
  const AcknowledgeResponse response = command_interface_->send(ConfigCommand{ request_id++, params });
  if (response.status() == Response::Status::Failure)
  {
    return CallbackReturn::FAILURE;
  }

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> StepitHardware::export_state_interfaces()
{
  RCLCPP_DEBUG(rclcpp::get_logger(kLogger), "export_state_interfaces");
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
  RCLCPP_DEBUG(rclcpp::get_logger(kLogger), "export_command_interfaces");
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
  RCLCPP_DEBUG(rclcpp::get_logger(kLogger), "start");
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
StepitHardware::on_deactivate([[maybe_unused]] const rclcpp_lifecycle::State& previous_state)
{
  RCLCPP_DEBUG(rclcpp::get_logger(kLogger), "stop");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type StepitHardware::read(const rclcpp::Time& time,
                                                     [[maybe_unused]] const rclcpp::Duration& period)
{
  StatusQuery query{ request_id++ };
  StatusResponse response = command_interface_->send(time, query);

  auto motor_states = response.motor_states();
  if (motor_states.size() != joints_.size())
  {
    RCLCPP_ERROR(rclcpp::get_logger(kLogger), "incorrect number of joints");
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

// How to set velocity commands using the command line:
// ros2 topic pub /velocity_controller/commands std_msgs/msg/Float64MultiArray "data: [0.5,0.1]"
hardware_interface::return_type StepitHardware::write(const rclcpp::Time& time,
                                                      [[maybe_unused]] const rclcpp::Duration& period)
{
  if (std::any_of(joints_.cbegin(), joints_.cend(), [](auto joint) { return !std::isnan(joint.command.velocity); }))
  {
    // Set velocities.

    std::vector<VelocityCommand::Goal> velocities;
    for (const auto& joint : joints_)
    {
      if (!std::isnan(joint.command.velocity))
      {
        VelocityCommand::Goal velocity{ joint.id, joint.command.velocity };
        velocities.push_back(velocity);
      }
    }
    VelocityCommand command{ request_id++, velocities };
    AcknowledgeResponse response = command_interface_->send(time, command);
  }
  else
  {
    // Set positions.

    std::vector<PositionCommand::Goal> positions;
    for (const auto& joint : joints_)
    {
      if (!std::isnan(joint.command.position))
      {
        PositionCommand::Goal position{ joint.id, joint.command.position };
        positions.push_back(position);
      }
    }
    PositionCommand command{ request_id++, positions };
    AcknowledgeResponse response = command_interface_->send(time, command);
  }
  return hardware_interface::return_type::OK;
}

}  // namespace stepit_hardware

PLUGINLIB_EXPORT_CLASS(stepit_hardware::StepitHardware, hardware_interface::SystemInterface)
