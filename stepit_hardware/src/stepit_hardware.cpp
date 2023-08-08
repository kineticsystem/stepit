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
#include <stepit_hardware/msgs/status_query.hpp>
#include <stepit_hardware/msgs/velocity_command.hpp>
#include <stepit_hardware/msgs/position_command.hpp>
#include <stepit_hardware/msgs/acknowledge_response.hpp>
#include <stepit_hardware/msgs/status_response.hpp>

#include <stepit_hardware/default_request_interface_factory.hpp>

#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <hardware_interface/types/lifecycle_state_names.hpp>

#include <pluginlib/class_list_macros.hpp>

#include <limits>
#include <vector>
#include <string>
#include <cmath>

namespace stepit_hardware
{
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
const auto kLogger = rclcpp::get_logger("StepitHardware");

StepitHardware::StepitHardware()
{
  command_interface_factory_ = std::make_unique<DefaultRequestInterfaceFactory>();
}

// This constructor is use for testing only.
StepitHardware::StepitHardware(std::unique_ptr<RequestInterfaceFactory> command_interface_factory)
  : command_interface_factory_{ std::move(command_interface_factory) }
{
}

hardware_interface::CallbackReturn StepitHardware::on_init(const hardware_interface::HardwareInfo& info)
{
  RCLCPP_DEBUG(kLogger, "on_init");
  try
  {
    // Store hardware info for later use.

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
      RCLCPP_INFO(kLogger, "joint_id %d: %d", i, joints_[i].id);
    }

    command_interface_ = command_interface_factory_->create(info);
    return CallbackReturn::SUCCESS;
  }
  catch (const std::exception& ex)
  {
    set_state(rclcpp_lifecycle::State(lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED,
                                      hardware_interface::lifecycle_state_names::UNCONFIGURED));
    return CallbackReturn::ERROR;
  }
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
StepitHardware::on_configure(const rclcpp_lifecycle::State& previous_state)
{
  RCLCPP_DEBUG(kLogger, "on_configure");
  try
  {
    if (hardware_interface::SystemInterface::on_configure(previous_state) != CallbackReturn::SUCCESS)
    {
      return CallbackReturn::ERROR;
    }

    // Open the serial port and handshake.
    command_interface_->connect();

    // Send configuration parameters to the hardware.
    std::vector<ConfigParam> params;
    for (const auto joint : joints_)
    {
      params.emplace_back(ConfigParam{ joint.id, joint.acceleration, joint.max_velocity });
    }
    const AcknowledgeResponse response = command_interface_->send(ConfigCommand{ params });
    if (response.status() == Response::Status::Failure)
    {
      return CallbackReturn::FAILURE;
    }
    return CallbackReturn::SUCCESS;
  }
  catch (const std::exception& ex)
  {
    set_state(rclcpp_lifecycle::State(lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED,
                                      hardware_interface::lifecycle_state_names::UNCONFIGURED));
    return CallbackReturn::ERROR;
  }
}

std::vector<hardware_interface::StateInterface> StepitHardware::export_state_interfaces()
{
  RCLCPP_DEBUG(kLogger, "export_state_interfaces");
  try
  {
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
  catch (const std::exception& ex)
  {
    set_state(rclcpp_lifecycle::State(lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED,
                                      hardware_interface::lifecycle_state_names::UNCONFIGURED));
    return {};
  }
}

std::vector<hardware_interface::CommandInterface> StepitHardware::export_command_interfaces()
{
  RCLCPP_DEBUG(kLogger, "export_command_interfaces");
  try
  {
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
  catch (const std::exception& ex)
  {
    set_state(rclcpp_lifecycle::State(lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED,
                                      hardware_interface::lifecycle_state_names::UNCONFIGURED));
    return {};
  }
}

hardware_interface::CallbackReturn
StepitHardware::on_activate([[maybe_unused]] const rclcpp_lifecycle::State& previous_state)
{
  RCLCPP_DEBUG(kLogger, "on_activate");
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
StepitHardware::on_deactivate([[maybe_unused]] const rclcpp_lifecycle::State& previous_state)
{
  RCLCPP_DEBUG(kLogger, "on_deactivate");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type StepitHardware::read(const rclcpp::Time& time,
                                                     [[maybe_unused]] const rclcpp::Duration& period)
{
  try
  {
    StatusQuery query{};
    StatusResponse response = command_interface_->send(time, query);

    auto motor_states = response.motor_states();
    if (motor_states.size() != joints_.size())
    {
      RCLCPP_ERROR(kLogger, "incorrect number of joints");
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
  catch (const std::exception& ex)
  {
    set_state(rclcpp_lifecycle::State(lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED,
                                      hardware_interface::lifecycle_state_names::UNCONFIGURED));
    return hardware_interface::return_type::ERROR;
  }
}

// How to set velocity commands using the command line:
// ros2 topic pub /velocity_controller/commands std_msgs/msg/Float64MultiArray "data: [0.5,0.1]"
hardware_interface::return_type StepitHardware::write(const rclcpp::Time& time,
                                                      [[maybe_unused]] const rclcpp::Duration& period)
{
  try
  {
    if (std::any_of(joints_.cbegin(), joints_.cend(), [](auto joint) { return !std::isnan(joint.command.velocity); }))
    {
      // Set velocities.

      std::vector<VelocityGoal> velocities;
      for (const auto& joint : joints_)
      {
        if (!std::isnan(joint.command.velocity))
        {
          VelocityGoal velocity{ joint.id, joint.command.velocity };
          velocities.push_back(velocity);
        }
      }
      VelocityCommand command{ velocities };
      AcknowledgeResponse response = command_interface_->send(time, command);
    }
    else if (std::any_of(joints_.cbegin(), joints_.cend(),
                         [](auto joint) { return !std::isnan(joint.command.position); }))
    {
      // Set positions.

      std::vector<PositionGoal> positions;
      for (const auto& joint : joints_)
      {
        if (!std::isnan(joint.command.position))
        {
          PositionGoal position{ joint.id, joint.command.position };
          positions.push_back(position);
        }
      }
      PositionCommand command{ positions };
      AcknowledgeResponse response = command_interface_->send(time, command);
    }
    return hardware_interface::return_type::OK;
  }
  catch (const std::exception& ex)
  {
    set_state(rclcpp_lifecycle::State(lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED,
                                      hardware_interface::lifecycle_state_names::UNCONFIGURED));
    return hardware_interface::return_type::ERROR;
  }
}

}  // namespace stepit_hardware

PLUGINLIB_EXPORT_CLASS(stepit_hardware::StepitHardware, hardware_interface::SystemInterface)
