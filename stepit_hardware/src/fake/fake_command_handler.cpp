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

#include <stepit_hardware/fake/fake_command_handler.hpp>

#include <rclcpp/rclcpp.hpp>

namespace stepit_hardware
{
constexpr auto kLoggerName = "FakeCommandHandler";

bool FakeCommandHandler::connect()
{
  return true;
}

void FakeCommandHandler::disconnect()
{
}

AcknowledgeResponse FakeCommandHandler::send(const ConfigCommand& command) const
{
  motors_.clear();
  for (const auto& param : command.params())
  {
    FakeMotor motor;
    motor.set_acceleration(param.acceleration());
    motor.set_max_velocity(param.max_velocity());
    motors_.insert({ param.motor_id(), motor });
  }

  return AcknowledgeResponse{ command.request_id(), Response::Status::Success };
}

AcknowledgeResponse FakeCommandHandler::send(const rclcpp::Time& time, const PositionCommand& command) const
{
  for (const auto& goal : command.goals())
  {
    auto it = motors_.find(goal.motor_id());
    if (it == motors_.end())
    {
      RCLCPP_ERROR(rclcpp::get_logger(kLoggerName), "Motor id does not exist.");
    }
    else
    {
      it->second.set_target_position(time, goal.position());
    }
  }
  return AcknowledgeResponse{ command.request_id(), Response::Status::Success };
}

AcknowledgeResponse FakeCommandHandler::send(const rclcpp::Time& time, const VelocityCommand& command) const
{
  for (const auto& goal : command.goals())
  {
    auto it = motors_.find(goal.motor_id());
    if (it == motors_.end())
    {
      RCLCPP_ERROR(rclcpp::get_logger(kLoggerName), "Motor id does not exist.");
    }
    else
    {
      it->second.set_target_velocity(time, goal.velocity());
    }
  }
  return AcknowledgeResponse{ command.request_id(), Response::Status::Success };
}

StatusResponse FakeCommandHandler::send(const rclcpp::Time& time, const StatusQuery& query) const
{
  std::vector<StatusResponse::MotorState> states;
  for (const auto& [motor_id, motor] : motors_)
  {
    auto position = motor.get_position(time);
    auto velocity = motor.get_velocity(time);
    auto distance_to_go = 0.0;
    StatusResponse::MotorState state{ motor_id, position, velocity, distance_to_go };
    states.emplace_back(state);
  }
  return StatusResponse(query.request_id(), Response::Status::Success, states);
}

InfoResponse FakeCommandHandler::send([[maybe_unused]] const rclcpp::Time& time, const InfoQuery& query) const
{
  return InfoResponse(query.request_id(), Response::Status::Success, "STEPIT");
}
}  // namespace stepit_hardware
