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

/**
 * @brief The FakeCommandHandler class receives commands and queries from the
 * hardware interface, sends them to a fake hardware and returns a
 * corresponding response.
 */
FakeCommandHandler::FakeCommandHandler()
{
}

AcknowledgeResponse FakeCommandHandler::send([[maybe_unused]] const rclcpp::Time& time,
                                             const MotorConfigCommand& command) const
{
  motors_.clear();
  for (const auto& param : command.params())
  {
    FakeMotor motor;
    motor.set_acceleration(param.acceleration());
    motor.set_max_velocity(param.max_velocity());
    motors_.emplace_back(motor);
  }

  return AcknowledgeResponse{ command.request_id(), Response::Status::Success };
}

AcknowledgeResponse FakeCommandHandler::send(const rclcpp::Time& time, const MotorPositionCommand& command) const
{
  for (const auto& goal : command.goals())
  {
    const auto motor_id = goal.motor_id();
    if (motor_id >= motors_.size())
    {
      RCLCPP_ERROR(rclcpp::get_logger(kLoggerName), "Motor id does not exist.");
    }
    else
    {
      motors_[motor_id].set_target_position(time, goal.position());
    }
  }
  return AcknowledgeResponse{ command.request_id(), Response::Status::Success };
}

AcknowledgeResponse FakeCommandHandler::send(const rclcpp::Time& time, const MotorVelocityCommand& command) const
{
  for (const auto& goal : command.goals())
  {
    const auto motor_id = goal.motor_id();
    if (motor_id >= motors_.size())
    {
      RCLCPP_ERROR(rclcpp::get_logger(kLoggerName), "Motor id does not exist.");
    }
    else
    {
      motors_[motor_id].set_target_velocity(time, goal.velocity());
    }
  }
  return AcknowledgeResponse{ command.request_id(), Response::Status::Success };
}

MotorStatusResponse FakeCommandHandler::send(const rclcpp::Time& time, const MotorStatusQuery& query) const
{
  uint8_t id = 0;
  std::vector<MotorStatusResponse::MotorState> states;
  for (const auto& motor : motors_)
  {
    auto position = motor.get_position(time);
    auto velocity = motor.get_velocity(time);
    auto distance_to_go = 0.0;
    MotorStatusResponse::MotorState state{ id++, position, velocity, distance_to_go };
    states.emplace_back(state);
  }
  return MotorStatusResponse(query.request_id(), Response::Status::Success, states);
}
}  // namespace stepit_hardware
