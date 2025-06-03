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

#include <stepit_driver/fake/fake_driver.hpp>

#include <rclcpp/rclcpp.hpp>

namespace stepit_driver
{
const auto kLogger = rclcpp::get_logger("stepit_fake_driver");

bool FakeDriver::connect()
{
  return true;
}

void FakeDriver::disconnect()
{
}

AcknowledgeResponse FakeDriver::configure(const ConfigCommand& command) const
{
  motors_.clear();
  for (const auto& param : command.params())
  {
    FakeMotor motor;
    motor.set_acceleration(param.acceleration());
    motor.set_max_velocity(param.max_velocity());
    motors_.insert({ param.motor_id(), motor });
  }

  return AcknowledgeResponse{ Response::Status::Success };
}

AcknowledgeResponse FakeDriver::set_position(const rclcpp::Time& time, const PositionCommand& command) const
{
  for (const auto& goal : command.goals())
  {
    auto it = motors_.find(goal.motor_id());
    if (it == motors_.end())
    {
      RCLCPP_ERROR(kLogger, "Motor id does not exist.");
    }
    else
    {
      it->second.set_target_position(time, goal.position());
    }
  }
  return AcknowledgeResponse{ Response::Status::Success };
}

AcknowledgeResponse FakeDriver::set_velocity(const rclcpp::Time& time, const VelocityCommand& command) const
{
  for (const auto& goal : command.goals())
  {
    auto it = motors_.find(goal.motor_id());
    if (it == motors_.end())
    {
      RCLCPP_ERROR(kLogger, "Motor id does not exist.");
    }
    else
    {
      it->second.set_target_velocity(time, goal.velocity());
    }
  }
  return AcknowledgeResponse{ Response::Status::Success };
}

StatusResponse FakeDriver::get_status(const rclcpp::Time& time) const
{
  std::vector<MotorState> states;
  for (const auto& [motor_id, motor] : motors_)
  {
    auto position = motor.get_position(time);
    auto velocity = motor.get_velocity(time);
    auto distance_to_go = 0.0;
    MotorState state{ motor_id, position, velocity, distance_to_go };
    states.emplace_back(state);
  }
  return StatusResponse(Response::Status::Success, states);
}

InfoResponse FakeDriver::get_info([[maybe_unused]] const rclcpp::Time& time) const
{
  return InfoResponse(Response::Status::Success, "STEPIT");
}
}  // namespace stepit_driver
