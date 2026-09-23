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

#include <algorithm>

#include <stepit_driver/fake/fake_driver.hpp>
#include <stepit_driver/hardware_limits.hpp>

#include <rclcpp/rclcpp.hpp>

namespace stepit_driver
{
const auto kLogger = rclcpp::get_logger("stepit_fake_driver");

using hardware_limits::kTolerance;

bool FakeDriver::connect()
{
  return true;
}

void FakeDriver::disconnect()
{
}

AcknowledgeResponse FakeDriver::configure(const ConfigCommand& command) const
{
  // Validate everything before creating any motor, mirroring the firmware:
  // a configuration the real controller refuses has to fail here too, or a
  // robot description that works in simulation will fail to activate the
  // moment it runs on hardware. The comparisons reject a NaN as well; a non
  // positive acceleration would also divide by zero in the kinematics.
  for (const auto& param : command.params())
  {
    if (param.motor_id() >= hardware_limits::kMotorCount)
    {
      RCLCPP_ERROR(kLogger, "Motor id %d does not exist: the controller drives %zu motors.", param.motor_id(),
                   hardware_limits::kMotorCount);
      return AcknowledgeResponse{ Response::Status::Failure };
    }
    if (!(param.acceleration() > 0.0) || param.acceleration() > hardware_limits::kMaxAcceleration * kTolerance)
    {
      RCLCPP_ERROR(kLogger, "Motor %d: acceleration %f rad/s^2 is not in (0, %f].", param.motor_id(),
                   param.acceleration(), hardware_limits::kMaxAcceleration);
      return AcknowledgeResponse{ Response::Status::Failure };
    }
    if (!(param.max_velocity() > 0.0) || param.max_velocity() > hardware_limits::kMaxVelocity * kTolerance)
    {
      RCLCPP_ERROR(kLogger, "Motor %d: max velocity %f rad/s is not in (0, %f].", param.motor_id(),
                   param.max_velocity(), hardware_limits::kMaxVelocity);
      return AcknowledgeResponse{ Response::Status::Failure };
    }
  }

  motors_.clear();
  for (const auto& param : command.params())
  {
    FakeMotor motor;
    // Within tolerance of a limit: keep the limit itself, as the firmware does.
    motor.set_acceleration(std::min(param.acceleration(), hardware_limits::kMaxAcceleration));
    motor.set_max_velocity(std::min(param.max_velocity(), hardware_limits::kMaxVelocity));
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
      // The firmware rejects the whole command in this case, so do the same.
      RCLCPP_ERROR(kLogger, "Motor id %d does not exist.", goal.motor_id());
      return AcknowledgeResponse{ Response::Status::Failure };
    }
    it->second.set_target_position(time, goal.position());
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
      // The firmware rejects the whole command in this case, so do the same.
      RCLCPP_ERROR(kLogger, "Motor id %d does not exist.", goal.motor_id());
      return AcknowledgeResponse{ Response::Status::Failure };
    }
    it->second.set_target_velocity(time, goal.velocity());
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
  // Report the same limits the validation above enforces, so that a host
  // asking the simulated controller what it can do gets the same answer the
  // real one would give.
  std::vector<MotorLimits> limits;
  for (std::size_t id = 0; id < hardware_limits::kMotorCount; ++id)
  {
    limits.emplace_back(
        MotorLimits{ static_cast<uint8_t>(id), hardware_limits::kMaxAcceleration, hardware_limits::kMaxVelocity });
  }
  return InfoResponse(Response::Status::Success, "STEPIT", hardware_limits::kFirmwareVersion, limits);
}
}  // namespace stepit_driver
