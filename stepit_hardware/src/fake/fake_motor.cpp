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

#include <stepit_hardware/fake/fake_motor.hpp>
#include <stepit_hardware/fake/position_control.hpp>
#include <stepit_hardware/fake/velocity_control.hpp>

namespace stepit_hardware
{
FakeMotor::FakeMotor(double acceleration, double max_velocity)
  : acceleration_{ acceleration }, max_velocity_{ max_velocity }, initial_position_{ 0.0 }, initial_velocity_{ 0.0 }
{
}

double FakeMotor::get_position([[maybe_unused]] const rclcpp::Time& time) const
{
  double t = time.seconds() - time_.seconds();
  if (control_type == ControlType::Position)
  {
    return position_control::position(max_velocity_, acceleration_, initial_velocity_, initial_position_,
                                      target_position_, t);
  }
  else
  {
    return velocity_control::position(max_velocity_, acceleration_, initial_position_, initial_velocity_,
                                      target_velocity_, t);
  }
}

void FakeMotor::set_target_position([[maybe_unused]] const rclcpp::Time& time, double position)
{
  time_ = time;
  target_position_ = position;
  control_type = ControlType::Position;
}

double FakeMotor::get_velocity([[maybe_unused]] const rclcpp::Time& time) const
{
  double t = time.seconds() - time_.seconds();
  if (control_type == ControlType::Position)
  {
    return position_control::velocity(max_velocity_, acceleration_, initial_velocity_, initial_position_,
                                      target_position_, t);
  }
  else
  {
    return velocity_control::velocity(max_velocity_, acceleration_, initial_velocity_, target_velocity_, t);
  }
}

void FakeMotor::set_target_velocity([[maybe_unused]] const rclcpp::Time& time, double velocity)
{
  time_ = time;
  target_velocity_ = velocity;
  control_type = ControlType::Velocity;
}
}  // namespace stepit_hardware
