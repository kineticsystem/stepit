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

#include <stepit_driver/fake/fake_motor.hpp>
#include <stepit_driver/fake/position_control.hpp>
#include <stepit_driver/fake/velocity_control.hpp>

#include <rclcpp/rclcpp.hpp>

namespace stepit_driver
{

FakeMotor::FakeMotor() : acceleration_{ 0.0 }, max_velocity_{ 0.0 }, initial_position_{ 0.0 }, initial_velocity_{ 0.0 }
{
}

void FakeMotor::set_acceleration(double acceleration)
{
  acceleration_ = acceleration;
}

void FakeMotor::set_max_velocity(double velocity)
{
  max_velocity_ = velocity;
}

double FakeMotor::get_position(const rclcpp::Time& time) const
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

void FakeMotor::set_target_position(const rclcpp::Time& time, double position)
{
  initial_position_ = get_position(time);
  initial_velocity_ = get_velocity(time);
  target_position_ = position;
  control_type = ControlType::Position;
  time_ = time;
}

double FakeMotor::get_velocity(const rclcpp::Time& time) const
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

void FakeMotor::set_target_velocity(const rclcpp::Time& time, double velocity)
{
  initial_position_ = get_position(time);
  initial_velocity_ = get_velocity(time);
  target_velocity_ = velocity;
  control_type = ControlType::Velocity;
  time_ = time;
}
}  // namespace stepit_driver
