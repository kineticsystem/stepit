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

#pragma once

#include <rclcpp/rclcpp.hpp>

namespace stepit_driver
{
/**
 * @brief The FakeMotor class simulates the kinematic of a real stepped motor
 * with given acceleration and absolute maximum velocity.
 */
class FakeMotor
{
public:
  FakeMotor();

  /**
   * @brief Set the motor acceleration.
   * @param acceleration The motor acceleration.
   */
  void set_acceleration(double acceleration);

  /**
   * @brief Set the motor absolute maximum velocity.
   * @param velocity The motor absolute maximum velocity.
   */
  void set_max_velocity(double velocity);

  /**
   * @brief Get the motor position at the given time.
   * @param time The time the position is computed.
   * @return The motor position at the given time.
   */
  double get_position(const rclcpp::Time& time) const;

  /**
   * @brief Set the motor target position.
   * @param time The time the command is received.
   * @param position The motor target position.
   */
  void set_target_position(const rclcpp::Time& time, double position);

  /**
   * @brief Get the motor velocity at the given time.
   * @param time The time the velocity is computed.
   * @return the motor velocity at the given time.
   */
  double get_velocity(const rclcpp::Time& time) const;

  /**
   * @brief Set the motor target velocity.
   * @param time The time the command is received.
   * @param velocity The motor target velocity.
   */
  void set_target_velocity(const rclcpp::Time& time, double velocity);

private:
  /* The motor acceleration (rad/s^2) */
  double acceleration_;

  /* The motor maximum velocity. */
  double max_velocity_;

  /* The motor position (rad) at the time a command is received. */
  double initial_position_;

  /* The motor velocity (rad/s) at the time a command is received. */
  double initial_velocity_;

  /* The motor target position. */
  double target_position_;

  /* The motor target velocity. */
  double target_velocity_;

  /* This store the time when a command is last received. */
  rclcpp::Time time_{ 0 };

  enum ControlType
  {
    Velocity,
    Position
  };

  /* Set the motor to be in velocity or position controlled mode depending on
   * the last received command.
   */
  ControlType control_type = ControlType::Position;
};
}  // namespace stepit_driver
