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

#include <gtest/gtest.h>
#include <stepit_hardware/fake/fake_motor.hpp>

#include <rclcpp/rclcpp.hpp>

namespace stepit_hardware::test
{
TEST(TestFakeMotor, test)
{
  static constexpr double EPSILON = 1E-5;

  double acceleration = 3.14159;  // Rad/s^2
  double max_velocity = 6.28319;  // Rad/s
  double target_velocity = 0.5 * max_velocity;

  FakeMotor motor;
  motor.set_acceleration(acceleration);
  motor.set_max_velocity(max_velocity);
  motor.set_target_velocity(rclcpp::Time{ 0, 0 }, target_velocity);

  ASSERT_NEAR(motor.get_velocity(rclcpp::Time{ 0, 500000000 }), 0.5 * target_velocity, EPSILON);
  ASSERT_NEAR(motor.get_velocity(rclcpp::Time{ 1, 0 }), target_velocity, EPSILON);
  ASSERT_NEAR(motor.get_velocity(rclcpp::Time{ 2, 0 }), target_velocity, EPSILON);
  ASSERT_NEAR(motor.get_velocity(rclcpp::Time{ 3, 0 }), target_velocity, EPSILON);
}
}  // namespace stepit_hardware::test
