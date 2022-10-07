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

rclcpp::Time dtot(double seconds)
{
  return rclcpp::Time{ static_cast<int64_t>(seconds * 1e9) };
}

TEST(TestFakeMotor, test_position_zero_initial_velocity)
{
  static constexpr double EPSILON = 1E-12;

  FakeMotor motor;
  motor.set_acceleration(3.14159);
  motor.set_max_velocity(6.28319);
  motor.set_target_velocity(rclcpp::Time{ 0, 0 }, 3.14159);

  double t[] = { 0.1, 0.2, 0.3, 0.4, 0.5, 1, 2 };
  double x[] = { 0.01570795, 0.0628318, 0.14137155, 0.2513272, 0.39269875, 1.570795, 4.712385 };

  ASSERT_NEAR(motor.get_position(dtot(t[0])), x[0], EPSILON);
  ASSERT_NEAR(motor.get_position(dtot(t[1])), x[1], EPSILON);
  ASSERT_NEAR(motor.get_position(dtot(t[2])), x[2], EPSILON);
  ASSERT_NEAR(motor.get_position(dtot(t[3])), x[3], EPSILON);
  ASSERT_NEAR(motor.get_position(dtot(t[4])), x[4], EPSILON);
  ASSERT_NEAR(motor.get_position(dtot(t[5])), x[5], EPSILON);
  ASSERT_NEAR(motor.get_position(dtot(t[6])), x[6], EPSILON);
}

TEST(TestFakeMotor, test_velocity_zero_initial_velocity)
{
  static constexpr double EPSILON = 1E-12;

  FakeMotor motor;
  motor.set_acceleration(3.14159);
  motor.set_max_velocity(6.28319);
  motor.set_target_velocity(rclcpp::Time{ 0, 0 }, 3.14159);

  double t[] = { 0.1, 0.2, 0.3, 0.4, 0.5, 1, 2 };
  double v[] = { 0.314159, 0.628318, 0.942477, 1.256636, 1.570795, 3.14159, 3.14159 };

  ASSERT_NEAR(motor.get_velocity(dtot(t[0])), v[0], EPSILON);
  ASSERT_NEAR(motor.get_velocity(dtot(t[1])), v[1], EPSILON);
  ASSERT_NEAR(motor.get_velocity(dtot(t[2])), v[2], EPSILON);
  ASSERT_NEAR(motor.get_velocity(dtot(t[3])), v[3], EPSILON);
  ASSERT_NEAR(motor.get_velocity(dtot(t[4])), v[4], EPSILON);
  ASSERT_NEAR(motor.get_velocity(dtot(t[5])), v[5], EPSILON);
  ASSERT_NEAR(motor.get_velocity(dtot(t[6])), v[6], EPSILON);
}

TEST(TestFakeMotor, test_sequence_of_velocity_commands)
{
  static constexpr double EPSILON = 1E-12;

  FakeMotor motor;
  motor.set_acceleration(3.14159);
  motor.set_max_velocity(6.28319);

  double t[] = { 0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 1, 2 };
  double v[] = { 0.0, 0.314159, 0.628318, 0.942477, 1.256636, 1.570795, 3.14159, 3.14159 };

  motor.set_target_velocity(dtot(t[0]), 3.14159);

  ASSERT_NEAR(motor.get_velocity(dtot(t[1])), v[1], EPSILON);
  motor.set_target_velocity(dtot(t[1]), 3.14159);

  ASSERT_NEAR(motor.get_velocity(dtot(t[2])), v[2], EPSILON);
  motor.set_target_velocity(dtot(t[2]), 3.14159);

  ASSERT_NEAR(motor.get_velocity(dtot(t[3])), v[3], EPSILON);
  motor.set_target_velocity(dtot(t[3]), 3.14159);

  ASSERT_NEAR(motor.get_velocity(dtot(t[4])), v[4], EPSILON);
  motor.set_target_velocity(dtot(t[4]), 3.14159);

  ASSERT_NEAR(motor.get_velocity(dtot(t[5])), v[5], EPSILON);
  motor.set_target_velocity(dtot(t[5]), 3.14159);

  ASSERT_NEAR(motor.get_velocity(dtot(t[6])), v[6], EPSILON);
  motor.set_target_velocity(dtot(t[6]), 3.14159);

  ASSERT_NEAR(motor.get_velocity(dtot(t[7])), v[7], EPSILON);
}
}  // namespace stepit_hardware::test
