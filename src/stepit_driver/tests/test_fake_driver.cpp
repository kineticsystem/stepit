// Copyright 2026 Giovanni Remigi
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

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <numbers>
#include <vector>

#include <stepit_driver/fake/fake_driver.hpp>
#include <stepit_driver/hardware_limits.hpp>

namespace stepit_driver::test
{
using hardware_limits::kMaxAcceleration;
using hardware_limits::kMaxVelocity;

/** A configuration for one motor, at the limits unless stated otherwise. */
ConfigCommand config(uint8_t motor_id, double acceleration = kMaxAcceleration, double max_velocity = kMaxVelocity)
{
  return ConfigCommand{ { ConfigParam{ motor_id, acceleration, max_velocity } } };
}

/**
 * A configuration within the limits of the controller is accepted.
 */
TEST(TestFakeDriver, configure_accepts_values_within_limits)
{
  FakeDriver driver;
  EXPECT_EQ(Response::Status::Success, driver.configure(config(0)).status());
  EXPECT_EQ(Response::Status::Success, driver.configure(config(0, kMaxAcceleration / 2, kMaxVelocity / 2)).status());
}

/**
 * The values the URDF declares are exactly the limits of the controller, so
 * they have to be accepted: were they not, simulation would refuse the very
 * configuration the robot ships with.
 */
TEST(TestFakeDriver, configure_accepts_the_declared_limits)
{
  // The literals used in stepit.ros2_control.xacro.
  const double urdf_acceleration = 12.5663706143592;
  const double urdf_max_velocity = 18.8495559215388;

  FakeDriver driver;
  EXPECT_EQ(Response::Status::Success, driver.configure(config(0, urdf_acceleration, urdf_max_velocity)).status());
}

/**
 * A configuration the real controller would refuse has to be refused here
 * too, otherwise a robot that runs in simulation fails to activate on
 * hardware.
 */
TEST(TestFakeDriver, configure_rejects_values_beyond_the_limits)
{
  FakeDriver driver;
  EXPECT_EQ(Response::Status::Failure, driver.configure(config(0, kMaxAcceleration * 2, kMaxVelocity)).status());
  EXPECT_EQ(Response::Status::Failure, driver.configure(config(0, kMaxAcceleration, kMaxVelocity * 2)).status());
}

/**
 * A non positive acceleration is refused: besides being meaningless, it
 * divides by zero in the position kinematics.
 */
TEST(TestFakeDriver, configure_rejects_non_positive_values)
{
  FakeDriver driver;
  EXPECT_EQ(Response::Status::Failure, driver.configure(config(0, 0.0, kMaxVelocity)).status());
  EXPECT_EQ(Response::Status::Failure, driver.configure(config(0, -kMaxAcceleration, kMaxVelocity)).status());
  EXPECT_EQ(Response::Status::Failure, driver.configure(config(0, kMaxAcceleration, 0.0)).status());
  EXPECT_EQ(Response::Status::Failure, driver.configure(config(0, kMaxAcceleration, -kMaxVelocity)).status());
}

/**
 * A motor id the controller does not drive is refused.
 */
TEST(TestFakeDriver, configure_rejects_an_unknown_motor_id)
{
  FakeDriver driver;
  EXPECT_EQ(Response::Status::Failure, driver.configure(config(99)).status());
}

/**
 * A batch whose last entry is invalid is refused as a whole: no motor is
 * configured, as on the firmware.
 */
TEST(TestFakeDriver, configure_rejects_a_batch_with_one_bad_entry)
{
  FakeDriver driver;
  const ConfigCommand command{ { ConfigParam{ 0, kMaxAcceleration, kMaxVelocity },
                                 ConfigParam{ 1, kMaxAcceleration, kMaxVelocity },
                                 ConfigParam{ 2, kMaxAcceleration * 99, kMaxVelocity } } };

  EXPECT_EQ(Response::Status::Failure, driver.configure(command).status());

  // Nothing was configured, so even the valid motors are unknown.
  EXPECT_EQ(Response::Status::Failure,
            driver.set_velocity(rclcpp::Time{}, VelocityCommand{ { VelocityGoal{ 0, 0.0 } } }).status());
}

/**
 * Commanding a motor that was never configured fails, as it does on the
 * firmware, instead of being silently ignored.
 */
TEST(TestFakeDriver, motion_commands_reject_an_unknown_motor_id)
{
  FakeDriver driver;
  ASSERT_EQ(Response::Status::Success, driver.configure(config(0)).status());

  EXPECT_EQ(Response::Status::Success,
            driver.set_velocity(rclcpp::Time{}, VelocityCommand{ { VelocityGoal{ 0, 0.0 } } }).status());
  EXPECT_EQ(Response::Status::Failure,
            driver.set_velocity(rclcpp::Time{}, VelocityCommand{ { VelocityGoal{ 3, 0.0 } } }).status());
  EXPECT_EQ(Response::Status::Failure,
            driver.set_position(rclcpp::Time{}, PositionCommand{ { PositionGoal{ 3, 0.0 } } }).status());
}

}  // namespace stepit_driver::test
