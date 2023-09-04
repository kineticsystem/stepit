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

#include <mock/mock_cobs_serial.hpp>
#include <stepit_driver/default_driver.hpp>
#include <stepit_driver/msgs/msgs.hpp>

#include <cobs_serial/data_utils.hpp>

namespace stepit_driver::test
{
using ::testing::_;
using ::testing::Return;
using ::testing::SaveArg;

using namespace cobs_serial::data_utils;

/**
 * In this test we send a status query to the request interface,
 * we check the expected binary request and response.
 */
TEST(TestDefaultDriver, send_status_query)
{
  const std::vector<uint8_t> expected_request{
    0x75,  // query ID
  };

  const std::vector<uint8_t> mocked_response{
    0x11,  // status success
    0x00,  // motor ID
    0x46,  // position = 32100.0 (rad)
    0xFA,  // position
    0xC8,  // position
    0x00,  // position
    0x3F,  // speed = 0.5 (rad/s)
    0x00,  // speed
    0x00,  // speed
    0x00,  // speed
    0x43,  // distance to go = 150.0 (rad)
    0x16,  // distance to go
    0x00,  // distance to go
    0x00,  // distance to go
    0x01,  // motor ID
    0xC5,  // position = -6500
    0xCB,  // position
    0x20,  // position
    0x00,  // position
    0x3F,  // speed = 0.75 (rad/s)
    0x40,  // speed
    0x00,  // speed
    0x00,  // speed
    0x48,  // distance to go = 150000.0 (rad)
    0x12,  // distance to go
    0x7C,  // distance to go
    0x00,  // distance to go
  };

  std::vector<uint8_t> actual_request;
  auto serial = std::make_unique<MockCobsSerial>();
  EXPECT_CALL(*serial, write(_)).WillOnce(SaveArg<0>(&actual_request));
  EXPECT_CALL(*serial, read()).WillOnce(Return(mocked_response));

  auto driver = std::make_unique<stepit_driver::DefaultDriver>(std::move(serial));

  StatusResponse response = driver->get_status(rclcpp::Time{});

  ASSERT_THAT(to_hex(actual_request), to_hex(expected_request));

  ASSERT_EQ(static_cast<std::size_t>(2), response.motor_states().size());
  ASSERT_EQ(32100, response.motor_states()[0].position());
  ASSERT_EQ(-6500, response.motor_states()[1].position());
  ASSERT_EQ(0.5, response.motor_states()[0].velocity());
  ASSERT_EQ(0.75, response.motor_states()[1].velocity());
}

/**
 * In this test we send velocity goals to the request interface,
 * we check the expected binary request and response.
 */
TEST(TestDefaultDriver, send_velocity_command)
{
  const std::vector<uint8_t> expected_request{
    0x77,  // command ID
    0x00,  // motor ID
    0x3F,  // velocity = 0.5 (rad/s)
    0x00,  // velocity
    0x00,  // velocity
    0x00,  // velocity
    0x01,  // motor ID
    0x3F,  // velocity = 0.75 (rad/s)
    0x40,  // velocity
    0x00,  // velocity
    0x00   // velocity
  };

  const std::vector<uint8_t> mocked_response{
    0x11  // Status
  };

  std::vector<uint8_t> actual_request;
  auto serial = std::make_unique<MockCobsSerial>();
  EXPECT_CALL(*serial, write(_)).WillOnce(SaveArg<0>(&actual_request));
  EXPECT_CALL(*serial, read()).WillOnce(Return(mocked_response));

  auto driver = std::make_unique<stepit_driver::DefaultDriver>(std::move(serial));
  VelocityCommand request{ { VelocityGoal{ 0, 0.5 }, VelocityGoal{ 1, 0.75 } } };
  AcknowledgeResponse response = driver->set_velocity(rclcpp::Time{}, request);

  ASSERT_THAT(to_hex(actual_request), to_hex(expected_request));
  ASSERT_EQ(Response::Status::Success, response.status());
}

/**
 * In this test we send positions goals to the request interface,
 * we check the expected binary request and response.
 */
TEST(TestDefaultDriver, send_position_command)
{
  const std::vector<uint8_t> expected_request{
    0x71,  // command ID
    0x00,  // motor ID
    0x3F,  // position = 0.5 (rad)
    0x00,  // position
    0x00,  // position
    0x00,  // position
    0x01,  // motor ID
    0x3F,  // position = 0.75 (rad)
    0x40,  // position
    0x00,  // position
    0x00   // position
  };

  const std::vector<uint8_t> mocked_response{
    0x11  // Status
  };

  std::vector<uint8_t> actual_request;
  auto serial = std::make_unique<MockCobsSerial>();
  EXPECT_CALL(*serial, write(_)).WillOnce(SaveArg<0>(&actual_request));
  EXPECT_CALL(*serial, read()).WillOnce(Return(mocked_response));

  auto driver = std::make_unique<stepit_driver::DefaultDriver>(std::move(serial));
  PositionCommand request{ { PositionGoal{ 0, 0.5 }, PositionGoal{ 1, 0.75 } } };
  AcknowledgeResponse response = driver->set_position(rclcpp::Time{}, request);

  ASSERT_THAT(to_hex(actual_request), to_hex(expected_request));
  ASSERT_EQ(Response::Status::Success, response.status());
}

/**
 * In this test we we configure a set of motors and
 * check the expected binary request and response.
 */
TEST(TestDefaultDriver, send_configure_command)
{
  const std::vector<uint8_t> expected_request{
    0x78,  // command ID
    0x00,  // motor ID
    0x3F,  // acceleration = 0.5 (rad/s^2)
    0x00,  // acceleration
    0x00,  // acceleration
    0x00,  // acceleration
    0x3F,  // max_velocity = 0.75 (rad/s)
    0x40,  // max_velocity
    0x00,  // max_velocity
    0x00,  // max_velocity
    0x01,  // motor ID
    0x3F,  // acceleration = 0.5 (rad/s^2)
    0x00,  // acceleration
    0x00,  // acceleration
    0x00,  // acceleration
    0x3F,  // max_velocity = 0.75 (rad/s)
    0x40,  // max_velocity
    0x00,  // max_velocity
    0x00,  // max_velocity
  };

  const std::vector<uint8_t> mocked_response{
    0x11  // Status
  };

  std::vector<uint8_t> actual_request;
  auto serial = std::make_unique<MockCobsSerial>();
  EXPECT_CALL(*serial, write(_)).WillOnce(SaveArg<0>(&actual_request));
  EXPECT_CALL(*serial, read()).WillOnce(Return(mocked_response));

  auto driver = std::make_unique<stepit_driver::DefaultDriver>(std::move(serial));
  ConfigCommand request{ { ConfigParam{ 0, 0.5, 0.75 }, ConfigParam{ 1, 0.5, 0.75 } } };
  AcknowledgeResponse response = driver->configure(request);

  ASSERT_THAT(to_hex(actual_request), to_hex(expected_request));
  ASSERT_EQ(Response::Status::Success, response.status());
}

}  // namespace stepit_driver::test
