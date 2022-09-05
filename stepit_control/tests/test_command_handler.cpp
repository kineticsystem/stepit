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

#include <mock/mock_data_interface.hpp>
#include <stepit_control/command_handler.hpp>
#include <stepit_control/data_utils.hpp>
#include <stepit_control/msgs/msgs.hpp>

namespace stepit_control::test
{
using ::testing::_;
using ::testing::Return;
using ::testing::SaveArg;

/**
 * In this test we send a status query to the command handler,
 * we check the expected binary request and response.
 */
TEST(TestCommandHandler, send_status_query)
{
  const std::vector<uint8_t> expected_request{
    0x00,  // request ID
    0x75,  // query ID
  };

  const std::vector<uint8_t> mocked_response{
    0x00,  // request ID
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
  auto mock_data_interface = std::make_unique<MockDataInterface>();
  EXPECT_CALL(*mock_data_interface, write(_)).WillOnce(SaveArg<0>(&actual_request));
  EXPECT_CALL(*mock_data_interface, read()).WillOnce(Return(mocked_response));

  auto command_handler = std::make_unique<stepit_control::CommandHandler>(std::move(mock_data_interface));
  MotorStatusQuery request{ 0 };

  MotorStatusResponse response = command_handler->send(rclcpp::Time{}, request);

  ASSERT_THAT(stepit_control::data_utils::to_hex(actual_request), stepit_control::data_utils::to_hex(expected_request));

  ASSERT_EQ(static_cast<std::size_t>(2), response.motor_states().size());
  ASSERT_EQ(32100, response.motor_states()[0].position());
  ASSERT_EQ(-6500, response.motor_states()[1].position());
  ASSERT_EQ(0.5, response.motor_states()[0].velocity());
  ASSERT_EQ(0.75, response.motor_states()[1].velocity());
}

/**
 * In this test we send velocity goals to the command handler,
 * we check the expected binary request and response.
 */
TEST(TestCommandHandler, send_velocity_command)
{
  const std::vector<uint8_t> expected_request{
    0x00,  // request ID
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
    0x00,  // Request Id
    0x11   // Status
  };

  std::vector<uint8_t> actual_request;
  auto mock_data_interface = std::make_unique<MockDataInterface>();
  EXPECT_CALL(*mock_data_interface, write(_)).WillOnce(SaveArg<0>(&actual_request));
  EXPECT_CALL(*mock_data_interface, read()).WillOnce(Return(mocked_response));

  auto command_handler = std::make_unique<stepit_control::CommandHandler>(std::move(mock_data_interface));
  MotorVelocityCommand request{ 0, { MotorVelocityCommand::Goal{ 0, 0.5 }, MotorVelocityCommand::Goal{ 1, 0.75 } } };
  AcknowledgeResponse response = command_handler->send(rclcpp::Time{}, request);

  ASSERT_THAT(stepit_control::data_utils::to_hex(actual_request), stepit_control::data_utils::to_hex(expected_request));
  ASSERT_EQ(0, response.request_id());
  ASSERT_EQ(Response::Status::Success, response.status());
}

/**
 * In this test we send positions goals to the command handler,
 * we check the expected binary request and response.
 */
TEST(TestCommandHandler, send_position_command)
{
  const std::vector<uint8_t> expected_request{
    0x00,  // request ID
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
    0x00,  // Request Id
    0x11   // Status
  };

  std::vector<uint8_t> actual_request;
  auto mock_data_interface = std::make_unique<MockDataInterface>();
  EXPECT_CALL(*mock_data_interface, write(_)).WillOnce(SaveArg<0>(&actual_request));
  EXPECT_CALL(*mock_data_interface, read()).WillOnce(Return(mocked_response));

  auto command_handler = std::make_unique<stepit_control::CommandHandler>(std::move(mock_data_interface));
  MotorPositionCommand request{ 0, { MotorPositionCommand::Goal{ 0, 0.5 }, MotorPositionCommand::Goal{ 1, 0.75 } } };
  AcknowledgeResponse response = command_handler->send(rclcpp::Time{}, request);

  ASSERT_THAT(stepit_control::data_utils::to_hex(actual_request), stepit_control::data_utils::to_hex(expected_request));
  ASSERT_EQ(0, response.request_id());
  ASSERT_EQ(Response::Status::Success, response.status());
}

/**
 * In this test we we configure a set of motors and
 * check the expected binary request and response.
 */
TEST(TestCommandHandler, send_configure_command)
{
  const std::vector<uint8_t> expected_request{
    0x00,  // request ID
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
    0x00,  // Request Id
    0x11   // Status
  };

  std::vector<uint8_t> actual_request;
  auto mock_data_interface = std::make_unique<MockDataInterface>();
  EXPECT_CALL(*mock_data_interface, write(_)).WillOnce(SaveArg<0>(&actual_request));
  EXPECT_CALL(*mock_data_interface, read()).WillOnce(Return(mocked_response));

  auto command_handler = std::make_unique<stepit_control::CommandHandler>(std::move(mock_data_interface));
  MotorConfigCommand request{ 0,
                              { MotorConfigCommand::Param{ 0, 0.5, 0.75 }, MotorConfigCommand::Param{ 1, 0.5, 0.75 } } };
  AcknowledgeResponse response = command_handler->send(request);

  ASSERT_THAT(stepit_control::data_utils::to_hex(actual_request), stepit_control::data_utils::to_hex(expected_request));
  ASSERT_EQ(0, response.request_id());
  ASSERT_EQ(Response::Status::Success, response.status());
}

}  // namespace stepit_control::test
