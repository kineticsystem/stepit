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

#include <gtest/gtest.h>

#include <stdexcept>

#include <mock/mock_cobs_serial.hpp>
#include <stepit_driver/default_driver.hpp>

#include <cobs_serial/data_utils.hpp>

namespace stepit_driver::test
{
using ::testing::_;
using ::testing::Return;
using ::testing::SaveArg;
using ::testing::Throw;

using cobs_serial::data_utils::to_hex;

/**
 * Build the response the controller sends to an info query: a status byte,
 * the firmware version, the motor count, the limits of each motor and, last,
 * the controller name.
 */
std::vector<uint8_t> info_response(const std::string& name, uint8_t version_major = 1, uint8_t motor_count = 5,
                                   float max_acceleration = 12.5663706f, float max_velocity = 18.8495559f)
{
  std::vector<uint8_t> out{ 0x11, version_major, 0x00, 0x00, motor_count };
  for (uint8_t id = 0; id < motor_count; id++)
  {
    out.push_back(id);
    for (uint8_t byte : cobs_serial::data_utils::from_float(max_acceleration))
    {
      out.push_back(byte);
    }
    for (uint8_t byte : cobs_serial::data_utils::from_float(max_velocity))
    {
      out.push_back(byte);
    }
  }
  out.insert(out.end(), name.begin(), name.end());
  return out;
}

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

/**
 * In this test we connect to a device that identifies itself as a StepIt
 * controller and we check that the connection succeeds.
 */
TEST(TestDefaultDriver, connect_to_stepit_controller)
{
  const std::vector<uint8_t> expected_request{
    0x76,  // info query ID
  };

  const std::vector<uint8_t> mocked_response = info_response("STEPIT");

  std::vector<uint8_t> actual_request;
  auto serial = std::make_unique<MockCobsSerial>();
  EXPECT_CALL(*serial, open());
  EXPECT_CALL(*serial, write(_)).WillOnce(SaveArg<0>(&actual_request));
  EXPECT_CALL(*serial, read()).WillOnce(Return(mocked_response));

  auto driver = std::make_unique<stepit_driver::DefaultDriver>(std::move(serial));

  ASSERT_TRUE(driver->connect());
  ASSERT_THAT(to_hex(actual_request), to_hex(expected_request));
}

/**
 * In this test we connect to a device that answers the info query but does
 * not identify itself as a StepIt controller: the connection must be refused
 * without further retries.
 */
TEST(TestDefaultDriver, connect_to_unknown_device)
{
  const std::vector<uint8_t> mocked_response = info_response("OTHER");

  auto serial = std::make_unique<MockCobsSerial>();
  EXPECT_CALL(*serial, open());
  EXPECT_CALL(*serial, write(_)).Times(1);
  EXPECT_CALL(*serial, read()).WillOnce(Return(mocked_response));

  auto driver = std::make_unique<stepit_driver::DefaultDriver>(std::move(serial));

  ASSERT_FALSE(driver->connect());
}

/**
 * In this test the device never answers: the driver must retry a bounded
 * number of times and then give up.
 */
TEST(TestDefaultDriver, connect_to_unresponsive_device)
{
  auto serial = std::make_unique<MockCobsSerial>();
  EXPECT_CALL(*serial, open());
  EXPECT_CALL(*serial, write(_)).Times(5);
  EXPECT_CALL(*serial, read()).Times(5).WillRepeatedly(Throw(std::runtime_error("timeout")));

  auto driver = std::make_unique<stepit_driver::DefaultDriver>(std::move(serial));

  ASSERT_FALSE(driver->connect());
}

/**
 * In this test we check that the limits reported by the controller in its
 * info response are parsed, alongside the name.
 */
TEST(TestDefaultDriver, parse_limits_in_info_response)
{
  const std::vector<uint8_t> mocked_response = info_response("STEPIT", 1, 5, 12.5663706f, 18.8495559f);

  auto serial = std::make_unique<MockCobsSerial>();
  EXPECT_CALL(*serial, write(_)).Times(1);
  EXPECT_CALL(*serial, read()).WillOnce(Return(mocked_response));

  auto driver = std::make_unique<stepit_driver::DefaultDriver>(std::move(serial));

  const InfoResponse response = driver->get_info(rclcpp::Time{});

  ASSERT_EQ(Response::Status::Success, response.status());
  ASSERT_EQ("STEPIT", response.info());
  ASSERT_EQ("1.0.0", response.version().to_string());
  ASSERT_EQ(static_cast<std::size_t>(5), response.limits().size());
  for (uint8_t id = 0; id < 5; id++)
  {
    ASSERT_EQ(id, response.limits()[id].id());
    ASSERT_NEAR(12.5663706, response.limits()[id].max_acceleration(), 1e-6);
    ASSERT_NEAR(18.8495559, response.limits()[id].max_velocity(), 1e-6);
  }
}

/**
 * In this test the controller answers an info query with a truncated packet.
 * The driver must not read past the end of it.
 */
TEST(TestDefaultDriver, connect_to_device_sending_a_truncated_info_response)
{
  // Announces five motors but carries the limits of none of them.
  const std::vector<uint8_t> mocked_response{ 0x11, 0x01, 0x00, 0x00, 0x05 };

  auto serial = std::make_unique<MockCobsSerial>();
  EXPECT_CALL(*serial, open());
  EXPECT_CALL(*serial, write(_)).Times(5);
  EXPECT_CALL(*serial, read()).Times(5).WillRepeatedly(Return(mocked_response));

  auto driver = std::make_unique<stepit_driver::DefaultDriver>(std::move(serial));

  ASSERT_FALSE(driver->connect());
}

/**
 * In this test the controller identifies itself correctly but reports a
 * firmware whose protocol version this driver does not speak. Its packets
 * cannot be trusted, so the connection is refused without further retries.
 */
TEST(TestDefaultDriver, connect_to_incompatible_firmware)
{
  const std::vector<uint8_t> mocked_response = info_response("STEPIT", 2);

  auto serial = std::make_unique<MockCobsSerial>();
  EXPECT_CALL(*serial, open());
  EXPECT_CALL(*serial, write(_)).Times(1);
  EXPECT_CALL(*serial, read()).WillOnce(Return(mocked_response));

  auto driver = std::make_unique<stepit_driver::DefaultDriver>(std::move(serial));

  ASSERT_FALSE(driver->connect());
}

}  // namespace stepit_driver::test
