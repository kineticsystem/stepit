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
#include <gmock/gmock.h>

#include <stepit_hardware/msgs/msgs.hpp>
#include <stepit_hardware/stepit_hardware.hpp>

#include <stepit_hardware_info.hpp>
#include <test_utils.hpp>

#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <hardware_interface/resource_manager.hpp>
#include <hardware_interface/types/lifecycle_state_names.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <ros2_control_test_assets/components_urdfs.hpp>
#include <ros2_control_test_assets/descriptions.hpp>

#include <data_interface/data_utils.hpp>
#include <data_interface/crc_utils.hpp>
#include <data_interface/serial_handler.hpp>
#include <data_interface/data_handler.hpp>

#include <thread>
#include <chrono>

namespace stepit_hardware::test
{

class TestDataInterface : public ::testing::Test
{
public:
  void SetUp()
  {
    if (skip_test())
    {
      GTEST_SKIP() << "Skipping all tests for this fixture";
    }
    auto serial_handler = std::make_unique<data_interface::SerialHandler>();
    serial_handler->set_port("/dev/ttyACM0");
    serial_handler->set_baudrate(9600);
    serial_handler->set_timeout(2000);

    data_handler = std::make_unique<data_interface::DataHandler>(std::move(serial_handler));
    data_handler->open();
  }

  void TearDown()
  {
    if (data_handler)
    {
      data_handler->close();
    }
  }

  std::unique_ptr<data_interface::DataHandler> data_handler = nullptr;
};

/**
 * Send a packet of data corresponding to a velocity command and check
 * if the microcontroller answers back with a successful response.
 */
TEST_F(TestDataInterface, test_velocity_command)
{
  const std::vector<uint8_t> data{
    0x77,  // Motor velocity command ID
    0x00,  // Motor ID
    0x40,  // Velocity = 4
    0x80,  // Velocity
    0x00,  // Velocity
    0x00,  // Velocity
    0x01,  // Motor ID
    0x40,  // Velocity = 3
    0x40,  // Velocity
    0x00,  // Velocity
    0x00   // Velocity
  };
  const std::vector<uint8_t> expected_response{ 0x11 };
  data_handler->write(data);
  const std::vector<uint8_t> response = data_handler->read();
  ASSERT_THAT(data_interface::to_hex(response), data_interface::to_hex(expected_response));
}

/**
 * Send a sequence of echo commands and check if they are bounced back by the
 * micro controller.
 * The test checks that everything works at the communication layer, that the
 * micro controller is running, listening and answering to incoming commands.
 * The test is designed to generate data buffers that contains escaped bytes,
 * including the CRC.
 */
TEST_F(TestDataInterface, test_echo_command)
{
  uint16_t crc = 0;
  for (uint8_t i = 0; i <= 100; ++i)
  {
    const std::vector<uint8_t> request{
      0x79,  // Echo command.
      0x40,  // Some random data.
      0x7E,  // A delimiter flag which must be escaped.
      0x03,  // Some random data.
      0x7D,  // An escape flag which must be escaped.
      0x01,  // Some random data.
      0x20,  // An escape XOR, just in case.
      i      // This value will cause some CRC to contains values which must be escaped.
    };

    data_handler->write(request);
    const std::vector<uint8_t> response = data_handler->read();
    ASSERT_THAT(data_interface::to_hex(response), data_interface::to_hex(request));
  }
}

/**
 * Send an info command to the micro controller and verify the expected response.
 */
TEST_F(TestDataInterface, test_info_command)
{
  const std::vector<uint8_t> request{
    0x76  // Info query.
  };
  const std::vector<uint8_t> expected_response{
    0x11,  // Success status.
    0x53,  // S.
    0x54,  // T.
    0x45,  // E.
    0x50,  // P.
    0x49,  // I.
    0x54   // T.
  };

  data_handler->write(request);
  const std::vector<uint8_t> response = data_handler->read();
  ASSERT_THAT(data_interface::to_hex(expected_response), data_interface::to_hex(response));
}

}  // namespace stepit_hardware::test
