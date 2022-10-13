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

#include <stepit_hardware/data_utils.hpp>
#include <stepit_hardware/crc_utils.hpp>
#include <stepit_hardware/msgs/msgs.hpp>
#include <stepit_hardware/stepit_hardware.hpp>

#include <stepit_hardware_info.hpp>

#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <hardware_interface/resource_manager.hpp>
#include <hardware_interface/types/lifecycle_state_names.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <ros2_control_test_assets/components_urdfs.hpp>
#include <ros2_control_test_assets/descriptions.hpp>

#include <stepit_hardware/serial_handler.hpp>
#include <stepit_hardware/data_handler.hpp>

namespace stepit_hardware::test
{

/**
 * This test requires connection to a real hardware.
 */
TEST(TestStepitHardware, test_connection)
{
  auto stepit_hardware = std::make_unique<stepit_hardware::StepitHardware>();

  StepitHardwareInfo info;
  info.hardware_parameters["use_dummy"] = false;
  info.hardware_parameters["timeout"] = "2.0";  // Seconds.

  // Load the component.
  hardware_interface::ResourceManager rm;
  rm.import_component(std::move(stepit_hardware), info);

  // Connect the hardware.
  rclcpp_lifecycle::State state{ lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
                                 hardware_interface::lifecycle_state_names::ACTIVE };
  rm.set_component_state("StepitHardware", state);

  // Invoke a read command.
  rm.read(rclcpp::Time{}, rclcpp::Duration::from_seconds(0));

  // Read state interface values.
  hardware_interface::LoanedStateInterface joint1_position_state = rm.claim_state_interface("joint1/position");
  hardware_interface::LoanedStateInterface joint2_position_state = rm.claim_state_interface("joint2/position");
  hardware_interface::LoanedStateInterface joint1_velocity_state = rm.claim_state_interface("joint1/velocity");
  hardware_interface::LoanedStateInterface joint2_velocity_state = rm.claim_state_interface("joint2/velocity");

  ASSERT_EQ(0, joint1_position_state.get_value());
  ASSERT_EQ(0, joint2_position_state.get_value());
  ASSERT_EQ(0, joint1_velocity_state.get_value());
  ASSERT_EQ(0, joint2_velocity_state.get_value());

  // Write velocity values.
  // Arduino uses its max internal speed if a too high velocity value is given.
  hardware_interface::LoanedCommandInterface joint1_velocity_command = rm.claim_command_interface("joint1/velocity");
  hardware_interface::LoanedCommandInterface joint2_velocity_command = rm.claim_command_interface("joint2/velocity");
  joint1_velocity_command.set_value(4);
  joint2_velocity_command.set_value(1);

  for (int i = 0; i < 100000; ++i)
  {
    // Invoke a write command.
    rm.write(rclcpp::Time{}, rclcpp::Duration::from_seconds(0));
    rm.read(rclcpp::Time{}, rclcpp::Duration::from_seconds(0));
    RCLCPP_INFO(rclcpp::get_logger("TestStepitHardware"), "%d, p1:%f, v1:%f, p2:%f, v2:%f", i,
                joint1_position_state.get_value(), joint1_velocity_state.get_value(), joint2_position_state.get_value(),
                joint2_velocity_state.get_value());
  }
}

/**
 * Send a packet of data corresponding to a velocity command and check
 * if the microcontroller answers back with a successful response.
 */
TEST(TestStepitHardware, test_velocity_command)
{
  auto serial_handler = std::make_unique<SerialHandler>();
  serial_handler->set_port("/dev/ttyUSB0");
  serial_handler->set_baudrate(9600);
  serial_handler->set_timeout(2000);

  auto data_handler = std::make_unique<DataHandler>(std::move(serial_handler));
  data_handler->open();
  std::vector<uint8_t> out = data_handler->read();

  const std::vector<uint8_t> data{
    0x01,  // Request ID
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
  const std::vector<uint8_t> expected_response{ 0x01, 0x11 };
  data_handler->write(data);
  const std::vector<uint8_t> response = data_handler->read();
  ASSERT_THAT(stepit_hardware::data_utils::to_hex(response), stepit_hardware::data_utils::to_hex(expected_response));
}

/**
 * Send a sequence of echo commands and check if they are bounced back by the
 * micro controller.
 * The test check if everything is working at the communication layer, if the
 * micro controller is running, listening and answering to incoming commands.
 * The test is designed to generate data buffers that contains bytes which must
 * be escaped, including the CRC.
 */
TEST(TestStepitHardware, test_echo)
{
  auto serial_handler = std::make_unique<SerialHandler>();
  serial_handler->set_port("/dev/ttyUSB0");
  serial_handler->set_baudrate(9600);
  serial_handler->set_timeout(2000);

  auto data_handler = std::make_unique<DataHandler>(std::move(serial_handler));
  data_handler->open();
  std::vector<uint8_t> out = data_handler->read();

  uint16_t crc = 0;
  for (uint8_t i = 0; i <= 100; ++i)
  {
    const std::vector<uint8_t> data{
      0x01,  // Request ID
      0x79,  // Echo command.
      0x40,  // Some random data.
      0x7E,  // A delimiter flag which must be escaped.
      0x03,  // Some random data.
      0x7D,  // An escape flag which must be escaped.
      0x01,  // Some random data.
      0x20,  // An escape XOR, just in case.
      i      // This value will cause some CRC to contains values which must be escaped.
    };

    data_handler->write(data);
    const std::vector<uint8_t> echo = data_handler->read();
    ASSERT_THAT(stepit_hardware::data_utils::to_hex(data), stepit_hardware::data_utils::to_hex(echo));
  }
}

}  // namespace stepit_hardware::test
