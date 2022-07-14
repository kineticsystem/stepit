/*
 * Copyright (C) 2022 Remigi Giovanni
 * g.remigi@kineticsystem.org
 * www.kineticsystem.org
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU Lesser General Public License as published by the Free
 * Software Foundation; either version 2 of the License, or (at your option) any
 * later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License for more
 * details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program; if not, write to the Free Software Foundation, Inc.,
 * 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <gmock/gmock.h>
#include <stepit_hardware/data_interface.h>
#include <stepit_hardware/data_utils.h>
#include <stepit_hardware/serial_exception.h>
#include <mock_serial_interface.h>
#include <custom_actions.h>

using ::testing::_;
using ::testing::Matcher;
using ::testing::Return;
using ::testing::SaveArg;

/**
 * Write some data and expect a request frame to be created including
 * delimiters, request ID and CRC.
 */
TEST(command_interface, write)
{
  MockSerialInterface mock;
  stepit_hardware::DataInterface data_interface{ &mock };

  std::vector<uint8_t> data{
    0x71,  // Motor Move To command ID
    0x01,  // Motor ID
    0x00,  // Position (MSB) = 3000
    0x00,  // Position
    0x0B,  // Position
    0xB8   // Position (LSB)
  };

  std::vector<uint8_t> expected_frame{
    0x7E,  // Delimiter
    0x00,  // Request ID
    0x71,  // Motor Move To command ID
    0x01,  // Motor ID
    0x00,  // Position (MSB)
    0x00,  // Position
    0x0B,  // Position
    0xB8,  // Position (LSB)
    0x05,  // CRC (MSB)
    0x17,  // CRC (LSB)
    0x7E   // Delimiter
  };
  std::vector<uint8_t> actual_frame;

  EXPECT_CALL(mock, write(_)).WillOnce(SaveArg<0>(&actual_frame));

  data_interface.write(data);
  ASSERT_THAT(stepit_hardware::data_utils::to_hex(actual_frame), stepit_hardware::data_utils::to_hex(expected_frame));
}

/**
 * Write some data and expect a request frame, with escaped bytes, to be
 * created including delimiters, request ID and CRC.
 */
TEST(command_interface, write_escaped)
{
  MockSerialInterface mock;
  stepit_hardware::DataInterface cmd_interface{ &mock };

  std::vector<uint8_t> data{
    0x71,  // Motor Move To command ID
    0x01,  // Motor ID
    0x00,  // Position (MSB) = 32256
    0x00,  // Position
    0x7E,  // Position
    0x00   // Position (LSB)
  };

  std::vector<uint8_t> expected_frame = {
    0x7E,  // Delimiter
    0x00,  // Request ID
    0x71,  // Motor Move To command ID
    0x01,  // Motor ID
    0x00,  // Position escaped (MSB)
    0x00,  // Position escaped
    0x7D,  // Position escaped
    0x5E,  // Position escaped
    0x00,  // Position escaped (LSB)
    0xBA,  // CRC (MSB)
    0xA0,  // CRC (LSB)
    0x7E   // Delimiter
  };
  std::vector<uint8_t> actual_frame;

  EXPECT_CALL(mock, write(_)).WillOnce(SaveArg<0>(&actual_frame));

  cmd_interface.write(data);
  ASSERT_THAT(stepit_hardware::data_utils::to_hex(actual_frame), stepit_hardware::data_utils::to_hex(expected_frame));
}

/**
 * Read a response frame and expect data to be returned.
 */
TEST(command_interface, read)
{
  MockSerialInterface mock;
  stepit_hardware::DataInterface cmd_interface{ &mock };

  std::vector<uint8_t> frame = {
    0x7E,  // Delimiter
    0x00,  // Response ID
    0x71,  // Status command ID
    0x01,  // Motor ID
    0x00,  // Position (MSB)
    0x00,  // Position
    0x0B,  // Position
    0xB8,  // Position (LSB)
    0x05,  // CRC (MSB)
    0x17,  // CRC (LSB)
    0x7E   // Delimiter
  };

  std::vector<uint8_t> expected_data{
    0x71,  // Status command ID
    0x01,  // Motor ID
    0x00,  // Position (MSB) = 3000
    0x00,  // Position
    0x0B,  // Position
    0xB8   // Position (LSB)
  };

  auto it = std::begin(frame);
  EXPECT_CALL(mock, read(Matcher<uint8_t*>(_), _)).WillRepeatedly(SetArgFromVector<0>(&it));

  std::vector<uint8_t> actual_data = cmd_interface.read();
  ASSERT_THAT(stepit_hardware::data_utils::to_hex(actual_data), stepit_hardware::data_utils::to_hex(expected_data));
}

/**
 * Read a response frame with escaped bytes and expect data to be returned.
 */
TEST(command_interface, read_escaped)
{
  MockSerialInterface mock;
  stepit_hardware::DataInterface cmd_interface{ &mock };

  std::vector<uint8_t> frame = {
    0x7E,  // Delimiter
    0x00,  // Request ID
    0x71,  // Motor Move To command ID
    0x01,  // Motor ID
    0x00,  // Position escaped (MSB)
    0x00,  // Position escaped
    0x7D,  // Position escaped
    0x5E,  // Position escaped
    0x00,  // Position escaped (LSB)
    0xBA,  // CRC (MSB)
    0xA0,  // CRC (LSB)
    0x7E   // Delimiter
  };

  std::vector<uint8_t> expected_data{
    0x71,  // Status command ID
    0x01,  // Motor ID
    0x00,  // Position (MSB) = 32256
    0x00,  // Position
    0x7E,  // Position
    0x00   // Position (LSB)
  };

  auto it = std::begin(frame);
  EXPECT_CALL(mock, read(Matcher<uint8_t*>(_), _)).WillRepeatedly(SetArgFromVector<0>(&it));

  std::vector<uint8_t> actual_data = cmd_interface.read();
  ASSERT_THAT(stepit_hardware::data_utils::to_hex(actual_data), stepit_hardware::data_utils::to_hex(expected_data));
}

/**
 * Test that an exception is thrown when a CRC error is found.
 */
TEST(command_interface, read_crc_error)
{
  MockSerialInterface mock;
  stepit_hardware::DataInterface cmd_interface{ &mock };

  std::vector<uint8_t> frame = {
    0x7E,  // Delimiter
    0x00,  // Request ID
    0x71,  // Motor Move To command ID
    0x01,  // Motor ID
    0x00,  // Position escaped (MSB)
    0x00,  // Position escaped
    0x7D,  // Position escaped
    0x5E,  // Position escaped
    0x00,  // Position escaped (LSB)
    0xB1,  // CRC (MSB) - incorrect
    0xA0,  // CRC (LSB)
    0x7E   // Delimiter
  };

  auto it = std::begin(frame);
  EXPECT_CALL(mock, read(Matcher<uint8_t*>(_), _)).WillRepeatedly(SetArgFromVector<0>(&it));

  EXPECT_THROW(
      {
        try
        {
          std::vector<uint8_t> actual_data = cmd_interface.read();
        }
        catch (const stepit_hardware::SerialException& e)
        {
          EXPECT_STREQ("SerialException: CRC error.", e.what());
          throw;
        }
      },
      stepit_hardware::SerialException);
}

/**
 * Test that an exception is thrown when the packet is too short.
 */
TEST(command_interface, read_incorrect_frame_length)
{
  MockSerialInterface mock;
  stepit_hardware::DataInterface cmd_interface{ &mock };

  std::vector<uint8_t> frame = {
    0x7E,  // Delimiter
    0x00,  // Request ID
    0x7E   // Delimiter
  };

  auto it = std::begin(frame);
  EXPECT_CALL(mock, read(Matcher<uint8_t*>(_), _)).WillRepeatedly(SetArgFromVector<0>(&it));

  EXPECT_THROW(
      {
        try
        {
          std::vector<uint8_t> actual_data = cmd_interface.read();
        }
        catch (const stepit_hardware::SerialException& e)
        {
          EXPECT_STREQ("SerialException: incorrect frame length.", e.what());
          throw;
        }
      },
      stepit_hardware::SerialException);
}

/**
 * Test that an exception is thrown when there is no start delimiter.
 */
TEST(command_interface, read_start_delimiter_missing)
{
  MockSerialInterface mock;
  stepit_hardware::DataInterface cmd_interface{ &mock };

  std::vector<uint8_t> frame = {
    // Missing delimiter
    0x00,  // Request ID
    0x71,  // Motor Move To command ID
    0x01,  // Motor ID
    0x00,  // Position escaped (MSB)
    0x00,  // Position escaped
    0x7D,  // Position escaped
    0x5E,  // Position escaped
    0x00,  // Position escaped (LSB)
    0xBA,  // CRC (MSB)
    0xA0,  // CRC (LSB)
    0x7E   // Delimiter
  };

  auto it = std::begin(frame);
  EXPECT_CALL(mock, read(Matcher<uint8_t*>(_), _)).WillRepeatedly(SetArgFromVector<0>(&it));

  EXPECT_THROW(
      {
        try
        {
          std::vector<uint8_t> actual_data = cmd_interface.read();
        }
        catch (const stepit_hardware::SerialException& e)
        {
          EXPECT_STREQ("SerialException: start delimiter missing.", e.what());
          throw;
        }
      },
      stepit_hardware::SerialException);
}

/**
 * Test that an exception is thrown when no data is read.
 */
TEST(command_interface, read_timeout)
{
  MockSerialInterface mock;
  stepit_hardware::DataInterface cmd_interface{ &mock };

  // We mock the read to simulate a timeout by returning 0 bytes.
  EXPECT_CALL(mock, read(Matcher<uint8_t*>(_), _)).WillOnce(Return(0));

  EXPECT_THROW(
      {
        try
        {
          std::vector<uint8_t> actual_data = cmd_interface.read();
        }
        catch (const stepit_hardware::SerialException& e)
        {
          EXPECT_STREQ("SerialException: timeout.", e.what());
          throw;
        }
      },
      stepit_hardware::SerialException);
}
