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
#include <mock_serial_interface.h>

using ::testing::_;
using ::testing::Action;
using ::testing::Matcher;
using ::testing::Return;
using ::testing::SaveArg;
using ::testing::SetArgPointee;
using ::testing::SetArgReferee;

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
 * This is a custom action that uses an iterator to take a value from a vector
 * and gives the value to a method output parameter.
 */
ACTION_TEMPLATE(SetArgFromVector, HAS_1_TEMPLATE_PARAMS(unsigned, param_index), AND_1_VALUE_PARAMS(p_iterator))
{
  // param_index indicates the position of the output parameter and
  // p_interator is a pointer to a vector iterator.
  *std::get<param_index>(args) = **p_iterator;
  ++(*p_iterator);
}

/**
 * Read a response frame and expect data to be extracted.
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
