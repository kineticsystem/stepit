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

#include <gmock/gmock.h>
#include <cobs_serial/default_cobs_serial.hpp>
#include <cobs_serial/data_utils.hpp>
#include <cobs_serial/serial_exception.hpp>
#include <mock/mock_serial.hpp>

namespace cobs_serial::test
{

using ::testing::_;
using ::testing::Invoke;
using ::testing::Return;

/**
 * Write some data and expect a request frame to be created including
 * delimiters, request ID and CRC.
 */
TEST(TestDefaultCobsSerial, write)
{
  const std::vector<uint8_t> data{
    0x00,  // Request ID
    0x71,  // Motor Move To command ID
    0x01,
    0x00,  // Position (MSB) = 3000
    0x00,  // Position
    0x0B,  // Position
    0xB8   // Position (LSB)
  };

  const std::vector<uint8_t> expected_frame{
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

  auto serial = std::make_unique<MockSerial>();

  // Use a lambda function to populate a vector with the buffer values.
  std::vector<uint8_t> actual_frame;
  auto write = [&actual_frame](const uint8_t* buffer, [[maybe_unused]] std::size_t size) -> std::size_t {
    actual_frame.emplace_back(buffer[0]);
    return 1;
  };
  EXPECT_CALL(*serial, write(_, _)).WillRepeatedly(Invoke(write));

  cobs_serial::DefaultCobsSerial data_interface{ std::move(serial) };
  data_interface.write(data);
  ASSERT_THAT(data_utils::to_hex(actual_frame), data_utils::to_hex(expected_frame));
}

/**
 * Write some data and expect a request frame, with escaped bytes, to be
 * created including delimiters, request ID and CRC.
 */
TEST(TestDefaultCobsSerial, write_escaped_data)
{
  const std::vector<uint8_t> data{
    0x00,  // Request ID
    0x71,  // Motor Move To command ID
    0x01,  // Motor ID
    0x00,  // Position (MSB) = 32256
    0x00,  // Position
    0x7E,  // Position
    0x00   // Position (LSB)
  };

  const std::vector<uint8_t> expected_frame = {
    0x7E,  // Delimiter
    0x00,  // Request ID
    0x71,  // Motor Move To command ID
    0x01,  // Motor ID
    0x00,  // Position (MSB)
    0x00,  // Position
    0x7D,  // Escape flag
    0x5E,  // Position escaped
    0x00,  // Position (LSB)
    0xBA,  // CRC (MSB)
    0xA0,  // CRC (LSB)
    0x7E   // Delimiter
  };

  auto serial = std::make_unique<MockSerial>();

  // Use a lambda function to populate a vector with the buffer values.
  std::vector<uint8_t> actual_frame;
  auto write = [&actual_frame](const uint8_t* buffer, [[maybe_unused]] std::size_t size) -> std::size_t {
    actual_frame.emplace_back(buffer[0]);
    return 1;
  };
  EXPECT_CALL(*serial, write(_, _)).WillRepeatedly(Invoke(write));

  cobs_serial::DefaultCobsSerial data_interface{ std::move(serial) };
  data_interface.write(data);
  ASSERT_THAT(data_utils::to_hex(actual_frame), data_utils::to_hex(expected_frame));
}

/**
 * Write some data and expect a request frame to be created including
 * delimiters, request ID and escaped CRC.
 */
TEST(TestDefaultCobsSerial, write_escaped_crc)
{
  // This specific data will generate a CRC value containing the 0x7D value
  // that must be escaped.
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

  const std::vector<uint8_t> expected_frame = {
    0x7E,  // Delimiter
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
    0x00,  // Velocity
    0x7D,  // Escape command.
    0x5D,  // XOR CRC (MSB)
    0x4D,  // CRC (LSB)
    0x7E   // Delimiter
  };

  auto serial = std::make_unique<MockSerial>();

  // Use a lambda function to populate a vector with the buffer values.
  std::vector<uint8_t> actual_frame;
  auto write = [&actual_frame](const uint8_t* buffer, [[maybe_unused]] std::size_t size) -> std::size_t {
    actual_frame.emplace_back(buffer[0]);
    return 1;
  };
  EXPECT_CALL(*serial, write(_, _)).WillRepeatedly(Invoke(write));

  cobs_serial::DefaultCobsSerial data_interface{ std::move(serial) };
  data_interface.write(data);
  ASSERT_THAT(data_utils::to_hex(actual_frame), data_utils::to_hex(expected_frame));
}

/**
 * Test that an exception is thrown is no data are written.
 */
TEST(TestDefaultCobsSerial, write_error)
{
  auto serial = std::make_unique<MockSerial>();

  // We mock the read to simulate a timeout by returning 0 bytes.
  EXPECT_CALL(*serial, write(_, _)).WillOnce(Return(0));

  cobs_serial::DefaultCobsSerial data_interface{ std::move(serial) };
  EXPECT_THROW(
      {
        try
        {
          data_interface.write({ 0, 0 });
        }
        catch (const cobs_serial::SerialException& e)
        {
          EXPECT_STREQ("SerialException: error writing to the serial port.", e.what());
          throw;
        }
      },
      cobs_serial::SerialException);
}

/**
 * Read a response frame and expect data to be returned.
 */
TEST(TestDefaultCobsSerial, read)
{
  const std::vector<uint8_t> frame = {
    0x7E,  // Delimiter
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

  const std::vector<uint8_t> expected_data{
    0x71,  // Status command ID
    0x01,  // Motor ID
    0x00,  // Position (MSB) = 3000
    0x00,  // Position
    0x0B,  // Position
    0xB8   // Position (LSB)
  };

  auto serial = std::make_unique<MockSerial>();

  // Use a lambda function to populate an input buffer with vector values.
  auto it = std::begin(frame);
  auto read = [&](uint8_t* buffer, [[maybe_unused]] std::size_t size) -> std::size_t {
    buffer[0] = *it++;
    return 1;
  };
  EXPECT_CALL(*serial, read(_, _)).WillRepeatedly(Invoke(read));

  cobs_serial::DefaultCobsSerial data_interface{ std::move(serial) };
  std::vector<uint8_t> actual_data = data_interface.read();
  ASSERT_THAT(data_utils::to_hex(actual_data), data_utils::to_hex(expected_data));
}

/**
 * Read a response frame with escaped bytes and expect data to be returned.
 */
TEST(TestDefaultCobsSerial, read_escaped)
{
  const std::vector<uint8_t> frame = {
    0x7E,  // Delimiter
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

  const std::vector<uint8_t> expected_data{
    0x71,  // Status command ID
    0x01,  // Motor ID
    0x00,  // Position (MSB) = 32256
    0x00,  // Position
    0x7E,  // Position
    0x00   // Position (LSB)
  };

  auto serial = std::make_unique<MockSerial>();

  // Use a lambda function to populate an input buffer with vector values.
  auto it = std::begin(frame);
  auto read = [&](uint8_t* buffer, [[maybe_unused]] std::size_t size) -> std::size_t {
    buffer[0] = *it++;
    return 1;
  };
  EXPECT_CALL(*serial, read(_, _)).WillRepeatedly(Invoke(read));

  cobs_serial::DefaultCobsSerial data_interface{ std::move(serial) };
  std::vector<uint8_t> actual_data = data_interface.read();
  ASSERT_THAT(data_utils::to_hex(actual_data), data_utils::to_hex(expected_data));
}

/**
 * Test that an exception is thrown when a CRC error is found.
 */
TEST(TestDefaultCobsSerial, read_crc_error)
{
  const std::vector<uint8_t> frame = {
    0x7E,  // Delimiter
    0x71,  // Motor Move To command ID
    0x01,  // Motor ID
    0x00,  // Position (MSB)
    0x00,  // Position
    0x7D,  // Escape flaf
    0x5E,  // Position escaped
    0x00,  // Position (LSB)
    0xB1,  // CRC (MSB) - incorrect
    0xA0,  // CRC (LSB)
    0x7E   // Delimiter
  };

  auto serial = std::make_unique<MockSerial>();

  // Use a lambda function to populate an input buffer with vector values.
  auto it = std::begin(frame);
  auto read = [&](uint8_t* buffer, [[maybe_unused]] std::size_t size) -> std::size_t {
    buffer[0] = *it++;
    return 1;
  };
  EXPECT_CALL(*serial, read(_, _)).WillRepeatedly(Invoke(read));

  cobs_serial::DefaultCobsSerial data_interface{ std::move(serial) };
  EXPECT_THROW(
      {
        try
        {
          std::vector<uint8_t> actual_data = data_interface.read();
        }
        catch (const cobs_serial::SerialException& e)
        {
          EXPECT_STREQ("SerialException: CRC error.", e.what());
          throw;
        }
      },
      cobs_serial::SerialException);
}

/**
 * Test that an exception is thrown when the packet is too short.
 * We always expect:
 * 1) a delimiter;
 * 2) at least one byte of data;
 * 3) crc;
 * 4) crc
 * 5) a delimiter.
 */
TEST(TestDefaultCobsSerial, read_incorrect_frame_length)
{
  const std::vector<uint8_t> frame = {
    0x7E,  // Delimiter
    0x00,  // Request ID
    0x7E   // Delimiter
  };

  auto serial = std::make_unique<MockSerial>();

  // Use a lambda function to populate an input buffer with vector values.
  auto it = std::begin(frame);
  auto read = [&](uint8_t* buffer, [[maybe_unused]] std::size_t size) -> std::size_t {
    buffer[0] = *it++;
    return 1;
  };
  EXPECT_CALL(*serial, read(_, _)).WillRepeatedly(Invoke(read));

  cobs_serial::DefaultCobsSerial data_interface{ std::move(serial) };
  EXPECT_THROW(
      {
        try
        {
          std::vector<uint8_t> actual_data = data_interface.read();
        }
        catch (const cobs_serial::SerialException& e)
        {
          EXPECT_STREQ("SerialException: incorrect frame length.", e.what());
          throw;
        }
      },
      cobs_serial::SerialException);
}

/**
 * Test that an exception is thrown when there is no start delimiter.
 */
TEST(TestDefaultCobsSerial, read_start_delimiter_missing)
{
  const std::vector<uint8_t> frame = {
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

  auto serial = std::make_unique<MockSerial>();

  // Use a lambda function to populate an input buffer with vector values.
  auto it = std::begin(frame);
  auto read = [&](uint8_t* buffer, [[maybe_unused]] std::size_t size) -> std::size_t {
    buffer[0] = *it++;
    return 1;
  };
  EXPECT_CALL(*serial, read(_, _)).WillRepeatedly(Invoke(read));

  cobs_serial::DefaultCobsSerial data_interface{ std::move(serial) };
  EXPECT_THROW(
      {
        try
        {
          std::vector<uint8_t> actual_data = data_interface.read();
        }
        catch (const cobs_serial::SerialException& e)
        {
          EXPECT_STREQ("SerialException: start delimiter missing.", e.what());
          throw;
        }
      },
      cobs_serial::SerialException);
}

/**
 * Test that an exception is thrown when no data is read.
 */
TEST(TestDefaultCobsSerial, read_timeout)
{
  auto serial = std::make_unique<MockSerial>();

  // We mock the read to simulate a timeout by returning 0 bytes.
  EXPECT_CALL(*serial, read(_, _)).WillOnce(Return(0));

  cobs_serial::DefaultCobsSerial data_interface{ std::move(serial) };
  EXPECT_THROW(
      {
        try
        {
          std::vector<uint8_t> actual_data = data_interface.read();
        }
        catch (const cobs_serial::SerialException& e)
        {
          EXPECT_STREQ("SerialException: timeout.", e.what());
          throw;
        }
      },
      cobs_serial::SerialException);
}
}  // namespace cobs_serial::test
