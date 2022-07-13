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

#include <stepit_hardware/command_interface.h>

#include <stepit_hardware/crc_utils.h>
#include <stepit_hardware/data_utils.h>

#include <cstdint>

namespace stepit_hardware
{
constexpr uint8_t DELIMITER_FLAG = 0x7E;  // Start and end of a packet
constexpr uint8_t ESCAPE_FLAG = 0x7D;     // Escaping byte.
constexpr uint8_t ESCAPED_XOR = 0x20;     // XOR value applied to escaped bytes.

CommandInterface::CommandInterface(SerialInterface* serial) : serial_{ serial }
{
}

const std::vector<uint8_t> CommandInterface::escape(uint8_t byte)
{
  std::vector<uint8_t> bytes;
  if (byte == DELIMITER_FLAG || byte == ESCAPE_FLAG)
  {
    bytes.emplace_back(ESCAPE_FLAG);
    bytes.emplace_back(byte ^ ESCAPED_XOR);
  }
  else
  {
    bytes.emplace_back(byte);
  }
  return bytes;
}

const std::vector<uint8_t> CommandInterface::create_frame(const std::vector<uint8_t>& bytes)
{
  std::vector<uint8_t> frame;
  frame.emplace_back(DELIMITER_FLAG);
  for (const uint8_t byte : bytes)
  {
    const std::vector<uint8_t> escaped_byte = escape(byte);
    frame.insert(std::end(frame), std::begin(escaped_byte), std::end(escaped_byte));
  }
  frame.emplace_back(DELIMITER_FLAG);
  return frame;
}

void CommandInterface::read()
{
  // In this method we will read data from the serial port until we find
  // a start and an end delimiter, or until timeout.

  // We will create a state machine to read the first and the last delimiter.
  // Once we have the frame entirely written to a buffer std::vector<uint8_t>,
  // we unescape it, parse it, check the CRC and call the related callback.

  // If by any chance the data is corrupted and we cannot identify the last
  // delimiter, we will read more bytes than required and this will throw an
  // exception.

  uint16_t crc = 0;
  state_ = State::StartReading;
  while (state_ != State::MessageRead)
  {
    uint8_t byte;
    serial_->read(&byte, 1);

    switch (state_)
    {
      case State::StartReading:
        if (byte == DELIMITER_FLAG)
        {
          state_ = State::ReadingMessage;
          read_buffer_.clear();
        }
        else
        {
          // throw exception
        }
        break;
      case State::ReadingMessage:
        if (byte == ESCAPE_FLAG)
        {
          state_ = State::ReadingEscapedByte;
        }
        else if (byte == DELIMITER_FLAG)
        {
          state_ = State::MessageRead;
        }
        else
        {
          crc = crc_utils::crc_ccitt_byte(crc, byte);
          read_buffer_.add_int8(byte, BufferPosition::Tail);
        }
        break;
      case State::ReadingEscapedByte:
        crc = crc_utils::crc_ccitt_byte(crc, byte ^ ESCAPED_XOR);
        read_buffer_.add_int8(byte ^ ESCAPED_XOR, BufferPosition::Tail);
        state_ = State::ReadingMessage;
        break;
      default:;
        // throw exception.
    }
  }
}

Response CommandInterface::write(const Request& request)
{
  const std::vector<uint8_t> bytes = request.bytes();

  std::vector<uint8_t> data;
  data.emplace_back(requestId++);
  data.insert(std::end(data), std::begin(bytes), std::end(bytes));
  const uint16_t crc = crc_utils::crc_ccitt(request.bytes());
  const uint8_t crc_lsb = (crc & 0xff00) >> 8;
  const uint8_t crc_msb = (crc & 0x00ff);
  data.emplace_back(crc_msb);
  data.emplace_back(crc_lsb);

  std::vector<uint8_t> frame = create_frame(data);
  serial_->write(frame);

  return {};
}

}  // namespace stepit_hardware
