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

#include <stepit_hardware/data_interface.h>

#include <stepit_hardware/crc_utils.h>
#include <stepit_hardware/data_utils.h>
#include <stepit_hardware/serial_exception.h>

#include <cstdint>

namespace stepit_hardware
{
constexpr uint8_t DELIMITER_FLAG = 0x7E;  // Start and end of a packet
constexpr uint8_t ESCAPE_FLAG = 0x7D;     // Escaping byte.
constexpr uint8_t ESCAPED_XOR = 0x20;     // XOR value applied to escaped bytes.

DataInterface::DataInterface(SerialInterface* serial) : serial_{ serial }
{
}

const std::vector<uint8_t> DataInterface::escape(uint8_t byte)
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

const std::vector<uint8_t> DataInterface::create_frame(const std::vector<uint8_t>& bytes)
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

std::vector<uint8_t> DataInterface::read()
{
  uint16_t crc = 0;
  state_ = State::StartReading;
  uint8_t byte = 0;

  while (true)
  {
    if (serial_->read(&byte, 1) == 0)
    {
      throw SerialException("timeout");
    }

    if (state_ == State::StartReading)
    {
      if (byte == DELIMITER_FLAG)
      {
        read_buffer_.clear();
        state_ = State::ReadingMessage;
      }
      else
      {
        throw SerialException("start delimiter missing");
      }
    }
    else if (state_ == State::ReadingMessage)
    {
      if (byte == ESCAPE_FLAG)
      {
        state_ = State::ReadingEscapedByte;
      }
      else if (byte == DELIMITER_FLAG)
      {
        if (read_buffer_.size() < 4)
        {
          throw SerialException("incorrect frame length");
        }
        else if (crc != 0)
        {
          throw SerialException("CRC error");
        }
        else
        {
          // Remove request ID.
          read_buffer_.remove(BufferPosition::Head);

          // Remove CRC from buffer.
          read_buffer_.remove(BufferPosition::Tail);
          read_buffer_.remove(BufferPosition::Tail);

          break;
        }
      }
      else
      {
        crc = crc_utils::crc_ccitt_byte(crc, byte);
        read_buffer_.add(byte, BufferPosition::Tail);
      }
    }
    else if (state_ == State::ReadingEscapedByte)
    {
      crc = crc_utils::crc_ccitt_byte(crc, byte ^ ESCAPED_XOR);
      read_buffer_.add(byte ^ ESCAPED_XOR, BufferPosition::Tail);
      state_ = State::ReadingMessage;
    }
  }

  std::vector<uint8_t> buffer;
  buffer.reserve(read_buffer_.size());
  while (read_buffer_.size() > 0)
  {
    buffer.emplace_back(read_buffer_.remove(BufferPosition::Head));
  }

  return buffer;
}

void DataInterface::write(const std::vector<uint8_t>& bytes)
{
  std::vector<uint8_t> data;
  data.emplace_back(requestId++);
  data.insert(std::end(data), std::begin(bytes), std::end(bytes));
  const uint16_t crc = crc_utils::crc_ccitt(bytes);
  const uint8_t crc_lsb = (crc & 0xff00) >> 8;
  const uint8_t crc_msb = (crc & 0x00ff);
  data.emplace_back(crc_msb);
  data.emplace_back(crc_lsb);

  std::vector<uint8_t> frame = create_frame(data);
  std::size_t size = serial_->write(frame);
}

}  // namespace stepit_hardware
