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

DataInterface::DataInterface(Serial* serial) : serial_{ serial }
{
}

std::vector<uint8_t> DataInterface::read()
{
  read_buffer_.clear();

  uint16_t crc = 0;
  state_ = ReadState::StartReading;
  uint8_t byte = 0;

  while (true)
  {
    if (serial_->read(&byte, 1) == 0)
    {
      throw SerialException("timeout");
    }

    if (state_ == ReadState::StartReading)
    {
      if (byte == DELIMITER_FLAG)
      {
        state_ = ReadState::ReadingMessage;
      }
      else
      {
        throw SerialException("start delimiter missing");
      }
    }
    else if (state_ == ReadState::ReadingMessage)
    {
      if (byte == ESCAPE_FLAG)
      {
        state_ = ReadState::ReadingEscapedByte;
      }
      else if (byte == DELIMITER_FLAG)
      {
        if (read_buffer_.size() < 3)
        {
          // We expect at least one byte of data and two bytes for the CRC.
          throw SerialException("incorrect frame length");
        }
        else if (crc != 0)
        {
          throw SerialException("CRC error");
        }
        else
        {
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
    else if (state_ == ReadState::ReadingEscapedByte)
    {
      crc = crc_utils::crc_ccitt_byte(crc, byte ^ ESCAPED_XOR);
      read_buffer_.add(byte ^ ESCAPED_XOR, BufferPosition::Tail);
      state_ = ReadState::ReadingMessage;
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
  write_buffer_.clear();
  uint16_t crc = 0;

  write_buffer_.add(DELIMITER_FLAG, BufferPosition::Tail);
  for (const uint8_t& byte : bytes)
  {
    crc = crc_utils::crc_ccitt_byte(crc, byte);
    if (byte == DELIMITER_FLAG || byte == ESCAPE_FLAG)
    {
      write_buffer_.add(ESCAPE_FLAG, BufferPosition::Tail);
      write_buffer_.add(byte ^ ESCAPED_XOR, BufferPosition::Tail);
    }
    else
    {
      write_buffer_.add(byte, BufferPosition::Tail);
    }
  }
  const uint8_t crc_lsb = (crc & 0xff00) >> 8;
  const uint8_t crc_msb = (crc & 0x00ff);
  write_buffer_.add(crc_msb, BufferPosition::Tail);
  write_buffer_.add(crc_lsb, BufferPosition::Tail);
  write_buffer_.add(DELIMITER_FLAG, BufferPosition::Tail);

  while (write_buffer_.size() > 0)
  {
    const uint8_t byte = write_buffer_.remove(BufferPosition::Head);
    std::size_t size = serial_->write(&byte, 1);
    if (size == 0)
    {
      throw SerialException("error writing to the serial port");
    }
  }
}

}  // namespace stepit_hardware
