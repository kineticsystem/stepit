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

#include <stepit_control/data_handler.hpp>

#include <stepit_control/crc_utils.hpp>
#include <stepit_control/data_utils.hpp>
#include <stepit_control/serial_exception.hpp>

#include <cstdint>

namespace stepit_control
{
constexpr uint8_t DELIMITER_FLAG = 0x7E;  // Start and end of a packet
constexpr uint8_t ESCAPE_FLAG = 0x7D;     // Escaping byte.
constexpr uint8_t ESCAPED_XOR = 0x20;     // XOR value applied to escaped bytes.

DataHandler::DataHandler(std::unique_ptr<SerialInterface> serial) : serial_{ std::move(serial) }
{
}

std::vector<uint8_t> DataHandler::read()
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

void DataHandler::write(const std::vector<uint8_t>& bytes)
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
  const uint8_t crc_lsb = static_cast<uint8_t>((crc & 0xff00) >> 8);
  const uint8_t crc_msb = static_cast<uint8_t>((crc & 0x00ff));
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

}  // namespace stepit_control
