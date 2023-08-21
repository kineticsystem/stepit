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

#pragma once

#include <cobs_serial/serial.hpp>
#include <cobs_serial/data_interface.hpp>
#include <cobs_serial/buffer.hpp>

#include <vector>
#include <cstdint>
#include <string>
#include <memory>

namespace cobs_serial
{
/**
 * This class is used to pack a sequence of bytes into a frame and send it to
 * the serial port and also to parse frames coming from the serial port.
 * A frames contains the data, a 16-bits CRC and delimiters.
 */
class DefaultDataInterface : public DataInterface
{
public:
  explicit DefaultDataInterface(std::unique_ptr<Serial> serial);

  /**
   * Open the serial connection.
   */
  void open() override;

  /**
   * Close the serial connection.
   */
  void close() override;

  /**
   * Write a sequence of bytes to the serial port.
   * @param bytes The bytes to read.
   * @throw cobs_serial::SerialException
   */
  void write(const std::vector<uint8_t>& bytes) override;

  /**
   * Read a sequence of bytes from the serial port.
   * @return The bytes read.
   * @throw cobs_serial::SerialException
   */
  std::vector<uint8_t> read() override;

private:
  /* States used while reading and parsiong a frame. */
  enum class ReadState
  {
    Waiting,
    ReadingMessage,
    ReadingEscapedByte
  } state_ = ReadState::Waiting;

  /* Circular buffer to read data from the serial port. */
  Buffer<uint8_t> read_buffer_{ 100 };

  /* Circular buffer to write data to the serial port. */
  Buffer<uint8_t> write_buffer_{ 100 };

  std::unique_ptr<Serial> serial_;
};
}  // namespace cobs_serial
