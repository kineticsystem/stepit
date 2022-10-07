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

#include <stepit_hardware/serial_interface.hpp>
#include <stepit_hardware/data_interface.hpp>
#include <stepit_hardware/buffer.hpp>

#include <vector>
#include <cstdint>
#include <string>
#include <memory>

namespace stepit_hardware
{
/**
 * This class is used to pack a sequence of bytes into a frame and send it to
 * the serial port and also to parse frames coming from the serial port.
 * A frames contains the data, a 16-bits CRC and delimiters.
 */
class DataHandler : public DataInterface
{
public:
  explicit DataHandler(std::unique_ptr<SerialInterface> serial);

  /**
   * Write a sequence of bytes to the serial port.
   * @param bytes The bytes to read.
   * @throw stepit_hardware::SerialException
   */
  void write(const std::vector<uint8_t>& bytes);

  /**
   * Read a sequence of bytes from the serial port.
   * @return The bytes read.
   * @throw stepit_hardware::SerialException
   */
  std::vector<uint8_t> read();

private:
  /* States used while reading and parsiong a frame. */
  enum class ReadState
  {
    StartReading,
    ReadingMessage,
    ReadingEscapedByte
  } state_ = ReadState::StartReading;

  /* Circular buffer to read data from the serial port. */
  Buffer<uint8_t> read_buffer_{ 100 };

  /* Circular buffer to write data to the serial port. */
  Buffer<uint8_t> write_buffer_{ 100 };

  std::unique_ptr<SerialInterface> serial_;
};
}  // namespace stepit_hardware
