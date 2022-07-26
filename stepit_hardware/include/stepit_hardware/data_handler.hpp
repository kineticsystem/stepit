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
