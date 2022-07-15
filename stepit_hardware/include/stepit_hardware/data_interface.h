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

#include <stepit_hardware/serial.h>
#include <stepit_hardware/buffer.h>

#include <vector>
#include <cstdint>
#include <string>

namespace stepit_hardware
{
class DataInterface
{
public:
  explicit DataInterface(Serial* serial);

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

  Serial* serial_ = nullptr;
};
}  // namespace stepit_hardware
