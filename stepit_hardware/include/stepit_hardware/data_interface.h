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

#include <stepit_hardware/serial_interface.h>
#include <stepit_hardware/buffer.h>

#include <vector>
#include <cstdint>
#include <string>

namespace stepit_hardware
{
class DataInterface
{
public:
  explicit DataInterface(SerialInterface* serial);
  void write(const std::vector<uint8_t>& bytes);
  std::vector<uint8_t> read();

private:
  /**
   * Escape the given byte according to the PPP specification.
   * @param value The byte to escape.
   */
  static const std::vector<uint8_t> escape(uint8_t byte);

  static const std::vector<uint8_t> create_frame(const std::vector<uint8_t>& data);

  enum class State
  {
    StartReading,
    ReadingMessage,
    ReadingEscapedByte,
    MessageRead
  } state_ = State::StartReading;

  uint8_t requestId = 0;

  /* Circular buffer to read data from the serial port. */
  Buffer<uint8_t> read_buffer_{ 100 };

  /* Circular buffer to write data to the serial port. */
  Buffer<uint8_t> write_buffer_{ 100 };

  SerialInterface* serial_ = nullptr;
};
}  // namespace stepit_hardware
