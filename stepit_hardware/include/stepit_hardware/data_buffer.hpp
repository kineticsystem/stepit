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

#include <stepit_hardware/buffer.hpp>
#include <cstdint>

namespace stepit_hardware
{
class DataBuffer
{
public:
  explicit DataBuffer(std::size_t capacity);

  // Returns how much data is currently stored in the buffer.
  std::size_t size() const;

  // This method resets the buffer into an original state (with no data).
  void clear();

  // Insert a byte at the given position.
  void add_int8(const int8_t value, BufferPosition position);

  // Remove a byte from the given position.
  int8_t remove_int8(BufferPosition position);

  // Insert an int at the given position.
  void add_int16(const int16_t value, BufferPosition position);

  // Remove an int from the given position.
  int16_t remove_int16(BufferPosition position);

  // Insert a long (4 bytes) at the given position.
  void add_int32(const int32_t value, BufferPosition position);

  // Remove a long (4 bytes) from the given position.
  int32_t remove_int32(BufferPosition position);

  // Insert a float (4 bytes) at the given position.
  void add_float(const float value, BufferPosition position);

  // Remove a float (4 bytes) from the given position.
  float remove_float(BufferPosition position);

private:
  Buffer<uint8_t> buffer_;
};
}  // namespace stepit_hardware
