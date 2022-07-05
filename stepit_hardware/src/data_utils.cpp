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

#include <stepit/stepit_hardware/data_utils.hpp>

namespace stepit::stepit_hardware::data_utils
{
std::array<uint8_t, 4> from_int_32(const int32_t value)
{
  std::array<uint8_t, 4> bytes;
  bytes[0] = (value >> 24) & 0xFF;
  bytes[1] = (value >> 16) & 0xFF;
  bytes[2] = (value >> 8) & 0xFF;
  bytes[3] = value & 0xFF;
  return bytes;
}

int32_t to_int_32(const std::array<uint8_t, 4>& bytes)
{
  int32_t value = (bytes[0] << 24) + (bytes[1] << 16) + (bytes[2] << 8) + bytes[3];
  return value;
}

std::array<uint8_t, 2> from_int_16(const int16_t value)
{
  std::array<uint8_t, 2> bytes;
  bytes[0] = (value >> 8) & 0xFF;
  bytes[1] = value & 0xFF;
  return bytes;
}

int16_t to_int_16(const std::array<uint8_t, 2>& bytes)
{
  int16_t value = (bytes[0] << 8) + bytes[1];
  return value;
}

std::array<uint8_t, 4> from_float(const float value)
{
  const uint8_t* p = reinterpret_cast<const uint8_t*>(&value);
  std::array<uint8_t, 4> bytes;
  bytes[0] = p[3];
  bytes[1] = p[2];
  bytes[2] = p[1];
  bytes[3] = p[0];
  return bytes;
}

float to_float(const std::array<uint8_t, 4>& bytes)
{
  float value;
  uint8_t* p = reinterpret_cast<uint8_t*>(&value);
  p[0] = bytes[3];
  p[1] = bytes[2];
  p[2] = bytes[1];
  p[3] = bytes[0];
  return value;
}

}  // namespace stepit::stepit_hardware::data_utils
