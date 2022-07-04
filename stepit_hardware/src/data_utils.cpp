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

constexpr std::array<char, 16> vChars = {
  '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'
};

namespace stepit::stepit_hardware::data_utils
{
std::array<uint8_t, 4> from_int32(const int32_t value)
{
  std::array<uint8_t, 4> bytes;
  bytes[0] = (value >> 24) & 0xFF;
  bytes[1] = (value >> 16) & 0xFF;
  bytes[2] = (value >> 8) & 0xFF;
  bytes[3] = value & 0xFF;
  return bytes;
}

int32_t to_int32(const std::array<uint8_t, 4>& bytes)
{
  int32_t value = (bytes[0] << 24) + (bytes[1] << 16) + (bytes[2] << 8) + bytes[3];
  return value;
}

std::array<uint8_t, 2> from_int16(const int16_t value)
{
  std::array<uint8_t, 2> bytes;
  bytes[0] = (value >> 8) & 0xFF;
  bytes[1] = value & 0xFF;
  return bytes;
}

int16_t to_int16(const std::array<uint8_t, 2>& bytes)
{
  int16_t value = (bytes[0] << 8) + bytes[1];
  return value;
}

std::array<uint8_t, 4> from_float(const float value)
{
  return {};
}

float to_float(const std::array<uint8_t, 4>& bytes)
{
  return 0.0;
}

std::string to_hex(const std::vector<uint8_t>& bytes)
{
  std::string hex;
  for (auto it = std::begin(bytes); it != std::end(bytes); ++it)
  {
    if (it != bytes.begin())
    {
      hex += " ";
    }
    uint8_t ch = *it;
    char ls = vChars[(ch & 0xF)];
    char ms = vChars[((ch >> 4) & 0xF)];
    hex += "0x";
    hex += ms;
    hex += ls;
  }

  return hex;
}

}  // namespace stepit::stepit_hardware::data_utils
