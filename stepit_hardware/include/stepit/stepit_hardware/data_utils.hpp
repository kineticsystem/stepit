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

#include <array>
#include <string>

/**
 * Utility class to convert standard types into sequence of bytes and back.
 * Please not that Linux on X86 and Rasperry PI are Little-Endian systems.
 * It is but convention to send data to the network in Big-Endian order.
 * To find out if a system is Little or Big-Endian, type the following command
 * in a terminal.
 *
 * lscpu | grep Endian
 */
namespace stepit::stepit_hardware::data_utils
{
constexpr std::array<char, 16> vChars = {
  '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'
};

/**
 * Convert a 32 bits integer into a 4 bytes array in Big Endian byte order.
 * @param value A 32 bits integer to convert.
 * @return The resulting 4 bytes array in Big Endian byte order.
 */
std::array<uint8_t, 4> from_int_32(const int32_t value);

/**
 * Convert a 4 bytes array in Big Endian byte order into a 32 bits integer.
 * @param bytes A 4 bytes array in Big Endian byte order.
 * @return The resuting 32 bits integer.
 */
int32_t to_int_32(const std::array<uint8_t, 4>& bytes);

/**
 * Convert a 16 bits integer into an 2 bytes array in Big Endian byte order.
 * @param value A 16 bits integer to convert.
 * @return The resulting 2 bytes array in Big Endian byte order.
 */
std::array<uint8_t, 2> from_int_16(const int16_t value);

/**
 * Convert a 2 bytes array in Big Endian byte order into a 16 bits integer.
 * @param bytes A 2 bytes array in Big Endian byte order.
 * @return The resuting 16 bits integer.
 */
int16_t to_int_16(const std::array<uint8_t, 2>& bytes);

std::array<uint8_t, 4> from_float(const float value);

float to_float(const std::array<uint8_t, 4>& bytes);

/**
 * Convert a sequence of bytes into a sequence of hex numbers.
 * @param bytes The sequence of bytes.
 * @return A string containing the sequence of hex numbers.
 */
template <std::size_t SIZE>
std::string to_hex(const std::array<uint8_t, SIZE>& bytes)
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
