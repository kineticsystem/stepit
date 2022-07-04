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
#include <vector>

/**
 * Utility class to convert standard types into sequence of bytes and back.
 */
namespace stepit::stepit_hardware::data_utils
{
/**
 * Convert a 32 bits integer into a 4 bytes array in Big Endian byte order.
 * @param value A 32 bits integer to convert.
 * @return The resulting 4 bytes array in Big Endian byte order.
 */
std::array<uint8_t, 4> from_int32(const int32_t value);

/**
 * Convert a 4 bytes array in Big Endian byte order into a 32 bits integer.
 * @param bytes A 4 bytes array in Big Endian byte order.
 * @return The resuting 32 bits integer.
 */
int32_t to_int32(const std::array<uint8_t, 4>& bytes);

/**
 * Convert a 16 bits integer into an 2 bytes array in Big Endian byte order.
 * @param value A 16 bits integer to convert.
 * @return The resulting 2 bytes array in Big Endian byte order.
 */
std::array<uint8_t, 2> from_int16(const int16_t value);

/**
 * Convert a 2 bytes array in Big Endian byte order into a 16 bits integer.
 * @param bytes A 2 bytes array in Big Endian byte order.
 * @return The resuting 16 bits integer.
 */
int16_t to_int16(const std::array<uint8_t, 2>& bytes);

std::array<uint8_t, 4> from_float(const float value);

float to_float(const std::array<uint8_t, 4>& bytes);

/**
 * Conver a sequence of bytes into a sequence of hex numbers.
 * @param bytes The sequence of bytes.
 * @return A string containing the sequence of hex numbers.
 */
std::string to_hex(const std::vector<uint8_t>& bytes);
}  // namespace stepit::stepit_hardware::data_utils
