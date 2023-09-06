// Copyright 2023 Giovanni Remigi
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Giovanni Remigi nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#pragma once

#include <array>
#include <vector>
#include <sstream>
#include <string>

/**
 * Utility class to convert standard types into sequence of bytes and back.
 * Please note that Linux on X86 and Rasperry PI are Little-Endian systems.
 * It is a convention to send data to the network in Big-Endian order.
 * To find out if a system is Little or Big-Endian, type the following command
 * in a terminal.
 *
 * lscpu | grep Endian
 */
namespace cobs_serial::data_utils
{
/**
 * Parse a string into an unsigned 16-bits value.
 * @param s The string to parse.
 * @return The unsigned 16-bits value.
 */
inline uint16_t stoui16(const std::string& s)
{
  std::istringstream reader(s);
  uint16_t val = 0;
  reader >> val;
  return val;
}

/**
 * Parse a string into an unsigned 32-bits value.
 * @param s The string to parse.
 * @return The unsigned 32-bits value.
 */
inline uint32_t stoui32(const std::string& s)
{
  std::istringstream reader(s);
  uint32_t val = 0;
  reader >> val;
  return val;
}

/**
 * Convert an 8 bits integer into one bytes array.
 * @param value Then 8 bits integer to convert.
 * @return The resulting one byte array.
 */
std::array<uint8_t, 1> from_int8(const int8_t value);

/**
 * Convert a one bytes array into an 8 bits integer.
 * @param bytes The one bytes array.
 * @return The resuting 8bits integer.
 */
int8_t to_int8(const std::array<uint8_t, 1>& bytes);

/**
 * Convert a 16 bits integer into an 2 bytes array in Big Endian byte order.
 * @param value The 16 bits integer to convert.
 * @return The resulting 2 bytes array in Big Endian byte order.
 */
std::array<uint8_t, 2> from_int16(const int16_t value);

/**
 * Convert a 2 bytes array in Big Endian byte order into a 16 bits integer.
 * @param bytes The 2 bytes array in Big Endian byte order.
 * @return The resuting 16 bits integer.
 */
int16_t to_int16(const std::array<uint8_t, 2>& bytes);

/**
 * Convert a 32 bits integer into a 4 bytes array in Big Endian byte order.
 * @param value The 32 bits integer to convert.
 * @return The resulting 4 bytes array in Big Endian byte order.
 */
std::array<uint8_t, 4> from_int32(const int32_t value);

/**
 * Convert a 4 bytes array in Big Endian byte order into a 32 bits integer.
 * @param bytes The 4 bytes array in Big Endian byte order.
 * @return The resuting 32 bits integer.
 */
int32_t to_int32(const std::array<uint8_t, 4>& bytes);

/**
 * Convert a float into a 4 bytes array.
 * @param value The float to convert.
 * @return The resulting 4 bytes array.
 */
std::array<uint8_t, 4> from_float(const float value);

/**
 * Convert a 4 bytes array into a float.
 * @param bytes The 4 bytes array.
 * @return The resuting float.
 */
float to_float(const std::array<uint8_t, 4>& bytes);

/**
 * Convert a sequence of uint8_t into a sequence of hex numbers.
 * @param bytes The sequence of bytes.
 * @return A string containing the sequence of hex numbers.
 */
std::string to_hex(const std::vector<uint8_t>& bytes);

/**
 * Convert a sequence of uint16_t into a sequence of hex numbers.
 * @param bytes The sequence of bytes.
 * @return A string containing the sequence of hex numbers.
 */
std::string to_hex(const std::vector<uint16_t>& bytes);
}  // namespace cobs_serial::data_utils
