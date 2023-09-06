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

#include <cobs_serial/data_utils.hpp>

namespace cobs_serial::data_utils
{
constexpr std::array<char, 16> vChars = {
  '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'
};

std::array<uint8_t, 2> from_int16(const int16_t value)
{
  // The result of this conversion does not depend on the endianness of the system.
  std::array<uint8_t, 2> bytes;
  bytes[0] = static_cast<uint8_t>(value >> 8) & 0xFF;  // MSB
  bytes[1] = static_cast<uint8_t>(value & 0xFF);       // LSB
  return bytes;
}

int16_t to_int16(const std::array<uint8_t, 2>& bytes)
{
  // The result of this conversion does not depend on the endianness of the system.
  const int16_t value = static_cast<int16_t>((bytes[0] << 8) + bytes[1]);
  return value;
}

std::array<uint8_t, 4> from_int32(const int32_t value)
{
  // The result of this conversion does not depend on the endianness of the system.
  std::array<uint8_t, 4> bytes;
  bytes[0] = static_cast<uint8_t>((value >> 24) & 0xFF);  // MSB
  bytes[1] = static_cast<uint8_t>((value >> 16) & 0xFF);
  bytes[2] = static_cast<uint8_t>((value >> 8) & 0xFF);
  bytes[3] = static_cast<uint8_t>(value & 0xFF);  // LSB
  return bytes;
}

int32_t to_int32(const std::array<uint8_t, 4>& bytes)
{
  // The result of this conversion does not depend on the endianness of the system.
  const int32_t value = (bytes[0] << 24) + (bytes[1] << 16) + (bytes[2] << 8) + bytes[3];
  return value;
}

std::array<uint8_t, 4> from_float(const float value)
{
  // The result of this conversion depends on the endianness of the system.
  const auto value_p = reinterpret_cast<const uint8_t*>(&value);
  std::array<uint8_t, 4> bytes;
  bytes[0] = value_p[3];
  bytes[1] = value_p[2];
  bytes[2] = value_p[1];
  bytes[3] = value_p[0];
  return bytes;
}

float to_float(const std::array<uint8_t, 4>& bytes)
{
  // The result of this conversion depends on the endianness of the system.
  float value;
  const auto value_p = reinterpret_cast<uint8_t*>(&value);
  value_p[0] = bytes[3];
  value_p[1] = bytes[2];
  value_p[2] = bytes[1];
  value_p[3] = bytes[0];
  return value;
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
    hex += "";
    uint8_t ch = *it;
    hex += vChars[((ch >> 4) & 0xF)];
    hex += vChars[(ch & 0xF)];
  }

  return hex;
}

std::string to_hex(const std::vector<uint16_t>& bytes)
{
  std::string hex;
  for (auto it = std::begin(bytes); it != std::end(bytes); ++it)
  {
    if (it != bytes.begin())
    {
      hex += " ";
    }
    hex += "";
    uint16_t ch = *it;
    hex += vChars[((ch >> 12) & 0xF)];
    hex += vChars[((ch >> 8) & 0xF)];
    hex += vChars[((ch >> 4) & 0xF)];
    hex += vChars[(ch & 0xF)];
  }

  return hex;
}

}  // namespace cobs_serial::data_utils
