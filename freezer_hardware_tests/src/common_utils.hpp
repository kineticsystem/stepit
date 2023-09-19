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

#include <string>
#include <stdexcept>
#include <limits>
#include <type_traits>

namespace common_utils
{
/**
 * Safely convert a string representing a number in a given base to an
 * unsigned integer.
 * The base is determined by the format of the string:
 * - If the string starts with "0x" or "0X", it's interpreted as hexadecimal (base 16).
 * - If the string starts with "0", it's interpreted as octal (base 8).
 * - Otherwise, it's interpreted as decimal (base 10).
 * @param str The string representing a number in a given.
 * @param base The representation base.
 * @return The resulting integer value.
 */
template <typename T>
T safe_convert(const std::string& str)
{
  static_assert(std::is_same<T, uint8_t>::value || std::is_same<T, uint16_t>::value || std::is_same<T, uint32_t>::value,
                "Type T must be uint8_t, uint16_t, or uint32_t.");

  unsigned long value;
  try
  {
    if (str.starts_with("0x"))  // Hexadecimal.
    {
      value = std::stoul(str.substr(2), nullptr, 16);
    }
    else if (str.starts_with("0b"))  // Binary.
    {
      value = std::stoul(str.substr(2), nullptr, 2);
    }
    else  // Decimal.
    {
      value = std::stoul(str, nullptr, 10);
    }
  }
  catch (const std::out_of_range& e)
  {
    throw std::out_of_range(std::string{ "Out of range error while parsing: " } + str);
  }
  if (value > std::numeric_limits<T>::max())
  {
    throw std::overflow_error("Overflow error while parsing: " + str);
  }
  return static_cast<T>(value);
}
}  // namespace common_utils
