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

#include <gtest/gtest.h>
#include <cobs_serial/data_utils.hpp>

namespace cobs_serial::test
{
TEST(TestDataUtils, to_hex)
{
  std::vector<uint8_t> bytes = { 0x7E, 0x00, 0x70, 0x00, 0x00, 0x00, 0x4E, 0x20, 0x75, 0x38, 0x7E };
  std::string hex = cobs_serial::data_utils::to_hex(bytes);
  ASSERT_EQ(hex, "7E 00 70 00 00 00 4E 20 75 38 7E");
}

TEST(TestDataUtils, from_float)
{
  // See https://www.h-schmidt.net/FloatConverter/IEEE754.html
  // Convert 4.2 into a sequence of bytes.
  auto bytes = cobs_serial::data_utils::from_float(4.2f);
  std::vector<uint8_t> bytes_v(bytes.begin(), bytes.end());
  auto hex = cobs_serial::data_utils::to_hex(bytes_v);
  ASSERT_EQ(hex, "40 86 66 66");
}

TEST(TestDataUtils, to_float)
{
  // See https://www.h-schmidt.net/FloatConverter/IEEE754.html
  // Check if the given sequence of bytes equals to the float 4.2.
  std::array<uint8_t, 4> bytes = { 0x40, 0x86, 0x66, 0x66 };
  auto value = cobs_serial::data_utils::to_float(bytes);
  ASSERT_FLOAT_EQ(value, 4.2f);
}

TEST(TestDataUtils, from_int32)
{
  auto bytes = cobs_serial::data_utils::from_int32(-1582119980);
  std::vector<uint8_t> bytes_v(bytes.begin(), bytes.end());
  auto hex = cobs_serial::data_utils::to_hex(bytes_v);
  ASSERT_EQ(hex, "A1 B2 C3 D4");
}

TEST(TestDataUtils, to_int32)
{
  std::array<uint8_t, 4> bytes = { 0xA1, 0xB2, 0xC3, 0xD4 };
  auto value = cobs_serial::data_utils::to_int32(bytes);
  ASSERT_EQ(value, -1582119980);
}

TEST(TestDataUtils, from_int16)
{
  auto bytes = cobs_serial::data_utils::from_int16(-3937);
  std::vector<uint8_t> bytes_v(bytes.begin(), bytes.end());
  auto hex = cobs_serial::data_utils::to_hex(bytes_v);
  ASSERT_EQ(hex, "F0 9F");
}

TEST(TestDataUtils, to_int16)
{
  std::array<uint8_t, 2> bytes = { 0xF0, 0x9F };
  auto value = cobs_serial::data_utils::to_int16(bytes);
  ASSERT_EQ(value, -3937);
}
}  // namespace cobs_serial::test
