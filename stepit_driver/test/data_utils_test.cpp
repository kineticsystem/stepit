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

#include <gmock/gmock.h>
#include <stepit_driver/data_utils.h>

TEST(data_utils, to_hex_test)
{
  std::vector<uint8_t> bytes = { 0x7E, 0x00, 0x70, 0x00, 0x00, 0x00, 0x4E, 0x20, 0x75, 0x38, 0x7E };
  std::string hex = stepit_driver::data_utils::to_hex(bytes);
  ASSERT_EQ(hex, "7E 00 70 00 00 00 4E 20 75 38 7E");
}

TEST(data_utils, from_float)
{
  // See https://www.h-schmidt.net/FloatConverter/IEEE754.html
  // Convert 4.2 into a sequence of bytes.
  auto bytes = stepit_driver::data_utils::from_float(4.2f);
  std::vector<uint8_t> bytes_v(bytes.begin(), bytes.end());
  auto hex = stepit_driver::data_utils::to_hex(bytes_v);
  ASSERT_EQ(hex, "40 86 66 66");
}

TEST(data_utils, to_float)
{
  // See https://www.h-schmidt.net/FloatConverter/IEEE754.html
  // Check if the given sequence of bytes equals to the float 4.2.
  std::array<uint8_t, 4> bytes = { 0x40, 0x86, 0x66, 0x66 };
  auto value = stepit_driver::data_utils::to_float(bytes);
  ASSERT_FLOAT_EQ(value, 4.2f);
}

TEST(data_utils, from_int32)
{
  auto bytes = stepit_driver::data_utils::from_int32(-1582119980);
  std::vector<uint8_t> bytes_v(bytes.begin(), bytes.end());
  auto hex = stepit_driver::data_utils::to_hex(bytes_v);
  ASSERT_EQ(hex, "A1 B2 C3 D4");
}

TEST(data_utils, to_int32)
{
  std::array<uint8_t, 4> bytes = { 0xA1, 0xB2, 0xC3, 0xD4 };
  auto value = stepit_driver::data_utils::to_int32(bytes);
  ASSERT_EQ(value, -1582119980);
}

TEST(data_utils, from_int16)
{
  auto bytes = stepit_driver::data_utils::from_int16(-3937);
  std::vector<uint8_t> bytes_v(bytes.begin(), bytes.end());
  auto hex = stepit_driver::data_utils::to_hex(bytes_v);
  ASSERT_EQ(hex, "F0 9F");
}

TEST(data_utils, to_int16)
{
  std::array<uint8_t, 2> bytes = { 0xF0, 0x9F };
  auto value = stepit_driver::data_utils::to_int16(bytes);
  ASSERT_EQ(value, -3937);
}
