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
#include <stepit_driver/crc_utils.h>

TEST(crc_utils, calculate_crc)
{
  ASSERT_EQ(stepit_driver::crc_utils::crc_ccitt({ 0xA1, 0xB2, 0xC3, 0xD4, 0xE5, 0xF6 }), 0x3B07);
  ASSERT_EQ(stepit_driver::crc_utils::crc_ccitt({ 0xE2, 0x12, 0xF1, 0xFF, 0x00, 0xD2 }), 0x7071);
  ASSERT_EQ(stepit_driver::crc_utils::crc_ccitt({ 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF }), 0xFBE9);
  ASSERT_EQ(stepit_driver::crc_utils::crc_ccitt({ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }), 0x0000);
  ASSERT_EQ(stepit_driver::crc_utils::crc_ccitt({ 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39 }), 0x2189);
  ASSERT_EQ(stepit_driver::crc_utils::crc_ccitt({ 0x80, 0x00, 0x00, 0x03 }), 0x1ff5);
}
