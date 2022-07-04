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

#include <gtest/gtest.h>
#include <stepit/stepit_hardware/data_utils.hpp>

namespace stepit::stepit_hardware::tests
{
TEST(UtilsTest, to_hex_test)
{
  std::string hex = stepit::stepit_hardware::data_utils::to_hex({ 126, 0, 112, 0, 0, 0, 78, 32, 117, 56, 126 });
  ASSERT_EQ(hex, "0x7E 0x00 0x70 0x00 0x00 0x00 0x4E 0x20 0x75 0x38 0x7E");
}
}  // namespace stepit::stepit_hardware::tests
