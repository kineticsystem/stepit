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
#include <stepit_hardware/serial_interface.h>

namespace stepit_hardware::tests
{

class MockSerialInterface : public SerialInterface
{
public:
  MOCK_METHOD(bool, is_open, (), (const));
  MOCK_METHOD(void, close, ());
  MOCK_METHOD(std::size_t, read, (std::vector<uint8_t> & buffer, size_t size));
  MOCK_METHOD(std::size_t, write, (const std::vector<uint8_t>& buffer));
  MOCK_METHOD(void, set_port, (const std::string& port));
  MOCK_METHOD(std::string, get_port, (), (const));
};
}  // namespace stepit_hardware::tests
