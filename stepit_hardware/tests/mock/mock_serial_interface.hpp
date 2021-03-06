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

#include <gmock/gmock.h>
#include <stepit_hardware/serial_interface.hpp>

namespace stepit_hardware::test
{
class MockSerialInterface : public stepit_hardware::SerialInterface
{
public:
  MOCK_METHOD(void, open, (), (override));
  MOCK_METHOD(bool, is_open, (), (override, const));
  MOCK_METHOD(void, close, (), (override));
  MOCK_METHOD(std::size_t, read, (uint8_t * buffer, size_t size), (override));
  MOCK_METHOD(std::size_t, write, (const uint8_t* buffer, size_t size), (override));
  MOCK_METHOD(void, set_port, (const std::string& port), (override));
  MOCK_METHOD(std::string, get_port, (), (override, const));
  MOCK_METHOD(void, set_timeout, (uint32_t timeout), (override));
  MOCK_METHOD(uint32_t, get_timeout, (), (override, const));
  MOCK_METHOD(void, set_baudrate, (uint32_t baudrate), (override));
  MOCK_METHOD(uint32_t, get_baudrate, (), (override, const));
};
}  // namespace stepit_hardware::test
