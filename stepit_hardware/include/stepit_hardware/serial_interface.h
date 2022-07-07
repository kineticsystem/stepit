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

#include <stepit_hardware/request.h>
#include <stepit_hardware/response.h>
#include <serial/serial.h>

#include <vector>
#include <cstdint>

namespace stepit_hardware
{
class SerialInterface
{
public:
  explicit SerialInterface(serial::Serial* serial);
  Response send(const Request& request);

private:
  /**
   * Escape the given byte according to the PPP specification.
   * @param value The byte to escape.
   */
  static const std::vector<uint8_t> escape(uint8_t byte);

  static const std::vector<uint8_t> create_frame(const std::vector<uint8_t>& data);

  uint8_t requestId = 0;

  serial::Serial* serial_ = nullptr;
};
}  // namespace stepit_hardware
