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

#include <stepit_hardware/serial_interface.h>

#include <stepit_hardware/crc_utils.h>
#include <stepit_hardware/data_utils.h>
#include <cstdint>

namespace stepit_hardware
{

constexpr uint8_t DELIMITER_FLAG = 0x7E;  // Start and end of a packet
constexpr uint8_t ESCAPE_FLAG = 0x7D;     // Escaping byte.
constexpr uint8_t ESCAPED_XOR = 0x20;     // XOR value applied to escaped bytes.

SerialInterface::SerialInterface(serial::Serial* serial) : serial_{ serial }
{
}

std::vector<uint8_t> SerialInterface::escape(uint8_t value) const
{
  std::vector<uint8_t> bytes;
  if (value == DELIMITER_FLAG || value == ESCAPE_FLAG)
  {
    bytes.emplace_back(ESCAPE_FLAG);
    bytes.emplace_back(value ^ ESCAPED_XOR);
  }
  else
  {
    bytes.emplace_back(value);
  }
  return bytes;
}

Response SerialInterface::send(const Request& request)
{
  std::vector<uint8_t> bytes;
  bytes.emplace_back(requestId++);
  bytes.emplace_back(request.bytes());
  uint16_t crc = crc_utils::calculate_crc(request.bytes());
  unsigned char crc_lsb = (crc & 0xff00) >> 8;
  unsigned char crc_msb = (crc & 0x00ff);
  bytes.emplace_back(crc_msb);
  bytes.emplace_back(crc_lsb);

  std::vector<uint8_t> packet;
  packet.emplace_back(DELIMITER_FLAG);
  for (const uint8_t byte : bytes)
  {
    packet.emplace_back(escape(byte));
  }
  packet.emplace_back(DELIMITER_FLAG);

  serial_->write(packet);

  return {};
}

}  // namespace stepit_hardware
