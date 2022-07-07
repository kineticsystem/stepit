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

std::vector<uint8_t> SerialInterface::escape(uint8_t byte) const
{
  std::vector<uint8_t> bytes;
  if (byte == DELIMITER_FLAG || byte == ESCAPE_FLAG)
  {
    bytes.emplace_back(ESCAPE_FLAG);
    bytes.emplace_back(byte ^ ESCAPED_XOR);
  }
  else
  {
    bytes.emplace_back(byte);
  }
  return bytes;
}

Response SerialInterface::send(const Request& request)
{
  const std::vector<uint8_t> bytes = request.bytes();

  std::vector<uint8_t> packet_data;
  packet_data.emplace_back(requestId++);
  packet_data.insert(std::end(packet_data), std::begin(bytes), std::end(bytes));
  const uint16_t crc = crc_utils::calculate_crc(request.bytes());
  const uint8_t crc_lsb = (crc & 0xff00) >> 8;
  const uint8_t crc_msb = (crc & 0x00ff);
  packet_data.emplace_back(crc_msb);
  packet_data.emplace_back(crc_lsb);

  std::vector<uint8_t> packet;
  packet.emplace_back(DELIMITER_FLAG);
  for (const uint8_t byte : packet_data)
  {
    const std::vector<uint8_t> escaped_byte = escape(byte);
    packet.insert(std::end(packet), std::begin(escaped_byte), std::end(escaped_byte));
  }
  packet.emplace_back(DELIMITER_FLAG);

  serial_->write(packet);

  return {};
}

}  // namespace stepit_hardware
