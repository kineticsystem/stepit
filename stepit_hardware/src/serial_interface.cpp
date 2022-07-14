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

#include <serial/serial.h>

namespace stepit_hardware
{
SerialInterface::SerialInterface() : serial_{ new serial::Serial() }
{
}

bool SerialInterface::is_open() const
{
  return serial_->isOpen();
}

void SerialInterface::close()
{
  serial_->close();
}

std::size_t SerialInterface::read(uint8_t* buffer, size_t size)
{
  return serial_->read(buffer, size);
}

std::size_t SerialInterface::write(const uint8_t* buffer, size_t size)
{
  return serial_->write(buffer, size);
}

void SerialInterface::set_port(const std::string& port)
{
  serial_->setPort(port);
}

std::string SerialInterface::get_port() const
{
  return serial_->getPort();
}
}  // namespace stepit_hardware
