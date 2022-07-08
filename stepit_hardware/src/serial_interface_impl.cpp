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

#include <stepit_hardware/serial_interface_impl.h>

#include <serial/serial.h>

namespace stepit_hardware
{
SerialInterfaceImpl::SerialInterfaceImpl() : pimpl_{ new serial::Serial() }
{
}

bool SerialInterfaceImpl::isOpen() const
{
  return pimpl_->isOpen();
}

void SerialInterfaceImpl::close()
{
  pimpl_->close();
}

std::size_t SerialInterfaceImpl::read(std::vector<uint8_t>& buffer, size_t size)
{
  return pimpl_->read(buffer, size);
}

size_t SerialInterfaceImpl::write(const std::vector<uint8_t>& data)
{
  return pimpl_->write(data);
}

void SerialInterfaceImpl::setPort(const std::string& port)
{
  pimpl_->setPort(port);
}

std::string SerialInterfaceImpl::getPort() const
{
  return pimpl_->getPort();
}
}  // namespace stepit_hardware
