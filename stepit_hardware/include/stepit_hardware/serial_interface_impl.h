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

#include <stepit_hardware/serial_interface.h>

#include <memory>
#include "serial/serial.h"

namespace serial
{
class Serial;
}

namespace stepit_hardware
{
class SerialInterfaceImpl : public SerialInterface
{
public:
  SerialInterfaceImpl();

  [[nodiscard]] bool is_open() const override;

  void close() override;

  [[nodiscard]] std::size_t read(uint8_t* buffer, size_t size = 1) override;

  [[nodiscard]] std::size_t write(const uint8_t* buffer, size_t size) override;

  void set_port(const std::string& port) override;

  [[nodiscard]] std::string get_port() const override;

private:
  std::unique_ptr<serial::Serial> serial_ = nullptr;
};
}  // namespace stepit_hardware
