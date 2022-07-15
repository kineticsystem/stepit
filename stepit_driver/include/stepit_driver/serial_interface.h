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

#include <stepit_driver/serial.h>

#include <memory>
#include "serial/serial.h"

namespace serial
{
class Serial;
}

namespace stepit_driver
{
class SerialInterface : public Serial
{
public:
  /**
   * Creates a Serial object to send and receive bytes to and from the serial
   * port.
   */
  SerialInterface();

  void open() override;

  [[nodiscard]] bool is_open() const override;

  void close() override;

  [[nodiscard]] std::size_t read(uint8_t* buffer, size_t size = 1) override;
  [[nodiscard]] std::size_t write(const uint8_t* buffer, size_t size) override;

  void set_port(const std::string& port) override;
  [[nodiscard]] std::string get_port() const override;

  void set_timeout(uint32_t timeout_ms) override;
  [[nodiscard]] uint32_t get_timeout() const override;

  void set_baudrate(uint32_t baudrate) override;
  [[nodiscard]] uint32_t get_baudrate() const override;

private:
  std::unique_ptr<serial::Serial> serial_ = nullptr;
  uint32_t timeout_ms_ = 0;
};
}  // namespace stepit_driver
