/*
 * Copyright (c) 2022, Giovanni Remigi
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

#include <cobs_serial/serial.hpp>

#include <memory>
#include "serial/serial.h"

namespace serial
{
class Serial;
}

namespace cobs_serial
{
class DefaultSerial : public Serial
{
public:
  /**
   * Creates a Serial object to send and receive bytes to and from the serial
   * port.
   */
  DefaultSerial();

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
}  // namespace cobs_serial
