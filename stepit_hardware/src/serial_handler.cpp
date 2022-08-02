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

#include <stepit_hardware/serial_handler.hpp>

#include <serial/serial.h>

namespace stepit_hardware
{
SerialHandler::SerialHandler() : serial_{ std::make_unique<serial::Serial>() }
{
}

void SerialHandler::open()
{
  serial_->open();
}

bool SerialHandler::is_open() const
{
  return serial_->isOpen();
}

void SerialHandler::close()
{
  serial_->close();
}

std::size_t SerialHandler::read(uint8_t* buffer, size_t size)
{
  return serial_->read(buffer, size);
}

std::size_t SerialHandler::write(const uint8_t* buffer, size_t size)
{
  std::size_t write_size = serial_->write(buffer, size);
  serial_->flush();
  return write_size;
}

void SerialHandler::set_port(const std::string& port)
{
  serial_->setPort(port);
}

std::string SerialHandler::get_port() const
{
  return serial_->getPort();
}

void SerialHandler::set_timeout(uint32_t timeout_ms)
{
  timeout_ms_ = timeout_ms;
  serial::Timeout timeout = serial::Timeout::simpleTimeout(timeout_ms);
  serial_->setTimeout(timeout);
}

uint32_t SerialHandler::get_timeout() const
{
  return timeout_ms_;
}

void SerialHandler::set_baudrate(uint32_t baudrate)
{
  serial_->setBaudrate(baudrate);
}

uint32_t SerialHandler::get_baudrate() const
{
  return serial_->getBaudrate();
}
}  // namespace stepit_hardware
