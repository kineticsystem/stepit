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

#include <cobs_serial/default_serial_factory.hpp>
#include <cobs_serial/default_serial.hpp>

#include <rclcpp/logging.hpp>

namespace cobs_serial
{

const auto kLogger = rclcpp::get_logger("DefaultSerialFactory");

constexpr auto kUsbPortParamName = "usb_port";
constexpr auto kUsbPortParamDefault = "/dev/ttyACM0";

constexpr auto kBaudrateParamName = "baudrate";
constexpr auto kBaudrateAddressParamDefault = 9600;

constexpr auto kTimeoutParamName = "timeout";
constexpr auto kTimeoutParamDefault = 200;  // milliseconds

std::unique_ptr<Serial> DefaultSerialFactory::create(const hardware_interface::HardwareInfo& info) const
{
  RCLCPP_INFO(kLogger, "Reading usb_port...");
  std::string usb_port = info.hardware_parameters.count(kUsbPortParamName) ?
                             info.hardware_parameters.at(kUsbPortParamName) :
                             kUsbPortParamDefault;
  RCLCPP_INFO(kLogger, "usb_port: %s", usb_port.c_str());

  RCLCPP_INFO(kLogger, "Reading baudrate...");
  uint32_t baudrate = info.hardware_parameters.count(kBaudrateParamName) ?
                          static_cast<uint32_t>(std::stoul(info.hardware_parameters.at(kBaudrateParamName))) :
                          kBaudrateAddressParamDefault;
  RCLCPP_INFO(kLogger, "baudrate: %dbps", baudrate);

  RCLCPP_INFO(kLogger, "Reading timeout...");
  uint32_t timeout = static_cast<uint32_t>(info.hardware_parameters.count(kTimeoutParamName) ?
                                               std::stoul(info.hardware_parameters.at(kTimeoutParamName)) :
                                               kTimeoutParamDefault);
  RCLCPP_INFO(kLogger, "timeout: %dms", timeout);

  auto serial = create_objects();
  serial->set_port(usb_port);
  serial->set_baudrate(baudrate);
  serial->set_timeout(timeout);
  return serial;
}

std::unique_ptr<Serial> DefaultSerialFactory::create_objects() const
{
  return std::make_unique<DefaultSerial>();
}
}  // namespace cobs_serial
