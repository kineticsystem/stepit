// Copyright 2023 Giovanni Remigi
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Giovanni Remigi nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <freezer_driver/default_driver.hpp>
#include <freezer_driver/default_driver_factory.hpp>

#include <cobs_serial/default_cobs_serial.hpp>
#include <cobs_serial/default_cobs_serial_factory.hpp>

#include <rclcpp/logging.hpp>

namespace freezer_driver
{
const auto kLogger = rclcpp::get_logger("freezer_default_driver_factory");

constexpr auto kConnectionTimeoutParamName = "connection-timeout";
constexpr auto kConnectionTimeoutParamDefault = 5;

std::unique_ptr<freezer_driver::Driver>
freezer_driver::DefaultDriverFactory::create(const hardware_interface::HardwareInfo& info)
{
  RCLCPP_INFO(kLogger, "Reading connection timeout...");
  double connection_timeout = info.hardware_parameters.count(kConnectionTimeoutParamName) ?
                                  std::stod(info.hardware_parameters.at(kConnectionTimeoutParamName)) :
                                  kConnectionTimeoutParamDefault;
  RCLCPP_INFO(kLogger, "connection timeout: %fs", connection_timeout);

  auto driver = create_driver(info);
  driver->set_connection_timeout(std::chrono::duration<double>{ connection_timeout });

  return driver;
}

std::unique_ptr<Driver> DefaultDriverFactory::create_driver(const hardware_interface::HardwareInfo& info) const
{
  auto cobs_serial = cobs_serial::DefaultCobsSerialFactory().create(info);
  auto driver = std::make_unique<DefaultDriver>(std::move(cobs_serial));
  return driver;
}
}  // namespace freezer_driver
