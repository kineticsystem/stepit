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

#include <stepit_driver/default_driver.hpp>
#include <stepit_driver/default_driver_factory.hpp>
#include <stepit_driver/fake/fake_driver.hpp>

#include <cobs_serial/default_cobs_serial.hpp>
#include <cobs_serial/default_cobs_serial_factory.hpp>

#include <rclcpp/logging.hpp>

namespace stepit_driver
{
const auto kLogger = rclcpp::get_logger("DefaultDriverFactory");

std::unique_ptr<stepit_driver::Driver>
stepit_driver::DefaultDriverFactory::create(const hardware_interface::HardwareInfo& info)
{
  if (info.hardware_parameters.find("use_dummy") != info.hardware_parameters.end() &&
      info.hardware_parameters.at("use_dummy") == "true")
  {
    return std::make_unique<FakeDriver>();
  }
  else
  {
    auto cobs_serial = cobs_serial::DefaultCobsSerialFactory().create(info);
    return std::make_unique<DefaultDriver>(std::move(cobs_serial));
  }
}
}  // namespace stepit_driver
