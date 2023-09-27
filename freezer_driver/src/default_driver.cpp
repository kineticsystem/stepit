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

#include <thread>

#include <freezer_driver/default_driver.hpp>

#include <cobs_serial/data_utils.hpp>

#include <rclcpp/logging.hpp>
#include <rclcpp/time.hpp>

namespace freezer_driver
{
const auto kLogger = rclcpp::get_logger("freezer_default_driver");

constexpr uint8_t kExecuteCommandId = 0x70;
constexpr uint8_t kReadyMessageId = 0x80;

using cobs_serial::data_utils::from_int32;
using cobs_serial::data_utils::to_float;
using cobs_serial::data_utils::to_hex;

DefaultDriver::DefaultDriver(std::unique_ptr<cobs_serial::CobsSerial> cobs_serial)
  : cobs_serial_{ std::move(cobs_serial) }
{
}

bool DefaultDriver::connect()
{
  cobs_serial_->open();
  if (cobs_serial_->is_open())
  {
    auto start_time = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start_time < connection_timeout_)
    {
      try
      {
        if (auto response = cobs_serial_->read(); response.size() == 1 && response[0] == kReadyMessageId)
        {
          return true;
        }
      }
      catch (const std::exception& e)
      {
        // Expected timeout.
      }
    }
  }
  return false;
}

void DefaultDriver::disconnect()
{
  cobs_serial_->close();
}

AcknowledgeResponse DefaultDriver::execute(const rclcpp::Time&, const BitsetCommand& command)
{
  std::vector<uint8_t> in;
  in.emplace_back(kExecuteCommandId);
  for (const auto& step : command.steps())
  {
    auto bits = from_int32(static_cast<int32_t>(step.bits()));
    in.emplace_back(bits[0]);
    in.emplace_back(bits[1]);
    in.emplace_back(bits[2]);
    in.emplace_back(bits[3]);
    auto delay = from_int32(static_cast<int32_t>(step.delay().count()));
    in.emplace_back(delay[0]);
    in.emplace_back(delay[1]);
    in.emplace_back(delay[2]);
    in.emplace_back(delay[3]);
  }
  RCLCPP_DEBUG(kLogger, "Bitset command: %s", to_hex(in).c_str());
  cobs_serial_->write(in);
  std::vector<uint8_t> out = cobs_serial_->read();
  RCLCPP_DEBUG(kLogger, "Bitset response: %s", to_hex(out).c_str());
  Response::Status status{ out[0] };
  AcknowledgeResponse response{ status };
  return response;
}

void DefaultDriver::set_connection_timeout(std::chrono::duration<double> connection_timeout)
{
  connection_timeout_ = connection_timeout;
}

std::chrono::duration<double> DefaultDriver::get_connection_timeout() const
{
  return connection_timeout_;
}

}  // namespace freezer_driver
