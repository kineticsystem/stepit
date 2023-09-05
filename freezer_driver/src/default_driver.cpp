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

#include <thread>

#include <freezer_driver/default_driver.hpp>

#include <cobs_serial/data_utils.hpp>

#include <rclcpp/logging.hpp>
#include <rclcpp/time.hpp>

namespace freezer_driver
{
const auto kLogger = rclcpp::get_logger("DefaultDriver");

constexpr uint8_t kStatusQueryId = 0x75;

using cobs_serial::data_utils::to_float;
using cobs_serial::data_utils::to_hex;

DefaultDriver::DefaultDriver(std::unique_ptr<cobs_serial::CobsSerial> cobs_serial)
  : cobs_serial_{ std::move(cobs_serial) }
{
}

bool DefaultDriver::connect()
{
  cobs_serial_->open();

  // Send a status command multiple times until an answer comes back.

  int trial = 0;
  bool connected = false;
  while (!connected && trial < 5)
  {
    try
    {
      RCLCPP_INFO(kLogger, "Connecting...");

      StatusResponse response = get_status(rclcpp::Time{});
      connected = response.status() == Response::Status::Success;

      RCLCPP_INFO(kLogger, "Connection established");
    }
    catch (const std::exception& ex)
    {
      RCLCPP_WARN(kLogger, "Connection failed");
      trial++;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  return connected;
}

void DefaultDriver::disconnect()
{
  cobs_serial_->close();
}

StatusResponse DefaultDriver::get_status([[maybe_unused]] const rclcpp::Time& time) const
{
  std::vector<uint8_t> in;
  in.emplace_back(kStatusQueryId);
  RCLCPP_DEBUG(kLogger, "Status query: %s", to_hex(in).c_str());
  cobs_serial_->write(in);
  auto out = cobs_serial_->read();
  RCLCPP_DEBUG(kLogger, "Status response: %s", to_hex(out).c_str());

  // The data array contains the following information.
  //
  // status               - 1 byte
  //
  // motor id             - 1 byte
  // motor position       - 4 bytes
  // motor speed          - 4 bytes
  // motor distance to go - 4 bytes
  //
  // motor id             - 1 byte
  // motor position       - 4 bytes
  // motor speed          - 4 bytes
  // motor distance to go - 4 bytes
  //
  // ...and so on.

  std::size_t i = 0;
  Response::Status status{ out[i++] };
  std::vector<MotorState> motor_states;
  while (i < out.size())
  {
    uint8_t id = out[i++];
    float position = to_float({ out[i++], out[i++], out[i++], out[i++] });
    float speed = to_float({ out[i++], out[i++], out[i++], out[i++] });
    float distance_to_go = to_float({ out[i++], out[i++], out[i++], out[i++] });
    motor_states.push_back(MotorState{ id, position, speed, distance_to_go });
  }

  StatusResponse response{ status, motor_states };
  return response;
}

}  // namespace freezer_driver
