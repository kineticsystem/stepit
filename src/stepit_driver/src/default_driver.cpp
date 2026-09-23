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

#include <stdexcept>
#include <thread>

#include <stepit_driver/default_driver.hpp>

#include <cobs_serial/data_utils.hpp>

#include <rclcpp/logging.hpp>
#include <rclcpp/time.hpp>

namespace stepit_driver
{
const auto kLogger = rclcpp::get_logger("stepit_default_driver");

constexpr uint8_t kMotorConfigCommandId = 0x78;
constexpr uint8_t kMotorPositionCommandId = 0x71;
constexpr uint8_t kInfoQueryId = 0x76;
constexpr uint8_t kMotorStatusQueryId = 0x75;

constexpr uint8_t kMaxConnectionTrials = 5;

// Name reported by the firmware in response to an info query.
constexpr auto kExpectedControllerName = "STEPIT";

// Major version of the protocol this driver speaks. A controller reporting a
// different one lays its packets out differently, so its bytes cannot be
// trusted: see VERSION_MAJOR in src/stepit_mcu/src/main.cpp.
constexpr uint8_t kExpectedProtocolVersion = 1;

using cobs_serial::data_utils::from_float;
using cobs_serial::data_utils::to_float;
using cobs_serial::data_utils::to_hex;

DefaultDriver::DefaultDriver(std::unique_ptr<cobs_serial::CobsSerial> cobs_serial)
  : cobs_serial_{ std::move(cobs_serial) }
{
}

bool DefaultDriver::connect()
{
  cobs_serial_->open();

  // Send an info query multiple times until an answer comes back, then
  // verify that the device on the other end identifies itself as a StepIt
  // controller: the serial port path alone does not tell us what is attached.

  int trial = 0;
  bool connected = false;
  while (!connected && trial < kMaxConnectionTrials)
  {
    try
    {
      RCLCPP_INFO(kLogger, "Connecting (%d of %d)...", trial + 1, kMaxConnectionTrials);

      InfoResponse response = get_info(rclcpp::Time{});
      if (response.status() != Response::Status::Success)
      {
        throw std::runtime_error("Info query failed.");
      }
      if (response.info() != kExpectedControllerName)
      {
        RCLCPP_ERROR(kLogger, "Unexpected device on serial port: expected \"%s\", got \"%s\".", kExpectedControllerName,
                     response.info().c_str());
        break;
      }
      if (response.version().major() != kExpectedProtocolVersion)
      {
        RCLCPP_ERROR(kLogger,
                     "The StepIt controller runs firmware %s, which speaks version %d of the protocol; this driver "
                     "speaks version %d. Flash the firmware that matches this workspace.",
                     response.version().to_string().c_str(), response.version().major(), kExpectedProtocolVersion);
        break;
      }

      connected = true;
      RCLCPP_INFO(kLogger, "Connection established with %s controller, firmware %s.", response.info().c_str(),
                  response.version().to_string().c_str());
    }
    catch (const std::exception& ex)
    {
      RCLCPP_WARN(kLogger, "Connection failed: %s", ex.what());
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

AcknowledgeResponse DefaultDriver::configure(const ConfigCommand& command) const
{
  std::vector<uint8_t> in;
  in.emplace_back(kMotorConfigCommandId);
  for (const auto& param : command.params())
  {
    in.emplace_back(param.motor_id());
    auto acceleration_bytes = from_float(static_cast<float>(param.acceleration()));
    in.emplace_back(acceleration_bytes[0]);
    in.emplace_back(acceleration_bytes[1]);
    in.emplace_back(acceleration_bytes[2]);
    in.emplace_back(acceleration_bytes[3]);
    auto max_velocity_bytes = from_float(static_cast<float>(param.max_velocity()));
    in.emplace_back(max_velocity_bytes[0]);
    in.emplace_back(max_velocity_bytes[1]);
    in.emplace_back(max_velocity_bytes[2]);
    in.emplace_back(max_velocity_bytes[3]);
  }
  RCLCPP_DEBUG(kLogger, "Config command: %s", to_hex(in).c_str());
  cobs_serial_->write(in);
  std::vector<uint8_t> out = cobs_serial_->read();
  RCLCPP_DEBUG(kLogger, "Config response: %s", to_hex(out).c_str());
  Response::Status status{ out[0] };
  AcknowledgeResponse response{ status };
  return response;
}

AcknowledgeResponse DefaultDriver::set_position(const rclcpp::Time&, const PositionCommand& command) const
{
  std::vector<uint8_t> in;
  in.emplace_back(kMotorPositionCommandId);
  for (const auto& goal : command.goals())
  {
    in.emplace_back(goal.motor_id());
    auto position_bytes = from_float(static_cast<float>(goal.position()));
    in.emplace_back(position_bytes[0]);
    in.emplace_back(position_bytes[1]);
    in.emplace_back(position_bytes[2]);
    in.emplace_back(position_bytes[3]);
  }
  RCLCPP_DEBUG(kLogger, "Position command: %s", to_hex(in).c_str());
  cobs_serial_->write(in);
  std::vector<uint8_t> out = cobs_serial_->read();
  RCLCPP_DEBUG(kLogger, "Position response: %s", to_hex(out).c_str());
  Response::Status status{ out[0] };
  AcknowledgeResponse response{ status };
  return response;
}

AcknowledgeResponse DefaultDriver::set_velocity(const rclcpp::Time&, const VelocityCommand& command) const
{
  std::vector<uint8_t> in;
  in.emplace_back(command.command_id());
  for (const auto& goal : command.goals())
  {
    in.emplace_back(goal.motor_id());
    auto velocity_bytes = from_float(static_cast<float>(goal.velocity()));
    in.emplace_back(velocity_bytes[0]);
    in.emplace_back(velocity_bytes[1]);
    in.emplace_back(velocity_bytes[2]);
    in.emplace_back(velocity_bytes[3]);
  }
  RCLCPP_DEBUG(kLogger, "Velocity command: %s", to_hex(in).c_str());
  cobs_serial_->write(in);
  std::vector<uint8_t> out = cobs_serial_->read();
  RCLCPP_DEBUG(kLogger, "Velocity response: %s", to_hex(out).c_str());
  Response::Status status{ out[0] };
  AcknowledgeResponse response{ status };
  return response;
}

StatusResponse DefaultDriver::get_status(const rclcpp::Time&) const
{
  std::vector<uint8_t> in;
  in.emplace_back(kMotorStatusQueryId);
  RCLCPP_DEBUG(kLogger, "Status query: %s", to_hex(in).c_str());
  cobs_serial_->write(in);
  auto out = cobs_serial_->read();
  RCLCPP_DEBUG(kLogger, "Status response: %s", to_hex(out).c_str());

  // The response contains the following information.
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

InfoResponse DefaultDriver::get_info(const rclcpp::Time&) const
{
  std::vector<uint8_t> in;
  in.emplace_back(kInfoQueryId);
  RCLCPP_DEBUG(kLogger, "Info query: %s", to_hex(in).c_str());
  cobs_serial_->write(in);
  auto out = cobs_serial_->read();
  RCLCPP_DEBUG(kLogger, "Info response: %s", to_hex(out).c_str());

  // The data array contains the following information.
  //
  // status               - 1 byte
  // version              - 3 bytes: major, minor and patch
  // motor count          - 1 byte
  // for each motor:
  //     motor ID             - 1 byte
  //     max acceleration     - 4 bytes
  //     max velocity         - 4 bytes
  // a variable number of bytes representing a string in ASCII format - N bytes

  constexpr std::size_t kBytesPerMotor = 9;

  if (out.size() < 5)
  {
    throw std::runtime_error("Info response is too short.");
  }

  std::size_t i = 0;
  Response::Status status{ out[i++] };
  const Version version{ out[i], out[i + 1], out[i + 2] };
  i += 3;
  const std::size_t motor_count = out[i++];

  if (out.size() < i + motor_count * kBytesPerMotor)
  {
    throw std::runtime_error("Info response does not contain the limits of every motor.");
  }

  std::vector<MotorLimits> limits;
  for (std::size_t motor = 0; motor < motor_count; ++motor)
  {
    uint8_t id = out[i++];
    double max_acceleration = to_float({ out[i], out[i + 1], out[i + 2], out[i + 3] });
    i += 4;
    double max_velocity = to_float({ out[i], out[i + 1], out[i + 2], out[i + 3] });
    i += 4;
    limits.emplace_back(MotorLimits{ id, max_acceleration, max_velocity });
  }

  std::string info{ out.begin() + static_cast<std::ptrdiff_t>(i), out.end() };
  InfoResponse response{ status, info, version, limits };
  return response;
}
}  // namespace stepit_driver
