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

#include <stepit_hardware/default_request_interface.hpp>

#include <data_interface/data_utils.hpp>

namespace stepit_hardware
{
constexpr auto kLogger = "DefaultRequestInterface";

DefaultRequestInterface::DefaultRequestInterface(std::unique_ptr<data_interface::DataInterface> data_interface)
  : data_interface_{ std::move(data_interface) }
{
}

bool DefaultRequestInterface::connect()
{
  data_interface_->open();

  // Send a status command multiple times until an answer comes back.

  int trial = 0;
  bool connected = false;
  while (!connected && trial < 5)
  {
    try
    {
      RCLCPP_INFO(rclcpp::get_logger(kLogger), "Connecting...");

      StatusQuery query{};
      StatusResponse response = send(rclcpp::Time{}, query);
      connected = response.status() == Response::Status::Success;

      RCLCPP_INFO(rclcpp::get_logger(kLogger), "Connection established");
    }
    catch (const std::exception& ex)
    {
      RCLCPP_WARN(rclcpp::get_logger(kLogger), "Connection failed");
      trial++;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  return connected;
}

void DefaultRequestInterface::disconnect()
{
  data_interface_->close();
}

AcknowledgeResponse DefaultRequestInterface::send(const ConfigCommand& command) const
{
  std::vector<uint8_t> in;
  in.emplace_back(command.command_id());
  for (const auto& param : command.params())
  {
    in.emplace_back(param.motor_id());
    auto acceleration_bytes = data_interface::from_float(static_cast<float>(param.acceleration()));
    in.emplace_back(acceleration_bytes[0]);
    in.emplace_back(acceleration_bytes[1]);
    in.emplace_back(acceleration_bytes[2]);
    in.emplace_back(acceleration_bytes[3]);
    auto max_velocity_bytes = data_interface::from_float(static_cast<float>(param.max_velocity()));
    in.emplace_back(max_velocity_bytes[0]);
    in.emplace_back(max_velocity_bytes[1]);
    in.emplace_back(max_velocity_bytes[2]);
    in.emplace_back(max_velocity_bytes[3]);
  }
  RCLCPP_DEBUG(rclcpp::get_logger(kLogger), "Config command: %s", data_interface::to_hex(in).c_str());
  data_interface_->write(in);
  std::vector<uint8_t> out = data_interface_->read();
  RCLCPP_DEBUG(rclcpp::get_logger(kLogger), "Config response: %s", data_interface::to_hex(out).c_str());
  Response::Status status{ out[0] };
  AcknowledgeResponse response{ status };
  return response;
}

AcknowledgeResponse DefaultRequestInterface::send([[maybe_unused]] const rclcpp::Time& time,
                                                  const PositionCommand& command) const
{
  std::vector<uint8_t> in;
  in.emplace_back(command.command_id());
  for (const auto& goal : command.goals())
  {
    in.emplace_back(goal.motor_id());
    auto position_bytes = data_interface::from_float(static_cast<float>(goal.position()));
    in.emplace_back(position_bytes[0]);
    in.emplace_back(position_bytes[1]);
    in.emplace_back(position_bytes[2]);
    in.emplace_back(position_bytes[3]);
  }
  RCLCPP_DEBUG(rclcpp::get_logger(kLogger), "Position command: %s", data_interface::to_hex(in).c_str());
  data_interface_->write(in);
  std::vector<uint8_t> out = data_interface_->read();
  RCLCPP_DEBUG(rclcpp::get_logger(kLogger), "Position response: %s", data_interface::to_hex(out).c_str());
  Response::Status status{ out[0] };
  AcknowledgeResponse response{ status };
  return response;
}

AcknowledgeResponse DefaultRequestInterface::send([[maybe_unused]] const rclcpp::Time& time,
                                                  const VelocityCommand& command) const
{
  std::vector<uint8_t> in;
  in.emplace_back(command.command_id());
  for (const auto& goal : command.goals())
  {
    in.emplace_back(goal.motor_id());
    auto velocity_bytes = data_interface::from_float(static_cast<float>(goal.velocity()));
    in.emplace_back(velocity_bytes[0]);
    in.emplace_back(velocity_bytes[1]);
    in.emplace_back(velocity_bytes[2]);
    in.emplace_back(velocity_bytes[3]);
  }
  RCLCPP_DEBUG(rclcpp::get_logger(kLogger), "Velocity command: %s", data_interface::to_hex(in).c_str());
  data_interface_->write(in);
  std::vector<uint8_t> out = data_interface_->read();
  RCLCPP_DEBUG(rclcpp::get_logger(kLogger), "Velocity response: %s", data_interface::to_hex(out).c_str());
  Response::Status status{ out[0] };
  AcknowledgeResponse response{ status };
  return response;
}

StatusResponse DefaultRequestInterface::send([[maybe_unused]] const rclcpp::Time& time, const StatusQuery& query) const
{
  std::vector<uint8_t> in;
  in.emplace_back(query.query_id());
  RCLCPP_DEBUG(rclcpp::get_logger(kLogger), "Status query: %s", data_interface::to_hex(in).c_str());
  data_interface_->write(in);
  auto out = data_interface_->read();
  RCLCPP_DEBUG(rclcpp::get_logger(kLogger), "Status response: %s", data_interface::to_hex(out).c_str());

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
  std::vector<StatusResponse::MotorState> motor_states;
  while (i < out.size())
  {
    uint8_t id = out[i++];
    float position = data_interface::to_float({ out[i++], out[i++], out[i++], out[i++] });
    float speed = data_interface::to_float({ out[i++], out[i++], out[i++], out[i++] });
    float distance_to_go = data_interface::to_float({ out[i++], out[i++], out[i++], out[i++] });
    motor_states.push_back(StatusResponse::MotorState{ id, position, speed, distance_to_go });
  }

  StatusResponse response{ status, motor_states };
  return response;
}

InfoResponse DefaultRequestInterface::send([[maybe_unused]] const rclcpp::Time& time, const InfoQuery& query) const
{
  std::vector<uint8_t> in;
  in.emplace_back(query.query_id());
  RCLCPP_DEBUG(rclcpp::get_logger(kLogger), "Info query: %s", data_interface::to_hex(in).c_str());
  data_interface_->write(in);
  auto out = data_interface_->read();
  RCLCPP_DEBUG(rclcpp::get_logger(kLogger), "Info response: %s", data_interface::to_hex(out).c_str());

  // The data array contains the following information.
  //
  // status               - 1 byte
  // a variable number of bytes representing a string in ASCII format - N bytes

  std::size_t i = 0;
  Response::Status status{ out[i++] };
  std::vector<uint8_t> data;
  while (i < out.size())
  {
    uint8_t ch = out[i++];
    data.push_back(ch);
  }

  std::string info{ data.begin(), data.end() };
  InfoResponse response{ status, info };
  return response;
}

}  // namespace stepit_hardware
