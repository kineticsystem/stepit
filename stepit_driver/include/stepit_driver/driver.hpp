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

#pragma once

#include <stepit_driver/msgs/acknowledge_response.hpp>
#include <stepit_driver/msgs/config_command.hpp>
#include <stepit_driver/msgs/info_response.hpp>
#include <stepit_driver/msgs/position_command.hpp>
#include <stepit_driver/msgs/status_response.hpp>
#include <stepit_driver/msgs/velocity_command.hpp>

#include <rclcpp/time.hpp>

namespace stepit_driver
{
/**
 * @brief This class receives commands and queries from the
 * hardware interface and sends them to a fake or a real hardware.
 */
class Driver
{
public:
  virtual ~Driver() = default;

  /**
   * @brief Initialize the command interface.
   */
  virtual bool connect() = 0;

  /**
   * @brief Disconnect the interface..
   */
  virtual void disconnect() = 0;

  /**
   * @brief Configure the hardware.
   * @param command A command with all required configuration.
   * @return An aknowledgment response, success or failure.
   */
  virtual AcknowledgeResponse configure(const ConfigCommand& command) const = 0;

  /**
   * @brief Send motor target positions (rad) to the hardware.
   * @param time The time the command is sent.
   * @param command A command with all positions (rad).
   * @return An aknowledgment response, success or failure.
   */
  virtual AcknowledgeResponse set_position(const rclcpp::Time& time, const PositionCommand& command) const = 0;

  /**
   * @brief Send motor target velocities (rad/s) to the hardware.
   * @param time The time the command is sent.
   * @param command A command with all motor velocities (rad/s).
   * @return An aknowledgment response, success or failure.
   */
  virtual AcknowledgeResponse set_velocity(const rclcpp::Time& time, const VelocityCommand& command) const = 0;

  /**
   * @brief Request the status of each motor connected to the hardware.
   * @param time The time the command is sent.
   * @return Each motor status and a request status, success or failure.
   */
  virtual StatusResponse get_status(const rclcpp::Time& time) const = 0;

  /**
   * @brief Request information about the software installed in the
   * micro-controller.
   * @param time The time the command is sent.
   * @return Information about the installed software.
   */
  virtual InfoResponse get_info(const rclcpp::Time& time) const = 0;
};
}  // namespace stepit_driver
