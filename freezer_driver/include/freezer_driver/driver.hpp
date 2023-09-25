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

#include <freezer_driver/msgs/acknowledge_response.hpp>
#include <freezer_driver/msgs/bitset_command.hpp>
#include <freezer_driver/msgs/shoot_command.hpp>
#include <freezer_driver/msgs/status_response.hpp>

#include <rclcpp/time.hpp>

namespace freezer_driver
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
   * Initialize the command interface.
   */
  virtual bool connect() = 0;

  /**
   * Disconnect the interface..
   */
  virtual void disconnect() = 0;

  /**
   * Send a sequence of bits and delays to be executed atomically on the
   * hardware.
   * @param command The command containing a sequence of bits and delays to
   * be executed.
   */
  virtual AcknowledgeResponse execute(const rclcpp::Time& time, const BitsetCommand& command) = 0;

  /**
   * Set the max connection timeout in seconds.
   * @param connection_timeout The max connection timeout in seconds.
   */
  virtual void set_connection_timeout(std::chrono::duration<double> connection_timeout) = 0;

  /**
   * Get the max connection timeout in seconds.
   * @return The max connection timeout in seconds.
   */
  virtual std::chrono::duration<double> get_connection_timeout() const = 0;
};
}  // namespace freezer_driver
