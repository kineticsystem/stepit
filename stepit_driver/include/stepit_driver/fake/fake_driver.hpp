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

#include <map>

#include <stepit_driver/driver.hpp>
#include <stepit_driver/fake/fake_motor.hpp>

#include <rclcpp/rclcpp.hpp>

namespace stepit_driver
{
/**
 * @brief The FakeCommandHandler class receives commands and queries from the
 * hardware interface and sends them to a fake hardware.
 */
class FakeDriver : public Driver
{
public:
  FakeDriver() = default;
  bool connect() override;
  void disconnect() override;
  AcknowledgeResponse configure(const ConfigCommand& command) const override;
  AcknowledgeResponse set_position(const rclcpp::Time& time, const PositionCommand& command) const override;
  AcknowledgeResponse set_velocity(const rclcpp::Time& time, const VelocityCommand& command) const override;
  StatusResponse get_status(const rclcpp::Time& time) const override;
  InfoResponse get_info(const rclcpp::Time& time) const override;

private:
  /* Virtual motors behaving like real stepper motors with given acceletation
   * and absolute maximum velocity.
   */
  mutable std::map<uint8_t, FakeMotor> motors_;
};
}  // namespace stepit_driver
