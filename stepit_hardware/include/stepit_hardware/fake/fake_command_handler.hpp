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

#pragma once

#include <stepit_hardware/command_interface.hpp>
#include <stepit_hardware/fake/fake_motor.hpp>
#include <stepit_hardware/msgs/msgs.hpp>

#include <rclcpp/rclcpp.hpp>

#include <map>

namespace stepit_hardware
{
/**
 * @brief The FakeCommandHandler class receives commands and queries from the
 * hardware interface and sends them to a fake hardware.
 */
class FakeCommandHandler : public CommandInterface
{
public:
  FakeCommandHandler() = default;
  bool connect() override;
  AcknowledgeResponse send(const ConfigCommand& command) const override;
  AcknowledgeResponse send(const rclcpp::Time& time, const PositionCommand& command) const override;
  AcknowledgeResponse send(const rclcpp::Time& time, const VelocityCommand& command) const override;
  StatusResponse send(const rclcpp::Time& time, const StatusQuery& query) const override;
  InfoResponse send(const rclcpp::Time& time, const InfoQuery& query) const override;

private:
  /* Virtual motors behaving like real stepper motors with given acceletation
   * and absolute maximum velocity.
   */
  mutable std::map<uint8_t, FakeMotor> motors_;
};
}  // namespace stepit_hardware
