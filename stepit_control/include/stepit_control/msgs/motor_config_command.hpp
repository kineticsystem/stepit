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

#include <stepit_control/msgs/request.hpp>

#include <vector>

namespace stepit_control
{
/**
 * @brief Command to configure a group of motors.
 */
class MotorConfigCommand : public Request
{
public:
  class Param
  {
  public:
    explicit Param(uint8_t motor_id, double acceleration, double max_velocity)
      : motor_id_{ motor_id }, acceleration_{ acceleration }, max_velocity_{ max_velocity } {};
    uint8_t motor_id() const
    {
      return motor_id_;
    }
    double acceleration() const
    {
      return acceleration_;
    }
    double max_velocity() const
    {
      return max_velocity_;
    }

  private:
    uint8_t motor_id_;
    double acceleration_;
    double max_velocity_;
  };

  explicit MotorConfigCommand(uint8_t request_id, const std::vector<Param>& params);
  uint8_t command_id() const;
  std::vector<Param> params() const;

private:
  std::vector<Param> params_;
};
}  // namespace stepit_control
