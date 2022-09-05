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

#include <stepit_control/msgs/response.hpp>

#include <vector>
#include <cstdint>

namespace stepit_control
{

class MotorStatusResponse : public Response
{
public:
  // Internal structure to store joint states and targets.
  class MotorState
  {
  public:
    explicit MotorState(uint8_t id, double position, double velocity, double distance_to_go)
      : id_{ id }, position_{ position }, velocity_{ velocity }, distance_to_go_{ distance_to_go }
    {
    }
    uint8_t id()
    {
      return id_;
    }
    double position()
    {
      return position_;
    }
    double velocity()
    {
      return velocity_;
    }
    double distance_to_go()
    {
      return distance_to_go_;
    }

  private:
    uint8_t id_;
    double position_;
    double velocity_;
    double distance_to_go_;
  };

  explicit MotorStatusResponse(uint8_t request_id, Status status, std::vector<MotorState> motor_states);
  std::vector<MotorState> motor_states() const;

private:
  std::vector<MotorState> motor_states_;
};
}  // namespace stepit_control
