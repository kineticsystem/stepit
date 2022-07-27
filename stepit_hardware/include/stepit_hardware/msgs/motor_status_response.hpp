/*
 * Copyright (C) 2022 Remigi Giovanni
 * g.remigi@kineticsystem.org
 * www.kineticsystem.org
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU Lesser General Public License as published by the Free
 * Software Foundation; either version 2 of the License, or (at your option) any
 * later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License for more
 * details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program; if not, write to the Free Software Foundation, Inc.,
 * 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#pragma once

#include <stepit_hardware/msgs/response.hpp>

#include <vector>
#include <cstdint>

namespace stepit_hardware
{

class MotorStatusResponse : public Response
{
public:
  // Internal structure to store joint states and targets.
  class MotorState
  {
  public:
    explicit MotorState(uint8_t id, int32_t position, float velocity, int32_t distance_to_go)
      : id_{ id }, position_{ position }, velocity_{ velocity }, distance_to_go_{ distance_to_go }
    {
    }
    uint8_t id()
    {
      return id_;
    }
    int32_t position()
    {
      return position_;
    }
    double velocity()
    {
      return velocity_;
    }
    int32_t distance_to_go()
    {
      return distance_to_go_;
    }

  private:
    uint8_t id_;
    int32_t position_;
    float velocity_;
    int32_t distance_to_go_;
  };

  explicit MotorStatusResponse(const std::vector<uint8_t>& data);
  std::vector<MotorState> motor_states() const;

private:
  std::vector<MotorState> motor_states_;
};
}  // namespace stepit_hardware
