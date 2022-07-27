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

#include <stepit_hardware/msgs/request.hpp>

#include <vector>

namespace stepit_hardware
{
class MotorVelocityCommand : public Request
{
public:
  class MotorVelocity
  {
  public:
    explicit MotorVelocity(uint8_t id, double velocity) : id_{ id }, velocity_{ velocity } {};
    uint8_t id() const
    {
      return id_;
    }
    double velocity() const
    {
      return velocity_;
    }

  private:
    uint8_t id_;
    double velocity_;
  };

  explicit MotorVelocityCommand(const std::vector<MotorVelocity>& velocities);
};
}  // namespace stepit_hardware
