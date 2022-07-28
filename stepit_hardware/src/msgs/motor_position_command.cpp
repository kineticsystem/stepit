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

#include <stepit_hardware/msgs/motor_position_command.hpp>
#include <stepit_hardware/data_utils.hpp>

namespace stepit_hardware
{
constexpr uint8_t kCommandId = 0x71;

MotorPositionCommand::MotorPositionCommand(uint8_t request_id, const std::vector<Goal>& goals)
{
  bytes_.emplace_back(request_id);
  bytes_.emplace_back(kCommandId);
  for (const auto& goal : goals)
  {
    bytes_.emplace_back(goal.motor_id());
    auto position_bytes = data_utils::from_float(static_cast<float>(goal.position()));
    bytes_.emplace_back(position_bytes[0]);
    bytes_.emplace_back(position_bytes[1]);
    bytes_.emplace_back(position_bytes[2]);
    bytes_.emplace_back(position_bytes[3]);
  }
}
}  // namespace stepit_hardware
