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

#include <stepit_hardware/msgs/motor_move_to_command.hpp>

#include <stepit_hardware/data_utils.hpp>

namespace stepit_hardware
{
constexpr uint8_t kCommandId = 0x71;

MotorMoveToCommand::MotorMoveToCommand(uint8_t motor_id, int32_t position)
  : motor_id_{ motor_id }, position_{ position }
{
  auto encoded_motor_id = motor_id;
  auto encoded_position = stepit_hardware::data_utils::from_int32(position);

  bytes_.emplace_back(kCommandId);
  bytes_.emplace_back(encoded_motor_id);
  for (const uint8_t& byte : encoded_position)
  {
    bytes_.emplace_back(byte);
  }
}
}  // namespace stepit_hardware