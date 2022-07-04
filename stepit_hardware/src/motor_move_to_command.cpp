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

#include "motor_move_to_command.hpp"

constexpr uint8_t kCommandId = 0x71;

MotorMoveToCommand::MotorMoveToCommand(uint8_t motor_id, uint32_t position)
{
  uint32_t encoded_position = 0;

  bytes_.emplace_back(kCommandId);
  bytes_.emplace_back(motor_id);
  // bytes_.emplace_back(encoded_position);
}
