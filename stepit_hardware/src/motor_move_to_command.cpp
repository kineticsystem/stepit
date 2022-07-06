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

#include <stepit_hardware/motor_move_to_command.h>

#include <stepit_hardware/data_utils.h>

namespace stepit_hardware
{
constexpr uint8_t kCommandId = 0x71;

void MotorMoveToCommand::create(uint8_t motor_id, int32_t position)
{
}

MotorMoveToCommand::MotorMoveToCommand(uint8_t motor_id, int32_t position)
  : motor_id_{ motor_id }, position_{ position }
{
}

void MotorMoveToCommand::write(DataBuffer& buffer)
{
  buffer.add_int8(kCommandId, BufferPosition::Tail);
  buffer.add_int8(motor_id_, BufferPosition::Tail);
  buffer.add_int16(motor_id_, BufferPosition::Tail);
}
}  // namespace stepit_hardware
