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

#include <stepit_driver/request.h>

#include <stepit_driver/data_buffer.h>

#include <cstdint>

namespace stepit_driver
{
/**
 * Command to move a motor to a given position.
 */
class MotorMoveToCommand : public Request
{
public:
  /**
   * Command to move a motor to a given position.
   * @param motor_id The motor id.
   * @param position The target position relative to the motor zero position,
   * positive or negative.
   */
  explicit MotorMoveToCommand(uint8_t motor_id, int32_t position);

private:
  uint8_t motor_id_;
  int32_t position_;
};
}  // namespace stepit_driver
