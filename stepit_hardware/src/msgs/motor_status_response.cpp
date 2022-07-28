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

#include <stepit_hardware/msgs/motor_status_response.hpp>
#include <stepit_hardware/data_utils.hpp>

namespace stepit_hardware
{
MotorStatusResponse::MotorStatusResponse(const std::vector<uint8_t>& data)
{
  /*
   * The data array contains the following information.
   *
   * request id           - 1 byte
   * status               - 1 byte
   *
   * motor id             - 1 byte
   * motor position       - 4 bytes
   * motor speed          - 4 bytes
   * motor distance to go - 4 bytes
   *
   * motor id             - 1 byte
   * motor position       - 4 bytes
   * motor speed          - 4 bytes
   * motor distance to go - 4 bytes
   *
   * ...and so on.
   */

  std::size_t i = 0;
  request_id_ = data[i++];
  status_ = data[i++];
  while (i < data.size())
  {
    uint8_t id = data[i++];
    int32_t position = data_utils::to_int32({ data[i++], data[i++], data[i++], data[i++] });
    float speed = data_utils::to_float({ data[i++], data[i++], data[i++], data[i++] });
    int32_t distance_to_go = data_utils::to_int32({ data[i++], data[i++], data[i++], data[i++] });
    motor_states_.push_back(MotorState{ id, position, speed, distance_to_go });
  }
}

std::vector<MotorStatusResponse::MotorState> MotorStatusResponse::motor_states() const
{
  return motor_states_;
}
}  // namespace stepit_hardware
