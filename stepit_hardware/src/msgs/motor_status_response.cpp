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
   * 0:     message id               (1 byte)
   *
   * 1-4:   1st motor position       (4 bytes)
   * 5-8:   1st motor speed          (4 bytes)
   * 9-12:  1st motor distance to go (4 bytes)
   *
   * 13-16: 2nd motor position       (4 bytes)
   * 17-20: 2nd motor speed          (4 bytes)
   * 21-24: 2nd motor distance to go (4 bytes)
   *
   * ...and so on.
   */

  for (std::size_t i = 1; i < data.size(); i += 12)
  {
    int32_t position = data_utils::to_int32({ data[i], data[i + 1], data[i + 2], data[i + 3] });
    float speed = data_utils::to_float({ data[i + 4], data[i + 5], data[i + 6], data[i + 7] });
    int32_t distance_to_go = data_utils::to_int32({ data[i + 8], data[i + 9], data[i + 10], data[i + 11] });
    joints_.push_back(Joint{ position, speed, distance_to_go });
  }
}

std::vector<MotorStatusResponse::Joint> MotorStatusResponse::joints() const
{
  return joints_;
}
}  // namespace stepit_hardware
