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
