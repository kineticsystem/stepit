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

#pragma once

namespace stepit_hardware::velocity_control
{
/**
 * @brief Compute the position of a motor with acceleration a while changing
 * velocity from v0 to v1.
 * @param v_max The maximum velocity.
 * @param a     The acceleration.
 * @param v0    The initial velocity.
 * @param v1    The initial velocity.
 * @param x0    The initial position.
 * @param t     The time.
 * @return      The motor position.
 */
float position(float v_max, float a, float x0, float v0, float v1, float t);

/**
 * @brief Compute the velocity of a motor with acceleration a while changing
 * velocity from v0 to v1.
 * @param v_max The maximum velocity.
 * @param a     The acceleration.
 * @param v0    The initial velocity.
 * @param v1    The final velocity.
 * @param t     The time.
 * @return      The motor velocity.
 */
float velocity(float v_max, float a, float v0, float v1, float t);

}  // namespace stepit_hardware::velocity_control
