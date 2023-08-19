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

namespace stepit_driver::position_control
{

/**
 * @brief Compute the absolute distance to stop for a motor with acceleration a
 * and rotating speed v0.
 * @param a  The motor acceleration (rad/s^2).
 * @param v0 The current speed      (rad/s).
 * @return   The distance to stop   (rad).
 */
double distance_to_stop(double a, double v0);

/**
 * @brief Compute the time for a motor, with acceleration a and initial velocity
 * v0, to move from position x0 to position x1.
 * @param v_max The maximum velocity (rad/s).
 * @param a     The acceleration     (rad/s^2).
 * @param v0    The initial velocity (rad/s).
 * @param x0    The initial position (rad/s).
 * @param x1    The target position  (rad).
 * @return      The time to go       (s).
 */
double time_to_go(double v_max, double a, double v0, double x0, double x1);

/**
 * @brief Compute the position of a motor with acceleration a and initial
 * velocity v0, moving from position x0 to x1.
 * @param v_max The maximum velocity (rad/s).
 * @param a     The acceleration     (rad/s^2).
 * @param v0    The initial velocity (rad/s).
 * @param x0    The initial position (rad).
 * @param x1    The target position  (rad).
 * @param t     The time             (s).
 * @return      The motor position   (rad).
 */
double position(double v_max, double a, double v0, double x0, double x1, double t);

/**
 * @brief Compute the velocity of a motor with acceleration a and initial
 * velocity v0, moving from position x0 to x1.
 * @param v_max The maximum velocity (rad/s).
 * @param a     The acceleration     (rad/s^2).
 * @param v0    The initial velocity (rad/s).
 * @param x0    The initial position (rad).
 * @param x1    The target position  (rad).
 * @param t     The time             (s).
 * @return      The motor velocity   (rad/s).
 */
double velocity(double v_max, double a, double v0, double x0, double x1, double t);

}  // namespace stepit_driver::position_control
