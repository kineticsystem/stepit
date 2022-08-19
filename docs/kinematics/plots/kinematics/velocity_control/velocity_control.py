#!/usr/bin/env python3

# Copyright (c) 2022, Giovanni Remigi
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


import math

# Exported functions.
__all__ = ["time_to_go", "position", "velocity"]


def sgn(number: float):
    """
    Return the sign of the given number.
    :param number: The number to calculate the sign of.
    :return: The sign of the given number.
    """
    return bool(number > 0) - bool(number < 0)


def time_to_stop(a: float, v0: float):
    """
    Return the time to stop for a motor with acceleration a and rotating speed v0.
    :param a:  The motor acceleration.
    :param v0: The current speed.
    :return:   The time for the motor to stop.
    """
    return abs(v0 / a)


def distance_to_stop(a: float, v0: float):
    """
    Return the absolute distance to stop for a motor with acceleration a and rotating speed v0.
    :param a:  The motor acceleration.
    :param v0: The current speed.
    :return:   The absolute distance for the motor to stop.
    """
    return 0.5 * pow(v0, 2) / a


def _time_to_go(v_max: float, a: float, x0: float, x1: float):
    """
    Return the time for a motor, with acceleration a and zero initial velocity, to move from position x0 to position x1.
    :param v_max: The maximum velocity.
    :param a:     The acceleration.
    :param x0:    The initial position.
    :param x1:    The final position.
    :return:      The motor velocity.
    """
    dx = x1 - x0
    total_time = 0
    if abs(dx) >= pow(v_max, 2) / a:
        t1 = v_max / a
        t2 = t1 + max(0, (abs(dx) - pow(v_max, 2) / a)) / v_max
        total_time = t1 + t2
    elif abs(dx) >= 0:
        t1 = math.sqrt(abs(dx) / a)
        total_time = 2 * t1
    return total_time


def time_to_go(v_max: float, a: float, v0: float, x0: float, x1: float):
    """
    Return the time for a motor, with acceleration a and initial velocity v0, to move from position x0 to position x1.
    :param v_max: The maximum velocity.
    :param a:     The acceleration.
    :param v0:    The initial velocity.
    :param x0:    The initial position.
    :param x1:    The final position.
    :return:      The motor velocity.
    """
    dx = x1 - x0
    d_stop = distance_to_stop(a, v0)
    t_stop = time_to_stop(a, v0)
    total_time = 0
    if (v0 >= 0 and dx >= d_stop) or (v0 <= 0 and dx <= -d_stop):  # Keep accelerating.
        total_time = _time_to_go(v_max, a, x0, x1 + sgn(v0) * d_stop) - t_stop
    elif abs(dx) >= 0:  # Decelerate and reverse.
        total_time = _time_to_go(v_max, a, x0, x1 - sgn(v0) * d_stop) + t_stop
    return total_time


def _position(v_max: float, a: float, x0: float, x1: float, t: float):
    """
    Velocity at time t of a motor with acceleration a and zero initial velocity, moving from position x0 to x1.
    :param v_max: The maximum velocity.
    :param a:     The acceleration.
    :param x0:    The initial position.
    :param x1:    The final position.
    :param t:     The time.
    :return:      The motor position.
    """
    dx = x1 - x0
    x = 0
    if abs(dx) >= pow(v_max, 2) / a:
        t1 = v_max / a
        t2 = t1 + max(0, (abs(dx) - pow(v_max, 2) / a)) / v_max
        t3 = t1 + t2
        if t <= t1:  # v = a * t
            x = 0.5 * a * pow(t, 2)
        elif t <= t2:  # v = a * t1
            x = 0.5 * pow(v_max, 2) / a + v_max * (t - t1)
        elif t <= t3:  # v = a * t1 - a * (t - t2)
            x = 0.5 * pow(v_max, 2) / a + v_max * (t - t1) - 0.5 * a * pow(t - t2, 2)
        else:
            x = pow(v_max, 2) / a + v_max * (t2 - t1)
    elif abs(dx) >= 0:
        t1 = math.sqrt(abs(dx) / a)
        t2 = 2 * t1
        if t <= t1:
            x = 0.5 * a * pow(t, 2)
        elif t <= t2:
            x = 0.5 * abs(dx) + a * t1 * (t - t1) - 0.5 * a * pow(t - t1, 2)
        else:
            x = abs(dx)
    return x * sgn(dx)


def position(v_max: float, a: float, v0: float, x0: float, x1: float, t: float):
    """
    Velocity at time t of a motor with acceleration a and initial velocity v0, moving from position x0 to x1.
    :param v_max: The maximum velocity.
    :param a:     The acceleration.
    :param v0:    The initial velocity.
    :param x0:    The initial position.
    :param x1:    The final position.
    :param t:     The time.
    :return:      The motor position.
    """

    dx = x1 - x0
    d_stop = distance_to_stop(a, v0)
    t_stop = time_to_stop(a, v0)
    if (v0 >= 0 and dx >= d_stop) or (v0 <= 0 and dx <= -d_stop):  # Keep accelerating.
        x = (
            _position(v_max, a, x0, x1 + sgn(v0) * d_stop, t + t_stop)
            - sgn(v0) * d_stop
        )
    else:
        if t <= t_stop:  # Decelerate.
            x = v0 * (t - t_stop) + sgn(v0) * (
                0.5 * a * (pow(t_stop, 2) - pow(t, 2)) + d_stop
            )
        else:  # Reverse.
            x = (
                _position(v_max, a, x0, x1 - sgn(v0) * d_stop, t - t_stop)
                + sgn(v0) * d_stop
            )
    return x + x0


def _velocity(v_max: float, a: float, v1: float, t: float):
    """
    Velocity at time t of a motor with acceleration a and zero initial velocity accelerating to velocity v1.
    :param v_max: The maximum velocity.
    :param a:     The acceleration.
    :param v1:    The final velocity.
    :param t:     The time.
    :return:      The motor velocity.
    """
    t_stop = time_to_stop(a, v1)
    if t <= t_stop:
        v = a * t * sgn(v1)
    else:
        v = v1
    return v


def velocity(v_max: float, a: float, v0: float, v1: float, t: float):
    """
    Velocity at time t of a motor with acceleration a and initial velocity v0, moving from position x0 to x1.
    :param v_max: The maximum velocity.
    :param a:     The acceleration.
    :param v0:    The initial velocity.
    :param v1:    The final velocity.
    :param t:     The time.
    :return:      The motor velocity.
    """
    d_stop = distance_to_stop(a, v0)
    t_stop = time_to_stop(a, v0)
    if (0 < v0 <= v1) or (0 > v0 >= v1):  # Keep accelerating.
        v = _velocity(v_max, a, v1, t + t_stop)
    else:
        if t <= t_stop:  # Decelerate.
            v = v0 - sgn(v0) * a * t
        else:  # Reverse.
            v = _velocity(v_max, a, v1, t - t_stop)
    return v
