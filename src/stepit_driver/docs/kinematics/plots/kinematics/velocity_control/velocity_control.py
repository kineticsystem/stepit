# Copyright 2023 Giovanni Remigi
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the Giovanni Remigi nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


# Exported functions.
__all__ = ["position", "velocity"]


def sgn(number: float):
    """
    Return the sign of the given number.
    :param number: The number to calculate the sign of.
    :return: The sign of the given number.
    """
    return bool(number > 0) - bool(number < 0)


def position(v_max: float, a: float, x0: float, v0: float, v1: float, t: float):
    """
    Position of a motor with acceleration a while changing velocity from v0 to v1.
    :param v_max: The maximum velocity.
    :param a:     The acceleration.
    :param v0:    The initial velocity.
    :param v1:    The initial velocity.
    :param x0:    The initial position.
    :param t:     The time.
    :return:      The motor position.
    """
    v0 = max(-v_max, min(v0, v_max))
    v1 = max(-v_max, min(v1, v_max))
    t1 = abs(v1 - v0) / a
    if t <= t1:
        x = x0 + v0 * t + 0.5 * a * pow(t, 2) * sgn(v1 - v0)
    else:
        x = x0 + (v0 - v1) * t1 + 0.5 * a * pow(t1, 2) * sgn(v1 - v0) + v1 * t
    return x


def velocity(v_max: float, a: float, v0: float, v1: float, t: float):
    """
    Velocity of a motor with acceleration a while changing velocity from v0 to v1.
    :param v_max: The maximum velocity.
    :param a:     The acceleration.
    :param v0:    The initial velocity.
    :param v1:    The final velocity.
    :param t:     The time.
    :return:      The motor velocity.
    """
    v0 = max(-v_max, min(v0, v_max))
    v1 = max(-v_max, min(v1, v_max))
    t1 = abs(v1 - v0) / a
    if t <= t1:
        v = v0 + a * t * sgn(v1 - v0)
    else:
        v = v1
    return v
