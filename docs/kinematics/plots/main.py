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

import matplotlib.pyplot as plt
import math

# This is a library that has been used to figure out trajectory profiles of
# motors given initial position x0, initial velocity v0, acceleration a and
# final position x1.
# Trajectory profiles are then plotted to validate the result.


def time_to_go(v_max: float, a: float, v0: float, x0: float, x1: float):
    dx = x1 - x0
    if dx >= pow(v_max, 2) / a:
        t1 = v_max / a
        t2 = t1 + max(0, (dx - pow(v_max, 2) / a)) / v_max
        t3 = t1 + t2
        total_time = t3
    elif dx >= 0:
        t1 = math.sqrt(dx / a)
        t2 = 2 * t1
        total_time = t2
    return total_time - v0 / a


def velocity(v_max: float, a: float, v0: float, x0: float, x1: float, t: float):
    dx = x1 - x0
    v = 0
    tt = t + v0 / a
    if dx >= pow(v_max, 2) / a:
        t1 = v_max / a
        t2 = t1 + max(0, (dx - pow(v_max, 2) / a)) / v_max
        t3 = t1 + t2
        if tt <= t1:
            v = a * tt
        elif tt <= t2:
            v = a * t1
        elif tt <= t3:
            v = a * t1 - a * (tt - t2)
        else:
            v = 0
    elif dx >= 0:
        t1 = math.sqrt(dx / a)
        t2 = 2 * t1
        if tt <= t1:
            v = a * tt
        elif tt <= t2:
            v = a * t1 - a * (tt - t1)
        else:
            v = 0
    return v


def position(v_max: float, a: float, v0: float, x0: float, x1: float, t: float):
    dx = x1 - x0
    d = 0
    tt = t + v0 / a
    if dx >= pow(v_max, 2) / a:
        t1 = v_max / a
        t2 = t1 + max(0, (dx - pow(v_max, 2) / a)) / v_max
        t3 = t1 + t2
        if tt <= t1:
            d = 0.5 * a * pow(tt, 2)
        elif tt <= t2:
            d = 0.5 * pow(v_max, 2) / a + v_max * (tt - t1)
        elif tt <= t3:
            d = 0.5 * pow(v_max, 2) / a + v_max * (tt - t1) - 0.5 * a * pow(tt - t2, 2)
        else:
            d = pow(v_max, 2) / a + v_max * (t2 - t1)
    elif dx >= 0:
        t1 = math.sqrt(dx / a)
        t2 = 2 * t1
        if tt <= t1:
            d = 0.5 * a * pow(tt, 2)
        elif tt <= t2:
            d = 0.5 * dx + a * t1 * (tt - t1) - 0.5 * a * pow(tt - t1, 2)
        else:
            d = 0.5 * dx + 0.5 * a * pow(t1, 2)
    return d


def plot():

    n = 200
    t_v = [0.0] * n
    d_v = [0.0] * n
    v_v = [0.0] * n

    v0 = 0.5
    a = 0.7
    v_max = 1.1
    x0 = 0

    plt.figure(figsize=(16, 6), dpi=200)

    plt.subplot(1, 2, 1)
    for x1 in [4.5, 4, 3.5, 3, 2.5, 2, 1.5, 1, 0.5]:
        total_time = time_to_go(v_max=v_max, a=a, v0=v0, x0=x0, x1=x1)
        for i in range(n):
            t = i * total_time / (n - 1)
            d = position(v_max=v_max, a=a, v0=v0, x0=x0, x1=x1, t=t)
            t_v[i] = t
            d_v[i] = d
        # plt.plot(x, y, "red")
        plt.plot(t_v, d_v)
    plt.title("Position")
    plt.xlabel("t")
    plt.ylabel("x")
    plt.xlim(-1)
    plt.ylim(-1)
    plt.axhline(y=0, color="black", linestyle="--", linewidth=0.5)
    plt.axvline(x=0, color="black", linestyle="--", linewidth=0.5)
    plt.plot(0, position(v_max=v_max, a=a, v0=v0, x0=x0, x1=x1, t=0), "k.")
    plt.subplot(1, 2, 2)

    for x1 in [4.5, 4, 3.5, 3, 2.5, 2, 1.5, 1, 0.5]:
        total_time = time_to_go(v_max=v_max, a=a, v0=v0, x0=x0, x1=x1)
        for i in range(n):
            t = i * total_time / (n - 1)
            v = velocity(v_max=v_max, a=a, v0=v0, x0=x0, x1=x1, t=t)
            t_v[i] = t
            v_v[i] = v
        # plt.plot(x, y, "red")
        plt.plot(t_v, v_v)
    plt.title("Velocity")
    plt.xlabel("t")
    plt.ylabel("v")
    plt.xlim(-1)
    plt.ylim(-1)
    plt.axhline(y=0, color="black", linestyle="--", linewidth=0.5)
    plt.axvline(x=0, color="black", linestyle="--", linewidth=0.5)
    plt.plot(0, velocity(v_max=v_max, a=a, v0=v0, x0=x0, x1=x1, t=0), "k.")
    # plt.text(xmax, ymax + 2, 'local max:' + str(ymax))

    plt.show()


if __name__ == "__main__":
    plot()
