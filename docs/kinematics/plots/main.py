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
import numpy as np

# This is a library that has been used to figure out trajectory profiles of
# motors given initial position x0, initial velocity v0, acceleration a and
# final position x1.
# Trajectory profiles are then plotted to validate the result.


def sgn(a):
    return bool(a > 0) - bool(a < 0)


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


def time_to_go(v_max: float, a: float, v0: float, x0: float, x1: float):
    dx = abs(x1 - x0)
    dt = v0 / a
    total_time = 0
    if dx >= pow(v_max, 2) / a:
        t1 = v_max / a
        t2 = t1 + max(0, (dx - pow(v_max, 2) / a)) / v_max
        t3 = t1 + t2
        total_time = t3 - dt
    elif dx >= 0:
        t1 = math.sqrt(dx / a)
        t2 = 2 * t1
        total_time = t2 - dt
    return total_time


def velocity_v(v_max: float, a: float, v0: float, x0: float, x1: float, t: list[float]):
    v = [0.0] * len(t)
    for i in range(0, len(t)):
        v[i] = velocity(v_max, a, v0, x0, x1, t[i])
    return v


def velocity_0(v_max: float, a: float, x0: float, x1: float, t: float):
    """
    Velocity at time t of a motor with acceleration a and zero initial velocity, moving from position x0 to position x1.
    :param v_max: The maximum velocity.
    :param a:     The acceleration.
    :param x0:    The initial position.
    :param x1:    The final position.
    :param t:     The time.
    :return:      The motor velocity.
    """
    dx = x1 - x0
    v = 0
    if abs(dx) >= pow(v_max, 2) / a:
        t1 = v_max / a
        t2 = t1 + max(0, (abs(dx) - pow(v_max, 2) / a)) / v_max
        t3 = t1 + t2
        if t <= t1:
            v = a * t
        elif t <= t2:
            v = a * t1
        elif t <= t3:
            v = a * t1 - a * (t - t2)
        else:
            v = 0
    elif abs(dx) >= 0:
        t1 = math.sqrt(abs(dx) / a)
        t2 = 2 * t1
        if t <= t1:
            v = a * t
        elif t <= t2:
            v = a * t1 - a * (t - t1)
        else:
            v = 0
    return v * sgn(dx)


def velocity(v_max: float, a: float, v0: float, x0: float, x1: float, t: float):
    """
    Velocity at time t of a motor with acceleration a and initial velocity v0, moving from position x0 to position x1.
    :param v_max: The maximum velocity.
    :param a:     The acceleration.
    :param v0:    The initial velocity.
    :param x0:    The initial position.
    :param x1:    The final position.
    :param t:     The time.
    :return:      The motor velocity.
    """

    dx = x1 - x0
    d_stop = distance_to_stop(a, v0)
    t_stop = time_to_stop(a, v0)
    v = 0
    if (v0 >= 0 and dx >= d_stop) or (v0 <= 0 and dx <= -d_stop):  # Keep accelerating.
        v = velocity_0(v_max, a, x0, x1 + sgn(v0) * d_stop, t + t_stop)
    else:
        if t <= t_stop:  # Decelerate.
            v = v0 - sgn(v0) * a * t
        else:  # Reverse.
            v = velocity_0(v_max, a, x0, x1 - sgn(v0) * d_stop, t - t_stop)
    return v


# def position(v_max: float, a: float, v0: float, x0: float, x1: float, t: float):
#
#     dx = abs(x1 - x0)
#     sign_dx = sgn(x1 - x0)
#     d = 0
#
#
#     a = 1
#     # time_to_stop = 0.5 * pow(v0, 2) / a
#
#     v0 = 1 # distance_to_stop = 0.5
#     x0 = 1
#     x1 = 2 # dx = 1 -> I must accelerate in the positive direction.
#
#     v0 = -1 # distance_to_stop = 0.5
#     x0 = 1
#     x1 = 2 # dx = 1
#
#     v0 = -1 # distance_to_stop = 0.5
#     x0 = -1
#     x1 = -2 # dx = 1
#
#     v0 = 1 # distance_to_stop = 0.5
#     x0 = -1
#     x1 = -2 # dx = -1
#
#
#     if v0 >= 0:
#         distance_to_stop = 0.5 * pow(v0, 2) / a
#     else:
#
#     tt = t + time_to_stop(a, v0)
#     if dx >= pow(v_max, 2) / a:
#         t1 = v_max / a
#         t2 = t1 + max(0, (dx - pow(v_max, 2) / a)) / v_max
#         t3 = t1 + t2
#         if tt <= t1:
#             d = 0.5 * a * pow(tt, 2)
#         elif tt <= t2:
#             d = 0.5 * pow(v_max, 2) / a + v_max * (tt - t1)
#         elif tt <= t3:
#             d = 0.5 * pow(v_max, 2) / a + v_max * (tt - t1) - 0.5 * a * pow(tt - t2, 2)
#         else:
#             d = pow(v_max, 2) / a + v_max * (t2 - t1)
#     elif dx >= 0:
#         t1 = math.sqrt(dx / a)
#         t2 = 2 * t1
#         if tt <= t1:
#             d = 0.5 * a * pow(tt, 2)
#         elif tt <= t2:
#             d = 0.5 * dx + a * t1 * (tt - t1) - 0.5 * a * pow(tt - t1, 2)
#         else:
#             d = 0.5 * dx + 0.5 * a * pow(t1, 2)
#     return sign_dx * (d - 0.5 * pow(v0, 2) / a)


def plot():

    v0 = 0
    a = 1
    v_max = 1
    x0 = 0
    x1 = 0
    t_max = 5

    t = np.linspace(0, t_max, 200)
    fig, ax = plt.subplots(figsize=(12, 6), dpi=200)

    ####################################################################################################################
    # PLOT 1

    plt.subplot(1, 2, 1)
    plt.xlabel("time [s]")
    plt.ylabel("velocity [rad/s]")
    plt.ylim(-v_max * 1.01, v_max * 1.01)
    plt.xlim(0.0, t_max)

    # adjust the main plot to make room for the sliders
    plt.subplots_adjust(left=0.18, bottom=0.25)

    (velocity_line,) = plt.plot(
        t, velocity_v(v_max=v_max, a=a, v0=v0, x0=x0, x1=x1, t=t), lw=2, color="red"
    )

    ####################################################################################################################
    # PLOT 2

    plt.subplot(1, 2, 2)
    plt.xlabel("time [s]")
    plt.ylabel("position [rad]")
    plt.ylim(-v_max * 1.01, v_max * 1.01)
    plt.xlim(0.0, t_max)

    (position_line,) = plt.plot(
        t, velocity_v(v_max=v_max, a=a, v0=v0, x0=x0, x1=x1, t=t), lw=2, color="blue"
    )

    ####################################################################################################################
    # SLIDERS AND BUTTONS

    # Make a horizontal slider to control the final coordinate.
    ax_x1 = plt.axes([0.18, 0.1, 0.33, 0.03])
    x1_slider = plt.Slider(
        ax=ax_x1,
        label="$x_1$",
        valmin=-5,
        valmax=5,
        valinit=0,
    )

    # Make a vertically oriented slider to control the initial velocity
    v0_slider = plt.Slider(
        ax=plt.axes([0.07, 0.25, 0.015, 0.63]),
        label="$v_0$",
        valmin=-v_max,
        valmax=v_max,
        valinit=v0,
        orientation="vertical",
    )

    # Make a vertically oriented slider to control the initial velocity
    a_slider = plt.Slider(
        ax=plt.axes([0.018, 0.25, 0.015, 0.63]),
        label="$a$",
        valmin=0.1,
        valmax=1,
        valinit=a,
        orientation="vertical",
    )

    # Make a reset button.
    button = plt.Button(plt.axes([0.8, 0.025, 0.1, 0.04]), "Reset", hovercolor="0.975")

    # The function to be called anytime a slider's value changes.
    def update(val):
        velocity_line.set_ydata(
            velocity_v(
                v_max=v_max,
                a=a_slider.val,
                v0=v0_slider.val,
                x0=x0,
                x1=x1_slider.val,
                t=t,
            )
        )
        position_line.set_ydata(
            velocity_v(
                v_max=v_max,
                a=a_slider.val,
                v0=v0_slider.val,
                x0=x0,
                x1=x1_slider.val,
                t=t,
            )
        )
        fig.canvas.draw_idle()

    # The function to be called to reset the sliders.
    def reset(event):
        v0_slider.reset()
        x1_slider.reset()

    # Events.
    a_slider.on_changed(update)
    v0_slider.on_changed(update)
    x1_slider.on_changed(update)
    button.on_clicked(reset)

    plt.show()


if __name__ == "__main__":
    plot()
