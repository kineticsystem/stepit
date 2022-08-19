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

import kinematics

import matplotlib.pyplot as plt
import numpy as np

# This is a library that has been used to figure out trajectory profiles of
# motors given initial position x0, initial velocity v0, acceleration a and
# final position x1.
# Trajectory profiles are then plotted to validate the result.


def position_v(v_max: float, a: float, v0: float, x0: float, x1: float, t: np.ndarray):
    x = [0.0] * len(t)
    for i in range(0, len(t)):
        x[i] = kinematics.position(v_max, a, v0, x0, x1, t[i])
    return x


def velocity_v(v_max: float, a: float, v0: float, x0: float, x1: float, t: np.ndarray):
    v = [0.0] * len(t)
    for i in range(0, len(t)):
        v[i] = kinematics.velocity(v_max, a, v0, x0, x1, t[i])
    return v


def plot():

    x_max = 5

    v0 = 0
    a = 1
    v_max = 1
    x0 = 0
    x1 = 0
    t_max = kinematics.time_to_go(v_max, a, -v_max, x0, x0 + x_max)

    t = np.linspace(0, t_max, 200)
    fig, ax = plt.subplots(figsize=(12, 6), dpi=200)

    ####################################################################################################################
    # PLOT 1

    plt.subplot(1, 2, 1)
    plt.xlabel("time [s]")
    plt.ylabel("velocity [rad/s]")
    plt.ylim(-v_max * 1.01, v_max * 1.01)
    plt.xlim(0.0, t_max)

    line1 = plt.axvline(x=t_max, lw=0.5, color="black", linestyle="dashed")

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
    plt.ylim(-x_max * 1.01, x_max * 1.01)
    plt.xlim(0.0, t_max)

    line2 = plt.axvline(x=t_max, lw=0.5, color="black", linestyle="dashed")

    (position_line,) = plt.plot(
        t, position_v(v_max=v_max, a=a, v0=v0, x0=x0, x1=x1, t=t), lw=2, color="blue"
    )

    ####################################################################################################################
    # SLIDERS AND BUTTONS

    # Make a horizontal slider to control x0 coordinate.
    ax_x0 = plt.axes([0.18, 0.1, 0.33, 0.03])
    x0_slider = plt.Slider(
        ax=ax_x0,
        label="$x_0$",
        valmin=-x_max,
        valmax=x_max,
        valinit=0,
    )

    # Make a horizontal slider to control x1 coordinate.
    ax_x1 = plt.axes([0.18, 0.05, 0.33, 0.03])
    x1_slider = plt.Slider(
        ax=ax_x1,
        label="$x_1$",
        valmin=-x_max,
        valmax=x_max,
        valinit=0,
    )

    # Make a vertically oriented slider to control the initial velocity
    v0_slider = plt.Slider(
        ax=plt.axes([0.055, 0.25, 0.015, 0.63]),
        label="$v_0$",
        valmin=-v_max,
        valmax=v_max,
        valinit=v0,
        orientation="vertical",
    )

    # Make a vertically oriented slider to control the initial velocity
    a_slider = plt.Slider(
        ax=plt.axes([0.028, 0.25, 0.015, 0.63]),
        label="$a$",
        valmin=0.1,
        valmax=1,
        valinit=a,
        orientation="vertical",
    )

    # Make a reset button.
    button = plt.Button(plt.axes([0.8, 0.025, 0.1, 0.04]), "Reset", hovercolor="0.975")

    # The function to be called anytime a slider's value changes.
    def update(_):
        nonlocal line1
        nonlocal line2
        nonlocal t_max

        velocity_line.set_ydata(
            velocity_v(
                v_max=v_max,
                a=a_slider.val,
                v0=v0_slider.val,
                x0=x0_slider.val,
                x1=x1_slider.val,
                t=t,
            )
        )
        position_line.set_ydata(
            position_v(
                v_max=v_max,
                a=a_slider.val,
                v0=v0_slider.val,
                x0=x0_slider.val,
                x1=x1_slider.val,
                t=t,
            )
        )

        t_max = kinematics.time_to_go(
            v_max=v_max,
            a=a_slider.val,
            v0=v0_slider.val,
            x0=x0_slider.val,
            x1=x1_slider.val,
        )

        plt.subplot(1, 2, 1)
        line1.remove()
        line1 = plt.axvline(x=t_max, lw=0.5, color="black", linestyle="dashed")

        plt.subplot(1, 2, 2)
        line2.remove()
        line2 = plt.axvline(x=t_max, lw=0.5, color="black", linestyle="dashed")

        fig.canvas.draw_idle()

    # The function to be called to reset the sliders.
    def reset(_):
        v0_slider.reset()
        x0_slider.reset()
        x1_slider.reset()

    # Events.
    a_slider.on_changed(update)
    v0_slider.on_changed(update)
    x0_slider.on_changed(update)
    x1_slider.on_changed(update)
    button.on_clicked(reset)

    plt.show()


if __name__ == "__main__":
    plot()
