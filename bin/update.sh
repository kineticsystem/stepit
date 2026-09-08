#!/bin/bash -e

# Source the ROS 2 environment. This is safe to do even if it was already
# sourced (e.g. by an interactive shell's .bashrc). Note ROS_DISTRO cannot be
# used to detect whether sourcing already happened: base ROS docker images
# export it unconditionally, whether or not setup.bash has been sourced.
source /opt/ros/jazzy/setup.bash

# colcon and rosdep act on the current working directory, so move to the
# workspace root. This lets the script be called from anywhere, e.g. via the
# build/test/update aliases in the container.
cd "$(dirname "$(readlink -f "$0")")/.."

# Initialize rosdep if not already done.
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rosdep init
fi

rosdep update
rosdep install --ignore-src --from-paths . -y -r
