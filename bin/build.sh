#!/bin/bash -e

# Source the ROS 2 environment. This is safe to do even if it was already
# sourced (e.g. by an interactive shell's .bashrc). Note ROS_DISTRO cannot be
# used to detect whether sourcing already happened: base ROS docker images
# export it unconditionally, whether or not setup.bash has been sourced.
source /opt/ros/jazzy/setup.bash

colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug -DCMAKE_EXPORT_COMPILE_COMMANDS=ON --symlink-install --event-handlers log-
