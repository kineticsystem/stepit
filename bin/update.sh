#!/bin/bash -e

# Source ROS 2 environment if not already sourced.
if [ -z "$ROS_DISTRO" ]; then
    source /opt/ros/jazzy/setup.bash
fi

# Initialize rosdep if not already done.
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rosdep init
fi

rosdep update
rosdep install --ignore-src --from-paths . -y -r
