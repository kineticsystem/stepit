#!/bin/bash -e

# Source ROS 2 environment if not already sourced.
if [ -z "$ROS_DISTRO" ]; then
    source /opt/ros/jazzy/setup.bash
fi

colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug -DCMAKE_EXPORT_COMPILE_COMMANDS=ON --symlink-install --event-handlers log-
