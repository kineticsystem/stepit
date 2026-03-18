#!/bin/bash -e

# Source ROS 2 environment if not already sourced.
if [ -z "$ROS_DISTRO" ]; then
    source /opt/ros/jazzy/setup.bash
fi

colcon test --return-code-on-test-failure
colcon test-result --all --verbose
