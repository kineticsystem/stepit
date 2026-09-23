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

colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug -DCMAKE_EXPORT_COMPILE_COMMANDS=ON --symlink-install --event-handlers log-

# Hide the build artifacts from rosdep. colcon already writes a COLCON_IGNORE
# into build/, but rosdep scans with rospkg, which only knows CATKIN_IGNORE:
# without it, running the CI locally with Nektos act fails, because rosdep
# walks into install/ and follows the dangling symbolic links that
# --symlink-install leaves behind. See "How to run GitHub Actions locally" in
# README.md.
for directory in build install log; do
    if [ -d "${directory}" ]; then
        touch "${directory}/CATKIN_IGNORE"
    fi
done
