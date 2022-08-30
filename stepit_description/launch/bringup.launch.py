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

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

import xacro


def generate_launch_description():
    robot_name = "stepit"
    package_name = robot_name + "_description"
    rviz_config = os.path.join(
        get_package_share_directory(package_name), "rviz", robot_name + ".rviz"
    )
    robot_description = os.path.join(
        get_package_share_directory(package_name), "urdf", robot_name + ".urdf.xacro"
    )
    robot_description_config = xacro.process_file(robot_description)

    controller_config = os.path.join(
        get_package_share_directory(package_name), "controllers", "controllers.yaml"
    )

    return LaunchDescription(
        [
            # The Controller Manager (CM) connects the controllers’ and hardware-abstraction sides of the ros2_control
            # framework. It also serves as the entry-point for users through ROS services.
            # https://control.ros.org/master/doc/getting_started/getting_started.html#architecture
            Node(
                package="controller_manager",
                executable="ros2_control_node",
                parameters=[
                    {"robot_description": robot_description_config.toxml()},
                    controller_config,
                ],
            ),
            # The broadcaster reads all state interfaces and reports them on /joint_states and /dynamic_joint_states.
            # https://control.ros.org/master/doc/ros2_controllers/joint_state_broadcaster/doc/userdoc.html
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "joint_state_broadcaster",
                    "--controller-manager",
                    "/controller_manager",
                ],
            ),
            # This is a controller that work using the “velocity” joint command interface.
            # https://control.ros.org/master/doc/ros2_controllers/velocity_controllers/doc/userdoc.html
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["velocity_controller", "-c", "/controller_manager"],
            ),
            # Controller for executing joint-space trajectories on a group of joints. Trajectories are specified as a
            # set of waypoints to be reached at specific time instants, which the controller attempts to execute as well
            # as the mechanism allows. Waypoints consist of positions, and optionally velocities and accelerations.
            # https://control.ros.org/master/doc/ros2_controllers/joint_trajectory_controller/doc/userdoc.html
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_trajectory_controller", "-c", "/controller_manager"],
            ),
            # robot_state_publisher uses the URDF specified by the parameter robot_description and the joint positions
            # from the topic /joint_states to calculate the forward kinematics of the robot and publish the results via
            # tf.
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                parameters=[{"robot_description": robot_description_config.toxml()}],
                output="screen",
            ),
            # joint_state_publisher_gui reads the robot description from the topic /robot/description and create a GUI
            # displaying a slider for each moving joint. Remember to set non-zero lower and upper limits on each joint,
            # otherwise the slider will not be displayed. By moving the slider we publish joint states to the topic
            # /joint_states which are used by the robot_state_publisher node. A non-graphical tool to do the same is the
            # joint_state_publisher node.
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                name="joint_state_publisher_gui",
            ),
            # RViz2 starts up the RViz2 application.
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", rviz_config],
            ),
        ]
    )
