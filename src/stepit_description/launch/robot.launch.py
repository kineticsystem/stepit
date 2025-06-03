#!/usr/bin/python3

# Copyright 2023 Giovanni Remigi
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the Giovanni Remigi nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import launch
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import xacro


def launch_setup(context, *args, **kwargs):
    # Declare all parameters.
    launch_rviz = LaunchConfiguration("launch_rviz")

    # Extract all parameters' values.
    description_pkg = FindPackageShare("stepit_description")
    description_file = PathJoinSubstitution(
        [description_pkg, "urdf", "stepit.urdf.xacro"]
    ).perform(context)
    rviz_config_file = PathJoinSubstitution(
        [description_pkg, "rviz", "stepit.rviz"]
    ).perform(context)
    controllers_config_file = PathJoinSubstitution(
        [
            description_pkg,
            "config",
            "controllers.yaml",
        ]
    ).perform(context)

    robot_description_content = xacro.process_file(description_file).toxml()

    # The Controller Manager (CM) connects the controllers’ and
    # hardware-abstraction sides of the ros2_control framework. It also serves
    # as the entry-point for users through ROS services.
    # https://control.ros.org/master/doc/getting_started/getting_started.html#architecture
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description_content},
            controllers_config_file,
        ],
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
    )

    # The broadcaster reads all state interfaces and reports them on
    # /joint_states and /dynamic_joint_states.
    # https://control.ros.org/master/doc/ros2_controllers/joint_state_broadcaster/doc/userdoc.html
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    # This is a controller that work using the “velocity” joint command
    # interface.
    # https://control.ros.org/master/doc/ros2_controllers/velocity_controllers/doc/userdoc.html
    velocity_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["velocity_controller", "-c", "/controller_manager"],
    )

    # This is a controller that work using the “position” joint command
    # interface.
    # https://control.ros.org/master/doc/ros2_controllers/position_controllers/doc/userdoc.html
    position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["position_controller", "-c", "/controller_manager"],
    )

    # Controller for executing joint-space trajectories on a group of joints.
    # Trajectories are specified as a set of waypoints to be reached at
    # specific time instants, which the controller attempts to execute as well
    # as the mechanism allows. Waypoints consist of positions, and optionally
    # velocities and accelerations.
    # https://control.ros.org/master/doc/ros2_controllers/joint_trajectory_controller/doc/userdoc.html
    joint_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "-c", "/controller_manager"],
    )

    # robot_state_publisher uses the URDF specified by the parameter robot
    # description and the joint positions from the topic /joint_states to
    # calculate the forward kinematics of the robot and publish the results via
    # tf.
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[{"robot_description": robot_description_content}],
        output="screen",
    )

    # joint_state_publisher_gui reads the robot description from the topic
    # /robot/description and create a GUI displaying a slider for each moving
    # joint. Remember to set non-zero lower and upper limits on each joint,
    # otherwise the slider will not be displayed. By moving the slider we
    # publish joint states to the topic /joint_states which are used by the
    # robot_state_publisher node. A non-graphical tool to do the same is the
    # joint_state_publisher node.
    # joint_state_publisher_gui_node = Node(
    #    package="joint_state_publisher_gui",
    #    executable="joint_state_publisher_gui",
    #    name="joint_state_publisher_gui",
    # )

    # Starts up the RViz2 application.
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(launch_rviz),
    )

    # Delay RViz2 start after joint_state_broadcaster_spawner.
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )

    nodes_to_start = [
        controller_manager,
        joint_state_broadcaster_spawner,
        delay_rviz_after_joint_state_broadcaster_spawner,
        velocity_controller_spawner,
        # position_controller_spawner,
        joint_trajectory_controller_spawner,
        robot_state_publisher_node,
    ]
    return nodes_to_start


def generate_launch_description():
    """
    A Python launch file is meant to help implement the markup based frontends
    like YAML and XML, and so it is declarative in nature rather than
    imperative. For this reason, it is not possible to directly access the
    content of LaunchConfiguration parameters, which are asyncio futures. To
    access the content of a LaunchConfiguration, we must provide a context by
    wrapping the initialization method into an OpaqueFunction.
    https://answers.ros.org/question/397123/how-to-access-the-runtime-value-of-a-launchconfiguration-instance-within-custom-launch-code-injected-via-an-opaquefunction-in-ros2
    https://github.com/Serafadam/interbotix_ros_manipulators/blob/xsarm_control_galactic/interbotix_ros_xsarms/interbotix_xsarm_control/launch/xsarm_control.launch.py
    """
    declared_arguments = [
        DeclareLaunchArgument(
            "launch_rviz", default_value="true", description="Launch RViz?"
        ),
        OpaqueFunction(function=launch_setup),
    ]

    return LaunchDescription(declared_arguments)


def main():
    """
    This is used to execute the launch file as a normal python script.
    Simply type the following in a terminal window:
    ./robot.launch.py
    """
    ls = launch.LaunchService()
    ld = generate_launch_description()
    ls.include_launch_description(ld)
    return ls.run()


if __name__ == "__main__":
    main()
