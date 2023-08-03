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

import xacro

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
)
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)


def launch_setup(context, *args, **kwargs):
    """
    Launch all nodes.
    """

    # Declare all parameters.
    description_package_param = LaunchConfiguration("description_package")
    description_file_param = LaunchConfiguration("description_file")
    controllers_config_file_param = LaunchConfiguration("controllers_file")

    # Extract all parameters' values.
    description_file = PathJoinSubstitution(
        [FindPackageShare(description_package_param), "urdf", description_file_param]
    ).perform(context)
    controllers_config_file = PathJoinSubstitution(
        [
            FindPackageShare(description_package_param),
            "config",
            controllers_config_file_param,
        ]
    ).perform(context)

    robot_description_content = xacro.process_file(description_file).toxml()

    # The Controller Manager (CM) connects the controllers’ and hardware-abstraction
    # sides of the ros2_control framework. It also serves as the entry-point for users
    # through ROS services.
    # https://control.ros.org/master/doc/getting_started/getting_started.html#architecture
    ros2_control_node = Node(
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

    # robot_state_publisher uses the URDF specified by the parameter robot_description
    # and the joint positions from the topic /joint_states to calculate the forward
    # kinematics of the robot and publish the results via tf.
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[{"robot_description": robot_description_content}],
        output="screen",
    )

    freezer_controller_spawner = Node(
        package="freezer_controller",
        executable="spawner",
        arguments=["freezer_controller", "-c", "/controller_manager"],
    )

    nodes_to_start = [
        ros2_control_node,
        robot_state_publisher_node,
        freezer_controller_spawner,
    ]
    return nodes_to_start


def generate_launch_description():
    """
    A Python launch file is meant to help implement the markup based frontends like YAML
    and XML, and so it is declarative in nature rather than imperative. For this reason,
    it is not possible to directly access the content of LaunchConfiguration parameters,
    which are asyncio futures. To access the content of a LaunchConfiguration, we must
    provide a context by wrapping the initialization method into an OpaqueFunction.
    https://answers.ros.org/question/397123/how-to-access-the-runtime-value-of-a-launchconfiguration-instance-within-custom-launch-code-injected-via-an-opaquefunction-in-ros2
    https://github.com/Serafadam/interbotix_ros_manipulators/blob/xsarm_control_galactic/interbotix_ros_xsarms/interbotix_xsarm_control/launch/xsarm_control.launch.py
    """
    declared_arguments = [
        DeclareLaunchArgument(
            "description_package",
            default_value="stepit_description",
            description="Package containing all robot configuration files.",
        ),
        DeclareLaunchArgument(
            "description_file",
            default_value="stepit.urdf.xacro",
            description="URDF/XACRO description file for the robot.",
        ),
        DeclareLaunchArgument(
            "controllers_file",
            default_value="controllers.yaml",
            description="YAML file with the controllers configuration.",
        ),
        OpaqueFunction(function=launch_setup),
    ]

    return LaunchDescription(declared_arguments)