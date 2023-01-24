# StepIt

[![CI](https://github.com/kineticsystem/stepit/actions/workflows/industrial_ci_action.yml/badge.svg)](https://github.com/kineticsystem/stepit/actions/workflows/industrial_ci_action.yml)

## Introduction

StepIt is a project to control stepper motors with a Teensy microcontroller and ROS2.

## Prerequisites

We need a computer with Ubuntu 22.04 and ROS 2 Humble.

To install ROS 2, please refer to the document [Install ROS2 Humble on Ubuntu](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html).

By default, StepIt runs in simulation mode and we do not need actual hardware to play around with it.

For a real application, we recommend attaching the stepper motors to a Teensy microcontroller 4.0 or 4.1. We can hook the motors in many different ways but we suggest the following hardware configuration.

- 1 x [Teensy 4.1](https://www.pjrc.com/store/teensy41.html) or Teensy 4.0.
- 1 x [PiBot multi-stepper motor driver board rev2.3 ](https://www.pibot.com/pibot-multi-stepper-motor-driver-board-rev2-3.html)
- 5 x [TMC2208](https://shop.watterott.com/SilentStepStick-TMC2208-Stepper-Motor-Driver-with-soldered-pinheaders)

The Teensy is connected to a computer using a USB cable. For a portable application, we can use a [Raspberry PI 4](docs/install_ros_on_rasperry_pi/install_ros2_on_rasperry_pi.md).

## StepIt microcontroller installation

We developed code for the Teensy microcontroller using Visual Studio Code because it provides very good tools to format and validate the code. To achieve this goal, we must install [PlatformIO](https://platformio.org) extension which supports different microcontrollers including Arduino. Installing Arduino IDE is not required.

## StepIt computer installation

Open a terminal and run the following command to source the ROS 2 Humble installation.

`source /opt/ros/humble/setup.bash`

Create a project folder anywhere inside the home directory, for example

`<HOME_DIR>/stepit_ws`

Through the document, we will use `<STEPIT_WS>` to refer to this folder.

```
cd ~
mkdir stepit_ws
cd stepit_ws
```

Create a source folder and check out this git repository.

```
mkdir src
cd src
git clone git@github.com:kineticsystem/stepit.git
```

Check out all external dependencies.

```
cd stepit
git submodule update --init
```

Remember to enable recursion for relevant git commands, such that regular commands recurse into submodules by default.


```
git config --global submodule.recurse true
```

Move into the base `<STEPIT_WS>` folder and install all required dependencies.

```
rosdep install --ignore-src --from-paths . -y -r
```

Run Colcon to build the project.

```
sudo apt install python3-colcon-common-extensions
colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo --symlink-install --event-handlers log-
```

Execute all tests.

```
colcon test
```

## Running the application

By default, the application runs with fake motors. Run the following commands on a terminal to start it up. This will also start up RViz.

```
cd <STEPIT_WS>
source install/setup.bash
ros2 launch stepit_description robot.launch.py
```

We can control the motors with a velocity controller or a position controller. Open a different terminal and run any of the following commands to spin up the fake motors.

```
cd <STEPIT_WS>
source install/setup.bash
```

To control the position run

```
ros2 topic pub -1 /position_controller/commands std_msgs/msg/Float64MultiArray "data: [0, 0]"
```

To control the velocity run

```
ros2 topic pub -1 /velocity_controller/commands std_msgs/msg/Float64MultiArray "data: [6,-6]"
```

To control a trajectory with a sequence of positions and velocities run

```
ros2 topic pub -1 /joint_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{
  joint_names: ["joint1", "joint2", "joint3", "joint4", "joint5"],
  points: [
    {
      positions: [0, 0, 0, 0, 0],
      velocities: [0, 0, 0, 0, 0],
      time_from_start: {
        sec: 0,
        nanosec: 0
      }
    },
    {
      positions: [31.4159265358979,37.6991118430775,43.9822971502571,50.2654824574367,56.5486677646163],
      velocities: [0, 0, 0, 0, 0],
      time_from_start: {
        sec: 10,
        nanosec: 0
      }
    },
    {
      positions: [0, 0, 0, 0, 0],
      velocities: [0, 0, 0, 0, 0],
      time_from_start: {
        sec: 12,
        nanosec: 0
      }
    },
    {
      positions: [-31.4159265358979,-37.6991118430775,-43.9822971502571,-50.2654824574367,-56.5486677646163],
      velocities: [0, 0, 0, 0, 0],
      time_from_start: {
        sec: 17,
        nanosec: 0
      }
    },
    {
      positions: [0, 0, 0, 0, 0],
      velocities: [0, 0, 0, 0, 0],
      time_from_start: {
        sec: 20,
        nanosec: 0
      }
    },
  ]
}"
```

The trajectory is a list of waypoints, each of them containing the desired position and velocity of each joint at a given time.

## How to run GitHub actions locally

At each commit, the GitHub repository runs all available tests using [GitHub actions](https://docs.github.com/en/actions) and [Industrial CI](https://github.com/ros-industrial/industrial_ci).

A GitHub action fires up a docker container with Ubuntu 22.04 and ROS2 Humble, checks out and builds the code inside the docker container and runs all tests.

Sometimes, it may be desirable to execute the Continuous Integration pipeline locally. This is possible by using [Nektos](https://github.com/nektos/act).

First of all, we must create a GitHub token to access the repository. Then, we
must install Nektos `act` command in the user folder `~/bin` as explained in Nektos README.md file. We need an `.env` file at the root of the repository to define a few global variables required by Industrial CI. Finally, we can run the following command from the same folder:

`~/bin/act pull_request --workflows ./.github/workflows/industrial_ci_action.yml -s GITHUB_TOKEN`
