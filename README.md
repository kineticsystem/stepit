[![CI](https://github.com/kineticsystem/stepit/actions/workflows/industrial_ci.yml/badge.svg)](https://github.com/kineticsystem/stepit/actions/workflows/industrial_ci.yml)
[![Format](https://github.com/kineticsystem/stepit/actions/workflows/ci-format.yml/badge.svg)](https://github.com/kineticsystem/stepit/actions/workflows/ci-format.yml)
[![Linters](https://github.com/kineticsystem/stepit/actions/workflows/ci-ros-lint.yml/badge.svg)](https://github.com/kineticsystem/stepit/actions/workflows/ci-ros-lint.yml)

<img src="docs/logo.png" width="50%">

## Table of Contents <!-- omit in toc -->

- [Introduction](#introduction)
- [Prerequisites](#prerequisites)
- [Install StepIt on the Microcontroller](#install-stepit-on-the-microcontroller)
- [Install StepIt on the Local Computer](#install-stepit-on-the-local-computer)
  - [Chekout the Git Repository](#chekout-the-git-repository)
  - [Pre-Commit Hooks](#pre-commit-hooks)
  - [Build the Project](#build-the-project)
- [Running the Application](#running-the-application)
- [How to run GitHub Actions locally](#how-to-run-github-actions-locally)

## Introduction

StepIt is a project to control stepper motors with a Teensy microcontroller and ROS2.

https://github.com/user-attachments/assets/e67d46ce-e133-4e34-bab8-7d924be3dee4

## Prerequisites

We need a computer with Ubuntu 22.04 and ROS 2 Humble.

To install ROS 2, please refer to the document [Install ROS2 Humble on Ubuntu](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html).

By default, StepIt runs in simulation mode and we do not need actual hardware to play around with it.

For a real application, we recommend attaching the stepper motors to a Teensy microcontroller 4.0 or 4.1. We can hook the motors in many different ways but we suggest the following hardware configuration.

- 1 x [Teensy 4.1](https://www.pjrc.com/store/teensy41.html) or Teensy 4.0.
- 1 x [PiBot multi-stepper motor driver board rev2.3 ](https://www.pibot.com/pibot-multi-stepper-motor-driver-board-rev2-3.html)
- 5 x [TMC2208](https://shop.watterott.com/SilentStepStick-TMC2208-Stepper-Motor-Driver-with-soldered-pinheaders)

The Teensy is connected to a computer using a USB cable. For a portable application, we can use a [Raspberry PI 4](docs/install_ros_on_rasperry_pi/install_ros2_on_rasperry_pi.md).

## Install StepIt on the Microcontroller

We developed code for the Teensy microcontroller using Visual Studio Code because it provides very good tools to format and validate the code. To achieve this goal, we must install [PlatformIO](https://platformio.org) extension which supports different microcontrollers including Arduino. Installing Arduino IDE is not required.

If PlatformIO cannot find the Python interpreter, install the following:

`sudo apt install python3-venv`

## Install StepIt on the Local Computer

### Chekout the Git Repository

Check out this git repository, including all required submodules.

```
git clone --recurse-submodules git@github.com:kineticsystem/stepit.git
```

If you missed the switch `--recurse-submodules`, you can clone all dependencies with the following commands:

```
cd stepit
git submodule update --init --recursive
```

Remember to enable recursion for relevant git commands, such that regular commands recurse into submodules by default.

```
git config --global submodule.recurse true
```

### Pre-Commit Hooks

Additionally, you should install git pre-commit hooks using the following command:

```
pre-commit install
```

### Build the Project

Move into the repo and install all required dependencies.

```
./bin/update.sh
```

Run Colcon to build the project.

```
./bin/build.sh
```

Execute all tests.

```
./bin/test.sh
```

## Running the Application

By default, the application runs with fake motors and a velocity controller. Thye position controller is disabled. Run the following commands on a terminal to start it up. This will also start up RViz.

```
source install/setup.bash
ros2 launch stepit_description robot.launch.py
```

Open a different terminal and run any of the following commands to spin up the fake motors.

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

If you change the `robot.launch.py` file and run the application with a position controller, you can use the following command instead:

```
ros2 topic pub -1 /position_controller/commands std_msgs/msg/Float64MultiArray "data: [0, 0]"
```

## How to run GitHub Actions locally

At each commit, the GitHub repository runs all available tests using [GitHub actions](https://docs.github.com/en/actions) and [Industrial CI](https://github.com/ros-industrial/industrial_ci).

A GitHub action fires up a docker container with Ubuntu 22.04 and ROS2 Humble, checks out and builds the code inside the docker container and runs all tests.

Sometimes, it may be desirable to execute the Continuous Integration pipeline locally. This is possible by using [Nektos](https://github.com/nektos/act).

First of all, we must create a GitHub token to access the repository. Then, we
must install Nektos `act` command in the user folder `~/bin` as explained in Nektos README.md file. We need an `.env` file at the root of the repository to define a few global variables required by Industrial CI. Finally, we can run the following command from the same folder:

`~/bin/act pull_request --workflows ./.github/workflows/industrial_ci_action.yml -s GITHUB_TOKEN`
