# StepIt

[![CI](https://github.com/kineticsystem/stepit/actions/workflows/industrial_ci_action.yml/badge.svg)](https://github.com/kineticsystem/stepit/actions/workflows/industrial_ci_action.yml)

A project to control stepper motors and professional cameras for 3D macro photography.

<img src="docs/pictures/stepit-architecture.png" width="100%">

## Prerequisites

We need a machine running Ubuntu 22.04 with ROS 2 Humble installed.

To install ROS 2, please refer to the document [Install ROS2 Humble on Ubuntu](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)

## StepIt Installation

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

Create a source folder and checkout this git repository.

```
mkdir src
cd src
git clone git@github.com:kineticsystem/stepit.git
```

Check out the external dependencies.

```
vcs import --force < stepit/stepit.repos
vcs pull
```

Install all required dependencies.

```
rosdep install --ignore-src --from-paths . -y
```

Move into the base `<STEPIT_WS>` folder and run Colcon to build the project.

```
cd ..
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug --symlink-install --event-handlers log-
```

Execute all tests.

```
colcon test
```

## Configuration

For Arduino IDE (or Visual Studio Code) to find the included libraries, modify the variable the `sketchbook.path` in the following configuration file:
`<USER_HOME>/snap/arduino/61/.arduino15/preferences.txt`
