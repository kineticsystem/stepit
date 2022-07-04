# StepIt

A project to control stepper motors and professional cameras for 3D macro photography.

<img src="docs/pictures/stepit-architecture.png" width="100%">

## Prerequisites

We need a machine running Ubuntu 22.04 with ROS 2 Humble installed.

To install ROS 2, please refer to the document [Install ROS2 Humble on Ubuntu](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)

## StepIt Installation

Open a terminal and create a project folder anywhere inside the home directory, for example

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
git clone
```

## Configuration

For Arduino IDE (or Visual Studio Code) to find the included libraries, modify the variable the `sketchbook.path` in the following configuration file:
`<USER_HOME>/snap/arduino/61/.arduino15/preferences.txt`

## Build

Run the following command in the exact location where this README.md file is.

`colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug --symlink-install --event-handlers log-`
