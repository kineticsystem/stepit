# Stepit 2

A project to control stepper motors and professional cameras for 3D macro photography.

<img src="docs/pictures/stepit-architecture.png" width="100%">

## Configuration

For Arduino IDE (or Visual Studio Code) to find the included libraries, modify the variable the `sketchbook.path` in the following configuration file:
`<USER_HOME>/snap/arduino/61/.arduino15/preferences.txt`

## Build

Run the following command in the exact location where this README.md file is.

`colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug --symlink-install --event-handlers log-`
