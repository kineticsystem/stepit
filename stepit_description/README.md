# StepIt Description

This package configures a sample robot that uses the StepIt Driver. The robot is displayed in RViz.

To activate the robot execute the following command:

```
ros2 launch stepit_description robot.launch.py
```

By default, the robot runs on fake hardware:

```
<hardware>
  <plugin>stepit_driver/StepitHardware</plugin>
  <param name="use_dummy">true</param>
  <param name="usb_port">/dev/ttyACM0</param>
  <param name="baud_rate">9600</param>
  <param name="timeout">2.0</param>
</hardware>
```

To make the robot move, we need to connect the hardware to a controller. This functionality is implemented in the package `stepit_bringup`.
