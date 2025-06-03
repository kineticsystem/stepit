# Serial Communication for Teensy

## Overview

The objective of this document is to detail the protocol and technicalities of establishing a serial communication link between stepper motors connected to a Teensy microcontroller board and a Robotic Operating System (ROS) node. ROS is the prevailing open-source framework in the field of robotics. Our intention is to design a ROS node responsible for low-level communications with the Teensy board, thus acting as a middleware layer for robotic applications.

Traditionally, ROS1 provided a feature-rich library known as [rosserial](http://wiki.ros.org/rosserial) for serial communications with microcontrollers. However, ROS2 lacks native support for the same. The closest alternative is [Micro-ROS](https://micro.ros.org/docs/overview/features/), which is unfortunately incompatible with AVR 8-bit microcontrollers like Arduino Uno and Arduino Nano.

To bridge this gap, we have devised a frame-based serial communication protocol employing Consistent Overhead Byte Stuffing (COBS). This protocol facilitates Request/Response communication patterns between a C++ ROS2 node and the Teensy board, thereby enabling effective data exchange.

### Technical Specifications

The COBS algorithm utilizes a delimiter flag `0x7e` to distinguish between different frames. In scenarios where the delimiter flag appears within the frame data, an escape flag `0x7D` is introduced, followed by the XOR of the delimiter flag with `0x20`. The resulting two-byte sequence becomes `0x7d 0x5e`.

Below is the structure of a data frame:

<table align="center" border="2px">
  <tr><td align="center"><b>Frame</b></td></tr>
  <tr><td align="center">Delimiter flag</td></tr>
  <tr><td align="center">...</td></tr>
  <tr><td align="center">Data</td></tr>
  <tr><td align="center">...</td></tr>
  <tr><td align="center">Checksum</td></tr>
  <tr><td align="center">Delimiter flag</td></tr>
</table>

Each frame encapsulates a specific command along with its requisite parameters in binary format. A practical example representing a 'Move Command' to propel motor 0 forward by 20,000 steps is detailed below.

Request:

<table align="center" border="2px">
  <tr>
    <td align="center"><b>Delimiter</b></td>
    <td align="center">0x7e</td>
  </tr>
 <tr>
    <td align="center"><b>Move command ID</b></td>
    <td align="center">0x70</td>
  </tr>
  <tr>
    <td align="center"><b>Motor ID</b></td>
    <td align="center">0x00</td>
  </tr>
  <tr>
    <td align="center" rowspan="4"><b>Steps</b></td>
    <td align="center">0x00</td>
  </tr>
  <tr>
    <td align="center">0x00</td>
  </tr>
  <tr>
    <td align="center">0x4E</td>
  </tr>
  <tr>
    <td align="center">0x20</td>
  </tr>
  <tr>
    <td align="center" rowspan="2"><b>Checksum</b></td>
    <td align="center">0x75</td>
  </tr>
  <tr>
    <td align="center">0x38</td>
  </tr>
  <tr>
    <td align="center"><b>Delimiter</b></td>
    <td align="center">0x7e</td>
  </tr>
</table>

Response:

<table align="center" border="2px">
  <tr>
    <td align="center"><b>Delimiter</b></td>
    <td align="center">0x7e</td>
  </tr>
  <tr>
    <td align="center"><b>Success response</b></td>
    <td align="center">0x11</td>
  </tr>
  <tr>
    <td align="center" rowspan="2"><b>Checksum</b></td>
    <td align="center">0x08</td>
  </tr>
  <tr>
    <td align="center">0x01</td>
  </tr>
  <tr>
    <td align="center"><b>Delimiter</b></td>
    <td align="center">0x7e</td>
  </tr>
</table>

## Data Types and Byte Order

For reference, listed below are the numeric data types supported by Arduino:

<table align="center" border="2px">
  <tr><td><b>Type</b></td><td><b>Bytes</b></td></tr>
  <tr><td>boolean</td><td>1 byte</td></tr>
  <tr><td>byte</td><td>1 byte</td></tr>
  <tr><td>char</td><td>1 byte</td></tr>
  <tr><td>unsigned char</td><td>1 byte</td></tr>
  <tr><td>int</td><td>2 bytes</td></tr>
  <tr><td>unsigned int</td><td>2 bytes</td></tr>
  <tr><td>short</td><td>2 bytes</td></tr>
  <tr><td>unsigned short</td><td>2 bytes</td></tr>
  <tr><td>word</td><td>2 bytes</td></tr>
  <tr><td>long</td><td>4 bytes</td></tr>
  <tr><td>unsigned long</td><td>4 bytes</td></tr>
  <tr><td>float</td><td>4 bytes</td></tr>
  <tr><td>double</td><td>4 bytes</td></tr>
</table>

In network communications, values of type `int`, `long`, and `float` are transmitted with the Most Significant Byte (MSB) first.

## Command Interface

The primary focus of this project lies in the real-time control of stepper motors connected to a Teensy microcontroller via motor driver circuits. For advanced control that includes acceleration and deceleration, we leverage the [AccelStepper](https://www.airspayce.com/mikem/arduino/AccelStepper/) library.

Below are the supported commands for controlling motor movements:

<table align="center" border="2px">
    <tr>
        <td><b>Command<b></td>
        <td><b>Code<b></td>
        <td><b>Command description<b></td>
        <td><b>Param<b></td>
        <td><b>Type<b></td>
        <td><b>Parameter description</b></td>
    </tr>
    <tr>
        <td rowspan="3">Move<b></td>
        <td rowspan="3">0x71</td>
        <td rowspan="3">Move a motor to a given position.</td>
        <td>motorId</td>
        <td>uchar</td>
        <td>The motor id.</td>
    </tr>
    <tr>
        <td>position</td>
        <td>float</td>
        <td>The absolute position in radians.</td>
    </tr>
    <tr>
        <td colspan="3">Repeat for each position to set.</td>
    </tr>
     <tr>
        <td rowspan="3">Speed<b></td>
        <td rowspan="3">0x77</td>
        <td rowspan="3">Move a motor at a given speed.<br/>The motor accelerates or decelerates until reaching the given speed.</td>
        <td>motorId</td>
        <td>uchar</td>
        <td>The motor id.</td>
    </tr>
    <tr>
        <td>speed</td>
        <td>float</td>
        <td>The motor speed in radians per second.</td>
    </tr>
    <tr>
        <td colspan="3">Repeat for each speed to set.</td>
    </tr>
    <tr>
        <td>Enable/disable motors<b></td>
        <td>0x7A</td>
        <td>Enable motors Service Interrupt Routine (SIR) call.</td>
        <td colspan="3"></td>
    </tr>
</table>

Note: Multiple position or speed parameters can be set in a single command by repeating the corresponding segments.

By implementing these protocols and command sets, we aim to facilitate seamless communication for robotic applications that involve stepper motors and ROS environments.
