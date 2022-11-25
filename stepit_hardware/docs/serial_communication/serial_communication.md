# Arduino serial communication

## Introduction

To control stepper motors connected to a Teensy board via motor drivers, we need first a solution to send commands to the microcontroller using the serial port. Because ROS is the most common open-source framework in robotics, we would like to create a ROS node to deal with the low-level communication with the Teensy and use it as a proxy in all our robotics projects.

In ROS1, there is a valuable library to exchange information to and from microcontrollers: this library is [rosserial](http://wiki.ros.org/rosserial). The official library description is that rosserial is a protocol for wrapping standard ROS serialized messages and multiplexing multiple topics and services over a character device such as a serial port or network socket. Unfortunately, this library is unavailable in ROS2; the closest option is [Micro-ROS](https://micro.ros.org/docs/overview/features/), but it does not support AVR 8-bit microcontrollers, including Arduino Uno and Arduino Nano.

For this reason, we implemented a simple Point-to-Point Protocol (PPP) data link layer from scratch to send and receive data to and from the Teensy microcontroller over a serial connection. On top of it, we implemented a Request/Response communication pattern between a C++ ROS2 node and Arduino. Please refer to [Point-to-PointProtocol.pdf (PPP)](<./references/Point-to-Point%20Protocol%20(PPP).pdf>) for more information.

PPP separates frames by using the delimiter flag 0x7e. Should the same byte code appear in the frame data, the escape flag 0x7D will be inserted in place, followed by the delimiter flag XOR'd with 0x20, resulting in the sequence of two bytes 0x7d 0x5e. Following is a table describing a frame structure.

<table align="center" border="2px">
  <tr><td><b>PPP frame</b></td></tr>
  <tr><td>delimiter flag</td></tr>
  <tr><td>...</td></tr>
  <tr><td>data</td></tr>
  <tr><td>...</td></tr>
  <tr><td>checksum</td></tr>
  <tr><td>delimiter flag</td></tr>
</table>

In this implementation, each frame contains a command and all required parameters in binary format. The following example shows a Move Command request to move motor 0 forward 20000 steps and the corresponding response.

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

Following is a short list of all numeric Arduino data types.

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

As a general rule in network communication, when sending a value of type int, long, or float, the Most Significant Byte (MSB) is transmitted first.

## Commands

As stated at the beginning of the document, the project's main scope is to control stepper motors connected to a Teensy microcontroller via a motor driver circuit. A motor does not move instantaneously at maximum speed but accelerates, maintains a predefined velocity, and then decelerates until reaching the target position. To control the motor movement with acceleration and deceleration, we use an Arduino well-known library called [AccelStepper](https://www.airspayce.com/mikem/arduino/AccelStepper/).

Following is a list of all desired commands to control the motor movements.

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
