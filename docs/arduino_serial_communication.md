# Arduino serial communication

## Introduction

In ROS1 there is a useful library to exchange information to and from Arduino: this library is [rosserial](http://wiki.ros.org/rosserial). The official library description is that rosserial is a protocol for wrapping standard ROS serialized messages and multiplexing multiple topics and services over a character device such as a serial port or network socket.

This library is not available in ROS2 and the closest option is [Micro-ROS](https://micro.ros.org/docs/overview/features/). Unfortunately, Micro-ROS does not support AVR 8-bit microcontrollers, which include Arduino Uno and Arduino Nano.

For this reason, to send and receive data to and from Arduino over a serial connection, we implemented a simple Point-to-Point Protocol (PPP) data link layer from scratch. On top of it, we implemented a Request/Response communication pattern between a C++ ROS2 node and Arduino. Please refer to [Point-to-PointProtocol.pdf (PPP)](<./lectures/Point-to-Point%20Protocol%20(PPP).pdf>).

PPP delimits frames by using the delimeter flag 0x7e. Should the same byte code appear in the frame data, the escape flag 0x7D will be inserted in place followed by the delimeter flag XOR'd with 0x20, resulting in the sequence of two bytes 0x7d 0x5e.

<table align="center" border="2px">
  <tr><td><b>PPP frame</b></td></tr>
  <tr><td>delimeter flag</td></tr>
  <tr><td>...</td></tr>
  <tr><td>data</td></tr>
  <tr><td>...</td></tr>
  <tr><td>checksum</td></tr>
  <tr><td>delimeter flag</td></tr>
</table>

## Arduino base types

<table align="center" border="2px">
  <tr><td><b>type</b></td><td><b>bytes</b></td></tr>
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

## Arduino base types

boolean 1 byte

byte 1 byte

char 1 byte
unsigned char 1 byte

int 2 bytes
unsigned int 2 bytes

short 2 bytes
unsigned short 2 bytes

long 4 bytes
unsigned long 4 bytes

float 4 bytes
double 4 bytes

word 2 bytes

## Commands

Following a list of commands to control steppers motors attached to an Arduino board.

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
        <td>Move<b></td>
        <td>0x70</td>
        <td rowspan="2">Move a motor for a given amount of steps (positive or negative).</td>
        <td>motorId</td>
        <td>uchar</td>
        <td>The motor id.</td>
    </tr>
    <tr>
        <td></td>
        <td></td>
        <td>steps</td>
        <td>long</td>
        <td>The number of steps to move, either negative or positive.</td>
    </tr>
    <tr>
        <td>Move to<b></td>
        <td>0x71</td>
        <td rowspan="2">Move a stepper to a given absolute position.</td>
        <td>motorId</td>
        <td>uchar</td>
        <td>The motor id.</td>
    </tr>
    <tr>
        <td></td>
        <td></td>
        <td>position</td>
        <td>long</td>
        <td>The absolute position.</td>
    </tr>
    <tr>
        <td>Keep alive<b></td>
        <td>0x73</td>
        <td >Safety command to keep the motors alive.</td>
        <td></td>
        <td></td>
        <td></td>
    </tr>
    <tr>
        <td>Stop all<b></td>
        <td>0x73</td>
        <td >Stop all motors and make them decelerate to zero speed.</td>
        <td></td>
        <td></td>
        <td></td>
    </tr>
    <tr>
        <td>Stop motor<b></td>
        <td>0x72</td>
        <td>Move a stepper to a given absolute position.</td>
        <td>motorId</td>
        <td>uchar</td>
        <td>The motor id.</td>
    </tr>
    <tr>
        <td>Set target speed<b></td>
        <td>0x77</td>
        <td rowspan="2">Set a motor relative speed between 0 and 100.<br/>The motor accelerats or develerates until the relative target speed is reached.</td>
        <td>motorId</td>
        <td>uchar</td>
        <td>The motor id.</td>
    </tr>
    <tr>
        <td></td>
        <td></td>
        <td>speed</td>
        <td>float</td>
        <td>The relative speed between 0 and 100.</td>
    </tr>
    <tr>
        <td>Set max speed<b></td>
        <td>0x78</td>
        <td rowspan="2">Set the maximum possible speed in steps/s.</td>
        <td>motorId</td>
        <td>uchar</td>
        <td>The motor id.</td>
    </tr>
    <tr>
        <td></td>
        <td></td>
        <td>speed</td>
        <td>float</td>
        <td>The maximum achievable speed in steps/s.</td>
    </tr>
    <tr>
        <td>Enable motors<b></td>
        <td>0x7A</td>
        <td>Enable the motors loop.</td>
        <td></td>
        <td></td>
        <td></td>
    </tr>
    <tr>
        <td>Disable motors<b></td>
        <td>0x7B</td>
        <td>Disable the motors loop.</td>
        <td></td>
        <td></td>
        <td></td>
    </tr>
    <tr>
        <td>Set motor position<b></td>
        <td>0x79</td>
        <td rowspan="2">Set the current motor position to be the given absolute position.</td>
        <td>motorId</td>
        <td>uchar</td>
        <td>The motor id.</td>
    </tr>
    <tr>
        <td></td>
        <td></td>
        <td>position</td>
        <td>long</td>
        <td>The motor absolute position.</td>
    </tr>
</table>
