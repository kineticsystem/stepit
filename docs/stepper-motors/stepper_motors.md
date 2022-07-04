# Stepper Motors

## Introduction

Two very good videos to understand the basic functionality of stepper motors are the following ones where an Hybrid Stepper Motor is described:

http://www.youtube.com/watch?v=MHdz3c6KLrg

http://www.youtube.com/watch?v=t-3VnLadIbc

This is also an exceptional source of information, made by the same person who created the above two videos:

http://www.pcbheaven.com/wikipages/How_Stepper_Motors_Work/

## Stepper motor components

- Coils: these are the electromagnets inside the motor.
- Stator: this is the external part of the motor, where the coils are attached.
- Rotor: this is the rotating part of the motor.
- Shaft: this is the bar holding the rotor. Italian: “albero di trasmissione”, “asse”.

## Phases

A Phase (or step) is the momentary condition of the stepper motor: which coil(s) is (are) energised and in which direction. The different combinations of states in which the coils can be determine the number of phases. Two phases, is the minimum number of phases needed for a stepper motor to operate. The more the phases, the smoother the operation of the motor and the higher the price. The number of phases is not strictly related to the number of coils: for example we can have a 4 phases 2-coil bipolar stepper motor and a 4 phases 4-coil unipolar stepper motor.

## Driving modes

A good video on different driving techniques is the following one:

http://www.youtube.com/watch?v=bngx2dKl5jU

### Wave drive or Single-Coil Excitation - Full Step Drive

With this technique only one coil (not phase) is activated at each time in sequence and this causes the motor to rotate in full steps.

### One phase - Full Step Drive

With this technique only one phase a time is activated and the movement of the rotor between each phases is one full step.

## Two Phases - Full Step Drive

With this technique there are always two phases activated at the same time. The motor still moves in full steps, so using this technique we do not increase the resolution, but we increase the torque of the motor, the reason is because we have more electromagnets on at the same time.

## One-Two Phases - Half Step Drive

With this technique a single phase ON is followed by two phases ON. This will double the resolution of the motor that can now be driven in half steps. Because two phases ON are stronger than a single phases ON this technique, if controlled but a pulse of current, introduces a variable torque and, as a consequence, vibrations. For this reason the current has to be properly modulated to keep a constant torque and this results in a more complex driving circuit.

## Variable Phases Intensity - Micro Step Drive

In this scenario each phase is not driven by a pulse of current, but is instead driven by a modulated current, for example a cos or sin modulated current. So a phase is not simply ON or OFF but can be in any state between the two. Although it seems that the microstepping increases the steps even further, usually this does not happen. In high accuracy applications, trapezoidal gears are used to increase the accuracy. This method is used to ensure smooth motion and reduce resonance between the internal parts of the motor. In fact, using micro stepping with a load attached to the motor, does not necessarily mean that the motor goes on that micro position.

## Motor construction

### Permanent Magnet Stepper Motor (PM)

This is a motor where the rotor is made with a Permanent Magnet with South-North polarity perpendicular to the rotation axis.

### Variable Reluctance Stepper Motor (VR)

This is a motor where the rotor is made of soft iron, with no magnets. It is good for accuracy but has a very low torque.

### Hybrid Stepper Motor

This is a motor where the rotor is made with a magnet (a disk) with South-North polarity parallel to the rotation axis. This is the motor described by the videos at the beginning of the document.
Coils Connections

### Bipolar

In this configuration every phase is connected to a 2-pins input. Internally coils can be connected in series or parallel. To invert the polarity of the coils related to a specific phase the current has to be inverted. Inverting the polarity is required to change the rotation of the motor. Because the current must flow in both direction a more complicated driving circuit is required.

### Unipolar

In unipolar stepper motor each phase is connected to a 3-pins input. One of the pin is connected between two pairs of coils. If this pin is connected to ground, the polarity of the coils can be controlled by powering one of the two remaining pins. The current always flows in the same direction so a simpler driving circuit is required. The main disadvantage is that with this configuration only one coil winding is on each time, halving the potential torque. The unipolar motor can be used as a bipolar motor, simply by leaving the common wire unconnected.

### 8-lead stepper

In this configuration each coil has its own 2-pins input connection. This motor can be used in bipolar or unipolar configuration.

## Notes

What are stepper motors good for?

- Positioning – Since steppers move in precise repeatable steps, they excel in applications requiring precise positioning such as 3D printers, CNC, Camera platforms and X,Y Plotters. Some disk drives also use stepper motors to position the read/write head.
- Speed Control – Precise increments of movement also allow for excellent control of rotational speed for process automation and robotics.
- Low Speed Torque - Normal DC motors don't have very much torque at low speeds. A Stepper motor has maximum torque at low speeds, so they are a good choice for applications requiring low speed with high precision.

## What are their limitations?

- Low Efficiency – Unlike DC motors, stepper motor current consumption is independent of load. They draw the most current when they are doing no work at all. Because of this, they tend to run hot.
- Limited High Speed Torque - In general, stepper motors have less torque at high speeds than at low speeds. Some steppers are optimized for better high-speed performance, but they need to be paired with an appropriate driver to achieve that performance.
- No Feedback – Unlike servo motors, most steppers do not have integral feedback for position. Although great precision can be achieved running ‘open loop’. Limit switches or ‘home’ detectors are typically required for safety and/or to establish a reference position.

## Links

http://www.solarbotics.net/library/pdflib/pdf/motorbas.pdf
