/*
 * Copyright (C) 2022 Remigi Giovanni
 * g.remigi@kineticsystem.org
 * www.kineticsystem.org
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU Lesser General Public License as published by the Free
 * Software Foundation; either version 2 of the License, or (at your option) any
 * later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License for more
 * details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program; if not, write to the Free Software Foundation, Inc.,
 * 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <AccelStepper.h>
#include <TimerInterrupt.h>
#include <SerialPort.h>
#include <DataBuffer.h>
#include <Buffer.h>
#include "SharedBucket.h"
#include "MotorGoal.h"
#include "MotorState.h"

// Motors connections.

static const byte STEPPER0_STEP_PIN = 9;
static const byte STEPPER0_DIR_PIN = 10;
static const byte STEPPER1_STEP_PIN = 11;
static const byte STEPPER1_DIR_PIN = 12;

// This is the information sent by Arduino during the connection handshake.
const char NAME[] = "STEPPIT\0";

const int INTERRUPT_TIME = 90; // Microseconds.

// Input commands. For each command received, a response must be returned.
// The response always contains a success or an error code.
// If a command requires additional information, the success response is
// followed by the requested data.
// All messages sent by Arduino have the most significant bit set to 0.

byte MOVE_CMD = 0x70;                 // Move motors a given amount of steps.
byte MOVE_TO_CMD = 0x71;              // Move motors to a given position.
byte STOP_CMD = 0x72;                 // Stop motors.
byte SET_ACCELERATION_CMD = 0x73;     // Set motors acceleration.
byte STATUS_CMD = 0x75;               // Request motors status, position and speed.
byte INFO_CMD = 0x76;                 // Request controller info for connection handshaking.
byte SET_TARGET_SPEED_CMD = 0x77;     // Set motors target relative speed (0-100).
byte SET_MAX_SPEED_CMD = 0x78;        // Set motors max absolute speed (step/s).
byte SET_CURRENT_POSITION_CMD = 0x79; // Set the motor current position.
byte SET_MOTORS_ENABLED_CMD = 0x7A;   // Enable interrupt to control the motors.

// Arduino reboots in around two seconds when a serial connection is initiated.
// To notify the client that Arduino is ready to receive and transmit data, a ready message
// is sent.
byte READY_MSG = 0x10;

// Success response code.
byte SUCCESS_MSG = 0x11;

// Error response code.
byte ERROR_MSG = 0x12;

// Stepper motors.

AccelStepper stepper[2] = {
    AccelStepper(AccelStepper::DRIVER, STEPPER0_STEP_PIN, STEPPER0_DIR_PIN),
    AccelStepper(AccelStepper::DRIVER, STEPPER1_STEP_PIN, STEPPER1_DIR_PIN)};

// A Goal is created by the main thread and read by the ISR.
// The ISR is way faster than the main thread, so a goal is consumed as soon as
// it is created.
// TODO: validate the scenario where two commands a written to the buffer before one is read.

Buffer<MotorGoal> motorGoals[2] = {
    Buffer<MotorGoal>{10},
    Buffer<MotorGoal>{10},
};

// This is a structure that contains motor states. It is read by the main
// thread and written by the interrupt callback.
Buffer<MotorState> motorState[2] = {
    Buffer<MotorState>{10},
    Buffer<MotorState>{10},
};

// Class to read and write over a serial port.
SerialPort serialPort{255, 255};

// Message to be written to the serial port, usually a response.
DataBuffer responseBuffer{255};

void sendReadyMessage()
{
    responseBuffer.addByte(READY_MSG, Location::END);
    serialPort.write(&responseBuffer);
}

/**
 * Default response when a correct command, not requiring information, has been received.
 */
void returnCommandSuccess(byte requestId)
{
    responseBuffer.addByte(requestId, Location::END);
    responseBuffer.addByte(SUCCESS_MSG, Location::END);
    serialPort.write(&responseBuffer);
}

/**
 * When a Status command is received, send back the position of the motor and
 * speed.
 */
void returnStatus(byte requestId)
{
    responseBuffer.addByte(requestId, Location::END);
    responseBuffer.addByte(SUCCESS_MSG, Location::END);
    for (int i = 0; i < 2; i++)
    {
        responseBuffer.addLong(motorState[i].getCurrentPosition(), Location::END);
        float speed = 100.0 * motorState[i].getSpeed() / motorState[i].getMaxSpeed();
        responseBuffer.addFloat(speed, Location::END);
        responseBuffer.addLong(motorState[i].getDistanceToGo(), Location::END);
    }
    serialPort.write(&responseBuffer);
}

/**
 * Send information about the program. It is used at connection handshake
 * to help the client, on the other side of the serial port, identify the
 * correct port where the Arduino is connected.
 */
void returnControllerInfo(byte requestId)
{
    responseBuffer.addByte(requestId, Location::END);
    responseBuffer.addByte(SUCCESS_MSG, Location::END);
    for (int i = 0; NAME[i] != '\0'; i++)
    {
        responseBuffer.addByte(NAME[i], Location::END);
    }
    serialPort.write(&responseBuffer);
}

/**
 * Enable or disable the Service Interrupt Routine (SIR) to move the motros.
 * @param requestId The request id.
 * @param enabled True to enable the motors control, false to disable.
 */
void setMotorsEnabled(byte requestId, byte enabled)
{
    if (enabled == 0)
    {
        TimerInterrupt::start(INTERRUPT_TIME);
    }
    else
    {
        TimerInterrupt::stop();
    }
    returnCommandSuccess(requestId);
}

void setMotorTargetSpeed(byte requestId, byte motorId, float maxSpeedPercentage)
{
    if (maxSpeedPercentage > 100.0)
    {
        maxSpeedPercentage = 100.0;
    }
    motorGoals[motorId].setTargetSpeed(bucket[motorId].getMaxSpeed() * maxSpeedPercentage / 100.0);
    motorGoals->setReadReady(true);
    returnCommandSuccess(requestId);
}

void motorMove(byte requestId, byte motorId, long steps)
{
    motorGoals[motorId].setTargetPosition(bucket[motorId].getCurrentPosition() + steps);
    motorGoals->setReadReady(true);
    returnCommandSuccess(requestId);
}

void motorMoveTo(byte requestId, byte motorId, long position)
{
    motorGoals[motorId].setTargetPosition(position);
    motorGoals->setReadReady(true);
    returnCommandSuccess(requestId);
}

/**
 * Decode the input command and execute the requested action.
 * @param msg The input message containing the requested command.
 */
void processBuffer(byte requestId, DataBuffer *msg)
{
    // State machine.

    if (msg->getSize() > 0)
    {
        // Get the command id.
        byte cmdId = msg->removeByte(Location::FRONT);

        if (cmdId == STATUS_CMD && msg->getSize() == 0)
        {
            returnStatus(requestId);
        }
        else if (cmdId == SET_MOTORS_ENABLED_CMD && msg->getSize() == 1)
        {
            byte enabled = msg->removeByte(Location::FRONT);
            setMotorsEnabled(requestId, enabled);
        }
        else if (cmdId == SET_TARGET_SPEED_CMD && msg->getSize() == 5)
        { // Set motor speed.
            byte motorId = msg->removeByte(Location::FRONT);
            float targetSpdPercentage = fabs(msg->removeFloat(Location::FRONT)); // Ensure speed is always positive.
            setMotorTargetSpeed(requestId, motorId, targetSpdPercentage);
        }
        else if (cmdId == MOVE_CMD && msg->getSize() == 5)
        { // Move motor by a specified number for steps.
            byte motorId = msg->removeByte(Location::FRONT);
            long steps = msg->removeLong(Location::FRONT);
            motorMove(requestId, motorId, steps);
        }
        else if (cmdId == MOVE_TO_CMD && msg->getSize() == 5)
        { // Move motor by a specified number for steps.
            byte motorId = msg->removeByte(Location::FRONT);
            long position = msg->removeLong(Location::FRONT);
            motorMoveTo(requestId, motorId, position);
        }
        else if (cmdId == STOP_CMD && (msg->getSize() == 0 || msg->getSize() == 1))
        {
            if (msg->getSize() == 0)
            { // Stop all motors.
                for (byte i = 0; i < 2; i++)
                {
                    bucket[i].setTargetSpeed(0);
                }
            }
            else if (msg->getSize() == 1)
            { // Stop a given motor.
                byte motorId = msg->removeByte(Location::FRONT);
                bucket[motorId].setTargetSpeed(0);
            }
            returnCommandSuccess(requestId);
        }
        else if (cmdId == SET_ACCELERATION_CMD && msg->getSize() == 5)
        { // Set motor acceleration.
            byte motorId = msg->removeByte(Location::FRONT);
            float acceleration = msg->removeFloat(Location::FRONT);
            bucket[motorId].setAcceleration(acceleration);
            returnCommandSuccess(requestId);
        }
        else if (cmdId == SET_MAX_SPEED_CMD && msg->getSize() == 5)
        { // Set motor max speed acceleration.
            byte motorId = msg->removeByte(Location::FRONT);
            float maxSpeed = msg->removeFloat(Location::FRONT);
            bucket[motorId].setMaxSpeed(maxSpeed);
            returnCommandSuccess(requestId);
        }
        else if (cmdId == INFO_CMD && msg->getSize() == 0)
        { // Return controller info for connection handshaking.
            returnControllerInfo(requestId);
        }
        else if (cmdId == SET_CURRENT_POSITION_CMD && msg->getSize() == 5)
        {
            if (msg->getSize() == 5)
            { // Set position for a given motor.
                byte motorId = msg->removeByte(Location::FRONT);
                long position = msg->removeLong(Location::FRONT);
                stepper[motorId].setCurrentPosition(position);
            }
            returnCommandSuccess(requestId);
        }
        msg->clear();
    }
}

/*
 * Method called by an interrupt timer to move the stepper motors.
 * AccelStepper library only support acceleration to maximum speed and deceleration
 * to 0 speed. It doesn't support deceleration to a different speed.
 * The following code implements deceleration stopping the stepper until it reaches
 * the target speed.
 */
void run()
{
    for (int i = 0; i < 2; i++)
    {
        // Read motor status.

        float actualSpeed = stepper[i].speed();
        long currentPosition = stepper[i].currentPosition();
        long distanceToGo = stepper[i].distanceToGo();

        // Save motor status.

        bucket[i].setSpeed(actualSpeed);
        bucket[i].setCurrentPosition(currentPosition);
        bucket[i].setDistanceToGo(distanceToGo);

        actualSpeed = fabs(actualSpeed);

        if (bucket[i].getTargetPosition() != bucket[i].getOldTargetPosition())
        { // Position has changed.
            bucket[i].setOldTargetPosition(bucket[i].getTargetPosition());
            if (!bucket[i].isDecelerating())
            {
                stepper[i].moveTo(bucket[i].getTargetPosition());
            }
        }
        else if (bucket[i].getTargetSpeed() != bucket[i].getOldTargetSpeed())
        { // Target speed has changed
            bucket[i].setOldTargetSpeed(bucket[i].getTargetSpeed());
            if (bucket[i].getTargetSpeed() > actualSpeed)
            {
                stepper[i].setMaxSpeed(bucket[i].getTargetSpeed());
                if (bucket[i].isDecelerating())
                {
                    stepper[i].moveTo(bucket[i].getTargetPosition());
                    bucket[i].setDecelerating(false);
                }
            }
            else if ((bucket[i].getTargetSpeed() < actualSpeed) && !bucket[i].isDecelerating())
            {
                bucket[i].setDecelerating(true);
                stepper[i].stop();
            }
        }
        else if (bucket[i].isDecelerating() && (actualSpeed < bucket[i].getTargetSpeed()))
        {
            bucket[i].setDecelerating(false);
            stepper[i].setMaxSpeed(bucket[i].getTargetSpeed());
            stepper[i].moveTo(bucket[i].getTargetPosition());
        }

        stepper[i].run();
    }
}

void setup()
{
    // Initialize serial port.

    serialPort.init(9600);
    serialPort.setCallback(&processBuffer);

    // Initialize stepper motors.

    for (byte i = 0; i < 2; i++)
    {
        stepper[i].setMaxSpeed(bucket[i].getMaxSpeed());
        stepper[i].setAcceleration(bucket[i].getAcceleration());
    }

    // Initialize interrupt.

    TimerInterrupt::setCallback(&run);
    TimerInterrupt::start(INTERRUPT_TIME);

    // Send a message to the client that Arduino is ready to read/write data.

    sendReadyMessage();
}

void loop()
{
    serialPort.update(); // Read/write serial port data.
}