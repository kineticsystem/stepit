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
#include "MotorGoal.h"
#include "MotorState.h"
#include "MotorConfig.h"
#include "Guard.h"

// Motors connections.

static const byte STEPPER0_STEP_PIN = 9;
static const byte STEPPER0_DIR_PIN = 10;
static const byte STEPPER1_STEP_PIN = 11;
static const byte STEPPER1_DIR_PIN = 12;

// This is the information sent by Arduino during the connection handshake.
const char NAME[] = "STEPPIT\0";

const int INTERRUPT_TIME_MICROSECOND = 90;

// For each command received, Arduino returns a response with success or error
// code and possibly some data.

byte MOVE_CMD = 0x70;                 // Move motors a given amount of steps.
byte MOVE_TO_CMD = 0x71;              // Move motors to a given position.
byte STOP_CMD = 0x72;                 // Stop motors.
byte STOP_ALL_CMD = 0x74;             // Stop all motors.
byte STATUS_CMD = 0x75;               // Request motors status, position and speed.
byte INFO_CMD = 0x76;                 // Request controller info for connection handshaking.
byte SET_TARGET_SPEED_CMD = 0x77;     // Set motors target relative speed (0-100).
byte SET_CURRENT_POSITION_CMD = 0x79; // Set the motor current position.
byte SET_MOTORS_ENABLED_CMD = 0x7A;   // Enable interrupt to control the motors.

// Arduino reboots in around two seconds when a serial connection is initiated.
// To notify the client that Arduino is ready to receive and transmit data, a ready message
// is sent.
byte READY_MSG = 0x80;

// Success response code.
byte SUCCESS_MSG = 0x11;

// Error response code.
byte ERROR_MSG = 0x12;

// Each time Arduino sends a command, this number increases.
byte messageId = 0;

// Stepper motors.
AccelStepper stepper[] = {
    AccelStepper{AccelStepper::DRIVER, STEPPER0_STEP_PIN, STEPPER0_DIR_PIN},
    AccelStepper{AccelStepper::DRIVER, STEPPER1_STEP_PIN, STEPPER1_DIR_PIN}};

// Stepper motors configuration: acceleration and max speed.
// If we make motor run to fast, Arduino will not be able to communicate through
// the serial port.
MotorConfig motorConfig[] = {
    MotorConfig{500.0, 1000.0},  // 2s to reach to max speed.
    MotorConfig{500.0, 1000.0}}; // 2s to reach to max speed.

// This structure holds motor goals: the main thread updates the goal and the
// ISR reads it.
MotorGoal motorGoal[] = {
    MotorGoal{},
    MotorGoal{}};

// This variable tells the ISR not to read a goal while the main thread is
// inserting a new one.
volatile bool writingMotorGoals = false;

// This structure holds motor states: the ISR updates the state and the main
// thread reads it.
MotorState motorState[] = {
    MotorState{},
    MotorState{}};

// This variable tells the ISR not to override the state while the main thread
// is reading it.
volatile bool readingMotorStates = false;

// Class to read and write over a serial port.
SerialPort serialPort{255, 255};

// Message to be written to the serial port, usually a response.
DataBuffer responseBuffer{255};

/**
 * Send a ready message when Arduino is ready to communicate.
 */
void sendReadyMessage()
{
    responseBuffer.addByte(messageId++, Location::END);
    responseBuffer.addByte(READY_MSG, Location::END);
    serialPort.write(&responseBuffer);
}

/**
 * Send a successful response when Arduino correctly executes a command.
 */
void returnCommandSuccess(byte requestId)
{
    responseBuffer.addByte(requestId, Location::END);
    responseBuffer.addByte(SUCCESS_MSG, Location::END);
    serialPort.write(&responseBuffer);
}

/**
 * When a Status command is received, send back the position of the motor and
 * relative speed.
 */
void returnStatus(byte requestId)
{
    responseBuffer.addByte(requestId, Location::END);
    responseBuffer.addByte(SUCCESS_MSG, Location::END);
    {
        Guard stateGuard{readingMotorStates};
        for (int i = 0; i < 2; i++)
        {
            responseBuffer.addLong(motorState[i].getCurrentPosition(), Location::END);
            float relativeSpeed = 100.0 * (motorState[i].getSpeed() / motorConfig[i].getMaxSpeed());
            responseBuffer.addFloat(relativeSpeed, Location::END);
            responseBuffer.addLong(motorState[i].getDistanceToGo(), Location::END);
        }
    }
    serialPort.write(&responseBuffer);
}

/**
 * Send information about the software installed on Arduino to help the client
 * identify the correct port where Arduino is connected.
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
 * Enable or disable the Service Interrupt Routine (SIR) to move the motors.
 * @param requestId The id of the request.
 * @param enabled True to enable the motors control, false to disable.
 */
void setMotorsEnabled(byte requestId, byte enabled)
{
    if (enabled == 0)
    {
        TimerInterrupt::start(INTERRUPT_TIME_MICROSECOND);
    }
    else
    {
        TimerInterrupt::stop();
    }
    returnCommandSuccess(requestId);
}

void setMotorSpeed(byte requestId, byte motorId, float relativeSpeed)
{
    if (relativeSpeed > 100.0)
    {
        relativeSpeed = 100.0;
    }
    {
        Guard goalGuard{writingMotorGoals};
        motorGoal[motorId].setMaxSpeed(motorConfig[motorId].getMaxSpeed() * relativeSpeed / 100.0);
    }
    returnCommandSuccess(requestId);
}

void moveMotor(byte requestId, byte motorId, long steps)
{
    {
        Guard goalGuard{writingMotorGoals};
        Guard stateGuard{readingMotorStates};
        // The motor position may not reflect exactly the one stored in the
        // state while the motor is moving.
        motorGoal[motorId].setTargetPosition(motorState[motorId].getCurrentPosition() + steps);
    }

    returnCommandSuccess(requestId);
}

void moveMotorTo(byte requestId, byte motorId, long position)
{
    {
        Guard goalGuard{writingMotorGoals};
        motorGoal[motorId].setTargetPosition(position);
    }
    returnCommandSuccess(requestId);
}

void stopAllMotors(byte requestId)
{
    {
        Guard goalGuard{writingMotorGoals};
        for (byte i = 0; i < 2; i++)
        {
            motorGoal[i].setMaxSpeed(0);
        }
    }
    returnCommandSuccess(requestId);
}

void stopMotor(byte requestId, byte motorId)
{
    {
        Guard goalGuard{writingMotorGoals};
        motorGoal[motorId].setMaxSpeed(0);
    }
    returnCommandSuccess(requestId);
}

void setMotorCurrentPosition(byte requestId, byte motorId, long position)
{
    stepper[motorId].setCurrentPosition(position);
    returnCommandSuccess(requestId);
}

/**
 * Decode the input command and execute the requested action.
 * @param msg The input message containing the requested command.
 */
void processBuffer(byte requestId, DataBuffer *msg)
{
    byte cmdId = msg->removeByte(Location::FRONT);

    if (cmdId == STATUS_CMD)
    {
        returnStatus(requestId);
    }
    else if (cmdId == SET_MOTORS_ENABLED_CMD)
    {
        byte enabled = msg->removeByte(Location::FRONT);
        setMotorsEnabled(requestId, enabled);
    }
    else if (cmdId == SET_TARGET_SPEED_CMD)
    {
        byte motorId = msg->removeByte(Location::FRONT);
        float targetSpdPercentage = fabs(msg->removeFloat(Location::FRONT));
        setMotorSpeed(requestId, motorId, targetSpdPercentage);
    }
    else if (cmdId == MOVE_CMD)
    {
        byte motorId = msg->removeByte(Location::FRONT);
        long steps = msg->removeLong(Location::FRONT);
        moveMotor(requestId, motorId, steps);
    }
    else if (cmdId == MOVE_TO_CMD)
    {
        byte motorId = msg->removeByte(Location::FRONT);
        long position = msg->removeLong(Location::FRONT);
        moveMotorTo(requestId, motorId, position);
    }
    else if (cmdId == STOP_CMD)
    {
        byte motorId = msg->removeByte(Location::FRONT);
        stopMotor(requestId, motorId);
    }
    else if (cmdId == STOP_ALL_CMD)
    {
        stopAllMotors(requestId);
    }
    else if (cmdId == INFO_CMD)
    {
        returnControllerInfo(requestId);
    }
    else if (cmdId == SET_CURRENT_POSITION_CMD)
    {
        byte motorId = msg->removeByte(Location::FRONT);
        long position = msg->removeLong(Location::FRONT);
        setMotorCurrentPosition(requestId, motorId, position);
    }
    msg->clear();
}

/*
 * Method called by an interrupt timer to move the stepper motors.
 * AccelStepper library only supports acceleration to maximum speed and deceleration
 * to 0 speed. By default, it does not support deceleration to a target speed.
 */
void run()
{
    for (int i = 0; i < 2; i++)
    {
        stepper[i].run();

        // Read motor states.

        float actualSpeed = stepper[i].speed();
        long currentPosition = stepper[i].currentPosition();
        long distanceToGo = stepper[i].distanceToGo();

        if (!readingMotorStates)
        {
            motorState[i].setSpeed(actualSpeed);
            motorState[i].setCurrentPosition(currentPosition);
            motorState[i].setDistanceToGo(distanceToGo);
        }

        if (!writingMotorGoals)
        {
            if (motorGoal[i].getTargetPosition() != motorGoal[i].getOldTargetPosition())
            { // Position has changed.
                motorGoal[i].setOldTargetPosition(motorGoal[i].getTargetPosition());
                if (!motorState[i].isDecelerating())
                {
                    stepper[i].moveTo(motorGoal[i].getTargetPosition());
                }
            }
            else if (motorGoal[i].getMaxSpeed() != motorGoal[i].getOldTargetSpeed())
            { // Target speed has changed
                motorGoal[i].setOldTargetSpeed(motorGoal[i].getMaxSpeed());
                if (motorGoal[i].getMaxSpeed() > actualSpeed)
                {
                    stepper[i].setMaxSpeed(motorGoal[i].getMaxSpeed());
                    if (motorState[i].isDecelerating())
                    {
                        stepper[i].moveTo(motorGoal[i].getTargetPosition());
                        motorState[i].setDecelerating(false);
                    }
                }
                else if ((motorGoal[i].getMaxSpeed() < actualSpeed) && !motorState[i].isDecelerating())
                {
                    motorState[i].setDecelerating(true);
                    stepper[i].stop();
                }
            }
            else if (motorState[i].isDecelerating() && (actualSpeed < motorGoal[i].getMaxSpeed()))
            {
                motorState[i].setDecelerating(false);
                stepper[i].setMaxSpeed(motorGoal[i].getMaxSpeed());
                stepper[i].moveTo(motorGoal[i].getTargetPosition());
            }
        }
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
        stepper[i].setMaxSpeed(motorConfig[i].getMaxSpeed());
        stepper[i].setAcceleration(motorConfig[i].getAcceleration());
        motorGoal[i].setMaxSpeed(motorConfig[i].getMaxSpeed());
    }

    // Initialize interrupt.

    TimerInterrupt::setCallback(&run);
    TimerInterrupt::start(INTERRUPT_TIME_MICROSECOND);

    // Send a message to the client that Arduino is ready to read/write data.

    sendReadyMessage();
}

void loop()
{
    serialPort.update(); // Read/write serial port data.
}
