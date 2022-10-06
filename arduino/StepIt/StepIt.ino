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

// The time between each execution of the run method.
const int INTERRUPT_TIME_US = 90;

// If no command is received within this time, motors are stopped and the
// command buffer cleaned.
const int TIMEOUT_MS = 1000;

// For each command received, Arduino returns a response with success or error
// code and possibly some data.

byte MOVE_CMD = 0x70;               // Move motors a given amount of steps.
byte MOVE_TO_CMD = 0x71;            // Move motors to a given position.
byte STOP_CMD = 0x72;               // Stop motors.
byte STOP_ALL_CMD = 0x74;           // Stop all motors.
byte STATUS_CMD = 0x75;             // Request motors status, position and speed.
byte INFO_CMD = 0x76;               // Request controller info for connection handshaking.
byte SET_SPEED_CMD = 0x77;          // Set motors target relative speed (0-100).
byte SET_MOTORS_ENABLED_CMD = 0x7A; // Enable interrupt to control the motors.

// Arduino reboots in around two seconds when a serial connection is initiated.
// To notify the client that Arduino is ready to receive and transmit data, a ready message
// is sent.
byte READY_MSG = 0x80;

// Success response code.
byte SUCCESS_MSG = 0x11;

// Error response code.
byte ERROR_MSG = 0x12;

// Last time a message was received.
long int timeMs = 0;

// Each time Arduino sends a command, this number increases.
byte messageId = 0;

// Stepper motors.
AccelStepper stepper[] = {
    AccelStepper{AccelStepper::DRIVER, STEPPER0_STEP_PIN, STEPPER0_DIR_PIN},
    AccelStepper{AccelStepper::DRIVER, STEPPER1_STEP_PIN, STEPPER1_DIR_PIN}};

// Stepper motors configuration: acceleration and max speed.
// If we make motors run too fast, Arduino will not be able to communicate through
// the serial port fast enough.
// AccelStepper library reports that on Arduino at 16Mhz we cannot achieve speeds
// higher than 4000 steps per second on a single motor. With two motors running
// at the same time, the maximum speed of each motor is 2000 steps per second.
// To give Arduino time to read and write from and to the serial port, the speed
// must be reduced further.
// http://www.airspayce.com/mikem/arduino/AccelStepper/classAccelStepper.html
MotorConfig motorConfig[] = {
    MotorConfig{1500.0, 1500.0},
    MotorConfig{1500.0, 1500.0}};

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
            responseBuffer.addLong(motorState[i].getPosition(), Location::END);
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
        TimerInterrupt::start(INTERRUPT_TIME_US);
    }
    else
    {
        TimerInterrupt::stop();
    }
    returnCommandSuccess(requestId);
}

/**
 * Set the motor maximum speed.
 * @param requestId The id of the request.
 * @param motorId The id of the motor to change the speed.
 * @param speed The speed relative to the maximum speed, from 0 to 100.
 */
void setMotorSpeed(byte requestId, byte motorId, float speed)
{
    if (speed > 100.0)
    {
        speed = 100.0;
    }
    {
        Guard goalGuard{writingMotorGoals};
        motorGoal[motorId].setSpeed(motorConfig[motorId].getMaxSpeed() * speed / 100.0);
    }
    returnCommandSuccess(requestId);
}

/**
 * Move the motor a given number of steps
 * @param requestId The id of the request.
 * @param motorId The id of the motor to change the speed.
 * @param steps The number of steps to move the motor.
 */
void moveMotor(byte requestId, byte motorId, long steps)
{
    {
        Guard goalGuard{writingMotorGoals};
        Guard stateGuard{readingMotorStates};
        // The motor position may not reflect exactly the one stored in the
        // state while the motor is moving.
        motorGoal[motorId].setPosition(motorState[motorId].getPosition() + steps);
    }

    returnCommandSuccess(requestId);
}

/**
 * Move the motor to a given position.
 * @param requestId The id of the request.
 * @param motorId The id of the motor to change the speed.
 * @param position The position to move the motor.
 */
void moveMotorTo(byte requestId, byte motorId, long position)
{
    {
        Guard goalGuard{writingMotorGoals};
        motorGoal[motorId].setPosition(position);
    }
    returnCommandSuccess(requestId);
}

/**
 * Stop all motors.
 * @param requestId The id of the request.
 */
void stopAllMotors(byte requestId)
{
    {
        Guard goalGuard{writingMotorGoals};
        for (byte i = 0; i < 2; i++)
        {
            motorGoal[i].setSpeed(0);
        }
    }
    returnCommandSuccess(requestId);
}

/**
 * Stop the given motor.
 * @param requestId The id of the request.
 * @param motorId The id of the motor to stop.
 */
void stopMotor(byte requestId, byte motorId)
{
    {
        Guard goalGuard{writingMotorGoals};
        motorGoal[motorId].setSpeed(0);
    }
    returnCommandSuccess(requestId);
}

/**
 * Decode the input command and execute the requested action.
 * @param msg The input message containing the command.
 */
void processBuffer(byte requestId, DataBuffer *cmd)
{
    byte cmdId = cmd->removeByte(Location::FRONT);

    if (cmdId == STATUS_CMD)
    {
        returnStatus(requestId);
    }
    else if (cmdId == SET_MOTORS_ENABLED_CMD)
    {
        byte enabled = cmd->removeByte(Location::FRONT);
        setMotorsEnabled(requestId, enabled);
    }
    else if (cmdId == SET_SPEED_CMD)
    {
        byte motorId = cmd->removeByte(Location::FRONT);
        float targetSpdPercentage = fabs(cmd->removeFloat(Location::FRONT));
        setMotorSpeed(requestId, motorId, targetSpdPercentage);
    }
    else if (cmdId == MOVE_CMD)
    {
        byte motorId = cmd->removeByte(Location::FRONT);
        long steps = cmd->removeLong(Location::FRONT);
        moveMotor(requestId, motorId, steps);
    }
    else if (cmdId == MOVE_TO_CMD)
    {
        byte motorId = cmd->removeByte(Location::FRONT);
        long position = cmd->removeLong(Location::FRONT);
        moveMotorTo(requestId, motorId, position);
    }
    else if (cmdId == STOP_CMD)
    {
        byte motorId = cmd->removeByte(Location::FRONT);
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
    cmd->clear();
}

/*
 * Method called by an interrupt timer to move the stepper motors.
 */
void run()
{
    for (int i = 0; i < 2; i++)
    {
        stepper[i].run();

        // Write motor states.

        float actualSpeed = stepper[i].speed();
        long currentPosition = stepper[i].currentPosition();
        long distanceToGo = stepper[i].distanceToGo();

        if (!readingMotorStates)
        {
            motorState[i].setSpeed(actualSpeed);
            motorState[i].setPosition(currentPosition);
            motorState[i].setDistanceToGo(distanceToGo);
        }

        // Read motor goals.

        // AccelStepper library only supports acceleration to maximum speed and
        // deceleration to 0 speed. The next code introduces support to
        // decelerate to a given speed.

        if (!writingMotorGoals)
        {
            float absSpeed = fabs(actualSpeed);
            if (motorState[i].isDecelerating())
            {
                if (motorGoal[i].getSpeed() > absSpeed)
                {
                    stepper[i].setMaxSpeed(motorGoal[i].getSpeed());
                    stepper[i].moveTo(motorGoal[i].getPosition());
                    motorState[i].setDecelerating(false);
                }
            }
            else
            {
                stepper[i].moveTo(motorGoal[i].getPosition());
                if (motorGoal[i].getSpeed() > absSpeed)
                {
                    stepper[i].setMaxSpeed(motorGoal[i].getSpeed());
                }
                else if ((motorGoal[i].getSpeed() < absSpeed))
                {
                    motorState[i].setDecelerating(true);
                    stepper[i].stop();
                }
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
        stepper[0].setCurrentPosition(0);
        motorGoal[i].setSpeed(motorConfig[i].getMaxSpeed());
    }

    // Initialize interrupt.

    TimerInterrupt::setCallback(&run);
    TimerInterrupt::start(INTERRUPT_TIME_US);

    // Send a message to the client that Arduino is ready to read/write data.

    sendReadyMessage();

    timeMs = millis();
}

void loop()
{
    serialPort.update(); // Read/write serial port data.

    // This is a safety measure; the client must keep sending messages.
    // If no command is received within the given timeframe, motors are
    // stopped.

    long newTimeMs = millis();
    long timeDiffMs = newTimeMs - timeMs;
    if (timeDiffMs > TIMEOUT_MS)
    {
        // Stop all motors.
        Guard goalGuard{writingMotorGoals};
        for (byte i = 0; i < 2; i++)
        {
            motorGoal[i].setSpeed(0);
        }
    }
    timeMs = newTimeMs;
}
