/*
 * Copyright (C) 2014 Remigi Giovanni
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
#include <BufferedSerial.h>
#include <RoundBuffer.h>
#include <Freezer.h>
#include "SharedBucket.h"

// Freezer conections

// Pin D2 connected to MR of of 74HC595 Shift Register. Low active.
static const byte OVERRIDING_CLEAR_PIN = 2;
// Pin D3 connected to SH_CP of 74HC595 Shift Register.
static const byte CLOCK_PIN = 3; // Connected to SH_CP of 74HC595 Shift Register.
// Pin D4 connected to DS of 74HC595 Shift Register.
static const byte DATA_PIN = 4;
// Pin D5 connected to OE of 74HC595 Shift Register. Low active.
static const byte OUTPUT_ENABLED_PIN = 5;
// Pin D6 connected to ST_CP of 74HC595 Shift Register.
static const byte LATCH_PIN = 6;
// Pin D7 in read mode.
static const byte INPUT_PIN = 7;

// Motors connections.

static const byte STEPPER0_STEP_PIN = 9;
static const byte STEPPER0_DIR_PIN = 10;
static const byte STEPPER1_STEP_PIN = 11;
static const byte STEPPER1_DIR_PIN = 12;

// This is the information sent by Arduino during the connection handshake.
const char name[] = "STEPPIT\0";

const int INTERRUPT_TIME = 90; // Microseconds.

// VERY IMPORTANT
// Commands sent from Arduino to client have the most significative bit set to 1.
// Commands sent from client to Arduino and corresponsing response have the most significative bit set to 0.
// There is a special reserved internal ACK response 0xFF used both by Arduino and client.

// Input commands.

byte MOVE_CMD = 0x70;                  // Move motors a given amount of steps.
byte MOVE_TO_CMD = 0x71;               // Move motors to a given position.
byte STOP_CMD = 0x72;                  // Stop motors.
byte SET_ACCELERATION_CMD = 0x73;      // Set motors acceleration.
byte STATUS_CMD = 0x75;                // Request motors status, position and speed.
byte INFO_CMD = 0x76;                  // Request controller info for connection handshaking.
byte SET_TARGET_SPEED_CMD = 0x77;      // Set motors target relative speed (0-100).
byte SET_MAX_SPEED_CMD = 0x78;         // Set motors max absolute speed (step/s).
byte SET_CURRENT_POSITION_CMD = 0x79;  // Set the motor current position.
byte SET_INTERRUPT_ENABLED_CMD = 0x7A; // Enable interrupts.
byte SWITCH_LIGHT_CMD = 0x7B;          // Switch light on/off.
byte SHOOT_CMD = 0x7C;                 // Shoot the camera.

// When a serial connection is initiated Arduino is rebooted and this takes up to 2 seconds.
// To notify the client that Arduino is ready to receive and trasmit data, a ready command
// is sent.
byte READY_CMD = 0x80; // Command send by Arduino after bootstrap.

// For each command received a response must be returned as soon as possible.
// If a command doesn't require a specific response, a default success response is
// delivered.
// If a command requires some information (e.g. the internal motor status) a response
// containing the requested information is delivered.

// Return a message indicating that a correct command has been successfully received.
byte SUCCESS_MSG = 0x00;

// Return motors status.
byte STATUS_MSG = 0x01;

// Return controller info for connection handshaking.
byte INFO_MSG = 0x03;

// Each time Arduino send a command this number is incremented.
byte arduinoCommandId = 0;

// This is the component used to control the shift register.
Freezer freezer(OVERRIDING_CLEAR_PIN, CLOCK_PIN, DATA_PIN, OUTPUT_ENABLED_PIN, LATCH_PIN);

// Stepper motors.

AccelStepper stepper[2] = {
    AccelStepper(AccelStepper::DRIVER, STEPPER0_STEP_PIN, STEPPER0_DIR_PIN),
    AccelStepper(AccelStepper::DRIVER, STEPPER1_STEP_PIN, STEPPER1_DIR_PIN)};

// This is the structure used to safetly exchange data beween the sub routine and the main loop.
SharedBucket bucket[2] = {
    SharedBucket(1000.0, 500.0),
    SharedBucket(1000.0, 500.0)};

// Buffer to read/write commands.

BufferedSerial serial(256, 256);
RoundBuffer bufferOut;

// The last delivered reponse.
// If the client doesn't receive a correct reponse (e.g. the CRC is incorrect) it will reissue
// the same request after a default timeout for a maximum specified number of times.
// Arduino doesn't process again the command but simply send back the previous response.
RoundBuffer cachedResponse;

void sendReadyCommand()
{
    bufferOut.addByte(arduinoCommandId, RoundBuffer::END);
    bufferOut.addByte(READY_CMD, RoundBuffer::END);
    serial.write(&bufferOut);
    arduinoCommandId++;
}

/**
 * Default reponse when a correct command, not requiring information, has been received.
 */
void returnCommandSuccess(byte cmdId)
{
    bufferOut.addByte(cmdId, RoundBuffer::END);
    bufferOut.addByte(SUCCESS_MSG, RoundBuffer::END);
    serial.write(&bufferOut);
}

/**
 * When a Status command is received, send back the motors position and
 * speed.
 */
void returnStatus(byte cmdId)
{
    bufferOut.addByte(cmdId, RoundBuffer::END);
    bufferOut.addByte(STATUS_MSG, RoundBuffer::END);
    for (int i = 0; i < 2; i++)
    {
        bufferOut.addLong(bucket[i].getCurrentPosition(), RoundBuffer::END);
        float speed = 100.0 * bucket[i].getSpeed() / bucket[i].getMaxSpeed();
        bufferOut.addFloat(speed, RoundBuffer::END);
        bufferOut.addLong(bucket[i].getDistanceToGo(), RoundBuffer::END);
    }
    serial.write(&bufferOut);
}

/**
 * Send information about the program. It is used at connection handshake
 * to help the client, on the other side of the serial port, identify the
 * correct port where the Arduino is connected.
 */
void returnControllerInfo(byte cmdId)
{
    bufferOut.addByte(cmdId, RoundBuffer::END);
    bufferOut.addByte(INFO_MSG, RoundBuffer::END);
    for (int i = 0; name[i] != '\0'; i++)
    {
        bufferOut.addByte(name[i], RoundBuffer::END);
    }
    serial.write(&bufferOut);
}

/**
 * When running this command Arduino becomes irresponsive.
 */
void shootPicture(byte cmdId, long flashTime)
{
    // When running this command Arduino becomes irresponsive. We must be sure to empty the
    // output buffer before running the shooting command.
    serial.flush();

    TimerInterrupt::stop();
    freezer.write(170u); // 0000000010101010 Pre charge
    delay(100);
    freezer.write(255u); // 0000000011111111 Open shutter
    delay(500);
    freezer.write(65535u); // 1111111111111111 Flash On
    delay(flashTime);
    freezer.write(255u); // 0000000011111111 Flash off
    delay(200);
    freezer.write(0u); // 0000000000000000 Close shutter
    delay(200);
    TimerInterrupt::start(INTERRUPT_TIME);
    returnCommandSuccess(cmdId);
}

void enableInterrupt(byte cmdId, byte enabled)
{
    if (enabled == 0)
    {
        TimerInterrupt::start(INTERRUPT_TIME);
    }
    else
    {
        TimerInterrupt::stop();
    }
    returnCommandSuccess(cmdId);
}

void setMotorTargetSpeed(byte cmdId, byte motorId, float maxSpeedPercentage)
{
    if (maxSpeedPercentage > 100.0)
    {
        maxSpeedPercentage = 100.0;
    }
    bucket[motorId].setTargetSpeed(bucket[motorId].getMaxSpeed() * maxSpeedPercentage / 100.0);
    returnCommandSuccess(cmdId);
}

void motorMove(byte cmdId, byte motorId, long steps)
{
    bucket[motorId].setTargetPosition(bucket[motorId].getCurrentPosition() + steps);
    returnCommandSuccess(cmdId);
}

void motorMoveTo(byte cmdId, byte motorId, long position)
{
    bucket[motorId].setTargetPosition(position);
    returnCommandSuccess(cmdId);
}

/**
 * Decode the input command and execute the requested action.
 * @param msg The input message containing the requested command.
 */
void processBuffer(byte requestId, RoundBuffer *msg)
{
    // State machine.

    if (msg->getSize() > 0)
    {

        // Get the command id.
        byte cmdId = msg->removeByte(RoundBuffer::FRONT);

        if (cmdId == STATUS_CMD && msg->getSize() == 0)
        {
            returnStatus(requestId);
        }
        else if (cmdId == SET_INTERRUPT_ENABLED_CMD && msg->getSize() == 1)
        {
            byte enabled = msg->removeByte(RoundBuffer::FRONT);
            enableInterrupt(requestId, enabled);
        }
        else if (cmdId == SHOOT_CMD && msg->getSize() == 4)
        { // Shoot a picture.
            long flashTime = msg->removeLong(RoundBuffer::FRONT);
            shootPicture(requestId, flashTime);
        }
        else if (cmdId == SET_TARGET_SPEED_CMD && msg->getSize() == 5)
        { // Set motor speed.
            byte motorId = msg->removeByte(RoundBuffer::FRONT);
            float targetSpdPercentage = fabs(msg->removeFloat(RoundBuffer::FRONT)); // Ensure speed is always positive.
            setMotorTargetSpeed(requestId, motorId, targetSpdPercentage);
        }
        else if (cmdId == MOVE_CMD && msg->getSize() == 5)
        { // Move motor by a specified number for steps.
            byte motorId = msg->removeByte(RoundBuffer::FRONT);
            long steps = msg->removeLong(RoundBuffer::FRONT);
            motorMove(requestId, motorId, steps);
        }
        else if (cmdId == MOVE_TO_CMD && msg->getSize() == 5)
        { // Move motor by a specified number for steps.
            byte motorId = msg->removeByte(RoundBuffer::FRONT);
            long position = msg->removeLong(RoundBuffer::FRONT);
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
                byte motorId = msg->removeByte(RoundBuffer::FRONT);
                bucket[motorId].setTargetSpeed(0);
            }
            returnCommandSuccess(requestId);
        }
        else if (cmdId == SET_ACCELERATION_CMD && msg->getSize() == 5)
        { // Set motor acceleration.
            byte motorId = msg->removeByte(RoundBuffer::FRONT);
            float acceleration = msg->removeFloat(RoundBuffer::FRONT);
            bucket[motorId].setAcceleration(acceleration);
            returnCommandSuccess(requestId);
        }
        else if (cmdId == SET_MAX_SPEED_CMD && msg->getSize() == 5)
        { // Set motor max speed acceleration.
            byte motorId = msg->removeByte(RoundBuffer::FRONT);
            float maxSpeed = msg->removeFloat(RoundBuffer::FRONT);
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
                byte motorId = msg->removeByte(RoundBuffer::FRONT);
                long position = msg->removeLong(RoundBuffer::FRONT);
                stepper[motorId].setCurrentPosition(position);
            }
            returnCommandSuccess(requestId);
        }
        else if (cmdId = SWITCH_LIGHT_CMD && msg->getSize() == 1)
        { // Switch light on/off
            byte lightOn = msg->removeByte(RoundBuffer::FRONT);
            if (lightOn == 1)
            {
                freezer.write(65280u); // On  - 1111111100000000
            }
            else
            {
                freezer.write(0u); // Off - 0000000000000000
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
    // Initialize shif regiter controller.

    freezer.initialize();
    freezer.write(0u);

    // Initialize serial port.

    serial.init(9600);
    serial.setCallback(&processBuffer);

    // Initialize command buffer.

    bufferOut.init(256);
    cachedResponse.init(256);

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

    sendReadyCommand();
}

void loop()
{
    serial.update(); // Read/write serial port data.
}
