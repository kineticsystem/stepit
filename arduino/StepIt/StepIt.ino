/*
 * Copyright (c) 2022, Giovanni Remigi
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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

constexpr byte STEPPER0_STEP_PIN = 9;
constexpr byte STEPPER0_DIR_PIN = 10;
constexpr byte STEPPER1_STEP_PIN = 11;
constexpr byte STEPPER1_DIR_PIN = 12;

// Number of full steps to achieve a rotation: 200 * 16 µ-steps
constexpr long TOTAL_STEPS = 3200;

// This is the information sent by Arduino during the connection handshake.
constexpr char NAME[] = "STEPPIT\0";

// The time between each execution of the run method.
constexpr int INTERRUPT_TIME_US = 90;

// If no command is received within this time, motors are stopped and the
// command buffer cleaned.
constexpr int TIMEOUT_MS = 1000;

// For each command received, Arduino returns a response with success or error
// code and possibly some data.

constexpr byte MOVE_CMD = 0x71;                // Move motors to a given position (rad) at maximum speed (rad/s).
constexpr byte STATUS_CMD = 0x75;              // Request motors position (rad) and velocity (rad/s).
constexpr byte INFO_CMD = 0x76;                // Request controller info for connection handshaking.
constexpr byte SPEED_CMD = 0x77;               // Move motors at the given velocity (rad/s).
constexpr byte SET_MOTORS_ENABLED_CMD = 0x7A;  // Enable interrupt to control the motors.

// Arduino reboots in around two seconds when a serial connection is initiated.
// To notify the client that Arduino is ready to receive and transmit data, a ready message
// is sent.
constexpr byte READY_MSG = 0x80;

// Success response code.
constexpr byte SUCCESS_MSG = 0x11;

// Error response code.
constexpr byte ERROR_MSG = 0x12;

// Last time a message was received.
long int timeMs = 0;

// Each time Arduino sends a command, this number increases.
byte messageId = 0;

// Stepper motors.
AccelStepper stepper[] = { AccelStepper{ AccelStepper::DRIVER, STEPPER0_STEP_PIN, STEPPER0_DIR_PIN },
                           AccelStepper{ AccelStepper::DRIVER, STEPPER1_STEP_PIN, STEPPER1_DIR_PIN } };

// Stepper motors configuration: acceleration and max speed.
// If we make motors run too fast, Arduino will not be able to communicate through
// the serial port fast enough.
// AccelStepper library reports that on Arduino at 16Mhz we cannot achieve speeds
// higher than 4000 steps per second on a single motor. With two motors running
// at the same time, the maximum speed of each motor is 2000 steps per second.
// To give Arduino time to read and write from and to the serial port, the speed
// must be reduced further.
// http://www.airspayce.com/mikem/arduino/AccelStepper/classAccelStepper.html
MotorConfig motorConfig[] = { MotorConfig{ 1500.0, 1500.0 }, MotorConfig{ 1500.0, 1500.0 } };

// This structure holds motor goals: the main thread updates the goal and the
// ISR reads it.
MotorGoal motorGoal[] = { MotorGoal{}, MotorGoal{} };

// This variable tells the ISR not to read a goal while the main thread is
// inserting a new one.
volatile bool writingMotorGoals = false;

// This structure holds motor states: the ISR updates the state and the main
// thread reads it.
MotorState motorState[] = { MotorState{}, MotorState{} };

// This variable tells the ISR not to override the state while the main thread
// is reading it.
volatile bool readingMotorStates = false;

// Class to read and write over a serial port.
SerialPort serialPort{ 50, 50 };

// Message to be written to the serial port, usually a response.
DataBuffer responseBuffer{ 50 };

/**
 * Convert a rotation angle in radians to a number of steps.
 * @param radians The angle in radians.
 * @return The angle in number of steps.
 */
float radiansToSteps(float radians)
{
  return 0.5f * radians * TOTAL_STEPS / PI;
}

/**
 * Convert a rotation angle in number of steps to radians.
 * @param steps The angle in steps.
 * @return The angle in radians.
 */
float stepsToRadians(float steps)
{
  return 2.0f * PI * steps / TOTAL_STEPS;
}

/**
 * @brief Compute the sign of the given number.
 * @param val The number to calculate the sign of.
 * @return The sign of the given number.
 */
float sgn(float val)
{
  if (val > 0)
  {
    return 1.0f;
  }
  if (val < 0)
  {
    return -1.0f;
  }
  return 0.0f;
}

/**
 * Send a ready message when Arduino is ready to communicate.
 */
void sendReadyMessage()
{
  responseBuffer.addByte(messageId++, BufferPosition::Tail);
  responseBuffer.addByte(READY_MSG, BufferPosition::Tail);
  serialPort.write(&responseBuffer);
}

/**
 * Send a successful response when Arduino correctly executes a command.
 */
void returnCommandSuccess(byte requestId)
{
  responseBuffer.addByte(requestId, BufferPosition::Tail);
  responseBuffer.addByte(SUCCESS_MSG, BufferPosition::Tail);
  serialPort.write(&responseBuffer);
}

/**
 * When a Status command is received, send back the position of the motor (rad) and
 * its speed (rad/s).
 */
void returnStatus(byte requestId)
{
  responseBuffer.addByte(requestId, BufferPosition::Tail);
  responseBuffer.addByte(SUCCESS_MSG, BufferPosition::Tail);
  {
    Guard stateGuard{ readingMotorStates };
    for (byte i = 0; i < 2; i++)
    {
      responseBuffer.addByte(i, BufferPosition::Tail);
      responseBuffer.addFloat(stepsToRadians(motorState[i].getPosition()), BufferPosition::Tail);
      responseBuffer.addFloat(stepsToRadians(motorState[i].getSpeed()), BufferPosition::Tail);
      responseBuffer.addFloat(stepsToRadians(motorState[i].getDistanceToGo()), BufferPosition::Tail);
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
  responseBuffer.addByte(requestId, BufferPosition::Tail);
  responseBuffer.addByte(SUCCESS_MSG, BufferPosition::Tail);
  for (int i = 0; NAME[i] != '\0'; i++)
  {
    responseBuffer.addByte(NAME[i], BufferPosition::Tail);
  }
  serialPort.write(&responseBuffer);
}

/**
 * Enable or disable the Service Interrupt Routine (SIR) to move the motors.
 * @param requestId The id of the request.
 * @param enabled True to enable the motors control, false to disable.
 */
void setMotorsEnabled(byte requestId, DataBuffer* cmd)
{
  byte enabled = cmd->removeByte(BufferPosition::Head);
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
 * Move the motors at given speed.
 * @param requestId The request id.
 * @param cmd The move command.
 */
void speedCommand(byte requestId, DataBuffer* cmd)
{
  for (int i = 0; i < 2; ++i)
  {
    byte motorId = cmd->removeByte(BufferPosition::Head);
    float speed = radiansToSteps(cmd->removeFloat(BufferPosition::Head));  // steps/s

    float absSpeed = min(abs(speed), motorConfig[motorId].getMaxSpeed());
    float sgnSpeed = sgn(speed);

    long position = motorState[motorId].getPosition() + sgnSpeed * TOTAL_STEPS;  // <-- ATTENTION, reading position.

    Guard goalGuard{ writingMotorGoals };

    // Move the motor a full rotation in the direction dictated by the speed sign.
    motorGoal[motorId].setPosition(position);
    motorGoal[motorId].setSpeed(absSpeed);
  }

  returnCommandSuccess(requestId);
}

/**
 * Move the motors at maximum speed to a given position.
 * @param requestId The id of the request.
 * @param cmd The move command.
 */
void moveCommand(byte requestId, DataBuffer* cmd)
{
  for (int i = 0; i < 2; ++i)
  {
    byte motorId = cmd->removeByte(BufferPosition::Head);
    long position = radiansToSteps(cmd->removeFloat(BufferPosition::Head));
    float speed = motorConfig[motorId].getMaxSpeed();

    Guard goalGuard{ writingMotorGoals };
    motorGoal[motorId].setPosition(position);
    motorGoal[motorId].setSpeed(speed);
  }
  returnCommandSuccess(requestId);
}

/**
 * Decode the input command and execute the requested action.
 * @param msg The input message containing the command.
 */
void processBuffer(byte requestId, DataBuffer* cmd)
{
  byte cmdId = cmd->removeByte(BufferPosition::Head);

  if (cmdId == STATUS_CMD)
  {
    returnStatus(requestId);
  }
  else if (cmdId == MOVE_CMD)
  {
    moveCommand(requestId, cmd);
  }
  else if (cmdId == SPEED_CMD)
  {
    speedCommand(requestId, cmd);
  }
  else if (cmdId == INFO_CMD)
  {
    returnControllerInfo(requestId);
  }
  else if (cmdId == SET_MOTORS_ENABLED_CMD)
  {
    setMotorsEnabled(requestId, cmd);
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

    // When we increase the maximum speed using AccellStepper, the motor will
    // accelerate to match the new configuration. When we reduce the maximum
    // speed, the motor will suddently drop the velocity without decelerating.
    // The following code adds support to decelerate until reaching the lower
    // maximum speed.

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
    stepper[i].setCurrentPosition(0);
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
  serialPort.update();  // Read/write serial port data.

  // This is a safety measure; the client must keep sending messages.
  // If no command is received within the given timeframe, motors are
  // stopped.

  long newTimeMs = millis();
  long timeDiffMs = newTimeMs - timeMs;
  if (timeDiffMs > TIMEOUT_MS)
  {
    // Stop all motors.
    Guard goalGuard{ writingMotorGoals };
    for (byte i = 0; i < 2; i++)
    {
      motorGoal[i].setSpeed(0);
    }
  }
  timeMs = newTimeMs;
}
