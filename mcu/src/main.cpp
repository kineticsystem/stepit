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
#include <IntervalTimer.h>
#include "SerialPort.h"
#include "DataBuffer.h"
#include "Buffer.h"
#include "MotorGoal.h"
#include "MotorState.h"
#include "MotorConfig.h"
#include "Guard.h"

constexpr byte NUBER_OF_MOTORS = 5;

// Stepper connections.

constexpr byte STEPPER0_EN_PIN = 0;    // Red
constexpr byte STEPPER0_DIR_PIN = 1;   // Green
constexpr byte STEPPER0_STEP_PIN = 2;  // Yellow

constexpr byte STEPPER1_EN_PIN = 3;    // Red
constexpr byte STEPPER1_DIR_PIN = 4;   // Green
constexpr byte STEPPER1_STEP_PIN = 5;  // Yellow

constexpr byte STEPPER2_EN_PIN = 6;    // Red
constexpr byte STEPPER2_DIR_PIN = 7;   // Green
constexpr byte STEPPER2_STEP_PIN = 8;  // Yellow

constexpr byte STEPPER3_EN_PIN = 9;     // Red
constexpr byte STEPPER3_DIR_PIN = 10;   // Green
constexpr byte STEPPER3_STEP_PIN = 11;  // Yellow

constexpr byte STEPPER4_EN_PIN = 13;    // Red
constexpr byte STEPPER4_DIR_PIN = 14;   // Green
constexpr byte STEPPER4_STEP_PIN = 15;  // Yellow

// Number of steps to achieve a full rotation: 360deg / 1.8deg * 16μsteps = 200
constexpr long STEPS_IN_ONE_ROTATION = 3200;

// This is the information sent by Arduino during the connection handshake.
constexpr char NAME[] = "STEPIT\0";

// The time between each execution of the run method.
// If the interval is too large the motors will lose steps.
// If the interval is too small the serial communication will be impacted.
constexpr int INTERRUPT_TIME_US = 5;

// If no command is received within this time, motors are stopped and the
// command buffers cleaned.
constexpr int TIMEOUT_MS = 1000;  // Millis

// For each request received, Arduino returns a response with a success or error
// code and possibly some data.

constexpr byte MOVE_CMD = 0x71;                // Move motors to a given position (rad) at maximum speed (rad/s).
constexpr byte STATUS_CMD = 0x75;              // Request motors position (rad) and velocity (rad/s).
constexpr byte INFO_CMD = 0x76;                // Request controller info for connection handshaking.
constexpr byte SPEED_CMD = 0x77;               // Move motors at the given velocity (rad/s).
constexpr byte CONFIG_CMD = 0x78;              // Configure the device.
constexpr byte ECHO_CMD = 0x79;                // Return the given command, for debugging.
constexpr byte SET_MOTORS_ENABLED_CMD = 0x7A;  // Enable interrupt to control the motors.

// Success response code.
constexpr byte SUCCESS_MSG = 0x11;

// Error response code.
constexpr byte ERROR_MSG = 0x12;

// Last time a message was received.
long int lastMessageReceivedTime = 0;

// Each time Arduino sends a command, this number increases.
byte messageId = 0;

// Interrupts.
IntervalTimer timer;

// Stepper motors.
AccelStepper stepper[] = { AccelStepper{ AccelStepper::DRIVER, STEPPER0_STEP_PIN, STEPPER0_DIR_PIN },
                           AccelStepper{ AccelStepper::DRIVER, STEPPER1_STEP_PIN, STEPPER1_DIR_PIN },
                           AccelStepper{ AccelStepper::DRIVER, STEPPER2_STEP_PIN, STEPPER2_DIR_PIN },
                           AccelStepper{ AccelStepper::DRIVER, STEPPER3_STEP_PIN, STEPPER3_DIR_PIN },
                           AccelStepper{ AccelStepper::DRIVER, STEPPER4_STEP_PIN, STEPPER4_DIR_PIN } };

// Stepper motors configuration: acceleration and max speed.
MotorConfig motorConfig[] = { MotorConfig{ 6000.0, 100530.964915 }, MotorConfig{ 6000.0, 100530.964915 },
                              MotorConfig{ 6000.0, 100530.964915 }, MotorConfig{ 6000.0, 100530.964915 },
                              MotorConfig{ 6000.0, 100530.964915 } };

// This structure holds motor goals: the main thread updates the goal and the
// ISR reads it.
MotorGoal motorGoal[] = { MotorGoal{}, MotorGoal{}, MotorGoal{}, MotorGoal{}, MotorGoal{} };

// This variable tells the ISR not to read a goal while the main thread is
// inserting a new one.
volatile bool writingMotorGoals = false;

// This structure holds motor states: the ISR updates the state and the main
// thread reads it.
MotorState motorState[] = { MotorState{}, MotorState{}, MotorState{}, MotorState{}, MotorState{} };

// This variable tells the ISR not to override the state while the main thread
// is reading it.
volatile bool readingMotorStates = false;

// Class to read and write over a serial port.
SerialPort serialPort{ 200, 200 };

// Message to be written to the serial port, usually a response.
DataBuffer responseBuffer{ 200 };

/**
 * Convert a rotation angle in radians to the number of steps.
 * @param angle The angle in radians.
 * @return The angle in the number of steps.
 */
float radiansToSteps(float angle)
{
  return (angle * 0.5f / PI) * STEPS_IN_ONE_ROTATION;
}

/**
 * Convert a rotation angle in the number of steps to radians.
 * @param steps The angle in steps.
 * @return The angle in radians.
 */
float stepsToRadians(float steps)
{
  return (steps / STEPS_IN_ONE_ROTATION) * 2.0f * PI;
}

/**
 * @brief Compute the sign of the given number.
 * @param value The number to calculate the sign of.
 * @return The sign of the given number.
 */
float sgn(float value)
{
  if (value > 0)
  {
    return 1.0f;
  }
  if (value < 0)
  {
    return -1.0f;
  }
  return 0.0f;
}

/**
 * Method called by an interrupt timer to move the stepper motors.
 */
void run()
{
  for (int i = 0; i < NUBER_OF_MOTORS; i++)
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
    // accelerate to match it. When we reduce the maximum speed, the motor will
    // suddenly drop the velocity without decelerating.
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

/** Send a successful response. */
void returnCommandSuccess()
{
  responseBuffer.addByte(SUCCESS_MSG, BufferPosition::Tail);
  serialPort.write(&responseBuffer);
}

/** Send an error response. */
void returnCommandError()
{
  responseBuffer.addByte(ERROR_MSG, BufferPosition::Tail);
  serialPort.write(&responseBuffer);
}

/**
 * When a Status command is received, send back the position of the motor (rad) and
 * its speed (rad/s).
 */
void returnStatus()
{
  responseBuffer.addByte(SUCCESS_MSG, BufferPosition::Tail);
  {
    Guard stateGuard{ readingMotorStates };
    for (byte i = 0; i < NUBER_OF_MOTORS; i++)
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
void returnControllerInfo()
{
  responseBuffer.addByte(SUCCESS_MSG, BufferPosition::Tail);
  for (int i = 0; NAME[i] != '\0'; i++)
  {
    responseBuffer.addByte(NAME[i], BufferPosition::Tail);
  }
  serialPort.write(&responseBuffer);
}

/**
 * Enable or disable the Service Interrupt Routine (SIR) to move the motors.
 * @param enabled True to enable the motors control, false to disable.
 */
void setMotorsEnabled(DataBuffer* cmd)
{
  byte enabled = cmd->removeByte(BufferPosition::Head);
  if (enabled == 0)
  {
    timer.begin(run, INTERRUPT_TIME_US);
  }
  else
  {
    timer.end();
  }
  returnCommandSuccess();
}

/**
 * Configure the motors.
 * @param cmd The move command.
 */
void configureCommand(DataBuffer* cmd)
{
  // We expect at least 9 bytes (motorId, speed) or a multiple of 9.
  if (cmd->getSize() < 9 || cmd->getSize() % 9 != 0)
  {
    returnCommandError();
  };

  while (cmd->getSize() > 0)
  {
    byte motorId = cmd->removeByte(BufferPosition::Head);
    float acceleration = radiansToSteps(cmd->removeFloat(BufferPosition::Head));
    float max_speed = radiansToSteps(cmd->removeFloat(BufferPosition::Head));

    // TODO: do something.
  }
  returnCommandSuccess();
}

/**
 * Move the motors at a given speed.
 * @param cmd The move command.
 */
void speedCommand(DataBuffer* cmd)
{
  // We expect at least 5 bytes (motorId, speed) or a multiple of 5.
  if (cmd->getSize() < 5 || cmd->getSize() % 5 != 0)
  {
    returnCommandError();
  };

  while (cmd->getSize() > 0)
  {
    byte motorId = cmd->removeByte(BufferPosition::Head);
    float speed = radiansToSteps(cmd->removeFloat(BufferPosition::Head));
    float absSpeed = min(abs(speed), motorConfig[motorId].getMaxSpeed());

    // We move the stepper at a constant speed to the maximum or the minimum
    // possible positions.
    // LONG_MAX (0x7FFFFFFF) and LONG_MIN (-80000000) do not work, probably
    // because of inner logic in AccelStepper library, so we chose close
    // enough values.

    Guard writeGuard{ writingMotorGoals };
    if (speed >= 0)
    {
      motorGoal[motorId].setPosition(0x7F000000);  // Move to +infinity.
    }
    else
    {
      motorGoal[motorId].setPosition(-0x7F000000);  // Move to -infinity.
    }
    motorGoal[motorId].setSpeed(absSpeed);
  }

  returnCommandSuccess();
}

/**
 * Move the motors at maximum speed to a given position.
 * @param cmd The move command.
 */
void moveCommand(DataBuffer* cmd)
{
  // We expect at least 5 bytes (motorId, position) or a multiple of 5.
  if (cmd->getSize() < 5 || cmd->getSize() % 5 != 0)
  {
    returnCommandError();
  };

  while (cmd->getSize() > 0)
  {
    byte motorId = cmd->removeByte(BufferPosition::Head);
    long position = radiansToSteps(cmd->removeFloat(BufferPosition::Head));
    float speed = motorConfig[motorId].getMaxSpeed();

    Guard goalGuard{ writingMotorGoals };
    motorGoal[motorId].setPosition(position);
    motorGoal[motorId].setSpeed(speed);
  }
  returnCommandSuccess();
}

/**
 * Echo back the given command to test the serial communication.
 * @param cmd The command to echo.
 */
void echoCommand(DataBuffer* cmd)
{
  responseBuffer.addByte(ECHO_CMD, BufferPosition::Tail);
  while (cmd->getSize() > 0)
  {
    byte ch = cmd->removeByte(BufferPosition::Head);
    responseBuffer.addByte(ch, BufferPosition::Tail);
  }
  serialPort.write(&responseBuffer);
}

/**
 * Decode the input command and execute the requested action.
 * @param cmd The the command data.
 */
void processBuffer(DataBuffer* cmd)
{
  lastMessageReceivedTime = millis();
  byte cmdId = cmd->removeByte(BufferPosition::Head);

  if (cmdId == STATUS_CMD)
  {
    returnStatus();
  }
  else if (cmdId == MOVE_CMD)
  {
    moveCommand(cmd);
  }
  else if (cmdId == SPEED_CMD)
  {
    speedCommand(cmd);
  }
  else if (cmdId == INFO_CMD)
  {
    returnControllerInfo();
  }
  else if (cmdId == SET_MOTORS_ENABLED_CMD)
  {
    setMotorsEnabled(cmd);
  }
  else if (cmdId == CONFIG_CMD)
  {
    configureCommand(cmd);
  }
  else if (cmdId == ECHO_CMD)
  {
    echoCommand(cmd);
  }
  cmd->clear();
}

void setup()
{
  // Enable steppers.
  pinMode(STEPPER0_EN_PIN, OUTPUT);
  digitalWrite(STEPPER0_EN_PIN, HIGH);

  pinMode(STEPPER1_EN_PIN, OUTPUT);
  digitalWrite(STEPPER1_EN_PIN, HIGH);

  pinMode(STEPPER2_EN_PIN, OUTPUT);
  digitalWrite(STEPPER2_EN_PIN, HIGH);

  pinMode(STEPPER3_EN_PIN, OUTPUT);
  digitalWrite(STEPPER3_EN_PIN, HIGH);

  pinMode(STEPPER4_EN_PIN, OUTPUT);
  digitalWrite(STEPPER4_EN_PIN, HIGH);

  // Initialize serial port.
  serialPort.init(9600);
  serialPort.setCallback(&processBuffer);

  // Initialize stepper motors.
  for (byte i = 0; i < NUBER_OF_MOTORS; i++)
  {
    stepper[i].setMaxSpeed(motorConfig[i].getMaxSpeed());
    stepper[i].setAcceleration(motorConfig[i].getAcceleration());
    stepper[i].setCurrentPosition(0);
    motorGoal[i].setSpeed(motorConfig[i].getMaxSpeed());
  }

  // Initialize interrupt.
  timer.begin(run, INTERRUPT_TIME_US);

  lastMessageReceivedTime = millis();
}

void loop()
{
  serialPort.update();  // Read/write serial port data.

  // This is a safety measure; the client must keep sending messages.
  // If no message (command or query) is received within the given timeframe,
  // motors are stopped.

  long timeFromLastMessage = millis() - lastMessageReceivedTime;
  if (timeFromLastMessage > TIMEOUT_MS)
  {
    // TODO: clean input and output buffers.
    // serialPort.reset();

    // Stop all motors.
    Guard goalGuard{ writingMotorGoals };
    for (byte i = 0; i < NUBER_OF_MOTORS; i++)
    {
      motorGoal[i].setSpeed(0);
    }
  }
}
