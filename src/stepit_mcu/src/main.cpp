// Copyright 2023 Giovanni Remigi
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Giovanni Remigi nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <AccelStepper.h>
#include <IntervalTimer.h>
#include "SerialPort.h"
#include "DataBuffer.h"
#include "Buffer.h"
#include "MotorGoal.h"
#include "MotorState.h"
#include "MotorConfig.h"
#include "Guard.h"

#include <array>

constexpr byte NUMBER_OF_MOTORS = 5;

// Stepper connections.
constexpr std::array<byte, NUMBER_OF_MOTORS> STEPPER_EN_PINS = { 0, 3, 6, 9, 13 };     // Red
constexpr std::array<byte, NUMBER_OF_MOTORS> STEPPER_DIR_PINS = { 1, 4, 7, 10, 14 };   // Green
constexpr std::array<byte, NUMBER_OF_MOTORS> STEPPER_STEP_PINS = { 2, 5, 8, 11, 15 };  // Yellow

// Number of steps to achieve a full rotation: 360deg / 1.8deg * 16μsteps = 200
constexpr long STEPS_IN_ONE_ROTATION = 3200;

// This is the information sent by Arduino during the connection handshake.
constexpr char NAME[] = "STEPIT\0";

// Firmware version, reported during the connection handshake.
//
// The major number is the wire compatibility of the protocol: bump it when
// the layout of a request or a response changes, so that a driver talking to
// a controller it cannot understand says so instead of misreading the bytes.
// Bump the minor and the patch number for anything else.
constexpr byte VERSION_MAJOR = 1;
constexpr byte VERSION_MINOR = 0;
constexpr byte VERSION_PATCH = 0;

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

// Interrupts.
IntervalTimer timer;

// Stepper motors.
std::array<AccelStepper, NUMBER_OF_MOTORS> stepper = {
  AccelStepper{ AccelStepper::DRIVER, STEPPER_STEP_PINS[0], STEPPER_DIR_PINS[0] },
  AccelStepper{ AccelStepper::DRIVER, STEPPER_STEP_PINS[1], STEPPER_DIR_PINS[1] },
  AccelStepper{ AccelStepper::DRIVER, STEPPER_STEP_PINS[2], STEPPER_DIR_PINS[2] },
  AccelStepper{ AccelStepper::DRIVER, STEPPER_STEP_PINS[3], STEPPER_DIR_PINS[3] },
  AccelStepper{ AccelStepper::DRIVER, STEPPER_STEP_PINS[4], STEPPER_DIR_PINS[4] }
};

// Limits of this board, found empirically: beyond these the motors lose steps
// and, being open loop, silently drift from the position they report. The host
// may configure anything up to them, never above.
constexpr float MAX_ACCELERATION = 6400.0;  // 2 rotations per square second
constexpr float MAX_SPEED = 9600.0;         // 3 rotations per second

// Tolerance applied when checking a configured value against the limits above.
// A host that asks for exactly the limit sends it in radians, and the round
// trip through radiansToSteps() can land a hair above it.
constexpr float CONFIG_TOLERANCE = 1.001;

// Stepper motors configuration. Starts at the limits and is replaced by
// whatever the host configures.
std::array<MotorConfig, NUMBER_OF_MOTORS> motorConfig = { MotorConfig{ MAX_ACCELERATION, MAX_SPEED },
                                                          MotorConfig{ MAX_ACCELERATION, MAX_SPEED },
                                                          MotorConfig{ MAX_ACCELERATION, MAX_SPEED },
                                                          MotorConfig{ MAX_ACCELERATION, MAX_SPEED },
                                                          MotorConfig{ MAX_ACCELERATION, MAX_SPEED } };

// This structure holds motor goals: the main thread updates the goal and the
// ISR reads it.
std::array<MotorGoal, NUMBER_OF_MOTORS> motorGoal;

// This variable tells the ISR not to read a goal while the main thread is
// inserting a new one.
volatile bool writingMotorGoals = false;

// This structure holds motor states: the ISR updates the state and the main
// thread reads it.
std::array<MotorState, NUMBER_OF_MOTORS> motorState;

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
  for (int i = 0; i < NUMBER_OF_MOTORS; i++)
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
    for (byte i = 0; i < NUMBER_OF_MOTORS; i++)
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
 * identify the correct port where Arduino is connected, together with the
 * motion limits of each motor.
 *
 * The response carries the limits before the name so that the name stays what
 * it has always been, the remaining bytes of the packet:
 *
 *   status        - 1 byte
 *   version       - 3 bytes: major, minor and patch
 *   motor count   - 1 byte
 *   per motor     - id (1 byte), max acceleration (4 bytes), max speed (4
 *                   bytes), both in rad/s^2 and rad/s
 *   name          - the remaining bytes, in ASCII
 *
 * The controller is the authority on the limits: the host cannot know what
 * these motors tolerate, and an open loop stepper pushed beyond them loses
 * steps without anything noticing.
 */
void returnControllerInfo()
{
  responseBuffer.addByte(SUCCESS_MSG, BufferPosition::Tail);
  responseBuffer.addByte(VERSION_MAJOR, BufferPosition::Tail);
  responseBuffer.addByte(VERSION_MINOR, BufferPosition::Tail);
  responseBuffer.addByte(VERSION_PATCH, BufferPosition::Tail);
  responseBuffer.addByte(NUMBER_OF_MOTORS, BufferPosition::Tail);
  for (byte i = 0; i < NUMBER_OF_MOTORS; i++)
  {
    // Reported per motor although the limits are the same for all of them
    // today, so that motors with different mechanics can report different
    // limits without changing the protocol.
    responseBuffer.addByte(i, BufferPosition::Tail);
    responseBuffer.addFloat(stepsToRadians(MAX_ACCELERATION), BufferPosition::Tail);
    responseBuffer.addFloat(stepsToRadians(MAX_SPEED), BufferPosition::Tail);
  }
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
  if (enabled != 0)
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
  // We expect at least 9 bytes (motorId, acceleration, max speed) or a
  // multiple of 9, and no more entries than there are motors to configure.
  const int size = cmd->getSize();
  if (size < 9 || size % 9 != 0 || size / 9 > NUMBER_OF_MOTORS)
  {
    returnCommandError();
    return;
  }

  const int count = size / 9;
  byte ids[NUMBER_OF_MOTORS];
  float accelerations[NUMBER_OF_MOTORS];
  float maxSpeeds[NUMBER_OF_MOTORS];

  // First pass: parse and validate every entry before touching a motor, so a
  // bad entry cannot leave the motors half configured.
  for (int i = 0; i < count; i++)
  {
    ids[i] = cmd->removeByte(BufferPosition::Head);
    accelerations[i] = radiansToSteps(cmd->removeFloat(BufferPosition::Head));
    maxSpeeds[i] = radiansToSteps(cmd->removeFloat(BufferPosition::Head));

    // Reject an id that does not name a physical motor: indexing the motor
    // arrays with it would run past the end.
    if (ids[i] >= NUMBER_OF_MOTORS)
    {
      returnCommandError();
      return;
    }

    // Configuration happens once at start up, so a value this board cannot
    // deliver is reported as an error rather than quietly reduced: the host
    // would otherwise plan motions the hardware never performs and, with no
    // encoders, never notice. Speed commands issued while running still clamp
    // (see speedCommand): failing a control cycle is worse than slowing it.
    // The comparisons reject a NaN too.
    if (!(accelerations[i] > 0.0f) || accelerations[i] > MAX_ACCELERATION * CONFIG_TOLERANCE ||
        !(maxSpeeds[i] > 0.0f) || maxSpeeds[i] > MAX_SPEED * CONFIG_TOLERANCE)
    {
      returnCommandError();
      return;
    }

    // Within tolerance of the limit: keep the limit itself.
    accelerations[i] = min(accelerations[i], MAX_ACCELERATION);
    maxSpeeds[i] = min(maxSpeeds[i], MAX_SPEED);
  }

  // Second pass: apply. The ISR writes the stepper maximum speed too, so hold
  // the goals guard while updating it.
  {
    Guard goalGuard{ writingMotorGoals };
    for (int i = 0; i < count; i++)
    {
      const byte motorId = ids[i];
      motorConfig[motorId] = MotorConfig{ accelerations[i], maxSpeeds[i] };
      stepper[motorId].setAcceleration(accelerations[i]);
      stepper[motorId].setMaxSpeed(maxSpeeds[i]);
      motorGoal[motorId].setSpeed(maxSpeeds[i]);
    }
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
    return;
  }

  while (cmd->getSize() > 0)
  {
    byte motorId = cmd->removeByte(BufferPosition::Head);
    // Reject an id that does not name a physical motor: indexing the motor
    // arrays with it would run past the end.
    if (motorId >= NUMBER_OF_MOTORS)
    {
      returnCommandError();
      return;
    }
    float speed = radiansToSteps(cmd->removeFloat(BufferPosition::Head));
    float absSpeed = min(abs(speed), motorConfig[motorId].getMaxSpeed());

    // We move the stepper at a constant speed to the maximum or the minimum
    // possible positions.
    // LONG_MAX (0x7FFFFFFF) and LONG_MIN (-80000000) do not work, probably
    // because of inner logic in the AccelStepper library, so we chose close
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
    return;
  }

  while (cmd->getSize() > 0)
  {
    byte motorId = cmd->removeByte(BufferPosition::Head);
    // Reject an id that does not name a physical motor: indexing the motor
    // arrays with it would run past the end.
    if (motorId >= NUMBER_OF_MOTORS)
    {
      returnCommandError();
      return;
    }
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
  for (byte i = 0; i < NUMBER_OF_MOTORS; ++i)
  {
    pinMode(STEPPER_EN_PINS[i], OUTPUT);
    digitalWrite(STEPPER_EN_PINS[i], HIGH);
  }

  // Initialize serial port.
  serialPort.init(9600);
  serialPort.setCallback(&processBuffer);

  // Initialize stepper motors.
  for (byte i = 0; i < NUMBER_OF_MOTORS; i++)
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
    for (byte i = 0; i < NUMBER_OF_MOTORS; i++)
    {
      motorGoal[i].setSpeed(0);
    }
  }
}
