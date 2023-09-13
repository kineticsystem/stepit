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

#include "SerialPort.h"
#include "DataBuffer.h"
#include "Freezer.h"
#include "Buffer.h"

#include <cstdint>
#include <vector>

// Freezer pins

// Pin D2 connected to MR of of 74HC595 Shift Register. Low active.
static const byte OVERRIDING_CLEAR_PIN = 2;
// Pin D3 connected to SH_CP of 74HC595 Shift Register.
static const byte CLOCK_PIN = 3;  // Connected to SH_CP of 74HC595 Shift Register.
// Pin D4 connected to DS of 74HC595 Shift Register.
static const byte DATA_PIN = 4;
// Pin D5 connected to OE of 74HC595 Shift Register. Low active.
static const byte OUTPUT_ENABLED_PIN = 5;
// Pin D6 connected to ST_CP of 74HC595 Shift Register.
static const byte LATCH_PIN = 6;
// Pin D7 in read mode.
static const byte INPUT_PIN = 7;

// This is the information sent by the MCU during the connection handshake.
constexpr char NAME[] = "FREEZER\0";

// For each request received, the MCU returns a response with a success or error
// code and possibly some data.

constexpr byte EXECUTE_CMD = 0x70;  // Execute a sequence of bitsets.
constexpr byte INFO_CMD = 0x76;     // Request controller info for connection handshaking.
constexpr byte ECHO_CMD = 0x79;     // Return the given command, for debugging.

// Success response code.
constexpr byte SUCCESS_MSG = 0x11;

// Error response code.
constexpr byte ERROR_MSG = 0x12;

// Class to read and write over a serial port.
SerialPort serialPort{ 200, 200 };

// Message to be written to the serial port, usually a response.
DataBuffer responseBuffer{ 200 };

// This is the component used to control the shift register.
Freezer freezer(OVERRIDING_CLEAR_PIN, CLOCK_PIN, DATA_PIN, OUTPUT_ENABLED_PIN, LATCH_PIN);

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

void executeCommand(DataBuffer* cmd)
{
  // Extract bitsets and delays.
  std::vector<uint16_t> bitsets;
  std::vector<uint32_t> delays;
  while (cmd->getSize() > 0)
  {
    bitsets.push_back(cmd->removeInt(BufferPosition::Head));
    delays.push_back(cmd->removeLong(BufferPosition::Head));
  }

  // Execute.
  for (size_t i = 0; i < bitsets.size(); ++i)
  {
    freezer.write(bitsets[i]);  // Set the hardware bits.
    delay(delays[i]);           // Wait for a given amount of time.
  }
  returnCommandSuccess();
}

/**
 * Send information about the software installed on the MCU to help the client
 * identify the correct port where the MCU is connected.
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
  byte cmdId = cmd->removeByte(BufferPosition::Head);
  if (cmdId == EXECUTE_CMD)
  {
    executeCommand(cmd);
  }
  else if (cmdId == INFO_CMD)
  {
    returnControllerInfo();
  }
  else if (cmdId == ECHO_CMD)
  {
    echoCommand(cmd);
  }
  cmd->clear();
}

void setup()
{
  // Initialize serial port.
  serialPort.init(9600);
  serialPort.setCallback(&processBuffer);
}

void loop()
{
  serialPort.update();  // Read/write serial port data.
}
