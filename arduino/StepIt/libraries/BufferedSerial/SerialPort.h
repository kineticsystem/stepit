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

#ifndef SERIAL_PORT_H
#define SERIAL_PORT_H

#include <Arduino.h>
#include <DataBuffer.h>

// This class reads and writes PPP frames received from the serial port.
// A callback method is used to process incoming frames.
// CRC is calculated on all incoming frames and in case of error the frame is
// dumped.
// As a general rule, when int, long or float are sent, the Most Significant
// Byte (MSB) is sent first.

class SerialPort
{
public:
  explicit SerialPort(unsigned int inBuffersize, unsigned int outBufferSize);
  ~SerialPort();

  boolean isBusyWriting();
  void setCallback(void (*callback)(byte requestId, DataBuffer*));

  /**
   * Initialize the serial connection communication speed.
   */
  void init(int baudRate);

  /**
   * This must be called in each arduino loop iteration. It simulatenously
   * read and write from and to the serial port.
   */
  void update();

  /**
   * Write the given data buffer.
   * @param buffer The buffer contains a sequence of bytes representing
   * a message to the client.
   */
  void write(DataBuffer* buffer);

  /** Write the data buffer. */
  void flush();

private:
  // This is the CRC-16 calculated on incoming data.
  unsigned short crc;

  // Buffer to read requests.
  DataBuffer* m_readBuffer;

  // Buffer to write responses.
  DataBuffer* m_writeBuffer;

  // Function to be called when a command is received.
  void (*callback)(byte requestId, DataBuffer*);

  // Escape bytes that are equal to the DELIMITER_FLAG or ESCAPE_FLAG.
  static void addByte(DataBuffer* buffer, byte ch);

  // Receiver current state.

  enum ReadingState
  {
    Waiting,
    ReadingMessage,
    ReadingEscapedByte
  };

  ReadingState m_state;
};

#endif  // SERIAL_PORT_H
