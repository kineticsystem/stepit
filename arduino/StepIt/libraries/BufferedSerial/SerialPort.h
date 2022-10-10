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
  unsigned short inCRC;

  // Protocol flags.
  static const byte DELIMITER_FLAG = 0x7E;
  static const byte ESCAPE_FLAG = 0x7D;
  static const byte ESCAPED_XOR = 0x20;

  // Buffer to read requests.
  DataBuffer* m_readBuffer;

  // Buffer to write responses.
  DataBuffer* m_writeBuffer;

  // Function to be called when a command is received.
  void (*callback)(byte requestId, DataBuffer*);

  // Escape bytes that are equal to the DELIMITER_FLAG or ESCAPE_FLAG.
  static void addEscapedByte(DataBuffer* buffer, byte value);

  // Receiver current state.

  enum State
  {
    WAITING_STATE,
    READING_MESSAGE_STATE,
    ESCAPING_BYTE_STATE
  };

  State m_state;
};

#endif  // SERIAL_PORT_H
