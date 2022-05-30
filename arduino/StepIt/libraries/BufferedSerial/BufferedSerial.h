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

#ifndef BUFFERED_SERIAL_H
#define BUFFERED_SERIAL_H

#include <Arduino.h>
#include "RoundBuffer.h"

// This class read and write PPP frames through the serial port.
// As a general rule, when int, long or float are sent, the Most Significant
// Byte (MSB) is sent first.

class BufferedSerial
{

public:
    BufferedSerial(unsigned int inBuffersize, unsigned int outBufferSize);

    boolean isBusyWriting();
    void setCallback(void (*callback)(byte requestId, RoundBuffer *));

    void init(int baudRate);

    /**
     * This must be called in each arduino loop iteration. It simulatenously
     * read and write from and to the serial port.
     */
    void update();

    /** Write the given buffer. */
    void write(RoundBuffer *buffer);

    /** Write all data until the output buffer is empty. */
    void flush();

private:
    // This is the CRC-16 calculated on incoming data.
    unsigned short inCRC;

    // Protocol flags.
    static const byte DELIMITER_FLAG = 0x7E;
    static const byte ESCAPE_FLAG = 0x7D;
    static const byte ESCAPED_XOR = 0x20;

    // This is a reserved command indicating a low level response acknowledgement.
    static const byte ACK = 0x7F; // 255 - 128

    // Buffer to read requests.
    RoundBuffer *readBuffer;

    // Buffer to write responses.
    RoundBuffer *writeBuffer;

    // Function to be called when a command is received.
    void (*callback)(byte requestId, RoundBuffer *);

    // Escape bytes that are equal to the DELIMITER_FLAG or ESCAPE_FLAG.
    static void addEscapedByte(RoundBuffer *buffer, byte value);

    // Send client data packed acknowledgement.
    void sendAck(byte requestId);

    // Receiver current state.

    enum State
    {
        WAITING_STATE,
        READING_MESSAGE_STATE,
        ESCAPING_BYTE_STATE
    };

    State state;
};

#endif
