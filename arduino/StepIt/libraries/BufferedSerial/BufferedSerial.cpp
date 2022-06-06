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

#include "BufferedSerial.h"
#include "CrcUtils.h"

BufferedSerial::BufferedSerial(unsigned int readBuffersize, unsigned int writeBufferSize)
{
    readBuffer = (DataBuffer *)malloc(sizeof(DataBuffer));
    readBuffer->init(readBuffersize);
    writeBuffer = (DataBuffer *)malloc(sizeof(DataBuffer));
    writeBuffer->init(writeBufferSize);
    state = WAITING_STATE;
}

void BufferedSerial::init(int baudRate)
{
    Serial.begin(baudRate);
}

boolean BufferedSerial::isBusyWriting()
{
    return (writeBuffer->getSize() > 0);
}

void BufferedSerial::flush()
{
    while (writeBuffer->getSize() > 0)
    {
        Serial.write(writeBuffer->removeByte(Location::FRONT));
    }
    Serial.flush();
}

void BufferedSerial::addEscapedByte(DataBuffer *buffer, byte value)
{
    if (value == ESCAPE_FLAG || value == DELIMITER_FLAG)
    {
        buffer->addByte(ESCAPE_FLAG, Location::END);
        buffer->addByte(value ^ ESCAPED_XOR, Location::END);
    }
    else
    {
        buffer->addByte(value, Location::END);
    }
}

/**
 * Write a sequence of bytes to the serial port. The sequence is wrapped inside
 * a PPP frame including CRC value. The data frame is described as follows.

 * DELIMITER_FLAG
 * ...
 * data - bytes possibly escaped with ESCAPE_FLAG and ESCAPED_XOR.
 * ...
 * checksum
 * DELIMITER_FLAG
 *
 * The XOR operation on the escaped byte guarantees that during the
 * serial communication neither a Delimiter Flag nor an Escape Flag
 * is part of the packet message.
 */
void BufferedSerial::write(DataBuffer *buffer)
{
    unsigned short outCRC = 0;
    writeBuffer->addByte(DELIMITER_FLAG, Location::END);

    while (buffer->getSize() > 0)
    {
        byte out = buffer->removeByte(Location::FRONT);
        outCRC = CrcUtils::updateCRC(outCRC, out);
        addEscapedByte(writeBuffer, out);
    }

    // Conversion of CRC-16 from Little Endian to Big Endian.
    unsigned char crcLSB = (outCRC & 0xff00) >> 8;
    unsigned char crcMSB = (outCRC & 0x00ff);
    addEscapedByte(writeBuffer, crcMSB);
    addEscapedByte(writeBuffer, crcLSB);

    writeBuffer->addByte(DELIMITER_FLAG, Location::END);
}

/**
 * The receiver reads the incoming stream of data.
 * Once a data frame is identified, the actual unescaped data in the
 * frame is sent to the callback function for further processing.
 */
void BufferedSerial::setCallback(void (*callback)(byte responseId, DataBuffer *))
{
    this->callback = callback;
}

void BufferedSerial::update()
{

    // Read.

    // Get the number of bytes (characters) available for reading from the serial port.
    if (Serial.available() > 0)
    {

        byte in = Serial.read();

        // State machine.

        switch (state)
        {

        case WAITING_STATE:
            if (in == DELIMITER_FLAG)
            {
                inCRC = 0;
                readBuffer->clear();
                state = READING_MESSAGE_STATE;
            }
            break;

        case READING_MESSAGE_STATE:
            if (in == ESCAPE_FLAG)
            { // Ignore the escape character.
                state = ESCAPING_BYTE_STATE;
            }
            else if (in == DELIMITER_FLAG)
            {

                if (readBuffer->getSize() >= 4)
                {

                    // A packet must contain minimum:
                    // 1 - a request id (1 byte);
                    // 2 - a command (1 byte);
                    // 3 - a CRC (2 bytes).

                    // CRC calculated on frame data is zero when the frame data
                    // is correct.
                    if (inCRC == 0)
                    {
                        // Remove the CRC from the buffer.
                        readBuffer->removeInt(Location::END);

                        byte requestId = readBuffer->removeByte(Location::FRONT);
                        callback(requestId, readBuffer);
                    }
                    state = WAITING_STATE;
                }
                inCRC = 0;
                readBuffer->clear();
            }
            else
            {
                inCRC = CrcUtils::updateCRC(inCRC, in);
                readBuffer->addByte(in, Location::END);
            }
            break;

        case ESCAPING_BYTE_STATE:
            inCRC = CrcUtils::updateCRC(inCRC, in ^ ESCAPED_XOR);
            readBuffer->addByte(in ^ ESCAPED_XOR, Location::END);
            state = READING_MESSAGE_STATE;
            break;
        }
    }

    // Write.

    if (writeBuffer->getSize() > 0)
    {
        Serial.write(writeBuffer->removeByte(Location::FRONT));
    }
}
