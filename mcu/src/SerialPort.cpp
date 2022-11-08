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

#include "SerialPort.h"
#include "CrcUtils.h"

// Protocol flags.
constexpr byte DELIMITER_FLAG = 0x7E;
constexpr byte ESCAPE_FLAG = 0x7D;
constexpr byte ESCAPED_XOR = 0x20;

SerialPort::SerialPort(unsigned int readBuffersize, unsigned int writeBufferSize)
{
  m_readBuffer = new DataBuffer{ readBuffersize };
  m_writeBuffer = new DataBuffer{ writeBufferSize };
  m_state = Waiting;
}

SerialPort::~SerialPort()
{
  delete m_readBuffer;
  delete m_writeBuffer;
}

void SerialPort::init(int baudRate)
{
  Serial.begin(baudRate);
}

boolean SerialPort::isBusyWriting()
{
  return (m_writeBuffer->getSize() > 0);
}

void SerialPort::flush()
{
  while (m_writeBuffer->getSize() > 0)
  {
    Serial.write(m_writeBuffer->removeByte(BufferPosition::Head));
  }
  Serial.flush();
}

void SerialPort::addByte(DataBuffer* buffer, byte ch)
{
  if (ch == ESCAPE_FLAG || ch == DELIMITER_FLAG)
  {
    buffer->addByte(ESCAPE_FLAG, BufferPosition::Tail);
    buffer->addByte(ch ^ ESCAPED_XOR, BufferPosition::Tail);
  }
  else
  {
    buffer->addByte(ch, BufferPosition::Tail);
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
void SerialPort::write(DataBuffer* buffer)
{
  unsigned short crc = 0;
  m_writeBuffer->addByte(DELIMITER_FLAG, BufferPosition::Tail);

  while (buffer->getSize() > 0)
  {
    byte out = buffer->removeByte(BufferPosition::Head);
    crc = crc_utils::crc_ccitt_byte(crc, out);
    addByte(m_writeBuffer, out);
  }

  // Conversion of CRC-16 from Little Endian to Big Endian.
  unsigned char crcLSB = (crc & 0xff00) >> 8;
  unsigned char crcMSB = (crc & 0x00ff);
  addByte(m_writeBuffer, crcMSB);
  addByte(m_writeBuffer, crcLSB);
  m_writeBuffer->addByte(DELIMITER_FLAG, BufferPosition::Tail);
}

/**
 * The receiver reads the incoming stream of data.
 * Once a data frame is identified, the actual unescaped data in the
 * frame is sent to the callback function for further processing.
 */
void SerialPort::setCallback(void (*callback)(byte responseId, DataBuffer*))
{
  this->callback = callback;
}

void SerialPort::update()
{
  // Read.

  // Get the number of bytes (characters) available for reading from the serial port.
  if (Serial.available() > 0)
  {
    byte ch = Serial.read();

    // State machine.

    switch (m_state)
    {
      case Waiting:
        if (ch == DELIMITER_FLAG)
        {
          crc = 0;
          m_readBuffer->clear();
          m_state = ReadingMessage;
        }
        break;

      case ReadingMessage:
        if (ch == ESCAPE_FLAG)
        {
          m_state = ReadingEscapedByte;
        }
        else if (ch == DELIMITER_FLAG)
        {
          if (m_readBuffer->getSize() >= 4)
          {
            // A packet must contain minimum:
            // 1 - a request id (1 byte);
            // 2 - a command (1 byte);
            // 3 - a CRC (2 bytes).

            // CRC calculated on frame data is zero when the frame data
            // is correct.
            if (crc == 0)
            {
              // Remove the CRC from the buffer.
              m_readBuffer->removeInt(BufferPosition::Tail);

              byte requestId = m_readBuffer->removeByte(BufferPosition::Head);
              callback(requestId, m_readBuffer);
            }
            m_state = Waiting;
          }
          crc = 0;
          m_readBuffer->clear();
        }
        else
        {
          crc = crc_utils::crc_ccitt_byte(crc, ch);
          m_readBuffer->addByte(ch, BufferPosition::Tail);
        }
        break;

      case ReadingEscapedByte:
        crc = crc_utils::crc_ccitt_byte(crc, ch ^ ESCAPED_XOR);
        m_readBuffer->addByte(ch ^ ESCAPED_XOR, BufferPosition::Tail);
        m_state = ReadingMessage;
        break;
    }
  }

  // Write.

  if (m_writeBuffer->getSize() > 0)
  {
    Serial.write(m_writeBuffer->removeByte(BufferPosition::Head));
  }
}
