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

#ifndef DATA_BUFFER_H
#define DATA_BUFFER_H

#include <Arduino.h>
#include "Buffer.h"

/**
 * This implements a circular byte buffer to be used during a serial port
 * communication.
 *
 * IMPORTANT NOTE
 * Arduino follows the Little Endian convention to store numbers: a pointer to
 * an int or long gives the address of the LSB (Less Significant Byte).
 * Numbers are sent and received to and from the network following IEEE 754
 * specification, with Most Significant Byte first.
 * We use the above specifications when converting numbers from bytes to
 * primitive types, and back.
 */
class DataBuffer
{
public:
  explicit DataBuffer(unsigned int bufferSize);

  // Returns how much data is currently stored in the buffer.
  int getSize();

  // Returns the maximum capacity of the buffer
  int getCapacity();

  // Insert a byte at the given location.
  void addByte(byte in, BufferPosition location);

  // Remove a byte from the given location.
  byte removeByte(BufferPosition location);

  // Insert an int at the given location.
  void addInt(int in, BufferPosition location);

  // Remove an int from the given location.
  int removeInt(BufferPosition location);

  // Insert a long (4 bytes) at the given location.
  void addLong(long in, BufferPosition location);

  // Remove a long (4 bytes) from the given location.
  long removeLong(BufferPosition location);

  // Insert a float (4 bytes) at the given location.
  void addFloat(float in, BufferPosition location);

  // Remove a float (4 bytes) from the given location.
  float removeFloat(BufferPosition location);

  // This method resets the buffer into an original state (with no data).
  void clear();

private:
  Buffer<byte> m_buffer;
};

#endif  // DATA_BUFFER_H
