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

#ifndef DATA_BUFFER_H
#define DATA_BUFFER_H

#include <Arduino.h>
#include <Buffer.h>
#include <Location.h>

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
  void addByte(byte in, Location location);

  // Remove a byte from the given location.
  byte removeByte(Location location);

  // Insert an int at the given location.
  void addInt(int in, Location location);

  // Remove an int from the given location.
  int removeInt(Location location);

  // Insert a long (4 bytes) at the given location.
  void addLong(long in, Location location);

  // Remove a long (4 bytes) from the given location.
  long removeLong(Location location);

  // Insert a float (4 bytes) at the given location.
  void addFloat(float in, Location location);

  // Remove a float (4 bytes) from the given location.
  float removeFloat(Location location);

  // This method resets the buffer into an original state (with no data).
  void clear();

private:
  Buffer<byte> m_buffer;
};

#endif  // DATA_BUFFER_H
