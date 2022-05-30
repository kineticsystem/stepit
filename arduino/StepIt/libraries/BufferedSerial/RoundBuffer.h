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

#ifndef BYTE_BUFFER_H
#define BYTE_BUFFER_H

#include <Arduino.h>

/**
 * This implement a circular byte buffer to be used during a serial port
 * communication.
 *
 * IMPORTANT NOTE
 * Arduino follows the Little Endian convention to store numbers, i.e. a
 * pointer to an int or long gives the address of the LSB (Less Significant
 * Byte).
 * Numbers are sent/received through the network following IEEE 754
 * specification, i.e. Most Significant Byte first.
 * We use the above specifications when converting numbers from bytes to
 * primitive types, and back.
 */
class RoundBuffer
{

public:
    enum Location
    {
        FRONT,
        END
    };

    RoundBuffer();

    // This method initializes the datastore of the buffer which must not be
    // used before this call is made.
    void init(unsigned int bufferSize);

    // Returns how much data is currently stored in the buffer.
    int getSize();

    // Returns the maximum capacity of the buffer
    int getCapacity();

    // Insert a byte at the give location.
    void addByte(byte in, Location location);

    // Remove a byte from the given location.
    byte removeByte(Location location);

    // Insert an int at the end of the buffer.
    void addInt(int in, Location location);

    // Remove an int from at the give location..
    int removeInt(Location location);

    // Insert a long (4 bytes) at the give location..
    void addLong(long in, Location location);

    // Remove a long (4 bytes) from the give location..
    long removeLong(Location location);

    // Insert a float (4 bytes) at the give location..
    void addFloat(float in, Location location);

    // Remove a float (4 bytes) at the give location..
    float removeFloat(Location location);

    // This method resets the buffer into an original state (with no data).
    void clear();

    // Deallocate all buffer data from memory. The buffer won't be usable anymore.
    void dispose();

private:
    byte *data;

    // The maximum capacity of the buffer
    unsigned int capacity;

    // This is the position where a byte is read from using the get method.
    unsigned int position;

    // This record how much data is currently stored in the buffer.
    unsigned int size;
};

#endif
