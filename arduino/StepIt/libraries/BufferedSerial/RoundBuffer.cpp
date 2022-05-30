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

#include "RoundBuffer.h"

RoundBuffer::RoundBuffer()
{
    capacity = 0;
    position = 0;
    size = 0;
    data = 0;
}

int RoundBuffer::getSize()
{
    return size;
}

int RoundBuffer::getCapacity()
{
    return capacity;
}

void RoundBuffer::clear()
{
    position = 0;
    size = 0;
}

void RoundBuffer::dispose()
{
    free(data);
}

void RoundBuffer::addByte(byte in, Location location)
{
    if (size < capacity)
    {
        if (location == END)
        {
            data[(position + size) % capacity] = in;
            size++;
        }
        else
        {
            if (position == 0)
            {
                position = capacity - 1;
            }
            else
            {
                position = (position - 1) % capacity;
            }
            data[position] = in;
            size++;
        }
    }
}

byte RoundBuffer::removeByte(Location location)
{
    byte out = 0;
    if (size > 0)
    {
        if (location == FRONT)
        {
            // Get data byte from the beginning of buffer,
            out = data[position];
            position = (position + 1) % capacity;
            size--;
        }
        else
        {
            out = data[(position + size - 1) % capacity];
            size--;
        }
    }
    return out;
}

// Following IEEE 754 specification an int is always sent/received
// with Most Significant Byte (MSB) first.
// In Arduino an int is stored in memory in Little Endian format,
// it mean a pointer to an int gives the address of the LSB.
// [LSB][...][...][MSB].
void RoundBuffer::addInt(int in, Location location)
{
    byte *pointer = (byte *)&in;
    if (location == END)
    {
        addByte(pointer[1], END);
        addByte(pointer[0], END);
    }
    else
    {
        addByte(pointer[0], FRONT);
        addByte(pointer[1], FRONT);
    }
}

// Following IEEE 754 specification an int is always sent/received
// with Most Significant Byte (MSB) first.
// In Arduino an int is stored in memory in Little Endian format,
// it mean a pointer to an int gives the address of the LSB.
// [LSB][...][...][MSB].
int RoundBuffer::removeInt(Location location)
{
    int ret;
    byte *pointer = (byte *)&ret;
    if (location == FRONT)
    {
        pointer[1] = removeByte(FRONT);
        pointer[0] = removeByte(FRONT);
    }
    else
    {
        pointer[0] = removeByte(END);
        pointer[1] = removeByte(END);
    }
    return ret;
}

// Following IEEE 754 specification a long is always sent/received
// with Most Significant Byte (MSB) first.
// In Arduino a long is stored in memory in Little Endian format,
// it mean a pointer to a long gives the address of the LSB.
// [LSB][...][...][MSB].
void RoundBuffer::addLong(long in, Location location)
{
    byte *pointer = (byte *)&in;
    if (location == END)
    {
        addByte(pointer[3], END);
        addByte(pointer[2], END);
        addByte(pointer[1], END);
        addByte(pointer[0], END);
    }
    else
    {
        addByte(pointer[0], FRONT);
        addByte(pointer[1], FRONT);
        addByte(pointer[2], FRONT);
        addByte(pointer[3], FRONT);
    }
}

// Following IEEE 754 specification a long is always sent/received
// with Most Significant Byte (MSB) first.
// In Arduino a long is stored in memory in Little Endian format,
// it mean a pointer to a long gives the address of the LSB.
// [LSB][...][...][MSB].
long RoundBuffer::removeLong(Location location)
{
    long ret;
    byte *pointer = (byte *)&ret;
    if (location == FRONT)
    {
        pointer[3] = removeByte(FRONT);
        pointer[2] = removeByte(FRONT);
        pointer[1] = removeByte(FRONT);
        pointer[0] = removeByte(FRONT);
    }
    else
    {
        pointer[0] = removeByte(END);
        pointer[1] = removeByte(END);
        pointer[2] = removeByte(END);
        pointer[3] = removeByte(END);
    }
    return ret;
}

// Following IEEE 754 specification a float is always sent/received
// with Sign and Exponent first followed by the Significand in
// Most Significant Byte (MSB) first.
void RoundBuffer::addFloat(float in, Location location)
{
    byte *pointer = (byte *)&in;
    if (location == END)
    {
        addByte(pointer[3], END);
        addByte(pointer[2], END);
        addByte(pointer[1], END);
        addByte(pointer[0], END);
    }
    else
    {
        addByte(pointer[0], FRONT);
        addByte(pointer[1], FRONT);
        addByte(pointer[2], FRONT);
        addByte(pointer[3], FRONT);
    }
}

// Following IEEE 754 specification a float is stored in the following way:
// - Sign: 1bit
// - Exponent: 8 bits
// - Significand: 23 bits.
float RoundBuffer::removeFloat(Location location)
{
    float ret;
    byte *pointer = (byte *)&ret;
    if (location == FRONT)
    {
        pointer[3] = removeByte(FRONT);
        pointer[2] = removeByte(FRONT);
        pointer[1] = removeByte(FRONT);
        pointer[0] = removeByte(FRONT);
    }
    else
    {
        pointer[0] = removeByte(END);
        pointer[1] = removeByte(END);
        pointer[2] = removeByte(END);
        pointer[3] = removeByte(END);
    }
    return ret;
}

void RoundBuffer::init(unsigned int bufferSize)
{
    data = (byte *)malloc(sizeof(byte) * bufferSize);
    capacity = bufferSize;
    position = 0;
    size = 0;
}
