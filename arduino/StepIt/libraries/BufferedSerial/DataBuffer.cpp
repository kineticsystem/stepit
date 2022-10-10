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

#include <DataBuffer.h>

DataBuffer::DataBuffer(unsigned int bufferSize) : m_buffer{ Buffer<byte>{ bufferSize } }
{
}

int DataBuffer::getSize()
{
  return m_buffer.size();
}

int DataBuffer::getCapacity()
{
  return m_buffer.capacity();
}

void DataBuffer::clear()
{
  m_buffer.clear();
}

void DataBuffer::addByte(byte in, Location location)
{
  m_buffer.add(in, location);
}

byte DataBuffer::removeByte(Location location)
{
  return m_buffer.remove(location);
}

// Following IEEE 754 specification a type int is always sent/received
// with Most Significant Byte (MSB) first.
// In Arduino a type int is stored in memory in Little Endian format,
// it means a pointer to an int gives the address of the LSB.
// [LSB][...][...][MSB].
void DataBuffer::addInt(int in, Location location)
{
  byte* pointer = (byte*)&in;
  if (location == Location::END)
  {
    m_buffer.add(pointer[1], Location::END);
    m_buffer.add(pointer[0], Location::END);
  }
  else
  {
    m_buffer.add(pointer[0], Location::FRONT);
    m_buffer.add(pointer[1], Location::FRONT);
  }
}

// Following IEEE 754 specification, a type int is always sent/received
// with Most Significant Byte (MSB) first.
// In Arduino a type int is stored in memory in Little Endian format,
// it means a pointer to an int gives the address of the LSB.
// [LSB][...][...][MSB].
int DataBuffer::removeInt(Location location)
{
  int ret;
  byte* pointer = (byte*)&ret;
  if (location == Location::FRONT)
  {
    pointer[1] = m_buffer.remove(Location::FRONT);
    pointer[0] = m_buffer.remove(Location::FRONT);
  }
  else
  {
    pointer[0] = m_buffer.remove(Location::END);
    pointer[1] = m_buffer.remove(Location::END);
  }
  return ret;
}

// Following IEEE 754 specification, a type long is always sent/received
// with Most Significant Byte (MSB) first.
// In Arduino a long is stored in memory in Little Endian format,
// it means a pointer to a long gives the address of the LSB.
// [LSB][...][...][MSB].
void DataBuffer::addLong(long in, Location location)
{
  byte* pointer = (byte*)&in;
  if (location == Location::END)
  {
    m_buffer.add(pointer[3], Location::END);
    m_buffer.add(pointer[2], Location::END);
    m_buffer.add(pointer[1], Location::END);
    m_buffer.add(pointer[0], Location::END);
  }
  else
  {
    m_buffer.add(pointer[0], Location::FRONT);
    m_buffer.add(pointer[1], Location::FRONT);
    m_buffer.add(pointer[2], Location::FRONT);
    m_buffer.add(pointer[3], Location::FRONT);
  }
}

// Following IEEE 754 specification, a type long is always sent/received
// with Most Significant Byte (MSB) first.
// In Arduino, a long is stored in memory in Little Endian format,
// it means a pointer to a long gives the address of the LSB.
// [LSB][...][...][MSB].
long DataBuffer::removeLong(Location location)
{
  long ret;
  byte* pointer = (byte*)&ret;
  if (location == Location::FRONT)
  {
    pointer[3] = m_buffer.remove(Location::FRONT);
    pointer[2] = m_buffer.remove(Location::FRONT);
    pointer[1] = m_buffer.remove(Location::FRONT);
    pointer[0] = m_buffer.remove(Location::FRONT);
  }
  else
  {
    pointer[0] = m_buffer.remove(Location::END);
    pointer[1] = m_buffer.remove(Location::END);
    pointer[2] = m_buffer.remove(Location::END);
    pointer[3] = m_buffer.remove(Location::END);
  }
  return ret;
}

// Following IEEE 754 specification, a type float is always sent/received
// with Sign and Exponent first followed by the Significand in
// Most Significant Byte (MSB) first.
void DataBuffer::addFloat(float in, Location location)
{
  byte* pointer = (byte*)&in;
  if (location == Location::END)
  {
    m_buffer.add(pointer[3], Location::END);
    m_buffer.add(pointer[2], Location::END);
    m_buffer.add(pointer[1], Location::END);
    m_buffer.add(pointer[0], Location::END);
  }
  else
  {
    m_buffer.add(pointer[0], Location::FRONT);
    m_buffer.add(pointer[1], Location::FRONT);
    m_buffer.add(pointer[2], Location::FRONT);
    m_buffer.add(pointer[3], Location::FRONT);
  }
}

// Following IEEE 754 specification a type float is stored in the following way:
// - Sign: 1bit
// - Exponent: 8 bits
// - Significand: 23 bits.
float DataBuffer::removeFloat(Location location)
{
  float ret;
  byte* pointer = (byte*)&ret;
  if (location == Location::FRONT)
  {
    pointer[3] = m_buffer.remove(Location::FRONT);
    pointer[2] = m_buffer.remove(Location::FRONT);
    pointer[1] = m_buffer.remove(Location::FRONT);
    pointer[0] = m_buffer.remove(Location::FRONT);
  }
  else
  {
    pointer[0] = m_buffer.remove(Location::END);
    pointer[1] = m_buffer.remove(Location::END);
    pointer[2] = m_buffer.remove(Location::END);
    pointer[3] = m_buffer.remove(Location::END);
  }
  return ret;
}
