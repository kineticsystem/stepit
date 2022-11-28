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

#include "DataBuffer.h"

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

void DataBuffer::addByte(byte in, BufferPosition location)
{
  m_buffer.add(in, location);
}

byte DataBuffer::removeByte(BufferPosition location)
{
  return m_buffer.remove(location);
}

// Following IEEE 754 specifications a type int is always sent/received
// with Most Significant Byte (MSB) first.
// In Arduino a type int is stored in memory in Little Endian format,
// it means a pointer to an int gives the address of the LSB.
// [LSB][...][...][MSB].
void DataBuffer::addInt(int in, BufferPosition location)
{
  byte* pointer = (byte*)&in;
  if (location == BufferPosition::Tail)
  {
    m_buffer.add(pointer[1], BufferPosition::Tail);
    m_buffer.add(pointer[0], BufferPosition::Tail);
  }
  else
  {
    m_buffer.add(pointer[0], BufferPosition::Head);
    m_buffer.add(pointer[1], BufferPosition::Head);
  }
}

// Following IEEE 754 specifications, a type int is always sent/received
// with Most Significant Byte (MSB) first.
// In Arduino a type int is stored in memory in Little Endian format,
// it means a pointer to an int gives the address of the LSB.
// [LSB][...][...][MSB].
int DataBuffer::removeInt(BufferPosition location)
{
  int ret;
  byte* pointer = (byte*)&ret;
  if (location == BufferPosition::Head)
  {
    pointer[1] = m_buffer.remove(BufferPosition::Head);
    pointer[0] = m_buffer.remove(BufferPosition::Head);
  }
  else
  {
    pointer[0] = m_buffer.remove(BufferPosition::Tail);
    pointer[1] = m_buffer.remove(BufferPosition::Tail);
  }
  return ret;
}

// Following IEEE 754 specifications, a type long is always sent/received
// with Most Significant Byte (MSB) first.
// In Arduino a long is stored in memory in Little Endian format,
// it means a pointer to a long gives the address of the LSB.
// [LSB][...][...][MSB].
void DataBuffer::addLong(long in, BufferPosition location)
{
  byte* pointer = (byte*)&in;
  if (location == BufferPosition::Tail)
  {
    m_buffer.add(pointer[3], BufferPosition::Tail);
    m_buffer.add(pointer[2], BufferPosition::Tail);
    m_buffer.add(pointer[1], BufferPosition::Tail);
    m_buffer.add(pointer[0], BufferPosition::Tail);
  }
  else
  {
    m_buffer.add(pointer[0], BufferPosition::Head);
    m_buffer.add(pointer[1], BufferPosition::Head);
    m_buffer.add(pointer[2], BufferPosition::Head);
    m_buffer.add(pointer[3], BufferPosition::Head);
  }
}

// Following IEEE 754 specifications, a type long is always sent/received
// with Most Significant Byte (MSB) first.
// In Arduino, a long is stored in memory in Little Endian format,
// it means a pointer to a long gives the address of the LSB.
// [LSB][...][...][MSB].
long DataBuffer::removeLong(BufferPosition location)
{
  long ret;
  byte* pointer = (byte*)&ret;
  if (location == BufferPosition::Head)
  {
    pointer[3] = m_buffer.remove(BufferPosition::Head);
    pointer[2] = m_buffer.remove(BufferPosition::Head);
    pointer[1] = m_buffer.remove(BufferPosition::Head);
    pointer[0] = m_buffer.remove(BufferPosition::Head);
  }
  else
  {
    pointer[0] = m_buffer.remove(BufferPosition::Tail);
    pointer[1] = m_buffer.remove(BufferPosition::Tail);
    pointer[2] = m_buffer.remove(BufferPosition::Tail);
    pointer[3] = m_buffer.remove(BufferPosition::Tail);
  }
  return ret;
}

// Following IEEE 754 specifications, a type float is always sent/received
// with Sign and Exponent first followed by the Significand in
// Most Significant Byte (MSB) first.
void DataBuffer::addFloat(float in, BufferPosition location)
{
  byte* pointer = (byte*)&in;
  if (location == BufferPosition::Tail)
  {
    m_buffer.add(pointer[3], BufferPosition::Tail);
    m_buffer.add(pointer[2], BufferPosition::Tail);
    m_buffer.add(pointer[1], BufferPosition::Tail);
    m_buffer.add(pointer[0], BufferPosition::Tail);
  }
  else
  {
    m_buffer.add(pointer[0], BufferPosition::Head);
    m_buffer.add(pointer[1], BufferPosition::Head);
    m_buffer.add(pointer[2], BufferPosition::Head);
    m_buffer.add(pointer[3], BufferPosition::Head);
  }
}

// Following IEEE 754 specifications a type float is stored in the following way:
// - Sign: 1bit
// - Exponent: 8 bits
// - Significand: 23 bits.
float DataBuffer::removeFloat(BufferPosition location)
{
  float ret;
  byte* pointer = (byte*)&ret;
  if (location == BufferPosition::Head)
  {
    pointer[3] = m_buffer.remove(BufferPosition::Head);
    pointer[2] = m_buffer.remove(BufferPosition::Head);
    pointer[1] = m_buffer.remove(BufferPosition::Head);
    pointer[0] = m_buffer.remove(BufferPosition::Head);
  }
  else
  {
    pointer[0] = m_buffer.remove(BufferPosition::Tail);
    pointer[1] = m_buffer.remove(BufferPosition::Tail);
    pointer[2] = m_buffer.remove(BufferPosition::Tail);
    pointer[3] = m_buffer.remove(BufferPosition::Tail);
  }
  return ret;
}
