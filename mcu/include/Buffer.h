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

#ifndef BUFFER_H
#define BUFFER_H

enum class BufferPosition
{
  Head,
  Tail
};

/**
 * This template implements a circular buffer. Items can be added and removed
 * to and from both sides of the buffer.
 */
template <typename T>
class Buffer
{
public:
  explicit Buffer(size_t capacity);
  ~Buffer();

  // Return the maximum capacity of the buffer.
  size_t capacity() const;

  // Return how much data is in the buffer.
  size_t size() const;

  // Insert an item at the given position.
  void add(T in, BufferPosition position);

  // Remove an item from the given position.
  // Removing an item from an empty buffer leads to undefined behavior.
  T remove(BufferPosition position);

  // Reset the buffer.
  void clear();

private:
  T* m_data;

  // The maximum capacity of the buffer.
  size_t m_capacity = 0;

  // "m_position" is the position of the first element of the buffer.
  size_t m_position = 0;

  // This records how much data is in the buffer: it is always greater
  // than zero and less than or equal to the capacity.
  // "m_position + m_size - 1" is the position of the last element of the
  // buffer.
  size_t m_size = 0;
};

template <typename T>
Buffer<T>::Buffer(size_t capacity) : m_data{ new T[capacity] }, m_capacity{ capacity }, m_position{ 0 }, m_size{ 0 }
{
}

template <typename T>
Buffer<T>::~Buffer()
{
  delete[] m_data;
}

template <typename T>
size_t Buffer<T>::size() const
{
  return m_size;
}

template <typename T>
size_t Buffer<T>::capacity() const
{
  return m_capacity;
}

template <typename T>
void Buffer<T>::clear()
{
  m_position = 0;
  m_size = 0;
}

template <typename T>
void Buffer<T>::add(T in, BufferPosition position)
{
  // Please note that there is no exception thrown in integer arithmetic
  // overflow or underflow.

  if (m_size < m_capacity)
  {
    if (position == BufferPosition::Tail)
    {
      // Add an item to the end of the buffer.
      m_data[(m_position + m_size) % m_capacity] = in;
    }
    else
    {
      // Add an item to the front of the buffer.
      if (m_position == 0)
      {
        m_position = m_capacity - 1;
      }
      else
      {
        m_position--;
      }
      m_data[m_position] = in;
    }
    m_size++;
  }
}

template <typename T>
T Buffer<T>::remove(BufferPosition position)
{
  // Please note that there is no exception thrown in integer arithmetic
  // overflow or underflow.

  T out;
  if (m_size > 0)
  {
    if (position == BufferPosition::Tail)
    {
      // Remove an item from the end of the buffer.
      out = m_data[(m_position + m_size - 1) % m_capacity];
    }
    else
    {
      // Remove an item from the front of the buffer.
      out = m_data[m_position];
      m_position = (m_position + 1) % m_capacity;
    }
    m_size--;
  }
  else
  {
    out = T{};  // This never happens.
  }
  return out;
}

#endif  // BUFFER_H
