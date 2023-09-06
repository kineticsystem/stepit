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

#pragma once

#include <vector>

namespace cobs_serial
{

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
  explicit Buffer(std::size_t capacity);

  // Return how much data is in the buffer.
  [[nodiscard]] std::size_t size() const;

  // Insert an item at the given position.
  void add(T in, BufferPosition position);

  // Remove an item from the given position.
  // Removing an item from an empty buffer leads to undefined behavior.
  T remove(BufferPosition position);

  // Reset the buffer.
  void clear();

private:
  // The buffer data structure.
  std::vector<T> data_;

  // The position of the first element of the buffer.
  std::size_t position_ = 0;

  // The buffer capacity.
  std::size_t capacity_ = 0;

  // This records how much data is in the buffer: it is always greater
  // than zero and less than or equal to the capacity.
  // "position_ + size_ - 1" is the position of the last element of the
  // buffer.
  std::size_t size_ = 0;
};

template <typename T>
Buffer<T>::Buffer(std::size_t capacity) : capacity_{ capacity }
{
  data_.reserve(capacity);
}

template <typename T>
std::size_t Buffer<T>::size() const
{
  return size_;
}

template <typename T>
void Buffer<T>::clear()
{
  position_ = 0;
  size_ = 0;
}

template <typename T>
void Buffer<T>::add(T in, BufferPosition position)
{
  // Please note that there is no exception thrown in integer arithmetic
  // overflow or underflow.

  if (size_ < capacity_)
  {
    if (position == BufferPosition::Tail)
    {
      // Add an item to the end of the buffer.
      data_[(position_ + size_) % capacity_] = in;
    }
    else
    {
      // Add an item to the front of the buffer.
      if (position_ == 0)
      {
        position_ = capacity_ - 1;
      }
      else
      {
        position_--;
      }
      data_[position_] = in;
    }
    size_++;
  }
}

template <typename T>
T Buffer<T>::remove(BufferPosition position)
{
  // Please note that there is no exception thrown in integer arithmetic
  // overflow or underflow.

  T out;
  if (size_ > 0)
  {
    if (position == BufferPosition::Tail)
    {
      // Remove an item from the end of the buffer.
      out = data_[(position_ + size_ - 1) % capacity_];
    }
    else
    {
      // Remove an item from the front of the buffer.
      out = data_[position_];
      position_ = (position_ + 1) % capacity_;
    }
    size_--;
  }
  return out;
}
}  // namespace cobs_serial
