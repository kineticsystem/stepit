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

#ifndef BUFFER_H
#define BUFFER_H

#include <Arduino.h>
#include <Location.h>

/**
 * This template implements a circular buffer.
 */
template <typename T>
class Buffer
{
public:
    explicit Buffer(unsigned int bufferSize);
    ~Buffer();

    // Return how much data is in the buffer.
    unsigned int size() const;

    // Return the maximum capacity of the buffer.
    unsigned int capacity() const;

    // Insert an item at the given location.
    void add(T in, Location location);

    // Remove an item from the given location.
    T remove(Location location);

    // Reset the buffer.
    void clear();

private:
    T *m_data;

    // The maximum capacity of the buffer
    const unsigned int m_capacity;

    // This is the position where to read or write.
    unsigned int m_position;

    // This records how much data is in the buffer: it is always greater
    // than zero and less than or equal to the capacity. Both Arduino main
    // thread and ISR (Interrupt Service Routines) can read it.
    //
    // Volatile byte explanation:
    //
    // 1) When Arduino main thread reads a variable but never modifies it, the
    // compiler would convert it into an inline constant as part of the
    // optimization process. The compiler does not know that an ISR may change
    // it. The variable must be marked volatile to avoid this optimization.
    //
    // 2) Additionally, the volatile keyword forces the compiler to generate
    // code that always reads the variable from RAM and does not cache the last
    // read value in a register. Volatile should always be used on any variable
    // that is modified by an interrupt.
    //
    // 3) If the volatile variable is greater than a byte, the microcontroller
    // cannot read it in one step because it is an 8-bit microcontroller.
    // This means that while the main code section reads the first 8 bits of the
    // variable, the interrupt might already change the second 8 bits producing
    // random values for the variable.
    //
    // There is a remedy to the last point: disabling interrupts when the
    // variable is read using ATOMIC_BLOCK macro.
    //
    // #include <util/atomic.h>
    //
    // volatile int input_from_interrupt;
    //
    // ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    //     // code with interrupts blocked
    //     int result = input_from_interrupt;
    // }
    volatile unsigned int m_size;
};

template <typename T>
Buffer<T>::Buffer(unsigned int bufferSize) : m_data{new T[bufferSize]}, m_capacity{bufferSize}, m_position{0}, m_size{0}
{
}

template <typename T>
Buffer<T>::~Buffer()
{
    delete[] m_data;
}

template <typename T>
unsigned int Buffer<T>::size() const
{
    return m_size;
}

template <typename T>
unsigned int Buffer<T>::capacity() const
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
void Buffer<T>::add(T in, Location location)
{
    // Please note that there is no exception thrown in integer arithmetic
    // overflow or underflow.

    if (m_size < m_capacity)
    {
        if (location == Location::END)
        {
            // Add an item to the end of the buffer.
            m_data[(m_position + m_size) % m_capacity] = in;
            m_size++;
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
            m_size++;
        }
    }
}

template <typename T>
T Buffer<T>::remove(Location location)
{
    // Please note that there is no exception thrown in integer arithmetic
    // overflow or underflow.

    T out = 0;
    if (m_size > 0)
    {
        if (location == Location::FRONT)
        {
            // Remove an item from the front of the buffer.
            out = m_data[m_position];
            m_position = (m_position + 1) % m_capacity;
            m_size--;
        }
        else
        {
            // Remove an item from the end of the buffer.
            out = m_data[(m_position + m_size - 1) % m_capacity];
            m_size--;
        }
    }
    return out;
}

#endif // BUFFER_H
