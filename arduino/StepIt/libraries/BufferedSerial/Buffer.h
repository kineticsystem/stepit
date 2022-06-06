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

    // Return how much data is currently stored in the buffer.
    int size();

    // Return the maximum capacity of the buffer
    int capacity();

    // Insert an item at the give location.
    void add(T in, Location location);

    // Remove an item from the given location.
    T remove(Location location);

    // Reset the buffer into an original state (with no data).
    void clear();

private:
    T *m_data;

    // The maximum capacity of the buffer
    unsigned int m_capacity;

    // This is the position where an item is read from using the get method.
    unsigned int m_position;

    // This record how much data is currently stored in the buffer.
    unsigned int m_size;
};

template <typename T>
Buffer<T>::Buffer(unsigned int bufferSize)
{
    m_data = new T[bufferSize];
    m_capacity = bufferSize;
    m_position = 0;
    m_size = 0;
}

template <typename T>
Buffer<T>::~Buffer()
{
    delete[] m_data;
}

template <typename T>
int Buffer<T>::size()
{
    return m_size;
}

template <typename T>
int Buffer<T>::capacity()
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
    if (m_size < m_capacity)
    {
        if (location == Location::END)
        {
            m_data[(m_position + m_size) % m_capacity] = in;
            m_size++;
        }
        else
        {
            if (m_position == 0)
            {
                m_position = m_capacity - 1;
            }
            else
            {
                m_position = (m_position - 1) % m_capacity;
            }
            m_data[m_position] = in;
            m_size++;
        }
    }
}

template <typename T>
T Buffer<T>::remove(Location location)
{
    T out = 0;
    if (m_size > 0)
    {
        if (location == Location::FRONT)
        {
            // Remove an item from the beginning of the buffer.
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
