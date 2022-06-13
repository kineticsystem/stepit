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

#ifndef MOTOR_GOAL_BUFFER_H
#define MOTOR_GOAL_BUFFER_H

#include "MotorGoal.h"
#include <Buffer.h>
#include <Location.h>

/**
 * This implements a circular byte buffer to be used during a serial port
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
class MotorGoalBuffer
{
public:
    explicit MotorGoalBuffer(unsigned int bufferSize);
    ~MotorGoalBuffer();

    void add(MotorGoal goal);

    MotorGoal remove();

    bool isEmpty() const;

private:
    Buffer<MotorGoal> *m_buffer;
    volatile boolean m_isReadyRead;
};

#endif // MOTOR_GOAL_BUFFER_H
