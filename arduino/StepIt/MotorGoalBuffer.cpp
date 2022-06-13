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

#include "MotorGoalBuffer.h"
#include <util/atomic.h>

MotorGoalBuffer::MotorGoalBuffer(unsigned int bufferSize)
{
    m_buffer = new Buffer<MotorGoal>{bufferSize};
}

MotorGoalBuffer::~MotorGoalBuffer()
{
    delete m_buffer;
}

void MotorGoalBuffer::add(MotorGoal goal)
{
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        m_buffer->add(goal, Location::END);
    }
}

MotorGoal MotorGoalBuffer::remove()
{
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        m_buffer->remove(Location::FRONT);
    }
}

bool MotorGoalBuffer::isEmpty() const
{
    return m_buffer->size() == 0;
}
