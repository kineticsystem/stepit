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

#include "MotorState.h"

MotorState::MotorState()
{
}

void MotorState::setSpeed(float speed)
{
    m_speed = speed;
}

float MotorState::getSpeed() const
{
    return m_speed;
}

void MotorState::setCurrentPosition(long position)
{
    m_currentPosition = position;
}

long MotorState::getCurrentPosition() const
{
    return m_currentPosition;
}

void MotorState::setDistanceToGo(long distanceToGo)
{
    m_distanceToGo = distanceToGo;
}

long MotorState::getDistanceToGo() const
{
    return m_distanceToGo;
}

void MotorState::setWriteReady(bool writeReady)
{
    m_writeReady = writeReady;
}

bool MotorState::isWriteReady()
{
    return m_writeReady;
}