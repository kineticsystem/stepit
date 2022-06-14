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

#include "MotorGoal.h"

MotorGoal::MotorGoal()
{
}

void MotorGoal::setTargetSpeed(float targetSpeed)
{
    m_targetSpeed = targetSpeed;
}

float MotorGoal::getTargetSpeed() const
{
    return m_targetSpeed;
}

void MotorGoal::setTargetPosition(long targetPosition)
{
    m_targetPosition = targetPosition;
}

long MotorGoal::getTargetPosition() const
{
    return m_targetPosition;
}

void MotorGoal::setTargetSteps(long targetSteps)
{
    m_targetSteps = targetSteps;
}

long MotorGoal::getTargetSteps() const
{
    return m_targetSteps;
}

void MotorGoal::setAcceleration(float acceleration)
{
    m_acceleration = acceleration;
}

float MotorGoal::getAcceleration() const
{
    return m_acceleration;
}

void MotorGoal::setMaxAbsoluteSpeed(float maxSpeed)
{
    m_maxAbsoluteSpeed = maxSpeed;
}

float MotorGoal::getMaxAbsoluteSpeed() const
{
    return m_maxAbsoluteSpeed;
}
