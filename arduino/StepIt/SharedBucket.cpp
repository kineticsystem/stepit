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

#include "SharedBucket.h"

SharedBucket::SharedBucket(float maxSpeed, float acceleration)
{
    this->maxSpeed = maxSpeed;
    this->acceleration = acceleration;
    this->speed = 0;
    this->currentPosition = 0;
    this->distanceToGo = 0;
    this->targetSpeed = maxSpeed;
    this->oldTargetSpeed = maxSpeed;
    this->targetPosition = 0;
    this->oldTargetPosition = 0;
    this->decelerating = false;
}

void SharedBucket::setSpeed(float speed)
{
    this->speed = speed;
}

float SharedBucket::getSpeed()
{
    return speed;
}

void SharedBucket::setCurrentPosition(long currentPosition)
{
    this->currentPosition = currentPosition;
}

long SharedBucket::getCurrentPosition()
{
    return currentPosition;
}

void SharedBucket::setDistanceToGo(long distanceToGo)
{
    this->distanceToGo = distanceToGo;
}

long SharedBucket::getDistanceToGo()
{
    return distanceToGo;
}

void SharedBucket::setTargetSpeed(float targetSpeed)
{
    this->targetSpeed = targetSpeed;
}

float SharedBucket::getTargetSpeed()
{
    return targetSpeed;
}

void SharedBucket::setTargetPosition(long targetPosition)
{
    this->targetPosition = targetPosition;
}

long SharedBucket::getTargetPosition()
{
    return targetPosition;
}

void SharedBucket::setOldTargetSpeed(float oldTargetSpeed)
{
    this->oldTargetSpeed = oldTargetSpeed;
}

float SharedBucket::getOldTargetSpeed()
{
    return oldTargetSpeed;
}

void SharedBucket::setOldTargetPosition(long oldTargetPosition)
{
    this->oldTargetPosition = oldTargetPosition;
}

long SharedBucket::getOldTargetPosition()
{
    return oldTargetPosition;
}

void SharedBucket::setAcceleration(float acceleration)
{
    this->acceleration = acceleration;
}

float SharedBucket::getAcceleration()
{
    return acceleration;
}

void SharedBucket::setMaxSpeed(float maxSpeed)
{
    this->maxSpeed = maxSpeed;
}

float SharedBucket::getMaxSpeed()
{
    return maxSpeed;
}

void SharedBucket::setDecelerating(bool decelerating)
{
    this->decelerating = decelerating;
}

bool SharedBucket::isDecelerating()
{
    return decelerating;
}
