/*
 * Copyright (C) 2014 Remigi Giovanni
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

#include "StageModel.h"

StageModel::StageModel(QObject* parent) : QObject(parent)
{
}

float StageModel::getSpeed() const
{
    return speed;
}

void StageModel::setSpeed(float speed)
{
    if (this->speed != speed) {
        if (armed) {
            emit speedSet(speed);
        }
        this->speed = speed;
    }
}

long StageModel::getStartPosition() const
{
    return startPosition;
}

void StageModel::setStartPosition(long position)
{
    if (startPosition != position) {
        if (armed) {
            emit startPositionSet(position);
        }
        startPosition = position;
    }
}

long StageModel::getEndPosition() const
{
    return endPosition;
}

void StageModel::setEndPosition(long position)
{
    if (endPosition != position) {
        if (armed) {
            emit endPositionSet(position);
        }
        endPosition = position;
    }
}

unsigned int StageModel::getSteps() const
{
    return steps;
}

void StageModel::setSteps(unsigned int steps)
{
    if (this->steps != steps) {
        if (armed) {
            emit stepsSet(steps);
        }
        this->steps = steps;
    }
}

long StageModel::getTargetPosition() const
{
    return targetPosition;
}

void StageModel::setTargetPosition(long position)
{
    if (targetPosition != position) {
        if (armed) {
            emit targetPositionSet(position);
        }
        targetPosition = position;
    }
}

float StageModel::getMaxSpeed() const
{
    return maxSpeed;
}

void StageModel::setMaxSpeed(float speed)
{
    if (maxSpeed != speed) {
        if (armed) {
            emit maxSpeedSet(speed);
        }
        maxSpeed = speed;
    }
}

float StageModel::getAcceleration() const
{
    return acceleration;
}

void StageModel::setAcceleration(float acceleration)
{
    if (this->acceleration != acceleration) {
        if (armed) {
            emit maxSpeedSet(acceleration);
        }
        this->acceleration = acceleration;
    }
}

long StageModel::getCurrentPosition() const
{
    return currentPosition;
}

void StageModel::setCurrentPosition(long position)
{
    if (currentPosition != position) {
        if (armed) {
            emit currentPositionSet(position);
        }
        currentPosition = position;
    }
}

/*/////////////////////////////////////////////////////////////////////////////
 * Signals control.
 */

bool StageModel::isArmed() const
{
    return armed;
}

void StageModel::setArmed(bool value)
{
    armed = value;
}

void StageModel::fireModelChanged()
{
    emit startPositionSet(this->startPosition);
    emit endPositionSet(this->endPosition);
    emit targetPositionSet(this->targetPosition);
    emit currentPositionSet(this->currentPosition);
    emit speedSet(this->speed);
    emit maxSpeedSet(this->maxSpeed);
    emit accelerationSet(this->acceleration);
    emit stepsSet(this->steps);
}

