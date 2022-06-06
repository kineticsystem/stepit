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

#ifndef SHARED_BUCKET_H
#define SHARED_BUCKET_H

/**
 * This structure holds data that are shared between the Interrupt Service Routine (ISR)
 * and the main loop.
 * Shared data are declared volatile.
 */
class SharedBucket
{

public:
    explicit SharedBucket(float maxSpeed, float acceleration);

    void setSpeed(float speed);
    float getSpeed();
    void setCurrentPosition(long currentPosition);
    long getCurrentPosition();
    void setDistanceToGo(long distanceToGo);
    long getDistanceToGo();

    void setTargetSpeed(float targetSpeed);
    float getTargetSpeed();
    void setTargetPosition(long targetPosition);
    long getTargetPosition();
    void setTargetSteps(long targetSteps);
    long getTargetSteps();
    void setAcceleration(float acceleration);
    float getAcceleration();
    void setMaxSpeed(float maxSpeed);
    float getMaxSpeed();
    void setDecelerating(bool decelerating);
    bool isDecelerating();

    void setOldTargetSpeed(float oldTargetSpeed);
    float getOldTargetSpeed();
    void setOldTargetPosition(long oldTargetPosition);
    long getOldTargetPosition();

private:
    // These values are written by the interrupt routine and read in the main loop.
    volatile float speed;
    volatile long currentPosition;
    volatile long distanceToGo;

    // These values are written in the main loop and read by the interrupt routine.
    volatile float targetSpeed;
    volatile long targetPosition;
    volatile float acceleration;
    volatile float maxSpeed;

    // These are value used exclusively inside the interrupt routine.

    float oldTargetSpeed;
    long oldTargetPosition;
    bool decelerating;
};

#endif
