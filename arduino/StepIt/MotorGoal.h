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

#ifndef MOTOR_GOAL_H
#define MOTOR_GOAL_H

/**
 * This class is used to exchange motor goals between the main thread and the
 * ISR (Interrupt Service Routine). Goals are target speed, acceleration,
 * target position, maximum speed.
 * Goals are written by the main thread and read by the ISR when the flag
 * readReady is true, which in turn will set it back to false after reading.
 */
class MotorGoal
{
    float m_targetSpeed = 0.0;
    long m_targetPosition = 0;
    long m_targetSteps = 0;
    float m_acceleration = 500.0;
    float m_maxSpeed = 1000.0;

    volatile bool m_readReady = false;

public:
    MotorGoal();

    void setTargetSpeed(float targetSpeed);
    float getTargetSpeed() const;
    void setTargetPosition(long targetPosition);
    long getTargetPosition() const;
    void setTargetSteps(long targetSteps);
    long getTargetSteps() const;
    void setAcceleration(float acceleration);
    float getAcceleration() const;
    void setMaxSpeed(float maxSpeed);
    float getMaxSpeed() const;

    void setReadReady(bool readReady);
    bool isReadReady() const;
};

#endif // MOTOR_GOAL_H
