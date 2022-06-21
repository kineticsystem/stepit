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
 */
class MotorGoal
{

public:
    MotorGoal();

    void setSpeed(float maxSpeed);
    float getSpeed() const;
    void setPosition(long targetPosition);
    long getPosition() const;

private:
    float m_speed = 0.0;
    long m_position = 0;
};

#endif // MOTOR_GOAL_H
