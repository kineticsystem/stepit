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

#ifndef MOTOR_STATE_H
#define MOTOR_STATE_H

/**
 * This class is used to exchange motor states between the ISR (Interrupt
 * Service Routine) and the main thread.
 * States are written by the ISR and read by the main thread.
 */
class MotorState
{

public:
    MotorState();

    void setSpeed(float speed);
    float getSpeed() const;
    void setCurrentPosition(long position);
    long getCurrentPosition() const;
    void setDistanceToGo(long distanceToGo);
    long getDistanceToGo() const;

private:
    float m_speed = 0.0;
    long m_currentPosition = 0;
    long m_distanceToGo = 0;
};

#endif // MOTOR_STATE_H