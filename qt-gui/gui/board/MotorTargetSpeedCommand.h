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

#ifndef MOTORSPEEDCOMMAND_H
#define MOTORSPEEDCOMMAND_H

#include "Request.h"

/**
 * Set a motor step speed as a value from 0 to 100. When 0 the motor doesn't
 * speed, when 100 the motor speeds at the maximum set speed.
 * The target speed is not achive immediately but the motor accelerates or
 * decelerates based on the set acceleration.
 */
class MotorTargetSpeedCommand : public Request
{
public:
    explicit MotorTargetSpeedCommand(unsigned char motorId, float speed);
private:
    static constexpr unsigned char SET_TARGET_SPEED_CMD = 0x77;
};

#endif // MOTORSPEEDCOMMAND_H
