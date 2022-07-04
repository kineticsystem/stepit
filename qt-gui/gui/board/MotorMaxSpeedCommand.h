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

#ifndef MOTORMAXSPEEDCOMMAND_H
#define MOTORMAXSPEEDCOMMAND_H

#include "Request.h"

/**
 * Set the maximum possible speed in steps/s.
 * With two steppers motors running at the same time, an acceptable maximum
 * speed is 1000 steps/s
 */
class MotorMaxSpeedCommand : public Request
{
public:
    explicit MotorMaxSpeedCommand(unsigned char motorId, float maxSpeed);
private:
    static constexpr unsigned char SET_MAX_SPEED_CMD = 0x78;
};

#endif // MOTORMAXSPEEDCOMMAND_H
