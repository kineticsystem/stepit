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

#ifndef MOTORSTATUSCOMMAND_H
#define MOTORSTATUSCOMMAND_H

#include "Request.h"

/**
 * Return the motor speed, position and distance to go.
 */
class MotorStatusCommand : public Request
{
public:
    explicit MotorStatusCommand();
private:
    static constexpr unsigned char STATUS_CMD = 0x75;
};

#endif // MOTORSTATUSCOMMAND_H
