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

#ifndef MOTORSTOPCOMMAND_H
#define MOTORSTOPCOMMAND_H

#include "Request.h"

/**
 * Stop all or one specified motor. The motor doesn't stop immediately
 * but decelerates to 0 speed base on the set motor acceleration.
 */
class MotorStopCommand : public Request
{
public:
    explicit MotorStopCommand();
    explicit MotorStopCommand(unsigned char motorId);
private:
    static constexpr unsigned char STOP_CMD = 0x72;
};

#endif // MOTORSTOPCOMMAND_H
