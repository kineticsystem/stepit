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

#ifndef MOTORMOVECOMMAND_H
#define MOTORMOVECOMMAND_H

#include "Request.h"

class MotorMoveCommand : public Request
{
public:

    enum class Direction {
        CLOCKWISE,
        ANTI_CLOCKWISE
    };

    explicit MotorMoveCommand(unsigned char motorId, long steps);
    explicit MotorMoveCommand(unsigned char motorId, Direction direction);

private:
    static constexpr unsigned char MOVE_CMD = 0x70;
};

#endif // MOTORMOVECOMMAND_H