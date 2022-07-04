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

#ifndef MOTORMOVETOCOMMAND_H
#define MOTORMOVETOCOMMAND_H

#include "Request.h"

class MotorMoveToCommand : public Request
{
public:
    explicit MotorMoveToCommand(unsigned char motorId, long position);
private:
    static constexpr unsigned char MOVE_TO_CMD = 0x71;
};

#endif // MOTORMOVETOCOMMAND_H
