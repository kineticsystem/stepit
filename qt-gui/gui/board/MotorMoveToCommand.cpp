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

#include "MotorMoveToCommand.h"
#include "DataUtils.h"

MotorMoveToCommand::MotorMoveToCommand(unsigned char motorId, long position) : Request()
{
    QByteArray encodedPosition = DataUtils::fromInt32(position);
    QByteArray cmd;
    cmd.append(MOVE_TO_CMD);     // move to command
    cmd.append(motorId);         // motor id
    cmd.append(encodedPosition); // position

    requestBytes = cmd;

    this->description = QString("MOTOR %1 MOVE TO POSITION %2")
        .arg(QString::number(motorId))
        .arg(QString::number(position));
}

