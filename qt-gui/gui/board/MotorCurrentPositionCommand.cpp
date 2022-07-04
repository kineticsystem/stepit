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

#include "MotorCurrentPositionCommand.h"
#include "DataUtils.h"

MotorCurrentPositionCommand::MotorCurrentPositionCommand(unsigned char motorId, long currentPosition)
{
    QByteArray encodedCurrentPosition = DataUtils::fromInt32(currentPosition);
    QByteArray cmd;
    cmd.append(SET_CURRENT_POSITION);   // set current position command
    cmd.append(motorId);                // motor id
    cmd.append(encodedCurrentPosition); // current position

    requestBytes = cmd;

    this->description = QString("MOTOR %1 RESET POSITION TO %2")
        .arg(QString::number(motorId))
        .arg(QString::number(currentPosition));
}
