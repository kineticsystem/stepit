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

#include "MotorMaxSpeedCommand.h"
#include "DataUtils.h"

MotorMaxSpeedCommand::MotorMaxSpeedCommand(unsigned char motorId, float speed) : Request()
{
    QByteArray encodedSpeed = DataUtils::fromFloat(speed);
    QByteArray cmd;
    cmd.append(SET_MAX_SPEED_CMD); // set speed command
    cmd.append(motorId);           // motor id
    cmd.append(encodedSpeed);      // max speed

    requestBytes = cmd;

    this->description = QString("MOTOR %1 SET MAX SPEED %2")
        .arg(QString::number(motorId))
        .arg(QString::number(speed));
}

