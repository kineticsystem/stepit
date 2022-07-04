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

#include "MotorStopCommand.h"

MotorStopCommand::MotorStopCommand() : Request()
{
    QByteArray cmd;
    cmd.append(STOP_CMD);  // stop command

    requestBytes = cmd;

    this->description = QString("STOP ALL MOTORS");
}

MotorStopCommand::MotorStopCommand(unsigned char motorId) : Request()
{
    QByteArray cmd;
    cmd.append(STOP_CMD);  // stop command
    cmd.append(motorId);   // motor id

    requestBytes = cmd;

    this->description = QString("STOP MOTOR %1").arg(QString::number(motorId));
}

