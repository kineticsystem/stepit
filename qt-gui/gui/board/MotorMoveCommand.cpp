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

#include "MotorMoveCommand.h"
#include "DataUtils.h"

MotorMoveCommand::MotorMoveCommand(unsigned char motorId, long steps) : Request()
{
    QByteArray encodedSteps = DataUtils::fromInt32(steps);
    QByteArray cmd;
    cmd.append(MOVE_CMD);     // move command
    cmd.append(motorId);      // motor id
    cmd.append(encodedSteps); // steps

    requestBytes = cmd;

    this->description = QString("MOTOR %1 MOVE STEPS %2")
        .arg(QString::number(motorId))
        .arg(QString::number(steps));
}

MotorMoveCommand::MotorMoveCommand(unsigned char motorId, Direction direction) {

    long steps = 0;
    switch (direction) {
    case Direction::ANTI_CLOCKWISE:
        steps = +1800000L; // 1,000 full rotations.
        break;
    case Direction::CLOCKWISE:
        steps = -1800000L; // 1,000 full rotations.
        break;
    }

    QByteArray encodedSteps = DataUtils::fromInt32(steps);
    QByteArray cmd;
    cmd.append(MOVE_CMD);     // move command
    cmd.append(motorId);      // motor id
    cmd.append(encodedSteps); // steps

    requestBytes = cmd;

    this->description = QString("MOTOR %1 MOVE ")
        .arg(QString::number(motorId));
    switch (direction) {
    case Direction::ANTI_CLOCKWISE:
        this->description.append("ANTI CLOCKWISE");
        break;
    case Direction::CLOCKWISE:
        this->description.append("CLOCKWISE");
        break;
    }
}

