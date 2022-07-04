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

#include "MotorStatusResponse.h"
#include "DataUtils.h"

MotorStatusResponse::MotorStatusResponse() : Response()
{
    this->description = "BLA";
}

MotorStatusResponse::MotorStatusResponse(const Response &msg) : Response(msg)
{
    /*
     * The message contains information about all motors.
     *
     * 0:     message id
     *
     * 1-4:   1st motor position
     * 5-8:   1st motor speed
     * 9-12:  1st motor distance to go
     *
     * 13-16: 2nd motor position
     * 17-20: 2nd motor speed
     * 21-24: 2nd motor distance to go
     */

    this->description = "STATUS MESSAGE: ";

    int index = 0;
    for (int motorId = 0; motorId < 2; motorId++) {

        QByteArray encodedPosition = responseBytes.mid(index + 1, index + 4);
        long position = DataUtils::toInt32(encodedPosition);

        QByteArray encodedSpeed = responseBytes.mid(index + 5, index + 8);
        float speed = DataUtils::toFloat(encodedSpeed);

        QByteArray encodedDistanceToGo = responseBytes.mid(index + 9, index + 12);
        long distanceToGo = DataUtils::toInt32(encodedDistanceToGo);

        motorStatus[motorId].setPosition(position);
        motorStatus[motorId].setSpeed(speed);
        motorStatus[motorId].setDistanceToGo(distanceToGo);

        index += 12;

        QString status = QString("{ID:%1, POS:%2, SPD:%3, DST:%4} ")
            .arg(QString::number(motorId))
            .arg(QString::number(position))
            .arg(QString::number(speed))
            .arg(QString::number(distanceToGo));
        this->description.append(status);
    }
}

MotorStatusResponse::~MotorStatusResponse()
{
}

const MotorStatus MotorStatusResponse::getMotor(int motorId) const
{
    return motorStatus[motorId];
}



