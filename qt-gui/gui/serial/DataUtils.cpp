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

#include "DataUtils.h"
#include <QDataStream>

DataUtils::DataUtils()
{
}

QByteArray DataUtils::fromInt32(const qint32& value)
{
    QByteArray target;
    QDataStream stream(&target, QIODevice::WriteOnly);
    stream.setByteOrder(QDataStream::BigEndian);
    stream << value;
    return target;
}

qint32 DataUtils::toInt32(const QByteArray& bytes)
{
    qint32 target;
    QDataStream stream(bytes);
    stream.setByteOrder(QDataStream::BigEndian);
    stream >> target;
    return target;
}

QByteArray DataUtils::fromInt16(const qint16& value)
{
    QByteArray target;
    QDataStream stream(&target, QIODevice::WriteOnly);
    stream.setByteOrder(QDataStream::BigEndian);
    stream << value;
    return target;
}

qint16 DataUtils::toInt16(const QByteArray& bytes)
{
    qint16 target;
    QDataStream stream(bytes);
    stream.setByteOrder(QDataStream::BigEndian);
    stream >> target;
    return target;
}

QByteArray DataUtils::fromFloat(const float& value)
{
    QByteArray target;
    QDataStream stream(&target, QIODevice::WriteOnly);
    stream.setByteOrder(QDataStream::BigEndian);
    stream.setFloatingPointPrecision(QDataStream::SinglePrecision);
    stream << value;
    return target;
}

float DataUtils::toFloat(const QByteArray& bytes)
{
    float target;
    QDataStream stream(bytes);
    stream.setByteOrder(QDataStream::BigEndian);
    stream.setFloatingPointPrecision(QDataStream::SinglePrecision);
    stream >> target;
    return target;
}

QString DataUtils::toHex(const QByteArray& bytes)
{
    QString hex;
    for (QByteArray::const_iterator it = bytes.begin(); it != bytes.end(); ++it) {
        if (it !=  bytes.begin()) {
            hex.append(" ");
        }
        char chars[] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'};
        uchar ch = *it;
        char ls = chars[(ch & 0xF)];
        char ms = chars[((ch >> 4) & 0xF)];
        hex.append("0x");
        hex.append(ms);
        hex.append(ls);
    }
    return hex;
}

QString DataUtils::toHex(const char& byte) {
    QByteArray bytes;
    bytes.append(byte);
    return toHex(bytes);
}
