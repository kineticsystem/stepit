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

#ifndef SERIALUTILS_H
#define SERIALUTILS_H

#include <QByteArray>
#include <QString>

class DataUtils
{
public:

    // Type conversion functions.

    static QByteArray fromInt32(const qint32& value);
    static qint32 toInt32(const QByteArray& bytes);

    static QByteArray fromInt16(const qint16& value);
    static qint16 toInt16(const QByteArray& bytes);

    static QByteArray fromFloat(const float& value);
    static float toFloat(const QByteArray& bytes);

    static QString toHex(const QByteArray& bytes);
    static QString toHex(const char& byte);

private:

    DataUtils();
};

#endif // SERIALUTILS_H
