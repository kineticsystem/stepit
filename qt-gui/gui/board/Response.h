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

#ifndef INMESSAGE_H
#define INMESSAGE_H

#include <QByteArray>
#include <QString>

class Response
{
public:
    explicit Response();
    virtual void setResponseBytes(const QByteArray& responseBytes);
    virtual const QString& getDescription() const;
protected:
    QByteArray responseBytes;

    // A human readable description of the message for debugging.
    mutable QString description;
};

#endif // INMESSAGE_H
