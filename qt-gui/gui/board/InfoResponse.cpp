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

#include "QByteArray"
#include "InfoResponse.h"

InfoResponse::InfoResponse() : Response()
{

}

InfoResponse::InfoResponse(const Response &msg) : Response(msg)
{
    /*
     * Message lenth is undefined.
     * 0:   command code
     * 1-n: string
     */

    QByteArray data = responseBytes.right(responseBytes.length() - 1);
    this->description = QString(data);
}



