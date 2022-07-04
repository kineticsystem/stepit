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

#ifndef MESSAGEINFO_H
#define MESSAGEINFO_H

#include "QString"

#include "Response.h"

/**
 * Return information about the software installed on Arduino.
 * This is used to automatically determine which COM/USB port Arduino is
 * connected to.
 * @see InfoCommand
 */
class InfoResponse : public Response
{
public:
    static constexpr unsigned char TYPE = 0x02;
    explicit InfoResponse();
    explicit InfoResponse(const Response& msg);
};

#endif // MESSAGEINFO_H
