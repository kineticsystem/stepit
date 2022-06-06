/*
 * Copyright (C) 2022 Remigi Giovanni
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

#include <CrcUtils.h>

bool CrcUtils::initialized = false;
unsigned short CrcUtils::crcTable[256] = {0};

void CrcUtils::init()
{
    // Initialize CRC table.

    int i, j;
    unsigned short crc, c;

    for (i = 0; i < 256; i++)
    {
        crc = 0;
        c = (unsigned short)i;
        for (j = 0; j < 8; j++)
        {
            if ((crc ^ c) & 0x0001)
            {
                crc = (crc >> 1) ^ 0x8408;
            }
            else
            {
                crc = crc >> 1;
            }
            c = c >> 1;
        }

        crcTable[i] = crc;
    }
}

unsigned short CrcUtils::updateCRC(unsigned short crc, char byte)
{
    if (!initialized)
    {
        init();
        initialized = true;
    }

    unsigned short tmp;
    tmp = crc ^ (0x00ff & (unsigned short)byte);
    crc = (crc >> 8) ^ crcTable[tmp & 0xff];
    return crc;
}

unsigned short CrcUtils::calculateCRC(const char *msg, int length)
{
    unsigned short crc = 0x0000;
    while (length--)
    {
        crc = updateCRC(crc, *msg++);
    }
    return crc;
}
