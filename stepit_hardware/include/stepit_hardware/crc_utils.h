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

#pragma once

#include <cstdint>
#include <vector>

/**
 * This utility provides methods to compute CRC-16 (Cyclic Redundancy Check) on
 * a sequence of bytes.
 * It implements an algorithm called CCITT CRC-16 (Kermit).
 *
 * A complete catalog of parametrized CRC algorithms with 16 bits is available at
 * http://reveng.sourceforge.net/crc-catalogue/16.htm
 *
 * A CRC online calculator useful for testing is available at
 * https://crccalc.com
 *
 * There are other online website to test a CRC but it seems the one above is
 * the only one to return the CRC in the correct little-endian order.
 * http://www.lammertbies.nl/comm/info/crc-calculation.html
 *
 * For a sample implementation of different CRC-16 see
 * http://www.lammertbies.nl/comm/software/
 */
namespace stepit_hardware::crc_utils
{
/**
 * This method updates the given CRC adding a new byte to the original
 * sequence of bytes where the given CRC was computed.
 * The CCITT CRC-16 (Kermit) requires to pass an initial CRC equal to 0x0000
 * the first time the method is invoked.
 * @param crc The CRC value to update.
 * @param byte The byte to be added to the original sequence of bytes where
 *     the given CRC was computed.
 * @return The new CRC computed on the full sequence of bytes.
 */
uint16_t crc_ccitt_byte(uint16_t crc, uint8_t byte);

/**
 * This method calculates the CRC on the given sequence bytes and length
 * stored in little-endian order.
 * @param bytes The sequence of bytes to calculate the CRC.
 * @return The CRC calculated on the given sequence of bytes.
 */
uint16_t crc_ccitt(const std::vector<uint8_t>& bytes);

}  // namespace stepit_hardware::crc_utils
