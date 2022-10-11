/*
 * Copyright (c) 2022, Giovanni Remigi
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef CRC_UTILS_H
#define CRC_UTILS_H

/**
 * This utility provides methods to compute CRC-16 (Cyclic Redundancy Check) on
 * a sequence of bytes.
 * It implements an algorithm called CCITT CRC-16 (Kermit).
 * Please note that the calculated CRC is given in Little Endian Form and the
 * two bytes produced must be inverted before sending the CRC-16 to the serial
 * port.
 *
 * A complete catalog of parametrized CRC algorithms with 16 bits is available at
 * http://reveng.sourceforge.net/crc-catalogue/16.htm
 *
 * A CRC online calculator useful for testing is available at
 * http://www.lammertbies.nl/comm/info/crc-calculation.html
 *
 * For a sample implementation of different CRC-16 see
 * http://www.lammertbies.nl/comm/software/
 */
class CrcUtils
{
public:
  /**
   * This method updates the given CRC adding a new byte to the original
   * sequence of bytes where the given CRC was computed.
   * The CCITT CRC-16 (Kermit) requires to pass an initial CRC equal to 0x0000
   * the first time the method is invoked.
   * Please note that the calculated CRC is given in Little Endian Form.
   * @param crc The CRC value to update.
   * @param byte The byte to be added to the original sequence of bytes where
   *     the given CRC was computed.
   * @return The new CRC computed on the full sequence of bytes.
   */
  static unsigned short crc_ccitt_byte(unsigned short crc, unsigned char byte);

  /**
   * This method calculates the CRC on the given sequence bytes and length.
   * Please note that the calculated CRC is given in Little Endian Form.
   * @param bytes The sequence of bytes to calculate the CRC.
   * @param length The sequence length in a number of bytes.
   * @return The CRC calculated on the given sequence of bytes.
   */
  static unsigned short crc_ccitt(const unsigned char* bytes, int length);
};

#endif  // CRC_UTILS_H
