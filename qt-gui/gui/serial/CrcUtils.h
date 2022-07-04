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
 * A full catalogue of parametrised CRC algorithms with 16 bits is available at
 * http://reveng.sourceforge.net/crc-catalogue/16.htm
 *
 * A useful CRC calculator online that can be used for testing is available at
 * http://www.lammertbies.nl/comm/info/crc-calculation.html
 *
 * For a sample implementation of different CRC-16 see
 * http://www.lammertbies.nl/comm/software/
 */
class CrcUtils
{

public:

    /**
     * Initialize the precomputed table that is used in the CRC calculation.
     * Initialization is done once when any of the class methods are invoked
     * but, should it be required, this method permits initialization beforehand.
     */
    static void init();

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
    static unsigned short updateCRC(unsigned short crc, char byte);

    /**
     * This method calculates the CRC on the given sequence bytes and length.
     * Please note that the calculated CRC is given in Little Endian Form.
     * @param bytes The sequence of bytes to calculate the CRC.
     * @param length The sequence length in number of bytes.
     * @return The CRC calculated on the given sequence of bytes.
     */
    static unsigned short calculateCRC(const char* bytes, int length);

private:

    // Variable to check if all required data structures are initialized.
    static bool initialized;

    // The algorithm uses a precomputed table for performance reason.
    static unsigned short crcTable[256];
};

#endif // CRC_UTILS_H
