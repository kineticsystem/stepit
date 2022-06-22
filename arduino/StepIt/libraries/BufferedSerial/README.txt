BUFFERED SERIAL PORT

Author Giovanni Remigi

This is a library to read/write data using the serial port.

Incoming and outgoing data are received and sent in form of packets using an implementation of the Point-to-Point (PPP) protocol with frame delimiters and escaped bytes.

The packet has the following form:

DELIMITER-TAG
sequence number
...
data
...
CCITT CRC-16 (Kermit)
DELIMITER-TAG

For each call to the method BufferedSerial::update() a single byte is read and written to minimize the time Arduino spends on the serial port.
