BUFFERED SERIAL PORT

Author Giovanni Remigi

This is a library to read/write data using serial port.

Incoming and outgoing data are received and sent if form of packets using an implementation of Point-to-Point (PPP) protocol with frame delimiters and escaped bytes.

The packet has the following form:

DELIMITER-TAG
sequence number
...
data
...
CCITT CRC-16 (Kermit)
DELIMITER-TAG

For each call to the method BufferedSerial::update() a single byte is read and write.
This minimize the time Arduino spend managing data on the serial port.
