# COBS Serial Communication Library

This package provides a C++ library for serial communication that uses COBS (Consistent Overhead Byte Stuffing) encoding. It is designed to simplify and improve the reliability of data transmission over serial connections in embedded systems or microcontroller applications.

## Package Overview

The COBS Serial library helps you send and receive data reliably over serial ports (like RS-232, RS-485, or USB-to-serial). It uses a special encoding technique called COBS to ensure that:

- Data packets are easily separated using zero bytes as delimiters.
- No unwanted zero bytes appear in the actual data being transmitted.
- Minimal extra data is added to the original message (low overhead).

This makes it ideal for applications where reliable communication is critical, such as industrial automation, robotics, or IoT devices.

## Advantages of COBS Serial

Reliable communication over serial connections can be tricky. Without proper framing, it's easy to lose track of where one packet ends and another begins. The COBS encoding used in this library solves these problems by:

- Making it simple to detect the start and end of each data packet.
- Eliminating false triggers caused by zero bytes in the data.
- Adding minimal overhead compared to other framing methods.

This means you can focus on your application logic while letting the library handle the details of reliable communication.

## Getting Started

The library provides an abstract interface (CobsSerial) that defines how serial communication should work. You can implement this interface for specific hardware or software (e.g., a USB-to-serial adapter). The key features are:

1. Open/Close Management: Easily manage the lifecycle of your serial connection.
2. Read/Write Operations: Send and receive data in a straightforward way.
3. COBS Encoding: Automatically handle the encoding and decoding of data to ensure reliable communication.
