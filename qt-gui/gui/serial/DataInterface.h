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

#ifndef BUFFEREDSERIAL_H
#define BUFFEREDSERIAL_H

#include <QObject>
#include <QSerialPort>
#include <QTimer>
#include "serial/DataUtils.h"
#include "serial/ConnectionSettings.h"

/**
 * This class implements a PPP (Point-to-Point Protocol) between the Qt client
 * and the Arduino controller. Binary messages are properly encoded and sent to
 * Arduino. It implements ACK and CRC controls.
 * The class knows nothing about commands, it simply parses the outgoing and
 * incoming streams of data producing packets of information.
 * The class also owns the serial connection in case it is necessary to move it
 * into its own thread.
 */
class DataInterface : public QObject
{
    Q_OBJECT

public:

    explicit DataInterface(QObject* parent = nullptr);

    bool open(const ConnectionSettings& settings) const;
    void close() const;

    bool isOpen() const;

    // All low-level Qt networking classes are asynchronous by design.
    // This method put a binary packet into the network buffer and exit
    // immediately. Data will be sent to the serial port at any time by the OS.
    // The method itseld encapsulate all the logic of the Point-to-Point
    // protocol.
    void sendRequestData(const QByteArray& requestData) const;

private slots:

    // When data are available for reading on the serial port, this method
    // is invoked.
    void onReadyRead() const;

    // Call when the client has not received a response from Arduino.
    void onTimeout() const;

signals:

    // This signal is raised when a full response message is received.
    // The message is cleaned from all PPP related information such as CRC,
    // and packet delimiters.
    // A response message is always sent back by Arduino following a
    // client request.
    void responseDataReceived(const QByteArray& responseData) const;

    // This signal is raised when a full request message is received.
    // The message is cleaned from all PPP related information such as CRC,
    // and packet delimiters.
    // The request message is sent by Arduino own initiative, for example
    // to inform the client that it is ready to communicate.
    void ready() const;

    // If the Data Interface doesn't receive a low level aknowledge response
    // (ACK) after a given amount of time and after a given amount of trials,
    // a network error signal is raised meaning that there is sometihing wrong
    // in the connection.
    void networkError() const;

private:

    // Function to escape the given byte according to PPP specification
    static QByteArray escape(unsigned char value);

    // The following values are described in PPP specification.

    static const unsigned char DELIMITER_FLAG = 0x7E; // Start and end of a packet
    static const unsigned char ESCAPE_FLAG = 0x7D;    // Escaping byte.
    static const unsigned char ESCAPED_XOR = 0x20;    // XOR value applied to escaped bytes.

    // This is a reserved command indicating a low level response acknowledgement.
    static const unsigned char ACK = 0x7F; // 255 - 128

    // All command from Arduino have the most significative bit on.
    static const unsigned char REQUEST_MASK = 0x80;

    // Maximum time in millis to wait for Arduino ACK.
    static const long WAIT_TIMEOUT = 500;

    // Maximum number of times to reissue a data packet in case an ACK response
    // is not received.
    static const char MAX_TRIALS = 5;

    // Number of times a packet of data is reissued.
    mutable unsigned char trials;

    // Timer used to reissue a command if no response is received from Arduino.
    mutable QTimer* timer;

    // If the network interface is in READY state a command can be
    // issued immediately, otherwise it has to be queue.
    enum class NetState {
        READY,
        BUSY
    };

    // The data interface state.
    mutable NetState netState;

    // These are all possible states used to parse an incoming response.
    enum class ParserState {
        WAIT_HEADER,
        IN_MESSAGE,
        AFTER_ESCAPE
    };

    // This holds the current state of the incoming data parser.
    mutable ParserState parserState;

    // The serial connection between the Qt client and the Arduino controller.
    QSerialPort* serial;

    // Cache the data being sent to Arduino in case a reissue is required.
    mutable unsigned char requestId;
    mutable QByteArray requestData;

    // This is used to store the response coming from the Arduino controller.
    mutable QByteArray readBuffer;

    // This is the CRC-16 calculated on incoming data.
    mutable unsigned short readCRC;

    void write() const;
};

#endif // BUFFEREDSERIAL_H
