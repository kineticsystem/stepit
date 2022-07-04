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

#include "DataInterface.h"
#include "CrcUtils.h"
#include <QDebug>
#include <QThread>

DataInterface::DataInterface(QObject* parent) : QObject(parent)
{
    // Objects with a parent will be automatically deleted when the parent is
    // deleted.
    // Additionally, if we call moveToThread on the parent, the these objects
    // will be moved to the same thread too.

    parserState = ParserState::WAIT_HEADER;

    serial = new QSerialPort(this);
    connect(serial, &QSerialPort::readyRead, this, &DataInterface::onReadyRead);

    timer = new QTimer(this);
    connect(timer, &QTimer::timeout, this, &DataInterface::onTimeout);

    netState = NetState::READY;
}

bool DataInterface::isOpen() const
{
    return serial->isOpen();
}

bool DataInterface::open(const ConnectionSettings& settings) const {
    serial->setPortName(settings.getPortName());
    serial->setBaudRate(settings.getBaudRate());
    serial->setDataBits(settings.getDataBits());
    serial->setParity(settings.getParity());
    serial->setStopBits(settings.getStopBits());
    serial->setFlowControl(settings.getFlowControl());

    // Open a serial connection with Arduino.
    // On connection Arduino resets itself and moves into a bootstrap phase
    // that may last few seconds. When the following method returns true it
    // means the connection has been established, but it doesn't mean Arduino
    // is ready to communicate. When Arduino is ready it sends back a ready
    // message that causes a "requestDataReceived" signal to be emitted.
    bool result = serial->open(QIODevice::ReadWrite);

    return result;
}

void DataInterface::close() const {
    serial->close();
}

/* Escape the given byte according to the PPP specification.
 * @param value The byte to escape.
 */
QByteArray DataInterface::escape(unsigned char value)
{
    QByteArray bytes;
    if (value == DELIMITER_FLAG || value == ESCAPE_FLAG) {
        bytes.append(ESCAPE_FLAG);
        bytes.append(value ^ ESCAPED_XOR);
    } else {
        bytes.append(value);
    }
    return bytes;
}

void DataInterface::sendRequestData(const QByteArray &requestData) const
{
    // If the network is waiting a response from Arduino don't do anything.
    if (netState == NetState::BUSY) {
        return;
    }
    netState = NetState::BUSY;

    requestId = requestData[0];

    // Calculate Kermit CCITT CRC-16 in Little Endian form.

    unsigned short writeCrc = CrcUtils::calculateCRC(requestData.data(), requestData.length());
    unsigned char writeCrcLSB = (writeCrc & 0xff00) >> 8;
    unsigned char writeCrcMSB = (writeCrc & 0x00ff);

    // Create the packet.

    this->requestData.clear();
    this->requestData.append(DELIMITER_FLAG); // Append delimiter.
    for (auto &it : requestData) {            // Append message.
        this->requestData.append(escape(it));
    }      
    this->requestData.append(escape(writeCrcMSB)); // Append CRC-16 MSB.
    this->requestData.append(escape(writeCrcLSB)); // Append CRC-16 LSB.
    this->requestData.append(DELIMITER_FLAG);      // Append delimiter.

    // Asynchronously send a data payload to the Arduino.

    trials = 0;
    timer->start(WAIT_TIMEOUT);
    write();
}

void DataInterface::write() const {
    //qDebug("Data out (%d) [Th:%d]: %s", trials, QThread::currentThreadId(), qPrintable(DataUtils::toHex(requestData)));
    serial->write(this->requestData, this->requestData.size());
}

/* The client communicates with the Arduino with a request-response
 * unidirectional protocol: the client sends a command and the Arduino answers.
 */
void DataInterface::onReadyRead() const
{
    QByteArray dataIn = serial->readAll();
    //qDebug("Data in: %s", qPrintable(DataUtils::toHex(dataIn)));

    for (auto &in : dataIn) {

        // State machine to detect messages from the stream of data.

        switch (parserState) {
            case ParserState::WAIT_HEADER:
                if (in == DELIMITER_FLAG) {
                    readCRC = 0;
                    parserState = ParserState::IN_MESSAGE;
                }
                break;
            case ParserState::IN_MESSAGE:

                if (in == ESCAPE_FLAG) { // Ignore the escape character.
                    parserState = ParserState::AFTER_ESCAPE;
                } else if (in == DELIMITER_FLAG) {

                    // A packet must contain minimum:
                    // 1 - a command sequence (1 byte);
                    // 2 - a command (1 byte);
                    // 3 - a CRC (2 bytes).

                    if (readBuffer.size() >= 4) {

                        if (readCRC == 0) { // CRC-16 validation.

                            QByteArray data = readBuffer.mid(0, readBuffer.length() - 2);
                            const unsigned char cmd = data[1];

                            //qDebug("Payload in [Th:%d]: %s",  QThread::currentThreadId(), qPrintable(DataUtils::toHex(dataIn)));

                            if (cmd == ACK) {
                                // Acknowledgement.
                                requestData.clear();
                                trials = 0;
                                timer->stop();
                            } else if ((cmd & REQUEST_MASK) != 0) {
                                // Arduino ready to communicate.
                                emit ready();
                            } else {
                                // Arduino response.
                                netState = NetState::READY;
                                emit responseDataReceived(data);
                            }

                            parserState = ParserState::WAIT_HEADER;
                        }
                    }

                    readCRC = 0;
                    readBuffer.clear();

                } else {
                    readCRC = CrcUtils::updateCRC(readCRC, in);
                    readBuffer.append(in);
                }

                break;
            case ParserState::AFTER_ESCAPE:
                readCRC = CrcUtils::updateCRC(readCRC, in ^ ESCAPED_XOR);
                readBuffer.append(in ^ ESCAPED_XOR);
                parserState = ParserState::IN_MESSAGE;
                break;
        }
    }
}

/* In case of errors in transmitting a packet or in case of missing AWK,
 * after some time the packet is reissued for a maximum number of times.
 * After that a network error signal is emitted.
 */
void DataInterface::onTimeout() const
{
    timer->stop();
    if (trials < MAX_TRIALS) {
        trials++;
        timer->start(WAIT_TIMEOUT);
        write();
    } else {
        trials = 0;
        readCRC = 0;
        readBuffer.clear();
        emit networkError();
    }
}
