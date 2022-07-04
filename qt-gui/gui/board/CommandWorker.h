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

#ifndef CONTROLLERINTERFACE_H
#define CONTROLLERINTERFACE_H

#include <QObject>
#include <QTimer>
#include <QSerialPort>
#include <QDateTime>
#include <QQueue>
#include <QMutex>
#include <QMutexLocker>
#include <QThread>
#include "Response.h"
#include "Request.h"
#include "CommandQueueItem.h"
#include "DataInterface.h"
#include "ConsoleModel.h"

using namespace std;

/**
 * This class send commands to the Arduino controller using the serial port.
 * Each method is blocking, this means that when a command is delivered to the
 * Arduino, the method always waits for a response.
 * When a response is not received in the expected timeframe, a timeout would
 * occur.
 * In case of timeout the command is reissued again until the maximum number
 * of trials is reached. In this last case an error is returned.
 * If the Arduino doesn't receive a continuos flow of commands, it
 * automatically stops all stepper motors.
 */
class CommandWorker : public QObject
{
    Q_OBJECT

public:

    explicit CommandWorker(QObject* parent = nullptr);

    /**
     * Send a command and when a reponse is returned then call the given
     * callback function.
     * @param command The command.
     * @param lambda The callback function.
     */
    void sendRequest(const Request& request);
    void sendRequest(const Request& request,
        const function<void(Response)>& successCallback);

    void open(const ConnectionSettings& settings);
    void close();
    bool isOpen() const;

private slots:

    // Called when Arduino answers to a command.
    void onResponseDataReceived(const QByteArray& responseData);

    // Called when Arduino is ready to communicate.
    void onReady();

    void onNetworkError();

    void onOpen();

    void onClose();

    void onRequestReady();

signals:

    void responseReceived(const Response& response) const;

    void openSignal() const;

    void closeSignal() const;

    void requestReady() const;

    void readySignal() const;

    void networkErrorSignal() const;

private:

    mutable ConnectionSettings connectionSettings;

    // This class is the Link Layer used to encapsulate a command into a
    // proper packed (a Point-to-Point Protocol implementation) and deliver
    // it to the Arduino using the serial port.
    DataInterface* dataInterface;

    // This is a sequence number used to match a server response to the
    // corresponsing command request.
    mutable unsigned char commandId;

    // Mutex to control access to the commandsQueue.
    QMutex mutex;

    // Store the command id and the corresponding callback.
    mutable QQueue<CommandQueueItem> commandsQueue;

    // Send the first command request available in the queue.
    void sendRequest();
};

#endif // CONTROLLERINTERFACE_H
