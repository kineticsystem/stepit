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

#include "CommandWorker.h"

#include <QBuffer>
#include <QDebug>
#include <QThread>
#include <QAbstractEventDispatcher>
#include <QCoreApplication>
#include "DataUtils.h"

CommandWorker::CommandWorker(QObject* parent) : QObject(parent)
{
    commandId = 0;
    dataInterface = new DataInterface(this);

    connect(dataInterface, &DataInterface::responseDataReceived, this, &CommandWorker::onResponseDataReceived);
    connect(dataInterface, &DataInterface::ready, this, &CommandWorker::onReady);
    connect(dataInterface, &DataInterface::networkError, this, &CommandWorker::onNetworkError);

    connect(this, &CommandWorker::openSignal, this, &CommandWorker::onOpen, Qt::ConnectionType::QueuedConnection);
    connect(this, &CommandWorker::closeSignal, this, &CommandWorker::onClose, Qt::ConnectionType::QueuedConnection);
    connect(this, &CommandWorker::requestReady, this, &CommandWorker::onRequestReady, Qt::ConnectionType::QueuedConnection);
}

void CommandWorker::open(const ConnectionSettings& connectionSettings)
{
    this->connectionSettings = connectionSettings;
    emit openSignal();
}

void CommandWorker::onOpen()
{
    dataInterface->open(this->connectionSettings);
}

bool CommandWorker::isOpen() const
{
    return dataInterface->isOpen();
}

void CommandWorker::close()
{
    emit closeSignal();
}

void CommandWorker::onClose()
{
    dataInterface->close();
}

void CommandWorker::sendRequest(const Request& request) {
    sendRequest(request, nullptr);
}

void CommandWorker::sendRequest(const Request& request,
        const function<void(Response)>& successCallback) {

    QMutexLocker locker(&mutex);

    // Insert the given command into the commands queue.

    //qDebug("Command %d [Th:0x%s]: %s", commandId, qPrintable(QString::number((long long) QThread::currentThreadId(), 16)), qPrintable(request.getDescription()));

    CommandQueueItem queueItem;
    queueItem.setCommandId(commandId);
    queueItem.setCommandData(request.getData());
    queueItem.setSuccessCallback(successCallback);
    commandsQueue.append(queueItem);
    commandId++;

    emit requestReady();
}

void CommandWorker::onRequestReady() {
    sendRequest();
}

void CommandWorker::sendRequest() {
    QMutexLocker locker(&mutex);
    if (!commandsQueue.isEmpty()) {
        CommandQueueItem queueItem = commandsQueue.first();
        QByteArray request;
        request.append(queueItem.getCommandId());
        request.append(queueItem.getCommandData());
        dataInterface->sendRequestData(request); // Send if Data Interface is ready.
    }
}

void CommandWorker::onResponseDataReceived(const QByteArray& responseData) {

    // The first byte is the command sequence.

    unsigned char commandId = responseData[0];
    QByteArray responseBytes = responseData.mid(1, responseData.length() - 1);
    Response response;
    response.setResponseBytes(responseBytes);

    // Invoke the callback function (if any) given with the original command.

    {
        QMutexLocker locker(&mutex);
        if (!commandsQueue.isEmpty()) {
            CommandQueueItem queueItem = commandsQueue.first();
            if (queueItem.getCommandId() == commandId) {
                commandsQueue.dequeue();
                function<void(Response)> successCallback = queueItem.getSuccessCallback();
                if (successCallback != nullptr) {
                    QMetaObject::invokeMethod(this, [=]{ successCallback(response); }, Qt::ConnectionType::QueuedConnection);
                }
            }
        }
    }

    // Send the next command.
    sendRequest();
}

void CommandWorker::onReady() {
    emit readySignal();
}

void CommandWorker::onNetworkError() {
    commandsQueue.clear();
    emit networkErrorSignal();
}
