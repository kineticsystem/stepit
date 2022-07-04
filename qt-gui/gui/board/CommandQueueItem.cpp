#include "CommandQueueItem.h"

CommandQueueItem::CommandQueueItem()
{
}

unsigned char CommandQueueItem::getCommandId() const
{
    return commandId;
}

void CommandQueueItem::setCommandId(unsigned char id)
{
    commandId = id;
}

const QByteArray &CommandQueueItem::getCommandData() const {
    return data;
}

void CommandQueueItem::setCommandData(const QByteArray& data) {
    this->data = data;
}

function<void(Response)> CommandQueueItem::getSuccessCallback() const
{
    return successCallback;
}

void CommandQueueItem::setSuccessCallback(const function<void(Response)>& value)
{
    successCallback = value;
}
