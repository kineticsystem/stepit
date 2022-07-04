#ifndef COMMANDLOOKUPITEM_H
#define COMMANDLOOKUPITEM_H

#include "Request.h"
#include "Response.h"
#include <functional>

using namespace std;

class CommandQueueItem
{

public:

    CommandQueueItem();

    function<void(Response)> getSuccessCallback() const;
    void setSuccessCallback(const function<void(Response)>& value);

    const QByteArray& getCommandData() const;
    void setCommandData(const QByteArray& data);

    unsigned char getCommandId() const;
    void setCommandId(unsigned char value);

private:

    QByteArray data;         // The command data.
    unsigned char commandId; // The command sequence id.

    function<void(Response)> successCallback;
};

#endif // COMMANDLOOKUPITEM_H
