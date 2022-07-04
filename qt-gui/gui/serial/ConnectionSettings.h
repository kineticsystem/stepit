#ifndef SERIALPORTSETTINGS_H
#define SERIALPORTSETTINGS_H

#include <QString>
#include <QSerialPort>

/**
 * This class contains all information required to open a connection to Arduino
 * using the serial port.
 */
class ConnectionSettings
{

public:

    ConnectionSettings();

    QString getPortName() const;
    void setPortName(const QString &value);

    QSerialPort::BaudRate getBaudRate() const;
    void setBaudRate(const  QSerialPort::BaudRate &value);

    QSerialPort::DataBits getDataBits() const;
    void setDataBits(const QSerialPort::DataBits &value);

    QSerialPort::Parity getParity() const;
    void setParity(const QSerialPort::Parity &value);

    QSerialPort::StopBits getStopBits() const;
    void setStopBits(const QSerialPort::StopBits &value);

    QSerialPort::FlowControl getFlowControl() const;
    void setFlowControl(const QSerialPort::FlowControl &value);

private:

    QString portName;
    QSerialPort::BaudRate baudRate;
    QSerialPort::DataBits dataBits;
    QSerialPort::Parity parity;
    QSerialPort::StopBits stopBits;
    QSerialPort::FlowControl flowControl;
};

#endif // SERIALPORTSETTINGS_H
