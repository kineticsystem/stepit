#include "ConnectionSettings.h"

ConnectionSettings::ConnectionSettings()
{

}

QString ConnectionSettings::getPortName() const
{
    return portName;
}

void ConnectionSettings::setPortName(const QString& value)
{
    portName = value;
}

QSerialPort::BaudRate ConnectionSettings::getBaudRate() const
{
    return baudRate;
}

void ConnectionSettings::setBaudRate(const QSerialPort::BaudRate& value)
{
    baudRate = value;
}

QSerialPort::DataBits ConnectionSettings::getDataBits() const
{
    return dataBits;
}

void ConnectionSettings::setDataBits(const QSerialPort::DataBits& value)
{
    dataBits = value;
}

QSerialPort::Parity ConnectionSettings::getParity() const
{
    return parity;
}

void ConnectionSettings::setParity(const QSerialPort::Parity& value)
{
    parity = value;
}

QSerialPort::StopBits ConnectionSettings::getStopBits() const
{
    return stopBits;
}

void ConnectionSettings::setStopBits(const QSerialPort::StopBits& value)
{
    stopBits = value;
}

QSerialPort::FlowControl ConnectionSettings::getFlowControl() const
{
    return flowControl;
}

void ConnectionSettings::setFlowControl(const QSerialPort::FlowControl& value)
{
    flowControl = value;
}







