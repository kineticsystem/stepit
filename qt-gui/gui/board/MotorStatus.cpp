#include "MotorStatus.h"

MotorStatus::MotorStatus()
{
}

long MotorStatus::getPosition() const
{
    return position;
}

void MotorStatus::setPosition(long position)
{
    this->position = position;
}

long MotorStatus::getDistanceToGo() const
{
    return distanceToGo;
}

void MotorStatus::setDistanceToGo(long distanceToGo)
{
    this->distanceToGo = distanceToGo;
}

float MotorStatus::getSpeed() const
{
    return speed;
}

void MotorStatus::setSpeed(float speed)
{
    this->speed = speed;
}
