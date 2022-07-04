#ifndef MOTORSTATUS_H
#define MOTORSTATUS_H

class MotorStatus
{
public:
    explicit MotorStatus();
    long getPosition() const;
    void setPosition(long position);
    long getDistanceToGo() const;
    void setDistanceToGo(long distanceToGo);
    float getSpeed() const;
    void setSpeed(float speed);

private:
    float speed;
    long position;
    long distanceToGo;
};

#endif // MOTORSTATUS_H
