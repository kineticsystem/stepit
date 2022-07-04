#ifndef BOARDUTILS_H
#define BOARDUTILS_H

#include <QtMath>

class CommandUtils
{
public:

    /**
     * Return the time to wait in milliseconds for the stepper to move
     * the given amount of steps, with the given acceleration/deceleration
     * and maximum speed.
     * @param steps The number of steps to move.
     * @param acceleration The acceleration.
     * @param maxSpeed The motor maximum speed.
     * @return The to wait for the movement to complete.
     */
    static int waitingTime(long steps, float acceleration, float maxSpeed);

};

#endif // BOARDUTILS_H
