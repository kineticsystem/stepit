#include "CommandUtils.h"

int CommandUtils::waitingTime(long distance, float acceleration, float maxSpeed) {
    float time = 0;
    distance = qAbs(distance);
    if (distance >= (maxSpeed * maxSpeed / acceleration)) {
        time = distance / maxSpeed + maxSpeed / acceleration;
    } else {
        time = qSqrt(4 * distance / acceleration);
    }
    return qCeil(1000 * time) + 200;
}
