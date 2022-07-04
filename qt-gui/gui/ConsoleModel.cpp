#include "ConsoleModel.h"

ConsoleModel::ConsoleModel(QObject* parent) : QObject(parent)
{
    linearStage = new StageModel(this);
    rotaryStage = new StageModel(this);
}

StageModel* ConsoleModel::getLinearStage() const
{
    return linearStage;
}

StageModel* ConsoleModel::getRotaryStage() const
{
    return rotaryStage;
}

unsigned long ConsoleModel::getPreShootTime() const
{
    return preShootTime;
}

void ConsoleModel::setPreShootTime(unsigned long time)
{
    if (preShootTime != time) {
        preShootTime = time;
        if (armed) {
            emit preShootTimeSet(time);
        }
    }
}

unsigned long ConsoleModel::getPostShootTime() const
{
    return postShootTime;
}

void ConsoleModel::setPostShootTime(unsigned long time)
{
    if (postShootTime != time) {
        postShootTime = time;
        if (armed) {
            emit postShootTimeSet(time);
        }
    }
}

unsigned int ConsoleModel::getCurrentStep() const
{
    return currentStep;
}

void ConsoleModel::setCurrentStep(unsigned int step)
{
    if (currentStep != step) {
        currentStep = step;
        if (armed) {
            emit currentStepSet(step);
        }
    }
}

unsigned long ConsoleModel::getFlashTime() const
{
    return flashTime;
}

void ConsoleModel::setFlashTime(unsigned long time)
{
    if (flashTime != time) {
        flashTime = time;
        if (armed) {
            emit flashTimeSet(time);
        }
    }
}

bool ConsoleModel::isLightOn() const
{
    return lightOn;
}

void ConsoleModel::setLightOn(bool value)
{
    if (lightOn != value) {
        lightOn = value;
        if (armed) {
            emit lightSet(value);
        }
    }
}

unsigned int ConsoleModel::getTotalSteps() const
{
    unsigned int totalSteps = this->getLinearStage()->getSteps() * this->getRotaryStage()->getSteps();
    return totalSteps;
}

/*/////////////////////////////////////////////////////////////////////////////
 * Signals control.
 */

bool ConsoleModel::isArmed() const
{
    return armed;
}

void ConsoleModel::setArmed(bool value)
{
    armed = value;
}

void ConsoleModel::fireModelChanged()
{
    emit currentStepSet(this->currentStep);
    emit preShootTimeSet(this->preShootTime);
    emit postShootTimeSet(this->postShootTime);
    emit flashTimeSet(this->flashTime);
    emit lightSet(this->lightOn);
}
