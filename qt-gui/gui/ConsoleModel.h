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

#ifndef CONSOLE_MODEL_H
#define CONSOLE_MODEL_H

#include <QObject>
#include <StageModel.h>

using namespace std;

/**
 * This class holds the status of all rotary and linear stage plus shot
 * parameters.
 * It is used to share data between the main user interface and the
 * messages processing thread .
 */
class ConsoleModel : public QObject
{
    Q_OBJECT

public:

    explicit ConsoleModel(QObject* parent = nullptr);

    StageModel* getLinearStage() const;
    StageModel* getRotaryStage() const;

    unsigned long getPreShootTime() const;
    void setPreShootTime(unsigned long time);

    unsigned long getPostShootTime() const;
    void setPostShootTime(unsigned long time);

    unsigned int getCurrentStep() const;
    void setCurrentStep(unsigned int step);

    unsigned long getFlashTime() const;
    void setFlashTime(unsigned long time);

    unsigned int getTotalSteps() const;

    bool isLightOn() const;
    void setLightOn(bool value);

    bool isArmed() const;
    void setArmed(bool value);
    void fireModelChanged();

signals:

    void currentStepSet(unsigned int step);
    void preShootTimeSet(unsigned long time);
    void postShootTimeSet(unsigned long time);
    void flashTimeSet(unsigned long time);
    void lightSet(bool value);

private:

    StageModel* linearStage;
    StageModel* rotaryStage;

    // Time to wait before taking the shoot, once al motors are idle.
    // GUI sets and MacroWorker gets.
    unsigned long preShootTime = 0;

    // Time to wait since the camera received the shooting impulse.
    // GUI sets and MacroWorker gets.
    unsigned long postShootTime = 0;

    // Time to keep lights on once the camera shutter is open.
    unsigned long flashTime = 0;

    // Current picture number taken during the 3D focus stacking.
    // MacroWorker sets and GUI gets.
    unsigned int currentStep = 0;

    // True when light is on, false otherwise.
    bool lightOn = false;

    // When true signals are emitted if properties change.
    bool armed = true;
};

#endif // CONSOLE_MODEL_H
