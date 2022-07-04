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

#ifndef STAGE_MODEL_H
#define STAGE_MODEL_H

#include <QObject>

/**
 * This class holds the status of either a rotary or a linear stage.
 * It is used to share data between the main user interface and the
 * messages processing thread .
 */
class StageModel : public QObject
{
    Q_OBJECT

public:

    explicit StageModel(QObject* parent = nullptr);

    float getSpeed() const;
    void setSpeed(float speed);

    long getStartPosition() const;
    void setStartPosition(long position);

    long getEndPosition() const;
    void setEndPosition(long position);

    unsigned int getSteps() const;
    void setSteps(unsigned int steps);

    long getTargetPosition() const;
    void setTargetPosition(long position);

    float getMaxSpeed() const;
    void setMaxSpeed(float speed);

    float getAcceleration() const;
    void setAcceleration(float acceleration);

    long getCurrentPosition() const;
    void setCurrentPosition(long position);

    bool isArmed() const;
    void setArmed(bool value);
    void fireModelChanged();

signals:

    void startPositionSet(long position);
    void endPositionSet(long position);
    void targetPositionSet(long position);
    void currentPositionSet(long position);
    void speedSet(float speed);
    void maxSpeedSet(float speed);
    void accelerationSet(float acceleration);
    void stepsSet(unsigned int steps);

private:

    // GUI sets and MacroWorker gets.
    unsigned int steps = 1;

    // GUI sets and MacroWorker gets.
    // This is the content of the Go-To-Start fields in the GUI.
    long startPosition = 0;

    // GUI sets and MacroWorker gets.
    // This is the content of the Go-To-End field in the GUI.
    long endPosition = 0;

    // GUI sets and MacroWorker gets.
    // This is the content of the Go-To field in the GUI.
    long targetPosition = 0;

    // GUI sets and MacroWorker gets.
    // Default maximum speed (steps/s).
    float maxSpeed = 1000.0;

    // GUI sets and MacroWorker gets.
    // Default maximum acceleration (steps/s^2).
    float acceleration = 500.0;

    // MacroWorker sets and GUI gets.
    // This is the value of the Current-Position label in the GUI.
    long currentPosition = 0;

    // MacroWorker sets and GUI gets.
    // This is the current speed of motors returned by Arduino.
    float speed = 0;

    // When true signals are emitted if properties change.
    bool armed = true;
};

#endif // STAGE_MODEL_H
