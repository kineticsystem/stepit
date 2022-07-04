#ifndef MAINWINDOW_H
#define MAINWINDOW_H

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

#include <QMainWindow>
#include <QtConcurrent/QtConcurrent>
#include <QtSerialPort/QSerialPort>
#include <QtSerialPort/QSerialPortInfo>
#include <QStandardItemModel>
#include <QStandardItem>
#include <QList>
#include <QTime>
#include <QDebug>
#include <QSlider>
#include <QLineEdit>

#include "board/CommandWorker.h"
#include "board/Commands.h"
#include "StageModel.h"
#include "MacroWorker.h"
#include "ConsoleModel.h"

namespace Ui
{
    class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget* parent = nullptr);
    ~MainWindow();

private:

    typedef QMainWindow super;

    enum class GuiState {
        DISCONNECTED,
        CONNECTING,
        CONNECTED,
        RUNNING
    };

    GuiState guiState;

    void setWindowState(GuiState guiState);

    Ui::MainWindow* ui;
    void scanPorts();
    QSerialPortInfo arduinoPortInfo;
    QStandardItemModel *portsModel;

    // Available serial ports.
    QList<QSerialPortInfo> ports;

    // Timer reading the position of the sliders and sending corresponding
    // commands to Arduino to move the motors.
    static const unsigned long POLLING_TIME = 100L;
    QTimer *commandTimer;

    // Timer to trig a timeout if not possible to initiate a communication with
    // Arduino.
    QTimer *connectionTimer;

    // This is the thread-safe model used by the GUI to exchange information
    // with the MacroWorker.
    ConsoleModel *model;

    // The command interface to talk to Arduino.
    CommandWorker *commandWorker;

    // State machine controlling the whole 3D stack process.
    MacroWorker *macroWorker;

    static int sgn(float value);
    static int sgn(int value);

    // Slider control the speed using an exponential curve to give
    // more sentibility at slow speed.
    // Slider values and speed are both between -100 and 100.
    static float sliderValueToSpeed(int value);
    static float speedToSliderValue(float speed);

    // Helper array to easily access sliders in for loops.
    QSlider* slider[2];

    // Called when the main window is closed.
    void closeEvent(QCloseEvent* event) override;

    void loadSettings();
    void saveSettings();

    static QString stepsToMillis(long steps);
    long millisToSteps(QString millis);

    static QString stepsToDegrees(long steps);
    static long degreesToSteps(QString degrees);

    void updateProgressBar(unsigned int step);

private slots:

    void onPortsScanned();
    void onLinearStageSliderReleased();
    void onRotaryStageSliderReleased();
    void onConnectButtonPressed();
    void onStartStopButtonPressed();
    void onStopButtonPressed();
    void onTimerTimeout();
    void onConnection();
    void onConnectionTimeout();
    void onLightButtonPressed();
    void onTranslateToStartButtonPressed();
    void onTranslateToEndButtonPressed();
    void onTranslateToButtonPressed();
    void onRotateToStartButtonPressed();
    void onRotateToEndButtonPressed();
    void onRotateToButtonPressed();
    void onTestButtonPressed();
    void onNetworkError();
};

#endif // MAINWINDOW_H
