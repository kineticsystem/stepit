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

#include "MainWindow.h"
#include "ui_MainWindow.h"
#include <QDebug>
#include <QThread>
#include <QtMath>

#include "MacroWorker.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    this->setFixedSize(this->geometry().width(), this->geometry().height());

    slider[0] = ui->translationVerticalSlider;
    slider[1] = ui->rotationVerticalSlider;

    // Combobox holding the list of all USB ports where a device is connected,
    // including Arduino.
    portsModel = new QStandardItemModel(this);
    ui->comboBox->setModel(portsModel);
    ui->comboBox->setEnabled(false);

    // Model to thread-safe exchange information between the GUI and the MacroWorker.
    model = new ConsoleModel();

    // Interface to send commands to Arduino and process reponses.
    commandWorker = new CommandWorker();

    // State machine automatically controlling the 3D focus staking.
    macroWorker = new MacroWorker(commandWorker);
    macroWorker->setModel(model);

    // Thread managing the Macro Worker.
    QThread *macroThread = new QThread(this);
    macroWorker->moveToThread(macroThread);
    macroThread->start();

    // Future task to scan all possible USB ports where a device is connected,
    // including Arduino.
    QFuture<void> task = QtConcurrent::run(this, &MainWindow::scanPorts);
    QFutureWatcher<void> taskWatcher;
    taskWatcher.setFuture(task);
    connect(&taskWatcher, &QFutureWatcher<void>::finished, this, &MainWindow::onPortsScanned);

    // Signal received when a data packed cannot be sent to Arduino.
    connect(commandWorker, &CommandWorker::networkErrorSignal, this, &MainWindow::onNetworkError);

    // Signal received when Arduino is ready to communicate.
    connect(commandWorker, &CommandWorker::readySignal, this, &MainWindow::onConnection);

    // Model events.

    connect(model, &ConsoleModel::preShootTimeSet, this, [this](unsigned long time) {
        ui->preShootField->setText(QString::number(time));
    });
    connect(model, &ConsoleModel::postShootTimeSet, this, [this](unsigned long time) {
        ui->postShootField->setText(QString::number(time));
    });
    connect(model, &ConsoleModel::flashTimeSet, this, [this](unsigned long time) {
        ui->flashTimeField->setText(QString::number(time));
    });
    connect(model, &ConsoleModel::currentStepSet, this, [this](unsigned int step) {
        updateProgressBar(step);
    });
    connect(model, &ConsoleModel::lightSet, this, [this](bool value) {
        if (value) {
            ui->lightButton->setText("Light OFF");
        } else {
            ui->lightButton->setText("Light ON");
        }
    });

    connect(model->getLinearStage(), &StageModel::startPositionSet, this, [this](long position) {
        ui->translationStartField->setText(stepsToMillis(position));
    });
    connect(model->getLinearStage(), &StageModel::endPositionSet, this, [this](long position) {
        ui->translationEndField->setText(stepsToMillis(position));
    });
    connect(model->getLinearStage(), &StageModel::stepsSet, this, [this](unsigned int steps) {
        ui->translationStepsField->setValue(steps);
        updateProgressBar(0);
    });
    connect(model->getLinearStage(), &StageModel::currentPositionSet, this, [this](long position) {
        ui->currentTranslationLabel->setText(stepsToMillis(position));
    });
    connect(model->getLinearStage(), &StageModel::targetPositionSet, this, [this](long position) {
        ui->translationField->setText(stepsToMillis(position));
    });
    connect(model->getLinearStage(), &StageModel::speedSet, this, [this](float speed) {
        if (!(slider[0]->isSliderDown())) {
            slider[0]->setValue(speedToSliderValue(speed));
        }
    });

    connect(model->getRotaryStage(), &StageModel::startPositionSet, this, [this](long position) {
        ui->rotationStartField->setText(stepsToDegrees(position));
    });
    connect(model->getRotaryStage(), &StageModel::endPositionSet, this, [this](long position) {
        ui->rotationEndField->setText(stepsToDegrees(position));
    });
    connect(model->getRotaryStage(), &StageModel::stepsSet, this, [this](unsigned int steps) {
        ui->rotationStepsField->setValue(steps);
        updateProgressBar(0);
    });
    connect(model->getRotaryStage(), &StageModel::currentPositionSet, this, [this](long position) {
        ui->currentRotationLabel->setText(stepsToDegrees(position));
    });
    connect(model->getRotaryStage(), &StageModel::targetPositionSet, this, [this](long position) {
        ui->rotationField->setText(stepsToDegrees(position));
    });
    connect(model->getRotaryStage(), &StageModel::speedSet, this, [this](float speed) {
        if (!(slider[1]->isSliderDown())) {
            slider[1]->setValue(speedToSliderValue(speed));
        }
    });

    // Operation buttons.

    connect(ui->connectButton, &QPushButton::pressed, this, &MainWindow::onConnectButtonPressed);
    connect(ui->startStopButton, &QPushButton::pressed, this, &MainWindow::onStartStopButtonPressed);
    connect(ui->testButton, &QPushButton::pressed, this, &MainWindow::onTestButtonPressed);

    // Linear stage buttons.

    connect(ui->translateToStartButton,
            &QPushButton::pressed,
            this,
            &MainWindow::onTranslateToStartButtonPressed);
    connect(ui->translateToEndButton,
            &QPushButton::pressed,
            this,
            &MainWindow::onTranslateToEndButtonPressed);
    connect(ui->translateToButton,
            &QPushButton::pressed,
            this,
            &MainWindow::onTranslateToButtonPressed);

    connect(ui->setTranslationStartButton, &QPushButton::pressed, this, [&]() {
        int position = model->getLinearStage()->getCurrentPosition();
        model->getLinearStage()->setStartPosition(position);
    });
    connect(ui->setTranslationEndButton, &QPushButton::pressed, this, [&]() {
        int position = model->getLinearStage()->getCurrentPosition();
        model->getLinearStage()->setEndPosition(position);
    });
    connect(ui->setCurrentTranslationButton, &QPushButton::pressed, this, [this]() {
        int position = model->getLinearStage()->getTargetPosition();
        commandWorker->sendRequest(MotorCurrentPositionCommand(0, position));
    });

    // Rotary stage buttons.

    connect(ui->rotateToStartButton,
            &QPushButton::pressed,
            this,
            &MainWindow::onRotateToStartButtonPressed);
    connect(ui->rotateToEndButton,
            &QPushButton::pressed,
            this,
            &MainWindow::onRotateToEndButtonPressed);
    connect(ui->rotateToButton, &QPushButton::pressed, this, &MainWindow::onRotateToButtonPressed);

    connect(ui->setRotationStartButton, &QPushButton::pressed, this, [&]() {
        int position = model->getRotaryStage()->getCurrentPosition();
        model->getRotaryStage()->setStartPosition(position);
    });
    connect(ui->setRotationEndButton, &QPushButton::pressed, this, [&]() {
        int position = model->getRotaryStage()->getCurrentPosition();
        model->getRotaryStage()->setEndPosition(position);
    });
    connect(ui->setCurrentRotationButton, &QPushButton::pressed, this, [this]() {
        int position = model->getRotaryStage()->getTargetPosition();
        commandWorker->sendRequest(MotorCurrentPositionCommand(1, position));
    });

    // Flash.

    connect(ui->preShootField, &QLineEdit::editingFinished, this, [&]() {
        unsigned long time = ui->preShootField->text().toLong();
        model->setPreShootTime(time);
    });
    connect(ui->flashTimeField, &QLineEdit::editingFinished, this, [&]() {
        unsigned long time = ui->flashTimeField->text().toLong();
        model->setFlashTime(time);
    });
    connect(ui->postShootField, &QLineEdit::editingFinished, this, [&]() {
        unsigned long time = ui->postShootField->text().toLong();
        model->setPostShootTime(time);
    });

    // Linear stage text fields events.

    connect(ui->translationStartField, &QLineEdit::editingFinished, this, [&]() {
        int position = millisToSteps(ui->translationStartField->text());
        model->getLinearStage()->setStartPosition(position);
    });
    connect(ui->translationEndField, &QLineEdit::editingFinished, this, [&]() {
        int position = millisToSteps(ui->translationEndField->text());
        model->getLinearStage()->setEndPosition(position);
    });
    connect(ui->translationField, &QLineEdit::editingFinished, this, [&]() {
        int position = millisToSteps(ui->translationField->text());
        model->getLinearStage()->setTargetPosition(position);
    });
    connect(ui->translationStepsField,
            static_cast<void (QSpinBox::*)(const int)>(&QSpinBox::valueChanged),
            this,
            [&](int steps) { model->getLinearStage()->setSteps(steps); });

    // Rotary stage text fields events.

    connect(ui->rotationStartField, &QLineEdit::editingFinished, this, [&]() {
        int position = degreesToSteps(ui->rotationStartField->text());
        model->getRotaryStage()->setStartPosition(position);
    });
    connect(ui->rotationEndField, &QLineEdit::editingFinished, this, [&]() {
        int position = degreesToSteps(ui->rotationEndField->text());
        model->getRotaryStage()->setEndPosition(position);
    });
    connect(ui->rotationField, &QLineEdit::editingFinished, this, [&]() {
        int position = degreesToSteps(ui->rotationField->text());
        model->getRotaryStage()->setTargetPosition(position);
    });
    connect(ui->rotationStepsField,
            static_cast<void (QSpinBox::*)(const int)>(&QSpinBox::valueChanged),
            this,
            [&](int steps) { model->getRotaryStage()->setSteps(steps); });

    // Timer to trig a timeout if not possible to initiate a communication with
    // Arduino.
    connectionTimer = new QTimer(this);
    connect(connectionTimer, &QTimer::timeout, this, &MainWindow::onConnectionTimeout);

    // Timer reading the position of the sliders and sending corresponding
    // commands to Arduino to move the motors.
    commandTimer = new QTimer(this);
    connect(commandTimer, &QTimer::timeout, this, &MainWindow::onTimerTimeout);

    // State machine events
    connect(
        macroWorker,
        &MacroWorker::finished,
        this,
        [this]() { setWindowState(GuiState::CONNECTED); },
        Qt::ConnectionType::QueuedConnection);

    // Lights
    connect(ui->lightButton, &QPushButton::pressed, this, &MainWindow::onLightButtonPressed);

    // Sliders
    connect(ui->translationVerticalSlider,
            &QSlider::sliderReleased,
            this,
            &MainWindow::onLinearStageSliderReleased);
    connect(ui->rotationVerticalSlider,
            &QSlider::sliderReleased,
            this,
            &MainWindow::onRotaryStageSliderReleased);

    ui->progressBar->setFormat("");

    setWindowState(GuiState::DISCONNECTED);

    // Load saved values and update GUI.

    loadSettings();
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::updateProgressBar(unsigned int step)
{
    unsigned int stacks = ui->rotationStepsField->value();
    unsigned int slices = ui->translationStepsField->value();
    unsigned int steps = slices * stacks;
    ui->progressBar->setValue(step);
    ui->progressBar->setMaximum(steps);
    ui->progressBar->setFormat("Shot " + QString::number(step) + " of " + QString::number(steps)
                               + " taken");
}

void MainWindow::loadSettings()
{
    model->setArmed(false);
    model->getLinearStage()->setArmed(false);
    model->getRotaryStage()->setArmed(false);

    QSettings settings;

    model->getLinearStage()->setStartPosition(settings.value("translationStart", 0).toLongLong());
    model->getLinearStage()->setEndPosition(settings.value("translationEnd", 0).toLongLong());
    model->getLinearStage()->setTargetPosition(settings.value("translationTarget", 0).toLongLong());
    model->getLinearStage()->setCurrentPosition(
        settings.value("currentTranslation", 0).toLongLong());
    model->getLinearStage()->setSteps(settings.value("linearSteps", 1).toUInt());

    model->getRotaryStage()->setStartPosition(settings.value("rotationStart", 0).toLongLong());
    model->getRotaryStage()->setEndPosition(settings.value("rotationEnd", 0).toLongLong());
    model->getRotaryStage()->setTargetPosition(settings.value("rotationTarget", 0).toLongLong());
    model->getRotaryStage()->setCurrentPosition(settings.value("currentRotation", 0).toLongLong());
    model->getRotaryStage()->setSteps(settings.value("rotationSteps", 1).toUInt());

    model->setPreShootTime(settings.value("preShootTime", 200).toLongLong());
    model->setPostShootTime(settings.value("postShootTime", 200).toLongLong());
    model->setFlashTime(settings.value("flashTime", 200).toULongLong());

    model->fireModelChanged();
    model->getLinearStage()->fireModelChanged();
    model->getRotaryStage()->fireModelChanged();

    model->setArmed(true);
    model->getLinearStage()->setArmed(true);
    model->getRotaryStage()->setArmed(true);
}

void MainWindow::saveSettings()
{
    QSettings settings;

    settings.setValue("translationStart",
                      QVariant((int) model->getLinearStage()->getStartPosition()));
    settings.setValue("translationEnd", QVariant((int) model->getLinearStage()->getEndPosition()));
    settings.setValue("translationTarget",
                      QVariant((int) model->getLinearStage()->getTargetPosition()));
    settings.setValue("currentTranslation",
                      QVariant((int) model->getLinearStage()->getCurrentPosition()));
    settings.setValue("linearSteps", QVariant((int) model->getLinearStage()->getSteps()));

    settings.setValue("rotationStart", QVariant((int) model->getRotaryStage()->getStartPosition()));
    settings.setValue("rotationEnd", QVariant((int) model->getRotaryStage()->getEndPosition()));
    settings.setValue("rotationTarget",
                      QVariant((int) model->getRotaryStage()->getTargetPosition()));
    settings.setValue("currentRotation",
                      QVariant((int) model->getRotaryStage()->getCurrentPosition()));
    settings.setValue("rotationSteps", QVariant((int) model->getRotaryStage()->getSteps()));

    settings.setValue("preShootTime", QVariant((int) model->getPreShootTime()));
    settings.setValue("postShootTime", QVariant((int) model->getPostShootTime()));
    settings.setValue("flashTime", QVariant((int) model->getFlashTime()));
}

/* Method called when the application is closed.
 * @param event The closing event.
 */
void MainWindow::closeEvent(QCloseEvent *event)
{
    saveSettings();
    super::closeEvent(event);
}

void MainWindow::onNetworkError()
{
    setWindowState(MainWindow::GuiState::DISCONNECTED);
}

/* Set the GUI appearance based  on a given state.
 * @param the GUI state.
 */
void MainWindow::setWindowState(MainWindow::GuiState state)
{
    this->guiState = state;
    switch (state) {
    case GuiState::DISCONNECTED:
        ui->comboBox->setEnabled(true);
        ui->connectButton->setEnabled(true);
        ui->translateToStartButton->setEnabled(false);
        ui->setTranslationStartButton->setEnabled(false);
        ui->translateToEndButton->setEnabled(false);
        ui->translateToButton->setEnabled(false);
        ui->setTranslationEndButton->setEnabled(false);
        ui->rotateToStartButton->setEnabled(false);
        ui->rotateToButton->setEnabled(false);
        ui->setRotationStartButton->setEnabled(false);
        ui->rotateToEndButton->setEnabled(false);
        ui->setRotationEndButton->setEnabled(false);
        ui->preShootField->setEnabled(false);
        ui->postShootField->setEnabled(false);
        ui->startStopButton->setEnabled(false);
        ui->startStopButton->setText("Stack");
        ui->translationStepsField->setEnabled(false);
        ui->rotationStepsField->setEnabled(false);
        ui->translationVerticalSlider->setEnabled(false);
        ui->rotationVerticalSlider->setEnabled(false);
        ui->lightButton->setEnabled(false);
        ui->testButton->setEnabled(false);
        ui->setCurrentTranslationButton->setEnabled(false);
        ui->setCurrentRotationButton->setEnabled(false);
        ui->connectButton->setText("Connect");
        ui->flashTimeField->setEnabled(false);
        ui->translationField->setEnabled(false);
        ui->rotationField->setEnabled(false);
        ui->translationStartField->setEnabled(false);
        ui->translationEndField->setEnabled(false);
        ui->rotationStartField->setEnabled(false);
        ui->rotationEndField->setEnabled(false);
        break;
    case GuiState::CONNECTING:
        ui->comboBox->setEnabled(false);
        ui->connectButton->setEnabled(false);
        ui->translateToStartButton->setEnabled(false);
        ui->setTranslationStartButton->setEnabled(false);
        ui->translateToEndButton->setEnabled(false);
        ui->translateToButton->setEnabled(false);
        ui->setTranslationEndButton->setEnabled(false);
        ui->rotateToStartButton->setEnabled(false);
        ui->setRotationStartButton->setEnabled(false);
        ui->rotateToEndButton->setEnabled(false);
        ui->rotateToButton->setEnabled(false);
        ui->setRotationEndButton->setEnabled(false);
        ui->preShootField->setEnabled(false);
        ui->postShootField->setEnabled(false);
        ui->startStopButton->setEnabled(false);
        ui->startStopButton->setText("Stack");
        ui->translationStepsField->setEnabled(false);
        ui->rotationStepsField->setEnabled(false);
        ui->translationVerticalSlider->setEnabled(false);
        ui->rotationVerticalSlider->setEnabled(false);
        ui->lightButton->setEnabled(false);
        ui->testButton->setEnabled(false);
        ui->setCurrentTranslationButton->setEnabled(false);
        ui->setCurrentRotationButton->setEnabled(false);
        ui->connectButton->setText("Connecting...");
        ui->flashTimeField->setEnabled(false);
        ui->translationField->setEnabled(false);
        ui->rotationField->setEnabled(false);
        ui->translationStartField->setEnabled(false);
        ui->translationEndField->setEnabled(false);
        ui->rotationStartField->setEnabled(false);
        ui->rotationEndField->setEnabled(false);
        break;
    case GuiState::CONNECTED:
        ui->comboBox->setEnabled(false);
        ui->connectButton->setEnabled(true);
        ui->translateToStartButton->setEnabled(true);
        ui->setTranslationStartButton->setEnabled(true);
        ui->translateToEndButton->setEnabled(true);
        ui->translateToButton->setEnabled(true);
        ui->setTranslationEndButton->setEnabled(true);
        ui->rotateToStartButton->setEnabled(true);
        ui->setRotationStartButton->setEnabled(true);
        ui->rotateToEndButton->setEnabled(true);
        ui->rotateToButton->setEnabled(true);
        ui->setRotationEndButton->setEnabled(true);
        ui->preShootField->setEnabled(true);
        ui->postShootField->setEnabled(true);
        ui->startStopButton->setEnabled(true);
        ui->startStopButton->setText("Stack");
        ui->translationStepsField->setEnabled(true);
        ui->rotationStepsField->setEnabled(true);
        ui->translationVerticalSlider->setEnabled(true);
        ui->rotationVerticalSlider->setEnabled(true);
        ui->lightButton->setEnabled(true);
        ui->testButton->setEnabled(true);
        ui->setCurrentTranslationButton->setEnabled(true);
        ui->setCurrentRotationButton->setEnabled(true);
        ui->connectButton->setText("Disconnect");
        ui->flashTimeField->setEnabled(true);
        ui->translationField->setEnabled(true);
        ui->rotationField->setEnabled(true);
        ui->translationStartField->setEnabled(true);
        ui->translationEndField->setEnabled(true);
        ui->rotationStartField->setEnabled(true);
        ui->rotationEndField->setEnabled(true);
        break;
    case GuiState::RUNNING:
        ui->comboBox->setEnabled(false);
        ui->connectButton->setEnabled(false);
        ui->translateToStartButton->setEnabled(false);
        ui->setTranslationStartButton->setEnabled(false);
        ui->translateToEndButton->setEnabled(false);
        ui->translateToButton->setEnabled(false);
        ui->setTranslationEndButton->setEnabled(false);
        ui->rotateToStartButton->setEnabled(false);
        ui->setRotationStartButton->setEnabled(false);
        ui->rotateToEndButton->setEnabled(false);
        ui->rotateToButton->setEnabled(false);
        ui->setRotationEndButton->setEnabled(false);
        ui->preShootField->setEnabled(false);
        ui->postShootField->setEnabled(false);
        ui->startStopButton->setEnabled(true);
        ui->startStopButton->setText("Stop");
        ui->translationStepsField->setEnabled(false);
        ui->rotationStepsField->setEnabled(false);
        ui->translationVerticalSlider->setEnabled(false);
        ui->rotationVerticalSlider->setEnabled(false);
        ui->lightButton->setEnabled(false);
        ui->testButton->setEnabled(false);
        ui->setCurrentTranslationButton->setEnabled(false);
        ui->setCurrentRotationButton->setEnabled(false);
        ui->connectButton->setText("Disconnect");
        ui->flashTimeField->setEnabled(false);
        ui->translationField->setEnabled(false);
        ui->rotationField->setEnabled(false);
        ui->translationStartField->setEnabled(false);
        ui->translationEndField->setEnabled(false);
        ui->rotationStartField->setEnabled(false);
        ui->rotationEndField->setEnabled(false);
        break;
    }
}

/* Send moving commands to Arduino.
 * A timer read the status of the GUI sliders and send the moving commands when
 * speeds or direction change.
 */
void MainWindow::onTimerTimeout()
{
    StageModel *stages[] = {model->getLinearStage(), model->getRotaryStage()};
    if (commandWorker->isOpen() && macroWorker->isNotRunning()) {
        for (int i = 0; i < 2; i++) {
            if (slider[i]->isSliderDown()) {
                float speed = sliderValueToSpeed(slider[i]->value()) * sgn(slider[i]->value());
                if (stages[i]->getSpeed() != speed) {
                    if (sgn(stages[i]->getSpeed()) != sgn(speed)) {
                        if (sgn(speed) > 0) {
                            commandWorker->sendRequest(
                                MotorMoveCommand(i, MotorMoveCommand::Direction::ANTI_CLOCKWISE));
                        } else {
                            commandWorker->sendRequest(
                                MotorMoveCommand(i, MotorMoveCommand::Direction::CLOCKWISE));
                        }
                    }
                    commandWorker->sendRequest(MotorTargetSpeedCommand(i, qAbs(speed)));
                    stages[i]->setSpeed(speed);
                }
            }
        }
    }
}

/* Shoot a picture. */
void MainWindow::onTestButtonPressed()
{
    setWindowState(GuiState::RUNNING);
    if (model->isLightOn()) {
        commandWorker->sendRequest(LightOffCommand(),
                                   [this](const Response &) { model->setLightOn(false); });
    }
    commandWorker->sendRequest(ShootCommand(model->getFlashTime()),
                               [this](const Response &) { setWindowState(GuiState::CONNECTED); });
}

/* Convert motor microsteps to linear stage mm.
 * @param steps The steps to convert.
 */
QString MainWindow::stepsToMillis(long steps)
{
    double millimetersPosition = steps / 2011.725105;
    return QString::number(millimetersPosition, 'f', 3);
}

/* Convert linear stage mm to motor microsteps.
 * @param millis The millis to convert.
 */
long MainWindow::millisToSteps(QString millis)
{
    double millimetersPosition = millis.toDouble();
    double steps = qRound(2011.725105 * millimetersPosition);
    return steps;
}

/* Convert motor microsteps to rotary stage degrees.
 * @param steps The steps to convert.
 */
QString MainWindow::stepsToDegrees(long steps)
{
    double degreesPosition = steps * 9.0 / 6400.0;
    return QString::number(degreesPosition, 'f', 3);
}

/* Convert rotary stage degrees to motor microsteps.
 * @param steps The degrees to convert.
 */
long MainWindow::degreesToSteps(QString degrees)
{
    double degreesPosition = degrees.toDouble();
    double steps = qRound(degreesPosition * 6400.0 / 9.0);
    return steps;
}

/* Move the linear stage to the start position. */
void MainWindow::onTranslateToStartButtonPressed()
{
    setWindowState(GuiState::RUNNING);
    macroWorker->execute(MacroWorker::Action::TRANSLATE_TO_START);
}

/* Move the linear stage to the end position. */
void MainWindow::onTranslateToEndButtonPressed()
{
    setWindowState(GuiState::RUNNING);
    macroWorker->execute(MacroWorker::Action::TRANSLATE_TO_END);
}

/* Move the linear stage to the given position. */
void MainWindow::onTranslateToButtonPressed()
{
    setWindowState(GuiState::RUNNING);
    macroWorker->execute(MacroWorker::Action::TRANSLATE_TO);
}

/* Move the rotary stage to the start position. */
void MainWindow::onRotateToStartButtonPressed()
{
    setWindowState(GuiState::RUNNING);
    macroWorker->execute(MacroWorker::Action::ROTATE_TO_START);
}

/* Move the rotary stage to the end position. */
void MainWindow::onRotateToEndButtonPressed()
{
    setWindowState(GuiState::RUNNING);
    macroWorker->execute(MacroWorker::Action::ROTATE_TO_END);
}

/* Move the rotary stage to the given position. */
void MainWindow::onRotateToButtonPressed()
{
    setWindowState(GuiState::RUNNING);
    macroWorker->execute(MacroWorker::Action::ROTATE_TO);
}

/* Start the stacking process or interrupt any current process.*/
void MainWindow::onStartStopButtonPressed()
{
    if (guiState == GuiState::RUNNING) {
        macroWorker->execute(MacroWorker::Action::STOP);
        setWindowState(GuiState::CONNECTED);
    } else if (guiState == GuiState::CONNECTED) {
        macroWorker->execute(MacroWorker::Action::SHOOT_STACK);
        setWindowState(GuiState::RUNNING);
    }
}

void MainWindow::onStopButtonPressed()
{
    setWindowState(GuiState::CONNECTED);
    macroWorker->execute(MacroWorker::Action::STOP);
}

void MainWindow::onLightButtonPressed()
{
    if (model->isLightOn()) {
        commandWorker->sendRequest(LightOffCommand(),
                                   [this](const Response &) { model->setLightOn(false); });

    } else {
        commandWorker->sendRequest(LightOnCommand(),
                                   [this](const Response &) { model->setLightOn(true); });
    }
}

/* Connect the client to Arduino using serial port. */
void MainWindow::onConnectButtonPressed()
{
    if (!commandWorker->isOpen()) {
        ConnectionSettings settings;
        settings.setPortName(ui->comboBox->currentText());
        settings.setBaudRate(QSerialPort::Baud9600);
        settings.setDataBits(QSerialPort::Data8);
        settings.setParity(QSerialPort::NoParity);
        settings.setStopBits(QSerialPort::OneStop);
        settings.setFlowControl(QSerialPort::NoFlowControl);

        commandWorker->open(settings);
        setWindowState(GuiState::CONNECTING);

        connectionTimer->start(5000); // Timeout in case something goes wrong with the connection.

    } else {
        commandTimer->stop();
        commandWorker->close();
        setWindowState(GuiState::DISCONNECTED);
    }
}

void MainWindow::onConnectionTimeout()
{
    connectionTimer->stop();
    commandWorker->close();
    setWindowState(GuiState::DISCONNECTED);
}

/* This method is called on succesful connection to Arduino. */
void MainWindow::onConnection()
{
    // Once a serial connection is esablished, send some commands to
    // reset the positions of Arduino stepper motors.

    if (commandWorker->isOpen()) {
        macroWorker->execute(MacroWorker::Action::MONITOR);
        connectionTimer->stop();
        //commandWorker->sendRequest(MotorCurrentPositionCommand(0, model->getLinearStage()->getCurrentPosition()));
        //commandWorker->sendRequest(MotorCurrentPositionCommand(1, model->getRotaryStage()->getCurrentPosition()));
        setWindowState(GuiState::CONNECTED);
        commandTimer->start(POLLING_TIME);
    }
}

void MainWindow::scanPorts()
{
    for (auto info : QSerialPortInfo::availablePorts()) {
        ports.append(info);
    }
    std::sort(ports.begin(), ports.end(), [](const QSerialPortInfo &a, const QSerialPortInfo &b) {
        bool comparison = (a.portName().compare(b.portName()) > 0);
        return comparison;
    });
    emit onPortsScanned();
}

void MainWindow::onPortsScanned()
{
    for (auto port : ports) {
        QStandardItem *item = new QStandardItem(port.portName());
        portsModel->appendRow(item);
    }
    ui->comboBox->setEnabled(true);
}

void MainWindow::onLinearStageSliderReleased()
{
    commandWorker->sendRequest(MotorStopCommand(0));
}

void MainWindow::onRotaryStageSliderReleased()
{
    commandWorker->sendRequest(MotorStopCommand(1));
}

float MainWindow::sliderValueToSpeed(int value)
{
    // Calculate the speed as a value from 0 to 100 using an exponential curve
    // to make the slider more sensitive at slow speed.
    // Both speed and slider values are between -100 and 100.

    float speed = pow(static_cast<float>(value) / 10, 2);
    return speed;
}

float MainWindow::speedToSliderValue(float speed)
{
    int direction = 0;
    if (speed > 0) {
        direction = 1;
    } else {
        direction = -1;
    }
    int value = static_cast<int>(10.0 * qSqrt(qAbs(speed))) * direction;
    return value;
}

int MainWindow::sgn(float value)
{
    int sign = 0;
    if (value > 0) {
        sign = 1;
    } else if (value < 0) {
        sign = -1;
    }
    return sign;
}

int MainWindow::sgn(int value)
{
    int sign = 0;
    if (value > 0) {
        sign = 1;
    } else if (value < 0) {
        sign = -1;
    }
    return sign;
}
