#include "MacroWorker.h"
#include "board/CommandUtils.h"
#include <QDebug>
#include <functional>

MacroWorker::MacroWorker(CommandWorker *commandWorker) : m_commandWorker{commandWorker}
{
    commandWorker->setParent(this);

    state = State::IDLE;

    // Timer

    pollingTimer = new QTimer(this);
    connect(pollingTimer, &QTimer::timeout, this, &MacroWorker::onTimerTimeout);
    connect(this, &MacroWorker::executeSignal, this, &MacroWorker::onExecute, Qt::ConnectionType::QueuedConnection);
}

void MacroWorker::setModel(ConsoleModel* model)
{
    this->model = model;
}

/**
 * Command the state machine to execute the given action.
 * @param action The action to be executed by the state machine.
 */
void MacroWorker::execute(Action action)
{
    this->action = action;
    emit executeSignal();
}

void MacroWorker::onExecute() {
    switch(action) {
    case Action::SHOOT_STACK:
        if (state == State::IDLE) {
            state = State::START;
            shootStack();
        }
        break;
    case Action::TRANSLATE_TO_START:
        if (state == State::IDLE) {
            state = State::START;
            translateToStart();
        }
        break;
    case Action::TRANSLATE_TO_END:
        if (state == State::IDLE) {
            state = State::START;
            translateToEnd();
        }
        break;
    case Action::TRANSLATE_TO:
        if (state == State::IDLE) {
            state = State::START;
            translateTo();
        }
        break;
    case Action::ROTATE_TO_START:
        if (state == State::IDLE) {
            state = State::START;
            rotateToStart();
        }
        break;
    case Action::ROTATE_TO_END:
        if (state == State::IDLE) {
            state = State::START;
            rotateToEnd();
        }
        break;
    case Action::ROTATE_TO:
        if (state == State::IDLE) {
            state = State::START;
            rotateTo();
        }
        break;
    case Action::MONITOR:
        monitor();
        break;
    case Action::STOP:
        stop();
        break;
    }
}

void MacroWorker::monitor() {
    if (!pollingTimer->isActive()) {
        pollingTimer->start(POLLING_TIME);
    }
}

/**
 * Stop any action being executed by the state machine.
 */
void MacroWorker::stop()
{
    if (!isConnected()) {
        return;
    }

    m_commandWorker->sendRequest(MotorStopCommand(),
        [this] (const Response&) {
            nextCommand = nullptr;
            state = State::IDLE;
        });
}

/* Check if the state machine is connected to Arduino.
 * @return True if connected, false otherwise.
 */
bool MacroWorker::isConnected() {
    bool isConnected = (m_commandWorker != nullptr && m_commandWorker->isOpen());
    return isConnected;
}

void MacroWorker::onTimerTimeout() {

    if (!isConnected()) {
        return;
    }

    pollingTimer->stop();
    m_commandWorker->sendRequest(MotorStatusCommand(), [this] (const Response& response) { // On success.
            MotorStatusResponse status = (MotorStatusResponse) response;
            StageModel* stages[] = {model->getLinearStage(), model->getRotaryStage()};
            for (int motorId = 0; motorId < 2; motorId++) {
                float speed = status.getMotor(motorId).getSpeed();
                stages[motorId]->setSpeed(speed);
                long currentPosition = status.getMotor(motorId).getPosition();
                stages[motorId]->setCurrentPosition(currentPosition);
            }

            // If Arduino is idle then wakeup the state machine.
            bool arduinoBusy = (status.getMotor(0).getDistanceToGo() != 0 || status.getMotor(1).getDistanceToGo() != 0);
            if (!arduinoBusy && nextCommand != nullptr) {
                (this->*nextCommand)(); // Invoke a callback method.
                nextCommand = nullptr;
            }

            // Wait some time and then send another status command.
            pollingTimer->start(POLLING_TIME);
        }
    );
}

/* Take all stacks for each rotation. */
void MacroWorker::shootStack()
{
    if (!isConnected()) {
        return;
    }

    if (state == State::START) {
        model->setCurrentStep(0);
        state = State::TRANSLATION;
        shootStack();
    } else if (state == State::TRANSLATION || state == State::TRANSLATION_OVERSHOOT) { // Move the linear stage to position.
        int overshootStep = (state == State::TRANSLATION_OVERSHOOT) ? 1 : 0;
        int linearStageStep =  model->getCurrentStep() % (model->getLinearStage()->getSteps()) - overshootStep;
        long linearStagePosition = qRound(model->getLinearStage()->getStartPosition()
            + static_cast<double>(linearStageStep) * (model->getLinearStage()->getEndPosition() - model->getLinearStage()->getStartPosition()) / std::max(1u, model->getLinearStage()->getSteps() - 1));
        m_commandWorker->sendRequest(MotorMoveToCommand(0, linearStagePosition), [this](const Response&) {
            m_commandWorker->sendRequest(MotorTargetSpeedCommand(0, 100), [this](const Response&) {
                state = State::ROTATION;
                nextCommand = &MacroWorker::shootStack;
            });
        });
    } else if (state == State::ROTATION || state == State::ROTATION_OVERSHOOT) { // Move the rotary stage to position.
        int overshootStep = (state == State::TRANSLATION_OVERSHOOT) ? 1 : 0;
        int rotaryStageStep =  model->getCurrentStep() / (model->getLinearStage()->getSteps()) - overshootStep;
        long rotaryStagePosition = qRound(model->getRotaryStage()->getStartPosition()
            + static_cast<double>(rotaryStageStep) * (model->getRotaryStage()->getEndPosition() - model->getRotaryStage()->getStartPosition()) / std::max(1u, model->getRotaryStage()->getSteps() - 1));
        m_commandWorker->sendRequest(MotorMoveToCommand(1, rotaryStagePosition), [this](const Response&) {
            m_commandWorker->sendRequest(MotorTargetSpeedCommand(1, 100), [this](const Response&) {
                state = State::SHOOT;
                nextCommand = &MacroWorker::shootStack;
            });
        });
    } else if (state == State::SHOOT) {
        m_commandWorker->sendRequest(ShootCommand(model->getFlashTime()), [this](const Response&) {
            model->setCurrentStep(model->getCurrentStep() + 1);
            if (model->getCurrentStep() == model->getTotalSteps()) {
                state = State::IDLE;
                emit finished();
            } else {
                state = State::TRANSLATION;
                shootStack();
            }
        });
    }
}

/* Translate the linear stage to the start position. */
void MacroWorker::translateToStart()
{
    if (!isConnected()) {
        return;
    }

    if (state == State::START) {
        m_commandWorker->sendRequest(MotorMoveToCommand(0, model->getLinearStage()->getStartPosition()), [this](const Response&) {
            m_commandWorker->sendRequest(MotorTargetSpeedCommand(0, 100), [this](const Response&) {
                nextCommand = &MacroWorker::translateToStart;
                state = State::WAITING;
            });
        });
    } else if (state == State::WAITING) {
        state = State::IDLE;
        emit finished();
    }
}

/* Translate the linear stage to the end position. */
void MacroWorker::translateToEnd()
{
    if (!isConnected()) {
        return;
    }

    if (state == State::START) {
        m_commandWorker->sendRequest(MotorMoveToCommand(0, model->getLinearStage()->getEndPosition()), [this](const Response&) {
            m_commandWorker->sendRequest(MotorTargetSpeedCommand(0, 100), [this](const Response&) {
                nextCommand = &MacroWorker::translateToEnd;
                state = State::WAITING;
            });
        });
    } else if (state == State::WAITING) {
        state = State::IDLE;
        emit finished();
    }
}


/* Translate the linear stage to the given position. */
void MacroWorker::translateTo()
{
    if (!isConnected()) {
        return;
    }

    if (state == State::START) {
        m_commandWorker->sendRequest(MotorMoveToCommand(0, model->getLinearStage()->getTargetPosition()), [this](const Response&) {
            m_commandWorker->sendRequest(MotorTargetSpeedCommand(0, 100), [this](const Response&) {
                nextCommand = &MacroWorker::translateTo;
                state = State::WAITING;
            });
        });
    } else if (state == State::WAITING) {
        state = State::IDLE;
        emit finished();
    }
}

/* Rotate the rotary stage to the start position. */
void MacroWorker::rotateToStart()
{
    if (!isConnected()) {
        return;
    }

    if (state == State::START) {
        m_commandWorker->sendRequest(MotorMoveToCommand(1, model->getRotaryStage()->getStartPosition()), [this](const Response&) {
            m_commandWorker->sendRequest(MotorTargetSpeedCommand(1, 100), [this](const Response&) {
                nextCommand = &MacroWorker::rotateToStart;
                state = State::WAITING;
            });
        });
    } else if (state == State::WAITING) {
        state = State::IDLE;
        emit finished();
    }
}

/* Rotate the rotary stage to the end position. */
void MacroWorker::rotateToEnd()
{
    if (!isConnected()) {
        return;
    }

    if (state == State::START) {
        m_commandWorker->sendRequest(MotorMoveToCommand(1, model->getRotaryStage()->getEndPosition()), [this](const Response&) {
            m_commandWorker->sendRequest(MotorTargetSpeedCommand(1, 100), [this](const Response&) {
                nextCommand = &MacroWorker::rotateToEnd;
                state = State::WAITING;
            });
        });
    } else if (state == State::WAITING) {
        state = State::IDLE;
        emit finished();
    }
}

/* Rotate the rotary stage to the given position. */
void MacroWorker::rotateTo()
{
    if (!isConnected()) {
        return;
    }

    if (state == State::START) {
        m_commandWorker->sendRequest(MotorMoveToCommand(1, model->getRotaryStage()->getTargetPosition()), [this](const Response&) {
            m_commandWorker->sendRequest(MotorTargetSpeedCommand(1, 100), [this](const Response&) {
                nextCommand = &MacroWorker::rotateTo;
                state = State::WAITING;
            });
        });
    } else if (state == State::WAITING) {
        state = State::IDLE;
        emit finished();
    }
}

bool MacroWorker::isNotRunning() const
{
    return (state == State::IDLE);
}

