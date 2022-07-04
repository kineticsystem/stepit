#ifndef MACROSTATEMACHINE_H
#define MACROSTATEMACHINE_H


#include <QTimer>
#include <QObject>
#include "board/CommandWorker.h"
#include "board/Commands.h"
#include "ConsoleModel.h"
#include "memory"

class MacroWorker : public QObject
{
    Q_OBJECT

public:

    /**
     * All possbole commands that can be issued to the state machine.
     */
    enum class Action {
        SHOOT_STACK,
        TRANSLATE_TO_START,
        TRANSLATE_TO_END,
        TRANSLATE_TO,
        ROTATE_TO_START,
        ROTATE_TO_END,
        ROTATE_TO,
        MONITOR,
        STOP
    };

    explicit MacroWorker(CommandWorker *commandWorker);

    /**
     * Set the thread-safe model used to share information between the
     * MacroWorker and the GUI.
     */
    void setModel(ConsoleModel* model);

    /**
     * Thread-safe method to issue and action to the state machine.
     * @param action The action to execute.
     */
    void execute(Action action);

    bool isNotRunning() const;

private:

    // The action to execute.
    Action action;

    // All possible state machine status.
    enum class State {
        START,
        EXECUTION,
        TRANSLATION,
        TRANSLATION_OVERSHOOT,
        ROTATION,
        ROTATION_OVERSHOOT,
        SHOOT,
        WAITING,
        IDLE
    };

    // The machine current status.
    State state;

    // This is a pointer to the next method to invoke once the Arduino
    // has completed a previous command and is now in idle mode.
    // A timer is used to constantly send info commands to the Arduino
    // to check its status. If the Arduino is idle, the timer invokes the
    // method pointed here.
    void (MacroWorker::*nextCommand)() = nullptr;

    // Send commands to serial port at given itervals.
    static const unsigned long POLLING_TIME = 100L;
    QTimer* pollingTimer;

    CommandWorker* m_commandWorker;

    // This is the thread-safe model used by the MacroWorker to exchange
    // information with the GUI.
    ConsoleModel* model = nullptr;

    bool isConnected();

    // Start polling Arduino to get information about motors status.
    void monitor();

    // Stop all current operations.
    void stop();

    // Initiate 3D phocus stacking.
    void shootStack();

    // Move the linear stage to start position.
    void translateToStart();

    // Move the linear stage to end position.
    void translateToEnd();

    // Move the linear stage to the given position.
    void translateTo();

    // Rotate the rotary stage to start position.
    void rotateToStart();

    // Rotate the rotary stage to end position.
    void rotateToEnd();

    // Rotate the rotary stage to the given position.
    void rotateTo();

signals:

    void finished();
    void statusReceived(const MotorStatusResponse& status);
    void executeSignal();

private slots:

    void onTimerTimeout();
    void onExecute();
};

#endif // MACROSTATEMACHINE_H
