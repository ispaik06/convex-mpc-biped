#ifndef KEYBOARD_COMMAND_H
#define KEYBOARD_COMMAND_H

#include <atomic>
#include <mutex>
#include <thread>

#include "Utilities/UserCommand.h"

class KeyboardCommand {
public:
    KeyboardCommand() = default;
    ~KeyboardCommand();

    void setWalkingLimits(double xDotLimit, double yDotLimit, double psiDotLimit);
    bool start();
    void stop();
    void setStandingControls(bool standingControls, bool announce = false);
    UserCommand getUserCommand() const;

private:
    enum class EscapeState {
        None,
        SawEscape,
        SawEscapeBracket,
    };

    void inputLoop();
    void applyKey(char key);
    void applyHeightKey(bool increase);
    void applyStandingOrientationKey(char key);

    mutable std::mutex _commandMutex;
    UserCommand _userCommand;
    std::thread _inputThread;
    std::atomic<bool> _running{false};
    std::atomic<bool> _standingControls{false};
    bool _terminalConfigured{false};
    double _linearStep{0.05};
    double _yawStep{0.1};
    double _heightStep{0.01};
    double _xLimit{1.0};
    double _yLimit{1.0};
    double _yawLimit{1.5};
    double _heightLimit{0.8};
};

#endif  // KEYBOARD_COMMAND_H
