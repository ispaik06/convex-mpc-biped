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

    void start();
    void stop();
    UserCommand getUserCommand() const;

private:
    enum class EscapeState {
        None,
        SawEscape,
        SawEscapeBracket,
    };

    void inputLoop();
    void applyKey(char key);
    void applyVerticalKey(bool increase);
    void applyStandingOrientationKey(char key);

    mutable std::mutex _commandMutex;
    UserCommand _userCommand;
    std::thread _inputThread;
    std::atomic<bool> _running{false};
    bool _terminalConfigured{false};
    double _linearStep{0.05};
    double _yawStep{0.10};
    double _verticalStep{0.05};
    double _linearLimit{1.0};
    double _yawLimit{1.5};
    double _verticalLimit{0.8};
};

#endif  // KEYBOARD_COMMAND_H
