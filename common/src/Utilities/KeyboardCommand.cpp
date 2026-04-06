#include "Utilities/KeyboardCommand.h"

#include <algorithm>
#include <cerrno>
#include <cctype>
#include <cstring>
#include <iostream>
#include <stdexcept>

#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

namespace {

termios g_originalTermios{};
bool g_termiosSaved = false;

bool configureTerminalRawMode(bool& terminalConfigured) {
    if (!isatty(STDIN_FILENO)) {
        std::cerr << "KeyboardCommand requires a TTY on stdin; keyboard input disabled.\n";
        return false;
    }

    if (!g_termiosSaved) {
        if (tcgetattr(STDIN_FILENO, &g_originalTermios) != 0) {
            throw std::runtime_error(
                std::string("KeyboardCommand failed to read terminal attributes: ") +
                std::strerror(errno));
        }
        g_termiosSaved = true;
    }

    termios raw = g_originalTermios;
    raw.c_lflag &= static_cast<unsigned long>(~(ICANON | ECHO));
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 0;

    if (tcsetattr(STDIN_FILENO, TCSANOW, &raw) != 0) {
        throw std::runtime_error(
            std::string("KeyboardCommand failed to enable raw terminal mode: ") +
            std::strerror(errno));
    }

    terminalConfigured = true;
    return true;
}

void restoreTerminal(bool& terminalConfigured) {
    if (!terminalConfigured || !g_termiosSaved) {
        return;
    }

    tcsetattr(STDIN_FILENO, TCSANOW, &g_originalTermios);
    terminalConfigured = false;
}

}  // namespace

KeyboardCommand::~KeyboardCommand() {
    stop();
}

void KeyboardCommand::start() {
    if (_running.load()) {
        return;
    }

    if (!configureTerminalRawMode(_terminalConfigured)) {
        return;
    }

    _running.store(true);
    std::cout << "KeyboardCommand active: w/s x_dot, a/d y_dot, q/e psi_dot, space reset\n";
    _inputThread = std::thread(&KeyboardCommand::inputLoop, this);
}

void KeyboardCommand::stop() {
    if (!_running.exchange(false)) {
        restoreTerminal(_terminalConfigured);
        return;
    }

    if (_inputThread.joinable()) {
        _inputThread.join();
    }

    restoreTerminal(_terminalConfigured);
    std::cout << '\n';
}

UserCommand KeyboardCommand::getUserCommand() const {
    std::lock_guard<std::mutex> lock(_commandMutex);
    return _userCommand;
}

void KeyboardCommand::inputLoop() {
    while (_running.load()) {
        fd_set readSet;
        FD_ZERO(&readSet);
        FD_SET(STDIN_FILENO, &readSet);

        timeval timeout{};
        timeout.tv_sec = 0;
        timeout.tv_usec = 100000;

        const int ready = select(STDIN_FILENO + 1, &readSet, nullptr, nullptr, &timeout);
        if (ready < 0) {
            if (errno == EINTR) {
                continue;
            }
            break;
        }

        if (ready == 0 || !FD_ISSET(STDIN_FILENO, &readSet)) {
            continue;
        }

        char key = 0;
        const ssize_t bytesRead = read(STDIN_FILENO, &key, 1);
        if (bytesRead == 1) {
            applyKey(key);
        }
    }
}

void KeyboardCommand::applyKey(char key) {
    const char lowerKey = static_cast<char>(std::tolower(static_cast<unsigned char>(key)));

    std::lock_guard<std::mutex> lock(_commandMutex);

    switch (lowerKey) {
        case 'w':
            _userCommand.x_dot = std::clamp(_userCommand.x_dot + _linearStep,
                                            -_linearLimit,
                                            _linearLimit);
            break;
        case 's':
            _userCommand.x_dot = std::clamp(_userCommand.x_dot - _linearStep,
                                            -_linearLimit,
                                            _linearLimit);
            break;
        case 'a':
            _userCommand.y_dot = std::clamp(_userCommand.y_dot + _linearStep,
                                            -_linearLimit,
                                            _linearLimit);
            break;
        case 'd':
            _userCommand.y_dot = std::clamp(_userCommand.y_dot - _linearStep,
                                            -_linearLimit,
                                            _linearLimit);
            break;
        case 'q':
            _userCommand.psi_dot = std::clamp(_userCommand.psi_dot + _yawStep,
                                              -_yawLimit,
                                              _yawLimit);
            break;
        case 'e':
            _userCommand.psi_dot = std::clamp(_userCommand.psi_dot - _yawStep,
                                              -_yawLimit,
                                              _yawLimit);
            break;
        case ' ':
            _userCommand = UserCommand{};
            break;
        default:
            return;
    }

    std::cout << "\rUserCommand | x_dot: " << _userCommand.x_dot
              << "  y_dot: " << _userCommand.y_dot
              << "  psi_dot: " << _userCommand.psi_dot << "   " << std::flush;
}
