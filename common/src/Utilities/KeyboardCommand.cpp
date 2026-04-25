#include "Utilities/KeyboardCommand.h"

#include <algorithm>
#include <cerrno>
#include <cctype>
#include <cstring>
#include <cmath>
#include <iostream>
#include <stdexcept>

#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

namespace {

termios g_originalTermios{};
bool g_termiosSaved = false;
constexpr double kCommandZeroEpsilon = 1e-12;
constexpr double kStandingOrientationStepRad = 2.0 * 3.141592653589793238462643383279502884 / 180.0;
constexpr double kRadToDeg = 180.0 / 3.141592653589793238462643383279502884;

double zeroTinyValue(const double value) {
    return (std::abs(value) < kCommandZeroEpsilon) ? 0.0 : value;
}

void sanitizeCommand(UserCommand& command) {
    command.x_dot = zeroTinyValue(command.x_dot);
    command.y_dot = zeroTinyValue(command.y_dot);
    command.psi_dot = zeroTinyValue(command.psi_dot);
    command.z_dot = zeroTinyValue(command.z_dot);
    command.standing_roll_offset_rad = zeroTinyValue(command.standing_roll_offset_rad);
    command.standing_pitch_offset_rad = zeroTinyValue(command.standing_pitch_offset_rad);
}

void printCommand(const UserCommand& command) {
    std::cout << "UserCommand | x_dot: " << zeroTinyValue(command.x_dot)
              << "  y_dot: " << zeroTinyValue(command.y_dot)
              << "  psi_dot: " << zeroTinyValue(command.psi_dot)
              << "  z_dot: " << zeroTinyValue(command.z_dot)
              << "  standing_pitch_deg: "
              << zeroTinyValue(command.standing_pitch_offset_rad) * kRadToDeg
              << "  standing_roll_deg: "
              << zeroTinyValue(command.standing_roll_offset_rad) * kRadToDeg
              << "  standing_mpc_debug_log_request: "
              << command.standing_mpc_debug_log_request << '\n';
}

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
    std::cout << "KeyboardCommand active: w/s x_dot, a/d y_dot, q/e psi_dot, "
              << "up/down z_dot, standing i/k pitch, j/l roll, Shift+L log, space reset\n";
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
    EscapeState escapeState = EscapeState::None;

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
            switch (escapeState) {
                case EscapeState::None:
                    if (key == '\x1b') {
                        escapeState = EscapeState::SawEscape;
                        continue;
                    }
                    applyKey(key);
                    break;
                case EscapeState::SawEscape:
                    if (key == '[') {
                        escapeState = EscapeState::SawEscapeBracket;
                        continue;
                    }
                    escapeState = EscapeState::None;
                    break;
                case EscapeState::SawEscapeBracket:
                    if (key == 'A') {
                        applyVerticalKey(true);
                    } else if (key == 'B') {
                        applyVerticalKey(false);
                    }
                    escapeState = EscapeState::None;
                    break;
            }
        }
    }
}

void KeyboardCommand::applyVerticalKey(bool increase) {
    std::lock_guard<std::mutex> lock(_commandMutex);

    const double delta = increase ? _verticalStep : -_verticalStep;
    _userCommand.z_dot =
        std::clamp(_userCommand.z_dot + delta, -_verticalLimit, _verticalLimit);
    sanitizeCommand(_userCommand);
    printCommand(_userCommand);
}

void KeyboardCommand::applyStandingOrientationKey(char key) {
    std::lock_guard<std::mutex> lock(_commandMutex);

    switch (key) {
        case 'i':
            _userCommand.standing_pitch_offset_rad += kStandingOrientationStepRad;
            break;
        case 'k':
            _userCommand.standing_pitch_offset_rad -= kStandingOrientationStepRad;
            break;
        case 'j':
            _userCommand.standing_roll_offset_rad -= kStandingOrientationStepRad;
            break;
        case 'l':
            _userCommand.standing_roll_offset_rad += kStandingOrientationStepRad;
            break;
        default:
            return;
    }

    sanitizeCommand(_userCommand);
    printCommand(_userCommand);
}

void KeyboardCommand::applyKey(char key) {
    if (key == 'L') {
        std::lock_guard<std::mutex> lock(_commandMutex);
        ++_userCommand.standing_mpc_debug_log_request;
        std::cout << "StandingMPCDebug request #"
                  << _userCommand.standing_mpc_debug_log_request
                  << " queued for the next MPC solve\n";
        return;
    }

    const char lowerKey = static_cast<char>(std::tolower(static_cast<unsigned char>(key)));

    if (lowerKey == 'i' || lowerKey == 'k' || lowerKey == 'j' || lowerKey == 'l') {
        applyStandingOrientationKey(lowerKey);
        return;
    }

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
        {
            const unsigned long long requestCount =
                _userCommand.standing_mpc_debug_log_request;
            _userCommand = UserCommand{};
            _userCommand.standing_mpc_debug_log_request = requestCount;
            break;
        }
        default:
            return;
    }

    sanitizeCommand(_userCommand);
    printCommand(_userCommand);
}
