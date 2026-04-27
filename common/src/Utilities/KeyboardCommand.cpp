#include "Utilities/KeyboardCommand.h"

#include <algorithm>
#include <cerrno>
#include <cctype>
#include <cstring>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <sstream>
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

std::string formatScalar(const double value, const int precision) {
    std::ostringstream out;
    out << std::fixed << std::setprecision(precision) << zeroTinyValue(value);
    return out.str();
}

std::string formatDegrees(const double radians, const int precision) {
    return formatScalar(radians * kRadToDeg, precision);
}

void sanitizeCommand(UserCommand& command) {
    command.x_dot = zeroTinyValue(command.x_dot);
    command.y_dot = zeroTinyValue(command.y_dot);
    command.psi_dot = zeroTinyValue(command.psi_dot);
    command.z_dot = zeroTinyValue(command.z_dot);
    command.standing_roll_offset_rad = zeroTinyValue(command.standing_roll_offset_rad);
    command.standing_pitch_offset_rad = zeroTinyValue(command.standing_pitch_offset_rad);
}

void printCommand(const UserCommand& command, const bool standingControls) {
    if (standingControls) {
        std::cout << "[KeyboardCommand] standing command | z_dot="
                  << formatScalar(command.z_dot, 3) << " m/s"
                  << "  pitch=" << formatDegrees(command.standing_pitch_offset_rad, 2)
                  << " deg"
                  << "  roll=" << formatDegrees(command.standing_roll_offset_rad, 2)
                  << " deg"
                  << "  debug_log_request=" << command.standing_mpc_debug_log_request
                  << '\n';
        return;
    }

    std::cout << "[KeyboardCommand] walking command | x_dot="
              << formatScalar(command.x_dot, 3) << " m/s"
              << "  y_dot=" << formatScalar(command.y_dot, 3) << " m/s"
              << "  psi_dot=" << formatScalar(command.psi_dot, 3) << " rad/s"
              << "  debug_log_request=" << command.standing_mpc_debug_log_request
              << '\n';
}

void printActiveControls(const bool standingControls) {
    if (standingControls) {
        std::cout << "[KeyboardCommand] standing controls active: up/down z_dot, "
                  << "standing i/k pitch, j/l roll, Shift+L log, space reset\n";
        return;
    }

    std::cout << "[KeyboardCommand] walking controls active: w/s x_dot, a/d y_dot, "
              << "q/e psi_dot, Shift+L log, space reset\n";
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
    std::cout << "[KeyboardCommand] input thread started\n";
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

void KeyboardCommand::setStandingControls(const bool standingControls, const bool announce) {
    const bool previousStandingControls = _standingControls.exchange(standingControls);
    const bool shouldPrint = announce || previousStandingControls != standingControls;
    if (!shouldPrint || (!_running.load() && !_terminalConfigured)) {
        return;
    }

    printActiveControls(standingControls);
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
    if (!_standingControls.load()) {
        return;
    }

    std::lock_guard<std::mutex> lock(_commandMutex);

    const double delta = increase ? _verticalStep : -_verticalStep;
    _userCommand.z_dot =
        std::clamp(_userCommand.z_dot + delta, -_verticalLimit, _verticalLimit);
    sanitizeCommand(_userCommand);
    printCommand(_userCommand, true);
}

void KeyboardCommand::applyStandingOrientationKey(char key) {
    if (!_standingControls.load()) {
        return;
    }

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
    printCommand(_userCommand, true);
}

void KeyboardCommand::applyKey(char key) {
    if (key == 'L') {
        std::lock_guard<std::mutex> lock(_commandMutex);
        ++_userCommand.standing_mpc_debug_log_request;
        std::cout << "[KeyboardCommand] debug log request #"
                  << _userCommand.standing_mpc_debug_log_request
                  << " queued for the next MPC solve\n";
        return;
    }

    const char lowerKey = static_cast<char>(std::tolower(static_cast<unsigned char>(key)));
    const bool standingControls = _standingControls.load();

    if (key == ' ') {
        std::lock_guard<std::mutex> lock(_commandMutex);
        const unsigned long long requestCount = _userCommand.standing_mpc_debug_log_request;
        _userCommand = UserCommand{};
        _userCommand.standing_mpc_debug_log_request = requestCount;
        sanitizeCommand(_userCommand);
        printCommand(_userCommand, standingControls);
        return;
    }

    if (standingControls) {
        switch (lowerKey) {
            case 'i':
            case 'k':
            case 'j':
            case 'l':
                applyStandingOrientationKey(lowerKey);
                return;
            case 'w':
            case 's':
            case 'a':
            case 'd':
            case 'q':
            case 'e':
                return;
            default:
                return;
        }
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
        default:
            return;
    }

    sanitizeCommand(_userCommand);
    printCommand(_userCommand, false);
}
