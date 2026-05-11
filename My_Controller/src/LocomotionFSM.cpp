#include "LocomotionFSM.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace {
LocomotionState stateFromMode(const LocomotionMode mode) {
    switch (mode) {
        case LocomotionMode::Walking:
            return LocomotionState::Walking;
        case LocomotionMode::Standing:
        case LocomotionMode::Interactive:
            return LocomotionState::Standing;
    }

    throw std::runtime_error("Unsupported locomotion mode");
}
}  // namespace

LocomotionFSM::LocomotionFSM(const LocomotionMode requestedMode,
                             const double postInitStandingSettleTime,
                             const double brakingSettleSpeedThreshold,
                             const int brakingSettleHoldTicks,
                             const double brakingTimeoutSeconds,
                             const int brakingTouchdownCount,
                             const double startTime)
    : _requestedMode(requestedMode),
      _targetMode(requestedMode == LocomotionMode::Walking ? LocomotionMode::Walking
                                                           : LocomotionMode::Standing),
      _postInitStandingSettleTime(std::max(0.0, postInitStandingSettleTime)),
      _brakingSettleSpeedThreshold(std::max(0.0, brakingSettleSpeedThreshold)),
      _brakingSettleHoldTicksThreshold(
          static_cast<std::size_t>(std::max(0, brakingSettleHoldTicks))),
      _brakingTimeoutSeconds(std::max(0.0, brakingTimeoutSeconds)),
      _brakingTouchdownCountThreshold(
          static_cast<std::size_t>(std::max(0, brakingTouchdownCount))),
      _stateStartTime(startTime),
      _state(initialState(requestedMode, postInitStandingSettleTime)) {}

LocomotionState LocomotionFSM::initialState(const LocomotionMode requestedMode,
                                            const double postInitStandingSettleTime) {
    if (requestedMode == LocomotionMode::Walking && postInitStandingSettleTime > 0.0) {
        return LocomotionState::StandingSettle;
    }

    if (requestedMode == LocomotionMode::Walking) {
        return LocomotionState::Walking;
    }

    if (requestedMode == LocomotionMode::Interactive &&
        postInitStandingSettleTime > 0.0) {
        return LocomotionState::StandingSettle;
    }

    return stateFromMode(requestedMode);
}

LocomotionMode LocomotionFSM::modeForState(const LocomotionState state) {
    switch (state) {
        case LocomotionState::StandingSettle:
        case LocomotionState::Standing:
            return LocomotionMode::Standing;
        case LocomotionState::Walking:
        case LocomotionState::BrakingToStanding:
            return LocomotionMode::Walking;
    }

    throw std::runtime_error("Unsupported locomotion state");
}

bool LocomotionFSM::isInteractive() const {
    return _requestedMode == LocomotionMode::Interactive;
}

void LocomotionFSM::transitionTo(const LocomotionState nextState, const double time) {
    if (_state == nextState) {
        return;
    }

    _state = nextState;
    _stateStartTime = time;
    if (nextState == LocomotionState::BrakingToStanding) {
        _brakingSettleTicks = 0;
        _brakingReady = false;
        _brakingReadyStartTime = time;
        _brakingTouchdownCount = 0;
    } else {
        _brakingSettleTicks = 0;
        _brakingReady = false;
        _brakingTouchdownCount = 0;
    }
}

void LocomotionFSM::requestToggle() {
    if (!isInteractive()) {
        return;
    }

    _targetMode = (_targetMode == LocomotionMode::Walking) ? LocomotionMode::Standing
                                                            : LocomotionMode::Walking;
}

void LocomotionFSM::registerBrakingTouchdown() {
    if (_state == LocomotionState::BrakingToStanding && _brakingReady) {
        ++_brakingTouchdownCount;
    }
}

LocomotionFSMOutput LocomotionFSM::update(const double time, const double planarBodySpeed) {
    if (!std::isfinite(time)) {
        throw std::runtime_error("LocomotionFSM::update received non-finite time");
    }
    if (!std::isfinite(planarBodySpeed) || planarBodySpeed < 0.0) {
        throw std::runtime_error("LocomotionFSM::update received invalid planar body speed");
    }

    bool justTransitioned = false;
    if (_state == LocomotionState::StandingSettle &&
        time - _stateStartTime >= _postInitStandingSettleTime) {
        transitionTo(stateFromMode(_targetMode), time);
        justTransitioned = true;
    }

    if (_targetMode == LocomotionMode::Walking) {
        if (_state == LocomotionState::Standing || _state == LocomotionState::BrakingToStanding) {
            transitionTo(LocomotionState::Walking, time);
            justTransitioned = true;
        }
    } else {
        if (_state == LocomotionState::Walking) {
            transitionTo(LocomotionState::BrakingToStanding, time);
            justTransitioned = true;
        } else if (_state == LocomotionState::BrakingToStanding) {
            if (!_brakingReady) {
                if (planarBodySpeed <= _brakingSettleSpeedThreshold) {
                    ++_brakingSettleTicks;
                    if (_brakingSettleTicks >= _brakingSettleHoldTicksThreshold) {
                        _brakingReady = true;
                        _brakingReadyStartTime = time;
                        _brakingTouchdownCount = 0;
                    }
                } else {
                    _brakingSettleTicks = 0;
                }
            } else if (_brakingTouchdownCount >= _brakingTouchdownCountThreshold ||
                       time - _brakingReadyStartTime >= _brakingTimeoutSeconds) {
                transitionTo(LocomotionState::Standing, time);
                justTransitioned = true;
            }
        }
    }

    return makeOutput(justTransitioned);
}

LocomotionFSMOutput LocomotionFSM::output() const {
    return makeOutput(false);
}

LocomotionFSMOutput LocomotionFSM::makeOutput(const bool justTransitioned) const {
    LocomotionFSMOutput output;
    output.state = _state;
    output.mode = modeForState(_state);
    output.justTransitioned = justTransitioned;
    output.resetGaitClock = justTransitioned && _state != LocomotionState::BrakingToStanding;
    output.resetSwingState = justTransitioned && _state != LocomotionState::BrakingToStanding;
    output.acceptVelocityCommand = output.mode == LocomotionMode::Walking &&
                                   _state != LocomotionState::BrakingToStanding;
    output.zeroMotionCommand = _state == LocomotionState::BrakingToStanding;
    output.dynamicsRequest.swingLegDynamics = output.mode == LocomotionMode::Walking;
    output.dynamicsRequest.standingFootJacobians = output.mode == LocomotionMode::Standing;

    return output;
}
