#include "LocomotionFSM.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>

namespace {
const char* stateName(const LocomotionState state) {
    switch (state) {
        case LocomotionState::StandingSettle:
            return "standing_settle";
        case LocomotionState::Standing:
            return "standing";
        case LocomotionState::Walking:
            return "walking";
        case LocomotionState::BrakingToStanding:
            return "braking_to_standing";
    }
    return "unknown";
}

const char* modeName(const LocomotionMode mode) {
    switch (mode) {
        case LocomotionMode::Walking:
            return "walking";
        case LocomotionMode::Standing:
            return "standing";
        case LocomotionMode::Interactive:
            return "interactive";
    }
    return "unknown";
}

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
                             const double brakingSettleYawRateThreshold,
                             const double brakingSettleAverageWindow,
                             const int brakingSettleHoldTicks,
                             const double brakingTimeoutSeconds,
                             const int brakingTouchdownCount,
                             const double startTime)
    : _requestedMode(requestedMode),
      _targetMode(requestedMode == LocomotionMode::Walking ? LocomotionMode::Walking
                                                           : LocomotionMode::Standing),
      _postInitStandingSettleTime(std::max(0.0, postInitStandingSettleTime)),
      _brakingSettleSpeedThreshold(std::max(0.0, brakingSettleSpeedThreshold)),
      _brakingSettleYawRateThreshold(std::max(0.0, brakingSettleYawRateThreshold)),
      _brakingSettleAverageWindow(std::max(0.0, brakingSettleAverageWindow)),
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
        _brakingSettleStartTime = time;
        _brakingTouchdownCount = 0;
        resetBrakingSettleAverage();
    } else {
        _brakingSettleTicks = 0;
        _brakingReady = false;
        _brakingTouchdownCount = 0;
        resetBrakingSettleAverage();
    }
}

bool LocomotionFSM::requestToggle() {
    if (!isInteractive()) {
        return false;
    }

    const LocomotionMode currentMode = modeForState(_state);
    if (_state == LocomotionState::StandingSettle ||
        _state == LocomotionState::BrakingToStanding ||
        _targetMode != currentMode) {
        std::cout << "[LocomotionFSM] toggle ignored: transition in progress"
                  << " state=" << stateName(_state)
                  << " target=" << modeName(_targetMode)
                  << std::endl;
        return false;
    }

    _targetMode = (_targetMode == LocomotionMode::Walking) ? LocomotionMode::Standing
                                                            : LocomotionMode::Walking;
    return true;
}

void LocomotionFSM::registerBrakingTouchdown() {
    if (_state == LocomotionState::BrakingToStanding && _brakingReady) {
        ++_brakingTouchdownCount;
    }
}

void LocomotionFSM::resetBrakingSettleAverage() {
    _brakingSettleSamples.clear();
    _brakingLastSettleLogTime = -1.0;
}

bool LocomotionFSM::brakingSettleAveragesReady(const double time,
                                               const double comVelocityX,
                                               const double comVelocityY,
                                               const double yawRate) {
    _brakingSettleSamples.push_back({time, comVelocityX, comVelocityY, yawRate});
    const double windowStart = time - _brakingSettleAverageWindow;
    while (_brakingSettleSamples.size() > 1 &&
           _brakingSettleSamples.front().time < windowStart) {
        _brakingSettleSamples.pop_front();
    }
    const double averagingElapsed = time - _brakingSettleStartTime;
    if (_brakingSettleAverageWindow > 0.0 &&
        averagingElapsed + 1e-9 < _brakingSettleAverageWindow) {
        if (_brakingLastSettleLogTime < 0.0 ||
            time - _brakingLastSettleLogTime >= 0.5) {
            _brakingLastSettleLogTime = time;
            std::cout << "[LocomotionFSM] braking settle averaging"
                      << " elapsed=" << averagingElapsed
                      << " window=" << _brakingSettleAverageWindow
                      << std::endl;
        }
        return false;
    }

    double comVelocityXSum = 0.0;
    double comVelocityYSum = 0.0;
    double yawRateSum = 0.0;
    for (const auto& sample : _brakingSettleSamples) {
        comVelocityXSum += sample.comVelocityX;
        comVelocityYSum += sample.comVelocityY;
        yawRateSum += sample.yawRate;
    }

    const double sampleCount = static_cast<double>(_brakingSettleSamples.size());
    const double comVelocityXMean = comVelocityXSum / sampleCount;
    const double comVelocityYMean = comVelocityYSum / sampleCount;
    const double yawRateMean = yawRateSum / sampleCount;
    const bool ready = std::abs(comVelocityXMean) <= _brakingSettleSpeedThreshold &&
                       std::abs(comVelocityYMean) <= _brakingSettleSpeedThreshold &&
                       std::abs(yawRateMean) <= _brakingSettleYawRateThreshold;
    if (!ready &&
        (_brakingLastSettleLogTime < 0.0 || time - _brakingLastSettleLogTime >= 0.5)) {
        _brakingLastSettleLogTime = time;
        std::cout << "[LocomotionFSM] braking settle waiting"
                  << " mean_vx=" << comVelocityXMean
                  << " mean_vy=" << comVelocityYMean
                  << " mean_wz=" << yawRateMean
                  << " thresholds(v=" << _brakingSettleSpeedThreshold
                  << ",wz=" << _brakingSettleYawRateThreshold << ")"
                  << std::endl;
    }
    return ready;
}

LocomotionFSMOutput LocomotionFSM::update(const double time,
                                          const double comVelocityX,
                                          const double comVelocityY,
                                          const double yawRate) {
    if (!std::isfinite(time)) {
        throw std::runtime_error("LocomotionFSM::update received non-finite time");
    }
    if (!std::isfinite(comVelocityX) || !std::isfinite(comVelocityY)) {
        throw std::runtime_error("LocomotionFSM::update received invalid COM velocity");
    }
    if (!std::isfinite(yawRate)) {
        throw std::runtime_error("LocomotionFSM::update received invalid yaw rate");
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
                if (brakingSettleAveragesReady(time,
                                               comVelocityX,
                                               comVelocityY,
                                               yawRate)) {
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
