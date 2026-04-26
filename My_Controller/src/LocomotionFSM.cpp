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
            return LocomotionState::Standing;
    }

    throw std::runtime_error("Unsupported locomotion mode");
}
}  // namespace

LocomotionFSM::LocomotionFSM(const LocomotionMode requestedMode,
                             const double postInitStandingSettleTime,
                             const double startTime)
    : _requestedMode(requestedMode),
      _postInitStandingSettleTime(std::max(0.0, postInitStandingSettleTime)),
      _stateStartTime(startTime),
      _state(initialState(requestedMode, postInitStandingSettleTime)) {}

LocomotionState LocomotionFSM::initialState(const LocomotionMode requestedMode,
                                            const double postInitStandingSettleTime) {
    if (requestedMode == LocomotionMode::Walking && postInitStandingSettleTime > 0.0) {
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
            return LocomotionMode::Walking;
    }

    throw std::runtime_error("Unsupported locomotion state");
}

void LocomotionFSM::transitionTo(const LocomotionState nextState, const double time) {
    if (_state == nextState) {
        return;
    }

    _state = nextState;
    _stateStartTime = time;
}

LocomotionFSMOutput LocomotionFSM::update(const double time) {
    if (!std::isfinite(time)) {
        throw std::runtime_error("LocomotionFSM::update received non-finite time");
    }

    bool justTransitioned = false;
    if (_state == LocomotionState::StandingSettle &&
        time - _stateStartTime >= _postInitStandingSettleTime) {
        transitionTo(stateFromMode(_requestedMode), time);
        justTransitioned = true;
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
    output.resetGaitClock = justTransitioned;
    output.resetSwingState = justTransitioned;
    output.acceptVelocityCommand = output.mode == LocomotionMode::Walking;
    output.dynamicsRequest.swingLegDynamics = output.mode == LocomotionMode::Walking;
    output.dynamicsRequest.standingFootJacobians = output.mode == LocomotionMode::Standing;

    return output;
}
