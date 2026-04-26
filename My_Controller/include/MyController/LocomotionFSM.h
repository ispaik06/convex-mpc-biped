#ifndef LOCOMOTION_FSM_H
#define LOCOMOTION_FSM_H

#include "ControllerConfig.h"

enum class LocomotionState {
    StandingSettle,
    Standing,
    Walking,
};

struct LocomotionFSMOutput {
    LocomotionState state{LocomotionState::Standing};
    LocomotionMode mode{LocomotionMode::Standing};
    bool justTransitioned{false};
    bool resetGaitClock{false};
    bool resetSwingState{false};
    bool acceptVelocityCommand{false};
    LegDynamicsRequest dynamicsRequest;
};

class LocomotionFSM {
public:
    LocomotionFSM(LocomotionMode requestedMode,
                  double postInitStandingSettleTime,
                  double startTime);

    LocomotionFSMOutput update(double time);
    LocomotionFSMOutput output() const;
    LocomotionState state() const { return _state; }

private:
    static LocomotionState initialState(LocomotionMode requestedMode,
                                        double postInitStandingSettleTime);
    static LocomotionMode modeForState(LocomotionState state);
    LocomotionFSMOutput makeOutput(bool justTransitioned) const;
    void transitionTo(LocomotionState nextState, double time);

    LocomotionMode _requestedMode{LocomotionMode::Standing};
    double _postInitStandingSettleTime{0.0};
    double _stateStartTime{0.0};
    LocomotionState _state{LocomotionState::Standing};
};

#endif  // LOCOMOTION_FSM_H
