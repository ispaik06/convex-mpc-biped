#ifndef LOCOMOTION_FSM_H
#define LOCOMOTION_FSM_H

#include <cstddef>
#include <deque>

#include "ControllerConfig.h"

enum class LocomotionState {
    StandingSettle,
    Standing,
    Walking,
    BrakingToStanding,
};

struct LocomotionFSMOutput {
    LocomotionState state{LocomotionState::Standing};
    LocomotionMode mode{LocomotionMode::Standing};
    bool justTransitioned{false};
    bool resetGaitClock{false};
    bool resetSwingState{false};
    bool acceptVelocityCommand{false};
    bool zeroMotionCommand{false};
    LegDynamicsRequest dynamicsRequest;
};

class LocomotionFSM {
public:
    LocomotionFSM(LocomotionMode requestedMode,
                  double postInitStandingSettleTime,
                  double brakingSettleSpeedThreshold,
                  double brakingSettleYawRateThreshold,
                  double brakingSettleAverageWindow,
                  int brakingSettleHoldTicks,
                  double brakingTimeoutSeconds,
                  int brakingTouchdownCount,
                  double startTime);

    bool requestToggle();
    void registerBrakingTouchdown();
    LocomotionFSMOutput update(double time,
                               double comVelocityX,
                               double comVelocityY,
                               double yawRate);
    LocomotionFSMOutput output() const;
    LocomotionState state() const { return _state; }

private:
    static LocomotionState initialState(LocomotionMode requestedMode,
                                        double postInitStandingSettleTime);
    static LocomotionMode modeForState(LocomotionState state);
    bool isInteractive() const;
    LocomotionFSMOutput makeOutput(bool justTransitioned) const;
    void transitionTo(LocomotionState nextState, double time);
    bool brakingSettleAveragesReady(double time,
                                    double comVelocityX,
                                    double comVelocityY,
                                    double yawRate);
    void resetBrakingSettleAverage();

    LocomotionMode _requestedMode{LocomotionMode::Standing};
    LocomotionMode _targetMode{LocomotionMode::Standing};
    double _postInitStandingSettleTime{0.0};
    double _brakingSettleSpeedThreshold{0.0};
    double _brakingSettleYawRateThreshold{0.0};
    double _brakingSettleAverageWindow{0.0};
    std::size_t _brakingSettleHoldTicksThreshold{0};
    std::size_t _brakingSettleTicks{0};
    bool _brakingReady{false};
    double _brakingReadyStartTime{0.0};
    double _brakingTimeoutSeconds{0.0};
    std::size_t _brakingTouchdownCountThreshold{0};
    std::size_t _brakingTouchdownCount{0};
    double _brakingSettleStartTime{0.0};
    double _brakingLastSettleLogTime{-1.0};
    struct BrakingSettleSample {
        double time{0.0};
        double comVelocityX{0.0};
        double comVelocityY{0.0};
        double yawRate{0.0};
    };
    std::deque<BrakingSettleSample> _brakingSettleSamples;
    double _stateStartTime{0.0};
    LocomotionState _state{LocomotionState::Standing};
};

#endif  // LOCOMOTION_FSM_H
