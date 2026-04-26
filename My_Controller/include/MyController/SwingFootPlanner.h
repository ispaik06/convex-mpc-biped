#ifndef SWING_FOOT_PLANNER_H
#define SWING_FOOT_PLANNER_H

#include <vector>

#include "GaitScheduler.h"
#include "HorizonClock.h"
#include "StateEstimator/StateEstimator.h"
#include "Utilities/UserCommand.h"

struct DesiredFootPositions {
    Vec3<double> left_des_W = Vec3<double>::Zero();
    Vec3<double> right_des_W = Vec3<double>::Zero();
};

class SwingFootPlanner {
public:
    SwingFootPlanner(GaitScheduler* gaitScheduler,
                     HorizonClock* horizonClock,
                     const StateEstimate<double>* stateEstimate,
                     const RobotParams<double>* robotParams,
                     const UserCommand* userCommand)
        : _gaitScheduler(gaitScheduler),
          _horizonClock(horizonClock),
          _stateEstimate(stateEstimate),
          _robotParams(robotParams),
          _userCommand(userCommand) {}

    void reset();
    DesiredFootPositions desiredFootPositions();

private:
    void syncHorizonClock();
    void ensureSwingTouchdownCache();
    Vec3<double> touchdownTargetWorldBodyVelocityHalfStance(std::size_t legIndex) const;
    Vec3<double> touchdownTargetWorldLegacy(std::size_t legIndex) const;

    GaitScheduler* _gaitScheduler = nullptr;
    HorizonClock* _horizonClock = nullptr;
    const StateEstimate<double>* _stateEstimate = nullptr;
    const RobotParams<double>* _robotParams = nullptr;
    const UserCommand* _userCommand = nullptr;
    std::vector<Vec3<double>> _bodyVelocityHalfStanceTouchdownTargets;
    std::vector<bool> _bodyVelocityHalfStanceTouchdownTargetValid;
    std::vector<bool> _wasInStance;
};

#endif  // SWING_FOOT_PLANNER_H
