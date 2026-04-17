#ifndef CONTROL_FSM_H
#define CONTROL_FSM_H

#include "HorizonClock.h"
#include "StateEstimator/StateEstimator.h"
#include "GaitScheduler.h"
#include "Utilities/UserCommand.h"

struct DesiredFootPositions {
    Vec3<double> left_des_W = Vec3<double>::Zero();
    Vec3<double> right_des_W = Vec3<double>::Zero();
};


class ControlFSM {
public:
    ControlFSM(GaitScheduler* gaitScheduler,
               HorizonClock* horizonClock,
               const StateEstimate<double>* stateEstimate,
               const RobotParams<double>* robotParams,
               const UserCommand* userCommand)
        : _gaitScheduler(gaitScheduler),
          _horizonClock(horizonClock),
          _stateEstimate(stateEstimate),
          _robotParams(robotParams),
          _userCommand(userCommand) {}

    void syncHorizonClock();
    DesiredFootPositions SwingFootDesPos();


private:
    GaitScheduler* _gaitScheduler = nullptr;
    HorizonClock* _horizonClock = nullptr;
    const StateEstimate<double>* _stateEstimate = nullptr;
    const RobotParams<double>* _robotParams = nullptr;
    const UserCommand* _userCommand = nullptr;
};

#endif  // CONTROL_FSM_H
