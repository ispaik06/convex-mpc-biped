#ifndef CONTROL_FSM_H
#define CONTROL_FSM_H

#include "StateEstimator/StateEstimator.h"
#include "GaitScheduler.h"
#include "Utilities/UserCommand.h"

struct PosFootDes {
    Vec3<double> p1_des_W = Vec3<double>::Zero();
    Vec3<double> p2_des_W = Vec3<double>::Zero();
};


class ControlFSM {
public:
    ControlFSM(GaitScheduler* gaitScheduler,
               StateEstimate<double>* stateEstimate,
               const RobotParams<double>* robotParams,
               const UserCommand* userCommand)
        : _gaitScheduler(gaitScheduler),
          _stateEstimate(stateEstimate),
          _robotParams(robotParams),
          _userCommand(userCommand) {}

    PosFootDes SwingFootDesPos();


private:
    GaitScheduler* _gaitScheduler = nullptr;
    StateEstimate<double>* _stateEstimate = nullptr;
    const RobotParams<double>* _robotParams = nullptr;
    const UserCommand* _userCommand = nullptr;
};

#endif  // CONTROL_FSM_H
